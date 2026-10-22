// SPDX-License-Identifier: GPL-2.0
/*
 * lucid — GrayRavens Game Mode Task Killer & OOM Manager
 *
 * Listens to selene's notifier chain.  On game start it raises the
 * game process's priority and deprioritises known background / non-
 * essential processes so the game gets maximum memory headroom.
 * On game stop everything is restored to baseline.
 *
 * The background "kill list" is a set of package-name / comm-name
 * patterns matched against /proc/<pid>/cmdline and /proc/<pid>/comm.
 * The list is configurable at runtime via sysfs.
 *
 * Sysfs: /sys/kernel/lucid/
 *   enabled       (RW) — 0/1 master switch
 *   kill_list     (RO) — dump current kill patterns
 *   kill_list_add (WO) — add a pattern (newline-terminated)
 *   kill_list_del (WO) — remove a pattern
 *   oom_score_adj_game  (RW) — OOM score to apply to game (-1000..1000, def -900)
 *   oom_score_adj_bg    (RW) — OOM score to apply to background (def 500)
 *   currently_killed    (RO) — number of processes killed last game session
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/mm.h>
#include <linux/pid.h>
#include <linux/string.h>
#include <linux/selene.h>



/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static DEFINE_MUTEX(lucid_lock);
static bool lucid_enabled = false;
static int lucid_oom_game = -900;
static int lucid_oom_bg   =  500;
static atomic_t lucid_killed_count = ATOMIC_INIT(0);

/* Kill list — process name patterns */
#define LUCID_PATTERN_MAX  64
#define LUCID_PATTERN_LEN  64
static char *lucid_patterns[LUCID_PATTERN_MAX];
static int lucid_pattern_count;

/* Cache of PIDs we adjusted so we can restore them */
#define LUCID_PID_CACHE 256
static struct {
	pid_t pid;
	int old_oom;
} lucid_pid_cache[LUCID_PID_CACHE];
static int lucid_pid_cache_count;

/* Track the game PID separately so we can restore oom_score_adj on stop */
static struct {
	pid_t pid;
	int old_oom;
	bool active;
} lucid_game;

/* ------------------------------------------------------------------ */
/* OOM helpers — direct task_struct access, no VFS                    */
/* ------------------------------------------------------------------ */

static inline int lucid_read_oom_adj(struct task_struct *task)
{
	struct sighand_struct *sighand;
	unsigned long flags;
	int val = 0;

	/*
	 * lock_task_sighand() takes the sighand ref and returns NULL
	 * (without locking) when the task is exiting and sighand is
	 * gone -- closing the race between a NULL check and the
	 * siglock deref on a dying task.
	 */
	sighand = lock_task_sighand(task, &flags);
	if (sighand) {
		if (task->signal)
			val = task->signal->oom_score_adj;
		unlock_task_sighand(task, &flags);
	}
	return val;
}

static void lucid_write_oom_adj(struct task_struct *task, int val)
{
	struct sighand_struct *sighand;
	unsigned long flags;

	/* Exiting tasks: sighand gone, nothing to do (race-safe). */
	sighand = lock_task_sighand(task, &flags);
	if (!sighand)
		return;

	if (task->signal)
		task->signal->oom_score_adj = val;
	unlock_task_sighand(task, &flags);
}

/* ------------------------------------------------------------------ */
/* Pattern matching — check if a process name matches a kill pattern  */
/* ------------------------------------------------------------------ */

static bool lucid_match_pattern(const char *name, const char *pattern)
{
	/* Simple prefix / substring match */
	return strstr(name, pattern) != NULL;
}

static bool lucid_is_killable(struct task_struct *task)
{
	char comm[TASK_COMM_LEN];
	char cmdline[256];
	int i, ret;

	get_task_comm(comm, task);

	for (i = 0; i < lucid_pattern_count; i++) {
		if (!lucid_patterns[i])
			continue;
		if (lucid_match_pattern(comm, lucid_patterns[i]))
			return true;
	}

	/* Also check cmdline for full package names (kernel API, no VFS) */
	ret = get_cmdline(task, cmdline, sizeof(cmdline) - 1);
	if (ret > 0) {
		cmdline[ret] = '\0';
		for (i = 0; i < lucid_pattern_count; i++) {
			if (!lucid_patterns[i])
				continue;
			if (lucid_match_pattern(cmdline, lucid_patterns[i]))
				return true;
		}
	}

	return false;
}

/* ------------------------------------------------------------------ */
/* Game start / stop                                                   */
/* ------------------------------------------------------------------ */

static void lucid_on_game_start(const char *game_name)
{
	struct task_struct *p;
	pid_t pids[LUCID_PID_CACHE];
	int npids = 0, cache_idx = 0, i;

	if (!lucid_enabled)
		return;

	mutex_lock(&lucid_lock);
	lucid_pid_cache_count = 0;
	atomic_set(&lucid_killed_count, 0);
	lucid_game.active = false;

	/* Phase 1: collect candidate PIDs under RCU (fast, no sleeping) */
	rcu_read_lock();
	for_each_process(p) {
		pid_t pid = task_pid_nr(p);

		if (pid <= 1)
			continue;

		if (npids >= LUCID_PID_CACHE)
			break;

		pids[npids++] = pid;
	}
	rcu_read_unlock();

	/* Phase 2: check + adjust OOM for each PID (can sleep) */
	for (i = 0; i < npids && cache_idx < LUCID_PID_CACHE; i++) {
		struct pid *pid_struct = find_get_pid(pids[i]);
		struct task_struct *task;
		bool is_game = false;

		if (!pid_struct)
			continue;
		task = get_pid_task(pid_struct, PIDTYPE_PID);
		put_pid(pid_struct);
		if (!task)
			continue;

		/*
		 * Check if this is the game process by matching comm/cmdline
		 * against the game_name that selene passed us.
		 */
		if (game_name && game_name[0]) {
			char comm[TASK_COMM_LEN];
			char cmdline[256];
			int ret;

			get_task_comm(comm, task);
			if (strstr(comm, game_name) || strstr(game_name, comm)) {
				is_game = true;
			} else {
				ret = get_cmdline(task, cmdline, sizeof(cmdline) - 1);
				if (ret > 0) {
					cmdline[ret] = '\0';
					if (strstr(cmdline, game_name))
						is_game = true;
				}
			}
		}

		if (is_game) {
			/* Boost the game process itself */
			lucid_game.pid = pids[i];
			lucid_game.old_oom = lucid_read_oom_adj(task);
			lucid_game.active = true;
			lucid_write_oom_adj(task, lucid_oom_game);
			pr_debug("lucid: boosted game (pid=%d, oom=%d)\n",
				 pids[i], lucid_oom_game);
		} else if (lucid_is_killable(task)) {
			/* Deprioritise background processes */
			int old = lucid_read_oom_adj(task);
			if (old >= 0) {
				lucid_pid_cache[cache_idx].pid = pids[i];
				lucid_pid_cache[cache_idx].old_oom = old;
				cache_idx++;
				lucid_write_oom_adj(task, lucid_oom_bg);
				atomic_inc(&lucid_killed_count);
			}
		}
		put_task_struct(task);
	}

	lucid_pid_cache_count = cache_idx;
	mutex_unlock(&lucid_lock);

	pr_info("GrayRavens: lucid: GAME_START '%s': game oom=%d, %d bg procs -> oom=%d\n",
		game_name, lucid_oom_game, cache_idx, lucid_oom_bg);

	pr_debug("lucid: game start — deprioritised %d processes\n",
		 cache_idx);
}

static void lucid_on_game_stop(void)
{
	int i, count;

	if (!lucid_enabled)
		return;

	mutex_lock(&lucid_lock);

	/* Restore game process OOM score first */
	if (lucid_game.active) {
		struct pid *pid_struct = find_get_pid(lucid_game.pid);
		struct task_struct *task;

		if (pid_struct) {
			task = get_pid_task(pid_struct, PIDTYPE_PID);
			put_pid(pid_struct);
			if (task) {
				lucid_write_oom_adj(task, lucid_game.old_oom);
				put_task_struct(task);
			}
		}
		lucid_game.active = false;
	}

	/* Restore background process OOM scores */
	count = lucid_pid_cache_count;
	for (i = 0; i < count; i++) {
		struct pid *pid_struct = find_get_pid(lucid_pid_cache[i].pid);
		struct task_struct *task;

		if (!pid_struct)
			continue;
		task = get_pid_task(pid_struct, PIDTYPE_PID);
		put_pid(pid_struct);
		if (!task)
			continue;

		lucid_write_oom_adj(task, lucid_pid_cache[i].old_oom);
		put_task_struct(task);
	}
	lucid_pid_cache_count = 0;
	mutex_unlock(&lucid_lock);

	pr_info("GrayRavens: lucid: GAME_STOP: restored %d processes\n", count);

	pr_debug("lucid: game stop — restored %d processes\n", count);
}

static int lucid_selene_notifier(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	const char *game_name = data ? (const char *)data : "";

	switch (event) {
	case SELENE_EVENT_GAME_START:
		lucid_on_game_start(game_name);
		break;
	case SELENE_EVENT_GAME_STOP:
		lucid_on_game_stop();
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block lucid_nb = {
	.notifier_call = lucid_selene_notifier,
};

/* ------------------------------------------------------------------ */
/* Sysfs                                                               */
/* ------------------------------------------------------------------ */
static struct kobject *lucid_kobj;

static ssize_t enabled_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", lucid_enabled);
}

static ssize_t enabled_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	lucid_enabled = !!val;
	return count;
}
static struct kobj_attribute enabled_attr =
	__ATTR_RW(enabled);

static ssize_t oom_score_adj_game_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", lucid_oom_game);
}

static ssize_t oom_score_adj_game_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val < -1000 || val > 1000)
		return -ERANGE;
	lucid_oom_game = val;
	return count;
}
static struct kobj_attribute oom_score_adj_game_attr =
	__ATTR_RW(oom_score_adj_game);

static ssize_t oom_score_adj_bg_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", lucid_oom_bg);
}

static ssize_t oom_score_adj_bg_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val < -1000 || val > 1000)
		return -ERANGE;
	lucid_oom_bg = val;
	return count;
}
static struct kobj_attribute oom_score_adj_bg_attr =
	__ATTR_RW(oom_score_adj_bg);

static ssize_t currently_killed_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", atomic_read(&lucid_killed_count));
}
static struct kobj_attribute currently_killed_attr =
	__ATTR_RO(currently_killed);

static ssize_t kill_list_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	int i, pos = 0;

	for (i = 0; i < lucid_pattern_count; i++) {
		if (lucid_patterns[i])
			pos += sysfs_emit_at(buf, pos, "%s\n", lucid_patterns[i]);
	}
	return pos;
}
static struct kobj_attribute kill_list_attr =
	__ATTR_RO(kill_list);

static ssize_t kill_list_add_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	while (count > 0 && (buf[count - 1] == '\n' || buf[count - 1] == ' '))
		count--;

	if (count == 0 || count >= LUCID_PATTERN_LEN)
		return -EINVAL;

	mutex_lock(&lucid_lock);
	if (lucid_pattern_count >= LUCID_PATTERN_MAX) {
		mutex_unlock(&lucid_lock);
		return -ENOSPC;
	}

	lucid_patterns[lucid_pattern_count] = kstrndup(buf, count, GFP_KERNEL);
	if (!lucid_patterns[lucid_pattern_count]) {
		mutex_unlock(&lucid_lock);
		return -ENOMEM;
	}
	lucid_pattern_count++;
	mutex_unlock(&lucid_lock);

	return count;
}
static struct kobj_attribute kill_list_add_attr =
	__ATTR_WO(kill_list_add);

static ssize_t kill_list_del_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int i;

	while (count > 0 && (buf[count - 1] == '\n' || buf[count - 1] == ' '))
		count--;

	if (count == 0)
		return -EINVAL;

	mutex_lock(&lucid_lock);
	for (i = 0; i < lucid_pattern_count; i++) {
		if (lucid_patterns[i] &&
		    strncmp(lucid_patterns[i], buf, count) == 0 &&
		    lucid_patterns[i][count] == '\0') {
			kfree(lucid_patterns[i]);
			lucid_patterns[i] = lucid_patterns[lucid_pattern_count - 1];
			lucid_patterns[lucid_pattern_count - 1] = NULL;
			lucid_pattern_count--;
			mutex_unlock(&lucid_lock);
			return count;
		}
	}
	mutex_unlock(&lucid_lock);
	return -ENOENT;
}
static struct kobj_attribute kill_list_del_attr =
	__ATTR_WO(kill_list_del);

static struct attribute *lucid_attrs[] = {
	&enabled_attr.attr,
	&oom_score_adj_game_attr.attr,
	&oom_score_adj_bg_attr.attr,
	&currently_killed_attr.attr,
	&kill_list_attr.attr,
	&kill_list_add_attr.attr,
	&kill_list_del_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(lucid);

/* ------------------------------------------------------------------ */
/* Init / Exit                                                         */
/* ------------------------------------------------------------------ */

static int __init lucid_init(void)
{
	int ret;

	lucid_kobj = kobject_create_and_add("lucid", kernel_kobj);
	if (!lucid_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(lucid_kobj, lucid_groups);
	if (ret) {
		kobject_put(lucid_kobj);
		return ret;
	}

	{
		int sret = selene_register_notifier(&lucid_nb);

		if (sret)
			pr_warn("lucid: selene notifier register failed (%d)\n", sret);
		else
			pr_info("lucid: subscribed to selene game detection chain\n");
	}
	pr_info("lucid: loaded (oom_game=%d, oom_bg=%d)\n",
		lucid_oom_game, lucid_oom_bg);
	return 0;
}

static void __exit lucid_exit(void)
{
	int i;

	selene_unregister_notifier(&lucid_nb);
	sysfs_remove_groups(lucid_kobj, lucid_groups);
	kobject_put(lucid_kobj);

	for (i = 0; i < lucid_pattern_count; i++)
		kfree(lucid_patterns[i]);

	pr_info("lucid: unloaded\n");
}

module_init(lucid_init);
module_exit(lucid_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Lucid — GrayRavens Game Mode Task Killer & OOM Manager");
MODULE_AUTHOR("GrayRavens");
