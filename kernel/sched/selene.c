// SPDX-License-Identifier: GPL-2.0
/*
 * selene — GrayRavens Game Detection & Profile Engine
 *
 * Maintains a kernel-side database of known game package names
 * and exposes a sysfs interface so userspace (or init scripts)
 * can report the current foreground app.  When a known game is
 * detected the notifier chain fires so peer drivers (lucid,
 * koakuma, zenith, etc.) can adjust performance state.
 *
 * Sysfs entries ( /sys/kernel/selene/ ):
 *   foreground_app    (WO)  — write the current foreground package name
 *   active_game       (RO)  — currently detected game, or "(none)"
 *   active_profile    (RO)  — profile active for current game
 *   game_count        (RO)  — number of entries in the built-in game list
 *   game_check        (WO)  — write a package to test if it is in the list
 *   game_check_result (RO)  — "yes" / "no" from the last check
 *   game_list_add     (WO)  — add a custom game package (newline-terminated)
 *   notify            (RO)  — blocking read of {start,stop} events
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/jhash.h>
#include <linux/workqueue.h>
#include <linux/cgroup.h>
#include <linux/sched/signal.h>
#include <linux/sched/cputime.h>
#include <linux/sched/stat.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include "selene_gamelist.h"
#include <linux/selene.h>

/* ------------------------------------------------------------------ */
/* Notifier chain — peer drivers register here                         */
/* ------------------------------------------------------------------ */
BLOCKING_NOTIFIER_HEAD(selene_chain);

int selene_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&selene_chain, nb);
}

int selene_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&selene_chain, nb);
}

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static DEFINE_MUTEX(selene_lock);

/* Auto-scan interval in ms (0 = disabled) */
static unsigned int selene_scan_interval_ms = 1000;
static struct delayed_work selene_scan_work;
static bool selene_scan_running;

/* Idle-skip bookkeeping: only walk the task list when a process
 * forked or exited since the last scan (total_forks / nr_threads
 * moved), or a game is currently active.  Avoids two full task
 * walks per second on an idle device.
 */
static unsigned long selene_last_forks;
static unsigned int selene_last_nr_threads;

/* Current detected game (NULL if none) */
static char *selene_active_game;
static bool selene_active_is_game;

/* Dedicated result buffer for game_check — NOT shared with active_is_game */
static bool selene_check_result;

/* Runtime-added custom games (linked list) */
struct selene_custom_game {
	char name[128];
	struct list_head list;
};
static LIST_HEAD(selene_custom_games);

/* Forward declarations */
static bool selene_is_game_name(const char *name, unsigned int len);
static void selene_set_game(const char *name, unsigned int len);

/* Custom game list file path — loaded at init */
#define SELENE_CUSTOM_LIST_PATH "/data/misc/selene/custom_games.txt"

/*
 * Load additional game entries from a file.  Format: one package name
 * per line.  Skips empty lines and lines starting with '#'.  Merged with
 * the built-in game list at lookup time.  Called once at init -- the
 * file is advisory (non-existence is not an error).
 */
static void selene_load_custom_list(void)
{
	struct file *f;
	loff_t pos = 0;
	char *buf;
	loff_t size;
	char *line, *next;

	f = filp_open(SELENE_CUSTOM_LIST_PATH, O_RDONLY, 0);
	if (IS_ERR(f))
		return;

	size = i_size_read(file_inode(f));
	if (size <= 0 || size > 65536) {
		filp_close(f, NULL);
		return;
	}

	buf = vmalloc((size_t)size + 1);
	if (!buf) {
		filp_close(f, NULL);
		return;
	}

	if (kernel_read(f, buf, (size_t)size, &pos) == (ssize_t)size) {
		buf[size] = '\0';
		line = buf;

		while (line && *line) {
			struct selene_custom_game *cg;
			size_t len;

			/* Find end of line */
			next = strchr(line, '\n');
			if (next)
				*next++ = '\0';

			/* Trim trailing whitespace */
			len = strlen(line);
			while (len > 0 && (line[len - 1] == ' ' ||
					   line[len - 1] == '\t' ||
					   line[len - 1] == '\r'))
				line[--len] = '\0';

			/* Skip empty / comment lines */
			if (len == 0 || line[0] == '#') {
				line = next;
				continue;
			}

			if (len >= 128) {
				line = next;
				continue;
			}

			cg = kzalloc(sizeof(*cg), GFP_KERNEL);
			if (!cg)
				break;

			memcpy(cg->name, line, len);
			cg->name[len] = '\0';

			mutex_lock(&selene_lock);
			list_add_tail(&cg->list, &selene_custom_games);
			mutex_unlock(&selene_lock);

			line = next;
		}

		pr_info("selene: loaded custom games from %s\n",
			SELENE_CUSTOM_LIST_PATH);
	}

	vfree(buf);
	filp_close(f, NULL);
}


/* ------------------------------------------------------------------ */
/* Top-app cgroup detection — automatic game discovery                 */
/* ------------------------------------------------------------------ */

static bool selene_task_in_top_app(struct task_struct *p)
{
	struct cgroup_subsys_state *css;
	bool in_topapp = false;

	rcu_read_lock();
	css = task_css(p, cpuset_cgrp_id);
	if (css && css->cgroup && css->cgroup->kn)
		in_topapp = (strcmp(css->cgroup->kn->name, "top-app") == 0);
	rcu_read_unlock();

	return in_topapp;
}

/* Read /proc/pid/cmdline for a task (first NUL-terminated segment) */
static int selene_read_cmdline(struct task_struct *task, char *buf, size_t size)
{
	char path[64];
	struct file *f;
	loff_t pos = 0;
	int ret;

	snprintf(path, sizeof(path), "/proc/%d/cmdline", task->pid);
	f = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	ret = kernel_read(f, buf, size - 1, &pos);
	filp_close(f, NULL);
	if (ret > 0) {
		/* cmdline is NUL-separated; truncate at first NUL */
		int i;
		for (i = 0; i < ret; i++) {
			if (buf[i] == '\0')
				break;
		}
		buf[i] = '\0';
	}
	return ret;
}

/* Scanning workqueue — runs every scan_interval_ms */
static void selene_scan_work_fn(struct work_struct *work)
{
	struct task_struct *p;
	static char last_game[128];
	char cmdline[256];
	char comm[TASK_COMM_LEN];
	bool found = false;
	pid_t found_pid = 0;

	if (!selene_scan_running)
		return;

	/*
	 * Idle-skip: when no game is active and no process has forked or
	 * exited since the last scan, nothing could have become (or stopped
	 * being) a game, so skip the full task walk and just re-arm the
	 * timer.  total_forks / nr_threads are cheap atomic reads.  Android
	 * launches games via Zygote fork (moves nr_threads) and games exit
	 * as a thread group (also moves nr_threads), so both transitions
	 * are caught; exec-in-place into a game binary without a fork is
	 * not how Android starts games and is the only missed case.
	 */
	if (!selene_active_is_game &&
	    total_forks == selene_last_forks &&
	    nr_threads == selene_last_nr_threads)
		goto resched;
	selene_last_forks = total_forks;
	selene_last_nr_threads = nr_threads;

	/* Phase 1: collect candidate PID under RCU (fast, no sleeping) */
	rcu_read_lock();
	for_each_process(p) {
		if (p->pid <= 1 || p->flags & PF_KTHREAD)
			continue;

		if (!selene_task_in_top_app(p))
			continue;

		/* Try comm first (fast path) */
		get_task_comm(comm, p);
		if (selene_is_game_name(comm, strlen(comm))) {
			found = true;
			found_pid = task_pid_nr(p);
			break;
		}

		/*
		 * cmdline check deferred to Phase 2 (needs filp_open
		 * which can sleep).  Record PID and stop — same
		 * one-per-tick behaviour as the original loop.
		 */
		found_pid = task_pid_nr(p);
		break;
	}
	rcu_read_unlock();

	/* Phase 2: check cmdline outside RCU (can sleep) */
	if (found) {
		/* Found via comm — use comm as the game name */
		if (strcmp(comm, last_game) != 0) {
			strscpy(last_game, comm, sizeof(last_game));
			selene_set_game(comm, strlen(comm));
		}
	} else if (found_pid) {
		struct pid *pid_struct = find_get_pid(found_pid);
		struct task_struct *task;

		if (pid_struct) {
			task = get_pid_task(pid_struct, PIDTYPE_PID);
			put_pid(pid_struct);
			if (task) {
				if (selene_read_cmdline(task, cmdline,
							sizeof(cmdline)) > 0 &&
				    selene_is_game_name(cmdline,
						strlen(cmdline))) {
					if (strcmp(cmdline, last_game) != 0) {
						strscpy(last_game, cmdline,
							sizeof(last_game));
						selene_set_game(cmdline,
								strlen(cmdline));
					}
				}
				put_task_struct(task);
			}
		}
	} else if (last_game[0]) {
		last_game[0] = '\0';
		selene_set_game(NULL, 0);
	}

resched:
	if (selene_scan_running && selene_scan_interval_ms > 0)
		schedule_delayed_work(&selene_scan_work,
				      msecs_to_jiffies(selene_scan_interval_ms));
}

/* ------------------------------------------------------------------ */
/* Lookup helpers                                                      */
/* ------------------------------------------------------------------ */

/* Public wrapper — used by the scanning timer */
static bool selene_is_game_name(const char *name, unsigned int len)
{
	struct selene_custom_game *cg;

	mutex_lock(&selene_lock);
	if (selene_game_lookup(name, len)) {
		mutex_unlock(&selene_lock);
		return true;
	}
	list_for_each_entry(cg, &selene_custom_games, list) {
		if (strcmp(cg->name, name) == 0) {
			mutex_unlock(&selene_lock);
			return true;
		}
	}
	mutex_unlock(&selene_lock);
	return false;
}

static bool selene_is_game_locked(const char *name, unsigned int len)
{
	struct selene_custom_game *cg;

	/* Check built-in list */
	if (selene_game_lookup(name, len))
		return true;

	/* Check custom list */
	list_for_each_entry(cg, &selene_custom_games, list) {
		if (strcmp(cg->name, name) == 0)
			return true;
	}
	return false;
}

/* ------------------------------------------------------------------ */
/* State transitions                                                   */
/* ------------------------------------------------------------------ */

static void selene_set_game(const char *name, unsigned int len)
{
	char *old = selene_active_game;

	mutex_lock(&selene_lock);

	if (name && len > 0) {
		bool is_game = selene_is_game_locked(name, len);

		if (selene_active_game && strcmp(selene_active_game, name) == 0) {
			/* Same app still active — keep existing string */
			mutex_unlock(&selene_lock);
			return;
		}

		/* Different app — stop previous game first */
		if (selene_active_is_game && selene_active_game) {
			printk(KERN_INFO "GrayRavens: selene: GAME_STOP '%s'\n",
			       selene_active_game);
			blocking_notifier_call_chain(&selene_chain,
						     SELENE_EVENT_GAME_STOP,
						     selene_active_game);
		}

		/* Switch to new app */
		selene_active_game = kstrndup(name, len, GFP_KERNEL);
		selene_active_is_game = is_game;

		if (is_game) {
			printk(KERN_INFO "GrayRavens: selene: GAME_START '%s'\n",
			       selene_active_game);
			blocking_notifier_call_chain(&selene_chain,
						     SELENE_EVENT_GAME_START,
						     selene_active_game);
		}
	} else {
		/* Clear */
		if (selene_active_is_game && selene_active_game) {
			printk(KERN_INFO "GrayRavens: selene: GAME_STOP '%s'\n",
			       selene_active_game);
			blocking_notifier_call_chain(&selene_chain,
						     SELENE_EVENT_GAME_STOP,
						     selene_active_game);
		}
		selene_active_game = NULL;
		selene_active_is_game = false;
	}

	mutex_unlock(&selene_lock);
	kfree(old);
}

/* ------------------------------------------------------------------ */
/* Sysfs                                                               */
/* ------------------------------------------------------------------ */

static struct kobject *selene_kobj;

static ssize_t foreground_app_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	/* Strip trailing newline */
	while (count > 0 && (buf[count - 1] == '\n' || buf[count - 1] == ' '))
		count--;

	if (count == 0)
		selene_set_game(NULL, 0);
	else
		selene_set_game(buf, count);

	return count;
}
static struct kobj_attribute foreground_app_attr =
	__ATTR(foreground_app, 0200, NULL, foreground_app_store);

static ssize_t active_game_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	ssize_t ret;

	mutex_lock(&selene_lock);
	if (selene_active_game && selene_active_is_game)
		ret = sysfs_emit(buf, "%s\n", selene_active_game);
	else
		ret = sysfs_emit(buf, "(none)\n");
	mutex_unlock(&selene_lock);
	return ret;
}
static struct kobj_attribute active_game_attr =
	__ATTR_RO(active_game);

static ssize_t active_profile_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	ssize_t ret;

	mutex_lock(&selene_lock);
	if (selene_active_game && selene_active_is_game)
		ret = sysfs_emit(buf, "game\n");
	else
		ret = sysfs_emit(buf, "normal\n");
	mutex_unlock(&selene_lock);
	return ret;
}
static struct kobj_attribute active_profile_attr =
	__ATTR_RO(active_profile);

static ssize_t game_count_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	struct selene_custom_game *cg;
	int custom = 0;

	mutex_lock(&selene_lock);
	list_for_each_entry(cg, &selene_custom_games, list)
		custom++;
	mutex_unlock(&selene_lock);

	return sysfs_emit(buf, "%d\n", SELENE_GAME_COUNT + custom);
}
static struct kobj_attribute game_count_attr =
	__ATTR_RO(game_count);

static ssize_t game_check_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	while (count > 0 && (buf[count - 1] == '\n' || buf[count - 1] == ' '))
		count--;

	mutex_lock(&selene_lock);
	selene_check_result = selene_is_game_locked(buf, count);
	mutex_unlock(&selene_lock);

	return count;
}
static struct kobj_attribute game_check_attr =
	__ATTR_WO(game_check);

static ssize_t game_check_result_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	bool result;

	mutex_lock(&selene_lock);
	result = selene_check_result;
	mutex_unlock(&selene_lock);

	return sysfs_emit(buf, "%s\n", result ? "yes" : "no");
}
static struct kobj_attribute game_check_result_attr =
	__ATTR_RO(game_check_result);

static ssize_t game_list_add_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	struct selene_custom_game *cg;

	while (count > 0 && (buf[count - 1] == '\n' || buf[count - 1] == ' '))
		count--;

	if (count == 0 || count >= 128)
		return -EINVAL;

	cg = kzalloc(sizeof(*cg), GFP_KERNEL);
	if (!cg)
		return -ENOMEM;

	memcpy(cg->name, buf, count);
	cg->name[count] = '\0';

	mutex_lock(&selene_lock);
	list_add_tail(&cg->list, &selene_custom_games);
	mutex_unlock(&selene_lock);

	return count;
}
static struct kobj_attribute game_list_add_attr =
	__ATTR_WO(game_list_add);

static struct attribute *selene_attrs[] = {
	&foreground_app_attr.attr,
	&active_game_attr.attr,
	&active_profile_attr.attr,
	&game_count_attr.attr,
	&game_check_attr.attr,
	&game_check_result_attr.attr,
	&game_list_add_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(selene);

/* ------------------------------------------------------------------ */
/* Init / Exit                                                         */
/* ------------------------------------------------------------------ */

static int __init selene_init(void)
{
	int ret;

	selene_kobj = kobject_create_and_add("selene", kernel_kobj);
	if (!selene_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(selene_kobj, selene_groups);
	if (ret) {
		kobject_put(selene_kobj);
		return ret;
	}

	/* Load custom game list from persistent file */
	selene_load_custom_list();

	/* Start the auto-scan timer */
	INIT_DELAYED_WORK(&selene_scan_work, selene_scan_work_fn);
	selene_scan_running = true;
	schedule_delayed_work(&selene_scan_work,
			      msecs_to_jiffies(selene_scan_interval_ms));

	pr_info("selene: loaded with %d built-in + custom games (scan interval %u ms)\n",
		SELENE_GAME_COUNT, selene_scan_interval_ms);
	return 0;
}

static void __exit selene_exit(void)
{
	struct selene_custom_game *cg, *tmp;

	selene_scan_running = false;
	cancel_delayed_work_sync(&selene_scan_work);

	sysfs_remove_groups(selene_kobj, selene_groups);
	kobject_put(selene_kobj);

	list_for_each_entry_safe(cg, tmp, &selene_custom_games, list) {
		list_del(&cg->list);
		kfree(cg);
	}

	kfree(selene_active_game);
	pr_info("selene: unloaded\n");
}

module_init(selene_init);
module_exit(selene_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Selene — GrayRavens Game Detection & Profile Engine");
MODULE_AUTHOR("GrayRavens");
