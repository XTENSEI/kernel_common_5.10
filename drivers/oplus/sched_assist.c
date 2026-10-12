// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 * Adapted for Linux 5.10 GKI by RapliVx
 *
 * Implements the OPlus Sched Assist (/proc/oplus_scheduler/sched_assist/)
 * interface utilized by ColorOS PerformanceService, Athena, and SystemUI
 * for user experience (UX) threads, scene-aware CPU QoS boosting,
 * and critical task latency management.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysctl.h>

#define MAX_TRACKED_TASKS 64
#define SCENE_WATCHDOG_TIMEOUT_MS 3000

/* Procfs directory entries */
static struct proc_dir_entry *oplus_sched_dir;
static struct proc_dir_entry *sched_assist_dir;
static struct proc_dir_entry *audio_dir;

/* Sched Assist state */
static int current_scene;
static unsigned long scene_timestamp;
static int disable_setting;
static int im_flag_val;
static int im_flag_app_val;
static int audio_enable_val;
static int audio_status_val;
static int tpd_cmds_val;
static int tpd_id_val;
static DEFINE_SPINLOCK(scene_lock);

/* Frequency QoS requests per online policy */
#define MAX_CPU_POLICIES 4
struct sched_assist_qos {
	struct cpufreq_policy *policy;
	struct freq_qos_request req;
	bool active;
};
static struct sched_assist_qos qos_controllers[MAX_CPU_POLICIES];
static int qos_count;
static struct timer_list scene_watchdog;

/* Task tracking table */
struct task_boost_entry {
	pid_t pid;
	char comm[TASK_COMM_LEN];
	int orig_nice;
	int boosted_nice;
	unsigned long timestamp;
	bool active;
};

static struct task_boost_entry ux_tasks[MAX_TRACKED_TASKS];
static DEFINE_SPINLOCK(ux_lock);

static struct task_boost_entry app_ux_tasks[MAX_TRACKED_TASKS];
static DEFINE_SPINLOCK(app_ux_lock);

static struct task_boost_entry ddl_tasks[MAX_TRACKED_TASKS];
static DEFINE_SPINLOCK(ddl_lock);

static struct task_boost_entry impt_tasks[MAX_TRACKED_TASKS];
static DEFINE_SPINLOCK(impt_lock);

static const char *get_scene_name(int scene)
{
	switch (scene) {
	case 0: return "NORMAL";
	case 1: return "APP_LAUNCH";
	case 2: return "SLIDE_SCROLL";
	case 3: return "ANIMATION_TOUCH";
	case 4: return "CAMERA";
	case 5: return "GAME";
	default: return "CUSTOM";
	}
}

static void apply_cpu_qos_boost(int scene)
{
	int i;
	s32 target_freq_khz = 0;

	if (disable_setting)
		scene = 0;

	switch (scene) {
	case 1: /* App Launch: aggressive boost */
		target_freq_khz = 1500000;
		break;
	case 2: /* Slide/Scroll: moderate boost */
		target_freq_khz = 1200000;
		break;
	case 3: /* Animation/Touch: low latency boost */
		target_freq_khz = 1000000;
		break;
	case 4: /* Camera */
		target_freq_khz = 1300000;
		break;
	case 5: /* Game */
		target_freq_khz = 1400000;
		break;
	default:
		target_freq_khz = 0;
		break;
	}

	for (i = 0; i < qos_count; i++) {
		if (qos_controllers[i].active) {
			s32 boost = target_freq_khz;

			if (boost > qos_controllers[i].policy->max)
				boost = qos_controllers[i].policy->max;
			freq_qos_update_request(&qos_controllers[i].req, boost);
		}
	}
}

static void scene_watchdog_fn(struct timer_list *t)
{
	unsigned long flags;

	spin_lock_irqsave(&scene_lock, flags);
	if (current_scene != 0) {
		current_scene = 0;
		apply_cpu_qos_boost(0);
	}
	spin_unlock_irqrestore(&scene_lock, flags);
}

static void add_or_update_task(struct task_boost_entry *table, spinlock_t *lock,
			       pid_t pid, int target_nice, bool enable)
{
	int i, slot = -1;
	unsigned long flags;
	struct task_struct *task;
	bool do_nice = false;
	int nice_to_apply = 0;

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();

	if (!task)
		return;

	spin_lock_irqsave(lock, flags);

	for (i = 0; i < MAX_TRACKED_TASKS; i++) {
		if (table[i].active && table[i].pid == pid) {
			slot = i;
			break;
		}
		if (!table[i].active && slot == -1)
			slot = i;
	}

	if (enable && !disable_setting) {
		if (slot != -1) {
			if (!table[slot].active) {
				table[slot].pid = pid;
				table[slot].orig_nice = task_nice(task);
				strscpy(table[slot].comm, task->comm, sizeof(table[slot].comm));
				table[slot].active = true;
			}
			table[slot].boosted_nice = target_nice;
			table[slot].timestamp = jiffies;

			if (task_nice(task) > target_nice) {
				do_nice = true;
				nice_to_apply = target_nice;
			}
		}
	} else {
		if (slot != -1 && table[slot].active) {
			do_nice = true;
			nice_to_apply = table[slot].orig_nice;
			table[slot].active = false;
		}
	}

	spin_unlock_irqrestore(lock, flags);

	if (do_nice)
		set_user_nice(task, nice_to_apply);

	put_task_struct(task);
}

/* /proc/oplus_scheduler/sched_assist/sched_assist_scene */
static int sched_assist_scene_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	int scene;
	unsigned long elapsed_ms = 0;

	spin_lock_irqsave(&scene_lock, flags);
	scene = current_scene;
	if (scene != 0)
		elapsed_ms = jiffies_to_msecs(jiffies - scene_timestamp);
	spin_unlock_irqrestore(&scene_lock, flags);

	seq_printf(m, "scene: %d\n", scene);
	seq_printf(m, "name: %s\n", get_scene_name(scene));
	seq_printf(m, "duration_ms: %lu\n", elapsed_ms);
	seq_printf(m, "disabled: %d\n", disable_setting);
	return 0;
}

static int sched_assist_scene_open(struct inode *inode, struct file *file)
{
	return single_open(file, sched_assist_scene_show, NULL);
}

static ssize_t sched_assist_scene_write(struct file *filp, const char __user *ubuf,
					size_t cnt, loff_t *ppos)
{
	char kbuf[32];
	int val;
	unsigned long flags;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	if (kstrtoint(strstrip(kbuf), 0, &val))
		return -EINVAL;

	spin_lock_irqsave(&scene_lock, flags);
	current_scene = val;
	scene_timestamp = jiffies;

	if (val > 0 && !disable_setting) {
		apply_cpu_qos_boost(val);
		mod_timer(&scene_watchdog, jiffies + msecs_to_jiffies(SCENE_WATCHDOG_TIMEOUT_MS));
	} else {
		apply_cpu_qos_boost(0);
		del_timer(&scene_watchdog);
	}
	spin_unlock_irqrestore(&scene_lock, flags);

	return cnt;
}

static const struct proc_ops sched_assist_scene_proc_ops = {
	.proc_open    = sched_assist_scene_open,
	.proc_read    = seq_read,
	.proc_write   = sched_assist_scene_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

#ifdef CONFIG_SYSCTL
static int sched_assist_scene_sysctl(struct ctl_table *table, int write,
				     void *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;
	unsigned long flags;

	ret = proc_dointvec(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	spin_lock_irqsave(&scene_lock, flags);
	scene_timestamp = jiffies;
	if (current_scene > 0 && !disable_setting) {
		apply_cpu_qos_boost(current_scene);
		mod_timer(&scene_watchdog,
			  jiffies + msecs_to_jiffies(SCENE_WATCHDOG_TIMEOUT_MS));
	} else {
		apply_cpu_qos_boost(0);
		del_timer(&scene_watchdog);
	}
	spin_unlock_irqrestore(&scene_lock, flags);

	return 0;
}

static struct ctl_table sched_assist_kern_table[] = {
	{
		.procname     = "sched_assist_scene",
		.data         = &current_scene,
		.maxlen       = sizeof(int),
		.mode         = 0644,
		.proc_handler = sched_assist_scene_sysctl,
	},
	{ }
};

static struct ctl_table sched_assist_root_table[] = {
	{
		.procname = "kernel",
		.mode     = 0555,
		.child    = sched_assist_kern_table,
	},
	{ }
};
static struct ctl_table_header *sched_assist_sysctl_header;
#endif

/* /proc/oplus_scheduler/sched_assist/ux_task */
static int ux_task_show(struct seq_file *m, void *v)
{
	int i, count = 0;
	unsigned long flags;

	seq_puts(m, "[UX Tasks]\n");
	seq_puts(m, "PID\tNICE(ORIG/CUR)\tCOMM\n");

	spin_lock_irqsave(&ux_lock, flags);
	for (i = 0; i < MAX_TRACKED_TASKS; i++) {
		if (ux_tasks[i].active) {
			count++;
			seq_printf(m, "%d\t%d/%d\t\t%s\n",
				   ux_tasks[i].pid,
				   ux_tasks[i].orig_nice,
				   ux_tasks[i].boosted_nice,
				   ux_tasks[i].comm);
		}
	}
	spin_unlock_irqrestore(&ux_lock, flags);

	seq_printf(m, "Total UX tasks: %d\n", count);
	return 0;
}

static int ux_task_open(struct inode *inode, struct file *file)
{
	return single_open(file, ux_task_show, NULL);
}

static ssize_t ux_task_write(struct file *filp, const char __user *ubuf,
			     size_t cnt, loff_t *ppos)
{
	char kbuf[64];
	char *token, *s;
	pid_t pid = 0;
	int val = 1;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	s = strstrip(kbuf);
	token = strsep(&s, " ");
	if (!token || kstrtoint(token, 0, &pid))
		return -EINVAL;

	if (s) {
		token = strstrip(s);
		if (kstrtoint(token, 0, &val))
			val = 1;
	}

	add_or_update_task(ux_tasks, &ux_lock, pid, -3, (val != 0));
	return cnt;
}

static const struct proc_ops ux_task_proc_ops = {
	.proc_open    = ux_task_open,
	.proc_read    = seq_read,
	.proc_write   = ux_task_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/ux_task_app */
static int ux_task_app_show(struct seq_file *m, void *v)
{
	int i, count = 0;
	unsigned long flags;

	seq_puts(m, "[App UX Tasks]\n");
	seq_puts(m, "PID\tNICE(ORIG/CUR)\tCOMM\n");

	spin_lock_irqsave(&app_ux_lock, flags);
	for (i = 0; i < MAX_TRACKED_TASKS; i++) {
		if (app_ux_tasks[i].active) {
			count++;
			seq_printf(m, "%d\t%d/%d\t\t%s\n",
				   app_ux_tasks[i].pid,
				   app_ux_tasks[i].orig_nice,
				   app_ux_tasks[i].boosted_nice,
				   app_ux_tasks[i].comm);
		}
	}
	spin_unlock_irqrestore(&app_ux_lock, flags);

	seq_printf(m, "Total App UX tasks: %d\n", count);
	return 0;
}

static int ux_task_app_open(struct inode *inode, struct file *file)
{
	return single_open(file, ux_task_app_show, NULL);
}

static ssize_t ux_task_app_write(struct file *filp, const char __user *ubuf,
				 size_t cnt, loff_t *ppos)
{
	char kbuf[64];
	char *token, *s;
	pid_t pid = 0;
	int val = 1;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	s = strstrip(kbuf);
	token = strsep(&s, " ");
	if (!token || kstrtoint(token, 0, &pid))
		return -EINVAL;

	if (s) {
		token = strstrip(s);
		if (kstrtoint(token, 0, &val))
			val = 1;
	}

	add_or_update_task(app_ux_tasks, &app_ux_lock, pid, -3, (val != 0));
	return cnt;
}

static const struct proc_ops ux_task_app_proc_ops = {
	.proc_open    = ux_task_app_open,
	.proc_read    = seq_read,
	.proc_write   = ux_task_app_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/ddl_task */
static int ddl_task_show(struct seq_file *m, void *v)
{
	int i, count = 0;
	unsigned long flags;

	seq_puts(m, "[DDL Tasks]\n");
	seq_puts(m, "PID\tNICE(ORIG/CUR)\tCOMM\n");

	spin_lock_irqsave(&ddl_lock, flags);
	for (i = 0; i < MAX_TRACKED_TASKS; i++) {
		if (ddl_tasks[i].active) {
			count++;
			seq_printf(m, "%d\t%d/%d\t\t%s\n",
				   ddl_tasks[i].pid,
				   ddl_tasks[i].orig_nice,
				   ddl_tasks[i].boosted_nice,
				   ddl_tasks[i].comm);
		}
	}
	spin_unlock_irqrestore(&ddl_lock, flags);

	seq_printf(m, "Total DDL tasks: %d\n", count);
	return 0;
}

static int ddl_task_open(struct inode *inode, struct file *file)
{
	return single_open(file, ddl_task_show, NULL);
}

static ssize_t ddl_task_write(struct file *filp, const char __user *ubuf,
			      size_t cnt, loff_t *ppos)
{
	char kbuf[64];
	char *token, *s;
	pid_t pid = 0;
	int val = 1;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	s = strstrip(kbuf);
	token = strsep(&s, " ");
	if (!token || kstrtoint(token, 0, &pid))
		return -EINVAL;

	if (s) {
		token = strstrip(s);
		if (kstrtoint(token, 0, &val))
			val = 1;
	}

	add_or_update_task(ddl_tasks, &ddl_lock, pid, -4, (val != 0));
	return cnt;
}

static const struct proc_ops ddl_task_proc_ops = {
	.proc_open    = ddl_task_open,
	.proc_read    = seq_read,
	.proc_write   = ddl_task_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/sched_impt_task */
static int sched_impt_task_show(struct seq_file *m, void *v)
{
	int i, count = 0;
	unsigned long flags;

	seq_puts(m, "[Important Tasks]\n");
	seq_puts(m, "PID\tNICE(ORIG/CUR)\tCOMM\n");

	spin_lock_irqsave(&impt_lock, flags);
	for (i = 0; i < MAX_TRACKED_TASKS; i++) {
		if (impt_tasks[i].active) {
			count++;
			seq_printf(m, "%d\t%d/%d\t\t%s\n",
				   impt_tasks[i].pid,
				   impt_tasks[i].orig_nice,
				   impt_tasks[i].boosted_nice,
				   impt_tasks[i].comm);
		}
	}
	spin_unlock_irqrestore(&impt_lock, flags);

	seq_printf(m, "Total Important tasks: %d\n", count);
	return 0;
}

static int sched_impt_task_open(struct inode *inode, struct file *file)
{
	return single_open(file, sched_impt_task_show, NULL);
}

static ssize_t sched_impt_task_write(struct file *filp, const char __user *ubuf,
				     size_t cnt, loff_t *ppos)
{
	char kbuf[64];
	char *token, *s;
	pid_t pid = 0;
	int val = 1;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	s = strstrip(kbuf);
	token = strsep(&s, " ");
	if (!token || kstrtoint(token, 0, &pid))
		return -EINVAL;

	if (s) {
		token = strstrip(s);
		if (kstrtoint(token, 0, &val))
			val = 1;
	}

	add_or_update_task(impt_tasks, &impt_lock, pid, -1, (val != 0));
	return cnt;
}

static const struct proc_ops sched_impt_task_proc_ops = {
	.proc_open    = sched_impt_task_open,
	.proc_read    = seq_read,
	.proc_write   = sched_impt_task_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/im_flag */
static int im_flag_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", im_flag_val);
	return 0;
}

static int im_flag_open(struct inode *inode, struct file *file)
{
	return single_open(file, im_flag_show, NULL);
}

static ssize_t im_flag_write(struct file *filp, const char __user *ubuf,
			     size_t cnt, loff_t *ppos)
{
	char kbuf[16];
	int val;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	if (kstrtoint(strstrip(kbuf), 0, &val))
		return -EINVAL;

	im_flag_val = val;
	return cnt;
}

static const struct proc_ops im_flag_proc_ops = {
	.proc_open    = im_flag_open,
	.proc_read    = seq_read,
	.proc_write   = im_flag_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/im_flag_app */
static int im_flag_app_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", im_flag_app_val);
	return 0;
}

static int im_flag_app_open(struct inode *inode, struct file *file)
{
	return single_open(file, im_flag_app_show, NULL);
}

static ssize_t im_flag_app_write(struct file *filp, const char __user *ubuf,
				 size_t cnt, loff_t *ppos)
{
	char kbuf[16];
	int val;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	if (kstrtoint(strstrip(kbuf), 0, &val))
		return -EINVAL;

	im_flag_app_val = val;
	return cnt;
}

static const struct proc_ops im_flag_app_proc_ops = {
	.proc_open    = im_flag_app_open,
	.proc_read    = seq_read,
	.proc_write   = im_flag_app_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/disable_setting */
static int disable_setting_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", disable_setting);
	return 0;
}

static int disable_setting_open(struct inode *inode, struct file *file)
{
	return single_open(file, disable_setting_show, NULL);
}

static ssize_t disable_setting_write(struct file *filp, const char __user *ubuf,
				     size_t cnt, loff_t *ppos)
{
	char kbuf[16];
	int val;
	unsigned long flags;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	if (kstrtoint(strstrip(kbuf), 0, &val))
		return -EINVAL;

	disable_setting = val;
	if (disable_setting) {
		spin_lock_irqsave(&scene_lock, flags);
		current_scene = 0;
		apply_cpu_qos_boost(0);
		del_timer(&scene_watchdog);
		spin_unlock_irqrestore(&scene_lock, flags);
	}

	return cnt;
}

static const struct proc_ops disable_setting_proc_ops = {
	.proc_open    = disable_setting_open,
	.proc_read    = seq_read,
	.proc_write   = disable_setting_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/audio/enable */
static int audio_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", audio_enable_val);
	return 0;
}

static int audio_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, audio_enable_show, NULL);
}

static ssize_t audio_enable_write(struct file *filp, const char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	char kbuf[16];
	int val;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	if (kstrtoint(strstrip(kbuf), 0, &val))
		return -EINVAL;

	audio_enable_val = val;
	return cnt;
}

static const struct proc_ops audio_enable_proc_ops = {
	.proc_open    = audio_enable_open,
	.proc_read    = seq_read,
	.proc_write   = audio_enable_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/audio/status */
static int audio_status_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", audio_status_val);
	return 0;
}

static int audio_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, audio_status_show, NULL);
}

static ssize_t audio_status_write(struct file *filp, const char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	char kbuf[16];
	int val;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	if (kstrtoint(strstrip(kbuf), 0, &val))
		return -EINVAL;

	audio_status_val = val;
	return cnt;
}

static const struct proc_ops audio_status_proc_ops = {
	.proc_open    = audio_status_open,
	.proc_read    = seq_read,
	.proc_write   = audio_status_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/tpd_cmds */
static int tpd_cmds_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", tpd_cmds_val);
	return 0;
}

static int tpd_cmds_open(struct inode *inode, struct file *file)
{
	return single_open(file, tpd_cmds_show, NULL);
}

static ssize_t tpd_cmds_write(struct file *filp, const char __user *ubuf,
			      size_t cnt, loff_t *ppos)
{
	char kbuf[16];
	int val;
	unsigned long flags;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	if (kstrtoint(strstrip(kbuf), 0, &val))
		return -EINVAL;

	tpd_cmds_val = val;
	if (val > 0 && !disable_setting) {
		spin_lock_irqsave(&scene_lock, flags);
		current_scene = 3; /* ANIMATION_TOUCH */
		scene_timestamp = jiffies;
		apply_cpu_qos_boost(3);
		mod_timer(&scene_watchdog,
			  jiffies + msecs_to_jiffies(1000));
		spin_unlock_irqrestore(&scene_lock, flags);
	}

	return cnt;
}

static const struct proc_ops tpd_cmds_proc_ops = {
	.proc_open    = tpd_cmds_open,
	.proc_read    = seq_read,
	.proc_write   = tpd_cmds_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_scheduler/sched_assist/tpd_id */
static int tpd_id_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", tpd_id_val);
	return 0;
}

static int tpd_id_open(struct inode *inode, struct file *file)
{
	return single_open(file, tpd_id_show, NULL);
}

static ssize_t tpd_id_write(struct file *filp, const char __user *ubuf,
			    size_t cnt, loff_t *ppos)
{
	char kbuf[16];
	int val;

	if (cnt >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	if (kstrtoint(strstrip(kbuf), 0, &val))
		return -EINVAL;

	tpd_id_val = val;
	return cnt;
}

static const struct proc_ops tpd_id_proc_ops = {
	.proc_open    = tpd_id_open,
	.proc_read    = seq_read,
	.proc_write   = tpd_id_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

static void init_qos_controllers(void)
{
	int cpu;
	struct cpufreq_policy *policy;

	qos_count = 0;
	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		/* Check if we already registered a controller for this policy */
		if (qos_count > 0 && qos_controllers[qos_count - 1].policy == policy) {
			cpufreq_cpu_put(policy);
			continue;
		}

		if (qos_count < MAX_CPU_POLICIES) {
			int ret = freq_qos_add_request(&policy->constraints,
						       &qos_controllers[qos_count].req,
						       FREQ_QOS_MIN, 0);
			if (ret >= 0) {
				qos_controllers[qos_count].policy = policy;
				qos_controllers[qos_count].active = true;
				qos_count++;
			} else {
				cpufreq_cpu_put(policy);
			}
		} else {
			cpufreq_cpu_put(policy);
		}
	}
}

static void cleanup_qos_controllers(void)
{
	int i;

	for (i = 0; i < qos_count; i++) {
		if (qos_controllers[i].active) {
			freq_qos_remove_request(&qos_controllers[i].req);
			cpufreq_cpu_put(qos_controllers[i].policy);
			qos_controllers[i].active = false;
		}
	}
	qos_count = 0;
}

static int __init oplus_sched_assist_init(void)
{
	timer_setup(&scene_watchdog, scene_watchdog_fn, 0);
	init_qos_controllers();

	oplus_sched_dir = proc_mkdir("oplus_scheduler", NULL);
	if (!oplus_sched_dir) {
		pr_err("oplus_sched_assist: Failed to create /proc/oplus_scheduler\n");
		cleanup_qos_controllers();
		return -ENOMEM;
	}

	sched_assist_dir = proc_mkdir("sched_assist", oplus_sched_dir);
	if (!sched_assist_dir) {
		pr_err("oplus_sched_assist: Failed to create sched_assist dir\n");
		remove_proc_entry("oplus_scheduler", NULL);
		cleanup_qos_controllers();
		return -ENOMEM;
	}

	audio_dir = proc_mkdir("audio", sched_assist_dir);

	proc_create("sched_assist_scene", 0644, sched_assist_dir,
		    &sched_assist_scene_proc_ops);
	proc_create("ux_task", 0644, sched_assist_dir, &ux_task_proc_ops);
	proc_create("ux_task_app", 0644, sched_assist_dir,
		    &ux_task_app_proc_ops);
	proc_create("ddl_task", 0644, sched_assist_dir, &ddl_task_proc_ops);
	proc_create("sched_impt_task", 0644, sched_assist_dir,
		    &sched_impt_task_proc_ops);
	proc_create("im_flag", 0644, sched_assist_dir, &im_flag_proc_ops);
	proc_create("im_flag_app", 0644, sched_assist_dir,
		    &im_flag_app_proc_ops);
	proc_create("disable_setting", 0644, sched_assist_dir,
		    &disable_setting_proc_ops);
	proc_create("tpd_cmds", 0644, sched_assist_dir, &tpd_cmds_proc_ops);
	proc_create("tpd_id", 0644, sched_assist_dir, &tpd_id_proc_ops);

	if (audio_dir) {
		proc_create("enable", 0644, audio_dir, &audio_enable_proc_ops);
		proc_create("status", 0644, audio_dir, &audio_status_proc_ops);
	}

#ifdef CONFIG_SYSCTL
	sched_assist_sysctl_header = register_sysctl_table(sched_assist_root_table);
#endif

	pr_info("oplus_sched_assist: Initialized sched_assist interfaces with %d QoS controllers\n",
		qos_count);
	return 0;
}

static void __exit oplus_sched_assist_exit(void)
{
#ifdef CONFIG_SYSCTL
	if (sched_assist_sysctl_header)
		unregister_sysctl_table(sched_assist_sysctl_header);
#endif

	del_timer_sync(&scene_watchdog);
	cleanup_qos_controllers();

	if (audio_dir) {
		remove_proc_entry("status", audio_dir);
		remove_proc_entry("enable", audio_dir);
		remove_proc_entry("audio", sched_assist_dir);
	}

	if (sched_assist_dir) {
		remove_proc_entry("tpd_id", sched_assist_dir);
		remove_proc_entry("tpd_cmds", sched_assist_dir);
		remove_proc_entry("disable_setting", sched_assist_dir);
		remove_proc_entry("im_flag_app", sched_assist_dir);
		remove_proc_entry("im_flag", sched_assist_dir);
		remove_proc_entry("sched_impt_task", sched_assist_dir);
		remove_proc_entry("ddl_task", sched_assist_dir);
		remove_proc_entry("ux_task_app", sched_assist_dir);
		remove_proc_entry("ux_task", sched_assist_dir);
		remove_proc_entry("sched_assist_scene", sched_assist_dir);
		remove_proc_entry("sched_assist", oplus_sched_dir);
	}

	if (oplus_sched_dir)
		remove_proc_entry("oplus_scheduler", NULL);
}

module_init(oplus_sched_assist_init);
module_exit(oplus_sched_assist_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("RapliVx <raffikun1203@gmail.com>");
MODULE_DESCRIPTION("OPlus Sched Assist and UX Task Acceleration Driver");
MODULE_VERSION("1.0.0");
MODULE_ALIAS("oplus_sched_assist");
