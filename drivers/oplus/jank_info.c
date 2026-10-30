// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 * Adapted for Linux 5.10 GKI by RapliVx
 *
 * Implements the /proc/jank_info/cpu_jank_info interface required by
 * ColorOS PerformanceService and FrameBoost framework for real-time
 * CPU jank monitoring, Core Load Monitoring (CLM), and task tracking.
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
#include <linux/sched/cputime.h>
#include <linux/cpufreq.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include "healthinfo.h"

static struct proc_dir_entry *jank_dir;
static struct proc_dir_entry *cpu_jank_dir;

/* State variables for CLM and Jank Tracker */
static int task_track_enable_val;
static int clm_enable_val = 1;
static int clm_report_thresh_val = 80;
static int fg_freqs_thresh_val = 1500000;
static int cpu_load32_scale_val = 100;
static int clm_highload_all_val;
static int clm_highload_grp_val;
static int clm_lowload_grp_val;
static int clm_mux_switch_val;
static int bg_dstat_percent_val;
static int cpu_info_sig_val;
static int load_indicator_val;
static char callstack_buf[128];
static DEFINE_SPINLOCK(jank_lock);

/* Tracked task table for task_track */
#define MAX_JANK_TRACK_TASKS 32
struct jank_task_entry {
	pid_t pid;
	char comm[TASK_COMM_LEN];
	u64 last_runtime;
	unsigned long last_jiffies;
	bool active;
};

static struct jank_task_entry tracked_tasks[MAX_JANK_TRACK_TASKS];
static DEFINE_SPINLOCK(task_track_lock);

/* /proc/jank_info/cpu_jank_info/task_track_enable */
static int task_track_enable_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", task_track_enable_val);
	return 0;
}

static int task_track_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, task_track_enable_show, NULL);
}

static ssize_t task_track_enable_write(struct file *filp, const char __user *ubuf,
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

	task_track_enable_val = val;
	return cnt;
}

static const struct proc_ops task_track_enable_proc_ops = {
	.proc_open    = task_track_enable_open,
	.proc_read    = seq_read,
	.proc_write   = task_track_enable_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/jank_info/cpu_jank_info/task_track */
static int task_track_show(struct seq_file *m, void *v)
{
	int i, count = 0;
	unsigned long flags;

	seq_puts(m, "[Tracked Tasks]\n");
	seq_puts(m, "PID\tCOMM\n");

	spin_lock_irqsave(&task_track_lock, flags);
	for (i = 0; i < MAX_JANK_TRACK_TASKS; i++) {
		if (tracked_tasks[i].active) {
			count++;
			seq_printf(m, "%d\t%s\n",
				   tracked_tasks[i].pid,
				   tracked_tasks[i].comm);
		}
	}
	spin_unlock_irqrestore(&task_track_lock, flags);

	seq_printf(m, "Total tracked: %d\n", count);
	return 0;
}

static int task_track_open(struct inode *inode, struct file *file)
{
	return single_open(file, task_track_show, NULL);
}

static ssize_t task_track_write(struct file *filp, const char __user *ubuf,
				size_t cnt, loff_t *ppos)
{
	char kbuf[64];
	char *token, *s;
	pid_t pid = 0;
	int val = 1;
	int i, slot = -1;
	unsigned long flags;
	struct task_struct *task;

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

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();

	if (!task)
		return -ESRCH;

	spin_lock_irqsave(&task_track_lock, flags);
	for (i = 0; i < MAX_JANK_TRACK_TASKS; i++) {
		if (tracked_tasks[i].active && tracked_tasks[i].pid == pid) {
			slot = i;
			break;
		}
		if (!tracked_tasks[i].active && slot == -1)
			slot = i;
	}

	if (val && slot != -1) {
		tracked_tasks[slot].pid = pid;
		strscpy(tracked_tasks[slot].comm, task->comm,
			sizeof(tracked_tasks[slot].comm));
		tracked_tasks[slot].last_jiffies = jiffies;
		tracked_tasks[slot].active = true;
	} else if (!val && slot != -1 && tracked_tasks[slot].active) {
		tracked_tasks[slot].active = false;
	}
	spin_unlock_irqrestore(&task_track_lock, flags);

	put_task_struct(task);
	return cnt;
}

static const struct proc_ops task_track_proc_ops = {
	.proc_open    = task_track_open,
	.proc_read    = seq_read,
	.proc_write   = task_track_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/jank_info/cpu_jank_info/cpu_load */
static int cpu_load_show(struct seq_file *m, void *v)
{
	int load = ohm_get_cur_cpuload();

	seq_printf(m, "%d\n", load);
	return 0;
}

static int cpu_load_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_load_show, NULL);
}

static const struct proc_ops cpu_load_proc_ops = {
	.proc_open    = cpu_load_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/jank_info/cpu_jank_info/cpu_load32 */
static int cpu_load32_show(struct seq_file *m, void *v)
{
	u32 load32 = (u32)ohm_get_cur_cpuload();

	seq_printf(m, "%u\n", load32);
	return 0;
}

static int cpu_load32_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_load32_show, NULL);
}

static const struct proc_ops cpu_load32_proc_ops = {
	.proc_open    = cpu_load32_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* Generic integer proc helpers */
#define DEFINE_JANK_INT_NODE(name, var)					\
static int name##_show(struct seq_file *m, void *v)			\
{									\
	seq_printf(m, "%d\n", var);					\
	return 0;							\
}									\
static int name##_open(struct inode *inode, struct file *file)		\
{									\
	return single_open(file, name##_show, NULL);			\
}									\
static ssize_t name##_write(struct file *filp, const char __user *ubuf,	\
			    size_t cnt, loff_t *ppos)			\
{									\
	char kbuf[16];							\
	int val;							\
	if (cnt >= sizeof(kbuf))					\
		return -EINVAL;						\
	if (copy_from_user(kbuf, ubuf, cnt))				\
		return -EFAULT;						\
	kbuf[cnt] = '\0';						\
	if (kstrtoint(strstrip(kbuf), 0, &val))				\
		return -EINVAL;						\
	var = val;							\
	return cnt;							\
}									\
static const struct proc_ops name##_proc_ops = {			\
	.proc_open    = name##_open,					\
	.proc_read    = seq_read,					\
	.proc_write   = name##_write,					\
	.proc_lseek   = seq_lseek,					\
	.proc_release = single_release,					\
}

DEFINE_JANK_INT_NODE(cpu_load32_scale, cpu_load32_scale_val);
DEFINE_JANK_INT_NODE(clm_enable, clm_enable_val);
DEFINE_JANK_INT_NODE(clm_report_threshold, clm_report_thresh_val);
DEFINE_JANK_INT_NODE(fg_freqs_threshold, fg_freqs_thresh_val);
DEFINE_JANK_INT_NODE(clm_highload_all, clm_highload_all_val);
DEFINE_JANK_INT_NODE(clm_highload_grp, clm_highload_grp_val);
DEFINE_JANK_INT_NODE(clm_lowload_grp, clm_lowload_grp_val);
DEFINE_JANK_INT_NODE(clm_mux_switch, clm_mux_switch_val);
DEFINE_JANK_INT_NODE(bg_dstat_percent, bg_dstat_percent_val);
DEFINE_JANK_INT_NODE(cpu_info_sig, cpu_info_sig_val);
DEFINE_JANK_INT_NODE(load_indicator, load_indicator_val);

/* /proc/jank_info/cpu_jank_info/cpu_info */
static int cpu_info_show(struct seq_file *m, void *v)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		seq_printf(m, "cpu%u: %u kHz\n", cpu, cpufreq_quick_get_max(cpu));
	return 0;
}

static int cpu_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_info_show, NULL);
}

static const struct proc_ops cpu_info_proc_ops = {
	.proc_open    = cpu_info_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/jank_info/cpu_jank_info/version */
static int version_show(struct seq_file *m, void *v)
{
	seq_puts(m, "1.0.0\n");
	return 0;
}

static int version_open(struct inode *inode, struct file *file)
{
	return single_open(file, version_show, NULL);
}

static const struct proc_ops version_proc_ops = {
	.proc_open    = version_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/jank_info/cpu_jank_info/callstack */
static int callstack_show(struct seq_file *m, void *v)
{
	unsigned long flags;

	spin_lock_irqsave(&jank_lock, flags);
	seq_printf(m, "%s\n", callstack_buf);
	spin_unlock_irqrestore(&jank_lock, flags);
	return 0;
}

static int callstack_open(struct inode *inode, struct file *file)
{
	return single_open(file, callstack_show, NULL);
}

static ssize_t callstack_write(struct file *filp, const char __user *ubuf,
			       size_t cnt, loff_t *ppos)
{
	unsigned long flags;
	char tmp[sizeof(callstack_buf)];
	size_t len = min(cnt, sizeof(callstack_buf) - 1);

	/* copy_from_user() may fault, so it must not run under the spinlock */
	if (copy_from_user(tmp, ubuf, len))
		return -EFAULT;
	tmp[len] = '\0';

	spin_lock_irqsave(&jank_lock, flags);
	memcpy(callstack_buf, tmp, len + 1);
	spin_unlock_irqrestore(&jank_lock, flags);

	return cnt;
}

static const struct proc_ops callstack_proc_ops = {
	.proc_open    = callstack_open,
	.proc_read    = seq_read,
	.proc_write   = callstack_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

static int __init oplus_jank_info_init(void)
{
	jank_dir = proc_mkdir("jank_info", NULL);
	if (!jank_dir) {
		pr_err("oplus_jank_info: Failed to create /proc/jank_info\n");
		return -ENOMEM;
	}

	cpu_jank_dir = proc_mkdir("cpu_jank_info", jank_dir);
	if (!cpu_jank_dir) {
		pr_err("oplus_jank_info: Failed to create cpu_jank_info\n");
		remove_proc_entry("jank_info", NULL);
		return -ENOMEM;
	}

	proc_create("task_track_enable", 0666, cpu_jank_dir,
		    &task_track_enable_proc_ops);
	proc_create("task_track", 0666, cpu_jank_dir,
		    &task_track_proc_ops);
	proc_create("cpu_load", 0444, cpu_jank_dir,
		    &cpu_load_proc_ops);
	proc_create("cpu_load32", 0444, cpu_jank_dir,
		    &cpu_load32_proc_ops);
	proc_create("cpu_load32_scale", 0666, cpu_jank_dir,
		    &cpu_load32_scale_proc_ops);
	proc_create("clm_enable", 0666, cpu_jank_dir,
		    &clm_enable_proc_ops);
	proc_create("clm_report_threshold", 0666, cpu_jank_dir,
		    &clm_report_threshold_proc_ops);
	proc_create("fg_freqs_threshold", 0666, cpu_jank_dir,
		    &fg_freqs_threshold_proc_ops);
	proc_create("clm_highload_all", 0666, cpu_jank_dir,
		    &clm_highload_all_proc_ops);
	proc_create("clm_highload_grp", 0666, cpu_jank_dir,
		    &clm_highload_grp_proc_ops);
	proc_create("clm_lowload_grp", 0666, cpu_jank_dir,
		    &clm_lowload_grp_proc_ops);
	proc_create("clm_mux_switch", 0666, cpu_jank_dir,
		    &clm_mux_switch_proc_ops);
	proc_create("bg_dstat_percent", 0666, cpu_jank_dir,
		    &bg_dstat_percent_proc_ops);
	proc_create("cpu_info", 0444, cpu_jank_dir,
		    &cpu_info_proc_ops);
	proc_create("version", 0444, cpu_jank_dir,
		    &version_proc_ops);
	proc_create("callstack", 0666, cpu_jank_dir,
		    &callstack_proc_ops);
	proc_create("cpu_info_sig", 0666, cpu_jank_dir,
		    &cpu_info_sig_proc_ops);
	proc_create("load_indicator", 0666, cpu_jank_dir,
		    &load_indicator_proc_ops);

	pr_info("oplus_jank_info: cpu_jank_info interfaces initialized\n");
	return 0;
}

static void __exit oplus_jank_info_exit(void)
{
	if (cpu_jank_dir) {
		remove_proc_entry("load_indicator", cpu_jank_dir);
		remove_proc_entry("cpu_info_sig", cpu_jank_dir);
		remove_proc_entry("callstack", cpu_jank_dir);
		remove_proc_entry("version", cpu_jank_dir);
		remove_proc_entry("cpu_info", cpu_jank_dir);
		remove_proc_entry("bg_dstat_percent", cpu_jank_dir);
		remove_proc_entry("clm_mux_switch", cpu_jank_dir);
		remove_proc_entry("clm_lowload_grp", cpu_jank_dir);
		remove_proc_entry("clm_highload_grp", cpu_jank_dir);
		remove_proc_entry("clm_highload_all", cpu_jank_dir);
		remove_proc_entry("fg_freqs_threshold", cpu_jank_dir);
		remove_proc_entry("clm_report_threshold", cpu_jank_dir);
		remove_proc_entry("clm_enable", cpu_jank_dir);
		remove_proc_entry("cpu_load32_scale", cpu_jank_dir);
		remove_proc_entry("cpu_load32", cpu_jank_dir);
		remove_proc_entry("cpu_load", cpu_jank_dir);
		remove_proc_entry("task_track", cpu_jank_dir);
		remove_proc_entry("task_track_enable", cpu_jank_dir);
		remove_proc_entry("cpu_jank_info", jank_dir);
	}

	if (jank_dir)
		remove_proc_entry("jank_info", NULL);
}

module_init(oplus_jank_info_init);
module_exit(oplus_jank_info_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RapliVx <raffikun1203@gmail.com>");
MODULE_DESCRIPTION("OPlus CPU Jank Tracking and Core Load Monitoring Driver");
MODULE_VERSION("1.0.0");
MODULE_ALIAS("oplus_jank_info");
