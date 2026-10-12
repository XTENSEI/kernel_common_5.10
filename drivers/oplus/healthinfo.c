// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 * Adapted for Linux 5.10 GKI by RapliVx
 *
 * Implements the /proc/oplus_healthinfo and /proc/fg_info procfs interfaces
 * utilized by ColorOS Phone Manager, Athena, and PerformanceService daemons
 * (vendor-oplus-hardware-performance-V1-service) for Trinity Engine resource monitoring.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/sched/stat.h>
#include <linux/sched/cputime.h>
#include <linux/sched/topology.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include "healthinfo.h"

/* Procfs directory entries */
static struct proc_dir_entry *healthinfo_dir;
static struct proc_dir_entry *sched_thresh_dir;
static struct proc_dir_entry *fg_info_dir;

/* Foreground UID tracker (/proc/fg_info/fg_uids) */
static int fg_uid_val = -555;

bool is_fg(int uid)
{
	return (uid == fg_uid_val);
}
EXPORT_SYMBOL_GPL(is_fg);

/* CPU load tracking state */
static DEFINE_MUTEX(cpu_load_lock);
static unsigned long last_load_jiffies;
static int cached_load;

struct cpu_load_sample {
	u64 t_user;
	u64 t_system;
	u64 t_idle;
	u64 t_iowait;
	u64 t_irq;
	u64 t_softirq;
};

static u64 get_idle_time_ns(struct kernel_cpustat *kcs, int cpu)
{
	u64 idle, idle_usecs = -1ULL;

	if (cpu_online(cpu))
		idle_usecs = get_cpu_idle_time_us(cpu, NULL);

	if (idle_usecs == -1ULL)
		idle = kcs->cpustat[CPUTIME_IDLE];
	else
		idle = idle_usecs * NSEC_PER_USEC;

	return idle;
}

static u64 get_iowait_time_ns(struct kernel_cpustat *kcs, int cpu)
{
	u64 iowait, iowait_usecs = -1ULL;

	if (cpu_online(cpu))
		iowait_usecs = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_usecs == -1ULL)
		iowait = kcs->cpustat[CPUTIME_IOWAIT];
	else
		iowait = iowait_usecs * NSEC_PER_USEC;

	return iowait;
}

static void get_cpu_sample(struct cpu_load_sample *s)
{
	int cpu;

	memset(s, 0, sizeof(*s));
	for_each_online_cpu(cpu) {
		struct kernel_cpustat kcs;
		u64 *cpustat = kcs.cpustat;

		kcpustat_cpu_fetch(&kcs, cpu);

		s->t_user    += cpustat[CPUTIME_USER] + cpustat[CPUTIME_NICE];
		s->t_system  += cpustat[CPUTIME_SYSTEM];
		s->t_idle    += get_idle_time_ns(&kcs, cpu);
		s->t_iowait  += get_iowait_time_ns(&kcs, cpu);
		s->t_irq     += cpustat[CPUTIME_IRQ];
		s->t_softirq += cpustat[CPUTIME_SOFTIRQ];
	}
}

/**
 * ohm_get_cur_cpuload - Calculates real-time system CPU load percentage (0-100)
 */
int ohm_get_cur_cpuload(void)
{
	struct cpu_load_sample s1, s2;
	u64 d_user, d_system, d_idle, d_iowait, d_irq, d_softirq;
	u64 load, sum;

	mutex_lock(&cpu_load_lock);

	/* Return cached load if polled within 100ms */
	if (last_load_jiffies && time_before(jiffies, last_load_jiffies + HZ / 10)) {
		int ret = cached_load;
		mutex_unlock(&cpu_load_lock);
		return ret;
	}

	get_cpu_sample(&s1);
	msleep(25);
	get_cpu_sample(&s2);

	d_user    = (s2.t_user >= s1.t_user) ? (s2.t_user - s1.t_user) : 0;
	d_system  = (s2.t_system >= s1.t_system) ? (s2.t_system - s1.t_system) : 0;
	d_idle    = (s2.t_idle >= s1.t_idle) ? (s2.t_idle - s1.t_idle) : 0;
	d_iowait  = (s2.t_iowait >= s1.t_iowait) ? (s2.t_iowait - s1.t_iowait) : 0;
	d_irq     = (s2.t_irq >= s1.t_irq) ? (s2.t_irq - s1.t_irq) : 0;
	d_softirq = (s2.t_softirq >= s1.t_softirq) ? (s2.t_softirq - s1.t_softirq) : 0;

	sum = d_user + d_system + d_idle + d_iowait + d_irq + d_softirq;
	load = d_user + d_system + d_iowait + d_irq + d_softirq;

	if (sum == 0)
		cached_load = 0;
	else {
		cached_load = (int)(100 * load / sum);
		if (cached_load > 100)
			cached_load = 100;
	}

	last_load_jiffies = jiffies;
	mutex_unlock(&cpu_load_lock);

	return cached_load;
}
EXPORT_SYMBOL_GPL(ohm_get_cur_cpuload);

/* /proc/oplus_healthinfo/cpu_loading */
static int cpu_load_show(struct seq_file *m, void *v)
{
	int load = ohm_get_cur_cpuload();

	seq_printf(m,
		"cur_cpuloading: %d\n"
		"cur_cpu_ctrl: true\n"
		"cur_cpu_logon: false\n"
		"cur_cpu_trig: false\n",
		load);
	return 0;
}

static int cpu_load_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_load_show, NULL);
}

static const struct proc_ops proc_cpu_load_fops = {
	.proc_open    = cpu_load_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/cpu_info */
static int cpu_info_show(struct seq_file *m, void *v)
{
	unsigned int cpu;
	unsigned long scale_capacity = 0, last_capacity = 0;

	for_each_possible_cpu(cpu) {
		scale_capacity = arch_scale_cpu_capacity(cpu);
		if (scale_capacity == last_capacity)
			continue;
		last_capacity = scale_capacity;
		seq_printf(m, "%u ", cpufreq_quick_get_max(cpu));
	}
	seq_puts(m, "\n");
	return 0;
}

static int cpu_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_info_show, NULL);
}

static const struct proc_ops proc_cpu_info_fops = {
	.proc_open    = cpu_info_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* Latency statistics formatter for /proc/oplus_healthinfo entries */
static int latency_stat_show(struct seq_file *m, void *v)
{
	const char *module = m->private;

	seq_printf(m,
		"%s_ctrl: true\n"
		"%s_logon: false\n"
		"%s_trig: false\n"
		"%s_delta_ms: 0\n"
		"%s_low_thresh_ms: 100\n"
		"%s_high_thresh_ms: 500\n"
		"%s_max_ms: 0\n"
		"%s_high_cnt: 0\n"
		"%s_low_cnt: 0\n"
		"%s_total_ms: 0\n"
		"%s_total_cnt: 0\n"
		"%s_fg_max_ms: 0\n"
		"%s_fg_high_cnt: 0\n"
		"%s_fg_low_cnt: 0\n"
		"%s_fg_total_ms: 0\n"
		"%s_fg_total_cnt: 0\n"
		"%s_ux_max_ms: 0\n"
		"%s_ux_high_cnt: 0\n"
		"%s_ux_low_cnt: 0\n"
		"%s_ux_total_ms: 0\n"
		"%s_ux_total_cnt: 0\n"
		"%s_rt_max_ms: 0\n"
		"%s_rt_high_cnt: 0\n"
		"%s_rt_low_cnt: 0\n"
		"%s_rt_total_ms: 0\n"
		"%s_rt_total_cnt: 0\n"
		"%s_top_app_max_ms: 0\n"
		"%s_top_app_high_cnt: 0\n"
		"%s_top_app_low_cnt: 0\n"
		"%s_top_app_total_ms: 0\n"
		"%s_top_app_total_cnt: 0\n"
		"%s_bg_max_ms: 0\n"
		"%s_bg_high_cnt: 0\n"
		"%s_bg_low_cnt: 0\n"
		"%s_bg_total_ms: 0\n"
		"%s_bg_total_cnt: 0\n"
		"%s_sys_bg_max_ms: 0\n"
		"%s_sys_bg_high_cnt: 0\n"
		"%s_sys_bg_low_cnt: 0\n"
		"%s_sys_bg_total_ms: 0\n"
		"%s_sys_bg_total_cnt: 0\n",
		module, module, module, module, module, module, module, module,
		module, module, module, module, module, module, module, module,
		module, module, module, module, module, module, module, module,
		module, module, module, module, module, module, module, module,
		module, module, module, module, module, module, module, module,
		module);
	return 0;
}

static int latency_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, latency_stat_show, PDE_DATA(inode));
}

static const struct proc_ops proc_latency_stat_fops = {
	.proc_open    = latency_stat_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/alloc_wait */
static int alloc_wait_show(struct seq_file *m, void *v)
{
	seq_puts(m,
		"total_alloc_wait_h_cnt: 0\n"
		"total_alloc_wait_l_cnt: 0\n"
		"ux_alloc_wait_h_cnt: 0\n"
		"ux_alloc_wait_l_cnt: 0\n"
		"ux_alloc_wait_max_ms: 0\n"
		"ux_alloc_wait_max_order: 0\n"
		"fg_alloc_wait_h_cnt: 0\n"
		"fg_alloc_wait_l_cnt: 0\n"
		"total_alloc_wait_max_ms: 0\n"
		"total_alloc_wait_max_order: 0\n"
		"fg_alloc_wait_max_ms: 0\n"
		"fg_alloc_wait_max_order: 0\n"
		"alloc_wait_ctrl: true\n"
		"alloc_wait_logon: false\n"
		"alloc_wait_trig: false\n");
	return 0;
}

static int alloc_wait_open(struct inode *inode, struct file *file)
{
	return single_open(file, alloc_wait_show, NULL);
}

static const struct proc_ops proc_alloc_wait_fops = {
	.proc_open    = alloc_wait_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/iowait_hung */
static int iowait_hung_show(struct seq_file *m, void *v)
{
	seq_puts(m,
		"iowait_hung_cnt: 0\n"
		"iowait_panic_cnt: 0\n"
		"ohm_iopanic_mon_ctrl: true\n"
		"ohm_iopanic_mon_logon: false\n"
		"ohm_iopanic_mon_trig: false\n");
	return 0;
}

static int iowait_hung_open(struct inode *inode, struct file *file)
{
	return single_open(file, iowait_hung_show, NULL);
}

static const struct proc_ops proc_iowait_hung_fops = {
	.proc_open    = iowait_hung_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/kswapd_loading */
static int kswapd_loading_show(struct seq_file *m, void *v)
{
	seq_puts(m, "kswapd_loading: 0\n");
	return 0;
}

static int kswapd_loading_open(struct inode *inode, struct file *file)
{
	return single_open(file, kswapd_loading_show, NULL);
}

static const struct proc_ops proc_kswapd_loading_fops = {
	.proc_open    = kswapd_loading_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/ufs_feature */
static int ufs_feature_show(struct seq_file *m, void *v)
{
	seq_puts(m, "0\n");
	return 0;
}

static int ufs_feature_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_feature_show, NULL);
}

static const struct proc_ops proc_ufs_feature_fops = {
	.proc_open    = ufs_feature_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/para_update */
static int para_update_show(struct seq_file *m, void *v)
{
	seq_puts(m,
		"action: false\n"
		"ctrl: 0x5a000000\n"
		"logon: 0x5a000000\n"
		"trig: 0x5a000000\n");
	return 0;
}

static int para_update_open(struct inode *inode, struct file *file)
{
	return single_open(file, para_update_show, NULL);
}

static ssize_t para_update_write(struct file *filp, const char __user *buff, size_t count, loff_t *off)
{
	return count;
}

static const struct proc_ops proc_para_update_fops = {
	.proc_open    = para_update_open,
	.proc_read    = seq_read,
	.proc_write   = para_update_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/sched_thresh/iowait_thresh */
static int iowait_thresh_show(struct seq_file *m, void *v)
{
	seq_puts(m, " iowait_thresh:\n h_ms = 500ms,l_ms = 100ms\n");
	return 0;
}

static int iowait_thresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, iowait_thresh_show, NULL);
}

static const struct proc_ops proc_iowait_thresh_fops = {
	.proc_open    = iowait_thresh_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/sched_thresh/sched_latency_thresh */
static int sched_latency_thresh_show(struct seq_file *m, void *v)
{
	seq_puts(m, " sched_latency_thresh:\n h_ms = 500ms,l_ms = 100ms\n");
	return 0;
}

static int sched_latency_thresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, sched_latency_thresh_show, NULL);
}

static const struct proc_ops proc_sched_latency_thresh_fops = {
	.proc_open    = sched_latency_thresh_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/fg_info/fg_uids */
static int fg_uids_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fg_uids: %d\n", fg_uid_val);
	return 0;
}

static int fg_uids_open(struct inode *inode, struct file *file)
{
	return single_open(file, fg_uids_show, inode);
}

static ssize_t fg_uids_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	char buffer[32];
	int val, err;

	if (count >= sizeof(buffer))
		return -EINVAL;

	if (copy_from_user(buffer, buff, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 0, &val);
	if (err)
		return err;

	fg_uid_val = val;
	return count;
}

static const struct proc_ops proc_fg_uids_fops = {
	.proc_open    = fg_uids_open,
	.proc_read    = seq_read,
	.proc_write   = fg_uids_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

int __init oplus_healthinfo_init(void)
{
	healthinfo_dir = proc_mkdir("oplus_healthinfo", NULL);
	if (!healthinfo_dir) {
		pr_err("healthinfo: failed to create /proc/oplus_healthinfo\n");
		return -ENOMEM;
	}

	proc_create("cpu_loading", 0444, healthinfo_dir, &proc_cpu_load_fops);
	proc_create("cpu_info", 0444, healthinfo_dir, &proc_cpu_info_fops);
	proc_create_data("sched_latency", 0444, healthinfo_dir, &proc_latency_stat_fops, "sched");
	proc_create_data("iowait", 0444, healthinfo_dir, &proc_latency_stat_fops, "iowait");
	proc_create_data("dstate", 0444, healthinfo_dir, &proc_latency_stat_fops, "dstate");
	proc_create_data("fsync_wait", 0444, healthinfo_dir, &proc_latency_stat_fops, "fsync");
	proc_create_data("emcdrv_iowait", 0444, healthinfo_dir, &proc_latency_stat_fops, "emcdrv_iowait");
	proc_create_data("scm_call", 0444, healthinfo_dir, &proc_latency_stat_fops, "scmcall");
	proc_create_data("ion_wait", 0444, healthinfo_dir, &proc_latency_stat_fops, "total_ion_wait");
	proc_create("alloc_wait", 0444, healthinfo_dir, &proc_alloc_wait_fops);
	proc_create("iowait_hung", 0444, healthinfo_dir, &proc_iowait_hung_fops);
	proc_create("kswapd_loading", 0444, healthinfo_dir, &proc_kswapd_loading_fops);
	proc_create("ufs_feature", 0444, healthinfo_dir, &proc_ufs_feature_fops);
	proc_create("para_update", 0666, healthinfo_dir, &proc_para_update_fops);

	sched_thresh_dir = proc_mkdir("sched_thresh", healthinfo_dir);
	if (sched_thresh_dir) {
		proc_create("iowait_thresh", 0666, sched_thresh_dir, &proc_iowait_thresh_fops);
		proc_create("sched_latency_thresh", 0666, sched_thresh_dir, &proc_sched_latency_thresh_fops);
	}

	fg_info_dir = proc_mkdir("fg_info", NULL);
	if (fg_info_dir) {
		proc_create("fg_uids", 0666, fg_info_dir, &proc_fg_uids_fops);
	}

	pr_info("healthinfo: OPLUS healthinfo interfaces initialized successfully\n");
	return 0;
}

void __exit oplus_healthinfo_exit(void)
{
	if (fg_info_dir) {
		remove_proc_entry("fg_uids", fg_info_dir);
		remove_proc_entry("fg_info", NULL);
	}

	if (sched_thresh_dir) {
		remove_proc_entry("iowait_thresh", sched_thresh_dir);
		remove_proc_entry("sched_latency_thresh", sched_thresh_dir);
		remove_proc_entry("sched_thresh", healthinfo_dir);
	}

	if (healthinfo_dir) {
		remove_proc_entry("para_update", healthinfo_dir);
		remove_proc_entry("ufs_feature", healthinfo_dir);
		remove_proc_entry("kswapd_loading", healthinfo_dir);
		remove_proc_entry("iowait_hung", healthinfo_dir);
		remove_proc_entry("alloc_wait", healthinfo_dir);
		remove_proc_entry("ion_wait", healthinfo_dir);
		remove_proc_entry("scm_call", healthinfo_dir);
		remove_proc_entry("emcdrv_iowait", healthinfo_dir);
		remove_proc_entry("fsync_wait", healthinfo_dir);
		remove_proc_entry("dstate", healthinfo_dir);
		remove_proc_entry("iowait", healthinfo_dir);
		remove_proc_entry("sched_latency", healthinfo_dir);
		remove_proc_entry("cpu_info", healthinfo_dir);
		remove_proc_entry("cpu_loading", healthinfo_dir);
		remove_proc_entry("oplus_healthinfo", NULL);
	}
}

module_init(oplus_healthinfo_init);
module_exit(oplus_healthinfo_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OPLUS Healthinfo and Resource Telemetry Driver for ColorOS Trinity Engine & Performance Framework");
MODULE_AUTHOR("RapliVx <raffikun1203@gmail.com>");
MODULE_VERSION("1.0.0");
MODULE_ALIAS("oplus_healthinfo");
