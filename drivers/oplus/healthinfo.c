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
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/vmstat.h>
#include <linux/bitops.h>
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

#define OHM_LATENCY_DIST_MAX 13

struct sched_stat_common {
	u64 max_ms;
	u64 high_cnt;
	u64 low_cnt;
	u64 total_ms;
	u64 total_cnt;
	u64 latency_dist[OHM_LATENCY_DIST_MAX];
};

struct sched_stat_para {
	bool ctrl;
	bool logon;
	bool trig;
	int low_thresh_ms;
	int high_thresh_ms;
	u64 delta_ms;
	spinlock_t lock;
	struct sched_stat_common all;
	struct sched_stat_common fg;
	struct sched_stat_common ux;
	struct sched_stat_common rt;
	struct sched_stat_common top;
	struct sched_stat_common bg;
	struct sched_stat_common sysbg;
};

struct alloc_wait_para {
	u64 total_alloc_wait_max_order;
	u64 fg_alloc_wait_max_order;
	u64 ux_alloc_wait_max_order;
	struct sched_stat_common total_alloc_wait;
	struct sched_stat_common fg_alloc_wait;
	struct sched_stat_common ux_alloc_wait;
};

static struct sched_stat_para sched_para[OHM_SCHED_TOTAL];
static struct alloc_wait_para allocwait_para;
static DEFINE_SPINLOCK(allocwait_lock);

static int alloc_wait_h_ms = 500;
static int alloc_wait_l_ms = 100;

static u32 para_ctrl_mask = 0x5a000000;
static u32 para_logon_mask = 0x5a000000;
static u32 para_trig_mask = 0x5a000000;

static void init_sched_para(void)
{
	int i;

	for (i = 0; i < OHM_SCHED_TOTAL; i++) {
		sched_para[i].ctrl = true;
		sched_para[i].logon = false;
		sched_para[i].trig = false;
		sched_para[i].low_thresh_ms = 100;
		sched_para[i].high_thresh_ms = 500;
		sched_para[i].delta_ms = 0;
		spin_lock_init(&sched_para[i].lock);
	}
}

static void update_latency_dist(u64 *dist, u64 delta_ms)
{
	int bucket;

	if (delta_ms == 0) {
		bucket = 0;
	} else {
		bucket = fls64(delta_ms);
		if (bucket >= OHM_LATENCY_DIST_MAX)
			bucket = OHM_LATENCY_DIST_MAX - 1;
	}
	dist[bucket]++;
}

void ohm_schedstats_record(int sched_type, struct task_struct *task, u64 delta_ms)
{
	struct sched_stat_para *sp;
	unsigned long flags;
	bool is_fg_task = false;
	bool is_rt_task = false;

	if (sched_type < 0 || sched_type >= OHM_SCHED_TOTAL || !task)
		return;

	sp = &sched_para[sched_type];
	if (!sp->ctrl)
		return;

	is_fg_task = is_fg(task->tgid);
	is_rt_task = rt_task(task);

	spin_lock_irqsave(&sp->lock, flags);
	sp->delta_ms = delta_ms;

	/* Record for 'all' */
	if (delta_ms > sp->all.max_ms)
		sp->all.max_ms = delta_ms;
	sp->all.total_ms += delta_ms;
	sp->all.total_cnt++;
	if (delta_ms >= sp->high_thresh_ms)
		sp->all.high_cnt++;
	else if (delta_ms >= sp->low_thresh_ms)
		sp->all.low_cnt++;
	update_latency_dist(sp->all.latency_dist, delta_ms);

	/* Record for 'fg' */
	if (is_fg_task) {
		if (delta_ms > sp->fg.max_ms)
			sp->fg.max_ms = delta_ms;
		sp->fg.total_ms += delta_ms;
		sp->fg.total_cnt++;
		if (delta_ms >= sp->high_thresh_ms)
			sp->fg.high_cnt++;
		else if (delta_ms >= sp->low_thresh_ms)
			sp->fg.low_cnt++;
		update_latency_dist(sp->fg.latency_dist, delta_ms);
	}

	/* Record for 'rt' */
	if (is_rt_task) {
		if (delta_ms > sp->rt.max_ms)
			sp->rt.max_ms = delta_ms;
		sp->rt.total_ms += delta_ms;
		sp->rt.total_cnt++;
		if (delta_ms >= sp->high_thresh_ms)
			sp->rt.high_cnt++;
		else if (delta_ms >= sp->low_thresh_ms)
			sp->rt.low_cnt++;
		update_latency_dist(sp->rt.latency_dist, delta_ms);
	}

	/* Record for background */
	if (!is_fg_task && !is_rt_task) {
		if (delta_ms > sp->bg.max_ms)
			sp->bg.max_ms = delta_ms;
		sp->bg.total_ms += delta_ms;
		sp->bg.total_cnt++;
		if (delta_ms >= sp->high_thresh_ms)
			sp->bg.high_cnt++;
		else if (delta_ms >= sp->low_thresh_ms)
			sp->bg.low_cnt++;
		update_latency_dist(sp->bg.latency_dist, delta_ms);
	}

	spin_unlock_irqrestore(&sp->lock, flags);
}
EXPORT_SYMBOL_GPL(ohm_schedstats_record);

void memory_alloc_monitor(gfp_t gfp_mask, unsigned int order, u64 wait_ms)
{
	unsigned long flags;
	bool fg;

	fg = is_fg(current->tgid);

	spin_lock_irqsave(&allocwait_lock, flags);
	if (wait_ms > allocwait_para.total_alloc_wait.max_ms) {
		allocwait_para.total_alloc_wait.max_ms = wait_ms;
		allocwait_para.total_alloc_wait_max_order = order;
	}
	allocwait_para.total_alloc_wait.total_ms += wait_ms;
	allocwait_para.total_alloc_wait.total_cnt++;
	if (wait_ms >= alloc_wait_h_ms)
		allocwait_para.total_alloc_wait.high_cnt++;
	else if (wait_ms >= alloc_wait_l_ms)
		allocwait_para.total_alloc_wait.low_cnt++;

	if (fg) {
		if (wait_ms > allocwait_para.fg_alloc_wait.max_ms) {
			allocwait_para.fg_alloc_wait.max_ms = wait_ms;
			allocwait_para.fg_alloc_wait_max_order = order;
		}
		allocwait_para.fg_alloc_wait.total_ms += wait_ms;
		allocwait_para.fg_alloc_wait.total_cnt++;
		if (wait_ms >= alloc_wait_h_ms)
			allocwait_para.fg_alloc_wait.high_cnt++;
		else if (wait_ms >= alloc_wait_l_ms)
			allocwait_para.fg_alloc_wait.low_cnt++;
	}
	spin_unlock_irqrestore(&allocwait_lock, flags);
}
EXPORT_SYMBOL_GPL(memory_alloc_monitor);

struct cpu_load_sample {
	u64 t_user;
	u64 t_system;
	u64 t_idle;
	u64 t_iowait;
	u64 t_irq;
	u64 t_softirq;
};

/* CPU load tracking state */
static DEFINE_MUTEX(cpu_load_lock);
static unsigned long last_load_jiffies;
static int cached_load;
static struct cpu_load_sample prev_sample;
static bool has_prev_sample;

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
	struct cpu_load_sample cur_sample;
	u64 d_user, d_system, d_idle, d_iowait, d_irq, d_softirq;
	u64 load, sum;
	unsigned long now = jiffies;

	mutex_lock(&cpu_load_lock);

	/* Return cached load if polled within 100ms */
	if (has_prev_sample && time_before(now, last_load_jiffies + HZ / 10)) {
		mutex_unlock(&cpu_load_lock);
		return cached_load;
	}

	get_cpu_sample(&cur_sample);

	if (!has_prev_sample) {
		prev_sample = cur_sample;
		last_load_jiffies = now;
		has_prev_sample = true;
		mutex_unlock(&cpu_load_lock);
		return cached_load;
	}

	d_user    = (cur_sample.t_user >= prev_sample.t_user) ? (cur_sample.t_user - prev_sample.t_user) : 0;
	d_system  = (cur_sample.t_system >= prev_sample.t_system) ? (cur_sample.t_system - prev_sample.t_system) : 0;
	d_idle    = (cur_sample.t_idle >= prev_sample.t_idle) ? (cur_sample.t_idle - prev_sample.t_idle) : 0;
	d_iowait  = (cur_sample.t_iowait >= prev_sample.t_iowait) ? (cur_sample.t_iowait - prev_sample.t_iowait) : 0;
	d_irq     = (cur_sample.t_irq >= prev_sample.t_irq) ? (cur_sample.t_irq - prev_sample.t_irq) : 0;
	d_softirq = (cur_sample.t_softirq >= prev_sample.t_softirq) ? (cur_sample.t_softirq - prev_sample.t_softirq) : 0;

	sum = d_user + d_system + d_idle + d_iowait + d_irq + d_softirq;
	load = d_user + d_system + d_iowait + d_irq + d_softirq;

	if (sum > 0) {
		cached_load = (int)(100 * load / sum);
		if (cached_load > 100)
			cached_load = 100;
	}

	prev_sample = cur_sample;
	last_load_jiffies = now;
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
static int get_sched_index_from_module(const char *module)
{
	if (!strcmp(module, "sched"))
		return OHM_SCHED_SCHEDLATENCY;
	if (!strcmp(module, "iowait"))
		return OHM_SCHED_IOWAIT;
	if (!strcmp(module, "dstate"))
		return OHM_SCHED_DSTATE;
	if (!strcmp(module, "fsync"))
		return OHM_SCHED_FSYNC;
	if (!strcmp(module, "emcdrv_iowait"))
		return OHM_SCHED_EMMCIO;
	return -1;
}

static int latency_stat_show(struct seq_file *m, void *v)
{
	const char *module = m->private;
	int idx = get_sched_index_from_module(module);
	struct sched_stat_para p;
	unsigned long flags;

	if (idx >= 0 && idx < OHM_SCHED_TOTAL) {
		spin_lock_irqsave(&sched_para[idx].lock, flags);
		p = sched_para[idx];
		spin_unlock_irqrestore(&sched_para[idx].lock, flags);
	} else {
		memset(&p, 0, sizeof(p));
		p.ctrl = true;
		p.low_thresh_ms = 100;
		p.high_thresh_ms = 500;
	}

	seq_printf(m,
		"%s_ctrl: %s\n"
		"%s_logon: %s\n"
		"%s_trig: %s\n"
		"%s_delta_ms: %llu\n"
		"%s_low_thresh_ms: %d\n"
		"%s_high_thresh_ms: %d\n"
		"%s_max_ms: %llu\n"
		"%s_high_cnt: %llu\n"
		"%s_low_cnt: %llu\n"
		"%s_total_ms: %llu\n"
		"%s_total_cnt: %llu\n"
		"%s_fg_max_ms: %llu\n"
		"%s_fg_high_cnt: %llu\n"
		"%s_fg_low_cnt: %llu\n"
		"%s_fg_total_ms: %llu\n"
		"%s_fg_total_cnt: %llu\n"
		"%s_ux_max_ms: %llu\n"
		"%s_ux_high_cnt: %llu\n"
		"%s_ux_low_cnt: %llu\n"
		"%s_ux_total_ms: %llu\n"
		"%s_ux_total_cnt: %llu\n"
		"%s_rt_max_ms: %llu\n"
		"%s_rt_high_cnt: %llu\n"
		"%s_rt_low_cnt: %llu\n"
		"%s_rt_total_ms: %llu\n"
		"%s_rt_total_cnt: %llu\n"
		"%s_top_app_max_ms: %llu\n"
		"%s_top_app_high_cnt: %llu\n"
		"%s_top_app_low_cnt: %llu\n"
		"%s_top_app_total_ms: %llu\n"
		"%s_top_app_total_cnt: %llu\n"
		"%s_bg_max_ms: %llu\n"
		"%s_bg_high_cnt: %llu\n"
		"%s_bg_low_cnt: %llu\n"
		"%s_bg_total_ms: %llu\n"
		"%s_bg_total_cnt: %llu\n"
		"%s_sys_bg_max_ms: %llu\n"
		"%s_sys_bg_high_cnt: %llu\n"
		"%s_sys_bg_low_cnt: %llu\n"
		"%s_sys_bg_total_ms: %llu\n"
		"%s_sys_bg_total_cnt: %llu\n",
		module, p.ctrl ? "true" : "false",
		module, p.logon ? "true" : "false",
		module, p.trig ? "true" : "false",
		module, p.delta_ms,
		module, p.low_thresh_ms,
		module, p.high_thresh_ms,
		module, p.all.max_ms,
		module, p.all.high_cnt,
		module, p.all.low_cnt,
		module, p.all.total_ms,
		module, p.all.total_cnt,
		module, p.fg.max_ms,
		module, p.fg.high_cnt,
		module, p.fg.low_cnt,
		module, p.fg.total_ms,
		module, p.fg.total_cnt,
		module, p.ux.max_ms,
		module, p.ux.high_cnt,
		module, p.ux.low_cnt,
		module, p.ux.total_ms,
		module, p.ux.total_cnt,
		module, p.rt.max_ms,
		module, p.rt.high_cnt,
		module, p.rt.low_cnt,
		module, p.rt.total_ms,
		module, p.rt.total_cnt,
		module, p.top.max_ms,
		module, p.top.high_cnt,
		module, p.top.low_cnt,
		module, p.top.total_ms,
		module, p.top.total_cnt,
		module, p.bg.max_ms,
		module, p.bg.high_cnt,
		module, p.bg.low_cnt,
		module, p.bg.total_ms,
		module, p.bg.total_cnt,
		module, p.sysbg.max_ms,
		module, p.sysbg.high_cnt,
		module, p.sysbg.low_cnt,
		module, p.sysbg.total_ms,
		module, p.sysbg.total_cnt);
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
	unsigned long flags;
	struct alloc_wait_para snapshot;

	spin_lock_irqsave(&allocwait_lock, flags);
	snapshot = allocwait_para;
	spin_unlock_irqrestore(&allocwait_lock, flags);

	seq_printf(m,
		"total_alloc_wait_h_cnt: %llu\n"
		"total_alloc_wait_l_cnt: %llu\n"
		"ux_alloc_wait_h_cnt: %llu\n"
		"ux_alloc_wait_l_cnt: %llu\n"
		"ux_alloc_wait_max_ms: %llu\n"
		"ux_alloc_wait_max_order: %llu\n"
		"fg_alloc_wait_h_cnt: %llu\n"
		"fg_alloc_wait_l_cnt: %llu\n"
		"total_alloc_wait_max_ms: %llu\n"
		"total_alloc_wait_max_order: %llu\n"
		"fg_alloc_wait_max_ms: %llu\n"
		"fg_alloc_wait_max_order: %llu\n"
		"alloc_wait_ctrl: true\n"
		"alloc_wait_logon: false\n"
		"alloc_wait_trig: false\n",
		snapshot.total_alloc_wait.high_cnt,
		snapshot.total_alloc_wait.low_cnt,
		snapshot.ux_alloc_wait.high_cnt,
		snapshot.ux_alloc_wait.low_cnt,
		snapshot.ux_alloc_wait.max_ms,
		snapshot.ux_alloc_wait_max_order,
		snapshot.fg_alloc_wait.high_cnt,
		snapshot.fg_alloc_wait.low_cnt,
		snapshot.total_alloc_wait.max_ms,
		snapshot.total_alloc_wait_max_order,
		snapshot.fg_alloc_wait.max_ms,
		snapshot.fg_alloc_wait_max_order);
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
	unsigned long flags;
	u64 hung_cnt = 0, panic_cnt = 0;

	spin_lock_irqsave(&sched_para[OHM_SCHED_IOWAIT].lock, flags);
	hung_cnt = sched_para[OHM_SCHED_IOWAIT].all.high_cnt;
	if (sched_para[OHM_SCHED_IOWAIT].all.max_ms >= 30000)
		panic_cnt = 1;
	spin_unlock_irqrestore(&sched_para[OHM_SCHED_IOWAIT].lock, flags);

	seq_printf(m,
		"iowait_hung_cnt: %llu\n"
		"iowait_panic_cnt: %llu\n"
		"ohm_iopanic_mon_ctrl: true\n"
		"ohm_iopanic_mon_logon: false\n"
		"ohm_iopanic_mon_trig: false\n",
		hung_cnt, panic_cnt);
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
static int get_kswapd_loading(void)
{
	int nid;
	int active = 0;

	for_each_online_node(nid) {
		pg_data_t *pgdat = NODE_DATA(nid);

		if (pgdat && pgdat->kswapd) {
			if (READ_ONCE(pgdat->kswapd->state) == TASK_RUNNING)
				active++;
		}
	}
	return active;
}

static int kswapd_loading_show(struct seq_file *m, void *v)
{
	seq_printf(m, "kswapd_loading: %d\n", get_kswapd_loading());
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
static int get_ufs_feature(void)
{
	struct device_node *np;
	int feature = 0;

	np = of_find_node_by_name(NULL, "ufs");
	if (!np) {
		struct device_node *soc = of_find_node_by_path("/soc");

		if (soc) {
			struct device_node *child;

			for_each_child_of_node(soc, child) {
				if (strstr(child->name, "ufs")) {
					np = child;
					break;
				}
			}
			of_node_put(soc);
		}
	}

	if (np) {
		if (of_device_is_available(np)) {
			if (of_property_read_bool(np, "ufshcd-hpb-enable") ||
			    of_property_read_bool(np, "hpb-enable"))
				feature = 2;
			else
				feature = 1;
		}
		of_node_put(np);
	}

	return feature;
}

static int ufs_feature_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", get_ufs_feature());
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
	seq_printf(m,
		"action: false\n"
		"ctrl: 0x%08x\n"
		"logon: 0x%08x\n"
		"trig: 0x%08x\n",
		para_ctrl_mask, para_logon_mask, para_trig_mask);
	return 0;
}

static int para_update_open(struct inode *inode, struct file *file)
{
	return single_open(file, para_update_show, NULL);
}

static ssize_t para_update_write(struct file *filp, const char __user *buff, size_t count, loff_t *off)
{
	char kbuf[64];
	u32 ctrl = 0, logon = 0, trig = 0;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buff, count))
		return -EFAULT;
	kbuf[count] = '\0';

	if (sscanf(kbuf, "%x %x %x", &ctrl, &logon, &trig) >= 1) {
		para_ctrl_mask = ctrl;
		if (logon)
			para_logon_mask = logon;
		if (trig)
			para_trig_mask = trig;
	}
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
	unsigned long flags;
	int h_ms, l_ms;

	spin_lock_irqsave(&sched_para[OHM_SCHED_IOWAIT].lock, flags);
	h_ms = sched_para[OHM_SCHED_IOWAIT].high_thresh_ms;
	l_ms = sched_para[OHM_SCHED_IOWAIT].low_thresh_ms;
	spin_unlock_irqrestore(&sched_para[OHM_SCHED_IOWAIT].lock, flags);

	seq_printf(m, " iowait_thresh:\n h_ms = %dms,l_ms = %dms\n", h_ms, l_ms);
	return 0;
}

static int iowait_thresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, iowait_thresh_show, NULL);
}

static ssize_t iowait_thresh_write(struct file *filp, const char __user *buff, size_t count, loff_t *off)
{
	char kbuf[64];
	int h_ms, l_ms;
	unsigned long flags;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buff, count))
		return -EFAULT;
	kbuf[count] = '\0';

	if (sscanf(kbuf, "%d %d", &h_ms, &l_ms) >= 2) {
		spin_lock_irqsave(&sched_para[OHM_SCHED_IOWAIT].lock, flags);
		sched_para[OHM_SCHED_IOWAIT].high_thresh_ms = h_ms;
		sched_para[OHM_SCHED_IOWAIT].low_thresh_ms = l_ms;
		spin_unlock_irqrestore(&sched_para[OHM_SCHED_IOWAIT].lock, flags);
	}
	return count;
}

static const struct proc_ops proc_iowait_thresh_fops = {
	.proc_open    = iowait_thresh_open,
	.proc_read    = seq_read,
	.proc_write   = iowait_thresh_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/sched_thresh/sched_latency_thresh */
static int sched_latency_thresh_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	int h_ms, l_ms;

	spin_lock_irqsave(&sched_para[OHM_SCHED_SCHEDLATENCY].lock, flags);
	h_ms = sched_para[OHM_SCHED_SCHEDLATENCY].high_thresh_ms;
	l_ms = sched_para[OHM_SCHED_SCHEDLATENCY].low_thresh_ms;
	spin_unlock_irqrestore(&sched_para[OHM_SCHED_SCHEDLATENCY].lock, flags);

	seq_printf(m, " sched_latency_thresh:\n h_ms = %dms,l_ms = %dms\n", h_ms, l_ms);
	return 0;
}

static int sched_latency_thresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, sched_latency_thresh_show, NULL);
}

static ssize_t sched_latency_thresh_write(struct file *filp, const char __user *buff, size_t count, loff_t *off)
{
	char kbuf[64];
	int h_ms, l_ms;
	unsigned long flags;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buff, count))
		return -EFAULT;
	kbuf[count] = '\0';

	if (sscanf(kbuf, "%d %d", &h_ms, &l_ms) >= 2) {
		spin_lock_irqsave(&sched_para[OHM_SCHED_SCHEDLATENCY].lock, flags);
		sched_para[OHM_SCHED_SCHEDLATENCY].high_thresh_ms = h_ms;
		sched_para[OHM_SCHED_SCHEDLATENCY].low_thresh_ms = l_ms;
		spin_unlock_irqrestore(&sched_para[OHM_SCHED_SCHEDLATENCY].lock, flags);
	}
	return count;
}

static const struct proc_ops proc_sched_latency_thresh_fops = {
	.proc_open    = sched_latency_thresh_open,
	.proc_read    = seq_read,
	.proc_write   = sched_latency_thresh_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/latency_dist */
static int latency_dist_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	u64 dist[OHM_LATENCY_DIST_MAX];
	int i;

	spin_lock_irqsave(&sched_para[OHM_SCHED_SCHEDLATENCY].lock, flags);
	memcpy(dist, sched_para[OHM_SCHED_SCHEDLATENCY].all.latency_dist, sizeof(dist));
	spin_unlock_irqrestore(&sched_para[OHM_SCHED_SCHEDLATENCY].lock, flags);

	seq_puts(m, "sched_latency_dist:");
	for (i = 0; i < OHM_LATENCY_DIST_MAX; i++)
		seq_printf(m, " %llu", dist[i]);
	seq_puts(m, "\n");
	return 0;
}

static int latency_dist_open(struct inode *inode, struct file *file)
{
	return single_open(file, latency_dist_show, NULL);
}

static const struct proc_ops proc_latency_dist_fops = {
	.proc_open    = latency_dist_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

/* /proc/oplus_healthinfo/flt_stat */
static int flt_stat_show(struct seq_file *m, void *v)
{
	unsigned long events[NR_VM_EVENT_ITEMS];

	all_vm_events(events);
	seq_printf(m, "pgfault: %lu\npgmajfault: %lu\n",
		   events[PGFAULT], events[PGMAJFAULT]);
	return 0;
}

static int flt_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, flt_stat_show, NULL);
}

static const struct proc_ops proc_flt_stat_fops = {
	.proc_open    = flt_stat_open,
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
	init_sched_para();

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
	proc_create("latency_dist", 0444, healthinfo_dir, &proc_latency_dist_fops);
	proc_create("flt_stat", 0444, healthinfo_dir, &proc_flt_stat_fops);
	proc_create("para_update", 0666, healthinfo_dir, &proc_para_update_fops);

	sched_thresh_dir = proc_mkdir("sched_thresh", healthinfo_dir);
	if (sched_thresh_dir) {
		proc_create("iowait_thresh", 0666, sched_thresh_dir,
			    &proc_iowait_thresh_fops);
		proc_create("sched_latency_thresh", 0666, sched_thresh_dir,
			    &proc_sched_latency_thresh_fops);
	}

	fg_info_dir = proc_mkdir("fg_info", NULL);
	if (fg_info_dir)
		proc_create("fg_uids", 0666, fg_info_dir, &proc_fg_uids_fops);

	/* Initialize baseline CPU load sample */
	get_cpu_sample(&prev_sample);
	last_load_jiffies = jiffies;
	has_prev_sample = true;

	pr_info("healthinfo: OPLUS healthinfo interfaces initialized successfully\n");
	return 0;
}

void __exit oplus_healthinfo_exit(void)
{
	if (fg_info_dir) {
		remove_proc_entry("fg_uids", fg_info_dir);
		remove_proc_entry("fg_info", NULL);
	}

	if (healthinfo_dir) {
		if (sched_thresh_dir) {
			remove_proc_entry("sched_latency_thresh", sched_thresh_dir);
			remove_proc_entry("iowait_thresh", sched_thresh_dir);
			remove_proc_entry("sched_thresh", healthinfo_dir);
		}
		remove_proc_entry("para_update", healthinfo_dir);
		remove_proc_entry("flt_stat", healthinfo_dir);
		remove_proc_entry("latency_dist", healthinfo_dir);
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
MODULE_DESCRIPTION("OPLUS Healthinfo and Resource Monitoring Driver");
MODULE_AUTHOR("RapliVx <raffikun1203@gmail.com>");
