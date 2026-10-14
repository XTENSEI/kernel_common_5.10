/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 * Adapted for Linux 5.10 GKI by RapliVx
 */

#ifndef _HEALTHINFO_H_
#define _HEALTHINFO_H_

#include <linux/types.h>
#include <linux/gfp.h>
#include <linux/sched.h>

enum {
	OHM_SCHED_IOWAIT = 0,
	OHM_SCHED_SCHEDLATENCY,
	OHM_SCHED_FSYNC,
	OHM_SCHED_EMMCIO,
	OHM_SCHED_DSTATE,
	OHM_SCHED_TOTAL,
};

int oplus_healthinfo_init(void);
void oplus_healthinfo_exit(void);

#if IS_ENABLED(CONFIG_OPLUS_HEALTHINFO)
int ohm_get_cur_cpuload(void);
bool is_fg(int uid);
void ohm_schedstats_record(int sched_type, struct task_struct *task, u64 delta_ms);
void memory_alloc_monitor(gfp_t gfp_mask, unsigned int order, u64 wait_ms);
#else
static inline int ohm_get_cur_cpuload(void)
{
	return 0;
}
static inline bool is_fg(int uid)
{
	return false;
}
static inline void ohm_schedstats_record(int sched_type, struct task_struct *task, u64 delta_ms)
{
}
static inline void memory_alloc_monitor(gfp_t gfp_mask, unsigned int order, u64 wait_ms)
{
}
#endif

#endif /* _HEALTHINFO_H_ */
