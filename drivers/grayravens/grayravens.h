/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _GRAYRAVENS_H
#define _GRAYRAVENS_H

#include <linux/kobject.h>

/* Root of /sys/kernel/grayravens, created by gr_perf.c. */
extern struct kobject *gr_kobj;

/*
 * Cap every policy at @pct of its maximum frequency, or release the cap
 * when @pct is 0.  Implemented as a freq_qos maximum request, so it can
 * only ever lower the ceiling and never overrides the governor's own
 * decision below it.
 */
void gr_set_freq_ceiling(unsigned int pct);

#endif /* _GRAYRAVENS_H */
