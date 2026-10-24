/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _GRAYRAVENS_H
#define _GRAYRAVENS_H

#include <linux/kobject.h>

/* Root of /sys/kernel/grayravens, created by gr_perf.c. */
extern struct kobject *gr_kobj;

#endif /* _GRAYRAVENS_H */
