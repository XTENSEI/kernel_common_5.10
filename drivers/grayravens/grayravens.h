/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _GRAYRAVENS_H
#define _GRAYRAVENS_H

#include <linux/kobject.h>
#include <linux/types.h>

/* Root of /sys/kernel/grayravens, created by gr_perf.c. */
extern struct kobject *gr_kobj;

/*
 * Cap every policy at @pct of its maximum frequency, or release the cap
 * when @pct is 0.  Implemented as a freq_qos maximum request, so it can
 * only ever lower the ceiling and never overrides the governor's own
 * decision below it.
 */
void gr_set_freq_ceiling(unsigned int pct);

/* Read a sysfs file into @buf, NUL terminated and stripped. */
int gr_read_file(const char *path, char *buf, size_t size);
/* Read a sysfs file as a single unsigned long. */
int gr_read_file_ulong(const char *path, unsigned long *val);
/* Write @data to a sysfs file. */
int gr_write_file(const char *path, const char *data);

/*
 * True when @name contains any of the comma separated substrings in
 * @patterns, compared without case.  Used to find thermal zones and devfreq
 * nodes, whose names differ per SoC.
 */
bool gr_match_list(const char *name, const char *patterns);

/*
 * Walk a directory, calling @visit for every entry except . and .., with a
 * slice (not NUL terminated) of the entry name.
 */
int gr_scan_dir(const char *path,
		int (*visit)(const char *name, int len, void *arg), void *arg);

#endif /* _GRAYRAVENS_H */
