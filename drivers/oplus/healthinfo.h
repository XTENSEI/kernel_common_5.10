/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 * Adapted for Linux 5.10 GKI by RapliVx
 */

#ifndef _HEALTHINFO_H_
#define _HEALTHINFO_H_

#include <linux/types.h>

int oplus_healthinfo_init(void);
void oplus_healthinfo_exit(void);

#if IS_ENABLED(CONFIG_OPLUS_HEALTHINFO)
int ohm_get_cur_cpuload(void);
bool is_fg(int uid);
#else
static inline int ohm_get_cur_cpuload(void)
{
	return 0;
}
static inline bool is_fg(int uid)
{
	return false;
}
#endif

#endif /* _HEALTHINFO_H_ */
