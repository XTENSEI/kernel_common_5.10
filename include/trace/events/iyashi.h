/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM iyashi

#if !defined(_TRACE_IYASHI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_IYASHI_H

#include <linux/tracepoint.h>

/*
 * Emitted from iyashi_clamp_target() when the performance floor
 * clamps or passes through.  target_in is the cooling state the
 * thermal framework requested; target_out is what Iyashi returns.
 * clamped is true when Iyashi raised the floor (target_out < target_in).
 * hikari_boosted is true when the Hikari cross-link raised floor_pct.
 */
TRACE_EVENT(iyashi_clamp,

	TP_PROTO(const char *cdev, unsigned long target_in,
		 unsigned long target_out, unsigned int floor_pct,
		 int headroom_c, bool clamped, bool hikari_boosted),

	TP_ARGS(cdev, target_in, target_out, floor_pct, headroom_c,
		clamped, hikari_boosted),

	TP_STRUCT__entry(
		__string(cdev,		cdev)
		__field(unsigned long,	target_in)
		__field(unsigned long,	target_out)
		__field(unsigned int,	floor_pct)
		__field(int,		headroom_c)
		__field(bool,		clamped)
		__field(bool,		hikari_boosted)
	),

	TP_fast_assign(
		__assign_str(cdev,	cdev);
		__entry->target_in	= target_in;
		__entry->target_out	= target_out;
		__entry->floor_pct	= floor_pct;
		__entry->headroom_c	= headroom_c;
		__entry->clamped	= clamped;
		__entry->hikari_boosted	= hikari_boosted;
	),

	TP_printk("cdev=%s in=%lu out=%lu floor=%u%% headroom=%dC clamped=%d hikari=%d",
		  __get_str(cdev), __entry->target_in,
		  __entry->target_out, __entry->floor_pct,
		  __entry->headroom_c, __entry->clamped,
		  __entry->hikari_boosted)
);

/*
 * Emitted when iyashi_apply_profile() changes the active tuning.
 */
TRACE_EVENT(iyashi_profile,

	TP_PROTO(unsigned int profile, unsigned int floor_pct,
		 unsigned int near_limit_c, unsigned int hikari_aware),

	TP_ARGS(profile, floor_pct, near_limit_c, hikari_aware),

	TP_STRUCT__entry(
		__field(unsigned int,	profile)
		__field(unsigned int,	floor_pct)
		__field(unsigned int,	near_limit_c)
		__field(unsigned int,	hikari_aware)
	),

	TP_fast_assign(
		__entry->profile	= profile;
		__entry->floor_pct	= floor_pct;
		__entry->near_limit_c	= near_limit_c;
		__entry->hikari_aware	= hikari_aware;
	),

	TP_printk("profile=%u floor=%u%% near_limit=%uC hikari_aware=%u",
		  __entry->profile, __entry->floor_pct,
		  __entry->near_limit_c, __entry->hikari_aware)
);

#endif /* _TRACE_IYASHI_H */

#include <trace/define_trace.h>
