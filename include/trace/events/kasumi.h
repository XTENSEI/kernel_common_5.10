/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM kasumi

#if !defined(_TRACE_KASUMI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_KASUMI_H

#include <linux/tracepoint.h>

/*
 * Emitted from thermal_zone_get_temp() after Kasumi applies its
 * dampening filter.  real_mc is the raw sensor reading; reported_mc
 * is what the thermal framework sees after offset / ramp / ceiling.
 * shape is the active ramp curve (0=linear, 1=quadratic, 2=step).
 */
TRACE_EVENT(kasumi_filter,

	TP_PROTO(const char *zone, int real_mc, int reported_mc,
		 unsigned int offset_mc, unsigned int ramp_mc,
		 unsigned int shape),

	TP_ARGS(zone, real_mc, reported_mc, offset_mc, ramp_mc, shape),

	TP_STRUCT__entry(
		__string(zone,		zone)
		__field(int,		real_mc)
		__field(int,		reported_mc)
		__field(unsigned int,	offset_mc)
		__field(unsigned int,	ramp_mc)
		__field(unsigned int,	shape)
	),

	TP_fast_assign(
		__assign_str(zone,	zone);
		__entry->real_mc	= real_mc;
		__entry->reported_mc	= reported_mc;
		__entry->offset_mc	= offset_mc;
		__entry->ramp_mc	= ramp_mc;
		__entry->shape		= shape;
	),

	TP_printk("zone=%s real=%d reported=%d offset=%u ramp=%u shape=%u",
		  __get_str(zone), __entry->real_mc,
		  __entry->reported_mc, __entry->offset_mc,
		  __entry->ramp_mc, __entry->shape)
);

/*
 * Emitted when kasumi_apply_profile() changes the active tuning.
 * Lets you correlate profile switches with thermal behaviour in
 * trace data.
 */
TRACE_EVENT(kasumi_profile,

	TP_PROTO(unsigned int profile, unsigned int offset_mc,
		 unsigned int ramp_mc, unsigned int shape,
		 unsigned int warmup_secs),

	TP_ARGS(profile, offset_mc, ramp_mc, shape, warmup_secs),

	TP_STRUCT__entry(
		__field(unsigned int,	profile)
		__field(unsigned int,	offset_mc)
		__field(unsigned int,	ramp_mc)
		__field(unsigned int,	shape)
		__field(unsigned int,	warmup_secs)
	),

	TP_fast_assign(
		__entry->profile	= profile;
		__entry->offset_mc	= offset_mc;
		__entry->ramp_mc	= ramp_mc;
		__entry->shape		= shape;
		__entry->warmup_secs	= warmup_secs;
	),

	TP_printk("profile=%u offset=%u ramp=%u shape=%u warmup=%us",
		  __entry->profile, __entry->offset_mc,
		  __entry->ramp_mc, __entry->shape,
		  __entry->warmup_secs)
);

#endif /* _TRACE_KASUMI_H */

#include <trace/define_trace.h>
