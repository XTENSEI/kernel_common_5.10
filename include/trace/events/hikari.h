/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM hikari

#if !defined(_TRACE_HIKARI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HIKARI_H

#include <linux/tracepoint.h>

/*
 * Emitted from hikari_publish_freq_hint() when Hikari publishes a
 * wake-demand floor hint to cpufreq.  cpu is the target, floor_khz
 * is the hint value, ttl_ms is how long it stays valid, ewma is the
 * smoothed wake-time that triggered the publish.
 */
TRACE_EVENT(hikari_freq_hint,

	TP_PROTO(int cpu, unsigned int floor_khz, unsigned int ttl_ms,
		 unsigned long ewma_ns),

	TP_ARGS(cpu, floor_khz, ttl_ms, ewma_ns),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	floor_khz)
		__field(unsigned int,	ttl_ms)
		__field(unsigned long,	ewma_ns)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->floor_khz	= floor_khz;
		__entry->ttl_ms		= ttl_ms;
		__entry->ewma_ns	= ewma_ns;
	),

	TP_printk("cpu=%d floor=%ukHz ttl=%ums ewma=%luns",
		  __entry->cpu, __entry->floor_khz,
		  __entry->ttl_ms, __entry->ewma_ns)
);

/*
 * Emitted from hikari_select_cpu() when placement steering picks a
 * target CPU for a waking task.  original is the CPU the scheduler
 * would have used; selected is Hikari's override (may be the same).
 * idle indicates whether the selected CPU was idle.
 */
TRACE_EVENT(hikari_placement,

	TP_PROTO(int original, int selected, bool idle, bool is_big),

	TP_ARGS(original, selected, idle, is_big),

	TP_STRUCT__entry(
		__field(int,	original)
		__field(int,	selected)
		__field(bool,	idle)
		__field(bool,	is_big)
	),

	TP_fast_assign(
		__entry->original	= original;
		__entry->selected	= selected;
		__entry->idle		= idle;
		__entry->is_big		= is_big;
	),

	TP_printk("orig=%d sel=%d idle=%d big=%d",
		  __entry->original, __entry->selected,
		  __entry->idle, __entry->is_big)
);

/*
 * Emitted when hikari_apply_profile() changes force-floor tunables.
 */
TRACE_EVENT(hikari_profile,

	TP_PROTO(unsigned int profile, unsigned int floor_big,
		 unsigned int floor_little),

	TP_ARGS(profile, floor_big, floor_little),

	TP_STRUCT__entry(
		__field(unsigned int,	profile)
		__field(unsigned int,	floor_big)
		__field(unsigned int,	floor_little)
	),

	TP_fast_assign(
		__entry->profile	= profile;
		__entry->floor_big	= floor_big;
		__entry->floor_little	= floor_little;
	),

	TP_printk("profile=%u floor_big=%u%% floor_little=%u%%",
		  __entry->profile, __entry->floor_big,
		  __entry->floor_little)
);

#endif /* _TRACE_HIKARI_H */

#include <trace/define_trace.h>
