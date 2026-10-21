/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM cpufreq_zenith

#if !defined(_TRACE_CPUFREQ_ZENITH_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CPUFREQ_ZENITH_H

#include <linux/tracepoint.h>

/* Main decision trace: records the path taken through
 * zenith_get_next_freq() and the freq it resolved. Emitted only when
 * the event is enabled (gated by trace_zenith_decision_enabled()).
 *
 * path strings are compile-time literals owned by the kernel text,
 * so __string / __assign_str stores a cheap per-record copy without
 * dynamic allocation.
 */
TRACE_EVENT(zenith_decision,

	TP_PROTO(int cpu, const char *path, unsigned long util,
		 unsigned long max_cap, unsigned int load_pct,
		 unsigned int freq_in, unsigned int freq_out,
		 unsigned int kcpustat_pct,
		 unsigned long uclamp_min, unsigned long uclamp_max,
		 bool cached_hit),

	TP_ARGS(cpu, path, util, max_cap, load_pct, freq_in, freq_out,
		kcpustat_pct, uclamp_min, uclamp_max, cached_hit),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__string(path,		path)
		__field(unsigned long,	util)
		__field(unsigned long,	max_cap)
		__field(unsigned int,	load_pct)
		__field(unsigned int,	freq_in)
		__field(unsigned int,	freq_out)
		__field(unsigned int,	kcpustat_pct)
		__field(unsigned long,	uclamp_min)
		__field(unsigned long,	uclamp_max)
		__field(bool,		cached_hit)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__assign_str(path,	path);
		__entry->util		= util;
		__entry->max_cap	= max_cap;
		__entry->load_pct	= load_pct;
		__entry->freq_in	= freq_in;
		__entry->freq_out	= freq_out;
		__entry->kcpustat_pct	= kcpustat_pct;
		__entry->uclamp_min	= uclamp_min;
		__entry->uclamp_max	= uclamp_max;
		__entry->cached_hit	= cached_hit;
	),

	TP_printk("cpu=%d path=%s util=%lu max=%lu load=%u%% in=%u out=%u kcpustat=%u%% umin=%lu umax=%lu cached=%d",
		  __entry->cpu, __get_str(path), __entry->util,
		  __entry->max_cap, __entry->load_pct,
		  __entry->freq_in, __entry->freq_out,
		  __entry->kcpustat_pct,
		  __entry->uclamp_min, __entry->uclamp_max,
		  __entry->cached_hit)
);

/* Auto-tune classifier decision emitted once per ZENITH_AUTO_TUNE_PERIOD_MS. */
TRACE_EVENT(zenith_auto_tune,

	TP_PROTO(int cpu, unsigned int sat_pct, unsigned int events_rate_x2,
		 unsigned int prev_profile, unsigned int new_profile),

	TP_ARGS(cpu, sat_pct, events_rate_x2, prev_profile, new_profile),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	sat_pct)
		__field(unsigned int,	events_rate_x2)
		__field(unsigned int,	prev_profile)
		__field(unsigned int,	new_profile)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->sat_pct	= sat_pct;
		__entry->events_rate_x2	= events_rate_x2;
		__entry->prev_profile	= prev_profile;
		__entry->new_profile	= new_profile;
	),

	TP_printk("cpu=%d sat=%u%% events_x2=%u prev=%u new=%u",
		  __entry->cpu, __entry->sat_pct, __entry->events_rate_x2,
		  __entry->prev_profile, __entry->new_profile)
);

/* Scenario-aware auto_tune classifier override.  Emitted from
 * zenith_auto_tune_work() when auto_tune_scenario=1 and a detected
 * scenario (audio / camera / render / mem-stall) overrides the
 * load-saturation + input-rate classifier.  prev_target is what the
 * vanilla classifier would have picked; scenario_target is what the
 * scenario bias substitutes.  When the two are equal, the overlay
 * was a no-op for this window (still emitted for tracing parity).
 */
TRACE_EVENT(zenith_auto_tune_scenario,

	TP_PROTO(int cpu, bool audio, bool camera, bool render,
		 bool memstall, unsigned int prev_target,
		 unsigned int scenario_target),

	TP_ARGS(cpu, audio, camera, render, memstall, prev_target,
		scenario_target),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(bool,		audio)
		__field(bool,		camera)
		__field(bool,		render)
		__field(bool,		memstall)
		__field(unsigned int,	prev_target)
		__field(unsigned int,	scenario_target)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->audio		= audio;
		__entry->camera		= camera;
		__entry->render		= render;
		__entry->memstall	= memstall;
		__entry->prev_target	= prev_target;
		__entry->scenario_target = scenario_target;
	),

	TP_printk("cpu=%d audio=%d camera=%d render=%d memstall=%d prev=%u scenario=%u",
		  __entry->cpu, __entry->audio, __entry->camera,
		  __entry->render, __entry->memstall,
		  __entry->prev_target, __entry->scenario_target)
);

TRACE_EVENT(zenith_auto_tune_v2,

	TP_PROTO(int cpu, unsigned int cluster, unsigned int old_state,
		 unsigned int new_state, unsigned int reason,
		 unsigned int flags, unsigned int target_profile),

	TP_ARGS(cpu, cluster, old_state, new_state, reason, flags,
		target_profile),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	cluster)
		__field(unsigned int,	old_state)
		__field(unsigned int,	new_state)
		__field(unsigned int,	reason)
		__field(unsigned int,	flags)
		__field(unsigned int,	target_profile)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->cluster	= cluster;
		__entry->old_state	= old_state;
		__entry->new_state	= new_state;
		__entry->reason		= reason;
		__entry->flags		= flags;
		__entry->target_profile	= target_profile;
	),

	TP_printk("cpu=%d cluster=%u old=%u new=%u reason=%u flags=0x%x target=%u",
		  __entry->cpu, __entry->cluster, __entry->old_state,
		  __entry->new_state, __entry->reason, __entry->flags,
		  __entry->target_profile)
);

/* Thermal-pressure-aware util derate.  Emitted from zenith_get_util()
 * each time arch_scale_thermal_pressure() reports a meaningful
 * fraction of capacity eaten by SoC thermal throttling and util_out
 * was scaled down accordingly.  before / after are the pre- and
 * post-derate util values; pressure_pct is (pressure * 100 / max).
 */
TRACE_EVENT(zenith_thermal_derate,

	TP_PROTO(int cpu, unsigned long before, unsigned long after,
		 unsigned int pressure_pct),

	TP_ARGS(cpu, before, after, pressure_pct),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned long,	before)
		__field(unsigned long,	after)
		__field(unsigned int,	pressure_pct)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->before		= before;
		__entry->after		= after;
		__entry->pressure_pct	= pressure_pct;
	),

	TP_printk("cpu=%d util_before=%lu util_after=%lu pressure_pct=%u",
		  __entry->cpu, __entry->before, __entry->after,
		  __entry->pressure_pct)
);

/* Predictive-util one-step-ahead extrapolation (predict_util_pct). One
 * record per zenith_get_util() call when the predictor is enabled and
 * the predicted value differs from the observed util. Useful for
 * sanity-checking that the predictor isn't over-reaching during
 * monotonic ramps.
 */
TRACE_EVENT(zenith_predict,

	TP_PROTO(int cpu, unsigned int pct,
		 unsigned long util_obs, unsigned long util_pred),

	TP_ARGS(cpu, pct, util_obs, util_pred),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	pct)
		__field(unsigned long,	util_obs)
		__field(unsigned long,	util_pred)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->pct		= pct;
		__entry->util_obs	= util_obs;
		__entry->util_pred	= util_pred;
	),

	TP_printk("cpu=%d pct=%u util_obs=%lu util_pred=%lu",
		  __entry->cpu, __entry->pct,
		  __entry->util_obs, __entry->util_pred)
);

/* Render-thread / display-pipeline floor activation. Emitted at most
 * once every ZENITH_RENDER_CACHE_TTL_NS per policy when render_aware=1
 * and the comm walk decides whether a render thread is currently
 * running on any of the policy's CPUs.
 */
TRACE_EVENT(zenith_render_floor,

	TP_PROTO(int cpu, bool active, unsigned int floor_pct,
		 unsigned int floor_freq),

	TP_ARGS(cpu, active, floor_pct, floor_freq),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(bool,		active)
		__field(unsigned int,	floor_pct)
		__field(unsigned int,	floor_freq)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->active		= active;
		__entry->floor_pct	= floor_pct;
		__entry->floor_freq	= floor_freq;
	),

	TP_printk("cpu=%d active=%d floor_pct=%u floor_freq=%u",
		  __entry->cpu, __entry->active,
		  __entry->floor_pct, __entry->floor_freq)
);

/* Adaptive frame-budget floor evaluation. Fires once per
 * zenith_get_next_freq() call when frame_budget_us and
 * frame_pace_floor_pct are both non-zero, regardless of whether the
 * floor actually moved the freq.  Useful for sanity-checking that
 * userspace is keeping the budget in sync with the panel's vrefresh.
 */
TRACE_EVENT(zenith_frame_pace,

	TP_PROTO(int cpu, unsigned int budget_us,
		 unsigned int eff_pct, unsigned int floor_freq),

	TP_ARGS(cpu, budget_us, eff_pct, floor_freq),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	budget_us)
		__field(unsigned int,	eff_pct)
		__field(unsigned int,	floor_freq)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->budget_us	= budget_us;
		__entry->eff_pct	= eff_pct;
		__entry->floor_freq	= floor_freq;
	),

	TP_printk("cpu=%d budget_us=%u eff_pct=%u floor_freq=%u",
		  __entry->cpu, __entry->budget_us,
		  __entry->eff_pct, __entry->floor_freq)
);

/* Audio low-jitter floor/cap activation. Emitted at most once every
 * ZENITH_AUDIO_CACHE_TTL_NS per policy when audio_aware=1 and the
 * comm walk decides whether an audio thread is currently running on
 * any of the policy's CPUs. floor_freq / cap_freq are the resolved
 * absolute frequencies (kHz); 0 means "tier disabled".
 */
TRACE_EVENT(zenith_audio_band,

	TP_PROTO(int cpu, bool active, unsigned int floor_pct,
		 unsigned int cap_pct, unsigned int floor_freq,
		 unsigned int cap_freq),

	TP_ARGS(cpu, active, floor_pct, cap_pct, floor_freq, cap_freq),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(bool,		active)
		__field(unsigned int,	floor_pct)
		__field(unsigned int,	cap_pct)
		__field(unsigned int,	floor_freq)
		__field(unsigned int,	cap_freq)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->active		= active;
		__entry->floor_pct	= floor_pct;
		__entry->cap_pct	= cap_pct;
		__entry->floor_freq	= floor_freq;
		__entry->cap_freq	= cap_freq;
	),

	TP_printk("cpu=%d active=%d floor_pct=%u cap_pct=%u floor_freq=%u cap_freq=%u",
		  __entry->cpu, __entry->active,
		  __entry->floor_pct, __entry->cap_pct,
		  __entry->floor_freq, __entry->cap_freq)
);

/* Camera capture-pipeline floor activation. Emitted at most once
 * every ZENITH_CAMERA_CACHE_TTL_NS per policy when camera_aware=1.
 * 'active' is the resolved decision after applying the userspace
 * override (camera_active=auto/force-on/force-off); 'auto_match' is
 * the raw comm-walk result before override.
 */
TRACE_EVENT(zenith_camera_floor,

	TP_PROTO(int cpu, bool active, bool auto_match,
		 unsigned int override, unsigned int floor_pct,
		 unsigned int floor_freq),

	TP_ARGS(cpu, active, auto_match, override, floor_pct, floor_freq),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(bool,		active)
		__field(bool,		auto_match)
		__field(unsigned int,	override)
		__field(unsigned int,	floor_pct)
		__field(unsigned int,	floor_freq)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->active		= active;
		__entry->auto_match	= auto_match;
		__entry->override	= override;
		__entry->floor_pct	= floor_pct;
		__entry->floor_freq	= floor_freq;
	),

	TP_printk("cpu=%d active=%d auto_match=%d override=%u floor_pct=%u floor_freq=%u",
		  __entry->cpu, __entry->active, __entry->auto_match,
		  __entry->override, __entry->floor_pct, __entry->floor_freq)
);

/* game_mode flip. Emitted whenever userspace writes a new value to
 * the game_mode sysfs node; lets you correlate frame-pacing diffs in
 * trace data with the moment the gameswitch helper armed/disarmed.
 */
TRACE_EVENT(zenith_game_mode,

	TP_PROTO(int cpu, bool active),

	TP_ARGS(cpu, active),

	TP_STRUCT__entry(
		__field(int,	cpu)
		__field(bool,	active)
	),

	TP_fast_assign(
		__entry->cpu	= cpu;
		__entry->active	= active;
	),

	TP_printk("cpu=%d active=%d", __entry->cpu, __entry->active)
);

/* Input-boost arming and extension trace.  Emitted from
 * zenith_input_event() at most once per qualifying input event.
 *
 *   active_ms     -- effective full-pin duration the boost was
 *                    armed for, in milliseconds.  Reflects any
 *                    quiet-period extension applied by the
 *                    first-tap-after-quiet logic.
 *   base_ms       -- the base full-pin duration the tunable
 *                    requested, before any extension.  Equal to
 *                    active_ms when no extension fired.
 *   extended      -- true iff the quiet-period extension fired
 *                    on this event (i.e. active_ms > base_ms).
 *   was_quiet     -- true iff the most recent input was older
 *                    than the quiet-period threshold (i.e. this
 *                    event is the first tap after a quiet period).
 *   quiet_gap_ms  -- gap, in milliseconds, between the previous
 *                    input event and this one.  Capped on the
 *                    emit side at U32_MAX ms.
 *   source        -- a small enum describing the input source:
 *                    0 = unknown / generic, 1 = touchscreen,
 *                    2 = key.  Other values reserved for future
 *                    classifiers; consumers should treat unknown
 *                    values as "other".
 *
 * Pure observability event.  Adding / enabling the tracepoint
 * does not change any zenith decision logic.  Default-disabled
 * via the tracepoint subsystem; enable per-CPU debug with
 *   echo 1 > /sys/kernel/debug/tracing/events/cpufreq_zenith/zenith_input_boost/enable
 */
TRACE_EVENT(zenith_input_boost,

	TP_PROTO(unsigned int active_ms, unsigned int base_ms,
		 bool extended, bool was_quiet, unsigned int quiet_gap_ms,
		 unsigned int source),

	TP_ARGS(active_ms, base_ms, extended, was_quiet, quiet_gap_ms, source),

	TP_STRUCT__entry(
		__field(unsigned int,	active_ms)
		__field(unsigned int,	base_ms)
		__field(bool,		extended)
		__field(bool,		was_quiet)
		__field(unsigned int,	quiet_gap_ms)
		__field(unsigned int,	source)
	),

	TP_fast_assign(
		__entry->active_ms	= active_ms;
		__entry->base_ms	= base_ms;
		__entry->extended	= extended;
		__entry->was_quiet	= was_quiet;
		__entry->quiet_gap_ms	= quiet_gap_ms;
		__entry->source		= source;
	),

	TP_printk("active_ms=%u base_ms=%u extended=%d was_quiet=%d gap_ms=%u source=%u",
		  __entry->active_ms, __entry->base_ms,
		  __entry->extended, __entry->was_quiet,
		  __entry->quiet_gap_ms, __entry->source)
);

#endif /* _TRACE_CPUFREQ_ZENITH_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
