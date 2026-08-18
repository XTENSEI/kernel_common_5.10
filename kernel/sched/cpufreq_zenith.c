// SPDX-License-Identifier: GPL-2.0
/*
 * Zenith CPUFreq Governor (Zenithed-V4)
 * Originally developed by ENI exclusively for LO.
 * V3+ continued by XTENSEI.
 *
 * Hybrid governor with Energy Model awareness, display-state coupling,
 * thermal-aware throttling, and a self-calibrating multi-layer
 * auto-tune stack:
 *
 *   - V1 classifier      load + input rate; per-profile target every
 *                        ZENITH_AUTO_TUNE_PERIOD_MS
 *   - V2 state machine   per-cluster {efficiency, balanced, latency,
 *                        sustained_perf, thermal_recovery}, scenario
 *                        overlay (camera/render/audio/memstall),
 *                        cluster-aware capping, hysteresis + cooldown
 *   - V3 self-tuner      observes V2 transition rate and bumps
 *                        hysteresis/cooldown offsets to fit live load
 *
 * On top of the auto-tune stack:
 *
 *   - Glides (round/U/Z) frequency-shaping helpers
 *   - K1 migration_floor sticky cluster-arrival floor
 *   - K2 psi_cpu_floor   PSI-CPU-stall floor
 *   - K3 frame_overrun   vblank-driven rescue (drm_handle_vblank
 *                        producer hook in drivers/gpu/drm/drm_vblank.c)
 *   - M1 psi_mem_cap     memory-pressure cap
 *   - M2 uclamp respect  peer_ramp / migration_floor uclamp_min sub-gates
 *   - M3 peer_ramp_off   screen-off peer_ramp window
 *   - M5 K3 deep tier    deep-streak frame_overrun amplification
 *
 * Producer/consumer split:
 *
 *   - Producers: input_handler, drm_handle_vblank, screen_state,
 *     thermal_state, PSI sampler, kcpustat, comm-walk
 *   - Consumers: zenith_get_next_freq() (per-tick) and
 *     zenith_auto_tune_work() (per-classifier-window)
 *
 * Telemetry: 12 trace events under include/trace/events/cpufreq_zenith.h
 * plus the auto_tune_status, at_log, last_decision_path, profile_values,
 * zenith_stats, zenith_input_stats, auto_tune_v3_state, game_auto_state
 * RO sysfs attrs for live diagnosis.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"

#include <linux/cpufreq.h>
#include <linux/cpufreq_zenith.h>
#include <linux/sched/cpufreq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/cgroup.h>
#include <linux/energy_model.h>
#include <linux/input.h>
#include <linux/perf_event.h>
#include <linux/jump_label.h>
#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/sysfs.h>
#include <linux/hikari.h>
#include <linux/selene.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/time.h>
#include <linux/kernel_stat.h>
#include <linux/thermal.h>

/* linux/fb.h transitively pulls linux/acpi.h, which redefines the
 * ACPI_PROBE_TABLE macro already defined by sched.h ->
 * asm-generic/vmlinux.lds.h. The linker-section variant from
 * vmlinux.lds.h is only meaningful in the kernel link stage and
 * unused in this translation unit, so undefining it before the
 * fb.h chain lets the acpi.h definition win without a warning.
 * Guarding the include on CONFIG_FB_NOTIFY also lets panels that
 * use a non-fb notifier framework skip the dependency entirely.
 */
#ifdef CONFIG_FB_NOTIFY
#undef ACPI_PROBE_TABLE
#undef ACPI_PROBE_TABLE_END
#include <linux/fb.h>
#endif
/*
 * Optional drm_panel_notifier path.
 *
 * The Common Android Kernel 5.10 GKI tree does not ship a drm panel
 * notifier; it is a vendor-only mechanism (Qualcomm's
 * drm/drm_panel_notifier.h, MediaTek's panel_event_notifier, etc.).
 * Vendor builds that backport such a notifier framework can opt in
 * by defining CONFIG_DRM_PANEL_NOTIFY=y and providing a header at
 * <drm/drm_panel_notifier.h> that declares:
 *
 *     int  drm_panel_notifier_register(struct notifier_block *nb);
 *     int  drm_panel_notifier_unregister(struct notifier_block *nb);
 *     enum { DRM_PANEL_EVENT_BLANK = ..., };
 *     enum { DRM_PANEL_BLANK_UNBLANK = 0, DRM_PANEL_BLANK_POWERDOWN = ..., };
 *     struct drm_panel_notifier { void *data; };  (data points to int *blank)
 *
 * On stock GKI builds the config is undefined and this whole block
 * compiles out, leaving the legacy CONFIG_FB_NOTIFY path unchanged.
 */
#ifdef CONFIG_DRM_PANEL_NOTIFY
#include <drm/drm_panel_notifier.h>
#endif
#include <trace/events/power.h>
#include <trace/events/sched.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_zenith.h>
#undef CREATE_TRACE_POINTS

/* Vendor-hook headers (B9-1 topology, B9-2 sched, B9-3 cpuidle, B9-3+
 * power).  Pulled in *after* CREATE_TRACE_POINTS has been undef'd
 * above, so each header only DECLAREs its android_vh_* /
 * android_rvh_* tracepoints (the #include <trace/define_trace.h> at
 * the bottom of every trace/hooks header is a no-op when
 * CREATE_TRACE_POINTS is not defined).  The canonical owner of every
 * __traceiter_android_* / __tracepoint_android_* symbol referenced
 * below is drivers/android/vendor_hooks.o, which sets
 * CREATE_TRACE_POINTS itself before pulling these same headers in
 * (drivers/android/vendor_hooks.c).  Defining them here would emit a
 * second copy of those symbols and the link would fail with duplicate
 * definitions in kernel/built-in.a vs drivers/built-in.a.
 *
 * All four headers are always pulled in regardless of
 * CONFIG_ANDROID_VENDOR_HOOKS: when the vendor-hook infrastructure is
 * compiled out, the trace_android_vh_* / register_trace_android_vh_*
 * macros collapse to empty / -ENODEV, and the per-tunable enable gates
 * (vh_arch_freq_scale_enable, vh_uclamp_observer_enable,
 * vh_cpu_idle_enable, vh_freq_qos_enable, vh_sched_move_task_enable,
 * vh_scheduler_tick_enable)
 * become runtime no-ops.
 *
 * Mirror of the pattern used by kernel/sched/core.c, which also defines
 * its own trace events via CREATE_TRACE_POINTS for <trace/events/sched.h>
 * and then includes <trace/hooks/sched.h> + <trace/hooks/dtask.h> as
 * declare-only consumers.
 */
#include <trace/hooks/topology.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/cpuidle.h>
#include <trace/hooks/power.h>

/* Constants & Defaults */
/* Permille of SCHED_CAPACITY_SCALE at which iowait boost starts.
 * 100 == SCHED_CAPACITY_SCALE, i.e. the boost saturates at full
 * capacity instead of overshooting past it.  The historical 125
 * let sustained iowait (app installs, syncs) push util past
 * capacity and pin high freq for the whole episode; capping at
 * 100 keeps the boost strong but bounds the battery cost.
 */
#define ZENITH_DEFAULT_IOWAIT_BOOST_MIN		100
#define ZENITH_DEFAULT_IOWAIT_STACK_PCT		50	/* 0 = legacy max(util, boost) */

/* iowait_backoff_after_ms (default 0, off):
 *
 * The doubling-on-each-iowait-flag climb in zenith_iowait_boost()
 * has no upper bound other than SCHED_CAPACITY_SCALE.  On long
 * sustained-iowait workloads (level loads, app installs, big-file
 * syncs) the boost saturates near max for the duration, with
 * diminishing return: by the time the boost has doubled past the
 * point where extra freq materially reduces I/O wait, the cpu is
 * burning power on cycles that gain almost nothing.
 *
 * When set, this tunable starts shrinking the boost stack once a
 * single iowait episode has been live for N milliseconds.  The
 * doubling step in zenith_iowait_boost() flips to a halving step
 * once the timer elapses, so the boost decays toward the floor at
 * the same rate the apply path would decay it during quiet ticks.
 * iowait flag observations no longer keep extending the boost; the
 * episode dies on its own and the next fresh iowait re-arms from
 * the floor with a clean timer.
 *
 * 0 disables the backoff entirely (legacy behaviour: doubling
 * climbs to SCHED_CAPACITY_SCALE without an upper time bound).
 */
#define ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS	0
#define ZENITH_DEFAULT_UP_THRESHOLD		80
#define ZENITH_DEFAULT_UP_THRESHOLD_HISPEED	0	/* disabled */
#define ZENITH_DEFAULT_DOWN_THRESHOLD		55
#define ZENITH_DEFAULT_HISPEED_FREQ		0	/* disabled */
/* Defensive sysfs upper bound for hispeed_freq (kHz).  50 GHz is well past
 * any real CPU; rejects UINT_MAX-style garbage at the sysfs layer.  Values
 * below this still receive the existing consumer-side comparison against
 * policy->cur / policy->max.
 */
#define ZENITH_HISPEED_FREQ_MAX			50000000U
#define ZENITH_DEFAULT_HISPEED_FREQ_PCT		45	/* fallback when hispeed_freq=0 */
#define ZENITH_DEFAULT_HISPEED_LOAD		75
#define ZENITH_DEFAULT_HISPEED_HYST_PCT		10	/* exit hysteresis margin */

/* hispeed_entry_streak (default 0, off):
 *
 * Symmetric entry-side hysteresis for the hispeed tier.  Today the
 * tier has only exit hysteresis (hispeed_hyst_pct): once load_pct
 * crosses hispeed_load it activates immediately, leaving the tier
 * vulnerable to single-sample noise spikes near the boundary.
 *
 * When set to N (>0), require load_pct >= hispeed_load to hold for
 * N+1 consecutive samples before flipping hispeed_active to true.
 * 0 preserves the historical immediate-flip behaviour.  Capped to
 * ZENITH_HISPEED_ENTRY_STREAK_MAX so the per-policy u8 counter
 * never overflows.
 */
#define ZENITH_DEFAULT_HISPEED_ENTRY_STREAK	2
#define ZENITH_HISPEED_ENTRY_STREAK_MAX		16

/* brutal_entry_streak (default 0, off):
 *
 * Symmetric to hispeed_entry_streak, but for the brutality
 * snap-to-max tier.  Today the brutality path flips
 * brutal_active=true and pins policy->max on the very first
 * sample where load_pct >= up_threshold, leaving the tier
 * vulnerable to single-sample spikes (scheduler wake-up
 * bursts, sampler quantisation noise) that pin the cluster
 * to max for the whole down_threshold hysteresis window.
 *
 * When set to N (>0), require load_pct >= up_threshold to
 * hold for N+1 consecutive samples before flipping
 * brutal_active to true.  0 preserves the historical
 * immediate-flip behaviour.  Only gates SNAP mode; STEP
 * climb mode is already gentle by design and is unaffected.
 * Exit hysteresis via down_threshold / brutal_active hold is
 * unchanged.  Capped to ZENITH_BRUTAL_ENTRY_STREAK_MAX so
 * the per-policy u8 counter never overflows.
 */
#define ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK	0
#define ZENITH_BRUTAL_ENTRY_STREAK_MAX		16

/* peak_headroom_rescue (default 1, on):
 *
 * Watchdog tier that rescues the cluster from sustained-high-util
 * starvation.  When the load-based + hispeed pipeline leaves the
 * cluster well below policy->max even though load_pct is pegged
 * (auto-tune profile cap, calibration drift, accumulated down-rate
 * progress, or efficient_freq ladder gating), this tier forces a
 * one-shot up-shift toward policy->max.
 *
 * Trigger condition (both must hold for STARVE_STREAK + 1
 * consecutive samples):
 *
 *   load_pct >= ZENITH_PEAK_HEADROOM_STARVE_LOAD_PCT (default 90)
 *   freq < (policy->max * ZENITH_PEAK_HEADROOM_FREQ_FLOOR_PCT / 100)
 *     (default 85, i.e. cluster freq is below 85%% of policy->max)
 *
 * Rescue action: bump freq up to (policy->max *
 * ZENITH_PEAK_HEADROOM_JUMP_PCT / 100) (default 100, i.e. pin to
 * policy->max) and arm a hold-down deadline so a second rescue
 * cannot fire within ZENITH_PEAK_HEADROOM_HOLD_MS (default 50 ms).
 *
 * Bounded by:
 *   - The streak counter is u8 and saturates at
 *     ZENITH_PEAK_HEADROOM_STREAK_MAX so it cannot wrap on long
 *     sustained runs.
 *   - The downstream caps (uclamp_max, light_cap, audio_cap,
 *     em_cap, PSI cap) all apply after the rescue, so userspace
 *     power hints and the EM validator stay authoritative.
 *   - The rescue is skipped when pin_to_target is already true
 *     (input_boost full-pin / brutality snap_max / brutal_hold);
 *     those paths have already pinned the cluster high.
 *
 * 0 disables the watchdog entirely (legacy behaviour: nothing
 * lifts the cluster off the load-based + hispeed pipeline output).
 */
#define ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE		1
#define ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_LOAD_PCT	90
#define ZENITH_DEFAULT_PEAK_HEADROOM_FREQ_FLOOR_PCT	85
#define ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_STREAK	2
#define ZENITH_DEFAULT_PEAK_HEADROOM_JUMP_PCT		100
#define ZENITH_DEFAULT_PEAK_HEADROOM_HOLD_MS		50
#define ZENITH_DEFAULT_PEAK_HEADROOM_PREARM		1
#define ZENITH_PEAK_HEADROOM_STREAK_MAX			16
#define ZENITH_PEAK_HEADROOM_HOLD_MS_MAX		1000

/* Patch 1.3 cluster-wake-pulse: when zenith_get_next_freq() is
 * entered after a >= cluster_wake_pulse_idle_ms gap (the cluster
 * was deeply idle), arm a soft floor at cluster_wake_pulse_floor_-
 * pct of policy->max for cluster_wake_pulse_ms milliseconds.
 *
 * The floor absorbs the PELT warm-up cost: the first util sample
 * after a long idle is by construction near zero and would normally
 * pin freq at min, even when the workload that just woke needs the
 * cluster (the EAS resolve catches up only on the second/third
 * sample, by which point a frame deadline can already be at risk).
 *
 * Defaults: cluster_wake_pulse_ms = 40 (one-frame budget on a
 * 60 Hz panel), cluster_wake_pulse_idle_ms = 80 (only fire after
 * a "real" deep idle, not just a burst of two consecutive idle
 * sample windows), cluster_wake_pulse_floor_pct = 55 (just above
 * a typical hispeed-entry band).
 *
 * Disabled by setting cluster_wake_pulse_ms = 0; profile-baked so
 * BATTERY and LEGACY hide the tier entirely while PERFORMANCE
 * widens it.
 */
#define ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS		30
#define ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_IDLE_MS	80
#define ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_FLOOR_PCT	55
#define ZENITH_CLUSTER_WAKE_PULSE_MS_MAX		200
#define ZENITH_CLUSTER_WAKE_PULSE_IDLE_MS_MAX		1000

/* Patch 1.2 batt_hold_scale_pct: percentage scale applied to
 * peak-rescue / peak-prearm hold-down millisecond budgets when
 * the system is running on battery.  Defaults to 100 (identity,
 * preserves pre-1.2 behaviour) so a no-op for users on AC, lab
 * boards, or hardware without a power_supply driver registered.
 * Bounded to 50..300 (50 -> halve the hold, 300 -> triple it).
 * Profile-baked via zenith_apply_profile() so users on PERFORMANCE
 * never have hold extended on battery, BALANCED gets a 1.2x scale,
 * BATTERY gets 1.8x to keep cores at floor longer when discharging,
 * LEGACY stays at 100 (identity, pre-V4 hybrid behaviour).
 */
#define ZENITH_DEFAULT_BATT_HOLD_SCALE_PCT		100
#define ZENITH_BATT_HOLD_SCALE_PCT_MIN			50
#define ZENITH_BATT_HOLD_SCALE_PCT_MAX			300

/* Wave A charger-aware floor.  Companion to the existing
 * batt_hold_scale_pct / on_battery infrastructure: when
 * charger_aware == 1 AND the lazy AC-vs-battery cache reports the
 * system is on AC (zenith_on_battery == 0), a freq floor of
 * (policy->max * charger_floor_pct / 100) is applied alongside the
 * existing audio / render / migration floors.  Both knobs default 0
 * (off) so a fresh boot is bit-identical to pre-Wave-A behaviour;
 * users opt in by writing 1 + floor_pct via sysfs.
 *
 * Rationale: while the device is plugged in, the energy cost of
 * holding hispeed approaches zero (the charger is feeding both the
 * battery and the SoC), so a configurable floor delivers extra
 * responsiveness with no on-battery cost.  Thermal still wins above
 * charger_floor_pct because the thermal_state / auto_thermal_cap
 * chain runs after the floor and can walk it back down if the SoC
 * overheats.  audio_floor / render_floor / migration_floor all run
 * before this site, so the charger floor only applies when none of
 * the situational floors have already raised freq above
 * charger_floor_pct.
 */
#define ZENITH_DEFAULT_CHARGER_AWARE			0
#define ZENITH_DEFAULT_CHARGER_FLOOR_PCT		0
#define ZENITH_CHARGER_FLOOR_PCT_MAX			100

/* Wave A cgroup-aware top-app floor.  Replaces the fragile comm-walk
 * heuristic for foreground detection (game_auto / render_aware /
 * audio_aware all match by thread name) with a rock-solid cgroup
 * read.  Android's ActivityManager assigns every UI-visible app's
 * threads to the cpuset cgroup directory "top-app"; system /
 * background threads land in "foreground", "background", or
 * "system-background".  Reading current->cgroups->subsys
 * [cpuset_cgrp_id]->cgroup->kn->name == "top-app" answers "is the
 * user looking at this task right now" without depending on which
 * APK happened to be installed or how its threads are named.
 *
 * When top_app_aware == 1 AND any CPU in the policy is currently
 * running a task in the top-app cpuset cgroup, apply a freq floor
 * of (policy->max * top_app_floor_pct / 100) alongside the existing
 * audio / render / camera / charger floors.  Cached for
 * ZENITH_TOP_APP_CACHE_TTL_NS to keep the hot path cheap (one
 * task_css() + strcmp() per cached interval, not per tick).
 *
 * Both knobs default 0 so the tier is opt-in and a fresh boot is
 * bit-identical to pre-Wave-A behaviour.  Requires CONFIG_CPUSETS=y
 * in the kernel (mandatory on Android GKI; on CONFIG_CPUSETS=n the
 * helper always returns false and the floor never applies).
 *
 * The cgroup name is fixed at "top-app" because every Android
 * release since Lollipop has used that exact directory name; if
 * a vendor renames it, set top_app_aware=0 and use the comm-walk
 * floors instead.
 */
#define ZENITH_DEFAULT_TOP_APP_AWARE			1
#define ZENITH_DEFAULT_TOP_APP_FLOOR_PCT		50
#define ZENITH_TOP_APP_FLOOR_PCT_MAX			100
#define ZENITH_TOP_APP_CACHE_TTL_NS			(4 * NSEC_PER_MSEC)
#define ZENITH_TOP_APP_CGROUP_NAME			"top-app"

/* Wave A render-thread util tracker.  More selective sibling of the
 * existing render_floor: the comm-walk-based render_floor fires
 * whenever a known render thread is observed on the policy, even
 * if the thread is sitting idle in its main loop (paused video,
 * static UI).  This tracker re-uses the same comm walk but adds a
 * second gate -- the matched task's PELT se.avg.util_avg must be
 * >= render_thread_util_thresh -- before applying a separate,
 * higher floor (render_thread_util_floor_pct).
 *
 * Threshold is in 1/SCHED_CAPACITY_SCALE units (0..1024) matching
 * util_avg's scale.  Typical values:
 *   256  =  25% of a CPU's capacity, low filter -- catches any
 *           render thread above background idle.
 *   512  =  50% capacity, medium filter -- catches active
 *           rendering (60 Hz scroll, video playback).
 *   768  =  75% capacity, strict filter -- catches heavy GPU
 *           workloads (gaming, complex animations) only.
 *
 * Reuses the cached zenith_policy_has_render() walk; the helper
 * also stores the matched task's util_avg in
 * z_policy->render_matched_util_avg, so this tier adds at most a
 * single load after the cache hit.  All three knobs default 0 so
 * a fresh boot is bit-identical to pre-Wave-A behaviour and the
 * tier is opt-in.  Requires render_aware=1 in addition to
 * render_thread_util_aware=1, because the comm-walk must run for
 * util_avg to be observed.
 */
#define ZENITH_DEFAULT_RENDER_THREAD_UTIL_AWARE		0
#define ZENITH_DEFAULT_RENDER_THREAD_UTIL_THRESH	0
#define ZENITH_DEFAULT_RENDER_THREAD_UTIL_FLOOR_PCT	0
#define ZENITH_RENDER_THREAD_UTIL_THRESH_MAX		1024
#define ZENITH_RENDER_THREAD_UTIL_FLOOR_PCT_MAX		100

/* Wave B PMU IPC tracker.  Per-CPU hardware perf_event counters
 * (instructions retired / CPU cycles) sampled once per
 * zenith_auto_tune_work() pass.  IPC == instructions / cycles is the
 * canonical efficiency signal: high IPC means the workload is
 * compute-bound and benefits from extra freq headroom; low IPC means
 * the workload is memory-bound or stalled and extra freq mostly burns
 * energy without helping throughput.  This tier raises a freq floor
 * when measured IPC crosses pmu_ipc_thresh, so user-tunable workloads
 * that want "freq when it actually pays" can opt in.
 *
 * IPC is reported as percent (100 = 1.0 IPC).  Modern Cortex-A series
 * cores typically run 0.5..2.0 IPC under common Android workloads, so
 * a default thresh of 100 (1.0 IPC) catches "actually using the CPU"
 * without firing on memory-bound stalls.  Cap at 1000 (10.0 IPC)
 * which is well above any real-world value -- the cap exists only to
 * keep the percent representation in unsigned int range.
 *
 * Gated on CONFIG_PERF_EVENTS at build time.  When the kernel is
 * built without perf_events, all helpers compile to no-ops, the
 * tunables remain visible but their effect is constant zero, and
 * the floor never applies.  Per-CPU events are allocated lazily in
 * zenith_start() and torn down in zenith_stop(); allocation
 * failures (PMU not exposed by the SoC, perf locked down) are
 * silently tolerated and the floor never applies on those CPUs.
 */
#define ZENITH_DEFAULT_PMU_AWARE			0
#define ZENITH_DEFAULT_PMU_IPC_THRESH			100
#define ZENITH_DEFAULT_PMU_IPC_FLOOR_PCT		0
#define ZENITH_PMU_IPC_THRESH_MAX			1000
#define ZENITH_PMU_IPC_FLOOR_PCT_MAX			100

/* Wave B EAS / Energy Model integration.  Reads the per-policy
 * struct em_perf_domain (registered by the cpufreq driver via
 * em_dev_register_perf_domain()) to locate the energy-knee OPP --
 * the performance state with the lowest 'cost' field, where cost ==
 * power * max_freq / freq is pre-computed by EM at registration
 * time.  Below the knee, voltage scaling makes the OPP
 * inefficient in joules-per-instruction; above the knee, voltage
 * scales linearly with freq while capacity scales sub-linearly,
 * so cost rises again.  The knee is therefore the most
 * energy-efficient sustained operating point.
 *
 * em_floor_pct applies a freq floor at (em_knee_freq *
 * em_floor_pct / 100) so the policy never undershoots the
 * energy-knee for a sustained workload.  Capped at 200% to allow
 * the user to express "a bit above the knee for safety margin".
 *
 * Both knobs default 0 so the tier is opt-in and a fresh boot is
 * bit-identical to pre-Wave-B behaviour.  Gated on
 * CONFIG_ENERGY_MODEL at build time; on CONFIG_ENERGY_MODEL=n the
 * helper returns a constant zero (em_cpu_get() returns NULL
 * unconditionally) and the floor never applies.
 *
 * Caches the knee freq per-policy at zenith_start() to avoid the
 * em->table walk on every cpufreq tick.  When the cpufreq driver
 * registers an EM after zenith_start() has run (rare; most boards
 * register the EM early during cpufreq driver probe), the cache
 * is refreshed lazily on the first floor application.
 */
#define ZENITH_DEFAULT_EM_AWARE				0
#define ZENITH_DEFAULT_EM_FLOOR_PCT			0
#define ZENITH_EM_FLOOR_PCT_MAX				200

/* Patch 1.10 quiet-hours cap.  Two start / end knobs (in minutes
 * since 00:00 UTC, range 0..1439) define a daily window; while
 * inside that window, freq is capped at quiet_hours_cap_pct of
 * policy->max.  When start == end, the window is zero-length and
 * the tier is disabled (the default).  When start > end the
 * window wraps midnight (e.g. 22:00 .. 06:00).
 *
 * UTC is the reference because the kernel only knows wall time;
 * userspace converts the user's local sleep window to UTC and
 * writes the two knobs at boot.  This avoids dragging timezone
 * state into the governor.
 *
 * quiet_hours_screen_off_only (default 1) gates the cap on
 * tunables->screen_state == 0, so an unintended throttle never
 * lands while the user is actively interacting -- the use case
 * is "slow the CPU while the phone is sleeping next to the bed",
 * not "throttle the device mid-call".  Setting it to 0 enables
 * the cap regardless of screen state.
 *
 * Profile-baked: PERFORMANCE / LEGACY keep cap_pct = 100 (no cap),
 * BALANCED holds at 70 %% (mild residency push if a window is
 * configured), BATTERY at 55 %% (aggressive).  The window itself
 * is *not* profile-baked because the user's quiet hours are
 * personal -- profiles only own the cap depth.
 */
#define ZENITH_DEFAULT_QUIET_HOURS_START_MIN		0
#define ZENITH_DEFAULT_QUIET_HOURS_END_MIN		0
#define ZENITH_DEFAULT_QUIET_HOURS_CAP_PCT		100
#define ZENITH_DEFAULT_QUIET_HOURS_SCREEN_OFF_ONLY	1

/* Patch 1.9 fg-transition pulse.  When a foreground (top-app)
 * task is woken for the first time after fork() -- detected via
 * a sched_wakeup_new tracepoint probe with uclamp_eff_value()
 * as the foreground proxy -- arm a one-shot freq floor for
 * fg_transition_pulse_ms milliseconds at fg_transition_pulse_pct
 * of policy->max.  Smooths app-launch / activity-start latency
 * by giving the cluster the freshly-forked task lands on a
 * brief headroom window above PELT cold-start.
 *
 * fg_transition_pulse_ms == 0 disables the tier (BATTERY /
 * LEGACY profiles).  fg_transition_pulse_pct == 0 also disables
 * the floor application; both knobs are profile-baked.
 */
#define ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS		30
#define ZENITH_DEFAULT_FG_TRANSITION_PULSE_PCT		65
#define ZENITH_FG_TRANSITION_PULSE_MS_MAX		200
#define ZENITH_FG_TRANSITION_PULSE_PCT_MAX		100
#define ZENITH_QUIET_HOURS_MINUTE_MAX			1439
#define ZENITH_QUIET_HOURS_CAP_PCT_MIN			50

/* Predictive up-shift via util-trend ring (tier 2a').
 *
 * The peak-headroom rescue (tier 2c) is reactive: it waits for
 * peak_headroom_starve_streak consecutive saturated samples below
 * the freq floor before lifting the cluster, which on a 16 ms
 * sampling cadence is 48..64 ms of starvation before the rescue
 * lands.  The pre-arm tier 2b' shaves one window off the front of
 * that.  This tier shaves two more by acting on a *trend*, not on
 * a level: when the recent util signal is rising fast enough to
 * predict that hispeed_load is about to be crossed, lift the
 * cluster to eff_hispeed_freq one or two ticks before the level-
 * triggered hispeed tier would have done so.
 *
 * Trigger:
 *   delta_x256 = (newest - oldest) over the last predict_up_window
 *                samples, expressed as 256ths of max_cap so the
 *                threshold is unitless.  The compare is

 *
 *                delta_x256 >= predict_up_thresh.
 *   freq < eff_hispeed_freq (otherwise the lift is a no-op).
 *   peak_starve_count == 0 (don't double-fire with rescue / pre-
 *                arm; let those handle it once starvation has
 *                begun).
 *   peak_rescue_until_ns has expired (don't refire inside a
 *                rescue hold-down window).
 *
 * Effect:
 *   Lift freq up to eff_hispeed_freq, set tp_path = "predict_up".
 *   Counter slot ZENITH_STAT_PREDICT_UP records the firing.  The
 *   subsequent hispeed tier (2b) will keep the floor in place if
 *   load_pct actually does cross hispeed_load on the next tick;
 *   otherwise the cluster naturally falls back through the EAS
 *   ladder after the rate-limit window closes.
 *
 * Risk:
 *   Oscillation when the window is short enough to react to PELT
 *   noise.  Mitigated by:
 *     - Gating on peak_starve_count == 0 keeps predict_up from
 *       fighting rescue / pre-arm during sustained-high regimes.
 *     - Default thresh of 64 ( == 25%% of max_cap rise across the
 *       window) is conservative; testers can dial it down to 32
 *       (~12.5%%) for more eager prediction.
 *     - The lift is to eff_hispeed_freq, not policy->max -- the
 *       cluster still has the rescue / brutality tiers above it
 *       when the trend turns out to be a real climb.
 *
 * 0 disables the tier (legacy behaviour: hispeed entry waits for
 * the level signal to arrive).  predict_up_window minimum is 2
 * (need at least two samples to compute a delta) and maximum is
 * ZENITH_PREDICT_UP_WINDOW_MAX so the per-policy ring buffer
 * stays small.
 */
#define ZENITH_DEFAULT_PREDICT_UP_THRESH		64
#define ZENITH_DEFAULT_PREDICT_UP_WINDOW		4
#define ZENITH_PREDICT_UP_THRESH_MAX			255
#define ZENITH_PREDICT_UP_WINDOW_MIN			2
#define ZENITH_PREDICT_UP_WINDOW_MAX			8

/* pelt_rising_edge_thresh (default 32) + pelt_rising_edge_min_pct
 * (default 50): companion to predict_up that catches sharp single-
 * sample slope-up events that the rolling-window delta dilutes.
 *
 * Rationale: predict_up integrates over predict_up_window samples
 * (4 by default), so a workload that takes ~8 samples to reach the
 * cumulative threshold will not lift until the half-way point.  A
 * cold cluster wake-up or a freshly-foregrounded GUI task often
 * shows the steepest util_avg slope on the *first* sample after
 * the cluster left idle; the rolling window misses that with too-
 * conservative thresholds.
 *
 * The rising-edge tier checks the slope between the two newest
 * util_history samples ((newest - prev) * 256 / max_cap).  Crossing
 * pelt_rising_edge_thresh AND newest >= pelt_rising_edge_min_pct *
 * max_cap / 100 lifts the cluster to eff_hispeed_freq with
 * tp_path "pelt_edge".  The min_pct gate prevents firing on
 * tiny-base spikes (e.g. a 5%% util cluster jumping to 8%% in one
 * sample looks steep on the slope but is not actionable).
 *
 * 0 disables the tier (legacy behaviour: only the rolling-window
 * predict_up fires).  Max ZENITH_PELT_RISING_EDGE_THRESH_MAX (255)
 * is the same domain as predict_up_thresh.
 */
#define ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH		32
#define ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT		40
#define ZENITH_PELT_RISING_EDGE_THRESH_MAX		255
#define ZENITH_PELT_RISING_EDGE_MIN_PCT_MAX		100

/* dl_task_floor_pct (default 0, range 0..100, [Patch C6]):
 *
 * SCHED_DEADLINE awareness floor.  When any CPU in the policy
 * has a SCHED_DEADLINE task on its rq (rq->dl.dl_nr_running >
 * 0), lift freq to (policy->max * dl_task_floor_pct / 100).
 *
 * Rationale: schedutil_cpu_util() already adds cpu_bw_dl() to
 * the util signal so DL bandwidth requirements feed into the
 * proportional math automatically.  That math guarantees
 * *average* DL throughput, but a freshly-woken DL task takes
 * roughly one PELT half-life (~32 ms) for its util_avg
 * contribution to fully land, during which the proportional
 * math runs against a stale picture and can miss the first
 * deadline.  Lifting to a per-policy floor closes that
 * responsiveness gap without forcing policy->max for every
 * DL task in the system.
 *
 * 0 disables the tier (legacy behaviour: rely solely on the
 * schedutil_cpu_util DL bandwidth contribution).  Max 100
 * (== policy->max).  pin_to_target paths skip the floor
 * because input_boost / brutality already pin higher.
 *
 * Profile bakes: PERFORMANCE=100, GAMING=100, AUDIO=80,
 * BALANCED=0, BATTERY=0, LEGACY=0.  Cold-boot default 0 keeps
 * legacy behaviour for CUSTOM users; flipping to a profile
 * that enables it does not require any further sysfs work.
 */
#define ZENITH_DEFAULT_DL_TASK_FLOOR_PCT		0
#define ZENITH_DL_TASK_FLOOR_PCT_MAX			100

/* io_floor_hyst_ms / io_floor_hyst_pct
 * (defaults 0 / 50, [Patch C9]):
 *
 * Sticky-floor hysteresis sibling for the iowait_boost path.
 * iowait_boost starts at iowait_boost_min, doubles on each
 * SCHED_CPUFREQ_IOWAIT flag, halves on idle samples; once the
 * episode ends the boost decays to 0 within a few PELT periods,
 * after which the freq drops back to the level signal.  For
 * sustained block IO (file-system flush, sqlite WAL replay,
 * media transcode) the level signal often does not pin a high
 * freq because the worker thread is mostly D-state -- the boost
 * was carrying the freq.  When the boost decays the freq drops,
 * latency on the next IO batch jumps, and the boost has to ramp
 * again.
 *
 * The hysteresis floor stamps a deadline 'now + io_floor_hyst_ms'
 * on the policy whenever zenith_iowait_boost() arms a positive
 * boost.  While that deadline has not expired, lift freq to
 * (policy->max * io_floor_hyst_pct / 100), tp_path "io_floor".
 *
 * 0 ms disables the tier (legacy: rely solely on iowait_boost
 * decay).  Max ZENITH_IO_FLOOR_HYST_MS_MAX (2000 ms; beyond that
 * the floor would routinely outlive the IO episode).  pct in
 * 0..100; 0 disables the floor effect even if window is set.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  500 ms / 70%%
 *   BALANCED:     200 ms / 50%%
 *   BATTERY:        0 ms /  0%%   (off, energy frame)
 *   LEGACY:         0 ms /  0%%   (historical-compat)
 *   GAMING:       300 ms / 60%%
 *   AUDIO:        500 ms / 60%%   (sustained DAC ring writes)
 *   CUSTOM:         0 ms / 50%%   (cold-boot off; pct populated
 *                                  for forward-compat)
 */
#define ZENITH_DEFAULT_IO_FLOOR_HYST_MS			0
#define ZENITH_DEFAULT_IO_FLOOR_HYST_PCT		50
#define ZENITH_IO_FLOOR_HYST_MS_MAX			2000
#define ZENITH_IO_FLOOR_HYST_PCT_MAX			100

/* vh_arch_freq_scale_enable (default 0, [Patch B9-1]):
 *
 * Master 0/1 gate for the android_vh_arch_set_freq_scale vendor-hook
 * observer.  When 1, every realisation of a per-cluster
 * frequency-scale change (the value the scheduler caches for capacity
 * accounting) drops into zenith_probe_arch_set_freq_scale(), updates
 * z_policy->vh_arch_freq_scale_last, and -- if the climb crossed
 * ZENITH_VH_ARCH_FREQ_SCALE_STEP -- arms the peer cluster's
 * peer_ramp window via zenith_peer_ramp_arm().  When 0 the probe is
 * still installed (the cost is one branch on `enable`) but performs
 * no work.
 *
 * Why default 0: the hook fires from arbitrary scheduler context and
 * is purely additive on top of the existing decision-time peer_ramp
 * arming.  Cold-boot users keep the historical timing; opting in is
 * a single sysfs write or a profile flip.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  1   (track realisation; pre-arm peer aggressively)
 *   BALANCED:     0   (cold-boot default; opt-in only)
 *   BATTERY:      0   (extra cross-cluster wakes are not worth it)
 *   LEGACY:       0   (historical-compat)
 *   GAMING:       1   (tighten cross-cluster coupling for input/render)
 *   AUDIO:        0   (audio path benefits from steady cluster, not
 *                      cross-cluster pre-arm)
 *   CUSTOM:       0   (cold-boot opt-in)
 *
 * ZENITH_VH_ARCH_FREQ_SCALE_STEP gates the peer-ramp arm: only a
 * climb of >= 51/1024 of SCHED_CAPACITY_SCALE counts (~5%, which
 * filters governor-noise re-evaluations of the same OPP without
 * dropping real cluster ramps).
 */
#define ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE	0
#define ZENITH_VH_ARCH_FREQ_SCALE_STEP			51

/* vh_uclamp_observer_enable (default 0, [Patch B9-2]):
 *
 * Master 0/1 gate for the android_vh_setscheduler_uclamp vendor-hook
 * observer.  When 1, every userspace uclamp_min raise (the path
 * Android Dynamic Performance Framework -- ADPF -- uses to express
 * "this thread needs more headroom") drops into
 * zenith_probe_setscheduler_uclamp(), looks up the task's current
 * CPU's policy, and -- if that CPU is in a zenith-driven policy --
 * arms peer_ramp on that policy's peer cluster.  Synchronous
 * peer-cluster pre-arm: previously zenith only saw the raised
 * uclamp_min after PELT propagation moved task util upwards, which
 * can take 4..32 ms on a hot game thread.  When 0 the probe is
 * still installed (one branch on `enable`) but performs no work.
 *
 * Filtering inside the probe:
 *   - clamp_id != UCLAMP_MIN -> ignore (uclamp_max raises do not
 *     justify a peer-cluster arm; they only constrain the task's
 *     own cluster downward).
 *   - value == 0 -> ignore (a clear, not a raise).
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  1   (synchronous ADPF response, no PELT lag)
 *   BALANCED:     0   (cold-boot default; opt-in only)
 *   BATTERY:      0   (extra cross-cluster wakes are not worth it)
 *   LEGACY:       0   (historical-compat)
 *   GAMING:       1   (tighten ADPF-triggered cluster coupling --
 *                      this is the headline workload for the hook)
 *   AUDIO:        0   (audio path benefits from steady cluster, not
 *                      cross-cluster pre-arm)
 *   CUSTOM:       0   (cold-boot opt-in)
 */
#define ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE	0

/* vh_cpu_idle_enable (default 1, [Patch B9-3]):
 *
 * Master 0/1 gate for the android_vh_cpu_idle_enter /
 * android_vh_cpu_idle_exit vendor-hook observer pair.  When 1, every
 * cpuidle exit on a CPU belonging to a zenith-driven policy stamps
 * the residency of that idle period (exit_ns - enter_ns) into
 * z_policy->vh_cpu_idle_last_residency_ns.  zenith_get_next_freq()
 * then suppresses cluster_wake_pulse arming when the cluster just
 * emerged from a deep idle (>= ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS).
 *
 * Rationale: cluster_wake_pulse exists to compensate for cold-cache
 * latency after a brief micro-idle.  After a deep idle the workload
 * waking the cluster is fresh / sparse and the next eval will tell
 * us the actual demand within one rate window -- forcing a pulse
 * floor would just waste energy ramping past real demand.  This
 * argument applies equally to every profile, so the gate is enabled
 * across the board out of the box; the sysfs knob remains as a
 * runtime kill-switch in case a regression needs to be triaged
 * without a rebuild.
 *
 * Read-only observer: the enter probe does NOT mutate the cpuidle
 * state index passed by the hook (the hook permits mutation; we
 * leave cpuidle's own selection untouched).  When 0 the probe is
 * still installed (one branch on `enable`) but performs no work.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  1   (fresh-demand reading after deep idle, no
 *                      pulse waste)
 *   BALANCED:     1   (cold-boot default; same energy-saving
 *                      argument as PERFORMANCE)
 *   BATTERY:      1   (energy-sensitive profile -- skipping
 *                      wasteful ramps directly serves the goal)
 *   LEGACY:       1   (historical-compat profile keeps the new
 *                      gate on; runtime kill-switch via sysfs is
 *                      always available)
 *   GAMING:       1   (frame-pace stability beats cold-cache pulse)
 *   AUDIO:        1   (audio path also benefits from skipping
 *                      pulses after long idles between callbacks)
 *   CUSTOM:       1   (cold-boot inherits the default constant)
 *
 * ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS gates the suppression: only
 * an idle period >= 4 ms counts as "deep" enough to gate the
 * pulse.  Brief idles (<<4 ms) leave cwp arming untouched.
 */
#define ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE		1
#define ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS		(4ULL * NSEC_PER_MSEC)

/* vh_freq_qos_enable (default 0, [Patch B9-3+]):
 *
 * Master 0/1 gate for the android_vh_freq_qos_update_request vendor-
 * hook observer.  When 1, every freq-QoS update against a zenith-
 * driven cpufreq policy that raises FREQ_QOS_MIN to a value at or
 * above ZENITH_VH_FREQ_QOS_MIN_PCT of cpuinfo.max_freq stamps a
 * pressure window timestamp on the per-tunables atomic
 * vh_freq_qos_pressure_until_ns.  zenith_auto_classify() then biases
 * to PERFORMANCE while the timestamp is still ahead of the current
 * ktime_get_ns(), so the auto-selector can pivot in response to a
 * deliberate vendor / thermal / ADPF "I want sustained high freq"
 * signal rather than waiting for PELT load to climb.
 *
 * Read-only observer: the probe never mutates req or value; it only
 * stamps a timestamp on a hit.  When 0 the probe is still installed
 * (one branch on `enable`) but performs no work, and the auto-
 * selector consume side likewise short-circuits.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  1   (probe runs for telemetry symmetry; auto-
 *                      classify check is a no-op self-pivot)
 *   BALANCED:     1   (default profile; this is where the AUTO
 *                      pivot to PERFORMANCE on QoS pressure
 *                      actually matters -- the engine sits in
 *                      BALANCED most of the time)
 *   BATTERY:      0   (explicit "ignore vendor pressure, save
 *                      battery"; user-chosen profile must win)
 *   LEGACY:       0   (historical-compat profile keeps the new
 *                      gate off; runtime opt-in via sysfs is
 *                      always available)
 *   GAMING:       0   (already aggressive headroom; redundant)
 *   AUDIO:        0   (audio takes precedence in the cascade
 *                      anyway; the bake is a no-op)
 *   CUSTOM:       0   (cold-boot inherits the default constant)
 *
 * Note: only FREQ_QOS_MIN raises trigger the pressure stamp.
 * FREQ_QOS_MAX updates (typically thermal / battery caps lowering
 * the ceiling) are deliberately ignored -- a thermal cap should not
 * perversely pivot the engine to PERFORMANCE.
 *
 * ZENITH_VH_FREQ_QOS_MIN_PCT (default 75) is the threshold relative
 * to policy->cpuinfo.max_freq; a request below this is not "high"
 * pressure and is ignored.  ZENITH_VH_FREQ_QOS_WINDOW_MS (default
 * 2000) is how long a single hit keeps the pressure flag armed; it
 * matches the default auto_hysteresis_ms so a transient single
 * request lands in the noise the auto-selector already debounces,
 * while a sustained sequence of QoS raises (HAL polling at 250 ms
 * cadence, ADPF push) keeps the flag continuously armed.
 */
#define ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE		0
#define ZENITH_VH_FREQ_QOS_MIN_PCT			75
#define ZENITH_VH_FREQ_QOS_WINDOW_MS			2000

/* vh_sched_move_task_enable (default 0, [Patch B9-5]):
 *
 * Master 0/1 gate for the android_vh_sched_move_task vendor-hook
 * observer.  When 1, every cgroup move that lands a task whose
 * task_cpu() belongs to a zenith-driven cpufreq policy stamps a
 * jiffies timestamp on z_policy->vh_sched_move_task_last_jiffies.
 * Cgroup churn on Android (Activity#onResume / shell cpuset
 * reassignment / top-app promotion) clusters tightly around the
 * "user just brought an app forward" instant; observing this churn
 * synchronously, instead of waiting for the auto-selector worker
 * to next tick, gives AUTO mode a low-latency foreground-transition
 * signal that does not require polling.
 *
 * Read-only observer: the probe never mutates the task; it only
 * stamps the timestamp on a hit.  When 0 the probe is still
 * installed (one branch on `enable`) but performs no work and the
 * timestamp never moves off zero.
 *
 * Hook flavor: this is android_vh_sched_move_task (a regular
 * DECLARE_HOOK, multiple registrants permitted).  The audit-list
 * adjacent candidates -- android_rvh_after_enqueue_task,
 * android_rvh_after_dequeue_task, android_rvh_wake_up_new_task --
 * are deliberately not wired here: they are DECLARE_RESTRICTED_HOOKs
 * (single registrant only, never unregisterable), so a kernel-image
 * registration would permanently monopolise them and block every
 * vendor SoC kernel module that wants to register the same hook for
 * production scheduling decisions.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  0   (no AUTO mode active under explicit
 *                      PERFORMANCE; the probe is observability-only,
 *                      cold-boot opt-in only)
 *   BALANCED:     0   (default profile; AUTO is the consumer of
 *                      this signal but the gate stays opt-in until
 *                      shipping data confirms the timestamp is
 *                      consumed without false positives)
 *   BATTERY:      0   (energy-sensitive profile; observability-only
 *                      probes default off)
 *   LEGACY:       0   (historical-compat profile keeps the new
 *                      gate off; runtime opt-in via sysfs is
 *                      always available)
 *   GAMING:       0   (game cgroup placement is upstream of the
 *                      profile pivot, not downstream; this signal
 *                      does not change a GAMING decision)
 *   AUDIO:        0   (audio path doesn't pivot on cgroup churn)
 *   CUSTOM:       0   (cold-boot inherits the default constant)
 *
 * Hot-path note: android_vh_sched_move_task fires once per
 * cgroup-move (sched_move_task() in core.c), which on Android is
 * sub-Hz in steady state and bursts to a few tens of events per
 * second during app launches / activity transitions.  The probe is
 * lock-free (cpufreq_cpu_get_raw + READ_ONCE on governor_data, then
 * a single WRITE_ONCE to a per-policy field), so even worst-case
 * burst rate is irrelevant to scheduler-tick budget.
 */
#define ZENITH_DEFAULT_VH_SCHED_MOVE_TASK_ENABLE	0

/* vh_scheduler_tick_enable (default 0, [Patch B9-4]):
 *
 * Master 0/1 gate for the android_vh_scheduler_tick vendor-hook
 * observer.  When 1, every scheduler tick that fires on a CPU
 * belonging to a zenith-driven cpufreq policy stamps the wall-time
 * (ktime_get_ns()) of that tick on z_cpu->vh_scheduler_tick_last_ns
 * and bumps z_cpu->vh_scheduler_tick_count by one.  Per-CPU storage:
 * the only writer is the local CPU's tick handler, so no atomics
 * are needed; remote-CPU readers use READ_ONCE.
 *
 * Read-only observer: the probe never mutates the rq, the task, or
 * any scheduler state; it only stamps the timestamp / count pair on
 * a hit.  When 0 the probe is still installed (one branch on
 * `enable`) but performs no work and the per-CPU fields stay at
 * zero.
 *
 * Hook flavor: this is android_vh_scheduler_tick (a regular
 * DECLARE_HOOK, multiple registrants permitted).  The audit-list
 * adjacent candidates -- android_rvh_after_enqueue_task,
 * android_rvh_after_dequeue_task, android_rvh_wake_up_new_task --
 * are deliberately not wired here: they are DECLARE_RESTRICTED_HOOKs
 * (single registrant only, never unregisterable), so a kernel-image
 * registration would permanently monopolise them and block every
 * vendor SoC kernel module that wants to register the same hook for
 * production scheduling decisions.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  0   (no AUTO mode active under explicit
 *                      PERFORMANCE; the probe is observability-only,
 *                      cold-boot opt-in only)
 *   BALANCED:     0   (default profile; gate stays opt-in until
 *                      shipping data confirms the count + timestamp
 *                      pair is consumed without false positives)
 *   BATTERY:      0   (energy-sensitive profile; observability-only
 *                      probes default off -- HZ * num_CPUs hits/s
 *                      means even a no-op probe path is a permanent
 *                      branch in the tick fast path)
 *   LEGACY:       0   (historical-compat profile keeps the new
 *                      gate off; runtime opt-in via sysfs is
 *                      always available)
 *   GAMING:       0   (already heavily-tuned tick treatment elsewhere
 *                      in the governor; no need to layer another
 *                      observer on top by default)
 *   AUDIO:        0   (audio path doesn't pivot on tick recency)
 *   CUSTOM:       0   (cold-boot inherits the default constant)
 *
 * Hot-path note: android_vh_scheduler_tick fires once per scheduler
 * tick, i.e. HZ * num_present_cpus calls per second (250 * 8 = 2000
 * /s on a typical Android arm64 board).  This is the strictest hot
 * path of any zenith vendor-hook observer.  The probe contract is
 * therefore: cpufreq_cpu_get_raw + READ_ONCE on governor_data and
 * tunable gate, then a single ktime_get_ns() and two WRITE_ONCEs to
 * per-CPU fields.  No mutex, no spinlock, no atomic_*.  Tick context
 * runs preempt-disabled with no rq lock held (the hook fires after
 * rq_unlock + trigger_load_balance in scheduler_tick()), so the
 * probe is already in a sleepless / lock-free regime; the only
 * remaining cost is the function call itself and the branch on the
 * enable gate.
 */
#define ZENITH_DEFAULT_VH_SCHEDULER_TICK_ENABLE		0

/* Patch B-AUTO-3: auto-selector engine cadence and hysteresis.
 *
 * auto_eval_ms (default 500): the deferrable workqueue runs the
 * classifier once every auto_eval_ms milliseconds when
 * active_profile == ZENITH_PROFILE_AUTO.  500 ms is the minimum
 * cadence that reliably catches the audio / camera open events
 * without polling so often that we waste wakeups.
 *
 * auto_hysteresis_ms (default 2000): the classifier's chosen
 * target must hold for at least auto_hysteresis_ms before zenith
 * commits the profile switch.  This debounces transient bursts
 * (e.g. a 1 s notification ping that briefly trips the audio
 * detector) so the device does not flap profiles every few
 * hundred ms.
 *
 * Both fields accept 0 -- 0 disables the engine wholesale (the
 * worker re-arms but exits the classifier early).  B-AUTO-5
 * promotes auto_eval_ms / auto_hysteresis_ms to profile-baked
 * defaults; until then they live as the universal defaults
 * defined here.
 *
 * Bounds:
 *   auto_eval_ms        100 .. 60000
 *   auto_hysteresis_ms  0   .. 60000
 *
 * The lower bound on auto_eval_ms keeps the worker out of the
 * 1-cpu-pinned-to-pollworker territory; the upper bound on either
 * cap keeps integration testing tractable.
 */
#define ZENITH_DEFAULT_AUTO_EVAL_MS		500
#define ZENITH_DEFAULT_AUTO_HYSTERESIS_MS	2000
#define ZENITH_AUTO_EVAL_MS_MIN			100
#define ZENITH_AUTO_EVAL_MS_MAX			60000
#define ZENITH_AUTO_HYSTERESIS_MS_MIN		0
#define ZENITH_AUTO_HYSTERESIS_MS_MAX		60000

/* peak_hysteresis_streak / peak_step_down_pct
 * (defaults 3 / 95, [Stage 4 / Patch E]):
 *
 * Peak-return hysteresis.  When the previous evaluation pinned
 * the cluster at or near peak (freq >=
 * ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT of policy->max) and the
 * current evaluation wants to drop sharply, hold a soft floor
 * at (prev_freq * peak_step_down_pct / 100) for the next
 * peak_hysteresis_streak samples.  After the streak drains, the
 * cluster falls naturally through the lower tiers.
 *
 * Goal: smooth the off-peak descent on bursty workloads.  A
 * render thread that just finished a frame and is now idle
 * waiting for vblank produces a single sample of low load while
 * the next frame is still queued; the natural descent would
 * plunge to EAS-suggested freq and then bounce back up the next
 * sample.  Hysteresis trades a few ms of higher freq for a
 * smoother descent and far fewer freq transitions.
 *
 * Either tunable at 0 disables the tier (legacy).  Range checks
 * keep streak in 0..ZENITH_PEAK_HYSTERESIS_STREAK_MAX and
 * step_down_pct in 0..100.
 */
#define ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK		3
#define ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT		95
#define ZENITH_PEAK_HYSTERESIS_STREAK_MAX		16
#define ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT		90

/* boost_idle_thresh / boost_idle_streak
 * (defaults 15 / 3, [Stage 4 / Patch F]):
 *
 * Boost early-exit on persistent idle.  When an input boost is
 * armed (now < zenith_input_boost_until_ns) but the cluster has
 * sat below load_pct boost_idle_thresh for boost_idle_streak
 * consecutive ticks, the per-policy tier-0 preempts the boost
 * on this tick instead of pinning to the boost ceiling.
 *
 * Goal: stop pinning the cluster to peak when the workload that
 * triggered the boost has clearly drained.  A user tap that
 * launches an app is a typical case: the launch animation
 * finishes well before input_boost_ms expires, but the natural
 * idle that follows would still see the cluster pinned at the
 * boost ceiling for the rest of the window.  Boost early-exit
 * trims the energy tail without affecting the launch-frame
 * latency the boost was actually for.
 *
 * Per-policy preemption only: the global
 * zenith_input_boost_until_ns is intentionally left armed so
 * other clusters that are still busy keep their boost.  Each
 * policy makes its own idle-streak decision.
 *
 * Either tunable at 0 disables the early-exit (legacy
 * boost-honoured-to-its-deadline behaviour).  Range checks:
 * thresh in 0..100 (load percent), streak in
 * 0..ZENITH_BOOST_IDLE_STREAK_MAX.
 */
#define ZENITH_DEFAULT_BOOST_IDLE_THRESH		15
#define ZENITH_DEFAULT_BOOST_IDLE_STREAK		3
#define ZENITH_BOOST_IDLE_STREAK_MAX			16

/* bg_util_scale_pct
 * (default 100 = off, [Stage 4 / Patch G]):
 *
 * Background-task util scaling.  When the display is off
 * (tunables->screen_state == 0), scale the util signal returned
 * by zenith_get_util() down to bg_util_scale_pct percent of its
 * natural value.  All downstream tiers (brutality, EAS,
 * ladder) see the lower signal, so freq decisions during
 * screen-off run further from policy->max for the same
 * underlying load.
 *
 * Goal: trim energy on background sync / wake-lock work that
 * runs while the device is locked.  These workloads are
 * typically not user-perceivable; running them on a slightly
 * cheaper freq point is a free energy win.
 *
 * Bypassed when screen_state == 1 so display-on responsiveness
 * is unchanged.  100 (default) is a no-op (full util passes
 * through).  0 is rejected by the sysfs store -- the kernel
 * already has cpu-idle paths for the "no work" case; this knob
 * is for *scaling* not for *gating*.  Range checked at
 * 1..100 to avoid that footgun.
 *
 * Reads via READ_ONCE on the eval hot path (zenith_get_util()
 * is called once per CPU per evaluation tick).
 */
#define ZENITH_DEFAULT_BG_UTIL_SCALE_PCT		100
#define ZENITH_BG_UTIL_SCALE_PCT_MIN			1

/* sleeper_tail_thresh_us / sleeper_tail_pct
 * (defaults 0 / 90, [Stage 4 / Patch H]):
 *
 * Sleeper-tail shaving.  When the cluster has been idle (no
 * runnable load_pct samples) for sleeper_tail_thresh_us
 * microseconds, shave the next freq decision down by
 * sleeper_tail_pct / 100, clamped at policy->min.  Saves
 * leakage on sleep entry by parking the cluster one DVFS rung
 * lower than it would otherwise sit on the wake-up tick.
 *
 * thresh_us == 0 (default) disables the tier; sleeper_tail_pct
 * is bounded in 50..100 (anything below 50 would slam the
 * cluster too low on wake and re-up immediately, which is the
 * opposite of what we want).
 *
 * Reads via READ_ONCE on the eval hot path; writes via
 * WRITE_ONCE from sysfs and from zenith_apply_profile().
 */
#define ZENITH_DEFAULT_SLEEPER_TAIL_THRESH_US		0
#define ZENITH_SLEEPER_TAIL_THRESH_US_MAX		100000
#define ZENITH_DEFAULT_SLEEPER_TAIL_PCT			90
#define ZENITH_SLEEPER_TAIL_PCT_MIN			50
#define ZENITH_SLEEPER_TAIL_PCT_MAX			100

/* peer_ramp_window_ms / peer_ramp_floor_pct
 * (defaults 25 / 60, [Stage 4 / Patch D]):
 *
 * Multi-cluster pre-arm coordination.  Cross-cluster IPC chains
 * (binder hops between an app-side BIG worker and a service-side
 * PRIME worker, audio pipelines feeding a render thread, etc.)
 * tend to need both clusters at a usable freq within a few
 * milliseconds of each other.  In the legacy path, when one
 * cluster is woken into peak by predict_up / peak_prearm /
 * peak_rescue, the peer cluster has to climb from idle by
 * itself, paying a full hispeed warm-up window before its half
 * of the IPC chain runs at speed.
 *
 * The arming side: when a cluster lifts to eff_hispeed (or
 * higher) via one of the peak tiers, stamp a deadline on the
 * peer cluster's slot.  The reading side: while that deadline
 * has not expired, the peer applies a soft floor at
 * peer_ramp_floor_pct of policy->max so it is no longer sitting
 * at idle when the cross-cluster wake arrives.  Class-based, not
 * policy-based: BIG arms PRIME and vice versa.  LITTLE does not
 * participate -- it is rarely on the producing side of a peer-
 * ramp-worthy IPC chain, and floor-arming it would interfere
 * with the bg_util_scale_pct screen-off path on devices that
 * route low-priority work to the small cluster.
 *
 * Self-disarms on the deadline.  No streak / debounce: the
 * triggers (predict_up trend window, peak_starve_count >=
 * starve_streak, peak_prearm gate) already require multiple
 * samples worth of evidence, so by the time the peer fires we
 * have all the confirmation we need.  Re-armings just bump the
 * deadline forward; harmless.
 *
 * peer_ramp_window_ms == 0 disables both sides (no arming
 * writes, no floor reads).  peer_ramp_floor_pct == 0 disables
 * just the floor (deadlines still get stamped but never
 * fire) -- mostly useful for trace consumers that want to see
 * the arming events without having the floor influence freq.
 *
 * 100 ms / 100% are the upper bounds.  The 100 ms cap is
 * loose: at peer_ramp_floor_pct=60 the floor only matters when
 * the natural freq would be below 60% of policy->max, which on
 * a real workload is a small fraction of the window.  The
 * 100% cap on the floor is the obvious one (anything higher is
 * just policy->max).
 */
#define ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS		25
#define ZENITH_PEER_RAMP_WINDOW_MS_MAX			100
#define ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT		60
#define ZENITH_PEER_RAMP_FLOOR_PCT_MAX			100

/* peer_ramp_window_off_ms (default 0, [Stage 5 / Patch M3]):
 *
 * Screen-state-aware override of peer_ramp_window_ms.  When the
 * screen is off the cross-cluster IPC chains peer_ramp exists to
 * accelerate are mostly absent: there is no compositor, no app
 * render thread, no input handler.  Pre-arming a peer cluster in
 * that regime burns idle big-cluster freq for nothing.
 *
 * Same shape as screen_off_glide_ms: when tunables->screen_state
 * is 0 the peer-ramp arming and floor-eval paths use this value
 * in place of peer_ramp_window_ms.  Default 0 means peer_ramp is
 * fully suppressed while the screen is off (no arm writes, no
 * floor reads).  Set equal to peer_ramp_window_ms to restore the
 * pre-Stage-5 always-on behaviour byte-identically.  Range
 * 0..ZENITH_PEER_RAMP_WINDOW_MS_MAX, same upper bound as the
 * screen-on knob since the off variant is just a different value
 * for the same physical timer.
 *
 * Energy-only refinement: cannot raise the peer-ramp floor higher
 * than the screen-on path already does, so this knob can never
 * hurt responsiveness; it can only stop spending energy on a
 * cluster the user is not looking at.
 */
#define ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS		0

/* migration_jump_pct / migration_floor_window_ms / migration_floor_pct
 * (defaults 20 / 30 / 60, [Stage 4 / Patch K1]):
 *
 * Migration-arrival soft floor.  When a high-util task migrates
 * between CPUs, the source CPU's util drops on the next sample
 * (the task is gone) but the destination's util takes ~32 ms
 * (one PELT half-life) to fully reflect the new load.  In that
 * gap, the destination cluster's eval can pick a freq based on
 * stale-low aggregate util.
 *
 * On every per-CPU update_util tick the governor compares the
 * current util to the value seen on the previous tick.  If the
 * sample-to-sample jump exceeds migration_jump_pct of the CPU's
 * max_capacity, treat it as evidence that a new task just landed
 * here and stamp a deadline on this policy's
 * migration_in_until_ns slot.  While that deadline holds, the
 * eval applies a soft floor at migration_floor_pct of
 * policy->max so the cluster is not running at idle freq for
 * the first half of the new task's PELT warm-up.
 *
 * Per-policy, not class-level: this tracks "task arrived here"
 * regardless of which cluster the task came from.  Composes
 * cleanly with peer_ramp (Patch D) which is the cross-cluster
 * IPC case; peer_ramp arms the *peer* of a ramping cluster,
 * migration_floor arms the *self* of an arrival.  No conflict.
 *
 * Self-disarms by deadline.  Re-armings just bump the deadline
 * forward.  Single-CPU policies (1+1+1 topologies, etc.) work
 * the same way: a task moving onto the only CPU in the policy
 * still triggers a util jump on that CPU.
 *
 * migration_jump_pct == 0 disables both sides (no arming
 * writes, no floor reads).  migration_floor_pct == 0 leaves the
 * stamping in place but suppresses the floor.
 *
 * The default jump threshold is 20%, which is the boundary
 * where empirically (a) PELT half-life dynamics + 4-CPU
 * averaging start producing visible per-CPU spikes from a
 * single task, and (b) routine util oscillation (sched_yield
 * loops, 1-tick-on-1-tick-off micro-bursts) tends to stay
 * below.  Window 30 ms covers most of one PELT half-life.
 */
#define ZENITH_DEFAULT_MIGRATION_JUMP_PCT		20
#define ZENITH_MIGRATION_JUMP_PCT_MAX			100
#define ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS	30
#define ZENITH_MIGRATION_FLOOR_WINDOW_MS_MAX		100
#define ZENITH_DEFAULT_MIGRATION_FLOOR_PCT		60
#define ZENITH_MIGRATION_FLOOR_PCT_MAX			100

/* psi_cpu_floor_thresh (default 0, off, [Stage 4 / Patch K2]):
 *
 * PSI-CPU-aware sustained-pressure floor.  Mirror of the existing
 * psi_cpu_thresh cap but pointing the other direction.  When
 * zenith_psi_cpu_some_pct() (the 10s-EWMA of system-wide CPU
 * stall %) is at or above psi_cpu_floor_thresh, lift freq to
 * zenith_eff_hispeed_freq().
 *
 * Why this sits next to psi_cpu_thresh and not next to predict_up
 * or peak_rescue: PSI's 10 s smoothing window is too slow for
 * sub-second decisions, so this tier is by design only useful
 * for *sustained* CPU pressure -- gaming + background sync,
 * screen-record + foreground app, multi-app multitasking with a
 * background compile, etc.  Predict_up / peak_rescue / peak_prearm
 * cover the sub-second up-decisions; this tier covers the
 * "we've been queueing for 10+ seconds and util is still under
 * hispeed entry threshold" case where the existing tiers don't
 * fire because aggregate util doesn't capture queueing pressure
 * cleanly.
 *
 * Conservative default (0, off): the user has to opt in.  Set
 * via the per-profile mirror in zenith_apply_profile() so PERF
 * gets a moderate threshold automatically and BAT/LEG keep it
 * disabled.  When set, only fires when zenith_eff_hispeed_freq()
 * is non-zero; if hispeed is unconfigured the tier no-ops rather
 * than fall back to policy->max (which would be too aggressive
 * for what is, after all, a smoothed-pressure signal).
 */
#define ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH		0
#define ZENITH_PSI_CPU_FLOOR_THRESH_MAX			100

/* frame_overrun_slack_us / frame_overrun_window_ms /
 * frame_overrun_floor_pct (defaults 0 / 50 / 80,
 * [Stage 4 / Patch K3]):
 *
 * Frame-budget overrun rescue.  Companion to the existing
 * frame_pace_floor tier (Stage 3) -- frame_pace arms a floor
 * sized to fit one frame in the budget, this tier corrects
 * after a frame missed.
 *
 * Detection runs in zenith_drm_vblank_event(), a new exported
 * symbol the panel driver / display HAL is expected to call on
 * every vblank.  The first call after governor attach (or after
 * the screen-state stale guard fires) just records the timestamp
 * and returns.  On subsequent calls the elapsed wall-clock delta
 * is compared against (zenith_drm_vblank_us + slack_us); if the
 * gap is wider, the renderer missed the budget.  Stamp a
 * deadline frame_overrun_window_ms in the future on the file-
 * scope zenith_frame_overrun_until_ns slot.  While the deadline
 * holds, every cluster's eval lifts to a soft floor at
 * frame_overrun_floor_pct of policy->max.
 *
 * Why a *new* exported function and not a reuse of
 * zenith_set_drm_vblank_us(): the existing setter takes the
 * vblank *period* in microseconds and is only called on
 * refresh-rate transitions (60 Hz <-> 120 Hz).  This patch needs
 * a *per-vblank* event, which is a different concept.  Decoupled
 * so a panel driver can wire up either, both, or neither
 * depending on what it knows.  When the panel driver does not
 * call zenith_drm_vblank_event(), the entire detection path is
 * a no-op (the deadline is never stamped) and the rest of the
 * governor behaves exactly as before -- same fail-safe shape as
 * the existing zenith_set_drm_vblank_us() path.
 *
 * frame_overrun_slack_us == 0 disables stamping (the producer
 * still updates last_vblank_ns so the next 0 -> non-zero
 * configuration change starts cleanly).
 * frame_overrun_floor_pct == 0 leaves stamping but suppresses
 * the floor.
 *
 * Cold-boot default for slack_us is 0 (off).  Per-profile
 * values turn it on: BALANCED uses 4000 us, which is roughly a
 * third of a 60 Hz vblank period (16667 us).  Smaller and
 * routine driver / scheduler jitter trips the detector; larger
 * and an actual single missed frame (16667 us late) doesn't
 * trip it.  Window 50 ms covers about three 60 Hz frames or six
 * 120 Hz frames -- enough recovery time after a single miss
 * without holding a high-freq pin past a brief stall.
 */
#define ZENITH_DEFAULT_FRAME_OVERRUN_SLACK_US		4000
#define ZENITH_FRAME_OVERRUN_SLACK_US_MAX		16667
#define ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS		50
#define ZENITH_FRAME_OVERRUN_WINDOW_MS_MAX		200
#define ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT		80
#define ZENITH_FRAME_OVERRUN_FLOOR_PCT_MAX		100

/* frame_overrun_deep_streak / frame_overrun_deep_floor_pct
 * (defaults 0 / 100, [Stage 5 / Patch M5]):
 *
 * Sub-knob inside K3.  A single overrun is plausibly a one-shot
 * scheduler / GC / page-fault hiccup; the standard K3 floor at
 * frame_overrun_floor_pct (typ. 80%) is sized for that case.  N
 * consecutive overruns at the same panel period is qualitatively
 * different -- something is sustained-overloaded -- and the
 * recovery floor should escalate.
 *
 * Implementation: the producer (zenith_drm_vblank_event())
 * already detects per-vblank overruns and stamps a deadline.
 * Track a governor-wide consecutive-overrun streak: bump on
 * each overrun, reset on each within-budget vblank.  In the
 * consumer (the K3 read site in zenith_get_next_freq()), once
 * the streak crosses frame_overrun_deep_streak the floor lifts
 * from frame_overrun_floor_pct to frame_overrun_deep_floor_pct.
 *
 * frame_overrun_deep_streak == 0 disables the deep tier
 * entirely (the consumer never reads the streak atomic).  The
 * default of 0 keeps Stage 4 K3 byte-identical for users who
 * don't opt in.  PERFORMANCE profile arms it at 2 / 100% --
 * two consecutive misses at 60 Hz is ~33 ms of stutter, well
 * outside any plausible jitter explanation, and the user has
 * already opted into the energy / responsiveness trade by
 * picking PERFORMANCE.  All other profiles ship at 0.
 *
 * Capped at 16 to keep streak overflow a non-issue (atomic_t
 * is 31 bits but practically the read-site comparison only
 * cares about reaching the threshold; once past, the streak
 * keeps bumping until reset and the comparison stays true).
 * deep_floor_pct is capped at 100 (anything higher is just
 * policy->max again) and lower-bounded by the read site at
 * the existing frame_overrun_floor_pct -- the deep tier never
 * produces a *lower* floor than the standard K3 floor.
 *
 * Sits inside K3's V2 arming gate: if V2 has disarmed K3 for
 * this state (eff_floor_pct == 0), the deep tier is also
 * inactive because the read-site short-circuit fires before
 * the streak comparison.  No new V2 tier bit; this is an
 * amplification of K3, not a separate tier.
 */
#define ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK	0
#define ZENITH_FRAME_OVERRUN_DEEP_STREAK_MAX		16
#define ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT	100
#define ZENITH_FRAME_OVERRUN_DEEP_FLOOR_PCT_MAX	100

/* peer_ramp_uclamp_min_respect / migration_floor_uclamp_min_respect
 * (defaults 1 / 1, [Stage 5 / Patch M2]):
 *
 * Per-tier sub-gates that let the peer_ramp (Patch D) and
 * migration_floor (Patch K1) floors respect a task's
 * uclamp_min hint.  Independent of the existing
 * uclamp_min_respect master gate (which controls the
 * *final-freq* floor at line ~6811): these gates only affect
 * the per-tier intermediate floors.
 *
 * Effective floor at each site becomes:
 *   max(static_floor_pct, uclamp_min_as_pct_of_max)
 *
 * When the inbound / on-policy task has uclamp_min == 0 (the
 * common case) the max() is a no-op and behaviour is byte-
 * identical to Stage 4.  When the task has an explicit ADPF-
 * driven uclamp_min, the new path lifts the floor *up* toward
 * what the scheduler already owes the task -- this can never
 * produce a *lower* floor than the static knob alone.
 *
 * Why two bools and not one: peer_ramp and migration_floor
 * are independently configurable in the existing per-profile
 * presets; some profiles arm one without the other (e.g.
 * BATTERY disables migration_floor but a future profile might
 * keep peer_ramp at non-zero).  Two bools keep the matrix
 * clean.
 *
 * Default 1 because the uclamp-respecting path is strictly a
 * floor-raise, never a floor-lower.  Set 0 to revert to the
 * pre-Stage-5 byte-identical behaviour for that specific tier.
 */
#define ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT		1
#define ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT	1

/* psi_mem_cap_thresh / psi_mem_cap_pct / psi_mem_cap_window_ms
 * (defaults 0 / 80 / 1000, [Stage 5 / Patch M1]):
 *
 * Symmetric companion to the existing psi_mem_thresh predicate.
 * The decision chain already gates the *up-push* on memstall via
 * zenith_psi_mem_some_pct() >= psi_mem_thresh.  But the cap side
 * (uclamp_max / light_cap / audio_cap / em_cap / existing
 * psi-cap-at-hispeed) doesn't pull the *final* freq down when
 * memstall climbs past a separate, lower threshold.  Pushing CPU
 * into peak under heavy paging just deepens the stall: the CPU
 * has nothing useful to do while it waits on mm.  This tier adds
 * a final-freq cap that fires exactly there.
 *
 * Knob shape:
 *   psi_mem_cap_thresh    (0..100, default 0 = off)
 *   psi_mem_cap_pct       (50..100, default 80%)
 *   psi_mem_cap_window_ms (100..5000, default 1000 ms)
 *
 * When psi_aware == 1 (master gate) AND psi_mem_cap_thresh > 0
 * AND zenith_psi_mem_some_pct() >= psi_mem_cap_thresh, the eval
 * path stamps z_policy->psi_mem_cap_until_ns with a deadline
 * `now + psi_mem_cap_window_ms`.  While that deadline has not
 * expired, the final freq is capped at psi_mem_cap_pct of
 * policy->max.  Once the EWMA falls below thresh and the window
 * lapses, the cap releases without further hysteresis.
 *
 * Defaults:
 *   - psi_mem_cap_thresh = 0 (off) so the patch is fully inert
 *     out of the box.  Users who opt into psi_aware = 1 already
 *     accept the existing PSI cap above; this tier is a stricter
 *     opt-in.
 *   - psi_mem_cap_pct = 80% so the cap is meaningful but not
 *     punitive (a healthy floor for browser / scrolling under
 *     mild memstall).  PERFORMANCE bumps to 90% for the user
 *     who has explicitly picked PERF.
 *   - psi_mem_cap_window_ms = 1000 ms so the cap stays in place
 *     long enough to absorb a full mmap_sem / kswapd burst
 *     without flapping every tick.  Aligned with the 10s EWMA
 *     timescale: the EWMA's natural reset-to-zero is glacial,
 *     so the window is what governs cap release.
 *
 * Range max (5000 ms) caps the worst-case stuck-cap to 5 s, the
 * order of magnitude where the user would notice "phone is slow"
 * regardless of governor reasoning.
 *
 * V2 tier mapping:
 *   ZENITH_AT_STATE_EFFICIENCY   -> arm  (back off on stall is
 *                                  exactly the EFFICIENCY job)
 *   ZENITH_AT_STATE_BALANCED     -> arm
 *   ZENITH_AT_STATE_THERMAL_RECOVERY -> arm
 *   ZENITH_AT_STATE_LATENCY      -> disarm (this is a cap, not
 *                                  a floor; LATENCY does not
 *                                  want any extra caps)
 *   ZENITH_AT_STATE_SUSTAINED_PERF -> disarm (user has explicitly
 *                                    asked for top-end)
 *   FRAME / GAME flag bypass     -> disarm (frame-pacing and game
 *                                  overrides need full headroom)
 *
 * Cannot regress at default: psi_mem_cap_thresh == 0 short-
 * circuits before any read of the EWMA; psi_aware == 0 short-
 * circuits a level higher; auto_tune_v2_tiers == 0 bypasses the
 * tier bit entirely.  When the tier *is* armed and fires, the
 * cap is bounded below by policy->min and above by policy->max,
 * so a misconfigured psi_mem_cap_pct cannot drop the policy
 * below its natural floor.
 */
#define ZENITH_DEFAULT_PSI_MEM_CAP_THRESH		0
#define ZENITH_PSI_MEM_CAP_THRESH_MAX			100
#define ZENITH_DEFAULT_PSI_MEM_CAP_PCT			80
#define ZENITH_PSI_MEM_CAP_PCT_MIN			50
#define ZENITH_PSI_MEM_CAP_PCT_MAX			100
#define ZENITH_DEFAULT_PSI_MEM_CAP_WINDOW_MS		1000
#define ZENITH_PSI_MEM_CAP_WINDOW_MS_MIN		100
#define ZENITH_PSI_MEM_CAP_WINDOW_MS_MAX		5000

/* up_threshold_adaptive (default 0, off):
 *
 * Variance-adaptive shaping of the brutality entry threshold.  The
 * static up_threshold is a single number that has to fit two
 * different workloads: bursty / interactive (UI scrolling, gestures,
 * frame pacing) where a low up_threshold is desirable so the cluster
 * climbs fast on a single hot sample, and sustained / steady
 * (transcoding, long compute) where a higher up_threshold avoids
 * pinning the cluster at max for the whole run.
 *
 * When set to N (1..30), zenith_get_next_freq() lowers the effective
 * up_threshold by up to N percent when the recent load signal is
 * bursty, leaving it unchanged on a steady signal.  The bursty/
 * steady signal is a rolling EWMA of |load_pct - prev_load_pct|;
 * high mean-absolute-change == bursty.  The adjustment is applied
 * only on the regular up_threshold path -- the screen-off (95),
 * thermal (90), and up_threshold_hispeed overrides are absolute
 * pinning values and stay verbatim.
 *
 * 0 disables the adjustment entirely (legacy: dynamic_up_thresh ==
 * tunables->up_threshold whenever no override fires).  Cap at 30 so
 * a runaway tunable can never lower up_threshold by more than 30%
 * of its value, which would be indistinguishable from "force snap"
 * behaviour.
 */
#define ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE	0
#define ZENITH_UP_THRESHOLD_ADAPTIVE_MAX	30

/* Time-based cache TTL for the uclamp_{min,max} per-policy walks.  The
 * per-rq UCLAMP values are maintained by the scheduler on every
 * enqueue / dequeue, so a 1 ms staleness bound on the cached
 * aggregate is imperceptible to userspace (ADPF sessions are open
 * for tens of milliseconds to seconds) but drops the per-policy rq
 * walk from every freq eval down to once per millisecond.
 */
#define ZENITH_UCLAMP_CACHE_TTL_NS		(1 * NSEC_PER_MSEC)

/* Maximum number of bins in the efficient_freq soft-cap ladder.
 *
 * The ladder is the array of (eff_freq, eff_delay_us) pairs the
 * efficient_freq= / up_delay_us= sysfs nodes accept and that
 * zenith_get_next_freq() walks once per evaluation.  This number
 * bounds three things: the parser's parsed[] stack buffer, the
 * static eff_freq[] / eff_delay_us[] tables on struct
 * zenith_tunables, and the per-policy eff_unlock_at_ns[] deadline
 * table on struct zenith_policy.  The runtime value of eff_nr
 * (set by the parser, capped to this ceiling) decides how many of
 * those slots the hot path actually walks; a CSV with fewer entries
 * leaves the rest of the array unused but does not save any
 * footprint.
 *
 * Was 4, raised to 8 to fit the more finely-binned freq tables
 * found on Dimensity 9000+ / Snapdragon 8 Gen 3-class SoCs.  A bin
 * count above 8 is hard to tune by hand and burns d-cache on the
 * walk for diminishing return -- the cap is deliberate and not
 * runtime-configurable.  Memory cost vs the old value: 8 - 4 = 4
 * additional uints per array; tunables grows by 32 bytes (two
 * arrays) and zenith_policy by 32 bytes (one u64 array), totals
 * negligible against the rest of those structs.  Hot-path cost is
 * unchanged: the loop bound is eff_nr, not the array ceiling.
 */
#define ZENITH_EFF_BINS_MAX			8

/* Depth of the per-policy auto-tune classifier ring buffer.
 *
 * Each entry records one V1 classifier window (the worker fires every
 * ZENITH_AUTO_TUNE_PERIOD_MS = 10 s) plus the V2 state machine view at
 * that moment.  16 entries gives roughly 160 s of post-mortem visible
 * via the read-only at_log sysfs node, which covers a typical bench
 * run, an app launch, a bursty UI sequence or a thermal-recovery cycle
 * without any extra tooling.
 *
 * The ring is a single-writer/multi-reader buffer: only the per-policy
 * delayed_work worker pushes; sysfs readers walk it from oldest to
 * newest under no extra locking, accepting at most one window of
 * tearing on the wrap (rare and harmless for diagnostics).  Storage is
 * ~64 bytes per entry, so 16 entries cost ~1 KiB per policy.
 */
#define ZENITH_AT_LOG_NR			16

/* M2 V2 state-transition history ring depth.  32 entries = 512 B per
 * policy (16 B / entry, see struct zenith_policy::at_history).  Sized
 * to span ~30 s of typical phone-class bursty workloads (camera open,
 * scroll, app launch) so a userspace triager reading the file can see
 * the last interesting cluster of state changes without round-tripping
 * to dmesg / perfetto.
 */
#define ZENITH_AT_HISTORY_NR			32

#define ZENITH_CLIMB_MODE_SNAP			0	/* default */
#define ZENITH_CLIMB_MODE_STEP			1
#define ZENITH_PROFILE_CUSTOM			0	/* default */
#define ZENITH_PROFILE_PERFORMANCE		1
#define ZENITH_PROFILE_BALANCED			2
#define ZENITH_PROFILE_BATTERY			3
#define ZENITH_PROFILE_LEGACY			4
#define ZENITH_PROFILE_GAMING			5
#define ZENITH_PROFILE_AUDIO			6

/* Patch B-AUTO-2: auto-profile selector meta-state.
 *
 * When active_profile == ZENITH_PROFILE_AUTO the auto-selector
 * engine (B-AUTO-3 / B-AUTO-4) drives profile bakes from observed
 * device state -- audio activity, game-engine threads, render-
 * thread saturation, foreground input recency, screen state, and
 * (B-AUTO-4) battery level / charging state.  The user sees a
 * single sysfs knob ("echo auto > profile") and zenith picks the
 * most appropriate concrete profile (BALANCED, PERFORMANCE,
 * BATTERY, GAMING, or AUDIO) on a 500 ms cadence with 2000 ms
 * hysteresis.
 *
 * AUTO is a *meta* profile: zenith_apply_profile(t, AUTO) is never
 * called.  Instead the engine writes the chosen concrete target
 * into tunables->auto_target and applies that.  active_profile
 * stays at AUTO; auto_target reflects the engine's current pick.
 *
 * Manual profiles (PERFORMANCE / BALANCED / BATTERY / LEGACY /
 * GAMING / AUDIO / CUSTOM) take precedence on write -- writing any
 * concrete profile to the profile sysfs node disengages auto until
 * the user explicitly writes "auto" again.
 *
 * LEGACY and CUSTOM are never auto-picked -- they are explicit
 * opt-out paths reserved for advanced users who have layered their
 * own per-knob tweaks on top.
 */
#define ZENITH_PROFILE_AUTO			7

/* Profile selected via the zenith.profile= kernel cmdline. Parsed by
 * zenith_setup_profile() at early_param time and consumed on the
 * first-init branch of zenith_init() so the governor comes up on the
 * requested preset before any userspace can write to the profile sysfs
 * node. Defaults to CUSTOM, which means "no cmdline override".
 */
static unsigned int zenith_cmdline_profile = ZENITH_PROFILE_CUSTOM;

/* Optional per-policy cmdline profile overrides parsed from
 * zenith.policy_profile=N:prof,M:prof,...  Indexed by the "policy
 * anchor cpu" (cpumask_first(policy->cpus) at init time -- what
 * SurfaceFlinger and the cpufreq sysfs tree already print as
 * policyN).  Each slot defaults to ZENITH_PROFILE_CUSTOM meaning
 * "no per-policy override; fall through to zenith_cmdline_profile".
 *
 * Using a fixed-size array indexed by CPU is intentional: the
 * parser runs at early_param time when per-cpu structures aren't
 * fully populated, and an NR_CPUS-sized u8 table costs one byte
 * per possible CPU on the kernel image (256 bytes on a common
 * aarch64 defconfig) -- cheaper than any dynamic allocation would
 * save, and lookup is a single load at init time.
 */
static u8 zenith_cmdline_policy_profile[NR_CPUS] = {
	[0 ... NR_CPUS - 1] = ZENITH_PROFILE_CUSTOM,
};

/* Static-branch fold for zero-default feature tunables.
 *
 * audio_aware, camera_aware, render_aware and psi_aware all default
 * to 0 (off) and are checked on every zenith_get_next_freq() call.
 * Folding them through a DEFINE_STATIC_KEY_FALSE turns the hot-path
 * "if (z_policy->tunables->X)" load-cmp-branch sequence into a
 * single never-taken jump while the feature is disabled, with the
 * cold path moved out of line for better i-cache density.
 *
 * The keys are governor-global (one zenith_tunables instance is
 * shared across all policies, see global_tunables_lock), so there
 * is no per-policy synchronisation question.  Each *_store callback
 * synchronises its key with the new tunable value via
 * static_branch_enable / static_branch_disable, which both sleep
 * acquiring cpus_read_lock() but are safe from sysfs store context.
 *
 * Init-time invariant:
 *
 *   - zenith_audio_aware_key and zenith_render_aware_key match scalars
 *     that were flipped to default 1 in the wave-2 auto-defaults
 *     round, so the keys must be explicitly enabled in zenith_init()
 *     after the tunable defaults are written, otherwise the hot path
 *     would read the scalar as 1 but skip the branch via the still-FALSE
 *     key.  zenith_init() now calls zenith_set_static_key() against
 *     each scalar's value (idempotent across re-attaches).
 *   - zenith_camera_aware_key and zenith_psi_aware_key match scalars
 *     that were flipped to default 1 in the auto-defaults round that
 *     mirrored audio_aware / render_aware, so the same init-time sync
 *     rule applies: zenith_init() must enable the key against the
 *     default scalar value.
 *   - zenith_game_auto_key and zenith_auto_tune_v3_key match scalars
 *     that were flipped to non-zero defaults in the wave-7
 *     auto-defaults round (game_auto = 1, auto_tune_v3 = 2), so the
 *     same init-time sync rule applies: zenith_init() must enable
 *     the key against the default scalar value.  The auto_tune_v3
 *     key is binary even though the scalar is tri-valued (0/1/2);
 *     zenith_set_static_key() treats any non-zero value as TRUE, so
 *     observe-only mode (scalar = 1) and apply mode (scalar = 2)
 *     both produce key = TRUE.  See ZENITH_DEFAULT_AUTO_TUNE_V3
 *     comment block.
 *   - zenith_thermal_aware_key matches thermal_aware, which defaults
 *     to 1 (the master gate is on out of the box).  Same init-time
 *     sync rule as audio_aware / render_aware: zenith_init() must
 *     call zenith_set_static_key() so the key starts TRUE; otherwise
 *     the gated thermal mechanisms (thermal_util_derate,
 *     auto_thermal_cap, the V2 THERMAL_RECOVERY transitions, and
 *     zenith_thermal_active()) would read the scalar as 1 but skip
 *     the branch via the still-FALSE key, silently disabling thermal
 *     handling on a default install.
 *   - zenith_set_profile_defaults() never touches any of the seven
 *     scalars (they are user-managed opt-ins, not preset state), so
 *     no profile-apply path needs to re-sync the keys.
 */
static DEFINE_STATIC_KEY_FALSE(zenith_audio_aware_key);
static DEFINE_STATIC_KEY_FALSE(zenith_camera_aware_key);
static DEFINE_STATIC_KEY_FALSE(zenith_render_aware_key);
static DEFINE_STATIC_KEY_FALSE(zenith_psi_aware_key);
static DEFINE_STATIC_KEY_FALSE(zenith_game_auto_key);
static DEFINE_STATIC_KEY_FALSE(zenith_auto_tune_v3_key);
static DEFINE_STATIC_KEY_FALSE(zenith_thermal_aware_key);
/* Patch K: master gate for game_perf_burst.  Defaults TRUE because
 * the matching scalar tunables->game_perf_burst defaults to 1 (the
 * user requested "all automatic"); zenith_init() syncs the key to
 * the scalar at attach time exactly as audio_aware / render_aware /
 * etc. above.  When the master is 0 the static branch costs nothing
 * on the hot path -- the FSM evaluator and floor application both
 * sit inside ZENITH_FEATURE_ENABLED(game_perf_burst) blocks.
 */
static DEFINE_STATIC_KEY_FALSE(zenith_game_perf_burst_key);

/* Transition invariant for the six feature static keys above:
 *
 *   tunables->X (sysfs-visible scalar)  ==  static-key state of zenith_X_key
 *
 * is established by every *_store callback below using
 *
 *   t->X = val;
 *   zenith_set_static_key(&zenith_X_key, val);
 *
 * in that order — store the scalar first, then sync the key.  All
 * stores run under attr_set->update_lock (held by the gov_attr
 * dispatcher), so the two writes are serialised against each other
 * and against any other store on the same attr_set.
 *
 * The hot path reads the static key (single never-taken jump while
 * the feature is off), not the scalar, so a momentary tear between
 * the two writes can at worst cause one tick of the get_next_freq()
 * fast path to take the wrong branch.  Acceptable: feature flips are
 * rare (sysfs writes from system_server / init shell only), the wrong
 * branch is itself benign (returns or skips a tier), and the next
 * tick will see the consistent state.
 *
 * Implications for anyone adding a new feature key here:
 *   - The init state of every DEFINE_STATIC_KEY_FALSE is FALSE.  If
 *     the matching scalar defaults to a non-zero value, the key must
 *     be explicitly enabled at init time (after tunables defaults are
 *     written), otherwise the hot path will read the scalar as 1 but
 *     skip the static-branch body.  zenith_init() syncs the
 *     audio_aware / render_aware / camera_aware / psi_aware /
 *     game_auto / auto_tune_v3 keys against their default scalars
 *     for exactly this reason.
 *   - Profile presets in zenith_apply_profile() must not silently
 *     toggle a feature scalar without also calling
 *     zenith_set_static_key(); doing so violates the invariant.  The
 *     four current presets (perf/balanced/battery/legacy) deliberately
 *     leave audio_aware / camera_aware / render_aware / psi_aware
 *     untouched for exactly this reason — they are user-managed
 *     opt-ins, not preset state.
 *   - static_branch_enable / static_branch_disable both sleep
 *     acquiring cpus_read_lock() but are safe from sysfs store
 *     context.  Do not call zenith_set_static_key() from the hot path
 *     or from any context that holds a spinlock; both will deadlock.
 */
static inline void zenith_set_static_key(struct static_key_false *key,
					 bool enable)
{
	if (enable)
		static_branch_enable(key);
	else
		static_branch_disable(key);
}

#define ZENITH_FEATURE_ENABLED(name)	\
	static_branch_likely(&zenith_##name##_key)

#define ZENITH_DEFAULT_CLIMB_MODE		ZENITH_CLIMB_MODE_SNAP
#define ZENITH_DEFAULT_FREQ_STEP_PCT		5

/* freq_step_adaptive (default 0, off):
 *
 * When STEP climb mode is selected, the per-sample step is a fixed
 * fraction of policy->max (freq_step_pct) regardless of how far
 * load_pct has overshot up_threshold.  A sample at load_pct = 76%
 * with up_threshold = 75% produces the same step as a sample at
 * load_pct = 99% -- the latter clearly wants to converge faster.
 *
 * When set to 1, the base step is scaled by (100 + overshoot)% where
 * overshoot is 100 * (load_pct - up_threshold) / (100 - up_threshold),
 * so:
 *   - At load_pct == up_threshold: 1.0x base step (no change)
 *   - At load_pct == 100:           2.0x base step (double)
 *   - Linearly interpolated in between.
 *
 * Only affects STEP climb mode; SNAP mode is load-independent by
 * design and unchanged.  Preserves a minimum step of 1 (same guard
 * as the base path) so a zero freq_step_pct cannot stall the climb.
 */
#define ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE	0
#define ZENITH_DEFAULT_THERMAL_AUTO		1
#define ZENITH_THERMAL_AUTO_PRESSURE_PCT	10
/*
 * thermal_aware: master gate for the cluster of thermal mechanisms
 * the governor exposes as separate tunables.  When 1 (default), the
 * gated mechanisms are active subject to their own per-mechanism
 * tunables; when 0, all of the following short-circuit to no-ops
 * regardless of their per-mechanism switch:
 *
 *   - thermal_util_derate (level term in zenith_get_util())
 *   - thermal_derate_rate_pct (slope term, lives inside the same
 *     thermal_util_derate block, so the master gate covers it)
 *   - the thermal_pressure_continuous up_thresh ramp inside the
 *     screen-off / zenith_thermal_active() path (gated indirectly
 *     because zenith_thermal_active() short-circuits to false when
 *     the master gate is off)
 *   - auto_thermal_cap (target_freq cap in zenith_get_next_freq())
 *   - the auto_tune_v2 THERMAL_RECOVERY state transitions
 *
 * Default 1 to preserve the current shipping behaviour: every
 * mechanism whose individual tunable is on stays on without any
 * userspace flip.  Audited 2026-05-07 (zenith-tunables-audit) as the
 * single naming/master-gate cleanup that lets userspace turn off all
 * thermal-driven freq adjustment at once for benchmarking, captures,
 * or thermal-test rigs without having to know the names of every
 * thermal sub-tunable.  Strict 0/1 boolean.  Static-key gated for
 * branchless cost when on; sysfs store calls
 * zenith_set_static_key() to keep the key state in sync with the
 * scalar.
 */
#define ZENITH_DEFAULT_THERMAL_AWARE		1
/*
 * thermal_pressure_continuous default flipped from 0 to 1 in the
 * wave-2 auto-defaults round.  The legacy hard-cliff path snapped
 * dynamic_up_thresh to 90% the moment thermal_state turned on; the
 * continuous path linearly ramps from up_threshold (at 0% pressure)
 * to 90% (at 100% pressure) using the same arch_scale_thermal_pressure
 * percentage that V2 consumes.  No KMI exposure; the runtime path is
 * gated on thermal_auto and a non-zero pressure reading.
 */
#define ZENITH_DEFAULT_THERMAL_PRESSURE_CONTINUOUS	1

/* prefer_silver_aware defaults.  See struct zenith_tunables for
 * semantics.  These fields remain as dead storage -- the Kconfig was
 * removed and nothing updates ps_hit_rate_pct, so the runtime path
 * never fires.
 *
 * Both knobs are tunable; the defaults are conservative.
 */
#define ZENITH_DEFAULT_PREFER_SILVER_AWARE			1
#define ZENITH_DEFAULT_PREFER_SILVER_HOT_THRESHOLD_PCT		50
#define ZENITH_DEFAULT_PREFER_SILVER_HOT_BUMP_PCT		5
#define ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT			20

/* brutal_decay_ms upper bound.  500ms is generous: a longer window
 * is functionally indistinguishable from "no cliff exit" because
 * the underlying EAS / load signal will move the floor anyway.
 */
#define ZENITH_DEFAULT_BRUTAL_DECAY_MS				0
#define ZENITH_BRUTAL_DECAY_MS_MAX				500

/* thermal_util_derate (default 1, on):
 *
 * When set, zenith_get_util() scales down its output by the
 * fraction of capacity currently being eaten by SoC thermal
 * pressure (arch_scale_thermal_pressure(cpu) / arch_scale_cpu_capacity(cpu)).
 *
 * Without this, util keeps demanding 100% of the *thermal-throttled*
 * max, which causes the governor to pin policy->max while the
 * thermal framework drops the cap.  The result is a continuous
 * yo-yo: thermal lowers max -> we still pin max -> SoC stays hot ->
 * thermal lowers further.  Derating util smooths this loop because
 * we ask for less than the throttled max, giving the SoC a chance
 * to cool.
 *
 * Concretely, with 25% of capacity thermal-pressured, util is
 * scaled by (cap - pressure) / cap = 0.75.  A util of 800/1024
 * becomes 600/1024.  Frequency selection then targets the
 * derated demand instead of clamping to the (already throttled)
 * max.
 *
 * Only applies the scale when at least
 * ZENITH_THERMAL_DERATE_FLOOR_PCT of the capacity is pressured;
 * for sub-threshold pressure the cost of the multiply isn't
 * worth the precision.
 *
 * Tunable-gated 0/1.  Default 1 (on) -- the existing thermal_auto
 * tier already handles the "throttle hard" case, but it doesn't
 * smooth the dance.  This tier adds the smoothing.
 */
#define ZENITH_DEFAULT_THERMAL_UTIL_DERATE	1
#define ZENITH_THERMAL_DERATE_FLOOR_PCT		5

/* thermal_derate_rate_pct (default 0, off):
 *
 * The static thermal_util_derate scales util by the *current*
 * pressure level, which lags the actual thermal event: by the time
 * pressure has risen to the level where the derate bites, the
 * cluster has already spent the rising slope at full demand.  On
 * SoCs with fast pressure tracking (e.g. tsensor-driven thermal
 * frameworks) this is visible as an overshoot before the derate
 * catches up.
 *
 * thermal_derate_rate_pct adds a derivative term: when pressure
 * is rising sample-to-sample on a cpu, additionally scale util by
 * up to thermal_derate_rate_pct percent based on how big the
 * single-step rise was relative to capacity.  Same shape as the
 * level derate -- just on the slope instead of the value.  Caps
 * the additional reduction at thermal_derate_rate_pct so a single
 * pressure spike can never zero util.
 *
 *   rise_pct = ((pressure - prev_pressure) * 100) / max  // 0..100
 *   if (rise_pct > thermal_derate_rate_pct)
 *           rise_pct = thermal_derate_rate_pct;
 *   util_out = util_out * (100 - rise_pct) / 100;
 *
 * Only applied when the static thermal_util_derate also fires
 * (pressure >= ZENITH_THERMAL_DERATE_FLOOR_PCT and pressure < max),
 * so the rate term piggybacks on the existing gating and adds no
 * cost when the level derate is silent.  prev_pressure is per-cpu
 * and zero-initialised by zenith_start()'s memset, so the first
 * sample after attach sees rise_pct == 0 (no derivative kick on
 * cold start).
 *
 * 0 disables the derivative term entirely; the level derate is
 * unaffected.  Range 0..100; values >100 rejected by sysfs.  No
 * upper cap on the rate of rise itself -- the rate_pct clamp at
 * thermal_derate_rate_pct is the only bound that matters for the
 * output.
 */
#define ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT	25

/* auto_thermal_cap (default 0, off):
 *
 * Final-stage hard cap on target_freq when the per-policy thermal
 * pressure (arch_scale_thermal_pressure() expressed as a
 * percentage of capacity) is sustained at or above
 * auto_thermal_cap_pressure_pct.  Layered AFTER the level /
 * derivative util_derate paths and the V2 THERMAL_RECOVERY state
 * machine, this tier acts as an absolute upper bound on freq.
 * Workloads that race past those mechanisms (input_boost,
 * peak_headroom_rescue, frame_overrun, dl_task_floor) cannot pin
 * policy->max indefinitely once thermal pressure exceeds the
 * configured threshold while this gate is on.
 *
 * Default is OFF (=0) so existing tunings are unchanged on
 * upgrade.  Operators that observe sustained thermal climbs in
 * spite of the V2 state machine flip auto_thermal_cap=1 per policy
 * and tune the threshold / cap pair to taste:
 *
 *   echo 1 > /sys/devices/system/cpu/cpufreq/policy0/zenith/\
 *           auto_thermal_cap
 *   echo 50 > .../auto_thermal_cap_pressure_pct  (fire at >= 50%%)
 *   echo 80 > .../auto_thermal_cap_freq_pct      (cap to 80%% of max)
 *
 * Pressure threshold is bounded ZENITH_AUTO_THERMAL_CAP_PRESSURE_-
 * PCT_{MIN,MAX} (1..100); freq cap is bounded ZENITH_AUTO_-
 * THERMAL_CAP_FREQ_PCT_{MIN,MAX} (50..100) so an accidental
 * "= 0" cannot zero the cluster.
 *
 * The cap is applied AFTER em_cap (Step 6) so the EM ladder still
 * has a chance to validate the resolved freq against the energy
 * model; auto_thermal_cap then clamps to the smaller of (em_cap
 * result, policy->max * auto_thermal_cap_freq_pct / 100).
 *
 * tp_path = "auto_thermal_cap" when the cap fires.  Counted in
 * ZENITH_STAT_AUTO_THERMAL_CAP via zenith_path_to_bucket() and
 * surfaced as auto_thermal_cap=N in the zenith_stats sysfs node.
 *
 * No KMI exposure (governor-private sysfs).  The runtime path is
 * gated on the boolean tunable; when 0 it short-circuits before
 * any pressure read, costing a single READ_ONCE per call.
 */
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP			0
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP_PRESSURE_PCT	50
#define ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MIN	1
#define ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MAX	100
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP_FREQ_PCT	80
#define ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MIN		50
#define ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MAX		100

/* freq_stability_margin_pct (default 3):
 *
 * When the resolved target_freq is within margin percent of policy->max
 * below the currently-requested frequency, keep the current request
 * instead of issuing a tiny downward transition.  This removes
 * bin-boundary oscillation where the target dances just below an OPP
 * edge and pays regulator / PLL transition cost for no perceptible
 * gain.  Upward transitions are never suppressed.
 *
 * 0 disables the margin entirely (legacy: every target != current
 * request can switch subject only to the rate limiter).  Range 0..10;
 * values above 10 are aggressive enough to feel sticky.
 */
#define ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT	3
#define ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX		10

/* down_rate_adaptive (default 1, on):
 *
 * Scales the effective down_rate_delay_ns by the recent load variance
 * EWMA.  Bursty workloads get up to 2x the base delay, keeping freq
 * elevated between adjacent frame/render bursts; steady workloads use
 * the configured delay unchanged.
 *
 *   eff_delay = base_delay * (256 + min(var, 256)) / 256
 *
 * 0 disables the scaling and restores the fixed down-rate delay.
 */
#define ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE	1

/* wakeup_boost (default 1, on):
 *
 * Detects idle-to-busy transitions where util jumps from below 10% to
 * at least 40% of capacity in one scheduler callback.  On detection,
 * the next two upward transitions bypass up_rate_limit so app launch,
 * screen-on and notification wakeups avoid the cold-start sample lag.
 *
 * 0 disables the detector.  The thresholds are expressed as capacity
 * percentages so they scale across ARM DynamIQ clusters.
 */
#define ZENITH_DEFAULT_WAKEUP_BOOST		1
#define ZENITH_WAKEUP_IDLE_THRESH_PCT		10
#define ZENITH_WAKEUP_BUSY_THRESH_PCT		40
#define ZENITH_WAKEUP_BOOST_TICKS		2

/* wakeup_boost_ms upper bound.  200ms is past the perceptible
 * threshold for a wakeup transition; longer windows just bleed
 * into the steady-state climb logic and waste battery.
 */
#define ZENITH_DEFAULT_WAKEUP_BOOST_MS		0
#define ZENITH_WAKEUP_BOOST_MS_MAX		200

/* down_threshold_adaptive (default 0, off):
 *
 * Mirrors up_threshold_adaptive on the brutality exit side.  When set
 * to N (1..20), the effective down_threshold is lowered by up to N
 * percent under bursty load, widening the hysteresis band so max freq
 * holds through frame bursts.  Steady load leaves the configured
 * down_threshold unchanged.
 *
 * 0 disables the adjustment.
 */
#define ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE	0
#define ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX	20

/* rate_limit_cluster_scale (default 1, on):
 *
 * Applies asymmetric cached rate limits to little-cluster policies on
 * heterogeneous SoCs: up_rate_delay_ns is doubled and
 * down_rate_delay_ns is halved.  Big / prime clusters keep the
 * configured limits.  On homogeneous SoCs every policy is treated as a
 * big cluster and this is a no-op.
 *
 * 0 disables the per-cluster scaling.
 */
#define ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE	1
#define ZENITH_CLUSTER_LITTLE_THRESH_PCT	60

#define ZENITH_DEFAULT_UP_RATE_LIMIT_US		100
#define ZENITH_DEFAULT_DOWN_RATE_LIMIT_US	4000
/* Defensive sysfs upper bound for up_rate_limit_us and down_rate_limit_us
 * (microseconds).  60 seconds is well past any sane cpufreq sampling
 * cadence; the goal is to reject UINT_MAX-style garbage at the sysfs
 * layer rather than to impose a tight policy.
 */
#define ZENITH_RATE_LIMIT_US_MAX		60000000U

/*
 * Boot-time hardware calibration.  Reads the cpufreq driver's
 * transition_latency at attach time and adjusts the default rate
 * limits so the governor never requests freq changes faster than
 * the hardware can deliver them.
 *
 * zenith_calibrate_enable:  0/1, default 1.  When on, the first
 *   policy to attach runs the calibration and caches the result;
 *   every subsequent attach reads the cached calibrated values.
 *
 * The calibration formula:
 *   calibrated_up_us   = max(transition_latency_ns / 500, 100)
 *   calibrated_down_us = max(calibrated_up_us * 40, 4000)
 *
 * On a typical SoC with 200 us transition latency this yields
 * up=400 / down=16000; on a fast SoC with 50 us latency it
 * yields the existing defaults (up=100 / down=4000).
 */
static unsigned int zenith_calibrated_up_us __read_mostly =
	ZENITH_DEFAULT_UP_RATE_LIMIT_US;
static unsigned int zenith_calibrated_down_us __read_mostly =
	ZENITH_DEFAULT_DOWN_RATE_LIMIT_US;
static unsigned int zenith_calibrate_enable __read_mostly = 1;

/* Multiplier for the effective down-rate delay while an input
 * boost is active.  Range 100..1000 (percent).  100 = no extension
 * (legacy behaviour); 200 (the default) = double the down-rate
 * delay during the input_boost full-pin window; 500 = quintuple.
 *
 * Rationale: within the input_boost_until_ns window the cluster is
 * pinned high precisely because the user is actively interacting.
 * Letting the down-rate gate fire at its normal cadence inside
 * that window pulls the cluster off peak the moment a sample's
 * load proportional math drops below the previous freq -- which
 * happens between every render tick and the next on a typical
 * scroll / swipe -- producing the "post-tap cliff drop" pattern
 * users describe as stutter even though the user is still
 * interacting.  Multiplying down_rate_delay during the boost
 * window holds the cluster up across the gaps without changing
 * any other tier.
 *
 * Self-disarms: once now >= input_boost_until_ns the multiplier
 * disappears on the very next call into zenith_up_down_rate_limit,
 * so the steady-state idle path is unaffected.  Capped at 1000%%
 * (10x) on store so a runaway value can not effectively pin the
 * cluster forever.
 */
#define ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT	200
#define ZENITH_INPUT_BOOST_DOWN_RATE_MULT_PCT_MAX	1000
#define ZENITH_DEFAULT_POWERSAVE_BIAS		0

/* Screen-on softening of powersave_bias.  Default 50: halve the
 * effective bias whenever the screen is on (tunables->screen_state
 * == 1), leaving the configured value in full effect on screen-off
 * and on the thermal-active path.  Rationale: powersave_bias
 * values inherited from the BALANCED (50, i.e. 5 %% shave) and
 * BATTERY (150, 15 %% shave) profiles are tuned for the average
 * over screen-on + screen-off duty cycles, but the screen-on path
 * itself is the single most responsiveness-sensitive window the
 * cluster has.  Halving the screen-on shave keeps the configured
 * profile's screen-off behaviour intact (where the existing 500 /
 * 50 %% screen-off override already dominates), while removing
 * about half of the steady-state shave that's been quietly
 * suppressing the cluster's peak ramp during interactive use.
 *
 * Range 0..100 (percent).  100 = no softening (legacy behaviour);
 * 50 = halve; 0 = zero out the bias entirely on screen-on.
 */
#define ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT	50
#define ZENITH_DEFAULT_IO_IS_BUSY		1
#define ZENITH_DEFAULT_INPUT_BOOST_MS		60
#define ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS	30

/* input_boost_touchdown_extra_ms (default 50, [Stage 4 / Patch C]):
 *
 * Touchdown vs coordinate-stream differentiation.  An EV_KEY/
 * BTN_TOUCH press (touchdown) is the user's "I just started
 * interacting" signal: latency from touchdown to first frame is
 * the visible feel of the device.  An EV_ABS coordinate stream
 * mid-gesture is "I'm already interacting" -- the cluster should
 * already be on a high tier from the touchdown that started the
 * gesture, so a per-coordinate widen-the-window effort is wasted
 * energy.
 *
 * This knob extends the input-boost active window by an extra
 * input_boost_touchdown_extra_ms milliseconds *only* on the
 * touchdown event.  Coordinate-stream EV_ABS events use the
 * unmodified input_boost_ms window plus the existing quiet-period
 * extension (see ZENITH_INPUT_QUIET_BOOST_MULT_PCT).
 *
 * 0 disables the touchdown extra entirely (legacy behaviour:
 * touchdown gets the same window as a coordinate event).  Capped
 * at ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX so a runaway echo
 * can't accidentally pin the cluster up for seconds.
 */
#define ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS	50
#define ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX	500

/* input_boost_decay_curve (default 0, linear):
 *
 * The input-boost decay path lowers a synthetic floor from the full
 * boost ceiling down to policy->min across input_boost_decay_ms.
 * Until now that ramp was strictly linear:
 *
 *   floor = ceiling - span * elapsed / decay_ns        (LINEAR)
 *
 * which drops fast at the start of the tail and slows near the
 * end -- the opposite of what a gesture actually wants.  The hand
 * leaves the screen, the compositor has already committed one or
 * two post-gesture frames at high freq, and we want the floor to
 * hold high for a short moment (keeping the render thread on a
 * comfortable bin for the settle frames) and then drop off
 * quickly at the end of the decay window.
 *
 * When set to 1 (CUBIC), the normalised elapsed time is cubed
 * before being consumed:
 *
 *   t256  = elapsed * 256 / decay_ns                   (0..256)
 *   cubic = t256^3 / 65536                             (0..256)
 *   floor = ceiling - span * cubic / 256               (CUBIC)
 *
 * At elapsed == 0 both curves give floor == ceiling; at
 * elapsed == decay_ns both give floor == policy->min.  The
 * midpoint differs sharply: LINEAR has dropped 50 %% at the
 * halfway mark, CUBIC has dropped ~12.5 %%.  The full-boost phase
 * (while remaining > decay_ns) is unaffected.
 *
 * All arithmetic is u32-bounded by design: t256 is capped at 256,
 * its cube at 16,777,216, the final per-step divisor fits a u32.
 */
#define ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE	0
#define ZENITH_DEFAULT_INPUT_BOOST_BIG_ONLY	1

/* Input-boost quiet-period extension (always-on, compile-time
 * constants).  When zenith_input_event observes that the gap since
 * the previous input event exceeds ZENITH_INPUT_QUIET_THRESHOLD_MS,
 * the next event's full-pin window is widened by
 * ZENITH_INPUT_QUIET_BOOST_MULT_PCT (>= 100) and clamped at
 * ZENITH_INPUT_QUIET_BOOST_MAX_MS so a misconfigured input_boost_ms
 * cannot grow the window without bound.
 *
 * Rationale: the very first tap / key / scroll after the user has
 * been idle (reading, watching a static frame) is the single
 * highest-leverage responsiveness window the governor has.  By the
 * second tap of a sustained interaction the cluster is already
 * pinned by either input_boost or the load-driven hispeed tier; the
 * marginal value of an extra-long boost on each subsequent tap is
 * small.  Extending only the first-after-quiet event keeps the
 * average power impact tiny while making the "device feels slow when
 * I pick it up" failure mode go away.
 *
 * QUIET_THRESHOLD_MS = 1000: anything shorter than this and the
 * existing input_boost_ms / input_boost_decay_ms windows already
 * cover the gap.  At 1 second of no input, even an input_boost_ms
 * of 200 ms (the maximum sane value) has fully decayed and the
 * cluster is back on natural shaping, so the next event genuinely
 * is a "wake-from-quiet" event.
 *
 * MULT_PCT = 200: doubles the active phase.  Conservative; could be
 * higher but doubling hits a clear "whole-frame" extra (16 ms at
 * 60 Hz on top of an 80 ms default = ~6 frames worth) without
 * spending energy beyond the natural decay tail.
 *
 * MAX_MS = 250: hard ceiling.  Guards against the extension scaling
 * an input_boost_ms that has been raised by userspace beyond a sane
 * range.  Past 250 ms the input_boost_decay_ms window dominates the
 * energy bill anyway, so capping the extension here costs nothing.
 */
#define ZENITH_INPUT_QUIET_THRESHOLD_MS		1000
#define ZENITH_INPUT_QUIET_BOOST_MULT_PCT	200
#define ZENITH_INPUT_QUIET_BOOST_MAX_MS		250

/* ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT controls the ceiling of the
 * full-pin phase of an active input boost: the first input_boost_ms
 * after a key / touch event.  0 means "no cap" -- pin to
 * policy->max for the duration of the full-pin phase, then decay
 * across input_boost_decay_ms back to policy->min.  Tester reports
 * of "device runs cold and never reaches peak frequency under
 * sustained interactive load (gameplay touch, fast scrolling)" trace
 * back to a non-zero cap eating the top of the cluster's range
 * during the very window where the user is actively asking for it.
 *
 * Default 80 (cap tap-boost at 80% of max): the battery-sane
 * default keeps the tap-boost tail from pinning to policy->max on
 * every touch; the remaining 20% headroom costs almost nothing on
 * light taps while sustained gameplay is driven by the classifier
 * and hispeed tiers, which are not capped here.  Userspace setpoints,
 * the BALANCED / BATTERY profiles, and per-policy local overrides via
 * profile_values can still cap the ceiling lower for power-sensitive
 * configurations.  The full-pin phase is short (default 60 ms) and
 * the trailing decay phase is shorter still (default 30 ms), so
 * "pin to the cap on every input event" is bounded in time and
 * downstream caps (uclamp_max, audio_cap, em_cap, light_cap,
 * thermal_state) all apply on top.
 */
#define ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT	80	/* cap tap-boost at 80% of max */
#define ZENITH_DEFAULT_EFFICIENT_FREQ		0

/* eff_bin_hyst_pct (default 0, off):
 *
 * The efficient_freq ladder releases a bin (and resets every higher
 * bin's wait-deadline) the moment the requested target_freq drops
 * to or below the bin's freq.  When userspace load sits right at a
 * bin boundary -- a 6 Hz frame-pacing thread asking for almost
 * exactly the bin freq -- this turns into ping-pong:
 *
 *   eval N   target = bin_freq + 1    arm bin, hold at bin_freq
 *   eval N+1 target = bin_freq        clear deadline, break
 *   eval N+2 target = bin_freq + 1    re-arm bin (full delay again)
 *   ...
 *
 * The bin never actually unlocks because the deadline keeps getting
 * reset.  The cluster sits one rung below where it should be.
 *
 * eff_bin_hyst_pct adds a release margin per bin: target must drop
 * to bin_freq * (100 - eff_bin_hyst_pct) / 100 before the deadline
 * is cleared.  Targets between that release threshold and bin_freq
 * land in a hysteresis band: the bin is *not* released (deadline is
 * preserved, so a re-cross doesn't have to re-arm) but the target
 * is also *not* held at bin_freq (so the operator gets the slightly
 * lower freq they asked for).  This breaks the ping-pong without
 * pinning the cluster up to the bin.
 *
 * Range 0..20.  20% is a generous upper bound -- bin spacing in
 * real freq tables tends to be larger than that, so a value of 20
 * gives the full hysteresis band; values higher would just clip to
 * the bin below.  0 disables the band entirely (legacy: any drop
 * to-or-below bin_freq releases the deadline).
 */
#define ZENITH_DEFAULT_EFF_BIN_HYST_PCT		0
#define ZENITH_EFF_BIN_HYST_PCT_MAX		20
#define ZENITH_DEFAULT_UP_DELAY_US		4000
#define ZENITH_DEFAULT_LIGHT_LOAD_FREQ		0
/* Defensive sysfs upper bound for light_load_freq (kHz).  50 GHz is well
 * past any real CPU; rejects UINT_MAX-style garbage at the sysfs layer.
 */
#define ZENITH_LIGHT_LOAD_FREQ_MAX		50000000U
#define ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD	20
#define ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR	2
#define ZENITH_MAX_SAMPLING_DOWN_FACTOR		10
#define ZENITH_DEFAULT_BOOST_EXIT_EXTEND	1	/* stretch down-rate after a boost ends */
#define ZENITH_DEFAULT_BIAS_LOAD_THRESHOLD	50

/* auto_tune classifier thresholds. Exposed as tunables so userspace can
 * tune what the observer considers "saturated" and the saturation /
 * input-event cutoffs that select performance vs battery.
 */
#define ZENITH_DEFAULT_AT_SAT_LOAD_PCT		80
#define ZENITH_DEFAULT_AT_HI_SAT_PCT		60
#define ZENITH_DEFAULT_AT_LO_SAT_PCT		25
#define ZENITH_DEFAULT_AT_HI_EVENTS_X2		4
#define ZENITH_DEFAULT_AT_LO_EVENTS_X2		2

/* auto_tune_scenario (default 0, off):
 *
 * When auto_tune=1 (the existing classifier worker is running) and
 * auto_tune_scenario=1, zenith_auto_tune_work() also samples the
 * detected scenario at classification time -- audio-thread enqueued,
 * camera HAL active (via the camera_active override or the comm
 * walk), render-thread active, and PSI memory-stall above
 * psi_mem_thresh -- and lets that scenario *override* the
 * load-saturation + input-rate target the vanilla classifier picks:
 *
 *   camera | render -> ZENITH_PROFILE_PERFORMANCE
 *   memstall (and not camera/render) -> ZENITH_PROFILE_BATTERY
 *   audio (and not camera/render/memstall) -> ZENITH_PROFILE_BALANCED
 *   no scenario -> classifier output unchanged
 *
 * The intent is the obvious one: when the device is actively
 * filming or driving a render pipeline, the user almost certainly
 * wants PERFORMANCE regardless of what the input-rate classifier
 * thinks; when it's purely playing audio in the background, BALANCED
 * is the natural floor (BATTERY would risk underrun); when memory
 * pressure is high the cycles wasted on stalls aren't worth the
 * energy.  The scenarios have a strict precedence (camera/render
 * beats memstall beats audio) to keep behaviour deterministic.
 *
 * Detection at classification time is a snapshot, not a window
 * average -- the comm walk caches (4 ms TTL each) reflect what's
 * running at the auto_tune sample point.  In practice that catches
 * the common case of "user opened the camera 8 seconds ago" cleanly
 * because the cameraserver / mtkcam-* processes stay enqueued.  For
 * scenarios that have already wound down by the moment we sample,
 * the classifier still picks via load and input-rate.
 *
 * Requires auto_tune=1.  Independent of audio_aware / camera_aware /
 * render_aware: the comm walks fire even when those gating flags
 * are 0, because here we're using them as detection signals, not as
 * floor/cap policy.  No KMI exposure.
 *
 * Default flipped from 0 to 1 in the wave-1 auto-defaults round so
 * V1-only builds (auto_tune_v2=0) also benefit from scenario-aware
 * profile selection.  Floor/cap behaviour is still off by default
 * because audio_floor_pct, render_floor_pct and camera_floor_pct
 * stay at 0 -- only the profile-bias path is enabled.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_SCENARIO	1

/* auto_tune_v2 safety layer (default 1, on):
 *
 * The legacy auto_tune path applies whole profile presets from one
 * classification window.  V2 keeps the same observer but adds a
 * bounded state machine: candidate targets must repeat for a small
 * hysteresis window, profile-specific guardrails clamp every automatic
 * write, and user-written knobs are skipped until the operator resets
 * overrides or changes profile.
 *
 * 0 preserves the legacy classifier path.  1 enables the bounded V2
 * state/actions without changing KMI or tracepoint ABI.
 *
 * Default flipped from 0 to 1 in the wave-1 auto-defaults round.  V2
 * only adjusts knobs whose user-set value is the per-knob default;
 * any operator who has pinned a value via sysfs continues to win
 * outright.  hysteresis_windows + cooldown_windows + override_mask
 * remain in place to bound state-thrash.  Set the knob back to 0 in
 * init.zenith.rc to lock the legacy classifier path.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V2		1
#define ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS	2
#define ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS	1
#define ZENITH_AT_HYSTERESIS_WINDOWS_MAX	8
#define ZENITH_AT_COOLDOWN_WINDOWS_MAX		8

/* Variance-promotion threshold (load_var_ewma_x256, units of 1/256
 * of the squared-deviation EWMA reported by the V1 classifier).
 * When the ewma is at-or-above this value AND the V2 state machine
 * is currently in LATENCY, V2 promotes to SUSTAINED_PERF on the
 * variance signal alone (reason=variance) -- the rationale is that
 * an oscillating workload spends too much time crossing thresholds
 * for LATENCY's per-tick reaction; SUSTAINED_PERF holds the freq
 * up across the bursts.
 *
 * 768 was chosen empirically as a good knee for phone workloads:
 * background music (steady) sits ~200, app launch / scroll
 * (medium burst) ~500, camera viewfinder / 3D ~900-1500.  Lower
 * values promote eagerly and risk holding sustained_perf longer
 * than necessary; higher values delay promotion and risk under-
 * frequency on bursty workloads.
 *
 * Tunable from userspace via auto_tune_v2_var_promote_thresh sysfs;
 * 0 disables variance-driven promotion entirely.
 */
#define ZENITH_DEFAULT_AT_V2_VAR_PROMOTE_THRESH	768
#define ZENITH_AT_V2_VAR_PROMOTE_THRESH_MAX	65535U

/* F2: PELT util-rising trend default.  25% window-to-window growth
 * is conservative -- a cold app launch typically pushes 50-100%, a
 * web first-paint 30-60%.  Setting to 0 disables the signal cleanly
 * (the V1 worker computes the delta but never raises the flag).
 */
#define ZENITH_DEFAULT_AT_UTIL_RISING_THRESH_PCT	25
#define ZENITH_AT_UTIL_RISING_THRESH_PCT_MAX	200U

/* F3: render-thread RT-priority floor (per-policy uclamp-min-style).
 *
 * When auto_tune_render_rt_floor_pct > 0 AND the V2 state machine
 * has committed to LATENCY or SUSTAINED_PERF AND ZENITH_AT_FLAG_RENDER
 * is currently active in at_last_flags, raise the freq floor to
 * (policy->max * auto_tune_render_rt_floor_pct / 100).  This is the
 * lower-risk variant of the original F3 design: instead of touching
 * task->sched_class via sched_setscheduler_nocheck() (which would
 * collide with audio_server's RT-priority inheritance trees), we
 * publish a per-policy uclamp-min-style hint that simply guarantees
 * RenderThread / surfaceflinger sees enough freq headroom to run
 * uninterrupted by background CFS tasks while V2 says the workload
 * is responsiveness-critical.
 *
 * Difference from render_floor_pct: render_floor_pct fires whenever
 * a render thread is observed running, regardless of V2 state.  The
 * F3 floor fires only after V2 has *already* committed to LATENCY
 * or SUSTAINED_PERF, so the user has signalled "this is jank-
 * sensitive workload".  In that regime, even a single CFS preemption
 * of a RenderThread is visible as a frame stutter; the floor adds
 * the headroom that makes the preemption window survivable.
 *
 * Default 0 (off, conservative).  Range 0..100; 0 disables the floor
 * cleanly.  Recommended user value if enabling: 85..95.  The floor
 * is OR-ed with (max of) the existing render_floor_pct floor; if
 * F3 is on and conditions fire, F3 always wins.
 */
#define ZENITH_DEFAULT_AT_RENDER_RT_FLOOR_PCT	0
#define ZENITH_AT_RENDER_RT_FLOOR_PCT_MAX	100U

/* auto_tune_v3 (default 2, apply):
 *
 * Self-calibrating layer on top of V2.  Reads the per-policy at_log
 * ring (ZENITH_AT_LOG_NR entries, each one V1-window wide) once per
 * ZENITH_AT_V3_INTERVAL_NS and counts V2 state transitions inside
 * the window.  Based on observed transition rate, V3 maintains
 * bounded signed offsets to two V2 reaction knobs:
 *
 *   - auto_tune_hysteresis_windows  (ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS = 2)
 *   - auto_tune_cooldown_windows    (ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS  = 1)
 *
 * If V2 was observed thrashing (transitions >= ZENITH_AT_V3_THRASH_HI),
 * the offsets bump up by one (more hysteresis, slower reaction).  If
 * V2 was observed sticky (transitions <= ZENITH_AT_V3_THRASH_LO), the
 * offsets bump down by one (less hysteresis, faster reaction).  Either
 * way the offset is clamped to [ZENITH_AT_V3_OFFSET_MIN ..
 * ZENITH_AT_V3_OFFSET_MAX] and the resulting eff_value is clamped to
 * the existing ZENITH_AT_HYSTERESIS_WINDOWS_MAX / _COOLDOWN_WINDOWS_MAX
 * caps and to a >=1 floor.
 *
 * Three modes via the auto_tune_v3 scalar:
 *
 *   - 0  off                  -- no observation, no adjustments.
 *   - 1  observe-only         -- collects stats, exposes them via
 *                                auto_tune_v3_state, does NOT apply
 *                                offsets.  Equivalent to a dry-run.
 *   - 2  observe + apply      -- collects stats AND applies the
 *                                bounded offsets to the V2 reaction
 *                                knobs.  This is the wave-7 default.
 *
 * Default 2 (apply, wave-7 round): once per
 * auto_tune_v3_interval_ms (default 60 s) the calibrator nudges
 * V2's effective hysteresis/cooldown windows toward whatever fits
 * the live workload.  Bounded offsets ([-1, +4]) and a >=1 floor on
 * the resulting effective value mean V3 cannot push V2 into a
 * state-change-impossible configuration even at the worst extreme.
 * The scalar is gated by the zenith_auto_tune_v3_key static branch
 * (FALSE while scalar = 0) so the calibration tail in
 * zenith_auto_tune_work() collapses to a single never-taken jump
 * per V1 window when an operator turns the feature off.
 *
 * Init-time invariant: the static key zenith_auto_tune_v3_key must
 * be synced TRUE in zenith_init() against the default-non-zero
 * scalar (same pattern as wave-2 audio_aware / render_aware).  See
 * the DEFINE_STATIC_KEY_FALSE comment block.
 *
 * Tunable surface:
 *   - auto_tune_v3              RW 0/1/2  master gate / mode
 *   - auto_tune_v3_state        RO        snapshot of current
 *                                         observed transitions/window
 *                                         and the two live offsets
 *   - auto_tune_v3_interval_ms  RW        calibration period (default
 *                                         60000, clamped to [10000,
 *                                         600000])
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V3		2
#define ZENITH_AT_V3_MODE_OFF			0
#define ZENITH_AT_V3_MODE_OBSERVE		1
#define ZENITH_AT_V3_MODE_APPLY			2
#define ZENITH_AT_V3_MODE_MAX			2

#define ZENITH_DEFAULT_AT_V3_INTERVAL_MS	60000
#define ZENITH_AT_V3_INTERVAL_MIN_MS		10000
#define ZENITH_AT_V3_INTERVAL_MAX_MS		600000

/* Transition-rate thresholds.  Counted within the ZENITH_AT_LOG_NR
 * (=16) window which spans up to ~16 V1 cycles (~160 s with the
 * default V1 cadence).  HI/LO are absolute counts, not rates per
 * unit time -- the calibration cadence is stable enough that
 * counts work fine.
 */
#define ZENITH_AT_V3_THRASH_HI			6
#define ZENITH_AT_V3_THRASH_LO			1

/* Bounded signed offset range for the two V2 reaction knobs. */
#define ZENITH_AT_V3_OFFSET_MIN			(-1)
#define ZENITH_AT_V3_OFFSET_MAX			(+4)

/* Per-policy V3 calibration ring depth.  Each slot records one
 * zenith_at_v3_calibrate() invocation (both OBSERVE and APPLY modes,
 * so operators can see when V3 was running and how it nudged the
 * offsets).  Eight entries cover ~8 minutes of history at the
 * default 60 s interval and ~80 minutes at the 600 s clamp; small
 * enough to fit the read inside a sysfs PAGE_SIZE comfortably with
 * the per-policy header line.  Bumping this is cheap (8 bytes per
 * entry) but anything past PAGE_SIZE bytes will get truncated by
 * the show handler's bound check.
 */
#define ZENITH_AT_V3_CALIB_LOG_NR		8

#define ZENITH_DEFAULT_AT_CLUSTER_AWARE		1
#define ZENITH_DEFAULT_AT_V2_SIGNALS		1
#define ZENITH_DEFAULT_AT_THERMAL_SLOPE		1
#define ZENITH_DEFAULT_AT_THERMAL_PRESSURE_PCT	18
#define ZENITH_DEFAULT_AT_THERMAL_SLOPE_PCT	4
#define ZENITH_DEFAULT_AT_FRAME_PACING		1
#define ZENITH_DEFAULT_AT_SUSTAINED_GAMING	1
#define ZENITH_AT_THERMAL_PCT_MAX		100

/* auto_tune_v2_glides (default 1, on):
 *
 * Master gate for V2-driven population of the round-U-z10 "glide" /
 * coordination knobs:
 *
 *   brutal_decay_ms, wakeup_boost_ms, boot_boost_decay_ms,
 *   screen_off_glide_ms, thermal_pressure_continuous,
 *   frame_budget_us_auto.
 *
 * On stock systems all seven knobs default to 0 (legacy hard
 * cliffs / off).  Without auto_tune_v2_glides the consumer has to
 * hand-tune each one via sysfs to opt into the new behaviour.
 *
 * When auto_tune_v2_glides is 1 (default), zenith_auto_tune_work()
 * populates per-policy "effective" copies of these knobs based on
 * the current V2 state and signal flags; consumers fall back to the
 * effective copy whenever the user-set tunable is 0.  Writes a
 * non-zero value to any of the seven tunables continue to win
 * outright -- the V2-derived copy is only consulted on the 0
 * (default) value.  Set auto_tune_v2_glides to 0 to lock all seven
 * back to byte-identical legacy behaviour at the cost of having to
 * hand-tune anything you want enabled.
 *
 * Costs nothing on auto_tune_v2=0 systems: zenith_at_apply_glides()
 * is only called from the V2 worker path.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES	1

/* auto_tune_v2_tiers (default 1, on) -- Patch L:
 *
 * Sibling of auto_tune_v2_glides for the Stage 4 K1 / K2 / K3
 * floor tiers.  Same pattern, different consumer set:
 *
 *   K1 -> migration_jump_pct, migration_floor_window_ms,
 *         migration_floor_pct
 *   K2 -> psi_cpu_floor_thresh
 *   K3 -> frame_overrun_slack_us, frame_overrun_window_ms,
 *         frame_overrun_floor_pct
 *
 * Difference from glides: these are *floor / lift* knobs, not
 * shape knobs.  Cold-boot defaults are non-zero on some profiles
 * (e.g. PERFORMANCE arms migration with jump=15) so the glide
 * accessor's "user value wins if non-zero" rule would let the
 * profile-set value defeat V2 every time.  Patch L instead uses
 * an *armed-mask* model: the V2 worker writes a bitmask of which
 * tiers should be active for the current state / flag set, and
 * the read site returns the profile-set tunable value if armed,
 * 0 (off) if disarmed.  User sysfs writes set per-knob bits in
 * tunables->auto_tune_override_mask which lock the read to the
 * tunable value regardless of V2.
 *
 * State / flag -> armed tiers mapping (also documented inline at
 * the ZENITH_AT_TIER_* defines):
 *
 *   LATENCY              -> migration + psi_cpu_floor
 *   FRAME flag           -> migration + frame_overrun
 *   GAME flag            -> migration + frame_overrun
 *   anything else        -> all disarmed
 *
 * Set auto_tune_v2_tiers to 0 to lock all three back to pure
 * profile-driven behaviour (the post-K3, pre-Patch-L state).
 *
 * Costs nothing on auto_tune_v2=0 systems: zenith_at_apply_tiers()
 * is only called from the V2 worker path.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS	1

/* Effective values applied by zenith_at_apply_glides() per state.
 * Picked to match the round-U-z10 doc recommendations and keep all
 * seven knobs inside their documented sysfs ranges.
 */
#define ZENITH_AT_GLIDE_BRUTAL_DECAY_MS		150
#define ZENITH_AT_GLIDE_WAKEUP_BOOST_MS		50
#define ZENITH_AT_GLIDE_BOOT_BOOST_DECAY_MS	5000
#define ZENITH_AT_GLIDE_SCREEN_OFF_MS		300
#define ZENITH_AT_GLIDE_THERMAL_PRESSURE_PCT	25

#define ZENITH_AT_STATE_EFFICIENCY		0
#define ZENITH_AT_STATE_BALANCED		1
#define ZENITH_AT_STATE_LATENCY			2
#define ZENITH_AT_STATE_SUSTAINED_PERF		3
#define ZENITH_AT_STATE_THERMAL_RECOVERY	4

#define ZENITH_AT_REASON_CLASSIFIER		0
#define ZENITH_AT_REASON_CAMERA_RENDER		1
#define ZENITH_AT_REASON_MEMSTALL		2
#define ZENITH_AT_REASON_AUDIO			3
#define ZENITH_AT_REASON_VARIANCE		4
#define ZENITH_AT_REASON_THERMAL		5
#define ZENITH_AT_REASON_COOLDOWN		6
#define ZENITH_AT_REASON_HYSTERESIS		7
#define ZENITH_AT_REASON_SCREEN			8
#define ZENITH_AT_REASON_PSI			9
#define ZENITH_AT_REASON_FRAME			10
#define ZENITH_AT_REASON_GAME			11
#define ZENITH_AT_REASON_THERMAL_SLOPE		12

#define ZENITH_AT_FLAG_AUDIO			BIT(0)
#define ZENITH_AT_FLAG_CAMERA			BIT(1)
#define ZENITH_AT_FLAG_RENDER			BIT(2)
#define ZENITH_AT_FLAG_MEMSTALL			BIT(3)
#define ZENITH_AT_FLAG_THERMAL			BIT(4)
#define ZENITH_AT_FLAG_SCREEN_OFF		BIT(5)
#define ZENITH_AT_FLAG_PSI_CPU			BIT(6)
#define ZENITH_AT_FLAG_PSI_IO			BIT(7)
#define ZENITH_AT_FLAG_FRAME			BIT(8)
#define ZENITH_AT_FLAG_GAME			BIT(9)
#define ZENITH_AT_FLAG_THERMAL_SLOPE		BIT(10)
#define ZENITH_AT_FLAG_LOCAL_ACTIONS		BIT(11)
/* Set by the V1 classifier worker when prefer_silver_aware is on AND
 * the prefer_silver hit-rate over the last classifier window crossed
 * the prefer_silver_hot_threshold_pct cutoff (dead -- Kconfig removed,
 * never fires).  Read-only signal; the actual dynamic_up_thresh bump
 * is applied directly in zenith_get_next_freq() (the signal does not
 * feed the V2 state machine because prefer_silver redistribution is
 * workload-dependent
 * and would race with the existing thermal / PSI / frame triggers).
 */
#define ZENITH_AT_FLAG_PREFER_SILVER_HOT	BIT(12)
/* Audit fix F2: PELT-derived util-rising trend signal.
 *
 * Set by the V1 auto-tune worker when the policy-wide util average
 * delta between this window and the last window exceeded
 * t->auto_tune_util_rising_thresh_pct.  Used by V2 to bias toward
 * LATENCY when load is rapidly ramping (e.g. cold app launch, web
 * page first-paint, game scene transition) before sat_pct fully
 * crosses the hi_sat_pct threshold.
 *
 * Read-only flag like THERMAL_SLOPE and PREFER_SILVER_HOT; the
 * actual state-machine consumption happens in zenith_v2_propose()
 * (specifically the BALANCED -> LATENCY edge).  Bit chosen to
 * leave the LSB nibble for stable scenario flags (camera/audio/
 * render/etc.) and the next nibble for environmental signals.
 */
#define ZENITH_AT_FLAG_UTIL_RISING		BIT(13)

#define ZENITH_AT_OVERRIDE_UP_RATE		BIT(0)
#define ZENITH_AT_OVERRIDE_DOWN_RATE		BIT(1)
#define ZENITH_AT_OVERRIDE_UP_THRESHOLD		BIT(2)
#define ZENITH_AT_OVERRIDE_DOWN_THRESHOLD	BIT(3)
#define ZENITH_AT_OVERRIDE_INPUT_BOOST_MS	BIT(4)
#define ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP	BIT(5)
#define ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE	BIT(6)
#define ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE	BIT(7)
#define ZENITH_AT_OVERRIDE_FRAME_PACE		BIT(8)
#define ZENITH_AT_OVERRIDE_GAME_MODE		BIT(9)

/* Patch L: V2-classifier "tier" override bits.
 *
 * The Stage 4 K1 / K2 / K3 floor tiers are profile-driven knobs --
 * the user picks PERFORMANCE / BALANCED / BATTERY / LEGACY and
 * zenith_apply_profile() writes the per-knob value.  Patch L lets
 * the V2 state classifier *additionally* arm or disarm those
 * tiers per-state (e.g. arm migration_floor in LATENCY, disarm it
 * in EFFICIENCY) without disturbing the profile-set values.
 *
 * Each bit, when set in tunables->auto_tune_override_mask, locks
 * the matching knob to whatever the user wrote via sysfs -- the
 * V2 worker stops touching it.  Profile changes clear the entire
 * mask in zenith_apply_profile() (existing behaviour), so a
 * profile flip rearms V2 as if the user had never overridden.
 *
 * (Bits 0..9 are the existing V2 actions overrides; bits 10..16
 * are the new tier overrides.)
 */
#define ZENITH_AT_OVERRIDE_MIGRATION_JUMP	BIT(10)
#define ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_WIN	BIT(11)
#define ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_PCT	BIT(12)
#define ZENITH_AT_OVERRIDE_PSI_CPU_FLOOR	BIT(13)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_SLACK	BIT(14)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_WINDOW	BIT(15)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_FLOOR	BIT(16)

/* Patch M1: PSI-mem light cap.  Three new override bits, sitting
 * at the end of the existing tier-overrides bank.  Same shape as
 * the K1/K2/K3 override bits above: a sysfs write to any of the
 * three knobs ORs in its bit, and the V2 worker stops touching
 * that specific knob until the next profile flip clears the
 * mask.  Profile-flip-clears-mask is implemented in
 * zenith_apply_profile() (existing code, no edit needed here).
 */
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_THRESH	BIT(17)
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_PCT	BIT(18)
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_WINDOW	BIT(19)

/* Floor/cap-knob override fence.  Five sysfs knobs that are read on
 * the freq-update hot path but are not driven by any V2 / V3 path
 * today.  The bits sit dormant: a sysfs write to any of the five
 * knobs ORs in its bit, and any future zenith_at_set_uint() caller
 * gated on these bits will short-circuit, leaving the user value
 * alone.  Profile-flip-clears-mask is in zenith_apply_profile()
 * already (existing code, no edit needed here).
 */
#define ZENITH_AT_OVERRIDE_RENDER_FLOOR_PCT	BIT(20)
#define ZENITH_AT_OVERRIDE_AUDIO_FLOOR_PCT	BIT(21)
#define ZENITH_AT_OVERRIDE_AUDIO_CAP_PCT	BIT(22)
#define ZENITH_AT_OVERRIDE_AUDIO_HYST_MS	BIT(23)
#define ZENITH_AT_OVERRIDE_CAMERA_FLOOR_PCT	BIT(24)

/* Patch L: V2-classifier tier-armed bitmask.
 *
 * Written by zenith_at_apply_tiers() per V2 worker pass; read by
 * the K1 / K2 / K3 reading sites in zenith_get_next_freq() via
 * zenith_tier_value().  Mapping (state / flag -> armed tiers):
 *
 *   LATENCY              -> migration + psi_cpu_floor
 *   FRAME flag (any)     -> migration + frame_overrun
 *   GAME flag (any)      -> migration + frame_overrun
 *   EFFICIENCY/BALANCED  -> none
 *   THERMAL_RECOVERY     -> none
 *   SUSTAINED_PERF       -> none (intentional: SUSTAINED_PERF
 *                          already pins via the actions path,
 *                          adding tier floors on top is double-
 *                          counting)
 *
 * A bit being clear means the tier is *disarmed* and the read
 * site treats the knob as 0 (off) for the duration of the V2
 * window, regardless of the profile-set value.  When the user
 * overrides via sysfs the override mask gates first and the tier
 * mask is irrelevant for that knob.
 */
#define ZENITH_AT_TIER_MIGRATION		BIT(0)
#define ZENITH_AT_TIER_PSI_CPU_FLOOR		BIT(1)
#define ZENITH_AT_TIER_FRAME_OVERRUN		BIT(2)
/* Patch M1: PSI-mem cap tier.  Armed in EFFICIENCY / BALANCED /
 * THERMAL_RECOVERY (states where backing off on memstall is the
 * desired behaviour); disarmed in LATENCY / SUSTAINED_PERF and
 * under the FRAME / GAME flag bypass.  Cap is final-freq, not a
 * predicate; disarm here turns the read-site into a no-op for
 * the duration of the V2 window regardless of the profile-set
 * thresh value.
 */
#define ZENITH_AT_TIER_PSI_MEM_CAP		BIT(3)

#define ZENITH_CLUSTER_LITTLE			0
#define ZENITH_CLUSTER_BIG			1
#define ZENITH_CLUSTER_PRIME			2

/* kcpustat-derived hispeed-floor blend (see cpufreq_zenith.c "kcpustat
 * hispeed blend" section for the algorithm). The feature ships OFF;
 * userspace flips kcpustat_hispeed_enable=1 once trace data shows the
 * blend actually lifts cold-start frequencies on the target SoC.
 * Enabled by default to compensate PELT's 32 ms cold-start lag.
 *
 *   kcpustat_window_us     - observation window in microseconds. Each
 *                            window samples idle / wall delta from
 *                            kcpustat to compute a raw busy_pct (0..100).
 *                            Smaller windows respond faster but get
 *                            noisier. 4 ms matches reflex's default.
 *   kcpustat_filter_shift  - asymmetric EWMA shift on busy_pct. Up
 *                            transitions are instant; down transitions
 *                            decay by (filtered - measured) >> shift
 *                            per window. 0 disables the EWMA.
 *   kcpustat_hispeed_enable- master gate. 0 (default) = sampler runs
 *                            for tracing but does not influence freq.
 *                            1 = blend the decayed kcpustat util into
 *                            the PELT util consumed by every tier of
 *                            zenith_get_next_freq().
 */
#define ZENITH_DEFAULT_KCPUSTAT_WINDOW_US	4000
#define ZENITH_DEFAULT_KCPUSTAT_FILTER_SHIFT	1
#define ZENITH_DEFAULT_KCPUSTAT_HISPEED_ENABLE	1

/* util_math_v2 (default 1): when 1, zenith_get_util() folds the cfs_rq
 * runnable_avg into the util signal alongside util_avg / util_est, in
 * the same shape as 6.x cpu_util_cfs_boost(). Helps intermittent
 * tasks (UI thread + render thread spikes) without changing PELT or
 * util_est semantics. Enabled by default for better responsiveness
 * to short burst workloads (UI/render threads).
 */
#define ZENITH_DEFAULT_UTIL_MATH_V2		1

/* uclamp_min_respect (default 1): make zenith honour ADPF-style
 * uclamp_min hints more robustly than the stock schedutil_cpu_util()
 * path alone.  Two behaviours are gated by this tunable:
 *
 *   (a) an explicit final-freq floor of map_util_freq(uclamp_min_eff)
 *       applied after every other decision tier (powersave_bias,
 *       light_load_freq cap, efficient_freq ladder), so rate-limit
 *       windows and soft caps cannot erode ADPF hints;
 *   (b) when screen_state == 0, the aggressive
 *       dynamic_bias = 500 / up_threshold = 95 screen-off override is
 *       suppressed if the policy's effective uclamp_min is at least
 *       ZENITH_UCLAMP_MIN_MEANINGFUL_PCT of SCHED_CAPACITY_SCALE.
 *       This keeps Android's PerformanceHint sessions effective for
 *       legitimate screen-off work (audio decode, nav, sync) without
 *       opening the screen-off budget for every task.
 *
 * Set 0 to revert to the pre-patch behaviour (uclamp_min still flows
 * through schedutil_cpu_util() via the RQ aggregate, but no explicit
 * floor / screen-off suppression is applied on top).
 */
#define ZENITH_DEFAULT_UCLAMP_MIN_RESPECT	1

/* predict_util_pct (default 0, off): when non-zero (1..100), zenith
 * applies a lightweight one-step-ahead linear predictor to the
 * util signal returned by zenith_get_util() before it is consumed by
 * zenith_get_next_freq().  The predictor is
 *
 *    delta = util - prev_util
 *    pred  = util + (delta * predict_util_pct / 100)   // clamped to max
 *    util' = max(util, pred)                            // up-only
 *
 * The intent is to dampen the sawtooth pattern where the governor
 * undershoots a transient ramp by one sample window and chases it
 * across 3-4 windows before catching up.  Up-only ensures we never
 * predict the load below what we actually observed -- ramp-down
 * stays purely PELT-driven.  prev_util is held per zenith_cpu and
 * tracks the *unpredicted* value to keep delta a true sample-to-
 * sample derivative.
 *
 * 0 (default) disables the predictor; the 1..100 range covers the
 * useful spectrum where 50 is half-step-ahead, 100 is one-step-
 * ahead.  Values >100 are accepted but rarely helpful (the predictor
 * gets noisy on small deltas).
 *
 * Default flipped from 0 to 10 in the wave-2 auto-defaults round.
 * 10 is intentionally mild: a tenth-of-one-step lookahead.  Combined
 * with predict_util_smooth=1 (also flipped to 1 in wave-2), the
 * resulting predictor is two-tap-averaged and only adds util on a
 * positive slope, so a downward util ramp is never amplified.  Set
 * this to 0 to disable the predictor entirely; the rest of the
 * governor path is unchanged.
 */
#define ZENITH_DEFAULT_PREDICT_UTIL_PCT		10
#define ZENITH_PREDICT_UTIL_PCT_MAX		200

/* predict_util_smooth (default 0, off):
 *
 * The base predictor at ZENITH_DEFAULT_PREDICT_UTIL_PCT uses a
 * single-tap slope:
 *
 *   slope = util - prev_util
 *   pred  = util + slope * pct / 100
 *
 * A single-tap slope is jumpy: a one-sample outlier in util
 * produces a full-strength prediction on the very next tick.
 *
 * When set to 1, average the slope across the two most recent
 * taps:
 *
 *   slope1 = util       - prev_util
 *   slope2 = prev_util  - prev_util_2
 *   pred   = util + ((slope1 + slope2) / 2) * pct / 100
 *
 * Both slopes are computed only when strictly positive, so the
 * "up-only" invariant of the base predictor is preserved (a
 * ramp-down is never amplified).  prev_util_2 is maintained as
 * long as predict_util_pct > 0; toggling smooth on/off is free.
 *
 * Same cap / clamp / trace / prev_util update semantics as the
 * base predictor, so reverting to 0 returns to the historical
 * single-tap math with no lingering state.
 *
 * Default flipped from 0 to 1 in the wave-2 auto-defaults round.
 * The smooth path is gated on predict_util_pct > 0, so this default
 * is a no-op until the predictor itself is enabled.  Pair with the
 * wave-2 predict_util_pct default of 10 for a mild two-tap-averaged
 * predictor.
 */
#define ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH	1

/* render_aware (default 0, off) + render_floor_pct (default 70):
 *
 * When render_aware=1, zenith_get_next_freq() walks the policy's
 * cpumask and checks each cpu_curr(cpu)->comm against a small list of
 * known render / display-pipeline thread names.  If any matches and
 * render_floor_pct > 0, the final freq is floored at
 *
 *     policy->max * render_floor_pct / 100
 *
 * before the rate-limit gate.  The intent is to keep frame-pacing
 * threads on a frequency tier that delivers their next frame's
 * deadline rather than ramping up only after PELT catches up.
 *
 * The comm check is cached per-policy with a short TTL
 * (ZENITH_RENDER_CACHE_TTL_NS) so the strncmp loop runs at most once
 * every few milliseconds, well above scroll-frame budgets but cheap
 * enough that a hot scroll path doesn't spend any meaningful time on
 * comm matching.
 *
 * Set render_aware=0 to fully disable the feature (no walks, no
 * cache, no floor).  Set render_floor_pct=0 to leave the comm walk
 * running (for tracepoints) but apply no floor.
 *
 * Default flipped from 0 to 1 in the wave-2 auto-defaults round so
 * SurfaceFlinger / RenderThread / RenderEngine / mali-cmar-back are
 * picked up by the comm walk out of the box.  The actual freq floor
 * only fires when one of those threads is the cpu_curr at the moment
 * of a cpufreq decision (cached for ZENITH_RENDER_CACHE_TTL_NS), so
 * idle screens see no floor; only active rendering windows do.  The
 * render_floor_pct default is unchanged (70%) and continues to win
 * over the V2 effective up_threshold tier.
 */
#define ZENITH_DEFAULT_RENDER_AWARE		1
#define ZENITH_DEFAULT_RENDER_FLOOR_PCT		70
#define ZENITH_RENDER_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)

/* render_floor_min_runtime_ms (default 50, [Stage 4 / Patch B]):
 *
 * Debounce window for the render-thread floor.  When a render
 * thread first becomes the cpu_curr after a quiet period, the
 * floor is *not* applied until the thread has been observed for
 * at least render_floor_min_runtime_ms milliseconds.  A render
 * thread that rises and falls inside the debounce window (e.g.
 * an idle SurfaceFlinger flush, a one-shot RenderEngine wakeup)
 * never floors the cluster.
 *
 * 0 disables the debounce: the floor fires the moment the
 * render thread is picked up, which is the original Wave-2
 * behaviour.  Capped at ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX
 * so a bad echo can't push the debounce into the seconds range
 * and silently disable the floor for whole frames.
 *
 * The debounce is tracked by a per-policy stamp
 * (render_first_seen_ns) that is taken on the first sample with
 * has_render==true and reset to 0 on the first sample with
 * has_render==false.  No timers, no work_struct: the check is a
 * single ktime_get_ns() comparison on the existing eval path.
 */
#define ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS	50
#define ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX		1000

/* game_mode (default 0, off):
 *
 * When game_mode=1, zenith applies two lightweight runtime overlays
 * that together match the heuristics most game-detection daemons
 * (Realme TouchBoost, OnePlus GameSpace, etc.) want:
 *
 *  (a) the effective hispeed_freq_pct used by zenith_eff_hispeed_freq()
 *      is multiplied by ZENITH_GAME_HISPEED_BOOST_PCT/100 -- by
 *      default a 10%% lift of the per-cluster auto-default floor;
 *  (b) the effective input_boost_decay_ms is multiplied by
 *      ZENITH_GAME_BOOST_DECAY_PCT/100 -- by default 130%%, so the
 *      trailing decay window is ~30%% longer to keep frametime
 *      stable across stick-flick / camera-pan inputs.
 *
 * The tunable accepts 0/1/2; userspace (a small gameswitch helper,
 * or a Realme `/proc/touchpanel/game_switch_enable` watcher) is
 * expected to flip it.  Level 2 ("turbo") stacks additional
 * runtime overrides on top of level 1 -- see the comment block
 * attached to ZENITH_GAME_L2_HISPEED_BOOST_PCT for the full list.
 * Values >2 are rejected by sysfs with -EINVAL.
 */
#define ZENITH_DEFAULT_GAME_MODE			0
#define ZENITH_GAME_HISPEED_BOOST_PCT		110
#define ZENITH_GAME_BOOST_DECAY_PCT		130

/* game_mode=2 ("turbo") stacks the game_mode=1 overlays on top of:
 *
 *   - a stronger hispeed boost multiplier (default 120%% vs 110%%),
 *   - a longer input_boost decay window (default 160%% vs 130%%),
 *   - effective input_boost_cap_pct treated as 0 (pin to policy->max
 *     during the full-boost phase regardless of the user-set cap),
 *   - effective climb_mode treated as SNAP for the brutality tier
 *     regardless of the user-set climb_mode.
 *
 * The stacked overrides are applied inline in the hot path and do
 * not mutate the underlying tunables, so switching back to 0 or 1
 * restores the user's original climb_mode / input_boost_cap_pct
 * bit-exactly.  Only the level 1 overlays (ZENITH_GAME_*_PCT above)
 * apply at game_mode=1; level 2 is a strict superset.
 */
#define ZENITH_GAME_L2_HISPEED_BOOST_PCT	120
#define ZENITH_GAME_L2_BOOST_DECAY_PCT		160
#define ZENITH_GAME_MODE_MAX			2

/* game_auto (default 1, on):
 *
 * In-kernel heuristic for raising the effective game_mode without a
 * userspace gameswitch helper.  Walks each policy's online cpus and
 * matches cpu_curr->comm against the rcu-protected zenith_game_auto
 * comm table (see zenith_game_auto_comms[] for the seed list).  When
 * the same cpufreq decision sees the comm match for at least
 * ZENITH_GAME_AUTO_DETECT_STREAK consecutive calls, the global latch
 * zenith_game_auto_active_until_ns is set to
 * (now + ZENITH_GAME_AUTO_ACTIVE_TTL_NS).
 *
 * While the latch is in the future, zenith_eff_game_mode() reports
 * max(user game_mode, V2 effective, 1) -- so the existing game_mode=1
 * overlays (hispeed boost + input_boost decay stretch) apply
 * automatically.  The latch is auto-renewed on every fresh detection;
 * absent re-detection, it expires after ZENITH_GAME_AUTO_ACTIVE_TTL_NS
 * and the system reverts to the user / V2 game_mode value.
 *
 * Default 1 (on, wave-7 round): the seed comm list is conservative
 * (Unity / Unreal main / il2cpp / GameThread) and the worst-case
 * false-positive cost is a 5-second level-1 game_mode bump that
 * cannot push V2 into a state-change-impossible configuration.  The
 * static key zenith_game_auto_key still gates the comm walk so the
 * hot-path cost when no game thread is present is a single bounded
 * for_each_cpu() with an early break on first match.
 *
 * Init-time invariant: the static key zenith_game_auto_key must be
 * synced TRUE in zenith_init() against the default-1 scalar (same
 * pattern as wave-2 audio_aware / render_aware).  See the
 * DEFINE_STATIC_KEY_FALSE comment block.
 *
 * Tunable surface:
 *   - game_auto         RW 0/1   master gate
 *   - game_auto_state   RO 0/1   shows the current global latch state
 *   - game_auto_comms   RW CSV   comm prefix table (RCU-swapped on
 *                                store like render_comms / audio_comms)
 */
#define ZENITH_DEFAULT_GAME_AUTO		1
#define ZENITH_GAME_AUTO_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)
#define ZENITH_GAME_AUTO_DETECT_STREAK		32
#define ZENITH_GAME_AUTO_ACTIVE_TTL_NS		(5ULL * NSEC_PER_SEC)

/* frame_budget_us (default 0, off) + frame_pace_floor_pct (default 0):
 *
 * Userspace-driven, lock-free, KMI-clean alternative to a real DRM
 * vblank hook.  The intent is the same as a frame-pacing governor:
 * keep the freq high enough that the render pipeline is never the
 * critical path of a frame's compute deadline.  The implementation
 * differs from real frame pacing in that nothing in the kernel sees
 * vblank events directly.  Instead, userspace (a small daemon that
 * watches /sys/class/drm/card0-DSI-1/vrefresh, or SurfaceFlinger if
 * patched, or the existing Realme display HAL) writes the current
 * vblank period in microseconds to frame_budget_us whenever the
 * panel switches refresh rate.
 *
 *   60 Hz   -> echo 16667 ...
 *   90 Hz   -> echo 11111 ...
 *  120 Hz   -> echo  8333 ...
 *  144 Hz   -> echo  6944 ...
 *  off      -> echo     0 ...
 *
 * The floor itself is computed adaptively: shorter budgets need a
 * higher floor because the same compute must finish in less wall
 * time.  The formula is
 *
 *   eff_pct = frame_pace_floor_pct * 16667 / frame_budget_us
 *   floor   = policy->max * min(eff_pct, 100) / 100
 *
 * so frame_pace_floor_pct is the *60 Hz baseline*; it auto-scales
 * upward at higher refresh rates without userspace re-tuning.
 *
 * frame_budget_us = 0 disables the feature regardless of
 * frame_pace_floor_pct.  frame_pace_floor_pct = 0 disables the
 * floor while keeping the budget set (useful for tracing the
 * tracepoint without changing freq).
 *
 * Floor is capped by uclamp_max downstream so an explicit ADPF
 * power-efficiency hint still wins.
 */
#define ZENITH_DEFAULT_FRAME_BUDGET_US		0
#define ZENITH_FRAME_BUDGET_US_MAX		50000

/* frame_budget_us_auto (default 0, off):
 *
 * When 1, the adaptive frame-budget floor in zenith_get_next_freq()
 * uses the cached refresh-rate value at zenith_drm_vblank_us in
 * preference to tunables->frame_budget_us / the per-policy override.
 * The cached value is the most recent vblank period (in us) reported
 * by the display driver via the exported zenith_set_drm_vblank_us()
 * kernel API; when zero (driver hasn't reported yet) the auto path
 * falls back to the userspace-set frame_budget_us so existing
 * tunings keep working.
 *
 * The "_drm" naming reflects the source of truth: any driver that
 * owns the active panel mode (drm-bridge, mipi-dsi panel, or vendor
 * display HAL upstreaming via drm) calls zenith_set_drm_vblank_us()
 * on every vblank-period change.  Eliminates the userspace
 * round-trip that otherwise loses 90 / 120 / 144 Hz bumps until the
 * HAL relays them to /sys/devices/.../zenith/frame_budget_us.
 */
#define ZENITH_DEFAULT_FRAME_BUDGET_US_AUTO	0

/* frame_budget_us_per_policy (default empty, off):
 *
 * frame_budget_us is global -- one vblank period applied to every
 * policy.  On big.LITTLE / 3-cluster SoCs the right value usually
 * differs per cluster: little wants 0 (no adaptive floor at all,
 * the cluster idles between frames), big wants the full 16667 /
 * 8333 / 6944 us depending on display refresh rate, prime in the
 * middle.  A single global value forces userspace to either over-
 * or under-floor at least one cluster.
 *
 * frame_budget_us_per_policy is a CSV override read as
 *
 *   anchor_cpu:budget_us[,anchor_cpu:budget_us]...
 *
 * where anchor_cpu is cpumask_first(policy->cpus) -- the same
 * "policyN" identifier the cpufreq sysfs tree already uses.  Stored
 * as a fixed-size array indexed by cpu, parsed once on store and
 * read lock-free in zenith_get_next_freq().  A non-zero entry for
 * the policy's anchor cpu overrides the global frame_budget_us
 * for that policy; a zero entry (the default) falls through to
 * the global value, preserving today's shape on every policy that
 * isn't called out in the CSV.
 *
 * Empty string clears all overrides.  Bounds: anchor_cpu < NR_CPUS,
 * budget_us <= ZENITH_FRAME_BUDGET_US_MAX.  Same disable semantics
 * as frame_budget_us itself: 0 means "no adaptive floor" for that
 * policy.
 */
#define ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT	0
#define ZENITH_FRAME_PACE_BASE_BUDGET_US	16667

/* psi_aware (default 1, on) + psi_mem_thresh (default 50)
 * + psi_cpu_thresh (default 0, off) + psi_io_thresh (default 0, off):
 *
 * When psi_aware=1, zenith_get_next_freq() reads the system-wide
 * pressure 10s averages from psi_system.avg[PSI_*_SOME][0].  Each
 * dimension has its own integer-percentage threshold; if the live
 * average is at or above the threshold for that dimension, the final
 * freq is *capped* at the effective hispeed floor.  Rationale:
 *
 *  - PSI_MEM_SOME: heavy memory stall.  Pushing above hispeed wastes
 *    energy on cycles that mostly stall waiting for memory; the
 *    workload is memory-bound, not compute-bound.
 *
 *  - PSI_CPU_SOME: oversubscribed runqueue.  More than one task is
 *    waiting on the cpu; ramping the freq above hispeed doesn't make
 *    runnable tasks run, it just burns power on the one that *is*
 *    running.  Useful on big.LITTLE where a small cluster gets piled
 *    on by a wakeup storm before the load balancer migrates anything.
 *
 *  - PSI_IO_SOME: I/O-bound stall.  The cpu is waiting on storage or
 *    block I/O; freq ramps don't reduce wait time.  Symmetric with
 *    PSI_MEM_SOME.
 *
 * When the hispeed tier is disabled (eff_hispeed == 0) the cap is
 * policy->max -- i.e. a no-op fallback.  The three caps stack: any
 * dimension over-threshold lowers the freq to the hispeed floor (we
 * don't subtract three times; the cap is at most one floor down).
 *
 * The reader is RCU-free and lock-free: psi_system.avg[][] is updated
 * by the avgs_work delayed work and a single READ_ONCE is sufficient
 * to get a coherent fixed-point value.  When CONFIG_PSI is off or
 * psi_disabled is set, the helper returns 0 and the caps never fire.
 *
 * 0..100 range; values >100 rejected by sysfs.  0 disables the cap
 * for that dimension even with psi_aware=1 (useful for tracing the
 * helpers without changing freq).  psi_cpu_thresh / psi_io_thresh
 * default to 0 so out-of-box behaviour matches pre-N1: only the mem
 * cap fires when psi_aware=1.
 *
 * Default flipped from 0 to 1 in the auto-defaults round that
 * accompanies camera_aware: zenith is intended to be self-tuning, and
 * leaving the gate off meant the memstall cap shipped dormant.  The
 * out-of-box impact is a memstall reader gated by psi_mem_thresh = 50
 * (i.e. only fires under sustained heavy memory pressure); the cpu
 * and io caps remain off-by-threshold (0).
 */
#define ZENITH_DEFAULT_PSI_AWARE		1
#define ZENITH_DEFAULT_PSI_MEM_THRESH		50
#define ZENITH_DEFAULT_PSI_CPU_THRESH		0
#define ZENITH_DEFAULT_PSI_IO_THRESH		0

/* psi_cgroup_path (default "" = use system-wide PSI):
 *
 * When non-empty, names a cgroup-v2 path (relative to the unified
 * hierarchy root, e.g. "/foreground") whose per-cgroup PSI averages
 * the zenith_psi_*_some_pct() consumers should read in place of
 * psi_system.avg[][].  Resolved at sysfs-store and at
 * zenith_apply_profile() time via cgroup_get_from_path(); the
 * resolved cgroup pointer is cached in zenith_psi_cgroup (RCU-
 * protected) so the hot-path read is still close to a single
 * READ_ONCE on the EWMA word.
 *
 * Empty string is the safe default and reproduces the pre-B10
 * behaviour exactly: the helpers fall back to &psi_system, which is
 * what every existing call site read before this knob existed.
 *
 * Path resolution failures (cgroup-v2 not mounted, path missing,
 * cgroup-v1-only system) are silently demoted to the empty/system-
 * wide path; no -ENOENT to userspace.  This keeps profile-baked
 * defaults safe across vendor cgroup naming variants.
 *
 * Path length cap is a generous compromise between cgroup hierarchy
 * depth and inline-buffer cost (one buffer per attr_set, not per
 * policy).
 */
#define ZENITH_PSI_CGROUP_PATH_MAX		128

/* audio_aware (default 0, off) + audio_floor_pct (default 0) +
 * audio_cap_pct (default 0):
 *
 * When audio_aware=1, zenith_get_next_freq() walks the policy's
 * cpumask and checks each cpu_curr(cpu)->comm against a small list
 * of known Android audio-thread names (AudioOut_*, audioserver,
 * MediaCodec_*, OMX*, SoundPool, ...).  If any matches, the final
 * freq is clamped into a configurable band:
 *
 *     floor = policy->max * audio_floor_pct / 100   (0 = no floor)
 *     cap   = policy->max * audio_cap_pct   / 100   (0 = no cap)
 *
 * The point is to keep the freq an audio thread is running on as
 * stable as possible.  Audio buffers underrun when freq drops
 * mid-buffer; transient ramps to policy->max waste energy and
 * incur a freq-transition stall that itself can blow a frame.  A
 * modest floor (e.g. 40) prevents the underrun half; a modest cap
 * (e.g. 60) prevents the burst-to-max half.  Either alone is
 * useful; together they form a stable band.
 *
 * The cap is applied *before* the uclamp_max final cap so an
 * explicit ADPF power-efficiency hint can still walk it down
 * further.  The floor is applied alongside the render_floor /
 * frame_pace_floor tier and is subject to the same uclamp_max
 * downstream cap.
 *
 * The comm check is cached per-policy with TTL
 * ZENITH_AUDIO_CACHE_TTL_NS so a hot path (e.g. a 60 / 90 / 120 Hz
 * scroll) only does the strncmp loop a few times per second.
 *
 * Set audio_aware=0 to fully disable the feature (no walks, no
 * cache, no clamp).  Set audio_floor_pct=audio_cap_pct=0 to leave
 * the comm walk running (for tracepoints) but apply no clamp.
 *
 * Default flipped from 0 to 1 in the wave-2 auto-defaults round.
 * audio_floor_pct and audio_cap_pct stay at 0 by default, so the
 * runtime impact is just the comm walk (cached for
 * ZENITH_AUDIO_CACHE_TTL_NS) and the V2 scenario flag -- no actual
 * frequency clamp is applied unless the operator opts in by writing
 * a non-zero floor / cap.
 */
#define ZENITH_DEFAULT_AUDIO_AWARE		1
#define ZENITH_DEFAULT_AUDIO_FLOOR_PCT		0
#define ZENITH_DEFAULT_AUDIO_CAP_PCT		0
#define ZENITH_AUDIO_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)
#define ZENITH_DEFAULT_AUDIO_HYST_MS		250
#define ZENITH_AUDIO_HYST_MS_MAX		2000

/* Patch B7-2: decision-ring depth.  Per-policy circular buffer of
 * the last ZENITH_DEC_RING_NR (path, lat_ns) entries.  Power-of-two
 * so head advance is a single AND.  Storage budget: 32 entries *
 * sizeof(struct zenith_dec_ring_entry) per policy (~512 B).
 */
#define ZENITH_DEC_RING_NR			32
#define ZENITH_DEC_RING_MASK			(ZENITH_DEC_RING_NR - 1)

/* camera_aware (default 1, on) + camera_active (default 0, auto)
 * + camera_floor_pct (default 0):
 *
 * When camera_aware=1, zenith_get_next_freq() walks the policy's
 * cpumask and checks each cpu_curr(cpu)->comm against a small list
 * of known Android camera HAL / framework thread names
 * (cameraserver, provider@N.M-se, provider.MTK*, mtkcam-*,
 * Camera2-*, CamX_*, ...).  If any matches, the final freq is
 * floored at (policy->max * camera_floor_pct / 100), mirroring
 * the render_aware tier.
 *
 * Camera workloads benefit from a stable high freq -- the capture
 * pipeline stalls when the freq dips below what its ISP/codec
 * pipeline needs.  No companion cap is provided (unlike audio):
 * camera bursts are short and infrequent, and capping freq there
 * costs frame-rate.
 *
 * The userspace override knob (camera_active) is provided because
 * vendor camera HALs sometimes use thread names that don't match
 * the comm table on every device.  Values:
 *
 *   0 (auto, default)  -- consult comm table
 *   1 (force-on)       -- floor always applied (skip comm walk)
 *   2 (force-off)      -- floor never applied (skip comm walk)
 *
 * The HAL or a Magisk module can write 1 on capture-start /
 * preview-start and 2 (or 0) on capture-stop, giving deterministic
 * behaviour without the kernel having to know every vendor name.
 *
 * The comm check is cached per-policy with TTL
 * ZENITH_CAMERA_CACHE_TTL_NS (mirrored from render).  Set
 * camera_aware=0 to fully disable; set camera_floor_pct=0 to leave
 * comm-walk + tracepoint visibility on but apply no floor.
 *
 * Default flipped from 0 to 1 in the auto-defaults round that
 * accompanies psi_aware: zenith is intended to be self-tuning, and
 * shipping the gate off meant the camera floor never fired without
 * an explicit sysfs poke.  Out-of-box impact is a comm-walk on the
 * eval-cadence path (cached for ZENITH_CAMERA_CACHE_TTL_NS); no
 * frequency clamp is applied unless camera_floor_pct is also set
 * to a non-zero value (it stays at 0 by default).
 */
#define ZENITH_DEFAULT_CAMERA_AWARE		1
#define ZENITH_DEFAULT_CAMERA_ACTIVE		0
#define ZENITH_DEFAULT_CAMERA_FLOOR_PCT		0
#define ZENITH_CAMERA_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)

#define ZENITH_CAMERA_OVERRIDE_AUTO		0
#define ZENITH_CAMERA_OVERRIDE_FORCE_ON		1
#define ZENITH_CAMERA_OVERRIDE_FORCE_OFF	2

/* boot_boost_ms (default 0, off):
 *
 * When non-zero, zenith pins the final freq to policy->max for the
 * first boot_boost_ms milliseconds after system boot.  Gated by
 * screen_state so it only applies after the screen comes on, and
 * skipped on small clusters when input_boost_big_only=1 (the existing
 * input-boost gate).  Intended to absorb the cold-cache, lots-of-zygote
 * boot path without paying for low-freq sample windows that PELT then
 * spends 200 ms catching up out of.
 *
 * boot_boost_ms is a one-shot: zenith_get_next_freq() compares
 * ktime_get_boottime_ns() against boot_boost_ms * NSEC_PER_MSEC and
 * disables the floor for everyone past that deadline.  Setting it to
 * 0 disables the feature; values up to a few minutes are accepted
 * but anything past 60_000 ms is wasteful.
 *
 * Recommended init.rc tune: 30000 (30 s).
 */
#define ZENITH_DEFAULT_BOOT_BOOST_MS		0

/* screen_off_glide_ms upper bound.  2s past the 1->0 transition is
 * generous: AOD / blanking handlers in Android typically settle in
 * well under 500ms.  0 keeps the historical cliff exactly.
 */
#define ZENITH_DEFAULT_SCREEN_OFF_GLIDE_MS	0
#define ZENITH_SCREEN_OFF_GLIDE_MS_MAX		2000

/* boot_boost_decay_ms upper bound.  30s is enough for the slowest
 * Android cold-boot animation; anything longer would conflict with
 * the screen_off detection that may legitimately follow boot.
 */
#define ZENITH_DEFAULT_BOOT_BOOST_DECAY_MS	0
#define ZENITH_BOOT_BOOST_DECAY_MS_MAX		30000
#define ZENITH_BOOT_BOOST_MAX_MS		300000

/* boot_complete latch (wave-3 follow-up):
 *
 * boot_boost_ms is a wall-clock one-shot that pins the cluster to
 * policy->max for boot_boost_ms milliseconds after boottime origin
 * regardless of whether the platform has actually finished booting.
 * On fast devices this leaves a noticeable energy / heat tail at
 * the end of the wall-clock window after Android has settled.
 *
 * The boot_complete latch lets either userspace or the in-kernel
 * auto-tune worker snap the boost deadline forward to "now" the
 * moment boot is observed complete:
 *
 *   - userspace:  init.zenith.rc raises the latch from
 *                 on property:sys.boot_completed=1 by writing 1 to
 *                 the boot_complete sysfs knob.
 *   - in-kernel:  the auto-tune worker observes a calm streak of at
 *                 least ZENITH_BOOT_COMPLETE_CALM_WINDOWS consecutive
 *                 EFFICIENCY windows past a small grace period and
 *                 raises the latch on the first qualifying policy.
 *
 * Once raised, the boot-boost path in zenith_get_next_freq() snaps
 * the deadline forward to zenith_boot_complete_ns -- the cluster
 * therefore transitions from Phase 1 (pin to max) to Phase 2 (decay
 * via boot_boost_decay_ms) immediately, instead of cliff-cutting to
 * load-derived freq the way "write boot_boost_ms 0" would.
 *
 * boot_complete_auto gates the in-kernel calm-detect path; default 1.
 * Set to 0 to require an explicit userspace write before the boost
 * is considered complete.
 */
#define ZENITH_DEFAULT_BOOT_COMPLETE_AUTO	1
#define ZENITH_BOOT_COMPLETE_CALM_WINDOWS	2
#define ZENITH_BOOT_COMPLETE_GRACE_NS \
	((u64)5000 * NSEC_PER_MSEC)

/* uclamp_max_respect (default 1): symmetric counterpart to
 * uclamp_min_respect.  When set, zenith_get_next_freq() applies an
 * explicit final-freq _cap_ derived from the RQ-aggregated uclamp_max
 * just before the ladder/rate-limit stage, mirroring the uclamp_min
 * floor.  This honours Android's PerformanceHint power-efficiency
 * hint (setPreferPowerEfficiency() -> per-task UCLAMP_MAX) for every
 * decision tier including the brutality snap, hispeed floor, and
 * input boost, which would otherwise walk over the cap.
 *
 * Set 0 to revert to pre-patch behaviour (uclamp_max still flows
 * through schedutil_cpu_util() via the RQ aggregate but no explicit
 * final cap is applied on top).
 */
#define ZENITH_DEFAULT_UCLAMP_MAX_RESPECT	1

/* uclamp_min threshold (in percent of SCHED_CAPACITY_SCALE) above
 * which the screen-off override suppression kicks in. 10 %% means the
 * task's ADPF hint has to reach uclamp_min >= ~102/1024 (about big-core
 * idle-loop capacity) before zenith considers it worth bypassing the
 * screen-off penalty.  Hardcoded rather than tunable -- it is a
 * definition of "meaningful", not a policy knob.
 */
#define ZENITH_UCLAMP_MIN_MEANINGFUL_PCT	10

/* kcpustat tunable bounds. window_us is clamped on store to keep the
 * sampler from thrashing or overflowing; filter_shift caps below the
 * width of an unsigned int.
 */
#define ZENITH_KCPUSTAT_WINDOW_MIN_US		1000
#define ZENITH_KCPUSTAT_WINDOW_MAX_US		100000
#define ZENITH_KCPUSTAT_FILTER_SHIFT_MAX	8

/* Patch L: master switch for human-readable zenith state logging.
 * When ZENITH_DEFAULT_VERBOSE_LOG is non-zero, profile-change /
 * profile-bake-summary / 7-master-switch-flip events emit a
 * pr_info() line tagged "zenith:" to dmesg (forwarded to logcat
 * under the KERNEL tag on Android).  Default 0 so production
 * builds do not get spammed; flipping verbose_log=1 at runtime
 * makes the logging hot.  Plain RW boolean tunable; not profile-
 * baked since logging policy is operator preference, not workload-
 * driven.
 */
#define ZENITH_DEFAULT_VERBOSE_LOG		0

/* Patch K: game / sustained-high-load performance burst.  Five
 * tunables form one mechanism:
 *
 *   game_perf_burst                       master 0/1, default 1
 *   game_perf_burst_floor_pct             freq floor while ARMED
 *   game_perf_burst_thermal_ceiling_dc    skin-temp guardrail (mC)
 *   game_perf_burst_disarm_grace_ms       sustained-clear hold for disarm
 *   game_perf_burst_cooldown_ms           post-disarm step-down glide
 *
 * Detection is a multi-signal AND gate evaluated once per
 * zenith_get_next_freq() invocation (250us..few-ms cadence
 * depending on load / kthread / fast-switch path):
 *
 *   A.  zenith_eff_game_mode() != 0
 *       (manual game_mode write or in-kernel game_auto latched on a
 *        sustained known-game comm-walk match -- see ZENITH_DEFAULT_-
 *        GAME_AUTO comment block).
 *   B.  Sustained big-cluster util >= 70% for >= 2 seconds.
 *       (Read from z_policy->last_load_pct, which is stamped every
 *        tick at the bottom of zenith_get_next_freq() with the load
 *        used by the up-thresh decision -- so this Signal sees the
 *        *previous* tick's load, exactly the right thing for a
 *        sustained-window gate.)
 *   C.  Not video-only / audio-pinned -- the existing audio_aware
 *       sticky-active deadline is treated as a video-pin proxy and
 *       suppresses arming when audio is hot but no game signal
 *       arrived first.  In practice this filters Netflix / VLC
 *       playback that would otherwise trip Signal B alone.
 *
 * Trigger: A AND (B for >= 2s) AND (NOT C-suppressing).
 *
 * Disarm: !A OR (B-clear for >= disarm_grace_ms).  Both edges are
 * cheap loads on the hot path.  After disarm the FSM enters a
 * COOLDOWN glide that linearly steps the floor down to 0 over
 * cooldown_ms milliseconds, so freq doesn't whiplash on Alt+Tab.
 *
 * Latency expectations:
 *   worst case  ~2s     (Signal B requirement) + 1 hot-path tick
 *   best case   ~250ms  (already-armed game_auto + already-saturated
 *                        big cluster, hot path runs straight to ARM)
 *   disarm      ~1s     (default disarm_grace_ms; clamps tail floor)
 *
 * Thermal guardrail (option (c) hybrid):
 *   - At zenith_start() resolve "cpu<N>-thermal" against the kernel
 *     thermal subsystem where N == cpumask_first(policy->cpus).
 *     Cache the resulting struct thermal_zone_device * in
 *     z_policy->gpb_tzd.  IS_ERR / NULL means "no zone for this
 *     policy" (foreign SoC, thermal subsystem not registered yet,
 *     etc.) and the helper falls back to arch_scale_thermal_pressure
 *     converted to a synthetic dC scale (30..70 dC across the 0..100
 *     pressure range) so the guardrail still works on platforms
 *     without a per-cluster thermal zone.
 *   - When the live skin temp >= ceiling_dc, the burst floor is
 *     suppressed for this tick and the existing auto_thermal_cap /
 *     thermal_util_derate path stays in charge.  ARMED state is
 *     retained -- as soon as temp drops below the ceiling on a
 *     subsequent tick the floor re-engages without re-arming.
 *
 * Defaults reasoning:
 *   - master = 1 because the user asked for "all automatic"; the
 *     gate keeps it from firing outside detected games / sustained
 *     load.
 *   - floor_pct = 85 sits above hispeed_freq for every preset and
 *     below policy->max enough to leave the existing thermal path
 *     useful headroom.
 *   - thermal_ceiling_dc = 48000 (48 dC) is the user's 45..50 dC
 *     range midpoint.
 *   - disarm_grace_ms = 1000 keeps Alt+Tab snappy without bouncing
 *     on a single sub-threshold tick.
 *   - cooldown_ms = 5000 lets the cluster drift back without freq
 *     whiplash; matches the existing peak_headroom / brutal_decay
 *     scale.
 */
#define ZENITH_DEFAULT_GAME_PERF_BURST			1
#define ZENITH_DEFAULT_GAME_PERF_BURST_FLOOR_PCT		85
#define ZENITH_DEFAULT_GAME_PERF_BURST_THERMAL_CEILING_DC	48000
#define ZENITH_DEFAULT_GAME_PERF_BURST_DISARM_GRACE_MS	1000
#define ZENITH_DEFAULT_GAME_PERF_BURST_COOLDOWN_MS		5000
#define ZENITH_GAME_PERF_BURST_FLOOR_PCT_MIN		50
#define ZENITH_GAME_PERF_BURST_FLOOR_PCT_MAX		100
#define ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MIN	40000
#define ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MAX	60000
#define ZENITH_GAME_PERF_BURST_DISARM_GRACE_MS_MAX	10000
#define ZENITH_GAME_PERF_BURST_COOLDOWN_MS_MAX		60000
#define ZENITH_GAME_PERF_BURST_B_THRESHOLD_PCT		70
#define ZENITH_GAME_PERF_BURST_B_REQUIRED_NS		(2ULL * NSEC_PER_SEC)
/* Patch M: Schmitt-trigger exit threshold for Signal B.  Once the
 * sustained-arming counter has stamped, hold it through dips above
 * this floor so a scene oscillating in the 65..72% band keeps
 * accumulating sustained time instead of resetting on every dip.
 * Enter at _B_THRESHOLD_PCT (70), exit at _B_EXIT_THRESHOLD_PCT (60).
 */
#define ZENITH_GAME_PERF_BURST_B_EXIT_THRESHOLD_PCT	60
/* Patch M: lazy retry interval for the per-policy thermal zone cache.
 * If thermal-core registers after the governor (boot-ordering quirk
 * on some Tensor builds), zenith_start()'s one-shot resolution leaves
 * gpb_tzd NULL and the guardrail permanently falls back to
 * arch_scale_thermal_pressure().  The hot-path helper retries
 * resolution at most once per this interval until a zone is bound.
 */
#define ZENITH_GPB_TZD_RETRY_INTERVAL_NS		(5ULL * NSEC_PER_SEC)

/* FSM states for game_perf_burst.  Read from sysfs as the
 * game_perf_burst_state RO node and from the floor helper to
 * decide what (if any) freq floor to apply this tick.
 */
#define ZENITH_GPB_STATE_IDLE		0
#define ZENITH_GPB_STATE_ARMED		1
#define ZENITH_GPB_STATE_COOLDOWN	2

/* Patch M: last-disarm reason tokens for the per-policy stats node.
 * Recorded by the FSM evaluator at every ARMED -> COOLDOWN edge so
 * userspace tuning of disarm_grace_ms / cooldown_ms can tell whether
 * users disarm by Alt+Tab/Home (FAST) or by load actually subsiding
 * (SUSTAINED).  NONE = no disarm has happened in the current attach.
 */
#define ZENITH_GPB_DISARM_NONE		0
#define ZENITH_GPB_DISARM_FAST		1
#define ZENITH_GPB_DISARM_SUSTAINED	2

/*
 * Zenith Tunables & State API
 */
struct zenith_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		up_threshold;

	/* See ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE.  Magnitude of the
	 * variance-adaptive lowering applied to dynamic_up_thresh on
	 * bursty workloads, in percent of the static up_threshold.
	 * Range 0..ZENITH_UP_THRESHOLD_ADAPTIVE_MAX.  0 disables.
	 */
	unsigned int		up_threshold_adaptive;
	unsigned int		down_threshold;	/* hysteresis lower bound */

	/* Hispeed floor tier: when load >= hispeed_load (% of max_cap),
	 * ensure the chosen target_freq is at least hispeed_freq (kHz).
	 * hispeed_freq=0 disables the explicit tier.
	 *
	 * When hispeed_freq=0 and hispeed_freq_pct>0, the tier falls
	 * back to a per-cluster auto-default: the effective hispeed
	 * floor for the policy becomes (policy->max * hispeed_freq_pct
	 * / 100).  This gives sensible defaults on both the little and
	 * big clusters without userspace having to read policyN/max and
	 * write one absolute kHz value per policy.  hispeed_freq_pct=0
	 * preserves the legacy "tier disabled" semantics.
	 */
	unsigned int		hispeed_freq;
	unsigned int		hispeed_freq_pct;
	unsigned int		hispeed_load;

	/* Hysteresis margin (percent of max_cap) applied to hispeed_load
	 * on the _exit_ side of the tier.  Entering the tier still
	 * triggers at load_pct >= hispeed_load; leaving it requires
	 * load_pct < (hispeed_load - hispeed_hyst_pct).  Collapses to
	 * the legacy no-hysteresis behaviour at 0.  See
	 * ZENITH_DEFAULT_HISPEED_HYST_PCT.
	 */
	unsigned int		hispeed_hyst_pct;

	/* Symmetric entry-side hysteresis for the hispeed tier.  See
	 * ZENITH_DEFAULT_HISPEED_ENTRY_STREAK.  Capped on store to
	 * ZENITH_HISPEED_ENTRY_STREAK_MAX so the per-policy u8 counter
	 * cannot overflow.
	 */
	unsigned int		hispeed_entry_streak;

	/* Symmetric entry-side hysteresis for the brutality tier.  See
	 * ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK.  Only gates SNAP climb
	 * mode; STEP mode is unaffected.  Capped on store to
	 * ZENITH_BRUTAL_ENTRY_STREAK_MAX so the per-policy u8 counter
	 * cannot overflow.
	 */
	unsigned int		brutal_entry_streak;

	/* Watchdog gate for the peak-headroom rescue tier.  See
	 * ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE for full semantics.
	 * 1 enables the rescue (the default); 0 disables it entirely.
	 * The rescue is also gated by max_cap and policy->max being
	 * non-zero and pin_to_target being false.
	 */
	unsigned int		peak_headroom_rescue;

	/* Per-policy parameters for the peak-headroom rescue tier.
	 * See ZENITH_DEFAULT_PEAK_HEADROOM_* for full semantics and
	 * default values.  All five exist as sysfs knobs so a tester
	 * can A/B-tune the rescue at runtime without rebuilding the
	 * kernel.
	 *
	 *   peak_headroom_starve_load_pct (1..100, default 90)
	 *     Minimum load_pct at which a sample counts as "starving".
	 *     A value of 100 means only fully-saturated samples count;
	 *     0 is rejected on store (rescue would fire continuously).
	 *
	 *   peak_headroom_freq_floor_pct (1..100, default 85)
	 *     Cluster freq must be below this fraction of policy->max
	 *     for a sample to count as "starving".  100 means rescue
	 *     fires whenever freq < policy->max (always-on under
	 *     starvation); values closer to 0 require the cluster to
	 *     be very deep below peak before the rescue triggers.
	 *
	 *   peak_headroom_starve_streak (0..PEAK_HEADROOM_STREAK_MAX,
	 *     default 3)
	 *     Number of consecutive starving samples required before
	 *     the rescue may fire.  The streak counter must exceed
	 *     this value, so the effective wait is streak+1 windows.
	 *     0 fires on the very first starving sample.
	 *
	 *   peak_headroom_jump_pct (1..100, default 100)
	 *     Target as a percentage of policy->max when the rescue
	 *     fires.  100 pins to policy->max; lower values rescue
	 *     to an intermediate freq.  0 is rejected on store
	 *     (rescue with target 0 makes no sense).
	 *
	 *   peak_headroom_hold_ms (0..PEAK_HEADROOM_HOLD_MS_MAX,
	 *     default 50)
	 *     Minimum gap between two consecutive rescue fires, in
	 *     milliseconds.  Prevents stacking rescues on adjacent
	 *     ticks before the cpufreq driver has applied the
	 *     previous request.  0 disables the hold-down (rescue
	 *     can fire on every starving sample past the streak).
	 *
	 * The streak field is u8 for cache locality, so the sysfs
	 * store caps at PEAK_HEADROOM_STREAK_MAX (16); hold_ms caps
	 * at PEAK_HEADROOM_HOLD_MS_MAX (1000) so a runaway value
	 * cannot effectively pin the rescue off forever.
	 */
	unsigned int		peak_headroom_starve_load_pct;
	unsigned int		peak_headroom_freq_floor_pct;
	unsigned int		peak_headroom_starve_streak;
	unsigned int		peak_headroom_jump_pct;
	unsigned int		peak_headroom_hold_ms;

	/* Patch 1.2 batt_hold_scale_pct.  Defaults to 100 (identity).
	 * When the AC-vs-battery cache reports zenith_on_battery == 1,
	 * peak-rescue / peak-prearm hold deadlines are scaled by this
	 * percentage before being applied.  Profile-baked; see the
	 * comment block above zenith_apply_profile() for per-profile
	 * values.  Bounded to ZENITH_BATT_HOLD_SCALE_PCT_{MIN,MAX}.
	 */
	unsigned int		batt_hold_scale_pct;

	/* Wave A charger-aware floor.  See the comment block above
	 * ZENITH_DEFAULT_CHARGER_AWARE for the full rationale.  Both
	 * default 0 so a fresh boot is bit-identical to pre-Wave-A
	 * behaviour.  charger_aware == 0 short-circuits the tier;
	 * charger_floor_pct == 0 leaves the gate live (for tracepoint
	 * visibility) but applies no floor.  Bounded 0..1 and 0..100
	 * respectively on store.
	 */
	unsigned int		charger_aware;
	unsigned int		charger_floor_pct;

	/* Wave A cgroup-aware top-app floor.  See the comment block
	 * above ZENITH_DEFAULT_TOP_APP_AWARE for the full rationale.
	 * Both default 0 so a fresh boot is bit-identical to pre-Wave-A
	 * behaviour.  top_app_aware == 0 short-circuits the tier;
	 * top_app_floor_pct == 0 leaves the gate live (for tracepoint
	 * visibility) but applies no floor.  Bounded 0..1 and 0..100
	 * respectively on store.  Requires CONFIG_CPUSETS=y; on
	 * CONFIG_CPUSETS=n the helper always returns false and the
	 * gate is effectively a no-op.
	 */
	unsigned int		top_app_aware;
	unsigned int		top_app_floor_pct;

	/* Wave A render-thread util tracker.  See the comment block
	 * above ZENITH_DEFAULT_RENDER_THREAD_UTIL_AWARE for the full
	 * rationale.  All three default 0 so a fresh boot is bit-
	 * identical to pre-Wave-A behaviour.  The tier requires
	 * render_aware=1 to function (the comm-walk must run to
	 * observe util_avg).  Bounded 0..1, 0..1024, 0..100
	 * respectively on store.
	 */
	unsigned int		render_thread_util_aware;
	unsigned int		render_thread_util_thresh;
	unsigned int		render_thread_util_floor_pct;

	/* Wave B PMU IPC tracker.  See the comment block above
	 * ZENITH_DEFAULT_PMU_AWARE for the full rationale.  All three
	 * default 0 (except pmu_ipc_thresh which defaults to 100 /
	 * 1.0 IPC because a 0 threshold would be meaningless) so a
	 * fresh boot is bit-identical to pre-Wave-B behaviour and the
	 * tier is opt-in.  Bounded 0..1, 0..1000, 0..100 respectively
	 * on store.
	 */
	unsigned int		pmu_aware;
	unsigned int		pmu_ipc_thresh;
	unsigned int		pmu_ipc_floor_pct;

	/* Wave B EAS / Energy Model integration.  See the comment
	 * block above ZENITH_DEFAULT_EM_AWARE for the full rationale.
	 * Both default 0 so a fresh boot is bit-identical to pre-
	 * Wave-B behaviour.  Bounded 0..1 and 0..200 respectively on
	 * store.
	 */
	unsigned int		em_aware;
	unsigned int		em_floor_pct;

	/* Patch 1.3 cluster-wake-pulse.  See the comment block above
	 * ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS for the full rationale.
	 * cluster_wake_pulse_ms == 0 disables the tier entirely (so
	 * BATTERY / LEGACY profiles short-circuit at no runtime cost).
	 * All three knobs are profile-baked.
	 */
	unsigned int		cluster_wake_pulse_ms;
	unsigned int		cluster_wake_pulse_idle_ms;
	unsigned int		cluster_wake_pulse_floor_pct;

	/* Patch 1.10 quiet-hours cap.  See the comment block above
	 * ZENITH_DEFAULT_QUIET_HOURS_START_MIN for the full rationale.
	 * quiet_hours_start_min == quiet_hours_end_min disables the
	 * tier (default).  cap_pct is the only profile-baked knob in
	 * this group; the start / end window is user-personal.
	 */
	unsigned int		quiet_hours_start_min;
	unsigned int		quiet_hours_end_min;
	unsigned int		quiet_hours_cap_pct;
	unsigned int		quiet_hours_screen_off_only;

	/* Patch 1.9 fg-transition pulse.  See the comment block
	 * above ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS for the full
	 * rationale.  Both knobs profile-baked; fg_transition_-
	 * pulse_ms == 0 disables the producer (no deadline ever
	 * stamped); fg_transition_pulse_pct == 0 disables the
	 * consumer (deadline stamped but no floor applied).
	 */
	unsigned int		fg_transition_pulse_ms;
	unsigned int		fg_transition_pulse_pct;

	/* Pre-arm tier for the peak-headroom rescue.  When 1 (the
	 * default), an early softer intervention fires while the
	 * starvation streak is accumulating but has not yet crossed
	 * peak_headroom_starve_streak.  Specifically: if the previous
	 * sample was starving (peak_starve_count > 0) and the current
	 * cluster freq is still below an effective hispeed_freq value
	 * that would itself escape the starvation floor, lift freq up
	 * to that hispeed value before the rescue tier even runs.
	 *
	 * Effect: a graduated response curve.  Sample 1 of starvation
	 * pulls the cluster into the hispeed band; subsequent samples
	 * either clear (because hispeed was enough) or progress to the
	 * full rescue (because hispeed wasn't enough and the streak
	 * crosses).  The pre-arm self-disables the moment it actually
	 * works -- once freq >= floor_freq, the next sample's starve
	 * check evaluates false and peak_starve_count resets to 0,
	 * which is also the gate that lets the pre-arm fire, so the
	 * intervention is naturally one-shot per starvation episode.
	 *
	 * 0 disables the pre-arm: starvation episodes go directly to
	 * the full rescue after the streak hits.  Mostly useful for
	 * debugging or for tracker A/B comparisons of the rescue tier
	 * alone vs. the rescue+pre-arm pair.  Also automatically a
	 * no-op when hispeed_freq is unconfigured (eff_hispeed == 0)
	 * or when the configured eff_hispeed value is itself below the
	 * floor_freq (lifting to a sub-floor hispeed wouldn't escape
	 * starvation, so we let the rescue handle it).
	 */
	unsigned int		peak_headroom_prearm;

	/* Predictive up-shift trend gate (tier 2a').  See
	 * ZENITH_DEFAULT_PREDICT_UP_THRESH for full semantics.  When
	 * non-zero, zenith_get_next_freq() compares the rise across
	 * the last predict_up_window util samples against this
	 * threshold (expressed as 256ths of max_cap so the value is
	 * unitless: 64 == ~25%% of max_cap rise across the window).
	 * 0 disables the tier; max ZENITH_PREDICT_UP_THRESH_MAX (255)
	 * is the largest value the unitless trend can take, at which
	 * point the tier effectively never fires (would require a
	 * full max_cap rise across the window).
	 */
	unsigned int		predict_up_thresh;

	/* Window size for the predict_up trend ring, in samples.
	 * Must be in [ZENITH_PREDICT_UP_WINDOW_MIN ..
	 * ZENITH_PREDICT_UP_WINDOW_MAX].  The trend compare reads the
	 * util sample written predict_up_window ticks ago against the
	 * one written this tick; out-of-range values are rejected on
	 * sysfs store.  A larger window smooths PELT noise out of the
	 * trend signal at the cost of one more tick of warm-up after
	 * a cold attach.
	 */
	unsigned int		predict_up_window;

	/* See ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH /
	 * ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT (Patch C3).  Single-
	 * sample slope tier that lifts to eff_hispeed_freq when the
	 * delta between the two most-recent util_history samples
	 * crosses pelt_rising_edge_thresh AND the newest sample is
	 * already above pelt_rising_edge_min_pct of max_cap.  0 in
	 * thresh disables the tier (legacy behaviour: only the
	 * rolling-window predict_up fires).
	 */
	unsigned int		pelt_rising_edge_thresh;
	unsigned int		pelt_rising_edge_min_pct;

	/* See ZENITH_DEFAULT_DL_TASK_FLOOR_PCT (Patch C6).  When a
	 * SCHED_DEADLINE task is present on any CPU in the policy,
	 * lift freq to (policy->max * dl_task_floor_pct / 100).  0
	 * disables the floor; non-zero in 1..100 sets the
	 * percentage of policy->max used as the floor.
	 */
	unsigned int		dl_task_floor_pct;

	/* See ZENITH_DEFAULT_IO_FLOOR_HYST_MS /
	 * ZENITH_DEFAULT_IO_FLOOR_HYST_PCT (Patch C9).  Either at 0
	 * disables the IO floor hysteresis tier.  Range checks:
	 * io_floor_hyst_ms in 0..ZENITH_IO_FLOOR_HYST_MS_MAX,
	 * io_floor_hyst_pct in 0..ZENITH_IO_FLOOR_HYST_PCT_MAX.
	 * Read/write via READ_ONCE / WRITE_ONCE (no torn-write
	 * hazard on word-sized scalars).
	 */
	unsigned int		io_floor_hyst_ms;
	unsigned int		io_floor_hyst_pct;

	/* See ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK /
	 * ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT (Patch E).  Either at 0
	 * disables the tier (legacy descent-from-peak).  Range
	 * checks: streak in 0..ZENITH_PEAK_HYSTERESIS_STREAK_MAX,
	 * step_down_pct in 0..100.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and from
	 * zenith_apply_profile().
	 */
	unsigned int		peak_hysteresis_streak;
	unsigned int		peak_step_down_pct;

	/* See ZENITH_DEFAULT_BOOST_IDLE_THRESH /
	 * ZENITH_DEFAULT_BOOST_IDLE_STREAK (Patch F).  Either at 0
	 * disables the boost early-exit (boost is honoured to its
	 * deadline regardless of in-cluster idle).  Range checks:
	 * thresh in 0..100 (load percent), streak in
	 * 0..ZENITH_BOOST_IDLE_STREAK_MAX.  Reads via READ_ONCE on
	 * the eval hot path.
	 */
	unsigned int		boost_idle_thresh;
	unsigned int		boost_idle_streak;

	/* See ZENITH_DEFAULT_BG_UTIL_SCALE_PCT (Patch G).  Scales
	 * zenith_get_util() output down to this percent of natural
	 * util when tunables->screen_state == 0.  Range 1..100.
	 * 100 (default) is a no-op pass-through.  Reads via
	 * READ_ONCE on the eval hot path.
	 */
	unsigned int		bg_util_scale_pct;

	/* See ZENITH_DEFAULT_SLEEPER_TAIL_THRESH_US /
	 * ZENITH_DEFAULT_SLEEPER_TAIL_PCT (Patch H).  thresh_us == 0
	 * disables.  pct bounded in 50..100; eval path reads via
	 * READ_ONCE.
	 */
	unsigned int		sleeper_tail_thresh_us;
	unsigned int		sleeper_tail_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS /
	 * ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT (Patch D).
	 * peer_ramp_window_ms == 0 disables both sides of the multi-
	 * cluster pre-arm coordination.  peer_ramp_floor_pct == 0
	 * suppresses the floor while leaving the deadline writes in
	 * place.  Reads via READ_ONCE on the eval hot path; writes
	 * via WRITE_ONCE from sysfs and zenith_apply_profile().
	 */
	unsigned int		peer_ramp_window_ms;
	unsigned int		peer_ramp_floor_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS (Patch M3).
	 * Screen-state-aware shadow of peer_ramp_window_ms.  Read
	 * via READ_ONCE in zenith_peer_ramp_effective_window_ms()
	 * whenever the arm path or the floor-eval path needs the
	 * effective window length; written via WRITE_ONCE from sysfs
	 * and zenith_apply_profile().
	 */
	unsigned int		peer_ramp_window_off_ms;

	/* See the migration_* macro block (Patch K1).  jump_pct == 0
	 * disables both sides; floor_pct == 0 suppresses the floor
	 * while leaving the per-CPU stamping in place.  All three
	 * are READ_ONCE on the eval / per-CPU update_util paths.
	 */
	unsigned int		migration_jump_pct;
	unsigned int		migration_floor_window_ms;
	unsigned int		migration_floor_pct;

	/* See ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH (Patch K2).  0
	 * disables the tier.  Read once on the eval path; written
	 * via WRITE_ONCE from sysfs and zenith_apply_profile().
	 */
	unsigned int		psi_cpu_floor_thresh;

	/* See the frame_overrun_* macro block (Patch K3).
	 * slack_us == 0 disables stamping (no overrun event ever
	 * arms a deadline); floor_pct == 0 leaves stamping in
	 * place but suppresses the floor.  All three are READ_ONCE
	 * on the eval / vblank-event paths.
	 */
	unsigned int		frame_overrun_slack_us;
	unsigned int		frame_overrun_window_ms;
	unsigned int		frame_overrun_floor_pct;

	/* See ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK /
	 * ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT (Patch M5).
	 * Sub-knob inside K3.  deep_streak == 0 disables the deep
	 * tier (consumer never reads the streak atomic).  When
	 * armed, the deep tier amplifies an active K3 floor on
	 * sustained overrun runs.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and
	 * zenith_apply_profile().
	 */
	unsigned int		frame_overrun_deep_streak;
	unsigned int		frame_overrun_deep_floor_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT /
	 * ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT
	 * (Patch M2).  Independent per-tier sub-gates: when set,
	 * the peer_ramp / migration_floor read sites compute their
	 * effective floor as max(static_pct, uclamp_min_pct).
	 * Default 1 because the uclamp path is a floor-raise; set
	 * 0 to revert to the static-pct-only Stage 4 behaviour for
	 * that specific tier without touching the master
	 * uclamp_min_respect gate.  Reads via READ_ONCE on the
	 * eval hot path; writes via WRITE_ONCE from sysfs.
	 */
	unsigned int		peer_ramp_uclamp_min_respect;
	unsigned int		migration_floor_uclamp_min_respect;

	/* See ZENITH_DEFAULT_PSI_MEM_CAP_* (Patch M1).  Three-tunable
	 * triple for the PSI-mem light cap tier.  All three reads
	 * are gated behind the existing psi_aware master gate AND
	 * the V2 PSI_MEM_CAP tier bit (via zenith_tier_value()).
	 * thresh == 0 short-circuits before any EWMA read, keeping
	 * the disabled path free.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and
	 * zenith_apply_profile().
	 */
	unsigned int		psi_mem_cap_thresh;
	unsigned int		psi_mem_cap_pct;
	unsigned int		psi_mem_cap_window_ms;

	/* Tail-decay window for the brutal-hold cliff exit, in
	 * milliseconds.  0 (default) preserves the historical hard-exit
	 * behaviour: the moment load_pct drops below the (possibly
	 * adaptive-shaped) eff_down threshold, brutal_active is cleared
	 * and the next sample's freq is whatever the EAS proportional
	 * math returns.  Non-zero arms a linear glide: policy->max at
	 * arm time, decaying toward the EAS-computed freq over
	 * brutal_decay_ms.  Eliminates the audible / visible drop that
	 * otherwise happens at the moment of cliff exit, especially on
	 * loads with bursty PELT signals.  Capped at
	 * ZENITH_BRUTAL_DECAY_MS_MAX in the sysfs store.
	 */
	unsigned int		brutal_decay_ms;

	/* Secondary up_threshold applied only when policy->cur has
	 * already climbed to hispeed_freq or above. 0 disables the
	 * substitution and falls back to up_threshold at every bin.
	 *
	 * The common shape is up_threshold=70 to get from idle to
	 * hispeed_freq aggressively, with up_threshold_hispeed=90 to
	 * demand a clearly heavier workload before committing to
	 * policy->max. Requires hispeed_freq != 0.
	 */
	unsigned int		up_threshold_hispeed;

	/* Alternative climb mechanism when load crosses up_threshold.
	 * SNAP (0) pins policy->max (the original ondemand-style
	 * behaviour). STEP (1) bumps target_freq by freq_step_pct
	 * percent of policy->max per sample, giving a slower,
	 * conservative-style climb. STEP mode bypasses the
	 * brutal_active / down_threshold hysteresis.
	 */
	unsigned int		climb_mode;
	unsigned int		freq_step_pct;

	/* Load-proportional scaling for the STEP climb step.  See
	 * ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE.  0 = off (fixed step);
	 * 1 = on (step scales 1.0x .. 2.0x with overshoot).  Only
	 * meaningful when climb_mode == STEP.
	 */
	unsigned int		freq_step_adaptive;

	/* Last-applied preset, or CUSTOM if one was never written. The
	 * tunable does NOT auto-revert to CUSTOM when individual fields
	 * are later modified — the user can always check sysfs to see
	 * which recipe they last applied.
	 */
	unsigned int		active_profile;

	/* Patch B-AUTO-2: auto-selector meta-state.  See ZENITH_PROFILE_AUTO.
	 *
	 * auto_target: when active_profile == ZENITH_PROFILE_AUTO the
	 * decision engine (B-AUTO-3 / B-AUTO-4) writes the most
	 * recently picked concrete target here (BALANCED, PERFORMANCE,
	 * BATTERY, GAMING, or AUDIO) and applies that profile via
	 * zenith_apply_profile().  Default ZENITH_PROFILE_BALANCED so
	 * cold boot is on a known-safe baseline before the first auto
	 * eval lands.  When active_profile != AUTO this field is
	 * stale; readers must check active_profile first.  Read via
	 * READ_ONCE on any path that races with the auto worker;
	 * written via WRITE_ONCE from the worker and from
	 * profile_store on AUTO entry.
	 */
	unsigned int		auto_target;

	/* Patch B-AUTO-3: deferrable workqueue + hysteresis tracking
	 * for the auto-selector engine.
	 *
	 * eval_work runs on the system unbound deferrable workqueue at
	 * a (default 500 ms, profile-baked) cadence whenever
	 * active_profile == ZENITH_PROFILE_AUTO.  It reads device-wide
	 * signals (B-AUTO-4 adds the classifier; B-AUTO-3 stubs it to
	 * return BALANCED) and, after auto_pending_target has held
	 * steady for >= auto_hysteresis_ms, applies the new profile
	 * via zenith_apply_profile() and stamps auto_target.
	 *
	 * eval_work is initialised in the tunables alloc path and
	 * cancelled in the tunables free path.  It re-arms itself at
	 * the end of every run (modulo active_profile == AUTO, which
	 * is the engine's only off-switch).  It is *not* scheduled
	 * automatically at alloc time -- profile_store on AUTO entry
	 * is the only producer of an initial schedule, and B-AUTO-5
	 * adds a cold-boot AUTO flip that triggers that path.
	 *
	 * Concurrency: the worker takes attr_set->update_lock around
	 * any mutation of profile-bake fields (zenith_apply_profile,
	 * zenith_refresh_rate_delays, zenith_invalidate_cache) so the
	 * existing sysfs profile_store path stays race-free.  The
	 * worker reads READ_ONCE(active_profile) outside the lock as
	 * an early-out so the cancel-in-progress path doesn't deadlock
	 * on update_lock with profile_store waiting for
	 * cancel_delayed_work_sync.
	 *
	 * auto_pending_target / auto_pending_first_seen_ns implement
	 * the hysteresis: when the classifier returns a different
	 * target than auto_target the new value lands in
	 * auto_pending_target with a fresh first_seen timestamp; if
	 * the same value re-appears in subsequent windows and
	 * (now - first_seen) >= auto_hysteresis_ms we commit.  When
	 * the classifier flaps back to auto_target the pending state
	 * resets.
	 */
	struct delayed_work	eval_work;
	unsigned int		auto_pending_target;
	u64			auto_pending_first_seen_ns;

	/* Patch B-AUTO-3: cadence + hysteresis tunables for the
	 * auto-selector engine.  See ZENITH_DEFAULT_AUTO_EVAL_MS and
	 * ZENITH_DEFAULT_AUTO_HYSTERESIS_MS for default rationale.
	 * Bounded by ZENITH_AUTO_EVAL_MS_{MIN,MAX} and
	 * ZENITH_AUTO_HYSTERESIS_MS_{MIN,MAX} on sysfs writes.
	 */
	unsigned int		auto_eval_ms;
	unsigned int		auto_hysteresis_ms;

	/* Permille (0..1000) of SCHED_CAPACITY_SCALE at which
	 * zenith_iowait_boost() arms and below which a doubling
	 * decay exits the boost. 125 (12.5%) matches the legacy
	 * SCHED_CAPACITY_SCALE / 8 constant. 0 disables the minimum
	 * floor but still allows the doubling climb to take effect
	 * from its first sample; use io_is_busy=0 to disable iowait
	 * boost wholesale instead.
	 */
	unsigned int		iowait_boost_min;

	/* Percentage (0..100) of the iowait-derived boost that is added
	 * to util when stacking is active.  0 is the legacy behaviour
	 * (max(util, boost)); 100 is full stacking (util + boost, clamped
	 * to max_cap).  The default 50 is a conservative middle ground:
	 * when a CPU is already under compute load and also servicing
	 * I/O, it gets a half-weighted boost on top of its current util
	 * rather than the original "freq takes whichever is larger" which
	 * loses the I/O signal entirely when util is high.  Uses
	 * upstream 6.x schedutil's observation that a task that's both
	 * CPU-heavy and I/O-heavy needs more headroom than either demand
	 * alone would justify.
	 */
	unsigned int		iowait_stack_pct;

	/* See ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS.  Time in ms after
	 * the start of a single iowait episode before the doubling-on-
	 * arm climb flips to a halving step.  0 disables the backoff.
	 */
	unsigned int		iowait_backoff_after_ms;

	/* When 1, a per-policy delayed_work periodically classifies the
	 * recent workload from load-saturation rate and input-event
	 * rate, and auto-selects performance / balanced / battery via
	 * zenith_apply_profile(). Default 0 (off).
	 */
	unsigned int		auto_tune;
	unsigned int		powersave_bias;

	/* Screen-on multiplier applied to powersave_bias before tier 3
	 * (Powersave Bias) consumes it.  Range 0..100 (percent).  When
	 * tunables->screen_state == 1 and the screen-off / thermal
	 * overrides have not replaced dynamic_bias, the effective
	 * dynamic_bias is scaled by screen_on_bias_pct / 100.  See
	 * ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT for the full rationale.
	 *
	 * 100 disables the softening entirely (legacy behaviour: full
	 * configured bias applies on screen-on); 50 (the default)
	 * halves the bias on screen-on; 0 zeroes the bias on
	 * screen-on (full responsiveness, no shave).  Values above 100
	 * are rejected on store; the range is clamped on the read side
	 * defensively in case userspace bypasses the sysfs validator.
	 */
	unsigned int		screen_on_bias_pct;

	unsigned int		io_is_busy;

	/* When 1, dampen the brutality-path load_pct by the fraction
	 * of recent CPU time spent in niced-user mode. Approximation
	 * of ondemand's ignore_nice_load for a PELT-based governor.
	 */
	unsigned int		ignore_nice_load;

	/* Zenith Environment API */
	unsigned int		screen_state;   /* 1 = ON, 0 = OFF */

	/* Soft-glide window for the 1 -> 0 transition on screen_state.
	 * 0 (default) preserves the legacy hard cliff: as soon as
	 * screen_state is observed at 0, dynamic_up_thresh snaps to
	 * 95 and dynamic_bias snaps to 500 (the 50%% powersave
	 * penalty).  Non-zero arms a linear ramp on both: starting
	 * from the natural up_threshold / powersave_bias at the
	 * moment screen_state went to 0, climbing to the cliff
	 * targets over screen_off_glide_ms.  Eliminates the cliff
	 * that otherwise lands the moment AOD / panel-blank handlers
	 * stamp screen_state=0 while userspace work is still
	 * winding down.  Capped at ZENITH_SCREEN_OFF_GLIDE_MS_MAX in
	 * the sysfs store.
	 */
	unsigned int		screen_off_glide_ms;

	/* When 1, zenith subscribes to the fb notifier chain and updates
	 * screen_state automatically on FB_EVENT_BLANK. screen_state
	 * written from userspace still takes effect and is only
	 * overridden on the next blank/unblank event.
	 */
	unsigned int		screen_auto;

	unsigned int		thermal_state;  /* 0 = COOL, 1 = THROTTLING */

	/* When 1, thermal_state is additionally inferred from
	 * arch_scale_thermal_pressure() on every update_util tick.
	 * thermal_state=1 written from userspace still forces it.
	 */
	unsigned int		thermal_auto;

	/* See ZENITH_DEFAULT_THERMAL_AWARE comment block.  Master gate
	 * for the cluster of thermal mechanisms (thermal_util_derate,
	 * the thermal_pressure_continuous up_thresh ramp,
	 * auto_thermal_cap, the V2 THERMAL_RECOVERY transitions, and
	 * zenith_thermal_active() itself).  Default 1.  Strict 0/1.
	 * Static-key gated via zenith_thermal_aware_key.
	 */
	unsigned int		thermal_aware;

	/* RO sysfs mirror.  Set on each call to zenith_thermal_active()
	 * to the function's return value, so userspace can observe
	 * whether the governor currently believes thermal pressure is
	 * high enough to justify the cluster of thermal mechanisms.
	 * Multiple policies may stomp on this from their own update
	 * paths; that race is benign because the field is observability
	 * only and not consumed by any decision tier.  Always 0 when
	 * thermal_aware is 0 (zenith_thermal_active() short-circuits
	 * before writing).
	 */
	unsigned int		thermal_active;

	/* When 1, replace the binary "thermal_active -> dynamic_up_thresh
	 * = 90" cliff with a linear ramp from up_threshold (cool, 0%
	 * pressure) to 90 (hot, 100% pressure).  Smooths long-session
	 * thermal throttling so users don't perceive a step in
	 * frequency the moment thermal_active flips on.  Default 0 to
	 * preserve legacy cliff behaviour for existing tunings.
	 */
	unsigned int		thermal_pressure_continuous;

	/* prefer_silver fields: dead storage.  The Kconfig was removed;
	 * nothing updates ps_hit_rate_pct, so the downstream bump never
	 * fires regardless of these values.  Fields kept to avoid
	 * shifting the struct layout.
	 */
	unsigned int		prefer_silver_aware;
	unsigned int		prefer_silver_hot_threshold_pct;
	unsigned int		prefer_silver_hot_bump_pct;

	/* See ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block.  When
	 * set, zenith_get_util() scales util_out by the (cap - pressure)
	 * / cap fraction whenever pressure exceeds
	 * ZENITH_THERMAL_DERATE_FLOOR_PCT.  Smooths the thermal dance
	 * by asking for less than the throttled max.
	 */
	unsigned int		thermal_util_derate;

	/* See ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT.  Magnitude of the
	 * derivative-term scaling applied on top of thermal_util_derate
	 * when pressure is rising sample-to-sample, in percent.  0..100,
	 * 0 disables.  Read lock-free in the derate site.
	 */
	unsigned int		thermal_derate_rate_pct;

	/* See ZENITH_DEFAULT_AUTO_THERMAL_CAP comment block.  Boolean
	 * gate; when 1, applies a hard cap on target_freq once the
	 * per-policy thermal pressure crosses auto_thermal_cap_-
	 * pressure_pct.  Default 0; flip to 1 via sysfs to opt in.
	 * Read with READ_ONCE in the eval-path tier so the lockless
	 * sysfs writer cannot tear the gate value across CPUs.
	 */
	unsigned int		auto_thermal_cap;

	/* Threshold in percent of arch_scale_thermal_pressure() at
	 * which auto_thermal_cap fires.  Bounded
	 * ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_{MIN,MAX} (1..100).
	 * Read in the eval-path tier with READ_ONCE alongside
	 * auto_thermal_cap and auto_thermal_cap_freq_pct.
	 */
	unsigned int		auto_thermal_cap_pressure_pct;

	/* Cap as a percent of policy->max applied when auto_thermal_-
	 * cap fires.  Bounded ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_{MIN,
	 * MAX} (50..100).  The min bound (50) prevents an accidental
	 * "= 0" from zeroing the cluster.
	 */
	unsigned int		auto_thermal_cap_freq_pct;

	/* See ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT.  Percent of
	 * policy->max below which tiny downward transitions are held at
	 * the current request.  0 disables; range 0..10.
	 */
	unsigned int		freq_stability_margin_pct;

	/* See ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE.  When set, bursty load
	 * variance stretches the effective down-rate delay up to 2x.
	 */
	unsigned int		down_rate_adaptive;

	/* See ZENITH_DEFAULT_WAKEUP_BOOST.  When set, idle-to-busy
	 * transitions arm a short up-rate bypass countdown.
	 */
	unsigned int		wakeup_boost;

	/* Time-based extension of wakeup_boost.  When 0 (default), the
	 * legacy tick-based ZENITH_WAKEUP_BOOST_TICKS countdown is used
	 * verbatim.  When non-zero, the detection sites additionally
	 * arm a per-CPU deadline at now + wakeup_boost_ms; the rate-
	 * limit bypass remains active until either the tick counter
	 * reaches zero AND the deadline has expired.  Lets userspace
	 * tune the bypass duration in real wall-clock time, regardless
	 * of how often zenith_freq_throttle() actually runs (which on
	 * Android can vary widely with up_rate_limit and topology).
	 * Capped at ZENITH_WAKEUP_BOOST_MS_MAX in the sysfs store.
	 */
	unsigned int		wakeup_boost_ms;

	/* See ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE.  Percent by which
	 * bursty load can lower the brutality exit threshold.  0 disables.
	 */
	unsigned int		down_threshold_adaptive;

	/* See ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE.  When set, little
	 * clusters use asymmetric cached up/down rate delays.
	 */
	unsigned int		rate_limit_cluster_scale;

	/* Input boost duration (ms). 0 = disabled. */
	unsigned int		input_boost_ms;
	unsigned int		input_boost_decay_ms;

	/* Touchdown-vs-coordinate-stream extra window for the input
	 * boost (Patch C).  Range
	 * 0..ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX.  See
	 * ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS for semantics.
	 * Mirrored to zenith_input_boost_touchdown_extra_ms_cache on
	 * store and on profile apply, read by zenith_input_event() on
	 * the EV_KEY/BTN_TOUCH press path.
	 */
	unsigned int		input_boost_touchdown_extra_ms;

	/* Shape of the input_boost decay-phase floor.  0 = linear
	 * (legacy), 1 = cubic ease-in (floor holds high, drops fast at
	 * the tail).  See ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE for
	 * semantics.  Range-checked 0..1 on store.
	 */
	unsigned int		input_boost_decay_curve;

	/* When 1 (default), input boost is applied only to policies whose
	 * top arch_scale_cpu_capacity equals SCHED_CAPACITY_SCALE -- i.e.
	 * the system's highest-capacity cluster(s).  On a homogeneous SoC
	 * every policy matches and behaviour is unchanged.  On big /
	 * little, this stops every touch from dragging the little cluster
	 * to policy->max, which rarely helps UI latency (UI / render run
	 * on big) and wastes energy on a cluster that is almost always in
	 * light-load territory at the moment of a tap.
	 *
	 * Set 0 to restore pre-patch behaviour (boost every policy).
	 */
	unsigned int		input_boost_big_only;

	/* Per-profile cap on the input-boost ceiling, expressed as a
	 * percentage of policy->max.  When non-zero, the full-pin phase
	 * targets policy->max * input_boost_cap_pct / 100 instead of
	 * policy->max, and the trailing decay phase ramps from that
	 * capped ceiling down to policy->min over input_boost_decay_ms.
	 * 0 (legacy / PERFORMANCE) means "no cap" -- pin to policy->max.
	 * Sized per-profile so BALANCED / BATTERY get a moderate boost
	 * (PELT can still climb past it under genuine load via the
	 * normal eval tiers, since the boost only sets a floor in the
	 * decay window) without spending the energy of a max-pin on
	 * every tap.  Range 0..100; values > 100 rejected by sysfs.
	 */
	unsigned int		input_boost_cap_pct;

	/* Multiplier for the effective down-rate delay applied to the
	 * cluster while an input boost is active (now <
	 * zenith_input_boost_until_ns).  Range 100..1000 (percent).
	 * 100 disables the extension and restores the legacy
	 * "down_rate_delay during boost == down_rate_delay outside
	 * boost" behaviour.  See ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_-
	 * MULT_PCT for the full rationale; the extension is gated by
	 * input_boost_big_only the same way the boost itself is, so
	 * configurations that suppress the boost on LITTLE also keep
	 * LITTLE on its normal down-rate cadence.
	 */
	unsigned int		input_boost_down_rate_mult_pct;

	/* Efficient-frequency soft-cap ladder, up to ZENITH_EFF_BINS_MAX
	 * entries. Sorted ascending by frequency. The up_delay_us array
	 * is paired 1:1 with eff_freq; writing a single scalar to
	 * up_delay_us broadcasts it to every bin.
	 *
	 * eff_nr == 0 disables the ladder entirely (equivalent to the
	 * pre-ladder "efficient_freq=0" state). The scalar shadows
	 * efficient_freq / up_delay_us remain for backward-compatible
	 * sysfs reads: efficient_freq = eff_freq[0], up_delay_us =
	 * up_delay[0].
	 */
	unsigned int		efficient_freq;
	unsigned int		up_delay_us;
	unsigned int		eff_nr;

	/* See ZENITH_DEFAULT_EFF_BIN_HYST_PCT.  Per-bin release margin
	 * for the efficient_freq ladder, in percent.  0..20.  0 keeps
	 * the legacy "any drop releases" shape.
	 */
	unsigned int		eff_bin_hyst_pct;
	unsigned int		eff_freq[ZENITH_EFF_BINS_MAX];
	unsigned int		eff_delay_us[ZENITH_EFF_BINS_MAX];

	/* Light-load hard cap. light_load_freq=0 disables. */
	unsigned int		light_load_freq;
	unsigned int		light_load_threshold;	/* in % of max_cap */

	/* Hold-at-max multiplier for down_rate_limit. 1 = disabled. */
	unsigned int		sampling_down_factor;

	/* When 1 (default), keep the sampling_down_factor multiplier in
	 * effect for one stretched down-rate window after an input boost
	 * exits, even after target_freq has fallen below policy->max.  The
	 * stretched window is sampling_down_factor * down_rate_limit_us
	 * long.  Mirrors the rationale for sit-at-max stretching: PELT
	 * needs a moment to catch up after a synthetic peak (the boost),
	 * and dropping the multiplier the instant target falls produces
	 * a tail-stutter on gestures whose load profile is bursty around
	 * the boost expiry.  Set 0 to restore pre-patch behaviour.
	 */
	unsigned int		boost_exit_extend;

	/* powersave_bias only applies below this load (% of max_cap).
	 * 100 = always apply (legacy behaviour); 0 = never apply.
	 */
	unsigned int		bias_load_threshold;

	/* auto_tune observer thresholds (see per-field comments at the
	 * ZENITH_DEFAULT_AT_* macros). All are in percent except the
	 * events_x2 pair, which are integer event counts per 2 seconds
	 * over the classification window.
	 */
	unsigned int		auto_tune_sat_load_pct;
	unsigned int		auto_tune_hi_sat_pct;
	unsigned int		auto_tune_lo_sat_pct;
	unsigned int		auto_tune_hi_events_x2;
	unsigned int		auto_tune_lo_events_x2;
	/* F2: util-rising trend threshold, percent.  When the policy-
	 * wide PELT util sum grows by more than this percentage between
	 * consecutive V1 windows, the worker raises ZENITH_AT_FLAG_
	 * UTIL_RISING.  Default 25 (i.e. >=25% increase in window-to-
	 * window mean util).  Setting to 0 disables the signal entirely
	 * (the flag never fires) without removing it from the surface.
	 * Range: 0..200.
	 */
	unsigned int		auto_tune_util_rising_thresh_pct;
	unsigned int		auto_tune_render_rt_floor_pct;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2.  V2 keeps auto-tune bounded by
	 * profile guardrails, hysteresis/cooldown and user override masks.
	 */
	unsigned int		auto_tune_v2;
	unsigned int		auto_tune_hysteresis_windows;
	unsigned int		auto_tune_cooldown_windows;
	/* See ZENITH_DEFAULT_AT_V2_VAR_PROMOTE_THRESH.  load_var_ewma_x256
	 * threshold above which V2 promotes LATENCY to SUSTAINED_PERF on
	 * the variance signal alone.  0 disables variance-driven promotion.
	 */
	unsigned int		auto_tune_v2_var_promote_thresh;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V3 comment block.  0/1/2 master
	 * gate for V3 self-calibration; the *_store callback also
	 * syncs the zenith_auto_tune_v3_key static key (FALSE when 0).
	 * auto_tune_v3_interval_ms is the calibration period; clamped
	 * to [ZENITH_AT_V3_INTERVAL_MIN_MS, ZENITH_AT_V3_INTERVAL_MAX_MS]
	 * on store.
	 */
	unsigned int		auto_tune_v3;
	unsigned int		auto_tune_v3_interval_ms;
	unsigned long		auto_tune_override_mask;
	unsigned int		auto_tune_cluster_aware;
	unsigned int		auto_tune_v2_signals;
	unsigned int		auto_tune_thermal_slope;
	unsigned int		auto_tune_thermal_pressure_pct;
	unsigned int		auto_tune_thermal_slope_pct;
	unsigned int		auto_tune_frame_pacing;
	unsigned int		auto_tune_sustained_gaming;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES.  When 1 (default), the
	 * V2 worker populates per-policy effective values for the
	 * round-U-z10 glide / coordination knobs (brutal_decay_ms,
	 * wakeup_boost_ms, boot_boost_decay_ms, screen_off_glide_ms,
	 * thermal_pressure_continuous,
	 * frame_budget_us_auto) based on V2 state.  Consumers use the
	 * effective value only when the user-set tunable is 0.  0 here
	 * locks all seven back to legacy behaviour exactly.  Ignored
	 * unless auto_tune_v2 is also 1.
	 */
	unsigned int		auto_tune_v2_glides;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS comment block (Patch L).
	 * When 1 (default), the V2 worker writes a per-policy bitmask
	 * of *armed* Stage-4 floor tiers (K1 / K2 / K3) based on the
	 * just-resolved V2 state plus the FRAME / GAME flags.  K1/K2/K3
	 * read sites consult the bitmask via zenith_tier_value() and
	 * treat the underlying tunable as 0 (off) when the matching
	 * tier bit is clear.  User sysfs writes to any of the seven
	 * tier knobs set per-knob bits in auto_tune_override_mask;
	 * those bits suppress the V2 gating and the read returns the
	 * tunable value directly (Profile changes clear the entire
	 * mask, the existing zenith_apply_profile() behaviour, so a
	 * profile flip rearms V2).  Set to 0 to revert all three tiers
	 * to pure profile-driven behaviour.  Ignored unless
	 * auto_tune_v2 is also 1.
	 */
	unsigned int		auto_tune_v2_tiers;

	/* See ZENITH_DEFAULT_AUTO_TUNE_SCENARIO comment block.  Master
	 * gate for the scenario overlay applied on top of the vanilla
	 * load + input-rate classifier in zenith_auto_tune_work().
	 * Requires auto_tune=1; ignored otherwise.
	 */
	unsigned int		auto_tune_scenario;

	/* kcpustat hispeed blend tunables (see ZENITH_DEFAULT_KCPUSTAT_*
	 * comments for semantics). All three default to safe values:
	 * sampler runs at 4 ms windows with EWMA shift=1, but the blend
	 * is gated off until userspace flips kcpustat_hispeed_enable.
	 */
	unsigned int		kcpustat_window_us;
	unsigned int		kcpustat_filter_shift;
	unsigned int		kcpustat_hispeed_enable;

	/* util_math_v2: 0 (default) keeps the historical
	 * cpu_util_cfs() input unchanged; 1 enables the
	 * 6.x-style runnable-aware util computation in
	 * zenith_get_util().
	 */
	unsigned int		util_math_v2;

	/* uclamp_min_respect: see the ZENITH_DEFAULT_UCLAMP_MIN_RESPECT
	 * block at the top of this file for full semantics. In short:
	 *   1 (default) = apply an explicit final-freq floor derived
	 *                  from RQ-aggregated uclamp_min, AND suppress
	 *                  the screen-off penalty when a meaningful
	 *                  uclamp_min is set;
	 *   0           = rely solely on schedutil_cpu_util() RQ uclamp
	 *                  aggregation (pre-G.1 behaviour).
	 */
	unsigned int		uclamp_min_respect;
	unsigned int		uclamp_max_respect;

	/* See ZENITH_DEFAULT_PREDICT_UTIL_PCT comment block. 0 = off. */
	unsigned int		predict_util_pct;

	/* See ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH comment block.  0 = off
	 * (single-tap), 1 = on (two-tap averaged slope).  Ignored when
	 * predict_util_pct == 0.
	 */
	unsigned int		predict_util_smooth;

	/* See ZENITH_DEFAULT_RENDER_AWARE / ZENITH_DEFAULT_RENDER_FLOOR_PCT. */
	unsigned int		render_aware;
	unsigned int		render_floor_pct;

	/* See ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS.  Debounce
	 * window in milliseconds.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and from
	 * zenith_apply_profile().  Range
	 * 0..ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX.
	 */
	unsigned int		render_floor_min_runtime_ms;

	/* See ZENITH_DEFAULT_GAME_MODE. 0/1, normalised on store. */
	unsigned int		game_mode;

	/* See ZENITH_DEFAULT_GAME_AUTO comment block.  0/1 master gate
	 * for the in-kernel game detector.  Flipped via the matching
	 * sysfs node; the *_store callback also syncs the
	 * zenith_game_auto_key static key (FALSE when scalar = 0).
	 */
	unsigned int		game_auto;

	/* See ZENITH_DEFAULT_PSI_AWARE / ZENITH_DEFAULT_PSI_MEM_THRESH /
	 * ZENITH_DEFAULT_PSI_CPU_THRESH / ZENITH_DEFAULT_PSI_IO_THRESH.
	 * All thresholds are 0..100 integer percent of the 10s SOME
	 * average.  0 disables that dimension's cap.
	 */
	unsigned int		psi_aware;
	unsigned int		psi_mem_thresh;
	unsigned int		psi_cpu_thresh;
	unsigned int		psi_io_thresh;

	/* See ZENITH_PSI_CGROUP_PATH_MAX (Patch B10-3).  Empty string =
	 * use system-wide PSI (psi_system); non-empty cgroup-v2 path =
	 * resolve via cgroup_get_from_path() at sysfs-store time and
	 * cache the resolved cgroup in zenith_psi_cgroup for the three
	 * zenith_psi_*_some_pct() helpers to read under rcu_read_lock().
	 *
	 * Buffer is inline (no kmalloc) so tunables free is unchanged
	 * (kfree(t) handles it).  All cross-policy state (the cached
	 * cgroup ref + the active_path no-op cache) lives in the file-
	 * scope zenith_psi_cgroup_* statics.  Last-writer-wins across
	 * policies; in practice all policies converge to the same path
	 * via zenith_apply_profile().
	 */
	char			psi_cgroup_path[ZENITH_PSI_CGROUP_PATH_MAX];

	/* See ZENITH_DEFAULT_AUDIO_AWARE / ZENITH_DEFAULT_AUDIO_FLOOR_PCT
	 * / ZENITH_DEFAULT_AUDIO_CAP_PCT.  Both *_pct fields range 0..100;
	 * 0 in either disables that side of the band.
	 */
	unsigned int		audio_aware;
	unsigned int		audio_floor_pct;
	unsigned int		audio_cap_pct;
	unsigned int		audio_hyst_ms;

	/* See ZENITH_DEFAULT_CAMERA_AWARE / ZENITH_DEFAULT_CAMERA_ACTIVE
	 * / ZENITH_DEFAULT_CAMERA_FLOOR_PCT.  camera_active is the
	 * userspace override (0 auto, 1 force-on, 2 force-off);
	 * camera_floor_pct ranges 0..100.
	 */
	unsigned int		camera_aware;
	unsigned int		camera_active;
	unsigned int		camera_floor_pct;

	/* See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO comment block.  Gates
	 * the in-kernel calm-detect arm of the boot_complete latch in
	 * zenith_auto_tune_work().  Default 1.  Setting to 0 disables
	 * the in-kernel arm; userspace writes to the boot_complete
	 * sysfs knob still work.
	 */
	unsigned int		boot_complete_auto;

	/* See ZENITH_DEFAULT_BOOT_BOOST_MS. 0 disables the one-shot. */
	unsigned int		boot_boost_ms;

	/* Trailing decay window for boot_boost, in milliseconds.  When 0
	 * (default), the boot-boost ends as a hard cliff at
	 * boot_boost_ms.  When non-zero, after boot_boost_ms expires
	 * zenith_get_next_freq() applies a linear floor that ramps from
	 * policy->max down to policy->min over boot_boost_decay_ms,
	 * mirroring input_boost_decay_ms's tail behaviour.  Useful when
	 * userspace boot animations or surface-flinger initial paints
	 * land just past boot_boost_ms and would otherwise see the
	 * sudden drop.  Capped at ZENITH_BOOT_BOOST_DECAY_MS_MAX.
	 */
	unsigned int		boot_boost_decay_ms;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US. Userspace writes the
	 * current vblank period in microseconds.  0 disables.
	 */
	unsigned int		frame_budget_us;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US_AUTO.  When 1, the adaptive
	 * frame-budget floor uses the cached drm-side vblank period
	 * (zenith_drm_vblank_us) instead of frame_budget_us /
	 * frame_budget_us_per_policy whenever the cached value is
	 * non-zero.  Lets a drm panel driver populate the rate without
	 * a userspace round-trip.  0 (default) preserves the legacy
	 * userspace-driven behaviour exactly.
	 */
	unsigned int		frame_budget_us_auto;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US/ frame_budget_us_per_policy
	 * comment block.  Indexed by cpumask_first(policy->cpus).  A
	 * non-zero entry overrides frame_budget_us for that policy;
	 * zero falls through to the global value.  All entries default
	 * to 0 (no override) at tunables_init() time.  Read lock-free
	 * in zenith_get_next_freq() under READ_ONCE; written under
	 * global_tunables_lock by the sysfs store path.
	 */
	unsigned int		frame_budget_us_per_policy[NR_CPUS];
	unsigned int		frame_pace_floor_pct;

	/* See ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE (Patch B9-1).
	 * Master 0/1 gate for the android_vh_arch_set_freq_scale
	 * vendor-hook observer.  When 0 the registered probe is a
	 * single-branch no-op; when 1 it caches the realised cluster
	 * freq scale and pre-arms peer_ramp on the local cluster.
	 * Read via READ_ONCE on the hot probe path; written via
	 * WRITE_ONCE from sysfs and from zenith_apply_profile().
	 */
	unsigned int		vh_arch_freq_scale_enable;

	/* See ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE (Patch B9-2).
	 * Master 0/1 gate for the android_vh_setscheduler_uclamp
	 * vendor-hook observer.  When 0 the registered probe is a
	 * single-branch no-op; when 1 a userspace uclamp_min raise on
	 * a task running in a zenith-driven policy synchronously arms
	 * peer_ramp on that policy's peer cluster (no PELT lag).
	 * Read via READ_ONCE on the probe path; written via
	 * WRITE_ONCE from sysfs and from zenith_apply_profile().
	 */
	unsigned int		vh_uclamp_observer_enable;

	/* See ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE (Patch B9-3).
	 * Master 0/1 gate for the android_vh_cpu_idle_enter /
	 * android_vh_cpu_idle_exit vendor-hook observer pair.  When 0
	 * both registered probes are single-branch no-ops; when 1 the
	 * exit probe stamps idle residency into z_policy->vh_cpu_idle_-
	 * last_residency_ns, and zenith_get_next_freq() suppresses
	 * cluster_wake_pulse arming when that residency exceeds
	 * ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS.  Read via READ_ONCE on
	 * both the probe path and the cwp arm site; written via
	 * WRITE_ONCE from sysfs and from zenith_apply_profile().
	 */
	unsigned int		vh_cpu_idle_enable;

	/* See ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE (Patch B9-3+).
	 * Master 0/1 gate for the android_vh_freq_qos_update_request
	 * vendor-hook observer.  When 0 the registered probe is a
	 * single-branch no-op and the auto-classify consume side
	 * short-circuits; when 1 a high-min FREQ_QOS update against a
	 * zenith-driven policy stamps vh_freq_qos_pressure_until_ns and
	 * zenith_auto_classify() pivots to PERFORMANCE while the
	 * timestamp is still ahead of ktime_get_ns().  Read via
	 * READ_ONCE on both the probe path and the auto-classify
	 * consumer; written via WRITE_ONCE from sysfs and from
	 * zenith_apply_profile().
	 */
	unsigned int		vh_freq_qos_enable;

	/* Per-tunables freq-QoS pressure window (Patch B9-3+).
	 * Set by zenith_probe_freq_qos_update_request() to
	 * ktime_get_ns() + ZENITH_VH_FREQ_QOS_WINDOW_MS * NSEC_PER_MSEC
	 * on each high-min FREQ_QOS hit; read by
	 * zenith_auto_classify() against ktime_get_ns() to determine
	 * whether the AUTO selector should bias to PERFORMANCE.
	 * atomic64_t so the producer (probe in arbitrary scheduler
	 * context) and consumer (auto-eval worker, sleepable context)
	 * race-free without a governor lock.  Initialised to 0 by
	 * kzalloc(); 0 means "no pressure" because every real hit
	 * stamps a value > 0.
	 */
	atomic64_t		vh_freq_qos_pressure_until_ns;

	/* See ZENITH_DEFAULT_VH_SCHED_MOVE_TASK_ENABLE (Patch B9-5).
	 * Master 0/1 gate for the android_vh_sched_move_task vendor-
	 * hook observer.  When 0 the registered probe is a single-
	 * branch no-op; when 1 every cgroup move that lands a task on
	 * a CPU belonging to a zenith-driven policy stamps jiffies on
	 * z_policy->vh_sched_move_task_last_jiffies.  Read via
	 * READ_ONCE on the probe path; written via WRITE_ONCE from
	 * sysfs and from zenith_apply_profile().
	 */
	unsigned int		vh_sched_move_task_enable;

	/* See ZENITH_DEFAULT_VH_SCHEDULER_TICK_ENABLE (Patch B9-4).
	 * Master 0/1 gate for the android_vh_scheduler_tick vendor-
	 * hook observer.  When 0 the registered probe is a single-
	 * branch no-op; when 1 every scheduler tick on a CPU belonging
	 * to a zenith-driven policy stamps ktime_get_ns() on
	 * z_cpu->vh_scheduler_tick_last_ns and bumps z_cpu->vh_-
	 * scheduler_tick_count.  Read via READ_ONCE on the probe
	 * (HZ * num_CPUs hot path); written via WRITE_ONCE from sysfs
	 * and from zenith_apply_profile().
	 */
	unsigned int		vh_scheduler_tick_enable;

	/* Patch L: see ZENITH_DEFAULT_VERBOSE_LOG comment block.
	 * Operator-controlled gate for the human-readable zenith
	 * dmesg / logcat trail.  Read via READ_ONCE on the (cold)
	 * sysfs and profile_store paths; written via WRITE_ONCE
	 * from the verbose_log_store handler.
	 */
	unsigned int		verbose_log;

	/* Patch K: game / sustained-high-load performance burst.  See
	 * ZENITH_DEFAULT_GAME_PERF_BURST and the long comment block
	 * above ZENITH_GPB_STATE_IDLE for the full mechanism.  Five
	 * cooperating fields:
	 *
	 *   game_perf_burst                       master 0/1, default 1
	 *                                         (also gates the
	 *                                          zenith_game_perf_burst_-
	 *                                          key static branch)
	 *   game_perf_burst_floor_pct             freq floor while ARMED,
	 *                                         applied as
	 *                                         policy->max * pct / 100
	 *                                         in zenith_get_next_freq()
	 *   game_perf_burst_thermal_ceiling_dc    skin-temp guardrail,
	 *                                         millidegrees C
	 *                                         (matches the kernel
	 *                                          thermal subsystem unit)
	 *   game_perf_burst_disarm_grace_ms       sustained-clear hold
	 *                                         before transitioning out
	 *                                         of ARMED on Signal-B drop
	 *   game_perf_burst_cooldown_ms           length of the COOLDOWN
	 *                                         glide that linearly
	 *                                         steps the floor down to 0
	 *
	 * All five are read on the hot path inside
	 * ZENITH_FEATURE_ENABLED(game_perf_burst); writes are via
	 * WRITE_ONCE in their respective sysfs handlers.  Not profile-
	 * baked because the burst mechanic is workload-detection driven,
	 * not preset state -- presets that wanted to bias the floor would
	 * have to write to game_perf_burst_floor_pct explicitly.
	 */
	unsigned int		game_perf_burst;
	unsigned int		game_perf_burst_floor_pct;
	unsigned int		game_perf_burst_thermal_ceiling_dc;
	unsigned int		game_perf_burst_disarm_grace_ms;
	unsigned int		game_perf_burst_cooldown_ms;
};

/*
 * Set by the input handler on every key/abs event. Read from the hot path
 * with atomic64_read so no governor lock is needed in the producer.
 */
static atomic64_t zenith_input_boost_until_ns = ATOMIC64_INIT(0);

/*
 * Wall-clock timestamp of the last input event observed by
 * zenith_input_event.  Producer: zenith_input_event itself, on every
 * EV_KEY / EV_ABS / EV_REL event.  Consumer: also zenith_input_event,
 * to detect a "first input after quiet period" and grant that first
 * event an extended full-pin window (see ZENITH_INPUT_QUIET_*).
 *
 * Initialised to 0 so the very first event after boot is always
 * treated as a quiet-period entry.  Maintained as atomic64 so the
 * producer can read-then-write without holding any lock; multiple
 * input devices can fire concurrently with no consistency hazard
 * worse than two adjacent events both deciding the gap was long
 * enough to extend, which is harmless.
 */
static atomic64_t zenith_input_last_event_ns = ATOMIC64_INIT(0);

/*
 * Peer-ramp deadlines (Patch D).  One slot per BIG/PRIME class.
 * The slot named for class X holds the deadline that X should
 * apply -- i.e. it is *written by X's peer* when the peer ramps,
 * and *read by X* on its next eval.  Concretely:
 *
 *   BIG ramps   -> writes zenith_peer_ramp_until_ns_prime
 *   PRIME ramps -> writes zenith_peer_ramp_until_ns_big
 *   BIG eval    -> reads  zenith_peer_ramp_until_ns_big
 *   PRIME eval  -> reads  zenith_peer_ramp_until_ns_prime
 *
 * LITTLE neither writes nor reads.  See section "peer_ramp_*"
 * in the macro block at the top of the file for the rationale.
 *
 * Both stay 0 until the first ramp; deadlines are a forward
 * wall-clock ns and naturally expire as ktime_get_ns() advances
 * past them.  No reset path is needed: the read side is just a
 * "is now < until" compare.
 */
static atomic64_t zenith_peer_ramp_until_ns_big   = ATOMIC64_INIT(0);
static atomic64_t zenith_peer_ramp_until_ns_prime = ATOMIC64_INIT(0);

/* Frame-overrun rescue (Patch K3) -- file-scope atomics.
 *
 * zenith_last_vblank_ns: timestamp of the previous
 * zenith_drm_vblank_event() call.  0 means "no previous call",
 * which the producer treats as "first vblank, just record and
 * return".  Cleared (set back to 0) by the screen-state stale
 * guard so a sleep / blank period doesn't leave a stale
 * timestamp that would look like a multi-second overrun on
 * resume.
 *
 * zenith_frame_overrun_until_ns: deadline.  Stamped by the
 * producer when an overrun is detected, read by every cluster's
 * floor tier in zenith_get_next_freq() to apply a soft floor.
 * Self-disarms by deadline; ktime_get_ns() advancing past the
 * value is the disarm.
 *
 * Both are governor-wide (not per-policy) because frame
 * overruns are observed at the display layer, which sits above
 * the cpufreq policy partitioning -- a missed frame benefits
 * from lifting both BIG and PRIME, not just one.
 */
static atomic64_t zenith_last_vblank_ns          = ATOMIC64_INIT(0);
static atomic64_t zenith_frame_overrun_until_ns  = ATOMIC64_INIT(0);

/* Frame-overrun deep tier (Patch M5) -- governor-wide streak
 * counter.  Bumped by zenith_drm_vblank_event() each time a
 * vblank gap exceeds the budget; reset to 0 when a vblank
 * arrives within budget.  Read by the K3 block in
 * zenith_get_next_freq() and compared against the per-policy
 * frame_overrun_deep_streak knob.  Same governor-wide-vs-per-
 * policy reasoning as the deadline atomic above: frame events
 * sit at the display layer above cluster partitioning, and a
 * sustained-overrun signal benefits both clusters.
 *
 * Initialised to 0; a fresh policy attach therefore starts in
 * the non-deep-tier state regardless of any previous policy's
 * history, which is the conservative default.  Never decrements
 * other than the within-budget reset, so no torn-read protection
 * needed beyond atomic_t's natural alignment guarantee.
 */
static atomic_t zenith_frame_overrun_streak       = ATOMIC_INIT(0);

/*
 * Cached drm-panel vblank period, in microseconds.  Producer:
 * display drivers / panel bridges call zenith_set_drm_vblank_us()
 * whenever the active vblank period changes (e.g. on a 60->120Hz
 * mode switch in DRM).  Consumer: zenith_get_next_freq()'s
 * adaptive frame-budget floor when tunables->frame_budget_us_auto
 * is non-zero.  Zero means "no driver reported yet"; the auto path
 * then falls back to the userspace-set frame_budget_us so existing
 * tunings keep working.
 *
 * Lockless read on the consumer side; producers use atomic_set()
 * with no ordering requirements other than "the latest write wins".
 */
static atomic_t zenith_drm_vblank_us = ATOMIC_INIT(0);

/* Boot-completion latch.  See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO
 * comment block above struct zenith_tunables.  Both globals are
 * read lock-free from zenith_get_next_freq() (the boost path) and
 * written either from the boot_complete sysfs *_store callback or
 * from zenith_auto_tune_work() once the calm streak qualifies.
 */
static atomic_t zenith_boot_complete = ATOMIC_INIT(0);
static u64 zenith_boot_complete_ns;

/* AC-vs-battery state cache (Patch 1.2).  Refreshed once per
 * auto_tune window (ZENITH_AUTO_TUNE_PERIOD_MS, default 10 s) by
 * zenith_auto_tune_work() via power_supply_is_system_supplied():
 *   0  -> AC power, system-supplied (default)
 *   1  -> running on battery
 * Read lock-free from zenith_batt_scaled() in the peak-rescue and
 * peak-prearm hot paths to scale hold-down deadlines when the
 * batt_hold_scale_pct profile knob requests it.  When no power
 * supply driver is registered the helper returns -ENODEV /
 * -ENOSYS and the cache stays at 0 (AC), so the default path
 * exactly preserves pre-1.2 behaviour on systems without a battery
 * (laptops on a dock, lab boards, AVDs).
 */
static atomic_t zenith_on_battery = ATOMIC_INIT(0);

/* In-kernel game detector global latch.  See the
 * ZENITH_DEFAULT_GAME_AUTO comment block.  Read lock-free via
 * READ_ONCE() from zenith_eff_game_mode() (the helper consumed by
 * the hot-path readers of game_mode), and written either from the
 * hot-path detector zenith_policy_game_auto_tick() or from the
 * game_auto sysfs *_store callback (which clears the latch on
 * disable so a stale latch does not survive game_auto = 0).
 *
 * The value is the boottime nanosecond deadline at which the latch
 * expires.  Zero means "no game detected".  The compare/expire
 * check is "now < latch", so wraparound is not a concern in any
 * realistic uptime.
 */
static u64 zenith_game_auto_active_until_ns;

/* Atomic notifier chain for game-mode state transitions.  Fired
 * whenever the effective game mode transitions 0->1 or 1->0, whether
 * triggered by a user sysfs write or by the V2 auto-detector.  The
 * action parameter passed to notifier callbacks is 1 (game mode
 * entered) or 0 (game mode exited).  External drivers (GPU switch,
 * thermal, etc.) register a callback and can block for immediate
 * policy synchronisation.
 *
 * The chain is ATOMIC so it is safe to fire from any context,
 * including the scheduler hot path where the V2 auto-detector runs.
 * Callbacks must not sleep.
 */
static ATOMIC_NOTIFIER_HEAD(zenith_game_mode_nh);

/* True if the in-kernel game detector latch is currently in the
 * future, i.e. a recent fresh detection has happened and is still
 * within ZENITH_GAME_AUTO_ACTIVE_TTL_NS.  Lock-free; readers tolerate
 * a stale value by at most one cpufreq tick.
 */
static bool zenith_game_auto_active(void)
{
	u64 until = READ_ONCE(zenith_game_auto_active_until_ns);

	if (!until)
		return false;
	return ktime_get_ns() < until;
}

/* Returns the effective game_mode used by all hot-path overlays:
 *   max(user_or_v2_game_mode, in-kernel auto detector)
 *
 * base_gm is whatever the existing zenith_tunable_or_local() pair
 * (t->game_mode vs z_policy->at_effective_game_mode) returned -- the
 * auto detector only ever bumps an under-1 result up to 1.  Higher
 * user / V2 values are preserved verbatim.  The static-branch gate
 * means a no-op single never-taken jump when game_auto = 0.
 */
static inline unsigned int zenith_eff_game_mode(unsigned int base_gm)
{
	if (static_branch_likely(&zenith_game_auto_key) &&
	    base_gm < 1 &&
	    zenith_game_auto_active())
		return 1;
	return base_gm;
}
/**
 * zenith_is_game_mode_active - query whether Zenith auto-detected a game
 *
 * Returns true when the in-kernel game-engine thread detector has
 * identified a game workload and the auto-detection latch is still
 * valid.  Intended for external drivers (GPU, thermal, etc.) that
 * want to synchronise their own policy with Zenith's game mode.
 *
 * Safe to call from any context.  Returns false when the governor
 * is not built, when game_auto is disabled, or when the latch has
 * expired -- same fail-safe shape as every other exported hook.
 */
bool zenith_is_game_mode_active(void)
{
	if (!static_branch_likely(&zenith_game_auto_key))
		return false;
	return zenith_game_auto_active();
}
EXPORT_SYMBOL_GPL(zenith_is_game_mode_active);

/**
 * zenith_register_game_mode_notifier - register a notifier for game mode
 *                                       transitions
 * @nb: notifier_block to register (callback receives action=1 on enter,
 *      action=0 on exit)
 *
 * Register a callback that fires when the effective game mode changes.
 * The callback runs in atomic context (RCU-read-side) and must not
 * sleep.  Typically the callback will schedule_work() to defer
 * heavyweight operations (governor switch, VM tuning) to process
 * context.
 */
int zenith_register_game_mode_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&zenith_game_mode_nh, nb);
}
EXPORT_SYMBOL_GPL(zenith_register_game_mode_notifier);

/**
 * zenith_unregister_game_mode_notifier - unregister a game mode notifier
 * @nb: notifier_block previously registered
 */
int zenith_unregister_game_mode_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&zenith_game_mode_nh, nb);
}
EXPORT_SYMBOL_GPL(zenith_unregister_game_mode_notifier);

/**
 * zenith_set_drm_vblank_us - publish active panel vblank period to zenith
 * @us: vblank period in microseconds; 0 clears the cache.
 *
 * Display drivers / drm-panel bridges call this whenever the active
 * panel's vblank period changes (e.g. 60 -> 120Hz switch via
 * drm_atomic_commit_tail).  Lock-free; safe to call from any context
 * including atomic.  Values larger than ZENITH_FRAME_BUDGET_US_MAX
 * (50ms, ~20Hz) are silently clamped down because anything above
 * that is past the useful rate-shaping range and would push the
 * eff_pct calculation in the consumer to 0 anyway.
 */
void zenith_set_drm_vblank_us(unsigned int us)
{
	if (us > ZENITH_FRAME_BUDGET_US_MAX)
		us = ZENITH_FRAME_BUDGET_US_MAX;
	atomic_set(&zenith_drm_vblank_us, (int)us);
}
EXPORT_SYMBOL_GPL(zenith_set_drm_vblank_us);

/* Audit fix K4: v4l2 fd-open hook callbacks.  Strong symbols that
 * override the weak stubs in drivers/media/v4l2-core/v4l2-dev.c.
 *
 * Both callbacks are called outside of v4l2's videodev_lock so they
 * can run on any context cheaply (just an atomic_inc / atomic_dec).
 * No filtering by vfl_type or v4l2_dev capabilities -- any
 * /dev/video* open counts.  See the zenith_v4l2_active_fds comment
 * for the false-positive analysis (short answer: harmless, all
 * v4l2 capture workloads benefit from LATENCY).
 *
 * Exported so the v4l2 driver (which is built from a different
 * compilation unit) can resolve them via weak-symbol override.  No
 * other in-kernel caller is expected; the EXPORT_SYMBOL_GPL is for
 * the link-time relaxation, not for module use.
 */
/* Opaque forward declaration: zenith does not look inside vdev, just
 * passes it through.  Avoids dragging linux/videodev2.h into a
 * cpufreq governor TU.
 */
struct video_device;

/* Forward declarations of the K4/K5 vendor-hook strong defs below.
 * The matching __weak prototypes live inline at the call sites
 * (drivers/media/v4l2-core/v4l2-dev.c and sound/core/pcm_native.c)
 * so callers stay zero-knowledge of the governor.  Declaring them
 * here makes the local definitions visible to -Wmissing-prototypes
 * and sparse, and ensures cpufreq_zenith.c's view of the prototype
 * is internally consistent.
 */
void zenith_v4l2_open_notify(struct video_device *vdev);
void zenith_v4l2_release_notify(struct video_device *vdev);
void zenith_alsa_pcm_open_notify(int stream);
void zenith_alsa_pcm_release_notify(int stream);

/* Tentative declarations of the K4/K5 refcount atomics.  Their
 * defining declarations (with ATOMIC_INIT(0)) live further down the
 * file next to the rest of the auto-tune state; the function bodies
 * below were originally written assuming forward visibility, which
 * the C tentative-definition rule grants only when the file-scope
 * tentative is visible at first use.  These two lines provide that
 * tentative visibility so the open / release notify functions
 * compile cleanly under CONFIG_CPU_FREQ_GOV_ZENITH=y.
 */
static atomic_t zenith_v4l2_active_fds;
static atomic_t zenith_alsa_active_fds;

void zenith_v4l2_open_notify(struct video_device *vdev)
{
	(void)vdev;	/* unused; we don't filter by vfl_type yet */
	atomic_inc(&zenith_v4l2_active_fds);
}
EXPORT_SYMBOL_GPL(zenith_v4l2_open_notify);

void zenith_v4l2_release_notify(struct video_device *vdev)
{
	int v;

	(void)vdev;
	v = atomic_dec_return(&zenith_v4l2_active_fds);
	if (unlikely(v < 0))
		atomic_set(&zenith_v4l2_active_fds, 0);
}
EXPORT_SYMBOL_GPL(zenith_v4l2_release_notify);

/* K5: ALSA pcm-open / pcm-release notify hooks.  Same shape as K4.
 * The `stream` parameter is the SNDRV_PCM_STREAM_PLAYBACK / CAPTURE
 * direction; we don't currently distinguish them (any active
 * stream counts), but the parameter is preserved for forward
 * compatibility (a future patch could split capture vs playback
 * for capture-only audio scenarios like voice memos).
 */
void zenith_alsa_pcm_open_notify(int stream)
{
	(void)stream;
	atomic_inc(&zenith_alsa_active_fds);
}
EXPORT_SYMBOL_GPL(zenith_alsa_pcm_open_notify);

void zenith_alsa_pcm_release_notify(int stream)
{
	int v;

	(void)stream;
	v = atomic_dec_return(&zenith_alsa_active_fds);
	if (unlikely(v < 0))
		atomic_set(&zenith_alsa_active_fds, 0);
}
EXPORT_SYMBOL_GPL(zenith_alsa_pcm_release_notify);

/* Governor-wide caches for the frame-overrun knobs (Patch K3).
 * The producer (zenith_drm_vblank_event()) runs from the display
 * driver context with no struct zenith_policy in scope; if the
 * arming logic needed a tunables lookup it would have to walk
 * the policy list.  Mirror the active values into file-scope
 * unsigned ints instead, written by sysfs store and
 * zenith_apply_profile(); same shape as
 * zenith_input_boost_active_ms above.
 *
 * Multiple policies sharing one governor-wide cache is fine
 * because the per-policy frame_overrun_slack_us /
 * frame_overrun_window_ms values are expected to be uniform
 * across clusters -- they describe a property of the display,
 * not of a specific cluster.
 */
static unsigned int zenith_frame_overrun_slack_us_cache =
	ZENITH_DEFAULT_FRAME_OVERRUN_SLACK_US;
static unsigned int zenith_frame_overrun_window_ms_cache =
	ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS;

/**
 * zenith_drm_vblank_event - notify zenith of a panel vblank
 *
 * Display drivers / drm-panel bridges call this from the per-
 * vblank IRQ handler so the governor can detect frame budget
 * overruns -- e.g. compositor + render thread missed a frame
 * and the next vblank arrives a full extra period late.
 *
 * Lock-free; safe to call from any context including IRQ.  When
 * the panel driver does not call this, the entire detection
 * path is a no-op (the deadline atomic stays at 0 and the
 * floor tier never fires).  Same fail-safe shape as
 * zenith_set_drm_vblank_us().
 *
 * The first call after policy attach (or after the screen-
 * state stale guard clears zenith_last_vblank_ns) just records
 * the timestamp and returns.  On every subsequent call the
 * elapsed wall-clock delta is compared against the cached
 * vblank period plus tunables->frame_overrun_slack_us; if the
 * gap is wider, the renderer missed the frame budget and a
 * deadline is stamped on zenith_frame_overrun_until_ns.
 */
void zenith_drm_vblank_event(void)
{
	unsigned int slack_us =
		READ_ONCE(zenith_frame_overrun_slack_us_cache);
	unsigned int window_ms;
	unsigned int period_us;
	u64 now_ns = ktime_get_ns();
	u64 last_ns = (u64)atomic64_read(&zenith_last_vblank_ns);
	u64 delta_ns;

	atomic64_set(&zenith_last_vblank_ns, (s64)now_ns);
	if (!slack_us || !last_ns || now_ns <= last_ns)
		return;
	period_us = (unsigned int)atomic_read(&zenith_drm_vblank_us);
	if (!period_us)
		return;
	delta_ns = now_ns - last_ns;
	if (delta_ns <= ((u64)period_us + slack_us) * NSEC_PER_USEC) {
		/* Within budget -- reset the deep-tier streak (Patch M5).
		 * A single good frame breaks any "sustained" pattern the
		 * deep tier was tracking, so the deep floor should
		 * back off on the very next eval.
		 */
		atomic_set(&zenith_frame_overrun_streak, 0);
		return;
	}
	window_ms = READ_ONCE(zenith_frame_overrun_window_ms_cache);
	if (!window_ms)
		return;
	atomic64_set(&zenith_frame_overrun_until_ns,
		     (s64)(now_ns + (u64)window_ms * NSEC_PER_MSEC));
	/* Track consecutive overruns for the deep-tier consumer in
	 * zenith_get_next_freq() (Patch M5).  Bumping is post-stamp
	 * so window_ms == 0 (K3 floor suppressed but stamping still
	 * occurs) does not feed the deep tier either: deep is an
	 * amplification of K3 and inherits its arming gate.
	 */
	atomic_inc(&zenith_frame_overrun_streak);
}
EXPORT_SYMBOL_GPL(zenith_drm_vblank_event);
/**
 * zenith_gpu_load_event - notify zenith of GPU load change
 * @gpu_load_pct: GPU utilization percentage (0-100)
 *
 * Called by the GPU driver (e.g. MSM DRM devfreq get_dev_status)
 * on every devfreq polling tick (~10ms).  When GPU load crosses
 * ZENITH_GPU_LOAD_THRESH_PCT, stamp a short input boost so the
 * CPU clusters are pre-emptively raised before PELT catches up
 * with the workload that generated the GPU load.
 *
 * Lock-free; safe to call from any context including atomic.
 * Stale/zero thresholds gate the stamping, so a driver that
 * stops calling this simply returns the governor to the legacy
 * PELT-only path -- same fail-safe shape as
 * zenith_set_drm_vblank_us() / zenith_drm_vblank_event().
 *
 * Rather than introducing a new file-scope deadline and a new
 * floor tier in zenith_get_next_freq(), we reuse the existing
 * zenith_input_boost_until_ns mechanism so that GPU-intensive
 * workloads benefit from the same cluster-boost path as touch
 * input.  The 10 ms devfreq polling cadence re-arms the
 * deadline on every tick while the GPU stays busy, so the CPU
 * remains boosted for the duration of the GPU workload.
 */
#define ZENITH_GPU_LOAD_THRESH_PCT	50
#define ZENITH_GPU_LOAD_WINDOW_MS	50

void zenith_gpu_load_event(unsigned int gpu_load_pct)
{

	u64 now_ns, deadline, boost_until;
	/* Only fire when GPU is meaningfully loaded */
	if (gpu_load_pct < ZENITH_GPU_LOAD_THRESH_PCT)

		return;
	now_ns = ktime_get_ns();

	deadline = now_ns + (u64)ZENITH_GPU_LOAD_WINDOW_MS * NSEC_PER_MSEC;
	boost_until = (u64)atomic64_read(&zenith_input_boost_until_ns);
	/* Only extend the deadline, never shorten it */

	if (deadline > boost_until)
		atomic64_set(&zenith_input_boost_until_ns, (s64)deadline);
}
EXPORT_SYMBOL_GPL(zenith_gpu_load_event);


/**
 * zenith_gpu_freq_event - notify zenith of high GPU frequency
 * @freq_pct: GPU frequency as percentage of max (0-100)
 *
 * Companion to zenith_gpu_load_event().  While load-based boost
 * reacts to current GPU utilization, freq-based boost anticipates
 * future CPU demand: a GPU that has ramped to a high OPP is a
 * leading indicator that a render workload is about to hit the CPU
 * (frame submission, buffer sync).
 *
 * Uses a longer window (80ms vs 50ms) because the predictive
 * signal needs to bridge the gap between GPU ramp-up and CPU
 * workload arrival, which can span multiple devfreq ticks.
 *
 * Lock-free; safe to call from any context including atomic.
 * Same fail-safe shape as zenith_gpu_load_event().
 */
#define ZENITH_GPU_FREQ_THRESH_PCT	70
#define ZENITH_GPU_FREQ_WINDOW_MS	80

void zenith_gpu_freq_event(unsigned int freq_pct)
{

	u64 now_ns, deadline, boost_until;
	/* Only fire when GPU is at a high OPP */
	if (freq_pct < ZENITH_GPU_FREQ_THRESH_PCT)
		return;
	now_ns = ktime_get_ns();
	deadline = now_ns + (u64)ZENITH_GPU_FREQ_WINDOW_MS * NSEC_PER_MSEC;
	boost_until = (u64)atomic64_read(&zenith_input_boost_until_ns);
	/* Use a longer window than load-based boost */
	if (deadline > boost_until)
		atomic64_set(&zenith_input_boost_until_ns, (s64)deadline);
}
EXPORT_SYMBOL_GPL(zenith_gpu_freq_event);

static unsigned int zenith_input_boost_active_ms = ZENITH_DEFAULT_INPUT_BOOST_MS;

/* Governor-wide cache for the touchdown-extra knob (Patch C).
 * Mirrored from t->input_boost_touchdown_extra_ms by sysfs store
 * and zenith_apply_profile().  Read by zenith_input_event() on
 * the EV_KEY/BTN_TOUCH down path.  Stored as plain unsigned int
 * with READ_ONCE/WRITE_ONCE; the racy reader doesn't care if it
 * sees a stale value across the store window.
 */
static unsigned int zenith_input_boost_touchdown_extra_ms_cache =
	ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS;

/* Monotonically-increasing global count of qualifying input events seen
 * by zenith_input_event. Auto-tune workers sample this periodically and
 * subtract their last-observed value to get an events-per-window rate.
 */
static atomic64_t zenith_auto_input_events = ATOMIC64_INIT(0);

/* Audit fix K4: deterministic camera detection via v4l2 fd-open hook.
 *
 * The runqueue-snapshot comm-walk in zenith_policy_has_camera() is a
 * probabilistic signal -- camera HALs that sleep most of the time
 * (cameraserver, camerahalserver) are rarely on-CPU when the walk
 * runs, so the cache fills with `false` and the camera flag never
 * fires.  K4 wires zenith into v4l2-core via two weak-symbol notify
 * callbacks (drivers/media/v4l2-core/v4l2-dev.c).  Every successful
 * v4l2 fd open bumps this refcount; release decrements.  Any nonzero
 * value is treated by the auto_tune scenario block as "camera active"
 * regardless of the comm-walk result.
 *
 * Counter is signed so a tearing race during release that would
 * otherwise underflow to UINT_MAX is observable as a negative value
 * during diagnosis instead of silently looking like a stuck camera.
 * The notify functions clamp to >= 0 on every store.
 *
 * False-positive surface: any process that opens /dev/videoN
 * (USB webcam, screen recorder sink, software encoder using v4l2
 * codec node) is also accounted.  All such workloads are similarly
 * media-bandwidth-bound and benefit from the LATENCY state, so the
 * "false positive" actually does the right thing.
 */
static atomic_t zenith_v4l2_active_fds = ATOMIC_INIT(0);

/* Audit fix K5: deterministic audio detection via ALSA pcm-open hook.
 *
 * Sibling to K4 -- same atomic refcount pattern, drives the audio
 * scenario flag.  See zenith_alsa_pcm_open_notify() below for the
 * exported strong symbols, and sound/core/pcm_native.c for the
 * weak symbols that hook into snd_pcm_*_open / snd_pcm_release.
 *
 * Why bother when audio_server already lights up the comm-walk
 * reliably:
 *   - first window after open: audio_server may not have started
 *     mixing yet, comm-walk misses, audio flag stays 0 for a full
 *     V1 window.
 *   - last window after close: audio_server already drained, but
 *     V1 doesn't know the stream is gone until the next walk.
 *
 * K5 closes both edges to single-tick precision.
 */
static atomic_t zenith_alsa_active_fds = ATOMIC_INIT(0);
#define ZENITH_AUTO_TUNE_PERIOD_MS	5000	/* classify every 5s   */

/* Audit fix F1: scenario-active classifier window.
 *
 * When any scenario flag (camera, render, frame, game, memstall,
 * thermal_slope, psi_cpu) is set, drop the V1 classifier reschedule
 * cadence from the default 10 s down to ~1.5 s.  Real-world bursty
 * workloads (camera open, app launch, scroll) finish in 1-3 s on
 * modern phone-class hardware; the 10 s default means V1 runs
 * exactly once during the burst, sees a half-saturated window, and
 * picks BALANCED -- making LATENCY commit only after the burst is
 * already over.
 *
 * This faster window only applies to the *V1 reschedule* cadence;
 * the V2 hysteresis windows continue to count in the same units (so
 * a 2-window hysteresis is now ~3 s instead of ~20 s).  V3
 * calibration interval is unchanged because V3 already has its own
 * timer (auto_tune_v3_interval_ms).
 */
#define ZENITH_AUTO_TUNE_FAST_PERIOD_MS	1500

/* Stage 4 / Patch I -- governor-wide input observability counters.
 *
 * Each counter is a monotonic atomic64; readers get a snapshot via
 * the `zenith_input_stats` sysfs node and subtract last-observed
 * values to compute rates.  Counters never reset; rollover on a 64-
 * bit atomic is "never" in practice.  The cost on the input path is
 * one atomic64_inc per counter touched; in the trace-disabled hot
 * path that's three increments per qualifying event, all cheap.
 *
 * Counters:
 *
 *   zenith_in_events_total
 *     Every EV_KEY/EV_ABS/EV_REL event seen by zenith_input_event,
 *     regardless of whether the boost is enabled.
 *   zenith_in_boosts_armed
 *     Events that wrote zenith_input_boost_until_ns (i.e.
 *     active != 0 at event time).
 *   zenith_in_boosts_quiet_extended
 *     Subset of armed boosts where the quiet-period extension
 *     widened the boost window past the configured active duration.
 *   zenith_in_boosts_skipped_disabled
 *     Events that fell through because input_boost_ms == 0
 *     (boost feature disabled at event time).
 *   zenith_in_boosts_early_exit
 *     [Stage 4 / Patch F] Per-policy decisions where the
 *     persistent-idle streak (boost_idle_low_streak) crossed
 *     the boost_idle_streak threshold and the policy preempted
 *     the still-armed input boost on its tier-0 path this tick.
 *     The global zenith_input_boost_until_ns is left untouched;
 *     other policies continue to honour the boost.
 */
static atomic64_t zenith_in_events_total = ATOMIC64_INIT(0);
static atomic64_t zenith_in_boosts_armed = ATOMIC64_INIT(0);
static atomic64_t zenith_in_boosts_quiet_extended = ATOMIC64_INIT(0);
static atomic64_t zenith_in_boosts_skipped_disabled = ATOMIC64_INIT(0);
static atomic64_t zenith_in_boosts_early_exit = ATOMIC64_INIT(0);

/* Per-policy decision-stat buckets exposed via the readonly
 * `zenith_stats` sysfs node.  See struct zenith_policy::stats[] for
 * the storage and zenith_path_to_bucket() for the tp_path -> bucket
 * mapping.  ZENITH_STAT_OTHER is a catch-all so a future tier added
 * without a bucket mapping still gets counted (against decisions,
 * but not against any specific tier).
 */
enum zenith_stat_idx {
	ZENITH_STAT_DECISIONS,		/* total get_next_freq() calls */
	ZENITH_STAT_CACHE_HITS,		/* cached_raw_freq early-return */
	ZENITH_STAT_INPUT_BOOST,	/* input_boost / input_boost_decay */
	ZENITH_STAT_BRUTAL,		/* snap_max / brutal_hold / climb_step */
	ZENITH_STAT_HISPEED,		/* hispeed */
	ZENITH_STAT_FRAME_PACE,		/* frame_pace */
	ZENITH_STAT_AUDIO,		/* audio_floor / audio_cap */
	ZENITH_STAT_RENDER_CAMERA,	/* render_floor / camera_floor */
	ZENITH_STAT_UCLAMP,		/* uclamp_min_floor / uclamp_max_cap */
	ZENITH_STAT_PSI,		/* psi_*_cap */
	ZENITH_STAT_BOOT_BOOST,		/* boot_boost */
	ZENITH_STAT_LIGHT_CAP,		/* light_cap */
	ZENITH_STAT_EM_CAP,		/* em_cap */
	ZENITH_STAT_EAS,		/* fall-through, no override */
	ZENITH_STAT_OTHER,		/* unmapped tp_path */
	/* Stage 4 / Patch I additions.  Split out the tp_paths
	 * previously bucketed into ZENITH_STAT_OTHER (predict_up,
	 * peak_prearm, peak_rescue) so testers can read the
	 * fire-rate of each Stage-3+ tier independently of the
	 * legacy buckets.
	 */
	ZENITH_STAT_PREDICT_UP,		/* predict_up */
	ZENITH_STAT_PEAK_PREARM,	/* peak_prearm */
	ZENITH_STAT_PEAK_RESCUE,	/* peak_rescue */
	ZENITH_STAT_PEAK_HYST,		/* peak_hyst (Patch E) */
	ZENITH_STAT_PEER_RAMP,		/* peer_ramp (Patch D) */
	ZENITH_STAT_MIGRATION_FLOOR,	/* migration_floor (Patch K1) */
	ZENITH_STAT_PSI_CPU_FLOOR,	/* psi_cpu_floor (Patch K2) */
	ZENITH_STAT_FRAME_OVERRUN,	/* frame_overrun (Patch K3) */
	ZENITH_STAT_AUTO_THERMAL_CAP,	/* auto_thermal_cap (Path B) */
	ZENITH_STAT_QUIET_HOURS_CAP,	/* quiet_hours_cap (Patch 1.10) */
	ZENITH_STAT_NR
};

/* One sample written by the auto-tune classifier worker into the
 * per-policy at_log ring (see ZENITH_AT_LOG_NR).  Mirrors the
 * at_last_* mirrors on struct zenith_policy at the moment the worker
 * resolved the new V1 target / V2 state, so userspace can correlate a
 * decision against the signals that drove it without bpftrace.
 */
struct zenith_at_log_entry {
	u64		ts_ns;
	u32		flags;			/* ZENITH_AT_FLAG_* mask */
	u32		var_x256;
	u32		thermal_pressure;	/* 0..1024 */
	u16		sat_pct;		/* 0..100 */
	u16		events_rate_x2;
	u16		thermal_slope;		/* signed-stored-as-u16 */
	u8		v1_target;		/* enum profile id */
	u8		v2_from_state;
	u8		v2_to_state;
	u8		reason;			/* enum at_reason */
	u8		emergency;		/* 1 if state_overridden by slope */
	u8		_pad[3];
};

/* Patch J: per-policy V3 calibration ring entry.  One slot per
 * zenith_at_v3_calibrate() invocation, recording the boottime ns,
 * the V2 transition count counted from the at_log walk, the V3 mode
 * the calibration ran under (OBSERVE / APPLY -- mode 0 OFF never
 * pushes), and the hyst/cool offset before and after any APPLY-mode
 * nudge.  before == after on OBSERVE-mode entries and on
 * rail-clamped APPLY entries; the differential lets the reader
 * trivially see "when did V3 actually move me" without bpftrace.
 */
struct zenith_at_v3_calib_log_entry {
	u64		ts_ns;
	u32		transitions;
	u8		mode;			/* ZENITH_AT_V3_MODE_* */
	s8		hyst_before;
	s8		hyst_after;
	s8		cool_before;
	s8		cool_after;
	u8		_pad[3];
};

struct zenith_policy {
	struct cpufreq_policy	*policy;
	struct zenith_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			min_rate_limit_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;

	struct irq_work		irq_work;
	struct kthread_work	work;
	/* Serialises the slow-path kthread_work against teardown */
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;

	/* Per-bin unlock deadlines for the multi-step efficient_freq
	 * ladder. eff_unlock_at_ns[i] is the ktime_get_ns() time after
	 * which target_freq is allowed past tunables->eff_freq[i].
	 * Zero means "idle" — not currently gating. Indexed 0..eff_nr-1.
	 */
	u64			eff_unlock_at_ns[ZENITH_EFF_BINS_MAX];

	/* Multiplier currently applied to down_rate_delay_ns. Bumped to
	 * tunables->sampling_down_factor while sitting at policy->max,
	 * reset to 1 the moment we leave max. Mirrors ondemand.
	 */
	unsigned int		down_rate_mult;

	/* True once load crossed up_threshold. Stays true, holding us at
	 * policy->max, until load drops below down_threshold. Collapses
	 * to old snap-on-every-sample behaviour when
	 * down_threshold >= up_threshold.
	 */
	bool			brutal_active;

	/* Hysteresis state for the hispeed tier (step 2b).  Sticky: goes
	 * true when load_pct first crosses hispeed_load with freq below
	 * the effective hispeed floor; stays true while load_pct stays
	 * above (hispeed_load - hispeed_hyst_pct).  Drops back to false
	 * only when the margin is crossed.  Prevents the tier from
	 * flapping in/out on noisy load trajectories that dance around
	 * hispeed_load, the same way brutal_active does for up_threshold.
	 */
	bool			hispeed_active;

	/* Entry-side streak counter for hispeed activation.  Increments
	 * on every sample where load_pct >= hispeed_load, resets when
	 * load_pct < hispeed_load.  hispeed_active flips to true only
	 * when streak > tunables->hispeed_entry_streak, providing
	 * symmetric entry/exit hysteresis.  Capped to a small u8 to
	 * avoid wraparound on long sustained-load runs.
	 */
	u8			hispeed_entry_count;

	/* Entry-side streak counter for brutality activation.  Mirror of
	 * hispeed_entry_count: increments on every sample where
	 * load_pct >= up_threshold while brutal_active is false, resets
	 * when load_pct drops below up_threshold OR when brutal_active
	 * flips true (past entry, no more entry credit to accumulate).
	 * Gates the SNAP snap-to-max path only when
	 * tunables->brutal_entry_streak > 0.  Capped to a small u8 to
	 * avoid wraparound on sustained-above-threshold runs that never
	 * quite reach the streak cap (shouldn't happen with the default
	 * cap of 16 but defensive against future cap bumps).
	 */
	u8			brutal_entry_count;

	/* Entry-side streak counter for the peak-headroom rescue tier.
	 * Increments on every sample where the cluster is starving
	 * (load_pct >= STARVE_LOAD_PCT and freq < FREQ_FLOOR_PCT of
	 * policy->max), resets when either condition breaks.  The
	 * rescue fires only when the streak exceeds STARVE_STREAK,
	 * giving STARVE_STREAK+1 consecutive sample windows of
	 * starvation as the entry hysteresis.  Capped to a small u8 to
	 * avoid wraparound on indefinitely-sustained heavy load.
	 */
	u8			peak_starve_count;

	/* Hold-down deadline for the peak-headroom rescue tier.  Stamped
	 * to ktime_get_ns() + PEAK_HEADROOM_HOLD_MS at the moment the
	 * rescue fires.  While now < this deadline, repeated streak
	 * crossings are observed (the streak counter still increments)
	 * but no second rescue freq-bump is applied.  Eliminates the
	 * pathological case where a rescue lifts freq, the very next
	 * sample observes load_pct still >= STARVE_LOAD_PCT and freq
	 * still < FLOOR_PCT (because the cpufreq driver hasn't applied
	 * the previous request yet), and another rescue fires --
	 * stacking two unwanted up-shifts on what should be a single
	 * rescue event.  Cleared (set to 0) at policy init.
	 */
	u64			peak_rescue_until_ns;

	/* Migration-arrival soft-floor deadline (Patch K1).  Stamped
	 * by the per-CPU update_util callback whenever any CPU in
	 * this policy sees a sample-to-sample util jump exceeding
	 * tunables->migration_jump_pct of its max_capacity.  Read by
	 * the migration_floor tier in zenith_get_next_freq() to
	 * decide whether to apply the soft floor.  Self-disarms by
	 * deadline; ktime_get_ns() advancing past the value is the
	 * disarm.  Cleared (set to 0) at policy init.  Updated under
	 * the per-policy update_lock by the shared callback;
	 * unlocked but ordered by raw_spin_lock acquire/release in
	 * the single-CPU callback.  No torn-write hazard either way:
	 * a 64-bit write on a 64-bit kernel is atomic, and the field
	 * is only ever monotonic-forward written.
	 */
	u64			migration_in_until_ns;

	/* Cluster-wake-pulse (Patch 1.3) deadlines.  cluster_wake_-
	 * pulse_until_ns is stamped at the top of zenith_get_next_-
	 * freq() when (a) cluster_wake_pulse_ms is non-zero and
	 * (b) the gap (now_ns - cluster_wake_last_eval_ns) is at
	 * or above cluster_wake_pulse_idle_ms.  cluster_wake_last_-
	 * eval_ns is updated on every eval (so a continuous active
	 * stream simply keeps refreshing the timestamp without ever
	 * arming the pulse).  Same single-writer-per-policy reason-
	 * ing as migration_in_until_ns above: both fields are
	 * touched only by the eval path under update_lock.
	 */
	u64			cluster_wake_pulse_until_ns;
	u64			cluster_wake_last_eval_ns;

	/* Patch 1.9 fg-transition pulse deadline.  Stamped by the
	 * sched_wakeup_new tracepoint probe (zenith_probe_wakeup_-
	 * new) when a foreground task is woken for the first time
	 * after fork() on a CPU that belongs to this policy.
	 *
	 * Cross-context writer: unlike the other deadline fields in
	 * this struct (which are touched only by the eval path
	 * under update_lock), this one is written from arbitrary
	 * scheduler context.  Both writer and the eval-path reader
	 * use WRITE_ONCE / READ_ONCE; on 64-bit the access is
	 * naturally torn-write-safe, on 32-bit the worst case is a
	 * sub-millisecond drift on the deadline read which is well
	 * under the resolution of fg_transition_pulse_ms.
	 */
	u64			fg_transition_pulse_until_ns;

	/* PSI-mem cap deadline (Patch M1).  Stamped by the eval path
	 * when zenith_psi_mem_some_pct() crosses psi_mem_cap_thresh
	 * (and master psi_aware + V2 PSI_MEM_CAP tier gates pass);
	 * read by the same eval path one tier later as
	 * `now < deadline -> cap fires`.  Same per-policy / single-
	 * writer reasoning as migration_in_until_ns above: each
	 * policy stamps its own deadline (so the BIG / PRIME caps
	 * release independently), and the eval is serialized by
	 * update_lock.
	 *
	 * Cleared (set to 0) at policy init by zenith_alloc_-
	 * policy(); never decremented other than by the deadline
	 * comparison expiring, so no torn-write hazard on a 64-bit
	 * kernel and natural alignment guarantee on 32-bit.
	 */
	u64			psi_mem_cap_until_ns;

	/* Patch C9: io_floor_hyst sticky-deadline.  Stamped by
	 * zenith_iowait_boost() on the 0->positive boost edge with
	 * 'now + io_floor_hyst_ms * NSEC_PER_MSEC'.  Read by
	 * zenith_get_next_freq() to gate the io_floor tier.  Same
	 * writer reasoning as psi_mem_cap_until_ns above; reads are
	 * naked u64 loads, harmless on a stale-by-one-tick read
	 * because the floor is a heuristic.
	 */
	u64			io_floor_until_ns;

	/* Util-trend ring for the predictive up-shift tier (2a').  The
	 * tail of zenith_get_next_freq() pushes the current sample's
	 * util into util_history[util_history_idx] and advances the
	 * index modulo predict_up_window.  util_history_count tracks
	 * how many slots have been written since policy attach (or
	 * since the last shrink-on-window-store), so the tier can wait
	 * for the ring to warm up before evaluating a trend on
	 * unwritten zeroes.  Capped at ZENITH_PREDICT_UP_WINDOW_MAX so
	 * the storage cost is bounded.
	 */
	unsigned long		util_history[ZENITH_PREDICT_UP_WINDOW_MAX];
	unsigned int		util_history_idx;
	unsigned int		util_history_count;

	/* Render-thread floor debounce stamp [Stage 4 / Patch B].
	 * Set to ktime_get_ns() on the first eval where
	 * zenith_policy_has_render() returns true after a quiet
	 * period (previous sample saw has_render==false).  Reset to
	 * 0 the first time has_render returns false, so the debounce
	 * window restarts on each fresh render-thread arrival.
	 * Compared against tunables->render_floor_min_runtime_ms
	 * before the floor is allowed to fire.  Zero-initialised at
	 * policy alloc; no special teardown.
	 */
	u64			render_first_seen_ns;

	/* Peak-return hysteresis streak counter [Stage 4 / Patch E].
	 * Increments on every consecutive sample where the previous
	 * cached_raw_freq was at peak class
	 * (>= ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT * policy->max
	 * / 100) and the current freq wants to drop sharply.  When
	 * < tunables->peak_hysteresis_streak, the soft floor at
	 * (prev_freq * peak_step_down_pct / 100) is applied; when
	 * the streak drains, the cluster falls naturally through
	 * the lower tiers.  Reset on any sample where the previous
	 * freq is below peak class, or when the freq tier already
	 * computes a value at or above the soft floor.  Capped at
	 * ZENITH_PEAK_HYSTERESIS_STREAK_MAX so a stuck-near-peak
	 * regime cannot accumulate unbounded streak credit.
	 */
	unsigned int		peak_low_streak;

	/* Anchor freq for the peak-return hysteresis tier
	 * [Stage 4 / Patch E].  Captured on the transition from a
	 * peak-class previous freq to a non-peak natural freq, then
	 * used as the source for the soft floor
	 * (anchor * peak_step_down_pct / 100) until the streak
	 * drains.  Cleared on streak drain or on a sample where the
	 * computed freq is already at or above the soft floor (no
	 * hysteresis needed).  0 means "no anchor armed".
	 */
	unsigned int		peak_hyst_anchor_freq;

	/* Persistent-idle streak counter for boost early-exit
	 * [Stage 4 / Patch F].  Increments on every tick where an
	 * input boost is armed (now < zenith_input_boost_until_ns)
	 * AND the cluster's load_pct is below
	 * tunables->boost_idle_thresh.  Reset on boost expiry, on
	 * any non-idle sample inside the boost window, or after a
	 * preemption tick fires.  Capped at
	 * ZENITH_BOOST_IDLE_STREAK_MAX so a stuck-near-zero workload
	 * cannot accumulate unbounded streak credit.
	 */
	unsigned int		boost_idle_low_streak;

	/* Last-runnable timestamp [Stage 4 / Patch H].  Stamped
	 * with ktime_get_ns() in zenith_get_next_freq() whenever
	 * tp_load_pct > 0 (the cluster has any util).  Read by the
	 * sleeper-tail tier to gate the freq shave.
	 */
	u64			last_runnable_ns;

	/* Last decision tag chosen by zenith_get_next_freq() on this
	 * policy [Stage 4 / Patch J].  Stamped right after the
	 * stats[] update at the end of every eval (cached and
	 * uncached paths both update it).  Read-only sysfs node
	 * "last_decision_path" dumps the current value per-policy,
	 * formatted "policy<cpu>(<cluster>): <tag>\n".  Pointers
	 * are to .rodata string literals so storage is a single
	 * pointer assignment, no copy.  Reads use READ_ONCE for
	 * coherency with the eval-side WRITE_ONCE.
	 */
	const char		*last_decision_path;

	/* Patch B7-2: decision-ring buffer.  Per-policy circular log
	 * of the last ZENITH_DEC_RING_NR (path, lat_ns) entries,
	 * paired so the sysfs reader can correlate which tier won
	 * with how long the eval took.  Updated in lockstep with
	 * dec_lat_buckets and last_decision_path at the end of every
	 * eval; head advances under the policy update_lock so the
	 * read side only needs READ_ONCE on path.  lat_ns is a 32-bit
	 * value (eval cost is bounded by tens of microseconds; truncating
	 * the high bits at u32_max ~= 4 s loses only pathological
	 * outliers, and they get clamped, not wrapped).
	 */
	struct zenith_dec_ring_entry {
		const char	*path;
		u32		lat_ns;
	} dec_ring[ZENITH_DEC_RING_NR];
	unsigned int		dec_ring_head;

	/* Time-bounded cache for the per-policy uclamp_{min,max}
	 * aggregations.  Each walk is O(n_cpus_in_policy) rq reads
	 * (cheap, no locks, no cachelines dirtied) but the eval path
	 * can run every rate_limit_us, i.e. 10 000 Hz on up_rate path;
	 * caching for ZENITH_UCLAMP_CACHE_TTL_NS drops the rq walks by
	 * ~10x on an 8-CPU policy with zero user-visible staleness.
	 * Valid when uclamp_cache_valid_ns != 0 and now < valid_ns + TTL.
	 */
	unsigned long		cached_uclamp_min;
	unsigned long		cached_uclamp_max;
	u64			uclamp_cache_stamp_ns;

	/* Cached nice-load ratio for the policy (0..100). Sampled in
	 * the update_util hook when ignore_nice_load=1 and read from
	 * zenith_get_next_freq. Guarded by update_lock in the shared
	 * path; on the single path only one CPU writes it.
	 */
	unsigned int		nice_pct;

	/* Auto-tune observer state. Counters are incremented in the
	 * hot path when tunables->auto_tune=1; the delayed_work handler
	 * samples and resets them every ZENITH_AUTO_TUNE_PERIOD_MS and
	 * chooses a preset.
	 *
	 * They are atomic because the single-CPU update path runs
	 * zenith_get_next_freq() without holding update_lock, while the
	 * delayed_work handler reads+resets them (and runs on any CPU).
	 * atomic_inc in the hot path and atomic_xchg(..., 0) in the
	 * worker give us lockless correctness for the sample window.
	 */
	atomic_t		at_samples_total;
	atomic_t		at_samples_saturated;
	u64			at_last_events;
	unsigned int		at_last_total;
	unsigned int		at_last_saturated;
	unsigned int		at_last_sat_pct;
	unsigned int		at_last_events_rate_x2;
	unsigned int		at_last_target;
	unsigned int		at_last_state;
	/* F2: PELT-derived util tracking.  Sum of per-cpu rq->cfs.avg
	 * .util_avg across this policy, sampled at every V1 work tick.
	 * Compared against the previous window's value to detect a
	 * rising-load trend before sat_pct fully saturates.  Stored
	 * as raw util sum (1024 * num_cpus capacity scale); rising
	 * threshold is normalised to a percentage in
	 * t->auto_tune_util_rising_thresh_pct.
	 */
	unsigned long		at_last_util_sum;
	/* M1: time-in-state accounting.  at_state_residency_ns[s] is the
	 * cumulative wall-clock time spent in V2 state s since governor
	 * start (or since the last reset).  Updated lazily at every V2
	 * commit point: on transition from old to new, accumulate the
	 * (now - last_change_ns) delta into residency[old_state] and
	 * stamp last_change_ns := now.  Cheap (one ktime_get + 5 u64
	 * stores per actual transition, NOT per tick).  Surfaced
	 * read-only via auto_tune_state_residency sysfs as a CSV that
	 * userspace tools can sample at low frequency to compute
	 * percent-time-in-state.
	 */
	u64			at_state_residency_ns[5];
	u64			at_state_last_change_ns;
	/* M2: V2 state transition history ring.  Last
	 * ZENITH_AT_HISTORY_NR commits per policy, expressed as
	 * { ts_boottime_ns, from_state, to_state, reason, flags }
	 * tuples.  Surfaced read-only via auto_tune_state_history
	 * sysfs as one line per entry, most recent first.
	 *
	 * Each entry is 16 bytes; 32 entries = 512 B per policy
	 * (typically 2 policies = 1 KB total).  Cheap.
	 */
	struct {
		u64		ts_ns;
		u32		flags;
		u8		from;
		u8		to;
		u8		reason;
		u8		_pad;
	}			at_history[ZENITH_AT_HISTORY_NR];
	unsigned int		at_history_head;
	unsigned int		at_history_count;
	/* at_last_applied_state is the V2 state AFTER the cluster-aware
	 * demotion path runs.  When auto_tune_cluster_aware = 1 the
	 * little cluster takes LATENCY/SUSTAINED_PERF -> BALANCED and the
	 * prime cluster takes BALANCED -> LATENCY; the at_last_state
	 * field holds the pre-demotion V2 state (so V3 / hysteresis /
	 * cooldown all see the canonical decision), and this field
	 * holds what was actually applied via zenith_at_apply_actions().
	 * Surfaced through auto_tune_status as applied_state= so an
	 * operator reading the file can tell at-a-glance which cluster
	 * was demoted vs the V2 logical state.  When cluster_aware is
	 * off, applied_state == at_last_state.
	 */
	unsigned int		at_last_applied_state;
	unsigned int		at_pending_state;
	unsigned int		at_pending_windows;
	unsigned int		at_cooldown_left;
	unsigned int		at_last_reason;
	unsigned int		at_last_flags;
	unsigned int		at_last_var_x256;
	unsigned int		at_last_psi_cpu;
	unsigned int		at_last_psi_io;
	unsigned int		at_last_psi_mem;
	unsigned int		at_last_thermal_pressure;
	unsigned int		at_last_thermal_slope;
	unsigned int		at_last_frame_budget_us;
	unsigned int		at_effective_up_rate_limit_us;
	unsigned int		at_effective_down_rate_limit_us;
	unsigned int		at_effective_up_threshold;
	unsigned int		at_effective_down_threshold;
	unsigned int		at_effective_input_boost_ms;
	unsigned int		at_effective_input_boost_cap_pct;
	unsigned int		at_effective_down_rate_adaptive;
	unsigned int		at_effective_down_threshold_adaptive;
	unsigned int		at_effective_frame_pace_floor_pct;
	unsigned int		at_effective_game_mode;
	bool			at_local_actions;

	/* Per-policy consecutive-EFFICIENCY-window counter for the
	 * boot_complete in-kernel calm detector.  See the
	 * ZENITH_DEFAULT_BOOT_COMPLETE_AUTO comment block.  Reset to 0
	 * whenever the resolved V2 state is anything other than
	 * EFFICIENCY; the auto detector latches zenith_boot_complete on
	 * the first per-policy worker that hits
	 * ZENITH_BOOT_COMPLETE_CALM_WINDOWS past the grace period.
	 */
	unsigned int		at_boot_calm_streak;

	/* V3 self-calibration state (see ZENITH_DEFAULT_AUTO_TUNE_V3
	 * comment block).  Updated at the tail of zenith_auto_tune_work()
	 * once per (auto_tune_v3_interval_ms) wall-clock period when the
	 * v3 key is enabled.  All fields are owned by the v2 worker
	 * thread and read either from the same thread (calibration
	 * step) or via sysfs *_show under the gov_attr_set rwsem (which
	 * guarantees a consistent snapshot).
	 *
	 *   at_v3_last_calib_ns    -- boottime ns of last calibration
	 *   at_v3_last_transitions -- transitions counted in last window
	 *   at_v3_hyst_offset      -- signed offset on hysteresis_windows
	 *   at_v3_cool_offset      -- signed offset on cooldown_windows
	 *
	 * Offsets are clamped to [ZENITH_AT_V3_OFFSET_MIN ..
	 * ZENITH_AT_V3_OFFSET_MAX] and only consumed when the v3 mode
	 * is APPLY (== 2).
	 */
	u64			at_v3_last_calib_ns;
	unsigned int		at_v3_last_transitions;
	signed char		at_v3_hyst_offset;
	signed char		at_v3_cool_offset;

	/* Patch J: V3 calibration audit ring.  Pushed by
	 * zenith_at_v3_calibrate() at the end of every calibration
	 * tick (both OBSERVE and APPLY).  Surfaced via the
	 * auto_tune_v3_calib_log RO sysfs node so the operator can
	 * see V3's actual drift trajectory without ftrace.
	 * Single-writer (the v2 worker thread) / multi-reader (sysfs
	 * *_show under the gov_attr_set rwsem); reset alongside the
	 * other observability surfaces in
	 * zenith_policy_observability_reset() and on V3 master
	 * MODE_OFF transitions in auto_tune_v3_store.
	 */
	struct zenith_at_v3_calib_log_entry
				at_v3_calib_log[ZENITH_AT_V3_CALIB_LOG_NR];
	unsigned int		at_v3_calib_log_head;
	unsigned int		at_v3_calib_log_count;

	/* round-U-z10 glide / coordination knobs auto-driven by the V2
	 * worker via zenith_at_apply_glides() when
	 * tunables->auto_tune_v2_glides is on.  Consumers fall through
	 * to these only when the user-set tunable is 0; otherwise the
	 * user value wins outright.  at_local_glides_active gates the
	 * fall-through; cleared on auto_tune_v2 disable so the next
	 * unrelated freq tick stops consulting stale values.
	 */
	bool			at_local_glides_active;
	unsigned int		at_local_brutal_decay_ms;
	unsigned int		at_local_wakeup_boost_ms;
	unsigned int		at_local_boot_boost_decay_ms;
	unsigned int		at_local_screen_off_glide_ms;
	unsigned int		at_local_thermal_pressure_continuous;
	unsigned int		at_local_prefer_silver_aware;
	unsigned int		at_local_frame_budget_us_auto;

	/* Patch L: Stage-4 K1/K2/K3 tier-armed bitmask, written by
	 * zenith_at_apply_tiers() per V2 worker pass.  See the
	 * ZENITH_AT_TIER_* comment block.  Read by zenith_tier_value()
	 * from the K1/K2/K3 read sites in zenith_get_next_freq().
	 *
	 * at_local_tiers_active gates the read: false (cleared on V2
	 * disable, profile change, or auto_tune_v2_tiers=0) means the
	 * K1/K2/K3 reads return the underlying tunable verbatim, the
	 * pre-Patch-L behaviour.  true means consult the armed mask.
	 *
	 * Single-writer (V2 worker) / single-reader (eval path under
	 * update_lock); staleness is bounded by ZENITH_AUTO_TUNE_PERIOD_MS.
	 */
	bool			at_local_tiers_active;
	unsigned long		at_local_tier_armed_mask;
	struct delayed_work	at_work;

	/* Auto-tune classifier ring buffer.  Single-writer (the
	 * delayed_work worker), multi-reader (sysfs at_log readers).
	 * at_log_head is the next write slot; at_log_count is the total
	 * number of entries pushed since the last reset, capped at the
	 * ring depth on read.  Both are reset by writing to
	 * zenith_stats_reset.
	 */
	struct zenith_at_log_entry at_log[ZENITH_AT_LOG_NR];
	unsigned int		at_log_head;
	unsigned int		at_log_count;

	/* brutal_decay_ms tail-glide deadline.  0 means no decay is
	 * armed and zenith_get_next_freq() takes the fast path; non-zero
	 * is an absolute ktime_get_ns() value at which the decay floor
	 * stops applying.  Single-writer (zenith_get_next_freq() under
	 * the policy's update_lock), single-reader (same site).
	 */
	u64			brutal_decay_until_ns;
	unsigned int		brutal_decay_arm_ms;

	/* screen_off_glide_ms tracking.  screen_state_last is the
	 * screen_state value seen on the previous zenith_get_next_freq()
	 * tick; transitions are detected by comparing tunables-> against
	 * this snapshot.  screen_off_arm_ns is set to ktime_get_ns() at
	 * the moment of the 1 -> 0 transition and cleared on the
	 * 0 -> 1 transition; non-zero on the screen-off branch means
	 * the glide ramp is candidate for application.  Single-writer,
	 * single-reader (zenith_get_next_freq() under update_lock).
	 */
	u64			screen_off_arm_ns;
	unsigned int		screen_state_last;

	/* prefer_silver_aware coordination state (dead -- Kconfig removed).
	 * Snapshot of the global prefer_silver hit / miss counters at
	 * the previous V1 classifier window, plus the resulting hit-rate
	 * (0..100) for the most-recent window.  These fields stay at 0
	 * (the worker never updates them) and the downstream bump never
	 * fires.
	 */
	unsigned int		ps_prev_hit;
	unsigned int		ps_prev_miss;
	unsigned int		ps_hit_rate_pct;

	/* Cached topology bit: true when any CPU in the policy has
	 * arch_scale_cpu_capacity == SCHED_CAPACITY_SCALE, i.e. the
	 * policy belongs to (one of) the system's highest-capacity
	 * cluster(s).  Computed once at zenith_start() time from the
	 * policy's cpumask and kept for the lifetime of the policy.
	 * Used by the input-boost gate (input_boost_big_only) to skip
	 * boosting small-cluster policies on heterogeneous SoCs.
	 */
	bool			is_big_cluster;

	/* Cached rate-limit scale factors applied when
	 * rate_limit_cluster_scale is enabled.  1/0 means unscaled;
	 * little clusters use 2/1 so upward ramps are less eager and
	 * downward ramps save power sooner.
	 */
	unsigned int		up_rate_scale;
	unsigned int		down_rate_scale_shift;
	unsigned int		cluster_class;

	/* Cached per-policy result of the render-aware comm walk.  Valid
	 * for ZENITH_RENDER_CACHE_TTL_NS after render_cache_stamp_ns.
	 * Refreshed on the next zenith_get_next_freq() call past the TTL.
	 * Zero stamp means "never sampled".
	 */
	bool			render_active;
	u64			render_cache_stamp_ns;

	/* Wave A render-thread util tracker.  Stores the
	 * se.avg.util_avg of the first comm-matched render task
	 * observed during zenith_policy_has_render()'s walk.  Refreshed
	 * alongside render_active / render_cache_stamp_ns; valid for
	 * ZENITH_RENDER_CACHE_TTL_NS.  Zero means "no matched task"
	 * (or never sampled); non-zero is the matched task's util in
	 * 1/SCHED_CAPACITY_SCALE units.
	 */
	unsigned int		render_matched_util_avg;

	/* Cached per-policy result of the cgroup-aware top-app walk.
	 * Valid for ZENITH_TOP_APP_CACHE_TTL_NS after
	 * top_app_cache_stamp_ns.  Refreshed on the next
	 * zenith_get_next_freq() call past the TTL.  Zero stamp means
	 * "never sampled".
	 */
	bool			top_app_active;
	u64			top_app_cache_stamp_ns;

	/* Wave B EAS energy-knee freq cache.  Populated at
	 * zenith_start() time from em_cpu_get(); refreshed lazily on
	 * the first em_floor application if the EM was not yet
	 * registered when zenith_start() ran.  Zero means "no EM"
	 * (or never sampled); non-zero is the energy-knee freq in
	 * KHz.
	 */
	unsigned int		em_knee_freq;

	/* Cached per-policy result of the audio-aware comm walk.  Same
	 * shape as the render cache above; TTL is
	 * ZENITH_AUDIO_CACHE_TTL_NS.  Zero stamp means "never sampled".
	 */
	bool			audio_active;
	u64			audio_cache_stamp_ns;

	/* Patch B7-1: sticky audio-active deadline.  When
	 * zenith_policy_has_audio() observes a positive detection
	 * (alsa fd > 0 or comm-walk match) we extend this deadline to
	 * now + audio_hyst_ms.  While now < audio_sticky_until_ns the
	 * helper returns true regardless of fresh signal, so a brief
	 * gap between two pcm releases / re-opens does not flip the
	 * cluster out of audio-aware mode.  audio_hyst_ms == 0
	 * disables the hysteresis (legacy behaviour).
	 */
	u64			audio_sticky_until_ns;

	/* Cached per-policy result of the camera-aware comm walk.
	 * Holds the *raw* comm-match result, before the userspace
	 * override (camera_active=auto/force-on/force-off) is applied.
	 * TTL is ZENITH_CAMERA_CACHE_TTL_NS.  Zero stamp means "never
	 * sampled".
	 */
	bool			camera_auto_match;
	u64			camera_cache_stamp_ns;

	/* Cached per-policy result of the game-auto comm walk + streak
	 * counter for the in-kernel game detector.  See
	 * ZENITH_DEFAULT_GAME_AUTO comment block.  TTL is
	 * ZENITH_GAME_AUTO_CACHE_TTL_NS for the comm-match cache; the
	 * streak counter increments on each hot-path call that observes
	 * a match (subject to the cache TTL) and resets to 0 on the
	 * first miss.  When it reaches ZENITH_GAME_AUTO_DETECT_STREAK,
	 * the global zenith_game_auto_active_until_ns latch is renewed
	 * for ZENITH_GAME_AUTO_ACTIVE_TTL_NS and the streak resets.
	 */
	bool			game_auto_match;
	u64			game_auto_cache_stamp_ns;
	unsigned int		game_auto_streak;

	/* Patch K: per-policy state for the game / sustained-high-load
	 * performance burst FSM.  See ZENITH_DEFAULT_GAME_PERF_BURST and
	 * the long comment block above ZENITH_GPB_STATE_IDLE for the
	 * full mechanism; this struct holds the live FSM state only.
	 *
	 *   gpb_state                ZENITH_GPB_STATE_{IDLE,ARMED,COOLDOWN}
	 *   gpb_state_entry_ns       ktime_get_ns() at the last FSM
	 *                            transition; used by the COOLDOWN
	 *                            glide to compute the linear ramp
	 *   gpb_b_arm_first_seen_ns  ktime_get_ns() at the first tick
	 *                            where Signal B (load >= 70%) was
	 *                            seen continuously; 0 = not seen
	 *   gpb_b_disarm_first_seen_ns
	 *                            ktime_get_ns() at the first ARMED
	 *                            tick where Signal B has dropped
	 *                            below 70%; 0 = currently still hot
	 *   gpb_tzd                  cached struct thermal_zone_device *
	 *                            for this policy's primary CPU
	 *                            ("cpu<N>-thermal"), resolved at
	 *                            zenith_start() and consulted by the
	 *                            thermal guardrail helper.  NULL means
	 *                            "no per-policy zone resolved", in
	 *                            which case the helper falls back to
	 *                            arch_scale_thermal_pressure().
	 *   gpb_tzd_retry_at_ns      ktime_get_ns() deadline for the next
	 *                            lazy zone-resolution retry when
	 *                            gpb_tzd is NULL.  Patch M closes the
	 *                            boot-ordering hole where thermal-core
	 *                            registers after the governor: the
	 *                            hot-path helper will re-attempt
	 *                            thermal_zone_get_zone_by_name() at
	 *                            most once per
	 *                            ZENITH_GPB_TZD_RETRY_INTERVAL_NS
	 *                            until a zone is bound.
	 *   gpb_arm_count            number of IDLE -> ARMED *and*
	 *                            COOLDOWN -> ARMED transitions since
	 *                            attach.  Bumped from the FSM
	 *                            evaluator at every rising edge.
	 *                            Counts re-arms so userspace can see
	 *                            "user keeps Alt+Tabbing back into
	 *                            the game" thrash.
	 *   gpb_disarm_count         number of ARMED -> COOLDOWN
	 *                            transitions since attach.  Equals
	 *                            gpb_arm_count when the FSM is
	 *                            currently IDLE/COOLDOWN, one less
	 *                            when ARMED.
	 *   gpb_idle_count           number of COOLDOWN -> IDLE
	 *                            transitions since attach.  Counts
	 *                            full glide completions; missing
	 *                            counts (vs disarm_count) are
	 *                            re-arms during cooldown.
	 *   gpb_last_disarm_reason   ZENITH_GPB_DISARM_{NONE,FAST,
	 *                            SUSTAINED}.  Recorded at every
	 *                            ARMED -> COOLDOWN edge.
	 *                            Patch M; surfaced by the
	 *                            game_perf_burst_stats RO sysfs.
	 *
	 * All ten fields are only touched from the per-policy hot path
	 * (zenith_get_next_freq() and the FSM evaluator it calls), which
	 * is serialised by the cpufreq core.  No locking required.  All
	 * are zero-initialised by zenith_start()'s memset path or
	 * explicitly cleared at attach.
	 */
	u8			gpb_state;
	u64			gpb_state_entry_ns;
	u64			gpb_b_arm_first_seen_ns;
	u64			gpb_b_disarm_first_seen_ns;
	struct thermal_zone_device *gpb_tzd;
	u64			gpb_tzd_retry_at_ns;
	u32			gpb_arm_count;
	u32			gpb_disarm_count;
	u32			gpb_idle_count;
	u8			gpb_last_disarm_reason;

	/* Last seen zenith_input_boost_until_ns deadline observed inside
	 * an active boost window for this policy.  Latched in the input
	 * boost step (0) and consumed by the sampling-down step (7) to
	 * keep the down-rate multiplier elevated for one stretched
	 * down-rate window after the boost exits, smoothing the boost
	 * tail when target_freq immediately falls below policy->max.
	 * Cleared once the stretched window passes.  Zero means "no
	 * recent boost to extend".
	 */
	u64			boost_active_until_ns;

	/* Variance-adaptive up_threshold state (see
	 * ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE).  load_var_ewma_x256 is
	 * an EWMA (alpha=1/8) of |load_pct - prev| in fixed-point
	 * 1/256ths -- so a value of 256 == 1.0 percentage-point of
	 * average sample-to-sample swing.  Read at the top of
	 * zenith_get_next_freq() to bias dynamic_up_thresh, updated at
	 * the bottom with the current sample's tp_load_pct.  Both fields
	 * zero-initialised by zenith_start()'s memset.
	 */
	unsigned int		load_var_ewma_x256;
	unsigned int		last_load_pct;

	/* Per-policy decision stats.  Bumped from zenith_get_next_freq()
	 * once per evaluation; total decisions and cache_hits are
	 * always counted, the tier buckets are mapped from the final
	 * tp_path string by zenith_path_to_bucket().  Reset to zero on
	 * zenith_start() so the values reflect the current attach
	 * cycle.  No locking: zenith_get_next_freq() is serialised
	 * per-policy by the cpufreq core (the leader cpu in a shared
	 * policy, the only cpu in a single policy), so a plain
	 * unsigned long ++ is race-free.
	 */
	unsigned long		stats[ZENITH_STAT_NR];

	/* Patch 1.4 decision-latency histogram.  4 unsigned-long
	 * buckets covering the eval cost in nanoseconds:
	 *   [0]   <  10 us
	 *   [1]  10..< 50 us
	 *   [2]  50..<100 us
	 *   [3]  >=100 us
	 *
	 * Storage budget: 4 * sizeof(unsigned long) per policy.
	 * Bumped at the same commit point as stats[]; the same
	 * single-writer reasoning applies (per-policy serialised by
	 * the cpufreq core).  Sysfs read via decision_latency_hist.
	 * Reset to zero on zenith_start() alongside stats[].
	 */
	unsigned long		dec_lat_buckets[4];

	/* Patch B9-1: realised per-cluster freq scale, cached from the
	 * android_vh_arch_set_freq_scale vendor hook.  Stores the most
	 * recent SCHED_CAPACITY_SCALE-domain value (0..1024) the
	 * scheduler observed for this policy after a freq write
	 * actually took effect.  Updated under no governor lock from
	 * the hook callback (which can fire in arbitrary scheduler
	 * context); written via WRITE_ONCE so the eval-path readers
	 * (which hold update_lock) and the cross-cluster comparator in
	 * the probe (which does not) both see torn-write-safe values
	 * on 32-bit.  Read via READ_ONCE.
	 *
	 * Cleared (set to 0) at policy init by kzalloc(); never
	 * decremented other than by the hook overwriting it on the
	 * next realisation.  When tunables->vh_arch_freq_scale_enable
	 * is 0 this field never moves off zero.
	 */
	unsigned long		vh_arch_freq_scale_last;

	/* Patch B9-3: most recent cpuidle residency observed on a CPU
	 * belonging to this policy, in ns.  Stamped by zenith_probe_-
	 * cpu_idle_exit() as (now_ns - per_cpu enter_ns).  Read by the
	 * cluster_wake_pulse arm site in zenith_get_next_freq() with
	 * READ_ONCE; the cluster_wake_pulse is suppressed when this
	 * value crosses ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS.  Cleared
	 * (set to 0) at policy init by kzalloc(); only written by the
	 * exit probe so a last-writer-wins race across CPUs in the
	 * cluster is acceptable (the next exit on any CPU resets it).
	 * When tunables->vh_cpu_idle_enable is 0 this field never
	 * moves off zero.
	 */
	u64			vh_cpu_idle_last_residency_ns;

	/* Patch B9-5: per-policy jiffies stamp updated by
	 * zenith_probe_sched_move_task() on every cgroup move that
	 * lands a task on a CPU belonging to this policy.  Cleared (set
	 * to 0) at policy init by kzalloc(); only written by the probe
	 * (under READ_ONCE / WRITE_ONCE, no governor lock taken).  A
	 * 0 value means "no cgroup move observed since boot or while
	 * this policy was zenith-driven".  Read with READ_ONCE from
	 * any consumer (the auto-selector worker is the intended
	 * future consumer; B9-5 only stamps -- no consumer is wired
	 * in this patch, by design).  When tunables->vh_sched_move_-
	 * task_enable is 0 this field never moves off zero.
	 */
	unsigned long		vh_sched_move_task_last_jiffies;
};

struct zenith_cpu {
	struct update_util_data	update_util;
	struct zenith_policy	*z_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;

	/* Wall-time the current iowait boost episode first armed; zero
	 * means "no episode in progress".  Stamped in zenith_iowait_boost()
	 * on the 0->floor transition, cleared whenever iowait_boost
	 * itself is cleared (zenith_iowait_reset / zenith_iowait_apply).
	 * Read by zenith_iowait_boost() to decide whether the
	 * iowait_backoff_after_ms timer has elapsed and the doubling
	 * step should flip to a halving step.
	 */
	u64			iowait_boost_first_ns;
	u64			last_update;
	unsigned long		bw_dl;

	/* Delta tracking for ignore_nice_load. Only read when the
	 * corresponding tunable is set.
	 */
	u64			prev_nice_time;
	u64			prev_nice_wall;
	unsigned long		max_capacity;

	/* Previous *unpredicted* zenith_get_util() output for this CPU,
	 * used as the second tap of the one-step-ahead predictor.  Only
	 * read/written when tunables->predict_util_pct != 0.  Initialised
	 * to zero by the kzalloc-style allocation in zenith_start().
	 */
	unsigned long		prev_util;

	/* Previous arch_scale_thermal_pressure() observation, used by
	 * the thermal_derate_rate_pct derivative term to compute a
	 * sample-to-sample rise.  Zero-initialised by zenith_start()'s
	 * memset; updated unconditionally whenever the static derate
	 * runs, so toggling the rate tunable doesn't see a stale
	 * prev_pressure for the first sample.
	 */
	unsigned long		prev_thermal_pressure;

	/* Tap from two samples ago, used only when
	 * tunables->predict_util_smooth is set and predict_util_pct != 0.
	 * Maintained in lockstep with prev_util (push the old prev_util
	 * into prev_util_2 before overwriting prev_util) so toggling the
	 * smooth tunable does not require any per-cpu init handshake.
	 * Initialised to zero by the kzalloc-style allocation in
	 * zenith_start().
	 */
	unsigned long		prev_util_2;

	/* Wakeup-boost state.  prev_util is independent from the predictor
	 * taps above so the detector works even when predict_util_pct=0.
	 * The countdown is decremented on upward transitions that bypass
	 * up_rate_limit.
	 */
	unsigned long		wakeup_prev_util;
	u8			wakeup_boost_ticks;

	/* Previous post-iowait_apply util on this CPU, used by the
	 * migration-arrival detector (Patch K1).  Independent from
	 * wakeup_prev_util because that field is only updated when
	 * tunables->wakeup_boost is set, and the migration detector
	 * needs to run regardless.  Zero-initialised by zenith_start()'s
	 * memset, which means the very first tick after policy attach
	 * looks like a "jump from 0" -- harmless: the migration tier
	 * disarms by 30 ms (default) wall-clock and any cluster
	 * starting from idle benefits from a brief floor anyway.
	 */
	unsigned long		migration_prev_util;

	/* Deadline mirror of wakeup_boost_ticks.  0 means no
	 * wall-clock-based bypass armed; non-zero is an absolute
	 * ktime_get_ns() value at which the time-based portion of the
	 * up-rate bypass stops applying.  Set together with
	 * wakeup_boost_ticks at the detection sites when
	 * tunables->wakeup_boost_ms is non-zero.
	 */
	u64			wakeup_boost_until_ns;

	/* kcpustat hispeed-blend sampler state (consumed by
	 * zenith_kcpustat_sample / zenith_kcpustat_blend). Two-phase
	 * windowed measurement: phase 1 clears at window expiry and
	 * arms hispeed_active for an immediate post-reset sample on
	 * the next callback.  Filtered busy_pct feeds an asymmetric
	 * EWMA (instant up, slow-decay down by filter_shift).
	 *
	 * hispeed_start_ns marks t=0 of the >>(elapsed_ms/32) decay
	 * applied in zenith_kcpustat_blend(); hispeed_idle_windows
	 * provides a one-window grace period before clearing the
	 * decay timer, so rapid idle/busy spinning doesn't keep the
	 * floor latched at full strength.
	 *
	 * All fields are zero-initialised by zenith_start()'s
	 * memset, which is the desired post-policy-attach state.
	 */
	u64			kc_prev_idle_time;	/* in usec */
	u64			kc_prev_wall_time;	/* in usec */
	unsigned int		kc_busy_pct;		/* raw, last window */
	unsigned int		kc_filtered_busy_pct;	/* EWMA-smoothed */
	bool			kc_hispeed_active;
	u64			kc_hispeed_start_ns;
	unsigned int		kc_idle_windows;

	/* Patch B9-3: ktime_get_ns() timestamp of the most recent
	 * cpuidle entry observed on this CPU.  Stamped by zenith_-
	 * probe_cpu_idle_enter() and consumed exactly once by
	 * zenith_probe_cpu_idle_exit() to compute (exit_ns -
	 * enter_ns) before stamping the per-policy
	 * vh_cpu_idle_last_residency_ns aggregate.  Cleared (set to
	 * 0) at zenith_start() time by the kzalloc-style memset; the
	 * exit probe treats 0 as "no enter observed" and skips the
	 * residency stamp.  When tunables->vh_cpu_idle_enable is 0
	 * this field never moves off zero (the enter probe gates on
	 * the same READ_ONCE).
	 */
	u64			vh_cpu_idle_last_enter_ns;

	/* Patch B9-4: per-CPU last-tick timestamp + tick count.
	 * Stamped by zenith_probe_scheduler_tick() once per
	 * android_vh_scheduler_tick fire on the local CPU.  Single
	 * writer per CPU (the local CPU's tick handler), so no
	 * atomics needed; remote-CPU readers use READ_ONCE.  Cleared
	 * (set to 0) at zenith_start() time by the kzalloc-style
	 * memset.  When tunables->vh_scheduler_tick_enable is 0 these
	 * fields never move off zero.
	 */
	u64			vh_scheduler_tick_last_ns;
	unsigned long		vh_scheduler_tick_count;
};

static DEFINE_PER_CPU(struct zenith_cpu, zenith_cpu);

/* Wave B PMU IPC tracker per-CPU state.  See the comment block above
 * ZENITH_DEFAULT_PMU_AWARE for the full rationale.  Allocated by
 * zenith_pmu_init_cpu() at zenith_start() time; released by
 * zenith_pmu_exit_cpu() at zenith_stop() time.  Sampled once per
 * zenith_auto_tune_work() pass; the resulting IPC is cached in
 * ipc_pct (1.0 IPC == 100) and read by zenith_policy_max_ipc_pct().
 *
 * On CONFIG_PERF_EVENTS=n the perf_event pointers are absent and
 * the helper functions all collapse to no-ops via the #else branch
 * below.
 */
struct zenith_pmu_state {
#if IS_ENABLED(CONFIG_PERF_EVENTS)
	struct perf_event	*inst_event;
	struct perf_event	*cycle_event;
#endif
	u64			last_inst;
	u64			last_cycles;
	unsigned int		ipc_pct;
};

static DEFINE_PER_CPU(struct zenith_pmu_state, zenith_pmu);

/* Wave B PMU IPC tracker forward declarations.  The bodies live next
 * to zenith_init() / zenith_start() because they share their lifecycle;
 * the call sites in zenith_auto_tune_work() and zenith_get_next_freq()
 * are upstream of the definitions in source order, so a forward
 * declaration is required.
 */
static int zenith_pmu_init_cpu(unsigned int cpu);
static void zenith_pmu_exit_cpu(unsigned int cpu);
static void zenith_pmu_sample_cpu(unsigned int cpu);
static unsigned int zenith_policy_max_ipc_pct(struct zenith_policy *z_policy);

/* Wave B EAS / Energy Model integration.  Resolve the energy-knee
 * frequency of the policy's perf_domain (the OPP with the lowest
 * em->table[].cost field).  Cached in z_policy->em_knee_freq so the
 * em->table walk runs at most once per zenith_start() pass.  Returns
 * 0 if no EM is registered for the policy or if all costs are zero
 * (badly-formed EM).  See the comment block above
 * ZENITH_DEFAULT_EM_AWARE for the full rationale.
 */
static unsigned int zenith_em_knee_freq(struct zenith_policy *z_policy);

static unsigned int zenith_tunable_or_local(struct zenith_policy *z_policy,
					    unsigned int tunable,
					    unsigned int local);
static unsigned int zenith_glide_value(struct zenith_policy *z_policy,
				       unsigned int tunable,
				       unsigned int local);

/************************ Schedutil: I/O Wait & DL Logic ***********************/

/* Resolve the configured iowait floor for the policy owning z_cpu, in
 * absolute SCHED_CAPACITY_SCALE units. Called from every iowait path so
 * kept inline and trivial.
 */
static inline unsigned int zenith_iowait_floor(struct zenith_cpu *z_cpu)
{
	unsigned int permille = z_cpu->z_policy->tunables->iowait_boost_min;

	return (SCHED_CAPACITY_SCALE * permille) / 1000;
}

static bool zenith_iowait_reset(struct zenith_cpu *z_cpu, u64 time, bool set_iowait_boost)
{
	s64 delta_ns = time - z_cpu->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	z_cpu->iowait_boost = set_iowait_boost ? zenith_iowait_floor(z_cpu) : 0;
	z_cpu->iowait_boost_pending = set_iowait_boost;
	z_cpu->iowait_boost_first_ns = set_iowait_boost ? time : 0;
	return true;
}

static void zenith_iowait_boost(struct zenith_cpu *z_cpu, u64 time,
				unsigned int flags, unsigned int io_is_busy)
{
	bool set_iowait_boost = (flags & SCHED_CPUFREQ_IOWAIT) && io_is_busy;

	/* Patch C9: stamp the io_floor hysteresis deadline on every
	 * iowait sample where the boost path would arm.  Bumps a
	 * sliding window forward; once the iowait_boost decays away,
	 * the floor outlives it for io_floor_hyst_ms past the last
	 * arming sample.  io_floor_hyst_ms == 0 stamps a 0 deadline,
	 * which the read-side check in zenith_get_next_freq() treats
	 * as no-floor (legacy behaviour).
	 *
	 * z_cpu->z_policy is established non-NULL by both callers
	 * (zenith_update_single, zenith_update_shared dereference
	 * z_policy->tunables before calling us); zenith_iowait_floor()
	 * and zenith_iowait_apply() likewise dereference it
	 * unconditionally, so we follow the same convention here.
	 */
	if (set_iowait_boost) {
		unsigned int hyst_ms =
			z_cpu->z_policy->tunables->io_floor_hyst_ms;

		if (hyst_ms)
			z_cpu->z_policy->io_floor_until_ns = time +
				(u64)hyst_ms * NSEC_PER_MSEC;
	}

	if (z_cpu->iowait_boost && zenith_iowait_reset(z_cpu, time, set_iowait_boost))
		return;
	if (!set_iowait_boost)
		return;
	if (z_cpu->iowait_boost_pending)
		return;

	z_cpu->iowait_boost_pending = true;

	if (z_cpu->iowait_boost) {
		unsigned int after_ms =
			z_cpu->z_policy->tunables->iowait_backoff_after_ms;
		u64 first = z_cpu->iowait_boost_first_ns;

		/* Sustained-iowait backoff: once the episode has been
		 * live for after_ms milliseconds, halve instead of
		 * doubling so the boost decays toward the floor and
		 * eventually clears.  See ZENITH_DEFAULT_IOWAIT_BACKOFF_
		 * AFTER_MS for the why.  after_ms=0 keeps the legacy
		 * unbounded-doubling behaviour.
		 */
		if (after_ms && first &&
		    (time - first) > (u64)after_ms * NSEC_PER_MSEC) {
			z_cpu->iowait_boost >>= 1;
			if (z_cpu->iowait_boost <
			    zenith_iowait_floor(z_cpu)) {
				z_cpu->iowait_boost = 0;
				z_cpu->iowait_boost_first_ns = 0;
			}
		} else {
			z_cpu->iowait_boost = min_t(unsigned int,
				z_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
		}
		return;
	}
	z_cpu->iowait_boost = zenith_iowait_floor(z_cpu);
	z_cpu->iowait_boost_first_ns = time;
}

static unsigned long zenith_iowait_apply(struct zenith_cpu *z_cpu, u64 time,
					 unsigned long util, unsigned long max_cap)
{
	unsigned long boost;
	unsigned int floor = zenith_iowait_floor(z_cpu);

	if (!z_cpu->iowait_boost)
		return util;
	if (zenith_iowait_reset(z_cpu, time, false))
		return util;

	if (!z_cpu->iowait_boost_pending) {
		z_cpu->iowait_boost >>= 1;
		if (z_cpu->iowait_boost < floor) {
			z_cpu->iowait_boost = 0;
			z_cpu->iowait_boost_first_ns = 0;
			return util;
		}
	}

	z_cpu->iowait_boost_pending = false;
	boost = (z_cpu->iowait_boost * max_cap) >> SCHED_CAPACITY_SHIFT;

	{
		unsigned int stack_pct =
			z_cpu->z_policy->tunables->iowait_stack_pct;

		if (stack_pct && stack_pct <= 100) {
			/* Blend: util + (boost * stack_pct / 100), clamped
			 * to max_cap.  Take max with the legacy result so
			 * we never do worse than the old path on a
			 * util-light / iowait-heavy workload.
			 */
			unsigned long stacked = util +
				((boost * stack_pct) / 100);

			if (stacked > max_cap)
				stacked = max_cap;
			boost = max3(boost, util, stacked);
		} else {
			boost = max(boost, util);
		}
	}
	boost = uclamp_rq_util_with(cpu_rq(z_cpu->cpu), boost, NULL);
	return boost;
}

static inline void zenith_ignore_dl_rate_limit(struct zenith_cpu *z_cpu,
					       struct zenith_policy *z_policy)
{
	if (cpu_bw_dl(cpu_rq(z_cpu->cpu)) > z_cpu->bw_dl)
		WRITE_ONCE(z_policy->limits_changed, true);
}

/*
 * Compute the cfs_rq util_cfs input fed into schedutil_cpu_util().
 *
 * v1 (default): cpu_util_cfs(rq), which on 5.10 returns
 *
 *      util_avg, optionally maxed with util_est.enqueued when the
 *      UTIL_EST sched_feat is on.
 *
 * v2 (tunable->util_math_v2 = 1): replicates the 6.x-style
 * cpu_util_cfs_boost() shape to give zenith a more responsive signal
 * for short, intermittent tasks (Android UI/render threads). The v2
 * formula is
 *
 *      util_v2 = max(util_avg,
 *                    util_est.enqueued (& ~UTIL_AVG_UNCHANGED),
 *                    runnable_avg)
 *
 * runnable_avg is what tasks contribute *while runnable* (regardless
 * of whether they are currently running), so spikes from
 * intermittent threads land in the util signal one PELT half-life
 * earlier than they do via util_avg alone. The UTIL_AVG_UNCHANGED
 * MSB on util_est.enqueued is masked defensively (cfs_rq sums tasks'
 * enqueued so the bit shouldn't be set there in practice, but
 * guarding against future kernel changes is cheap).
 *
 * Flipping the tunable does not change PELT or util_est accounting --
 * only what zenith feeds into the proportional math. It is safe to
 * toggle at runtime; zenith_invalidate_cache() is hit by the _store
 * path so the next callback recomputes immediately.
 */
static unsigned long zenith_get_util(struct zenith_cpu *z_cpu)
{
	struct rq *rq = cpu_rq(z_cpu->cpu);
	unsigned long util, util_out;
	unsigned long max = arch_scale_cpu_capacity(z_cpu->cpu);
	unsigned int predict_pct;

	z_cpu->max_capacity = max;
	z_cpu->bw_dl = cpu_bw_dl(rq);

	if (z_cpu->z_policy->tunables->util_math_v2) {
		unsigned long util_avg = READ_ONCE(rq->cfs.avg.util_avg);
		unsigned long runnable = READ_ONCE(rq->cfs.avg.runnable_avg);

		util = util_avg;
		if (sched_feat(UTIL_EST)) {
			unsigned long enq = READ_ONCE(rq->cfs.avg.util_est.enqueued);

			enq &= ~UTIL_AVG_UNCHANGED;
			util = max(util, enq);
		}
		util = max(util, runnable);
	} else {
		util = cpu_util_cfs(rq);
	}

	util_out = schedutil_cpu_util(z_cpu->cpu, util, max, FREQUENCY_UTIL, NULL);

	/* Thermal-pressure-aware util derate.  When the SoC thermal
	 * framework has eaten a meaningful fraction of capacity, scale
	 * util_out by (max - pressure) / max so the freq decision
	 * targets what we can actually deliver instead of pinning to
	 * the (already throttled) policy->max.  See the
	 * ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block.  Wrapped
	 * by the thermal_aware master gate so a userspace flip to 0
	 * disables both the level term and the slope term inside this
	 * block at zero hot-path cost when the master gate is on (the
	 * common case).
	 *
	 * Sub-floor pressure is ignored to avoid the multiply cost on
	 * the noise.  trace_zenith_thermal_derate fires only when the
	 * derate actually changes util.
	 */
	if (ZENITH_FEATURE_ENABLED(thermal_aware) &&
	    READ_ONCE(z_cpu->z_policy->tunables->thermal_util_derate) && max) {
		unsigned long pressure = arch_scale_thermal_pressure(z_cpu->cpu);
		unsigned long pressure_pct = (pressure * 100) / max;

		if (pressure_pct >= ZENITH_THERMAL_DERATE_FLOOR_PCT &&
		    pressure < max) {
			unsigned long avail = max - pressure;
			unsigned long derated = (util_out * avail) / max;
			unsigned int rate_pct = READ_ONCE(
				z_cpu->z_policy->tunables->
					thermal_derate_rate_pct);

			/* Derivative term: when pressure is rising,
			 * scale derated further by the single-step rise
			 * relative to capacity, capped to rate_pct.  See
			 * ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT for the
			 * why.  rate_pct == 0 keeps the legacy level-only
			 * shape; the rate-of-change term piggybacks on
			 * the existing gate so it adds no cost when off.
			 */
			if (rate_pct &&
			    pressure > z_cpu->prev_thermal_pressure) {
				unsigned long rise =
					pressure - z_cpu->prev_thermal_pressure;
				unsigned long rise_pct =
					(rise * 100) / max;

				if (rate_pct > 100)
					rate_pct = 100;
				if (rise_pct > rate_pct)
					rise_pct = rate_pct;
				derated = (derated * (100 - rise_pct)) / 100;
			}

			if (trace_zenith_thermal_derate_enabled())
				trace_zenith_thermal_derate(z_cpu->cpu,
							    util_out, derated,
							    (unsigned int)pressure_pct);
			util_out = derated;
		}
		z_cpu->prev_thermal_pressure = pressure;
	}

	/* One-step-ahead linear predictor (up-only). See the
	 * ZENITH_DEFAULT_PREDICT_UTIL_PCT comment block at the top of
	 * this file for semantics. predict_pct == 0 short-circuits the
	 * whole block; non-zero loads the previous unpredicted value
	 * and applies pred = util + delta * pct / 100, taking max with
	 * the observed util so we never under-predict on a ramp-down.
	 * prev_util is updated unconditionally inside the gated block
	 * so disabling the predictor re-arms it cleanly on next enable.
	 */
	predict_pct = READ_ONCE(z_cpu->z_policy->tunables->predict_util_pct);
	if (predict_pct) {
		unsigned long prev = z_cpu->prev_util;
		unsigned int smooth = READ_ONCE(
			z_cpu->z_policy->tunables->predict_util_smooth);

		if (predict_pct > ZENITH_PREDICT_UTIL_PCT_MAX)
			predict_pct = ZENITH_PREDICT_UTIL_PCT_MAX;

		if (util_out > prev) {
			unsigned long slope1 = util_out - prev;
			unsigned long eff_slope = slope1;
			unsigned long pred;

			/* Two-tap smoothing (I7).  Average slope1 with
			 * the previous observed slope only when it was
			 * also positive -- preserves the base predictor's
			 * up-only invariant (we never smooth in a
			 * negative-slope memory).  Cheap: one add, one
			 * shift, no divide.
			 */
			if (smooth && prev > z_cpu->prev_util_2) {
				unsigned long slope2 =
					prev - z_cpu->prev_util_2;
				eff_slope = (slope1 + slope2) >> 1;
			}

			pred = util_out + (eff_slope * predict_pct) / 100;
			if (pred > max)
				pred = max;
			if (trace_zenith_predict_enabled())
				trace_zenith_predict(z_cpu->cpu, predict_pct,
						     util_out, pred);
			z_cpu->prev_util_2 = prev;
			z_cpu->prev_util = util_out;
			util_out = pred;
		} else {
			z_cpu->prev_util_2 = prev;
			z_cpu->prev_util = util_out;
		}
	}

	/* Patch G: background-task util scaling.  When the display
	 * is off, scale util_out down to bg_util_scale_pct percent
	 * of its natural value so the downstream freq decision
	 * lands lower for the same underlying load.  Bypassed when
	 * the screen is on so display-on responsiveness is
	 * unchanged.  100 is a pass-through; the multiply path is
	 * skipped to avoid the cost on the common case.
	 */
	{
		unsigned int scale_pct =
			READ_ONCE(z_cpu->z_policy->tunables->bg_util_scale_pct);
		unsigned int screen =
			READ_ONCE(z_cpu->z_policy->tunables->screen_state);

		if (!screen && scale_pct && scale_pct < 100)
			util_out = (util_out * scale_pct) / 100;
	}

	return util_out;
}

/************************ kcpustat hispeed sampler *********************
 *
 * Adapted from reflex (firelzrd, MIT-compatible / GPL-2.0):
 *
 *   https://github.com/firelzrd/reflex/blob/main/patches/0001-Reflex-CPUFreq-Governor-v0.3.0r2.patch
 *
 * Reflex's insight: PELT util has a 32 ms half-life, so on a sudden
 * busy-from-idle transition the scheduler util signal lags real load
 * by ~96-200 ms. kcpustat (kernel idle-time accounting) gives us the
 * raw busy ratio over the last observation window without smoothing,
 * which is perfect as a *temporary* util floor. To avoid double
 * counting once PELT catches up, we decay the floor with PELT's own
 * 32 ms half-life so total coverage stays ~100% across the ramp.
 *
 * Reflex implements the decay in log-domain with a 256-entry LUT to
 * get 1 ms granularity.  For Android phones we trade that precision
 * for simplicity: we decay in 32 ms quanta with a single right-shift,
 * which is exact at half-life boundaries and a few percent off in
 * between.  That's well below noise floor for cpufreq decisions.
 *
 * The integration point is the upcoming patch that calls
 * zenith_kcpustat_blend() in zenith_update_{single,shared}.  This
 * patch adds the sampler and the blend helper but doesn't call
 * either, so it is a pure no-op until the integration patch lands.
 */

/* Cap the decay shift so >> shift always produces 0 when the
 * contribution should be negligible.  After 8 half-lives the floor is
 * 1/256 of its initial value, well past the point of mattering.
 */
#define ZENITH_KC_DECAY_HALF_LIFE_MS	32
#define ZENITH_KC_DECAY_MAX_SHIFT	8

/*
 * Sample CPU busy ratio over the last window via kcpustat.
 *
 * Two-phase: when the window has elapsed, phase 1 latches the new
 * prev_idle / prev_wall snapshot and arms hispeed_active.  Phase 2
 * runs on the next callback and computes the actual busy_pct delta;
 * this avoids racing time-of-day skew between the snapshot reset and
 * the busy% calculation in the same callback.
 *
 * busy_pct flows through an asymmetric EWMA (instant up, decay down
 * by filter_shift).  filter_shift==0 disables the EWMA and uses the
 * raw value.
 *
 * Updates the hispeed decay timer:
 *   - any non-zero filtered busy_pct refreshes log_hispeed and starts
 *     (or keeps) hispeed_start_ns;
 *   - two consecutive idle windows clear the timer so the floor goes
 *     fully transparent until activity resumes.
 *
 * Caller must serialise (single path: only one CPU writes; shared
 * path: caller holds update_lock).
 */
static void zenith_kcpustat_sample(struct zenith_cpu *z_cpu,
				   unsigned int window_us,
				   unsigned int filter_shift, u64 time)
{
	u64 cur_idle, cur_wall;
	u64 wall_delta, idle_delta;

	cur_idle = get_cpu_idle_time(z_cpu->cpu, &cur_wall, 1);
	/* kc_prev_wall_time and cur_wall are u64 microseconds; the
	 * delta can exceed UINT_MAX on long-idle CPUs (~71 minutes).
	 * Truncating to unsigned int there made wall_delta wrap to a
	 * tiny value and immediately falsely satisfy the
	 * wall_delta >= window_us phase-1 condition, so the sampler
	 * silently re-armed instead of producing a real busy_pct.
	 * Keep everything u64 through the divide.
	 */
	wall_delta = cur_wall - z_cpu->kc_prev_wall_time;

	if (wall_delta >= window_us) {
		/*
		 * Phase 1: window elapsed.  Latch the new prev_*
		 * snapshot and let the next callback take the actual
		 * busy delta.  We don't touch the decay timer here:
		 * the momentary busy_pct=0 is a measurement artefact,
		 * not a genuine idle signal.
		 */
		z_cpu->kc_busy_pct = 0;
		z_cpu->kc_hispeed_active = true;
		z_cpu->kc_prev_idle_time = cur_idle;
		z_cpu->kc_prev_wall_time = cur_wall;
		return;
	}

	/* Within the current window. Skip unless phase 2 is armed. */
	if (!z_cpu->kc_hispeed_active)
		return;

	z_cpu->kc_hispeed_active = false;

	idle_delta = (cur_idle > z_cpu->kc_prev_idle_time) ?
		     (cur_idle - z_cpu->kc_prev_idle_time) : 0;

	z_cpu->kc_busy_pct = (wall_delta > idle_delta) ?
		(unsigned int)div64_u64(100ULL * (wall_delta - idle_delta),
					wall_delta) : 0;

	z_cpu->kc_prev_idle_time = cur_idle;
	z_cpu->kc_prev_wall_time = cur_wall;

	/* Asymmetric EWMA: instant up, configurable decay down. */
	if (!filter_shift ||
	    z_cpu->kc_busy_pct >= z_cpu->kc_filtered_busy_pct) {
		z_cpu->kc_filtered_busy_pct = z_cpu->kc_busy_pct;
	} else {
		unsigned int step =
			(z_cpu->kc_filtered_busy_pct - z_cpu->kc_busy_pct)
			>> filter_shift;
		z_cpu->kc_filtered_busy_pct -= step ? step :
			(z_cpu->kc_filtered_busy_pct - z_cpu->kc_busy_pct);
	}

	/*
	 * Decay timer with one-window grace period: avoid resetting
	 * hispeed_start_ns on every transient idle window so a busy
	 * task with sub-window idle gaps still sees a coherent decay
	 * trajectory.
	 */
	if (z_cpu->kc_filtered_busy_pct) {
		z_cpu->kc_idle_windows = 0;
		if (!z_cpu->kc_hispeed_start_ns)
			z_cpu->kc_hispeed_start_ns = time;
	} else if (++z_cpu->kc_idle_windows >= 2) {
		z_cpu->kc_hispeed_start_ns = 0;
		z_cpu->kc_filtered_busy_pct = 0;
	}
}

/*
 * Blend PELT util with a kcpustat-derived util floor that decays at
 * PELT's 32 ms half-life.  Returns pelt_util unchanged when the
 * blend is inactive (zero busy%, no start timestamp, or decay fully
 * elapsed).  When active, returns pelt_util plus the decayed floor,
 * capped at the raw kcpustat-implied util (so the blend can never
 * exceed the actual measured busy fraction).
 */
static unsigned long zenith_kcpustat_blend(struct zenith_cpu *z_cpu,
					   unsigned long pelt_util,
					   unsigned long max_cap, u64 time)
{
	unsigned long hispeed_util, decayed;
	u64 elapsed_ns;
	unsigned int half_lives;

	if (!z_cpu->kc_filtered_busy_pct || !z_cpu->kc_hispeed_start_ns ||
	    !max_cap)
		return pelt_util;

	hispeed_util = (max_cap * z_cpu->kc_filtered_busy_pct) / 100u;
	if (hispeed_util <= pelt_util)
		return pelt_util;

	elapsed_ns = time - z_cpu->kc_hispeed_start_ns;
	half_lives = (unsigned int)
		(elapsed_ns / (ZENITH_KC_DECAY_HALF_LIFE_MS * NSEC_PER_MSEC));

	if (half_lives >= ZENITH_KC_DECAY_MAX_SHIFT)
		return pelt_util;

	decayed = hispeed_util >> half_lives;
	if (decayed <= pelt_util)
		return pelt_util;

	return min(pelt_util + decayed, hispeed_util);
}

/************************ Nice-load sampling *********************************/

/* Return the fraction (0..100) of wall time since the last call that
 * this CPU spent in niced-user mode. Uses the scheduler's cputime
 * accounting (same source as /proc/stat). Only called when
 * ignore_nice_load=1 so the cost is paid lazily.
 */
static unsigned int zenith_sample_nice_pct(struct zenith_cpu *z_cpu, u64 now)
{
	struct kernel_cpustat *kcs = &kcpustat_cpu(z_cpu->cpu);
	u64 cur_nice = kcpustat_field(kcs, CPUTIME_NICE, z_cpu->cpu);
	u64 nice_delta, wall_delta;
	unsigned int pct;

	if (unlikely(!z_cpu->prev_nice_wall)) {
		z_cpu->prev_nice_time = cur_nice;
		z_cpu->prev_nice_wall = now;
		return 0;
	}

	nice_delta = cur_nice - z_cpu->prev_nice_time;
	wall_delta = now - z_cpu->prev_nice_wall;
	z_cpu->prev_nice_time = cur_nice;
	z_cpu->prev_nice_wall = now;

	if (!wall_delta)
		return 0;
	if (nice_delta > wall_delta)
		nice_delta = wall_delta;

	pct = (unsigned int)((nice_delta * 100) / wall_delta);
	return pct > 100 ? 100 : pct;
}

/************************ Thermal State Resolution ***************************/

/* Return true when zenith should behave as if thermally throttled.
 *
 * Userspace-written thermal_state=1 always wins. When thermal_auto=1 is
 * set, we additionally consult arch_scale_thermal_pressure() on the
 * first CPU of the policy and consider the policy throttled when
 * pressure has eaten at least ZENITH_THERMAL_AUTO_PRESSURE_PCT of the
 * capacity. This lets the governor respond to the kernel thermal
 * framework (via arch_update_thermal_pressure) with no userspace in
 * the loop.
 */
static bool zenith_thermal_active(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long pressure, cap;
	bool active;
	int cpu;

	/* Master gate.  When thermal_aware == 0 every consumer of this
	 * helper (the screen-off thermal cliff path, the V2 sustained
	 * paths that read pressure, anything that calls
	 * zenith_thermal_active() directly) sees "cool" regardless of
	 * actual pressure or thermal_state.  Static-key gated for
	 * branchless cost when on, the common case.  Also clear the
	 * sysfs mirror so userspace never sees a stale 1 after a
	 * userspace flip of thermal_aware to 0.
	 */
	if (!ZENITH_FEATURE_ENABLED(thermal_aware)) {
		WRITE_ONCE(tunables->thermal_active, 0);
		return false;
	}

	if (tunables->thermal_state) {
		WRITE_ONCE(tunables->thermal_active, 1);
		return true;
	}
	if (!tunables->thermal_auto) {
		WRITE_ONCE(tunables->thermal_active, 0);
		return false;
	}

	cpu = cpumask_first(policy->cpus);
	if (cpu >= nr_cpu_ids) {
		WRITE_ONCE(tunables->thermal_active, 0);
		return false;
	}

	cap = arch_scale_cpu_capacity(cpu);
	if (!cap) {
		WRITE_ONCE(tunables->thermal_active, 0);
		return false;
	}

	pressure = arch_scale_thermal_pressure(cpu);
	active = (pressure * 100 / cap) >= ZENITH_THERMAL_AUTO_PRESSURE_PCT;
	WRITE_ONCE(tunables->thermal_active, active ? 1 : 0);
	return active;
}

static unsigned int zenith_policy_thermal_pressure_pct(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned long pressure;
	unsigned long cap;
	int cpu;

	cpu = cpumask_first(policy->cpus);
	if (cpu >= nr_cpu_ids)
		return 0;

	cap = arch_scale_cpu_capacity(cpu);
	if (!cap)
		return 0;

	pressure = arch_scale_thermal_pressure(cpu);
	return min_t(unsigned int, (pressure * 100) / cap, 100);
}

/************************ Energy Model (EM) Evaluation ***********************/

static unsigned int zenith_em_cap_freq(struct zenith_policy *z_policy, unsigned int target_freq)
{
	struct cpufreq_policy *policy = z_policy->policy;
	struct em_perf_domain *pd = em_cpu_get(policy->cpu);
	struct em_perf_state *ps;
	int i;

	/* If no Energy Model is registered or we aren't thermal throttling, skip */
	if (!pd || !zenith_thermal_active(z_policy))
		return target_freq;

	/* Scan EM array to find the mW cost of the target frequency */
	for (i = 0; i < pd->nr_perf_states; i++) {
		ps = &pd->table[i];
		if (ps->frequency >= target_freq) {
			/*
			 * If this state consumes disproportionately high power (heuristic:
			 * if it's the absolute highest state and we are throttling), cap it
			 * to the previous state to save mW.
			 */
			if (i == pd->nr_perf_states - 1 && i > 0)
				return pd->table[i - 1].frequency;
			break;
		}
	}
	return target_freq;
}

/************************ Zenith Scaling Math ***********************/

/* Spike detection threshold: when the requested frequency jump exceeds
 * policy->max >> ZENITH_SPIKE_SHIFT, bypass the up_rate_limit entirely.
 * This gives instant response to load spikes (task wakeup, game frame
 * start, UI touch) while still rate-limiting small, noise-driven
 * oscillations.
 */
#define ZENITH_SPIKE_SHIFT	3	/* 1 << 3 = divide by 8 = 12.5% */

static bool zenith_up_down_rate_limit(struct zenith_policy *z_policy, u64 time,
				      unsigned int next_freq)
{
	s64 delta_ns = time - z_policy->last_freq_update_time;
	struct zenith_tunables *tunables = z_policy->tunables;

	/* Snapshot the cached delays once.  Concurrent writers are
	 * profile_store / auto_tune / up_rate_limit_us_store; readers
	 * are this hot path and zenith_should_update_freq().  Without
	 * READ_ONCE the compiler may reload between the comparison and
	 * the down_delay multiply, yielding inconsistent decisions.
	 */
	s64 up_delay = READ_ONCE(z_policy->up_rate_delay_ns);
	s64 down_delay = READ_ONCE(z_policy->down_rate_delay_ns) *
			 (s64)max(z_policy->down_rate_mult, 1U);

	if (READ_ONCE(tunables->down_rate_adaptive)) {
		unsigned int var = READ_ONCE(z_policy->load_var_ewma_x256);

		if (var > 256)
			var = 256;
		down_delay = (down_delay * (256 + var)) / 256;
	}

	/* Input-boost-aware down-rate extension.  While the
	 * input_boost full-pin window is in effect (now <
	 * zenith_input_boost_until_ns), multiply down_delay by
	 * tunables->input_boost_down_rate_mult_pct / 100.  Holds the
	 * cluster up across the inter-frame gaps inside an active
	 * scroll / swipe.  See ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_-
	 * MULT_PCT for the full rationale.  No-op when:
	 *   - the multiplier is at 100%% (legacy behaviour);
	 *   - no input boost is active (until == 0 or expired);
	 *   - the configured cluster gate (input_boost_big_only)
	 *     would suppress the boost itself for this cluster
	 *     (LITTLE in the default config, where down-rate
	 *     extension would just delay parking the cluster).
	 */
	{
		unsigned int mult_pct =
			READ_ONCE(tunables->input_boost_down_rate_mult_pct);

		if (mult_pct > 100 &&
		    (!READ_ONCE(tunables->input_boost_big_only) ||
		     z_policy->is_big_cluster)) {
			u64 until = (u64)atomic64_read(
					&zenith_input_boost_until_ns);

			if (until && (u64)time < until)
				down_delay = (down_delay * mult_pct) / 100;
		}
	}

	if (next_freq > z_policy->next_freq) {
		unsigned int spike = z_policy->policy->max >> ZENITH_SPIKE_SHIFT;
		unsigned int cpu;
		struct zenith_cpu *z_cpu;

		for_each_cpu(cpu, z_policy->policy->cpus) {
			z_cpu = &per_cpu(zenith_cpu, cpu);

			/* Tick-based bypass: legacy fast path that bypasses
			 * the up-rate limit for ZENITH_WAKEUP_BOOST_TICKS
			 * upward transitions after an idle->busy detection.
			 */
			if (z_cpu->wakeup_boost_ticks) {
				z_cpu->wakeup_boost_ticks--;
				return false;
			}

			/* Wall-clock-based bypass armed by wakeup_boost_ms.
			 * Stays active until the deadline passes; no need
			 * to decrement.  Self-disarms on first sample past
			 * the deadline so subsequent ticks fall through to
			 * the normal up_rate_limit gate.
			 */
			if (z_cpu->wakeup_boost_until_ns) {
				if (ktime_get_ns() <
				    z_cpu->wakeup_boost_until_ns)
					return false;
				z_cpu->wakeup_boost_until_ns = 0;
			}
		}
		if (next_freq - z_policy->next_freq >= spike)
			return false;
		if (delta_ns < up_delay)
			return true;
	}

	if (next_freq < z_policy->next_freq && delta_ns < down_delay)
		return true;

	return false;
}

static bool zenith_should_update_freq(struct zenith_policy *z_policy, u64 time)
{
	s64 delta_ns;

	if (!cpufreq_this_cpu_can_update(z_policy->policy))
		return false;

	/* Pair the smp_store_release() in zenith_limits() with an
	 * smp_load_acquire() here so the new policy->{min,max} written
	 * by cpufreq_policy_apply_limits() is observed by the
	 * subsequent eval on this CPU.  The previous smp_wmb()/smp_mb()
	 * pair only ordered prior stores; it did not guarantee that the
	 * load of policy->{min,max} below the flag check was observed
	 * after the store of those fields above the flag store on the
	 * writer side.
	 */
	if (unlikely(smp_load_acquire(&z_policy->limits_changed))) {
		WRITE_ONCE(z_policy->limits_changed, false);
		z_policy->need_freq_update = true;
		return true;
	}

	if (z_policy->work_in_progress)
		return true;

	delta_ns = time - z_policy->last_freq_update_time;
	return delta_ns >= READ_ONCE(z_policy->min_rate_limit_ns);
}

/*
 * Return the max RQ-aggregated UCLAMP_MIN (in capacity units) across all
 * CPUs in this policy.  Uses uclamp_rq_get() which is just a READ_ONCE
 * on the rq->uclamp[UCLAMP_MIN].value field already maintained by the
 * scheduler on every enqueue/dequeue -- no locking needed, no
 * measurable fast-path cost even on 8-CPU policies.
 *
 * Returns 0 if CONFIG_UCLAMP_TASK is off (uclamp_rq_get stub returns 0)
 * or if uclamp is compiled in but no task on the policy has set
 * uclamp_min.  Callers use the 0 return as "no floor to apply".
 */
/* Refresh both per-policy uclamp aggregates in a single rq walk when
 * the cache is cold or stale.  Called from the uclamp_min / uclamp_max
 * helpers below; not meant to be invoked directly.
 */
static void zenith_uclamp_cache_refresh(struct zenith_policy *z_policy)
{
#ifdef CONFIG_UCLAMP_TASK
	unsigned long max_umin = 0;
	unsigned long max_umax = 0;
	int cpu;

	if (!uclamp_is_used()) {
		z_policy->cached_uclamp_min = 0;
		z_policy->cached_uclamp_max = SCHED_CAPACITY_SCALE;
		z_policy->uclamp_cache_stamp_ns = ktime_get_ns();
		return;
	}

	for_each_cpu(cpu, z_policy->policy->cpus) {
		struct rq *rq = cpu_rq(cpu);
		unsigned long umin = uclamp_rq_get(rq, UCLAMP_MIN);
		unsigned long umax = uclamp_rq_get(rq, UCLAMP_MAX);

		if (umin > max_umin)
			max_umin = umin;
		if (umax > max_umax)
			max_umax = umax;
	}
	z_policy->cached_uclamp_min = max_umin;
	z_policy->cached_uclamp_max = max_umax ? max_umax : SCHED_CAPACITY_SCALE;
	z_policy->uclamp_cache_stamp_ns = ktime_get_ns();
#else
	z_policy->cached_uclamp_min = 0;
	z_policy->cached_uclamp_max = SCHED_CAPACITY_SCALE;
	z_policy->uclamp_cache_stamp_ns = ktime_get_ns();
#endif
}

static inline bool zenith_uclamp_cache_fresh(struct zenith_policy *z_policy)
{
	u64 stamp = z_policy->uclamp_cache_stamp_ns;

	if (!stamp)
		return false;
	return ktime_get_ns() - stamp < ZENITH_UCLAMP_CACHE_TTL_NS;
}

static unsigned long zenith_policy_uclamp_min(struct zenith_policy *z_policy)
{
	if (!zenith_uclamp_cache_fresh(z_policy))
		zenith_uclamp_cache_refresh(z_policy);
	return z_policy->cached_uclamp_min;
}

/* Collect the effective uclamp_max for the policy.
 *
 * uclamp_max is Android's opt-in power-saving hint
 * (PerformanceHint.setPreferPowerEfficiency() -> per-task UCLAMP_MAX).
 * uclamp_rq_get(rq, UCLAMP_MAX) returns the maximum UCLAMP_MAX over
 * currently enqueued tasks, which honours the rule "if any task on
 * this rq wants no cap, don't cap".  The same rule has to hold at
 * policy granularity: if any CPU in the policy has a task that
 * doesn't want the freq capped, we must not cap the freq for the
 * whole cluster.  That's an _max_ reduction across per-rq values.
 *
 * SCHED_CAPACITY_SCALE is the canonical "no cap" sentinel (tasks
 * with no explicit UCLAMP_MAX set).  Callers treat any value
 * >= SCHED_CAPACITY_SCALE as "tier disabled, apply no freq cap".
 *
 * Returns SCHED_CAPACITY_SCALE if CONFIG_UCLAMP_TASK is off or uclamp
 * is not in use -- callers interpret that as "no cap".
 */
static unsigned long zenith_policy_uclamp_max(struct zenith_policy *z_policy)
{
	if (!zenith_uclamp_cache_fresh(z_policy))
		zenith_uclamp_cache_refresh(z_policy);
	return z_policy->cached_uclamp_max;
}

/* Effective hispeed floor in kHz for this policy.  If tunables->hispeed_freq
 * is set explicitly, honour it verbatim (legacy behaviour).  Otherwise fall
 * back to the per-cluster auto-default: (policy->max * hispeed_freq_pct / 100).
 * Returns 0 when the tier is disabled (both absolute and percentage values
 * are zero, or hispeed_freq_pct is zero while hispeed_freq is zero).
 *
 * When game_mode=1, the percentage path is multiplied by
 * ZENITH_GAME_HISPEED_BOOST_PCT/100 so userspace game-detection
 * daemons can lift the per-cluster auto-default floor without
 * touching hispeed_freq_pct itself.  The absolute hispeed_freq path
 * is left untouched: a userspace value written there is honoured
 * verbatim even with game_mode=1.
 */
static inline unsigned int zenith_eff_hispeed_freq(struct zenith_policy *z_policy)
{
	unsigned int eff = z_policy->tunables->hispeed_freq;
	unsigned int pct = z_policy->tunables->hispeed_freq_pct;

	if (!eff && pct) {
		unsigned int gm = zenith_eff_game_mode(
				z_policy->tunables->game_mode);

		if (gm >= 2)
			pct = (pct * ZENITH_GAME_L2_HISPEED_BOOST_PCT) / 100;
		else if (gm == 1)
			pct = (pct * ZENITH_GAME_HISPEED_BOOST_PCT) / 100;
		eff = (z_policy->policy->max * pct) / 100;
		if (eff > z_policy->policy->max)
			eff = z_policy->policy->max;
	}
	return eff;
}

/* B10-3: optional per-cgroup PSI source (see ZENITH_PSI_CGROUP_PATH_MAX
 * and psi_cgroup_path).
 *
 * zenith_psi_cgroup is the resolved cgroup whose embedded psi_group
 * (cgroup->psi) the three zenith_psi_*_some_pct() helpers below
 * dereference under rcu_read_lock().  NULL means "use psi_system",
 * which is the pre-B10 behaviour and the default at boot.
 *
 * Gated on CONFIG_PSI && CONFIG_CGROUPS.  Without CONFIG_CGROUPS the
 * struct cgroup type is forward-declared only and cgroup_psi(),
 * cgroup_get_from_path(), cgroup_put() are absent; the cached pointer
 * and apply helper degrade to no-ops, the picker returns &psi_system
 * unconditionally, and the sysfs store accepts (and silently
 * discards) any path.
 *
 * Lifecycle:
 *   - zenith_psi_cgroup_apply() is the only writer.  It serialises
 *     mutators with zenith_psi_cgroup_lock, holds a refcount on the
 *     cached cgroup (via cgroup_get_from_path), drops the previous
 *     refcount with cgroup_put() after a synchronize_rcu() so the
 *     readers below have left their grace period.
 *   - zenith_psi_cgroup_active_path is the most-recently-applied path
 *     string.  Identical-path stores are no-ops, so repeat profile-
 *     bake or sysfs writes don't churn the cgroup ref.
 *   - The cgroup ref lives as long as the path is set; zenith is
 *     built-in (Kconfig: bool), so we never run a teardown path.
 *
 * Hot-path readers do a single rcu_read_lock() / rcu_dereference() /
 * READ_ONCE() / rcu_read_unlock(); RCU read-side critical sections
 * are essentially free under PREEMPT_RCU (no atomic, no memory
 * barrier on the load side), so this stays cheap when the feature
 * is on, and is identical to the pre-B10 read when the cached
 * pointer is NULL.
 */
#if defined(CONFIG_PSI) && defined(CONFIG_CGROUPS)
static struct cgroup __rcu *zenith_psi_cgroup;
static DEFINE_MUTEX(zenith_psi_cgroup_lock);
static char zenith_psi_cgroup_active_path[ZENITH_PSI_CGROUP_PATH_MAX];
#endif

/* Replace the cached zenith_psi_cgroup pointer to match @path.
 *
 *   path == ""    -> drop to NULL (system-wide PSI), the safe default.
 *   path == X     -> resolve via cgroup_get_from_path(); on success
 *                    swap the cached pointer and drop the old refcount;
 *                    on failure (not mounted, missing, cgroup-v1-only)
 *                    leave the cache as-is and clear active_path so a
 *                    later identical-path store will retry.
 *
 * Caller is the sysfs store path or zenith_apply_profile().  Both run
 * outside any hot path.  No-op on identical path matches and on
 * !CONFIG_CGROUPS / !CONFIG_PSI builds.
 */
static void zenith_psi_cgroup_apply(const char *path)
{
#if defined(CONFIG_PSI) && defined(CONFIG_CGROUPS)
	struct cgroup *new_cgrp = NULL;
	struct cgroup *old_cgrp;

	if (!path)
		path = "";

	mutex_lock(&zenith_psi_cgroup_lock);

	if (!strncmp(zenith_psi_cgroup_active_path, path,
		     ZENITH_PSI_CGROUP_PATH_MAX))
		goto out_unlock;

	if (path[0]) {
		new_cgrp = cgroup_get_from_path(path);
		if (IS_ERR(new_cgrp)) {
			new_cgrp = NULL;
			zenith_psi_cgroup_active_path[0] = '\0';
			goto out_unlock;
		}
	}

	old_cgrp = rcu_dereference_protected(zenith_psi_cgroup,
			lockdep_is_held(&zenith_psi_cgroup_lock));
	rcu_assign_pointer(zenith_psi_cgroup, new_cgrp);
	strscpy(zenith_psi_cgroup_active_path, path,
		sizeof(zenith_psi_cgroup_active_path));

	if (old_cgrp) {
		synchronize_rcu();
		cgroup_put(old_cgrp);
	}

out_unlock:
	mutex_unlock(&zenith_psi_cgroup_lock);
#endif
}

/* Pick the psi_group the zenith_psi_*_some_pct() helpers should read
 * from for the current call.  rcu_read_lock() must be held by the
 * caller; the returned pointer is only valid for the duration of
 * that read-side critical section.
 *
 * Falls back to &psi_system if the cgroup-v2 cached pointer is NULL
 * (or if CONFIG_CGROUPS=n), which is the pre-B10 behaviour and the
 * cold-boot default.
 */
#ifdef CONFIG_PSI
static __always_inline struct psi_group *zenith_psi_pick_group(void)
{
#ifdef CONFIG_CGROUPS
	struct cgroup *cgrp = rcu_dereference(zenith_psi_cgroup);

	return cgrp ? &cgrp->psi : &psi_system;
#else
	return &psi_system;
#endif
}
#endif

/* Read the memory pressure 10s average from PSI as an integer
 * percentage (0..100).  Returns 0 when CONFIG_PSI is off, when
 * psi_disabled is set, or when the value isn't yet populated (early
 * boot).
 *
 * Source psi_group is selected by zenith_psi_pick_group(): the
 * B10-3 cached cgroup-v2 group when one is configured via
 * psi_cgroup_path, otherwise psi_system (system-wide PSI, the
 * pre-B10 behaviour).  Single READ_ONCE on the EWMA word inside a
 * minimal rcu_read_lock() critical section -- the avgs_work
 * aggregator is what writes to ->avg[][] and we tolerate up to 2 s
 * of staleness on the read.
 */
static inline unsigned int zenith_psi_mem_some_pct(void)
{
#ifdef CONFIG_PSI
	struct psi_group *grp;
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	rcu_read_lock();
	grp = zenith_psi_pick_group();
	avg = READ_ONCE(grp->avg[PSI_MEM_SOME][0]);
	rcu_read_unlock();
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

/* Same shape as zenith_psi_mem_some_pct() for the PSI_CPU_SOME and
 * PSI_IO_SOME dimensions.  See the psi_aware / psi_*_thresh comment
 * block for what each pressure source means.  Both helpers honour
 * the same B10-3 cgroup-v2 cached pick, are lock-free outside the
 * RCU read-side, and tolerate CONFIG_PSI=n at compile time.
 */
static inline unsigned int zenith_psi_cpu_some_pct(void)
{
#ifdef CONFIG_PSI
	struct psi_group *grp;
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	rcu_read_lock();
	grp = zenith_psi_pick_group();
	avg = READ_ONCE(grp->avg[PSI_CPU_SOME][0]);
	rcu_read_unlock();
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

static inline unsigned int zenith_psi_io_some_pct(void)
{
#ifdef CONFIG_PSI
	struct psi_group *grp;
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	rcu_read_lock();
	grp = zenith_psi_pick_group();
	avg = READ_ONCE(grp->avg[PSI_IO_SOME][0]);
	rcu_read_unlock();
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

/* RCU-protected, sysfs-configurable comm-prefix lists.
 *
 * The render / audio / camera comm tables used to be three plain
 * `static const char * const X[]` arrays compiled into the kernel.
 * Vendors using non-AOSP userspace -- MTK skins, Qualcomm OEM
 * builds, OEM-rebrand audio servers -- needed kernel rebuilds just
 * to add their thread names.  This block reworks the storage as a
 * trio of RCU-protected tables, populated at init from the original
 * arrays (so out-of-box behaviour is unchanged) and replaceable
 * lock-free via three new sysfs nodes (render_comms / audio_comms
 * / camera_comms, comma-separated).
 *
 * The reader (each of the three zenith_policy_has_X helpers) does:
 *   rcu_read_lock();
 *   table = rcu_dereference(zenith_<X>_table);
 *   for (i = 0; i < table->nr; i++) strncmp(curr->comm, table->entries[i], ...);
 *   rcu_read_unlock();
 *
 * The writer (each of the three sysfs_store helpers) builds an
 * entirely new struct zenith_comm_table from the CSV, RCU-swaps the
 * pointer, then kfree_rcu()'s the old one.  No hot-path allocation,
 * no lock contention with the readers, no synchronize_rcu() at
 * commit time -- the readers walk a stable snapshot until their
 * grace period closes, the kfree_rcu callback drops the old table
 * once everyone has moved on.
 *
 * Storage layout: each table is a single allocation containing the
 * pointer array plus the raw NUL-separated buffer the entries[]
 * pointers index into, sized to ZENITH_COMM_LIST_MAX entries and
 * ZENITH_COMM_BUF_MAX raw bytes.  Empty CSV (just "\n" or "")
 * resets to defaults; a parse error fails the entire write so
 * userspace doesn't see a half-applied table.
 */
#define ZENITH_COMM_LIST_MAX		32
#define ZENITH_COMM_BUF_MAX		1024

struct zenith_comm_table {
	struct rcu_head	rcu;
	unsigned int	nr;
	const char	*entries[ZENITH_COMM_LIST_MAX];
	char		raw[ZENITH_COMM_BUF_MAX];
};

static struct zenith_comm_table __rcu *zenith_render_table;
static struct zenith_comm_table __rcu *zenith_audio_table;
static struct zenith_comm_table __rcu *zenith_camera_table;
static struct zenith_comm_table __rcu *zenith_game_auto_table;
static DEFINE_MUTEX(zenith_comm_table_lock);

static struct zenith_comm_table *
zenith_alloc_comm_table_from_defaults(const char * const *defaults,
				      size_t nr_defaults)
{
	struct zenith_comm_table *t;
	size_t off = 0;
	unsigned int i;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return NULL;

	for (i = 0; i < nr_defaults && i < ZENITH_COMM_LIST_MAX; i++) {
		size_t len = strlen(defaults[i]) + 1;

		if (off + len > ZENITH_COMM_BUF_MAX)
			break;
		memcpy(t->raw + off, defaults[i], len);
		t->entries[i] = t->raw + off;
		off += len;
		t->nr = i + 1;
	}
	return t;
}

/* Parse a CSV (entries separated by ',' or whitespace) into a fresh
 * zenith_comm_table.  Returns NULL on alloc failure, ERR_PTR on
 * parse error.  Caller owns the table and must rcu_assign_pointer
 * + kfree_rcu the old one to publish.
 */
static struct zenith_comm_table *
zenith_alloc_comm_table_from_csv(const char *buf, size_t count)
{
	struct zenith_comm_table *t;
	size_t off = 0;
	const char *p = buf;
	const char *end = buf + count;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return NULL;

	while (p < end && t->nr < ZENITH_COMM_LIST_MAX) {
		const char *tok_start;
		size_t len;

		while (p < end && (*p == ',' || *p == ' ' ||
				   *p == '\t' || *p == '\n'))
			p++;
		if (p == end)
			break;

		tok_start = p;
		while (p < end && *p != ',' && *p != ' ' &&
		       *p != '\t' && *p != '\n')
			p++;
		len = p - tok_start;
		if (!len)
			continue;
		if (off + len + 1 > ZENITH_COMM_BUF_MAX) {
			kfree(t);
			return ERR_PTR(-ENOSPC);
		}
		memcpy(t->raw + off, tok_start, len);
		t->raw[off + len] = '\0';
		t->entries[t->nr++] = t->raw + off;
		off += len + 1;
	}
	return t;
}

static ssize_t zenith_show_comm_table(struct zenith_comm_table __rcu **slot,
				      char *buf)
{
	struct zenith_comm_table *t;
	ssize_t len = 0;
	unsigned int i;
	bool first = true;

	rcu_read_lock();
	t = rcu_dereference(*slot);
	if (t) {
		for (i = 0; i < t->nr; i++) {
			len += sysfs_emit_at(buf, len,
					 "%s%s", first ? "" : ",",
					 t->entries[i]);
			first = false;
		}
	}
	rcu_read_unlock();
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static ssize_t
zenith_store_comm_table(struct zenith_comm_table __rcu **slot,
			const char *const *defaults, size_t nr_defaults,
			const char *buf, size_t count)
{
	struct zenith_comm_table *new_t;
	struct zenith_comm_table *old_t;
	const char *p = buf;
	const char *end = buf + count;

	while (p < end && (*p == ' ' || *p == '\t' || *p == '\n'))
		p++;
	if (p == end)
		new_t = zenith_alloc_comm_table_from_defaults(defaults,
							      nr_defaults);
	else
		new_t = zenith_alloc_comm_table_from_csv(buf, count);

	if (!new_t)
		return -ENOMEM;
	if (IS_ERR(new_t))
		return PTR_ERR(new_t);

	mutex_lock(&zenith_comm_table_lock);
	old_t = rcu_dereference_protected(*slot,
			lockdep_is_held(&zenith_comm_table_lock));
	rcu_assign_pointer(*slot, new_t);
	mutex_unlock(&zenith_comm_table_lock);

	if (old_t)
		kfree_rcu(old_t, rcu);
	return count;
}

/* List of comm prefixes treated as render / display-pipeline threads.
 * Matched by strncmp() over the first N characters where N is the
 * length of the table entry, so the userspace task only needs to
 * have its first N chars match (Android's "RenderThread NNN" naming
 * for libui's per-app render thread, for instance, matches the
 * "RenderThread" prefix).  Used as the seed values for the RCU
 * zenith_render_table at zenith_gov_init() time and as the reset
 * target when the render_comms sysfs node is written empty; live
 * matching always reads the RCU table.
 *
 * Coverage rationale (B-AUTO-1 expansion -- entries are vendor-
 * comprehensive so the zenith auto-profile selector cannot miss a
 * render-thread context, and so manual render_aware mode catches
 * non-AOSP graphics stacks too):
 *   - RenderThread         AOSP libui per-app render thread ("RenderThread N")
 *   - surfaceflinger       SurfaceFlinger main thread
 *   - RenderEngine         SurfaceFlinger render engine worker
 *   - mali-cmar-back       ARM Mali Bifrost / Valhall command-stream backend
 *   - GLThread             SurfaceView Java GL thread ("GLThread N"; cocos2d, libgdx, ...)
 *   - kgsl_worker_th       Qualcomm Adreno KGSL worker (truncated from kgsl_worker_thread)
 *   - kgsl-3d0             Qualcomm Adreno KGSL device thread
 *   - kbase_event          ARM Mali Bifrost / Valhall event-completion thread
 *   - composer-servic      HWC2 composer HAL service (truncated from composer-service)
 *   - vsync_thread         generic vsync producer thread
 *   - Choreographer        Android frame Choreographer thread
 *   - UnrealRenderTh       Unreal Engine render thread (truncated)
 *   - Cocos2d-Render       Cocos2d-x render thread (truncated)
 *   - CompositorThr        Chromium / WebView compositor thread (truncated)
 *
 * All entries are <= 15 chars to fit within task->comm[16] (the
 * trailing NUL leaves 15 usable bytes); strncmp() compares only
 * strlen(needle) bytes, so adding extra entries costs at most one
 * cache-miss-bounded strncmp loop iteration per CPU.  The hot path
 * is gated by ZENITH_RENDER_CACHE_TTL_NS so even a 14-entry walk
 * runs at most a few times per second per policy.
 */
static const char * const zenith_render_comms[] = {
	"RenderThread",
	"surfaceflinger",
	"RenderEngine",
	"mali-cmar-back",
	"GLThread",
	"kgsl_worker_th",
	"kgsl-3d0",
	"kbase_event",
	"composer-servic",
	"vsync_thread",
	"Choreographer",
	"UnrealRenderTh",
	"Cocos2d-Render",
	"CompositorThr",
};

/* Walk the policy's online cpumask and check each cpu_curr's comm
 * against zenith_render_comms[].  Returns true on the first match.
 * The result is cached for ZENITH_RENDER_CACHE_TTL_NS so a hot path
 * (e.g. a 60 / 90 / 120 Hz scroll) only does the strncmp loop a few
 * times per second.  Caller is expected to gate the call on
 * tunables->render_aware != 0; this helper does not re-check that.
 */
static bool zenith_policy_has_render(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;
	unsigned int matched_util = 0;

	if (z_policy->render_cache_stamp_ns &&
	    now - z_policy->render_cache_stamp_ns < ZENITH_RENDER_CACHE_TTL_NS)
		return z_policy->render_active;

	rcu_read_lock();
	{
		struct zenith_comm_table *t =
			rcu_dereference(zenith_render_table);

		for_each_cpu(cpu, policy->cpus) {
			struct task_struct *curr = rcu_dereference(cpu_curr(cpu));
			unsigned int i;

			if (!curr || !t)
				continue;
			for (i = 0; i < t->nr; i++) {
				const char *needle = t->entries[i];

				if (!strncmp(curr->comm, needle,
					     strlen(needle))) {
					match = true;
					/* Wave A render-thread util
					 * tracker: capture the matched
					 * task's PELT util_avg in 1/1024
					 * units.  Single READ_ONCE() so
					 * the matched_util field is
					 * always fresh when render_-
					 * active is true.
					 */
					matched_util = (unsigned int)
						READ_ONCE(curr->se.avg.util_avg);
					break;
				}
			}
			if (match)
				break;
		}
	}
	rcu_read_unlock();

	z_policy->render_active = match;
	z_policy->render_matched_util_avg = matched_util;
	z_policy->render_cache_stamp_ns = now;
	return match;
}

/* Wave A cgroup-aware top-app helper.  Reads the cpuset cgroup the
 * task currently belongs to and checks whether the leaf directory
 * name is "top-app".  Caller must NOT hold rcu_read_lock; this
 * helper takes it internally (RCU is recursive, so calling from a
 * context that already holds the lock is also fine).
 *
 * Gated on CONFIG_CPUSETS because cpuset_cgrp_id is only defined
 * when the cpuset subsystem is built; on !CONFIG_CPUSETS the
 * helper compiles to a constant false and the floor never
 * applies.
 */
#if IS_ENABLED(CONFIG_CPUSETS)
static bool zenith_task_in_top_app(struct task_struct *t)
{
	struct cgroup_subsys_state *css;
	bool match = false;

	rcu_read_lock();
	css = task_css(t, cpuset_cgrp_id);
	if (css && css->cgroup && css->cgroup->kn) {
		const char *name = css->cgroup->kn->name;

		if (name && !strcmp(name, ZENITH_TOP_APP_CGROUP_NAME))
			match = true;
	}
	rcu_read_unlock();
	return match;
}
#else
static inline bool zenith_task_in_top_app(struct task_struct *t)
{
	return false;
}
#endif

/* Walk the policy's online cpumask and check each cpu_curr's cpuset
 * cgroup membership.  Returns true on the first match against the
 * "top-app" cgroup.  Result is cached for ZENITH_TOP_APP_CACHE_TTL_NS
 * (4 ms) so a hot path (e.g. a 60 / 90 / 120 Hz scroll) only does
 * the cgroup walk a few times per second per policy.  Caller is
 * expected to gate the call on tunables->top_app_aware != 0; this
 * helper does not re-check that.
 */
static bool zenith_policy_has_top_app(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;

	if (z_policy->top_app_cache_stamp_ns &&
	    now - z_policy->top_app_cache_stamp_ns < ZENITH_TOP_APP_CACHE_TTL_NS)
		return z_policy->top_app_active;

	rcu_read_lock();
	for_each_cpu(cpu, policy->cpus) {
		struct task_struct *curr = rcu_dereference(cpu_curr(cpu));

		if (!curr)
			continue;
		if (zenith_task_in_top_app(curr)) {
			match = true;
			break;
		}
	}
	rcu_read_unlock();

	z_policy->top_app_active = match;
	z_policy->top_app_cache_stamp_ns = now;
	return match;
}

/* Audio low-jitter comm match.  Same shape as zenith_render_comms[]:
 * a NUL-terminated table of comm prefixes; strncmp() walks each
 * cpu_curr->comm against each entry up to the table prefix length.
 *
 * Entries are picked from the standard Android audio thread names:
 *   - AudioOut_*       per-AudioFlinger fast/normal mixer threads
 *   - AudioMixer       AudioFlinger mixer threads (older naming)
 *   - audioserver      AudioFlinger main thread
 *   - audio_server     vendor variant of the same
 *   - MediaCodec_*     framework media codec callback threads
 *   - OMX*             OpenMAX vendor codec threads
 *   - SoundPool        framework SoundPool worker
 *   - PlaybackThread   AudioFlinger playback thread
 *   - RecordThread     AudioFlinger record thread
 *   - vendor.qti.audi  Qualcomm vendor audio HAL (truncated)
 *   - vendor.google.a  Tensor / Pixel vendor audio HAL (truncated)
 *   - vendor.oplus.au  OPlus / OnePlus / Realme audio HAL family
 *   - audio.hw.servic  Samsung audio.hw service (truncated)
 *
 * B-AUTO-1 expansion (additions, all <= 15 chars to fit comm[16]):
 *   - fast_mixer       AudioFlinger FastMixer thread (low-latency path)
 *   - TrackBase        AudioFlinger TrackBase worker family
 *   - AudioTrack       libaudioclient JNI AudioTrack thread
 *   - AudioRecord      libaudioclient JNI AudioRecord thread
 *   - audioPolicySrv   AudioPolicyService main thread
 *   - audio.cb.thread  vendor audio callback thread (qcom / mtk family)
 *   - audio_track_thr  vendor track-driver thread (truncated)
 *   - vendor.mtk.audi  MediaTek vendor audio HAL (truncated)
 *
 * Order is tuned for cache-friendliness on phone workloads (the most
 * common per-frame matches first).  Default-list extensions are
 * runtime-augmentable via the audio_comms RW sysfs (CSV format).
 */
static const char * const zenith_audio_comms[] = {
	"AudioOut_",
	"AudioMixer",
	"audioserver",
	"audio_server",
	"MediaCodec_",
	"OMX",
	"SoundPool",
	"PlaybackThread",
	"RecordThread",
	"vendor.qti.audi",
	"vendor.google.a",
	"vendor.oplus.au",
	"audio.hw.servic",
	"fast_mixer",
	"TrackBase",
	"AudioTrack",
	"AudioRecord",
	"audioPolicySrv",
	"audio.cb.thread",
	"audio_track_thr",
	"vendor.mtk.audi",
};

/* Walk the policy's online cpumask and check each cpu_curr's comm
 * against zenith_audio_comms[].  Returns true on the first match.
 * Cached for ZENITH_AUDIO_CACHE_TTL_NS so the strncmp loop runs
 * once every few milliseconds at most.  Caller is expected to gate
 * the call on tunables->audio_aware != 0; this helper does not
 * re-check that.
 */
static bool zenith_policy_has_audio(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	struct zenith_tunables *t_hyst = z_policy->tunables;
	unsigned int hyst_ms = t_hyst ? t_hyst->audio_hyst_ms : 0;
	unsigned int cpu;
	bool match = false;

	/* Audit fix K5: deterministic short-circuit on any open ALSA
	 * pcm fd.  Same pattern as K4: bypass the comm-walk and the
	 * cache TTL because the refcount is event-driven and always
	 * fresh.
	 */
	if (atomic_read(&zenith_alsa_active_fds) > 0) {
		z_policy->audio_active = true;
		z_policy->audio_cache_stamp_ns = now;
		if (hyst_ms)
			WRITE_ONCE(z_policy->audio_sticky_until_ns,
				   now + (u64)hyst_ms * NSEC_PER_MSEC);
		return true;
	}

	/* Patch B7-1: sticky audio-active window.  After a fresh
	 * positive detection the helper continues to report true for
	 * audio_hyst_ms past the last hit.  Bypasses the cache TTL
	 * (the sticky window is the strictly-longer guard).
	 */
	if (hyst_ms) {
		u64 until = READ_ONCE(z_policy->audio_sticky_until_ns);

		if (until && now < until)
			return true;
	}

	if (z_policy->audio_cache_stamp_ns &&
	    now - z_policy->audio_cache_stamp_ns < ZENITH_AUDIO_CACHE_TTL_NS)
		return z_policy->audio_active;

	rcu_read_lock();
	{
		struct zenith_comm_table *t =
			rcu_dereference(zenith_audio_table);

		for_each_cpu(cpu, policy->cpus) {
			struct task_struct *curr = rcu_dereference(cpu_curr(cpu));
			unsigned int i;

			if (!curr || !t)
				continue;
			for (i = 0; i < t->nr; i++) {
				const char *needle = t->entries[i];

				if (!strncmp(curr->comm, needle,
					     strlen(needle))) {
					match = true;
					break;
				}
			}
			if (match)
				break;
		}
	}
	rcu_read_unlock();

	z_policy->audio_active = match;
	z_policy->audio_cache_stamp_ns = now;
	if (match && hyst_ms)
		WRITE_ONCE(z_policy->audio_sticky_until_ns,
			   now + (u64)hyst_ms * NSEC_PER_MSEC);
	return match;
}

/* Camera capture-pipeline comm match.  Picks names commonly used by
 * Android camera framework / HAL processes:
 *   - cameraserver       framework cameraserver process
 *   - camerahalserver    Pixel/Tensor vendor camera HAL daemon
 *   - cameraprovider     newer Treble cameraprovider
 *   - provider@          HIDL camera HAL service threads
 *                        ("provider@2.4-se", "provider@2.5-se", ...)
 *   - provider.MTK       MediaTek vendor variant
 *   - mtkcam-            MediaTek ISP/camera daemon threads
 *   - mtkcamutil         MediaTek camera utility threads
 *   - Camera2-           framework Camera2 internal threads
 *   - CamX_              Qualcomm CamX HAL threads
 *   - CamX-              CamX subsystem threads (alt naming)
 *   - vendor.qti.camera  Qualcomm vendor camera service
 *   - vendor.qti.hardwa  Qualcomm 8-gen+ truncated comm
 *                        (vendor.qti.hardware.camera.provider@*)
 *   - vendor.oplus.cam   OPlus / OnePlus / Realme camera HAL family
 *   - vendor.samsung.ca  Samsung Camera HAL (truncated to 16 chars)
 *
 * B-AUTO-1 expansion (additions, all <= 15 chars):
 *   - vendor.mtk.came  MediaTek camera HAL service (truncated)
 *   - vendor.google.c  Pixel / Tensor vendor.google.camera (truncated)
 *   - vendor.qti.imag  Qualcomm image processor service (truncated)
 *   - C2DColorConver   Qualcomm camera color-space conversion thread
 *
 * Order is tuned for cache-friendliness: most common matches first.
 * Default-list extensions are runtime-augmentable via the
 * camera_comms RW sysfs (CSV format).
 */
static const char * const zenith_camera_comms[] = {
	"cameraserver",
	"camerahalserver",
	"cameraprovider",
	"provider@",
	"provider.MTK",
	"mtkcam-",
	"mtkcamutil",
	"Camera2-",
	"CamX_",
	"CamX-",
	"vendor.qti.camera",
	"vendor.qti.hardwa",
	"vendor.oplus.cam",
	"vendor.samsung.ca",
	"vendor.mtk.came",
	"vendor.google.c",
	"vendor.qti.imag",
	"C2DColorConver",
};

/* Walk the policy's online cpumask and check each cpu_curr's comm
 * against zenith_camera_comms[].  Returns true on the first match.
 * Cached for ZENITH_CAMERA_CACHE_TTL_NS.  Caller is expected to
 * gate on tunables->camera_aware != 0; this helper does not
 * re-check that.  The returned value is the *raw* comm-match
 * decision; the caller applies the camera_active override on top.
 */
static bool zenith_policy_has_camera(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;

	/* Audit fix K4: deterministic short-circuit.  Any open v4l2 fd
	 * means camera (or webcam, or v4l2 codec node, or screen
	 * recorder sink) is active; treat as match without doing the
	 * runqueue walk at all.  Bypasses the comm-walk cache too --
	 * the v4l2 hook updates atomically on every open / release so
	 * the value is always fresh, no TTL needed.
	 */
	if (atomic_read(&zenith_v4l2_active_fds) > 0) {
		z_policy->camera_auto_match = true;
		z_policy->camera_cache_stamp_ns = now;
		return true;
	}

	if (z_policy->camera_cache_stamp_ns &&
	    now - z_policy->camera_cache_stamp_ns < ZENITH_CAMERA_CACHE_TTL_NS)
		return z_policy->camera_auto_match;

	rcu_read_lock();
	for_each_cpu(cpu, policy->cpus) {
		struct task_struct *curr = rcu_dereference(cpu_curr(cpu));
		int i;

		if (!curr)
			continue;
		{
			struct zenith_comm_table *t =
				rcu_dereference(zenith_camera_table);

			if (!t)
				continue;
			for (i = 0; i < t->nr; i++) {
				const char *needle = t->entries[i];

				if (!strncmp(curr->comm, needle,
					     strlen(needle))) {
					match = true;
					break;
				}
			}
		}
		if (match)
			break;
	}
	rcu_read_unlock();

	z_policy->camera_auto_match = match;
	z_policy->camera_cache_stamp_ns = now;
	return match;
}

/* Default seed list for the in-kernel game detector.  See the
 * ZENITH_DEFAULT_GAME_AUTO comment block.  Used at zenith_gov_init()
 * time and as the reset target when game_auto_comms is written empty;
 * live matching always reads the RCU table.
 *
 * Entries are tuned for typical Android game engines:
 *
 *   - UnityMain          Unity main thread (most common Unity name)
 *   - UnityGfxDeviceW    Unity gfx device worker
 *   - il2cpp             Unity IL2CPP scripting backend worker
 *   - GameThread         Unreal Engine main thread (also some custom
 *                        engines).  This is one Android system-wide
 *                        contender that stylistically conflicts with
 *                        Android's own RenderThread; we keep it on
 *                        the assumption that it is dominant on big
 *                        cores only when an actual Unreal title is
 *                        running.  Userspace can drop it via the
 *                        game_auto_comms knob if it conflicts.
 *
 * B-AUTO-1 expansion (additions, all <= 15 chars, vendor-comprehensive
 * so the auto-profile selector cannot miss a game-engine context):
 *
 *   - TaskGraphThr       Unreal Engine task-graph worker (truncated;
 *                        UE4/UE5 spawn TaskGraphThread N for parallel
 *                        engine tasks)
 *   - RHIThread          Unreal Engine Render Hardware Interface thread
 *                        (UE4/UE5; bridges renderer to GPU API backends)
 *   - Cocos2dxRender     Cocos2d-x render thread (truncated; engine name
 *                        used by many phone games shipped via cocos2d-x)
 *   - Job.Worker         Unity Burst job system worker thread
 *                        (DOTS / ECS / parallel-for jobs)
 *   - EnlightenWork      Unity Enlighten realtime-GI worker
 *
 * Patch M expansion (post-audit additions; all distinct from existing
 * Android system thread names so the prefix walk does not collide):
 *
 *   - RenderingThread    Unreal Engine alternate render thread name
 *                        (15 chars, exactly fits TASK_COMM_LEN-1).
 *                        Distinct from Android HWUI's "RenderThread"
 *                        (no "ing") -- the prefix walk uses strncmp
 *                        with strlen(needle) so the system thread
 *                        does not match this longer prefix.
 *   - Roblox             Roblox engine prefix.  Roblox spawns
 *                        threads named "RobloxAppMain", "RobloxRender",
 *                        "RobloxNetwork", etc., all of which prefix
 *                        with "Roblox".  One of the most popular
 *                        mobile titles globally; not covered by any
 *                        Unity / Unreal / Cocos2d engine matcher.
 *   - miHoYoSDK          miHoYo / HoYoverse common SDK thread.  Live
 *                        for Genshin Impact, Honkai: Star Rail,
 *                        Zenless Zone Zero, and Honkai Impact 3rd.
 *                        Belt-and-suspenders coverage on top of the
 *                        UnityMain match -- the SDK thread persists
 *                        through scenes where UnityMain is briefly
 *                        scheduled out (login flows, IAP, scene
 *                        transitions).
 */
static const char * const zenith_game_auto_comms[] = {
	"UnityMain",
	"UnityGfxDeviceW",
	"il2cpp",
	"GameThread",
	"TaskGraphThr",
	"RHIThread",
	"Cocos2dxRender",
	"Job.Worker",
	"EnlightenWork",
	"RenderingThread",
	"Roblox",
	"miHoYoSDK",
};

/* Hot-path comm walk used by the in-kernel game detector.  TTL'd
 * for ZENITH_GAME_AUTO_CACHE_TTL_NS so a 60/120/144 Hz cpufreq
 * decision rate only does the strncmp loop a few times per second.
 * Caller gates on the static_branch_unlikely(zenith_game_auto_key)
 * branch and the live tunables->game_auto scalar; this helper does
 * not re-check either.  Returns the raw comm-match boolean for the
 * caller (zenith_policy_game_auto_tick) to feed into the streak
 * counter.
 */
static bool zenith_policy_has_game_auto(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;

	if (z_policy->game_auto_cache_stamp_ns &&
	    now - z_policy->game_auto_cache_stamp_ns <
	    ZENITH_GAME_AUTO_CACHE_TTL_NS)
		return z_policy->game_auto_match;

	rcu_read_lock();
	{
		struct zenith_comm_table *t =
			rcu_dereference(zenith_game_auto_table);

		for_each_cpu(cpu, policy->cpus) {
			struct task_struct *curr = rcu_dereference(cpu_curr(cpu));
			unsigned int i;

			if (!curr || !t)
				continue;
			for (i = 0; i < t->nr; i++) {
				const char *needle = t->entries[i];

				if (!strncmp(curr->comm, needle,
					     strlen(needle))) {
					match = true;
					break;
				}
			}
			if (match)
				break;
		}
	}
	rcu_read_unlock();

	z_policy->game_auto_match = match;
	z_policy->game_auto_cache_stamp_ns = now;
	return match;
}

/* Per-policy hot-path tick for the in-kernel game detector.  Called
 * once per zenith_get_next_freq() invocation, gated by the
 * zenith_game_auto_key static branch and the live tunables->game_auto
 * scalar.  Maintains the per-policy streak counter and renews the
 * global zenith_game_auto_active_until_ns latch when the streak
 * crosses ZENITH_GAME_AUTO_DETECT_STREAK.  Resets streak on miss so
 * we only latch on a sustained match.
 *
 * Streak reset on detection (rather than continued increment) keeps
 * the atomic write rate bounded -- one write per
 * (DETECT_STREAK * cache_TTL) at the most pessimistic, ~128 ms
 * worst case at the default knob values.
 */
static void zenith_auto_predict_tick(struct zenith_policy *z_policy);
static void zenith_policy_game_auto_tick(struct zenith_policy *z_policy)
{
	bool match = zenith_policy_has_game_auto(z_policy);
	u64 until;

	if (match)
		z_policy->game_auto_streak++;
	else
		z_policy->game_auto_streak = 0;

	if (z_policy->game_auto_streak >= ZENITH_GAME_AUTO_DETECT_STREAK) {
		u64 prev = READ_ONCE(zenith_game_auto_active_until_ns);
		until = ktime_get_ns() + ZENITH_GAME_AUTO_ACTIVE_TTL_NS;
		WRITE_ONCE(zenith_game_auto_active_until_ns, until);
		if (!prev)
			atomic_notifier_call_chain(&zenith_game_mode_nh, 1, NULL);
		z_policy->game_auto_streak = 0;
	}
}

/*
 * Kasumi (drivers/thermal/thermal_helpers.c) intercepts
 * thermal_zone_get_temp() and dampens the reported temperature.
 * For Zenith's game_perf_burst guardrail we want the *real* (un-
 * dampened) value so the FSM can't be fooled by a configured
 * Kasumi offset; the dampened value still goes to the framework /
 * userspace as before.  Forward-declared here (no public header
 * for Kasumi) -- defined and EXPORT_SYMBOL_GPL'd by Kasumi.
 *
 * Returns the last raw value seen by kasumi_dampen(), in millideg C.
 * Returns 0 if Kasumi has never dampened a reading (e.g. Kasumi
 * disabled, or no thermal zone the filter accepted has been read
 * yet) -- caller treats 0 as "fall back to whatever I have".
 */
#if IS_ENABLED(CONFIG_KASUMI)
extern int kasumi_get_last_real_mc(void);
extern void kasumi_apply_profile(unsigned int profile);
#else
static inline int kasumi_get_last_real_mc(void) { return 0; }
static inline void kasumi_apply_profile(unsigned int profile) { }
#endif

#if IS_ENABLED(CONFIG_IYASHI)
extern void iyashi_apply_profile(unsigned int profile);
#else
static inline void iyashi_apply_profile(unsigned int profile) { }
#endif

#if IS_ENABLED(CONFIG_VINDICATOR_EQUILIBRIUM)
extern void equilibrium_apply_profile(unsigned int profile);
#else
static inline void equilibrium_apply_profile(unsigned int profile) { }
#endif
#if IS_ENABLED(CONFIG_VINDICATOR_NOCTURNE)
extern void nocturne_apply_profile(unsigned int profile);
#else
static inline void nocturne_apply_profile(unsigned int profile) { }
#endif

/* Patch K: live skin-temp readout for the game_perf_burst guardrail.
 * Returns millidegrees C.
 *
 * Primary path: thermal_zone_get_temp() against the per-policy zone
 * resolved at zenith_start() time.  This is the kernel thermal
 * subsystem's authoritative reading -- same number userspace would
 * see at /sys/class/thermal/thermal_zone<N>/temp.
 *
 * Patch K v2 (Zenith x Kasumi awareness): after a successful
 * thermal_zone_get_temp() we ask Kasumi for the last *real* (un-
 * dampened) reading and prefer it over the framework value.  This
 * lets Zenith make burst-guardrail decisions on the truth even
 * when Kasumi is veiling heat from the framework.  Tiny race
 * window: between our get_temp() returning and our
 * kasumi_get_last_real_mc() call, another CPU could re-dampen a
 * different zone and overwrite kasumi_last_real_mc.  The window is
 * sub-millisecond and the guardrail is approximate (FSM thresholds
 * are degrees apart), so this is acceptable.
 *
 * Fallback: if the zone is unresolved (NULL / IS_ERR -- foreign SoC,
 * thermal subsystem not registered yet, etc.) or the read returns
 * an error, synthesize an approximate dC value from
 * arch_scale_thermal_pressure().  The pressure is a 0..1024 capacity-
 * reduction scalar; mapping 0..100% pressure linearly to 30..70 dC
 * gives the burst guardrail a reasonable best-effort estimate even
 * on platforms where the thermal subsystem hasn't published a
 * per-cluster zone.
 *
 * Both paths are cheap (single load + one library call) so the
 * helper is safe to call once per zenith_get_next_freq().  No
 * caching layer here; the thermal subsystem already caches its own
 * sensor reads (driver dependent), and the fallback path is a
 * single arch_scale_thermal_pressure() read.
 */
static int zenith_gpb_get_temp_dc(struct zenith_policy *z_policy)
{
	struct thermal_zone_device *tzd = z_policy->gpb_tzd;
	unsigned int pct;
	int temp = 0;
	int real;

	/* Patch M: lazy retry if zenith_start() couldn't bind the
	 * per-cluster thermal zone (boot ordering: thermal-core may
	 * register after the governor on some Tensor builds).  Rate
	 * limited to one re-resolution per
	 * ZENITH_GPB_TZD_RETRY_INTERVAL_NS so a permanently-foreign SoC
	 * (no per-cpu thermal zones at all) costs only one
	 * thermal_zone_get_zone_by_name() per N seconds in the hot path.
	 */
	if (!tzd) {
		u64 now = ktime_get_ns();

		if (now >= z_policy->gpb_tzd_retry_at_ns) {
			char zone_name[16];
			unsigned int first_cpu =
				cpumask_first(z_policy->policy->cpus);

			scnprintf(zone_name, sizeof(zone_name),
				  "cpu%u-thermal", first_cpu);
			tzd = thermal_zone_get_zone_by_name(zone_name);
			if (IS_ERR(tzd))
				tzd = NULL;
			z_policy->gpb_tzd = tzd;
			z_policy->gpb_tzd_retry_at_ns =
				now + ZENITH_GPB_TZD_RETRY_INTERVAL_NS;
		}
	}

	if (tzd && !IS_ERR(tzd) && !thermal_zone_get_temp(tzd, &temp)) {
		real = kasumi_get_last_real_mc();
		return real > 0 ? real : temp;
	}

	pct = zenith_policy_thermal_pressure_pct(z_policy);
	if (pct > 100)
		pct = 100;
	return 30000 + (int)(pct * 400);
}

/* Patch K: game_perf_burst FSM evaluator.  Called once per
 * zenith_get_next_freq() invocation, gated by the
 * zenith_game_perf_burst_key static branch and the live tunables
 * scalar (defends against momentary tear during sysfs store).
 *
 * Reads:
 *   - zenith_eff_game_mode()                 Signal A
 *   - z_policy->last_load_pct (previous tick) Signal B
 *   - z_policy->audio_sticky_until_ns         Signal C suppressor
 *
 * Writes:
 *   - z_policy->gpb_state                    FSM state
 *   - z_policy->gpb_state_entry_ns           transition timestamp
 *   - z_policy->gpb_b_arm_first_seen_ns      Signal-B continuous-on
 *   - z_policy->gpb_b_disarm_first_seen_ns   Signal-B continuous-off
 *
 * No locking required: per-policy hot path is serialised by the
 * cpufreq core (single writer for this set of fields).
 *
 * Note: zenith_eff_game_mode() peeks at zenith_game_auto_active_until_ns
 * which is updated by zenith_policy_game_auto_tick() above; the
 * caller invokes the auto_tick first so a fresh tick's match has
 * already been latched when the FSM evaluator runs.
 */
static void zenith_gpb_evaluate(struct zenith_policy *z_policy)
{
	struct zenith_tunables *t = z_policy->tunables;
	u64 now_ns = ktime_get_ns();
	bool sig_a, sig_b, sig_c_suppress;
	unsigned int load_pct;
	unsigned int disarm_grace_ms;
	unsigned int base_gm;

	/* Signal A: zenith_eff_game_mode().  Pass the manual game_mode
	 * scalar as the base; the helper bumps it to 1 when game_auto
	 * has latched a sustained known-game comm-walk match.  Read via
	 * READ_ONCE so a momentary tear during a sysfs game_mode store
	 * does not produce a transient false negative on this tick.
	 */
	base_gm = READ_ONCE(t->game_mode);
	sig_a = (zenith_eff_game_mode(base_gm) != 0);

	load_pct = READ_ONCE(z_policy->last_load_pct);

	/* Signal C suppressor: an active audio sticky window with NO
	 * game signal arriving first means "the user is watching
	 * something, not playing something".  When sig_a is already
	 * true (game_auto latched or manual game_mode write), the
	 * suppressor is moot -- a user can play a game while music
	 * plays, and we do not want to deny them the burst.
	 */
	sig_c_suppress = !sig_a &&
		(READ_ONCE(z_policy->audio_sticky_until_ns) > now_ns);

	/* Patch M: Schmitt-trigger Signal B.  The non-zero
	 * gpb_b_arm_first_seen_ns stamp doubles as the latch state.
	 *
	 *   off (stamp == 0):  arm only when load >= 70 (enter)
	 *   on  (stamp != 0):  release only when load < 60 (exit)
	 *
	 * This holds sig_b across dips inside the 60..70 hysteresis
	 * band so a scene oscillating in that range keeps accumulating
	 * sustained-arming time instead of resetting on every dip.
	 * The 2s sustained gate enforced at the IDLE -> ARMED edge
	 * below uses the same first-seen stamp, so once we cross 70
	 * and stay above 60, the timer keeps ticking.
	 */
	if (z_policy->gpb_b_arm_first_seen_ns) {
		if (load_pct < ZENITH_GAME_PERF_BURST_B_EXIT_THRESHOLD_PCT) {
			z_policy->gpb_b_arm_first_seen_ns = 0;
			sig_b = false;
		} else {
			sig_b = true;
		}
	} else {
		if (load_pct >= ZENITH_GAME_PERF_BURST_B_THRESHOLD_PCT) {
			z_policy->gpb_b_arm_first_seen_ns = now_ns;
			sig_b = true;
		} else {
			sig_b = false;
		}
	}

	/* Signal-B continuous-off tracking (only meaningful while
	 * ARMED).  Stamp first-seen on the ARMED-side falling edge,
	 * clear on any tick that re-observes B.
	 */
	if (z_policy->gpb_state == ZENITH_GPB_STATE_ARMED) {
		if (!sig_b) {
			if (z_policy->gpb_b_disarm_first_seen_ns == 0)
				z_policy->gpb_b_disarm_first_seen_ns = now_ns;
		} else {
			z_policy->gpb_b_disarm_first_seen_ns = 0;
		}
	} else {
		z_policy->gpb_b_disarm_first_seen_ns = 0;
	}

	disarm_grace_ms = READ_ONCE(t->game_perf_burst_disarm_grace_ms);

	switch (z_policy->gpb_state) {
	case ZENITH_GPB_STATE_IDLE:
		/* IDLE -> ARMED: A AND (B sustained for >= 2s) AND
		 * (NOT C suppressing).
		 */
		if (sig_a && !sig_c_suppress &&
		    z_policy->gpb_b_arm_first_seen_ns &&
		    (now_ns - z_policy->gpb_b_arm_first_seen_ns) >=
		    ZENITH_GAME_PERF_BURST_B_REQUIRED_NS) {
			z_policy->gpb_state = ZENITH_GPB_STATE_ARMED;
			z_policy->gpb_state_entry_ns = now_ns;
			z_policy->gpb_b_disarm_first_seen_ns = 0;
			z_policy->gpb_arm_count++;
		}
		break;
	case ZENITH_GPB_STATE_ARMED:
		/* Fast disarm: !A => COOLDOWN immediately.  This catches
		 * the user Alt+Tabbing / pressing Home -- game_auto's
		 * ACTIVE_TTL has expired or game_mode was written 0.
		 */
		if (!sig_a) {
			z_policy->gpb_state = ZENITH_GPB_STATE_COOLDOWN;
			z_policy->gpb_state_entry_ns = now_ns;
			z_policy->gpb_disarm_count++;
			z_policy->gpb_last_disarm_reason =
				ZENITH_GPB_DISARM_FAST;
			break;
		}
		/* Sustained-clear disarm: B has been false continuously
		 * for >= disarm_grace_ms.  Tail of a level / loading
		 * screen.  Glide back through COOLDOWN.
		 */
		if (z_policy->gpb_b_disarm_first_seen_ns &&
		    (now_ns - z_policy->gpb_b_disarm_first_seen_ns) >=
		    ((u64)disarm_grace_ms * NSEC_PER_MSEC)) {
			z_policy->gpb_state = ZENITH_GPB_STATE_COOLDOWN;
			z_policy->gpb_state_entry_ns = now_ns;
			z_policy->gpb_disarm_count++;
			z_policy->gpb_last_disarm_reason =
				ZENITH_GPB_DISARM_SUSTAINED;
		}
		break;
	case ZENITH_GPB_STATE_COOLDOWN:
		/* COOLDOWN -> ARMED on a fresh re-arm: user came back
		 * within the cooldown window.  Skip the 2s sustained
		 * gate this once (we were just here) so the floor
		 * re-engages without a second multi-second confidence
		 * build-up.
		 */
		if (sig_a && sig_b && !sig_c_suppress) {
			z_policy->gpb_state = ZENITH_GPB_STATE_ARMED;
			z_policy->gpb_state_entry_ns = now_ns;
			z_policy->gpb_b_disarm_first_seen_ns = 0;
			z_policy->gpb_arm_count++;
			break;
		}
		/* COOLDOWN -> IDLE when the glide window expires. */
		{
			unsigned int cooldown_ms =
				READ_ONCE(t->game_perf_burst_cooldown_ms);
			u64 elapsed = now_ns - z_policy->gpb_state_entry_ns;

			if (elapsed >= ((u64)cooldown_ms * NSEC_PER_MSEC)) {
				z_policy->gpb_state = ZENITH_GPB_STATE_IDLE;
				z_policy->gpb_state_entry_ns = now_ns;
				z_policy->gpb_idle_count++;
			}
		}
		break;
	default:
		/* Defensive: any unknown state -> IDLE. */
		z_policy->gpb_state = ZENITH_GPB_STATE_IDLE;
		z_policy->gpb_state_entry_ns = now_ns;
		break;
	}
}

/* Patch K: compute the per-tick freq floor contribution from the
 * game_perf_burst FSM.  Returns 0 when the FSM is IDLE, the master
 * is off, or the thermal guardrail is engaged this tick.  Otherwise
 * returns the floor freq in policy units (Hz).
 *
 * ARMED state: returns policy->max * floor_pct / 100.
 *
 * COOLDOWN state: linearly steps the floor from the ARMED level
 * down to 0 across cooldown_ms.  At t=0  ms post-disarm the floor
 * still equals the ARMED floor; at t=cooldown_ms it has reached 0.
 * This avoids freq whiplash when the user Alt+Tabs out.
 *
 * Thermal guardrail: when the live skin temp >= ceiling_dc, return
 * 0 -- the existing auto_thermal_cap / thermal_util_derate path
 * remains in charge.  The FSM stays in ARMED so the floor will
 * re-engage automatically once the temp drops back below ceiling.
 */
static unsigned int zenith_gpb_floor(struct zenith_policy *z_policy,
				     unsigned int policy_max)
{
	struct zenith_tunables *t = z_policy->tunables;
	unsigned int floor_pct;
	unsigned int floor;
	int temp_dc;
	int ceiling_dc;
	u8 state;

	state = z_policy->gpb_state;
	if (state == ZENITH_GPB_STATE_IDLE)
		return 0;

	floor_pct = READ_ONCE(t->game_perf_burst_floor_pct);
	if (!floor_pct || !policy_max)
		return 0;

	temp_dc = zenith_gpb_get_temp_dc(z_policy);
	ceiling_dc = (int)READ_ONCE(t->game_perf_burst_thermal_ceiling_dc);
	if (temp_dc >= ceiling_dc)
		return 0;

	floor = (policy_max / 100) * floor_pct;
	if (floor > policy_max)
		floor = policy_max;

	if (state == ZENITH_GPB_STATE_COOLDOWN) {
		unsigned int cooldown_ms =
			READ_ONCE(t->game_perf_burst_cooldown_ms);
		u64 cooldown_ns = (u64)cooldown_ms * NSEC_PER_MSEC;
		u64 now_ns = ktime_get_ns();
		u64 elapsed = now_ns - z_policy->gpb_state_entry_ns;

		if (!cooldown_ns || elapsed >= cooldown_ns)
			return 0;
		/* Linear ramp: floor *= (cooldown_ns - elapsed) / cooldown_ns
		 * Computed in u64 to avoid wrap on policy_max * remaining.
		 */
		{
			u64 remaining = cooldown_ns - elapsed;
			u64 scaled = ((u64)floor * remaining) / cooldown_ns;

			floor = (unsigned int)scaled;
		}
	}
	return floor;
}

/* Patch K: stringify the FSM state for sysfs read-back.  Stable
 * tokens so userspace tooling can grep / parse the state node.
 */
static const char *zenith_gpb_state_name(u8 state)
{
	switch (state) {
	case ZENITH_GPB_STATE_IDLE:
		return "idle";
	case ZENITH_GPB_STATE_ARMED:
		return "ARMED";
	case ZENITH_GPB_STATE_COOLDOWN:
		return "COOLDOWN";
	default:
		return "?";
	}
}

/* Patch M: stringify the last-disarm reason for the stats sysfs.
 * Stable tokens so userspace tooling can grep / parse the field.
 */
static const char *zenith_gpb_disarm_name(u8 reason)
{
	switch (reason) {
	case ZENITH_GPB_DISARM_NONE:
		return "none";
	case ZENITH_GPB_DISARM_FAST:
		return "fast";
	case ZENITH_GPB_DISARM_SUSTAINED:
		return "sustained";
	default:
		return "?";
	}
}

/* Predicate used by the cached_raw_freq shortcut in
 * zenith_get_next_freq().  Returns true when the efficient_freq
 * ladder has any armed bin deadline; in that case the cache hit
 * cannot be used because the ladder loop must run to drain
 * deadlines on schedule.  Walking ZENITH_EFF_BINS_MAX (small) once
 * per tick is cheap and avoids the latch hazard.
 */
static bool zenith_ladder_pending(struct zenith_policy *z_policy)
{
	unsigned int nr = z_policy->tunables->eff_nr;
	int i;

	if (!nr)
		return false;
	if (nr > ZENITH_EFF_BINS_MAX)
		nr = ZENITH_EFF_BINS_MAX;
	for (i = 0; i < nr; i++)
		if (z_policy->eff_unlock_at_ns[i])
			return true;
	return false;
}

/* Map a tp_path string to a stats bucket.  Called once per
 * zenith_get_next_freq() evaluation right before return, so the
 * cost is one O(few-strcmp) chain per decision -- negligible vs.
 * the rest of the eval pass.  Order is by frequency-of-hit so the
 * common case (eas / hispeed) short-circuits early.  Catches
 * everything; unknown tags fall through to ZENITH_STAT_OTHER.
 */
static enum zenith_stat_idx zenith_path_to_bucket(const char *path)
{
	if (unlikely(!path))
		return ZENITH_STAT_EAS;
	if (!strcmp(path, "eas"))
		return ZENITH_STAT_EAS;
	if (!strcmp(path, "hispeed"))
		return ZENITH_STAT_HISPEED;
	if (!strncmp(path, "input_boost", 11))	/* input_boost / _decay */
		return ZENITH_STAT_INPUT_BOOST;
	if (!strcmp(path, "snap_max") ||
	    !strcmp(path, "brutal_hold") ||
	    !strcmp(path, "climb_step"))
		return ZENITH_STAT_BRUTAL;
	if (!strcmp(path, "frame_pace"))
		return ZENITH_STAT_FRAME_PACE;
	if (!strcmp(path, "audio_floor") || !strcmp(path, "audio_cap"))
		return ZENITH_STAT_AUDIO;
	if (!strcmp(path, "render_floor") || !strcmp(path, "camera_floor"))
		return ZENITH_STAT_RENDER_CAMERA;
	if (!strcmp(path, "uclamp_min_floor") ||
	    !strcmp(path, "uclamp_max_cap"))
		return ZENITH_STAT_UCLAMP;
	if (!strncmp(path, "psi_", 4))		/* psi_mem_cap / _cpu / _io */
		return ZENITH_STAT_PSI;
	if (!strcmp(path, "boot_boost"))
		return ZENITH_STAT_BOOT_BOOST;
	if (!strcmp(path, "light_cap"))
		return ZENITH_STAT_LIGHT_CAP;
	if (!strcmp(path, "em_cap"))
		return ZENITH_STAT_EM_CAP;
	if (!strcmp(path, "predict_up"))
		return ZENITH_STAT_PREDICT_UP;
	if (!strcmp(path, "peak_prearm"))
		return ZENITH_STAT_PEAK_PREARM;
	if (!strcmp(path, "peak_rescue"))
		return ZENITH_STAT_PEAK_RESCUE;
	if (!strcmp(path, "peak_hyst"))
		return ZENITH_STAT_PEAK_HYST;
	if (!strcmp(path, "peer_ramp"))
		return ZENITH_STAT_PEER_RAMP;
	if (!strcmp(path, "migration_floor"))
		return ZENITH_STAT_MIGRATION_FLOOR;
	if (!strcmp(path, "psi_cpu_floor"))
		return ZENITH_STAT_PSI_CPU_FLOOR;
	if (!strcmp(path, "frame_overrun"))
		return ZENITH_STAT_FRAME_OVERRUN;
	if (!strcmp(path, "auto_thermal_cap"))
		return ZENITH_STAT_AUTO_THERMAL_CAP;
	if (!strcmp(path, "quiet_hours_cap"))
		return ZENITH_STAT_QUIET_HOURS_CAP;
	return ZENITH_STAT_OTHER;
}

/* Cached "does this SoC have a dedicated BIG / mid cluster?" probe.
 *
 * On a 3+-cluster topology (1+3+4 et al) at least one cluster sits
 * strictly between the LITTLE and PRIME capacity bands, so its policy
 * is classified ZENITH_CLUSTER_BIG by zenith_update_cluster_rate_scale().
 * On a 2-cluster (true big.LITTLE) topology the lone non-LITTLE
 * cluster is at max_cap and gets classified ZENITH_CLUSTER_PRIME -- the
 * BIG class is unused.
 *
 * The prefer_silver_aware bump path (dead -- Kconfig removed) would
 * need to distinguish these two cases so it can fire on the lowest
 * non-LITTLE cluster in either topology (BIG on tri-cluster, PRIME on
 * 2-cluster) without wrongly inflating up_threshold on PRIME when a
 * separate BIG cluster also exists.
 *
 * Topology is invariant after boot, so the result is computed once on
 * the first call and cached.  capacity_orig is read via
 * arch_scale_cpu_capacity() which matches what
 * zenith_update_cluster_rate_scale() uses, keeping classification and
 * topology probe consistent on every SoC.
 */
static bool zenith_topology_has_big_class(void)
{
	/*
	 * State: 0 = unknown, 1 = false, 2 = true.  Encoding both the
	 * cached value and its validity in a single atomic_t means a
	 * concurrent caller cannot observe "valid" without also seeing
	 * the corresponding result -- side-stepping the
	 * smp_wmb / smp_rmb pairing that a separate (cached_result,
	 * cached_valid) pair would require on weakly-ordered ARM64.
	 * Topology is invariant after boot, so multiple racing first
	 * callers all compute the same result and converge.
	 */
	static atomic_t cached = ATOMIC_INIT(0);
	int snap = atomic_read(&cached);
	unsigned int little_thresh, big_cap = 0, second_cap = 0, cpu;
	bool has_big;

	if (snap)
		return snap == 2;

	little_thresh =
		(SCHED_CAPACITY_SCALE * ZENITH_CLUSTER_LITTLE_THRESH_PCT) / 100;

	for_each_possible_cpu(cpu) {
		unsigned int cap = arch_scale_cpu_capacity(cpu);

		if (cap > big_cap) {
			second_cap = big_cap;
			big_cap = cap;
		} else if (cap < big_cap && cap > second_cap) {
			second_cap = cap;
		}
	}

	/* A dedicated BIG class exists when there is a non-LITTLE
	 * capacity level strictly below the system maximum.  On
	 * 2-cluster phones second_cap is either zero (single non-LITTLE
	 * cluster) or below little_thresh (every non-max CPU was a
	 * LITTLE-equivalent).  On 3+-cluster phones second_cap sits
	 * comfortably above little_thresh and below big_cap.
	 */
	has_big = (second_cap >= little_thresh && second_cap < big_cap);
	atomic_set(&cached, has_big ? 2 : 1);
	return has_big;
}

/* Peak-return hysteresis (Patch E).  Pulled out of
 * zenith_get_next_freq() so the deeply-nested gating logic can be
 * expressed without overflowing checkpatch's max-tab limit.
 *
 * Pre: caller has computed freq through the lower freq-tier
 * passes.  Post: returns the freq with the soft floor applied
 * (or the input freq unchanged if the tier is disabled or the
 * cluster isn't in a peak-exit transition).  *tp_path is set to
 * "peak_hyst" iff the soft floor fired.
 */
static unsigned int
zenith_apply_peak_hysteresis(struct zenith_policy *z_policy,
			     struct cpufreq_policy *policy,
			     unsigned int freq, bool pin_to_target,
			     const char **tp_path)
{
	unsigned int hyst_streak =
		READ_ONCE(z_policy->tunables->peak_hysteresis_streak);
	unsigned int step_down_pct =
		READ_ONCE(z_policy->tunables->peak_step_down_pct);
	unsigned int prev, peak_thresh, anchor, floor_freq;

	if (!hyst_streak || !step_down_pct ||
	    !policy->max || pin_to_target) {
		z_policy->peak_hyst_anchor_freq = 0;
		z_policy->peak_low_streak = 0;
		return freq;
	}

	prev = z_policy->cached_raw_freq;
	peak_thresh = (policy->max / 100) *
		ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT;

	/* (Re)anchor on every sample where the previous
	 * cached_raw_freq sits in the peak class.  This lets the
	 * anchor track the cluster while it is pinned at peak, then
	 * survive the descent because the cap-to-floor write below
	 * pushes prev out of the peak class on subsequent ticks.
	 */
	if (prev >= peak_thresh) {
		z_policy->peak_hyst_anchor_freq = prev;
		z_policy->peak_low_streak = 0;
	}

	anchor = z_policy->peak_hyst_anchor_freq;
	if (!anchor)
		return freq;

	floor_freq = (anchor / 100) * step_down_pct;
	if (floor_freq > policy->max)
		floor_freq = policy->max;

	if (freq >= floor_freq) {
		/* Natural freq is at or above the soft floor; release
		 * the anchor.
		 */
		z_policy->peak_hyst_anchor_freq = 0;
		z_policy->peak_low_streak = 0;
		return freq;
	}

	if (z_policy->peak_low_streak >= hyst_streak) {
		/* Streak drained; release and let the natural descent
		 * resume.
		 */
		z_policy->peak_hyst_anchor_freq = 0;
		z_policy->peak_low_streak = 0;
		return freq;
	}

	if (z_policy->peak_low_streak < ZENITH_PEAK_HYSTERESIS_STREAK_MAX)
		z_policy->peak_low_streak++;
	*tp_path = "peak_hyst";
	return floor_freq;
}

/* Peer-ramp helpers (Patch D).  Translate a cluster_class to the
 * deadline atomic each side of the protocol cares about.
 *
 * peer_atomic_for() picks the slot that the *peer* of the given
 * cluster will read on its next eval.  An arming write goes here.
 *
 * self_atomic_for() picks the slot that the given cluster reads
 * itself.  This was written by the cluster's peer the last time
 * the peer ramped.
 *
 * Returns NULL for LITTLE on both, since LITTLE neither arms nor
 * is armed under this scheme (see the macro block at the top of
 * the file for why).
 */
static atomic64_t *
zenith_peer_ramp_peer_atomic(unsigned int cluster_class)
{
	switch (cluster_class) {
	case ZENITH_CLUSTER_BIG:
		return &zenith_peer_ramp_until_ns_prime;
	case ZENITH_CLUSTER_PRIME:
		return &zenith_peer_ramp_until_ns_big;
	default:
		return NULL;
	}
}

static atomic64_t *
zenith_peer_ramp_self_atomic(unsigned int cluster_class)
{
	switch (cluster_class) {
	case ZENITH_CLUSTER_BIG:
		return &zenith_peer_ramp_until_ns_big;
	case ZENITH_CLUSTER_PRIME:
		return &zenith_peer_ramp_until_ns_prime;
	default:
		return NULL;
	}
}

/* uclamp_min expressed as a percent of policy->max, suitable for
 * folding into the existing static-pct floors via max() at the
 * peer_ramp / migration_floor read sites (Patch M2).  Returns 0
 * when the policy has no uclamp_min set or when uclamp is not
 * compiled in (zenith_policy_uclamp_min() handles both cases),
 * which keeps the max() call a no-op.
 *
 * Independent of the master uclamp_min_respect knob; the new
 * per-tier bools at the call sites are gated separately.  Cheap:
 * one cache read, one mul + div.  Computed per call site rather
 * than once per zenith_get_next_freq() because both sites are
 * already short-circuited by their static-pct == 0 guard, and
 * the helper itself is called from inside an existing if-guard.
 */
static unsigned int
zenith_uclamp_min_pct_of_max(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned long umin = zenith_policy_uclamp_min(z_policy);
	unsigned int max_cap;
	unsigned int umin_freq;

	if (!umin || !policy || !policy->max)
		return 0;
	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	if (!max_cap)
		return 0;
	umin_freq = map_util_freq(umin, policy->cpuinfo.max_freq, max_cap);
	if (!umin_freq)
		return 0;
	return (umin_freq * 100U) / policy->max;
}

/* Effective peer_ramp window length, accounting for screen state
 * (Patch M3).  When the screen is on, the legacy peer_ramp_window_ms
 * applies.  When the screen is off, the shadow knob takes over.
 * Both reads are READ_ONCE so a concurrent sysfs write cannot tear
 * the value across the arming and floor-reading paths even though
 * those paths run on different CPUs.
 *
 * Returning 0 disables peer_ramp on the calling path: the arm
 * function bails on a 0 window; the floor-eval site treats 0 as
 * "no floor" via the existing tunable->peer_ramp_window_ms == 0
 * short-circuit (replaced here by the same check on the effective
 * value).  This is the design lever that makes peer_ramp_window_off_ms
 * == 0 fully suppress peer_ramp while the screen is off without
 * touching the existing screen-on path.
 */
static unsigned int
zenith_peer_ramp_effective_window_ms(const struct zenith_tunables *t)
{
	if (READ_ONCE(t->screen_state))
		return READ_ONCE(t->peer_ramp_window_ms);
	return READ_ONCE(t->peer_ramp_window_off_ms);
}

/* Stamp a deadline on the peer cluster's slot.  Called from the
 * three peak tiers (predict_up, peak_prearm, peak_rescue) right
 * after they decide to lift the cluster.  Cheap: one tunable
 * read, one switch, one atomic64_set.  Re-armings just bump the
 * deadline forward, so concurrent stamps from the same cluster
 * (e.g. predict_up on tick N then peak_prearm on tick N+1) end
 * up with the latest deadline winning, which is what we want.
 *
 * Gated entirely on the effective window length: when screen is
 * on this is peer_ramp_window_ms, when screen is off it is
 * peer_ramp_window_off_ms (default 0, so screen-off arms are
 * suppressed by default).  Either way, set the relevant knob to
 * 0 and this function is a couple of branches and a return.
 */
static void
zenith_peer_ramp_arm(struct zenith_policy *z_policy, u64 now_ns)
{
	unsigned int window_ms =
		zenith_peer_ramp_effective_window_ms(z_policy->tunables);
	atomic64_t *peer;

	if (!window_ms)
		return;
	peer = zenith_peer_ramp_peer_atomic(z_policy->cluster_class);
	if (!peer)
		return;
	atomic64_set(peer, now_ns + (u64)window_ms * NSEC_PER_MSEC);
}

/* Migration-arrival detector (Patch K1).  Called once per CPU per
 * update_util tick, after iowait_apply has folded its boost into
 * util but before kcpustat_blend or the wakeup-boost detector run.
 *
 * Compares util against the previous tick's value on the same CPU.
 * If the upward jump exceeds tunables->migration_jump_pct of
 * max_cap, treat it as evidence a task just landed here and stamp
 * z_policy->migration_in_until_ns with a deadline N ms in the
 * future.  N == migration_floor_window_ms.
 *
 * Always updates migration_prev_util so the next tick has a fresh
 * comparison baseline regardless of whether the threshold tripped.
 *
 * No-op when migration_jump_pct == 0 or max_cap == 0 (impossible
 * but cheap to guard).  Caller is responsible for whatever
 * locking the surrounding update_util path needs; this helper
 * does not take any.
 */
static void
zenith_migration_arrival_check(struct zenith_cpu *z_cpu,
			       unsigned long util, unsigned long max_cap,
			       struct zenith_policy *z_policy)
{
	unsigned int jump_pct =
		READ_ONCE(z_policy->tunables->migration_jump_pct);
	unsigned int window_ms;
	unsigned long prev = z_cpu->migration_prev_util;

	z_cpu->migration_prev_util = util;
	if (!jump_pct || !max_cap)
		return;
	if (util <= prev)
		return;
	if ((util - prev) * 100 < (unsigned long)jump_pct * max_cap)
		return;
	window_ms = READ_ONCE(z_policy->tunables->migration_floor_window_ms);
	if (!window_ms)
		return;
	z_policy->migration_in_until_ns =
		ktime_get_ns() + (u64)window_ms * NSEC_PER_MSEC;
}

/* Forward decl: zenith_tier_value() is the V2 tier-classifier
 * accessor, defined later in the file alongside the rest of the
 * auto_tune_v2 worker.  zenith_get_next_freq() (and a handful of
 * its sub-blocks below) read knobs through it, so we need the
 * prototype visible here.  Definition lives near the V2 worker so
 * the override-mask / tier-bit semantics are documented in one
 * place; the declaration just exposes the symbol earlier without
 * hoisting the whole body up out of context.
 */
static unsigned int zenith_tier_value(struct zenith_policy *z_policy,
				      unsigned int tunable,
				      unsigned long override_bit,
				      unsigned long tier_bit);

/* Reset the V1 sample counters and the V2 pending-window
 * accumulator to a fresh-window starting point.  Used by the
 * three call sites that all need the same fresh-classifier
 * substrate:
 *
 *   - the screen 0 -> 1 (resume) edge in zenith_get_next_freq()
 *     (audit fix M7 / M7b),
 *   - auto_tune_store() on the disable -> enable transition,
 *   - zenith_start() when auto_tune is enabled at policy bring-up.
 *
 * Leaves at_last_state / at_last_applied_state / at_cooldown_left
 * untouched so the just-recorded classification and any pending
 * post-transition cooldown survive the reset.  Callers that need
 * a complete classifier reset (boot, sysfs disable->enable) clear
 * at_cooldown_left themselves and reschedule at_work.
 */
static inline void zenith_at_v_reset_window(struct zenith_policy *z_policy)
{
	atomic_set(&z_policy->at_samples_total, 0);
	atomic_set(&z_policy->at_samples_saturated, 0);
	z_policy->at_last_events =
		atomic64_read(&zenith_auto_input_events);
	z_policy->at_pending_windows = 0;
}

/* Patch 1.10: return true when the wall clock is currently inside
 * the quiet-hours window described by [start_min, end_min) on a
 * 0..1439 minute-of-day grid (UTC).  Returns false when the
 * window is zero-length (start == end), which is the configured-
 * disabled state.  When start > end the window wraps midnight,
 * matching how a user would expect to write "22:00 to 06:00".
 *
 * Uses ktime_get_real_seconds() + time64_to_tm() to derive the
 * minute-of-day; both are read-only with no allocation, so the
 * helper is safe to call from the hot eval path.
 */
static inline bool zenith_in_quiet_hours(const struct zenith_tunables *t)
{
	unsigned int start = t->quiet_hours_start_min;
	unsigned int end = t->quiet_hours_end_min;
	struct tm tm;
	unsigned int now_min;

	if (start == end ||
	    start > ZENITH_QUIET_HOURS_MINUTE_MAX ||
	    end > ZENITH_QUIET_HOURS_MINUTE_MAX)
		return false;
	time64_to_tm(ktime_get_real_seconds(), 0, &tm);
	now_min = (unsigned int)tm.tm_hour * 60U +
		  (unsigned int)tm.tm_min;
	if (start < end)
		return now_min >= start && now_min < end;
	return now_min >= start || now_min < end;
}

/* Scale a hold-down millisecond budget by batt_hold_scale_pct when
 * the system is running on battery (Patch 1.2).  Returns @ms
 * unchanged when on AC, when scale_pct is 100, or when scale_pct
 * is 0 (treated as "no scaling configured" so an
 * accidentally-zeroed knob does not silently disable hold).  The
 * comparator (* / 100) keeps the math integer; saturating at
 * UINT_MAX is a non-issue because scale_pct is bounded to 50..300
 * and ms to ZENITH_PEAK_HEADROOM_HOLD_MS_MAX (a few hundred ms).
 */
static inline unsigned int zenith_batt_scaled(unsigned int ms,
					      unsigned int scale_pct)
{
	if (!atomic_read(&zenith_on_battery) || scale_pct == 100 || !scale_pct)
		return ms;
	return (unsigned int)(((u64)ms * scale_pct) / 100U);
}

/*
 * Forward declaration; defined near the bottom of this file
 * alongside the Hikari notifier subscription.  Used from
 * zenith_get_next_freq() below.
 */
static unsigned int zenith_hikari_policy_floor(struct cpufreq_policy *policy);

static unsigned int zenith_get_next_freq(struct zenith_policy *z_policy,
					 unsigned long util, unsigned long max_cap)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int freq, target_freq;
	unsigned int margin;
	/* Tracepoint breadcrumb: updated at each decision branch. Read
	 * once at the end of the function when the event is enabled.
	 */
	const char *tp_path = "eas";
	unsigned int tp_load_pct = 0;
	/* Set when freq is pinned by an explicit user-experience tier
	 * (input_boost full-pin, brutality snap_max / brutal_hold,
	 * climb_step).  Suppresses the post-resolve efficient-freq
	 * ladder and light-load hard cap so those tiers can't clip a
	 * boost back down to a lower bin.  EM validation and the
	 * sampling-down multiplier still run; both are correctness
	 * tiers, not user-experience clips.
	 */
	bool pin_to_target = false;
	/* Patch 1.4: eval-entry timestamp for the decision-latency
	 * histogram.  Single ktime_get_ns() at function entry; the
	 * bucketing arithmetic at commit is constant-time so the
	 * total cost of the always-on histogram is one ktime read +
	 * one subtract + one bucket increment per eval.
	 */
	u64 dec_eval_start_ns = ktime_get_ns();
	unsigned int dynamic_up_thresh, dynamic_bias, input_boost_floor;
	unsigned long uclamp_min, uclamp_max;
	bool uclamp_min_meaningful;

	/* Patch 1.3 cluster-wake-pulse arm.  Compute now_ns once at
	 * the top of the eval and use it both for the gap measurement
	 * and the deadline stamp.  Skip the very first eval after
	 * policy bring-up (cluster_wake_last_eval_ns == 0) so a fresh
	 * policy that has never sampled doesn't trip a spurious pulse
	 * just because the field is zero.  cluster_wake_pulse_ms == 0
	 * short-circuits the arm; the floor application below is also
	 * gated by the deadline being non-zero, so the tier is a
	 * compile-time-shaped no-op when the profile disables it.
	 */
	{
		u64 now_arm_ns = dec_eval_start_ns;
		u64 prev = z_policy->cluster_wake_last_eval_ns;
		unsigned int pulse_ms =
			z_policy->tunables->cluster_wake_pulse_ms;
		unsigned int idle_ms =
			z_policy->tunables->cluster_wake_pulse_idle_ms;
		bool deep_idle_seen = false;

		/* Patch B9-3: suppress cluster_wake_pulse arm when the
		 * cluster just emerged from a deep cpuidle residency.
		 * The cwp tier exists to compensate for cold-cache latency
		 * after a brief micro-idle; a >= ZENITH_VH_CPU_IDLE_-
		 * RESIDENCY_LONG_NS idle period means the workload waking
		 * the cluster is fresh, not a continuation of a hot stream,
		 * and the next eval window will measure actual demand
		 * directly.  Forcing a pulse floor here would ramp past
		 * real demand and waste energy.  Reads are READ_ONCE on
		 * both gates; the residency aggregate is stamped lock-free
		 * by the cpu_idle_exit probe.  When vh_cpu_idle_enable is
		 * 0 the residency field never moves off zero so the gate
		 * is a compile-time-shaped no-op for that build.
		 */
		if (READ_ONCE(z_policy->tunables->vh_cpu_idle_enable) &&
		    READ_ONCE(z_policy->vh_cpu_idle_last_residency_ns) >=
			ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS)
			deep_idle_seen = true;

		if (pulse_ms && prev && !deep_idle_seen &&
		    now_arm_ns - prev >=
			(u64)idle_ms * NSEC_PER_MSEC) {
			z_policy->cluster_wake_pulse_until_ns =
				now_arm_ns +
				(u64)pulse_ms * NSEC_PER_MSEC;
		}
		z_policy->cluster_wake_last_eval_ns = now_arm_ns;
	}

	/* Dynamic Environment Overrides */
	dynamic_up_thresh = zenith_tunable_or_local(z_policy,
		z_policy->tunables->up_threshold,
		z_policy->at_effective_up_threshold);
	dynamic_bias = z_policy->tunables->powersave_bias;

	/* ADPF / uclamp_min floor.  Sampled once here so every decision
	 * tier below (screen-off override, light-load cap, powersave_bias,
	 * final resolve) sees a consistent view.  zero when CONFIG_UCLAMP_TASK
	 * is off, when no task on the policy has set uclamp_min, or when
	 * the governor-level tunable disables respect entirely.
	 */
	uclamp_min = z_policy->tunables->uclamp_min_respect ?
		zenith_policy_uclamp_min(z_policy) : 0;
	uclamp_min_meaningful = uclamp_min >=
		((SCHED_CAPACITY_SCALE * ZENITH_UCLAMP_MIN_MEANINGFUL_PCT) / 100);

	/* Input-boost decay floor.  Computed in the input-boost block
	 * below when we are in the trailing decay window of an active
	 * boost; applied as a minimum on the final freq just before the
	 * resolve label.  Zero means no floor (full-boost phase, or no
	 * boost at all).
	 */
	input_boost_floor = 0;

	/* ADPF / uclamp_max cap.  Sampled once so every decision tier
	 * below sees a consistent view.  SCHED_CAPACITY_SCALE means
	 * "no cap" -- the helper returns that sentinel when uclamp is
	 * not in use, when no task has set a UCLAMP_MAX, or when the
	 * governor-level respect tunable is off.
	 */
	uclamp_max = z_policy->tunables->uclamp_max_respect ?
		zenith_policy_uclamp_max(z_policy) : SCHED_CAPACITY_SCALE;

	/* In-kernel game detector tick.  Gated by the static branch
	 * (default FALSE while game_auto = 0) and the live tunables
	 * scalar (defends against a momentary tear during a sysfs
	 * store; the branch can be true while the scalar transitions
	 * back to 0).  Maintains the per-policy streak counter and
	 * renews the global zenith_game_auto_active_until_ns latch on
	 * sustained match.  See the ZENITH_DEFAULT_GAME_AUTO comment
	 * block.  Order: must follow the local declarations above and
	 * precede any other executable code in this function so the
	 * latter is allowed to declare additional locals without
	 * tripping -Wdeclaration-after-statement.
	 */
	if (static_branch_likely(&zenith_game_auto_key) &&
	    READ_ONCE(z_policy->tunables->game_auto))
		zenith_policy_game_auto_tick(z_policy);
	zenith_auto_predict_tick(z_policy);

	/* Patch K: game_perf_burst FSM tick.  Must run AFTER the
	 * game_auto tick above so a fresh streak-driven latch on
	 * zenith_game_auto_active_until_ns is visible to Signal A
	 * (zenith_eff_game_mode()) on the same tick that detected it.
	 * Gated by the game_perf_burst static branch + live tunables
	 * scalar; both off => zero hot-path cost.  The FSM only
	 * reads load (Signal B) from z_policy->last_load_pct stamped
	 * by the previous tick, so the call site here -- before the
	 * current tick computes its load -- is correct.
	 */
	if (static_branch_likely(&zenith_game_perf_burst_key) &&
	    READ_ONCE(z_policy->tunables->game_perf_burst))
		zenith_gpb_evaluate(z_policy);

	{
		/* Screen-off glide tracking.  Detect 1 -> 0 / 0 -> 1
		 * transitions on tunables->screen_state and stamp
		 * screen_off_arm_ns at the moment we go to 0 so the
		 * glide block below can interpolate towards the cliff
		 * targets across screen_off_glide_ms.  Cleared on the
		 * way back up.  Default 0 leaves both sides unarmed
		 * which makes this a no-op.
		 */
		unsigned int cur_screen = z_policy->tunables->screen_state;

		if (z_policy->screen_state_last && !cur_screen)
			z_policy->screen_off_arm_ns = ktime_get_ns();
		else if (!z_policy->screen_state_last && cur_screen)
			z_policy->screen_off_arm_ns = 0;
		/* Patch K3 stale guard: clear any cached vblank
		 * timestamp on either edge so a multi-second sleep
		 * doesn't make the first event after resume look
		 * like a giant overrun.  Cheap unconditional store on
		 * a transition (rare, observable as a tunable flip),
		 * and harmless even when frame_overrun_slack_us is 0
		 * since the producer treats last_ns == 0 as "first
		 * event".
		 */
		if (z_policy->screen_state_last != cur_screen)
			atomic64_set(&zenith_last_vblank_ns, 0);
		/* Audit fix M7: on the 0 -> 1 (resume) edge, reset the
		 * V1 classifier counters and the V2 pending-window
		 * accumulator.  Without this, the first auto_tune window
		 * after a long suspend / blank would aggregate samples
		 * collected pre-suspend (when the screen was on and the
		 * workload was real) with a window-sized post-resume
		 * gap (where the device had been idle), producing a
		 * misleadingly low sat_pct and biasing V1 toward
		 * EFFICIENCY for one extra window after resume.
		 *
		 * Treat resume as a fresh measurement: clear the atomic
		 * sample counters and the pending-window state, leaving
		 * at_last_state / at_last_applied_state untouched (so
		 * the just-restored state survives the reset and the
		 * cooldown timer continues to gate further moves).
		 *
		 * Audit fix M7b extends the same reset edge to clear
		 * the per-policy streak / hysteresis state that would
		 * otherwise carry stale pre-suspend continuity into the
		 * first post-resume sample window: peak_starve_count
		 * (entry hysteresis for peak-headroom rescue),
		 * hispeed_entry_count and hispeed_active (entry / sticky
		 * state for the hispeed tier), brutal_entry_count and
		 * brutal_active (the up_threshold streak hysteresis).
		 * The deadline atomics in this family
		 * (input_boost_until_ns, peer_ramp_until_ns_*,
		 * frame_overrun_until_ns, peak_rescue_until_ns) already
		 * self-disarm: their absolute ktime_get_ns() deadlines
		 * are in the past after any non-trivial suspend, so a
		 * subsequent atomic64_read() naturally evaluates the
		 * boost as expired without an explicit clear.
		 *
		 * Cheap: 4 atomic_sets and 1 unsigned-int store on the
		 * V1/V2 path, plus 5 plain stores for the streak/
		 * hysteresis state, only on a transition (not every
		 * tick).  No effect on the 1 -> 0 (suspend) edge --
		 * those samples are still useful for the screen-off
		 * glide path.
		 */
		if (!z_policy->screen_state_last && cur_screen) {
			zenith_at_v_reset_window(z_policy);
			z_policy->peak_starve_count = 0;
			z_policy->hispeed_entry_count = 0;
			z_policy->brutal_entry_count = 0;
			z_policy->hispeed_active = false;
			z_policy->brutal_active = false;
		}
		z_policy->screen_state_last = cur_screen;
	}

	if (z_policy->tunables->screen_state == 0 && !uclamp_min_meaningful) {
		unsigned int glide_ms = zenith_glide_value(z_policy,
				z_policy->tunables->screen_off_glide_ms,
				z_policy->at_local_screen_off_glide_ms);
		u64 now = z_policy->screen_off_arm_ns ? ktime_get_ns() : 0;
		u64 elapsed_ns = (z_policy->screen_off_arm_ns &&
				  now > z_policy->screen_off_arm_ns) ?
				  now - z_policy->screen_off_arm_ns : 0;
		u64 glide_ns = (u64)glide_ms * NSEC_PER_MSEC;

		if (glide_ms && glide_ns && elapsed_ns < glide_ns) {
			/* Glide phase: ramp dynamic_up_thresh from the
			 * natural up_threshold (at the moment the screen
			 * went off) up to the legacy 95 cliff target
			 * across screen_off_glide_ms; mirror the same
			 * proportional ramp on dynamic_bias from the
			 * configured powersave_bias up to 500 (50 %%
			 * penalty).  Eliminates the cliff-on-cliff that
			 * happens when AOD / blanking handlers stamp
			 * screen_state=0 while userspace work is still
			 * winding down.  After glide_ms elapses the next
			 * tick re-enters this branch with elapsed_ns >=
			 * glide_ns and the legacy assignments below take
			 * effect verbatim.
			 */
			unsigned int floor =
				zenith_tunable_or_local(z_policy,
				    z_policy->tunables->up_threshold,
				    z_policy->at_effective_up_threshold);
			unsigned int bias_floor =
				z_policy->tunables->powersave_bias;
			u64 t256 = div64_u64(elapsed_ns * 256ULL, glide_ns);

			if (t256 > 256)
				t256 = 256;
			if (floor < 95)
				dynamic_up_thresh = floor +
				    (unsigned int)(((95U - floor) * t256) >> 8);
			else
				dynamic_up_thresh = floor;
			if (bias_floor < 500)
				dynamic_bias = bias_floor +
				    (unsigned int)(((500U - bias_floor) * t256)
							>> 8);
			else
				dynamic_bias = bias_floor;
		} else {
			dynamic_up_thresh = 95; /* Hard to wake up */
			dynamic_bias = 500;     /* 50% penalty */
		}
		z_policy->brutal_active = false; /* no hysteresis screen-off */
	} else if (z_policy->tunables->screen_state == 0) {
		/* Screen is off but userspace has set a meaningful ADPF
		 * uclamp_min on at least one task in this policy --
		 * respect the hint.  Skip the 50 %% penalty and the
		 * 95 %% up_threshold bump; fall through to the normal
		 * eval path.  The final-freq floor applied below still
		 * guarantees we deliver at least the uclamp_min-implied
		 * frequency.
		 */
		z_policy->brutal_active = false;
	} else if (zenith_thermal_active(z_policy)) {
		/* Hot.  Default behaviour: snap dynamic_up_thresh to 90
		 * (the legacy cliff).  When thermal_pressure_continuous
		 * is set, ramp from the policy's normal up_threshold
		 * (at 0%% pressure) to 90 (at 100%% pressure) using the
		 * same arch_scale_thermal_pressure()-derived percentage
		 * the auto-tune V2 classifier uses; that removes the
		 * audible / observable step that otherwise happens the
		 * moment thermal_active flips on after a long burst.
		 */
		if (zenith_glide_value(z_policy,
				z_policy->tunables->thermal_pressure_continuous,
				z_policy->at_local_thermal_pressure_continuous)) {
			unsigned int floor = zenith_tunable_or_local(z_policy,
				z_policy->tunables->up_threshold,
				z_policy->at_effective_up_threshold);
			unsigned int pct =
				zenith_policy_thermal_pressure_pct(z_policy);

			if (pct > 100)
				pct = 100;
			if (floor < 90)
				dynamic_up_thresh = floor +
					((90U - floor) * pct) / 100U;
			else
				dynamic_up_thresh = floor;
		} else {
			dynamic_up_thresh = 90; /* Relaxed for thermals */
		}
	} else if (z_policy->tunables->up_threshold_hispeed &&
		   zenith_eff_hispeed_freq(z_policy) &&
		   policy->cur >= zenith_eff_hispeed_freq(z_policy)) {
		/* Above the hispeed floor, require the stiffer
		 * threshold before escalating all the way to
		 * policy->max. Screen-off and thermal overrides take
		 * precedence because both already pin an even
		 * higher value.
		 */
		dynamic_up_thresh = z_policy->tunables->up_threshold_hispeed;
	} else if (z_policy->tunables->up_threshold_adaptive &&
		   dynamic_up_thresh == zenith_tunable_or_local(z_policy,
				z_policy->tunables->up_threshold,
				z_policy->at_effective_up_threshold)) {
		/* Variance-adaptive shaping: lower dynamic_up_thresh by
		 * up to up_threshold_adaptive percent of its value when
		 * the recent load signal is bursty.  See
		 * ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE for semantics.
		 * Skipped when any of the harder overrides above is in
		 * effect (those are absolute pinning values and must not
		 * be softened).  load_var_ewma_x256 is in 1/256ths of a
		 * percentage-point of mean abs change; 30*256 == fully
		 * saturated bursty signal.
		 */
		unsigned int adaptive =
			z_policy->tunables->up_threshold_adaptive;
		unsigned int var = z_policy->load_var_ewma_x256 / 256;
		unsigned int swing;

		if (adaptive > ZENITH_UP_THRESHOLD_ADAPTIVE_MAX)
			adaptive = ZENITH_UP_THRESHOLD_ADAPTIVE_MAX;
		if (var > ZENITH_UP_THRESHOLD_ADAPTIVE_MAX)
			var = ZENITH_UP_THRESHOLD_ADAPTIVE_MAX;
		/* swing = up_threshold * adaptive% * (var/30)
		 *       = up_threshold * adaptive * var
		 *         / (100 * ZENITH_UP_THRESHOLD_ADAPTIVE_MAX)
		 */
		swing = ((unsigned int)dynamic_up_thresh * adaptive * var) /
			(100u * ZENITH_UP_THRESHOLD_ADAPTIVE_MAX);
		if (swing < dynamic_up_thresh)
			dynamic_up_thresh -= swing;
	}

	/* prefer_silver_aware coordination (dead -- Kconfig removed):
	 * the code below is permanently gated because ps_hit_rate_pct
	 * stays at 0.  The logic is preserved for documentation;
	 * re-enable by re-importing the prefer_silver hit/miss
	 * counters and restoring CONFIG_SCHED_PREFER_SILVER.
	 */
	if (zenith_glide_value(z_policy,
			z_policy->tunables->prefer_silver_aware,
			z_policy->at_local_prefer_silver_aware) &&
	    (z_policy->cluster_class == ZENITH_CLUSTER_BIG ||
	     (z_policy->cluster_class == ZENITH_CLUSTER_PRIME &&
	      !zenith_topology_has_big_class())) &&
	    z_policy->ps_hit_rate_pct >=
		    z_policy->tunables->prefer_silver_hot_threshold_pct) {
		unsigned int natural = zenith_tunable_or_local(z_policy,
			z_policy->tunables->up_threshold,
			z_policy->at_effective_up_threshold);

		if (dynamic_up_thresh <= natural) {
			unsigned int bump_max =
			    z_policy->tunables->prefer_silver_hot_bump_pct;
			unsigned int rate     = z_policy->ps_hit_rate_pct;
			unsigned int thresh   =
			    z_policy->tunables->prefer_silver_hot_threshold_pct;
			unsigned int bump;

			if (bump_max > ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT)
				bump_max = ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT;

			/*
			 * Linear ramp: at hit_rate == threshold, bump = 0;
			 * at hit_rate == 100, bump = bump_max.  Replaces the
			 * step function (flat bump_max above threshold, 0
			 * below) so the tail of the ramp is smooth, a 51%
			 * hit rate doesn't yield the same up_threshold
			 * inflation as a 99% one, and the worst-case bump
			 * is unchanged from the previous fixed form.
			 *
			 * thresh >= 100 would make the denominator zero or
			 * negative; the outer >=-test already required
			 * rate >= thresh to enter this branch, so for
			 * thresh == 100 the only legal rate is also 100,
			 * collapsing the ramp to bump_max as an exact case.
			 * Treat thresh > 100 (impossible per the sysfs
			 * store handler's 0..100 clamp, but defensive)
			 * as no bump.
			 */
			if (rate <= thresh || thresh > 100)
				bump = 0;
			else if (thresh == 100)
				bump = bump_max;
			else
				bump = (bump_max * (rate - thresh)) /
				       (100u - thresh);

			if (dynamic_up_thresh + bump <= 95)
				dynamic_up_thresh += bump;
			else
				dynamic_up_thresh = 95;
		}
	}

	if (max_cap)
		tp_load_pct = (unsigned int)((util * 100) / max_cap);

	/* Patch H: stamp last_runnable_ns whenever the cluster has
	 * any util.  Read by the post-tier sleeper-tail shave to
	 * decide whether the cluster has been idle long enough to
	 * justify shaving the next freq decision.
	 */
	if (tp_load_pct > 0)
		z_policy->last_runnable_ns = ktime_get_ns();

	/* 0. Input Boost — pin to policy->max for the non-decay portion of
	 * input_boost_ms after a key or touch event, then linearly ramp
	 * down across the trailing input_boost_decay_ms so the gesture
	 * tail doesn't cliff-drop back to the load-dependent target.
	 * Gated by screen_state so we don't wake clusters while the
	 * display is off, and optionally gated by input_boost_big_only
	 * so small-cluster policies skip the boost on heterogeneous SoCs.
	 */
	if (zenith_tunable_or_local(z_policy, z_policy->tunables->input_boost_ms,
				    z_policy->at_effective_input_boost_ms) &&
	    z_policy->tunables->screen_state &&
	    (!z_policy->tunables->input_boost_big_only ||
	     z_policy->is_big_cluster)) {
		u64 now = ktime_get_ns();
		u64 until = (u64)atomic64_read(&zenith_input_boost_until_ns);

		if (now < until) {
			unsigned int decay_ms =
				z_policy->tunables->input_boost_decay_ms;
			u64 decay_ns;
			u64 remaining = until - now;
			unsigned int cap_pct = zenith_tunable_or_local(
				z_policy, z_policy->tunables->input_boost_cap_pct,
				z_policy->at_effective_input_boost_cap_pct);
			unsigned int gm = zenith_eff_game_mode(
				zenith_tunable_or_local(z_policy,
					z_policy->tunables->game_mode,
					z_policy->at_effective_game_mode));
			unsigned int boost_ceiling;
			unsigned int idle_streak_th =
				READ_ONCE(z_policy->tunables->boost_idle_streak);

			/* Patch F: persistent-idle preemption.  If a
			 * boost is still armed but the cluster has been
			 * idle for boost_idle_streak ticks, skip the
			 * boost on this tier-0 path so the lower tiers
			 * pick the freq.  The global until_ns is left
			 * armed; other policies still see the boost.
			 */
			if (idle_streak_th &&
			    z_policy->boost_idle_low_streak >= idle_streak_th) {
				atomic64_inc(&zenith_in_boosts_early_exit);
				z_policy->boost_idle_low_streak = 0;
				goto skip_input_boost;
			}

			/* game_mode=2 (turbo) overrides the user-set cap and
			 * pins the full-boost phase to policy->max regardless
			 * of input_boost_cap_pct.  Runtime-only; the stored
			 * tunable is left untouched.
			 */
			if (gm >= 2)
				cap_pct = 0;
			boost_ceiling = (cap_pct && cap_pct <= 100) ?
				(policy->max / 100) * cap_pct : policy->max;

			/* game_mode decay stretch: lengthen the trailing decay
			 * window.  Level 1 uses ZENITH_GAME_BOOST_DECAY_PCT
			 * (default 130%%, ~30%% longer); level 2 uses
			 * ZENITH_GAME_L2_BOOST_DECAY_PCT (default 160%%,
			 * ~60%% longer) so input boosts hold the floor even
			 * longer across stick-flick / camera pan inputs.
			 * Pure no-op when game_mode=0.
			 */
			if (gm >= 2 && decay_ms)
				decay_ms = (decay_ms *
					    ZENITH_GAME_L2_BOOST_DECAY_PCT) / 100;
			else if (gm == 1 && decay_ms)
				decay_ms = (decay_ms *
					    ZENITH_GAME_BOOST_DECAY_PCT) / 100;
			decay_ns = (u64)decay_ms * NSEC_PER_MSEC;

			/* Latch the deadline for the boost-exit hold-down
			 * (step 7).  Refreshed on every active-boost tick so
			 * the stretched down-rate window starts from the
			 * actual boost expiry, not from when the latch was
			 * first set.
			 */
			z_policy->boost_active_until_ns = until;

			/* Patch F: update the persistent-idle streak.
			 * Increment when the cluster's load_pct sits
			 * below boost_idle_thresh; reset otherwise.
			 * Capped at ZENITH_BOOST_IDLE_STREAK_MAX so a
			 * stuck-near-zero workload can't accumulate
			 * unbounded streak credit.
			 */
			{
				unsigned int idle_thresh =
					READ_ONCE(z_policy->tunables->boost_idle_thresh);

				if (idle_thresh && tp_load_pct < idle_thresh) {
					if (z_policy->boost_idle_low_streak <
					    ZENITH_BOOST_IDLE_STREAK_MAX)
						z_policy->boost_idle_low_streak++;
				} else {
					z_policy->boost_idle_low_streak = 0;
				}
			}

			/* A capped ceiling that lands below policy->min would
			 * push the decay floor negative; clamp to min so the
			 * floor always remains a no-op or upward force.
			 */
			if (boost_ceiling < policy->min)
				boost_ceiling = policy->min;

			if (remaining > decay_ns) {
				/* Full-boost phase: pin to the capped ceiling
				 * (or policy->max when no cap is set).
				 */
				freq = boost_ceiling;
				tp_path = "input_boost";
				pin_to_target = true;
				goto apply_uclamp_max_cap;
			} else if (decay_ns) {
				/* Decay phase: ramp a floor from boost_ceiling
				 * down toward policy->min over the trailing
				 * decay_ns.  Shape is picked by
				 * input_boost_decay_curve: 0 = linear (the
				 * historical behaviour), 1 = cubic ease-in
				 * (floor holds high for most of the window,
				 * drops fast at the tail).  Normal eval runs
				 * after this point and may pick a higher freq;
				 * the floor only kicks in if the load has
				 * already dropped so far that eval undershoots
				 * the ramp.
				 */
				u64 elapsed = decay_ns - remaining;
				u64 span = boost_ceiling - policy->min;
				unsigned int dropped;

				if (z_policy->tunables->input_boost_decay_curve) {
					u32 t256 = (u32)div64_u64(elapsed * 256,
									  decay_ns);
					u32 cubic;

					if (t256 > 256)
						t256 = 256;
					cubic = (t256 * t256 * t256) >> 16;
					if (cubic > 256)
						cubic = 256;
					dropped = (unsigned int)
						((span * cubic) >> 8);
				} else {
					dropped = (unsigned int)
						div64_u64(span * elapsed,
							  decay_ns);
				}

				input_boost_floor = boost_ceiling - dropped;
			} else {
				/* Decay window not configured: original cliff. */
				freq = boost_ceiling;
				tp_path = "input_boost";
				pin_to_target = true;
				goto apply_uclamp_max_cap;
			}
		} else {
			/* Boost expired naturally; drop any accumulated
			 * idle-streak credit so the next boost arming
			 * starts from zero (Patch F).
			 */
			z_policy->boost_idle_low_streak = 0;
		}
	} else {
		/* Boost feature gated off (input_boost_ms == 0,
		 * screen off, or input_boost_big_only excluded this
		 * cluster): clear the idle streak so a later re-enable
		 * sees a fresh count.
		 */
		z_policy->boost_idle_low_streak = 0;
	}
skip_input_boost:

	/* 1. Ondemand Brutality (with hysteresis).
	 *
	 * Snap to policy->max when load crosses up_threshold and stay there
	 * while load remains above down_threshold. This creates a band
	 * around the transition so we do not ping-pong between policy->max
	 * and the bin just below it on every tick. Clearing brutal_active
	 * falls through to the EAS proportional path.
	 */
	if (max_cap) {
		unsigned int load_pct = (util * 100) / max_cap;

		if (z_policy->tunables->auto_tune) {
			atomic_inc(&z_policy->at_samples_total);
			if (load_pct >= z_policy->tunables->auto_tune_sat_load_pct)
				atomic_inc(&z_policy->at_samples_saturated);
		}

		/* ignore_nice_load: dampen the load percentage by the
		 * fraction of wall time recently spent in niced-user
		 * mode, so background niced work does not trigger
		 * snap-to-max. Approximate — PELT-based util already
		 * weighs niced tasks by their scheduler weight.
		 */
		if (z_policy->tunables->ignore_nice_load && z_policy->nice_pct)
			load_pct = load_pct * (100 - z_policy->nice_pct) / 100;

		if (load_pct >= dynamic_up_thresh) {
			unsigned int b_streak =
				z_policy->tunables->brutal_entry_streak;
			unsigned int climb_mode =
				z_policy->tunables->climb_mode;

			/* game_mode=2 (turbo) forces SNAP climb regardless of
			 * the user-set climb_mode, so the brutality path
			 * always pins policy->max on threshold crossing.
			 * Runtime-only; the stored tunable is left untouched.
			 */
			if (zenith_eff_game_mode(
					zenith_tunable_or_local(z_policy,
						z_policy->tunables->game_mode,
						z_policy->at_effective_game_mode)) >= 2)
				climb_mode = ZENITH_CLIMB_MODE_SNAP;

			if (climb_mode == ZENITH_CLIMB_MODE_STEP) {
				/* Gentle climb: step by freq_step_pct of
				 * policy->max from the current bin.
				 * Bypasses hysteresis entirely, including
				 * brutal_entry_streak -- STEP mode is
				 * already gentle by design so there is
				 * nothing to debounce.
				 */
				unsigned int step =
				    (policy->max *
				     z_policy->tunables->freq_step_pct) / 100;

				/* Load-proportional scaling (I4).  When
				 * freq_step_adaptive is set, widen the step
				 * linearly with overshoot above the active
				 * up_threshold so genuinely heavy load
				 * converges faster without changing the
				 * light-overshoot behaviour at the boundary.
				 * dynamic_up_thresh already reflects the
				 * screen-off / thermal / hispeed overrides
				 * above, so the span denominator is always
				 * the effective ceiling for this sample.
				 */
				if (z_policy->tunables->freq_step_adaptive &&
				    dynamic_up_thresh < 100) {
					unsigned int span =
						100 - dynamic_up_thresh;
					unsigned int overshoot =
						load_pct > dynamic_up_thresh ?
						load_pct - dynamic_up_thresh : 0;
					if (overshoot > span)
						overshoot = span;
					step += (step * overshoot) / span;
				}

				if (!step)
					step = 1;
				freq = policy->cur + step;
				if (freq > policy->max)
					freq = policy->max;
				z_policy->brutal_active = false;
				z_policy->brutal_entry_count = 0;
				tp_path = "climb_step";
				pin_to_target = true;
				goto apply_uclamp_max_cap;
			}

			/* Entry-side streak hysteresis for the brutality
			 * snap-to-max path.  Mirror of the hispeed tier:
			 * increment the entry counter each qualifying
			 * sample (saturate at the cap to avoid overflow),
			 * and only flip brutal_active once the counter
			 * exceeds tunables->brutal_entry_streak.
			 * b_streak == 0 collapses to the historical
			 * immediate-flip behaviour because the fresh
			 * counter crosses 1 > 0 on the very first sample.
			 * Counter reset sites: the STEP path above, the
			 * below-threshold fall-through, and the brutal_hold
			 * exit (all three mean "not in entry territory
			 * anymore").
			 */
			if (!z_policy->brutal_active) {
				if (z_policy->brutal_entry_count <
				    ZENITH_BRUTAL_ENTRY_STREAK_MAX)
					z_policy->brutal_entry_count++;
				if (z_policy->brutal_entry_count <= b_streak) {
					/* Streak not satisfied yet -- fall
					 * through to the hispeed / EAS path
					 * below without flipping brutal_active.
					 * No goto: the hispeed tier gets a
					 * chance to apply its own floor.
					 */
					goto brutal_entry_deferred;
				}
			}

			z_policy->brutal_active = true;
			z_policy->brutal_entry_count = 0;
			freq = policy->max;
			tp_path = "snap_max";
			pin_to_target = true;
			goto apply_uclamp_max_cap;
		}
		z_policy->brutal_entry_count = 0;

brutal_entry_deferred:
		if ((z_policy->tunables->climb_mode == ZENITH_CLIMB_MODE_SNAP ||
		     zenith_eff_game_mode(
			zenith_tunable_or_local(z_policy,
				z_policy->tunables->game_mode,
				z_policy->at_effective_game_mode)) >= 2) &&
		    z_policy->brutal_active) {
			unsigned int eff_down =
				zenith_tunable_or_local(z_policy,
					READ_ONCE(z_policy->tunables->down_threshold),
					z_policy->at_effective_down_threshold);
			unsigned int adaptive = zenith_tunable_or_local(z_policy,
				READ_ONCE(z_policy->tunables->down_threshold_adaptive),
				z_policy->at_effective_down_threshold_adaptive);

			if (adaptive) {
				unsigned int var =
					z_policy->load_var_ewma_x256 / 256;
				unsigned int swing;

				if (adaptive > ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX)
					adaptive = ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX;
				if (var > ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX)
					var = ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX;
				swing = (eff_down * adaptive * var) /
					(100u * ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX);
				if (swing < eff_down)
					eff_down -= swing;
			}

			if (load_pct >= eff_down) {
				freq = policy->max;
				tp_path = "brutal_hold";
				pin_to_target = true;
				goto apply_uclamp_max_cap;
			}
		}

		/* Brutal-hold cliff exit.  When tunables->brutal_decay_ms
		 * is non-zero, arm a tail-glide deadline so the EAS
		 * post-floor below tapers freq from policy->max down to
		 * the load-dependent target across the configured window
		 * instead of cliff-dropping in a single tick.  The
		 * deadline is sticky: it survives until either expired
		 * or replaced by a fresh brutal-hold re-entry (which
		 * implicitly clears it on the next exit).  No-op when
		 * brutal_decay_ms == 0.
		 */
		if (z_policy->brutal_active) {
			unsigned int decay_ms = zenith_glide_value(z_policy,
				z_policy->tunables->brutal_decay_ms,
				z_policy->at_local_brutal_decay_ms);

			if (decay_ms) {
				if (decay_ms > ZENITH_BRUTAL_DECAY_MS_MAX)
					decay_ms =
						ZENITH_BRUTAL_DECAY_MS_MAX;
				z_policy->brutal_decay_arm_ms = decay_ms;
				z_policy->brutal_decay_until_ns =
					ktime_get_ns() +
					(u64)decay_ms * NSEC_PER_MSEC;
			}
		}
		z_policy->brutal_active = false;
	}

	/* 2. Schedutil EAS Proportional Math with Headroom */
	if (arch_scale_freq_invariant())
		freq = policy->cpuinfo.max_freq;
	else
		freq = policy->cur + (policy->cur >> 2);

	{
		unsigned long _vh_next_freq = 0;
		trace_android_vh_map_util_freq(util, freq, max_cap,
					       &_vh_next_freq, policy,
					       &z_policy->need_freq_update);
		if (_vh_next_freq)
			freq = _vh_next_freq;
		else
			freq = map_util_freq(util, freq, max_cap);
	}

	/* 2a. Brutal-hold tail glide.  When the cliff exit above armed
	 * a brutal_decay_ms deadline, linearly interpolate a floor
	 * between policy->max (at arm time) and the EAS-computed freq
	 * (at expiry).  Eliminates the audible / visible drop that the
	 * legacy cliff produces on bursty workloads.  Self-disarms once
	 * the deadline passes.  When the deadline is unarmed (the
	 * common case), this whole block is a single zero-test branch.
	 */
	if (z_policy->brutal_decay_until_ns) {
		u64 now = ktime_get_ns();

		if (now >= z_policy->brutal_decay_until_ns) {
			z_policy->brutal_decay_until_ns = 0;
			z_policy->brutal_decay_arm_ms = 0;
		} else if (z_policy->brutal_decay_arm_ms) {
			u64 total_ns = (u64)z_policy->brutal_decay_arm_ms *
				NSEC_PER_MSEC;
			u64 remaining_ns =
				z_policy->brutal_decay_until_ns - now;
			unsigned int max_freq = policy->max;
			u64 span;

			if (max_freq > freq && total_ns) {
				span = (u64)(max_freq - freq) * remaining_ns;
				span = div64_u64(span, total_ns);
				if ((u64)freq + span > max_freq)
					freq = max_freq;
				else
					freq = freq + (unsigned int)span;
			}
		}
	}

	/* 2a'. Predictive up-shift via util-trend ring.  Lifts the
	 * cluster up to eff_hispeed_freq before the level-triggered
	 * hispeed tier (2b) catches a rising workload.  See the block
	 * comment above ZENITH_DEFAULT_PREDICT_UP_THRESH for the full
	 * rationale.
	 *
	 * Trigger:
	 *   - tunables->predict_up_thresh > 0 (gate);
	 *   - max_cap and policy->max non-zero (init guard);
	 *   - pin_to_target false (don't fight an explicit pin tier);
	 *   - eff_hispeed_freq configured and freq < eff_hispeed_freq;
	 *   - peak_starve_count == 0 (don't double-fire with rescue
	 *     / pre-arm during sustained-high regimes);
	 *   - peak_rescue_until_ns expired (don't refire inside a
	 *     rescue hold-down window);
	 *   - util_history_count >= predict_up_window (warm-up gate);
	 *   - delta_x256 = ((newest - oldest) * 256) / max_cap >=
	 *     predict_up_thresh.
	 *
	 * Effect: lift freq to eff_hispeed_freq, set tp_path
	 * "predict_up".  The hispeed tier (2b) sees freq already at
	 * the floor and naturally treats the lift as a continuation;
	 * the rescue tier (2c) sees freq lifted out of the starvation
	 * window and resets peak_starve_count on its first sample.
	 */
	if (z_policy->tunables->predict_up_thresh &&
	    max_cap && policy->max && !pin_to_target) {
		unsigned int window = z_policy->tunables->predict_up_window;
		unsigned int eff_hispeed = zenith_eff_hispeed_freq(z_policy);
		u64 now_ns = ktime_get_ns();

		if (window < ZENITH_PREDICT_UP_WINDOW_MIN)
			window = ZENITH_PREDICT_UP_WINDOW_MIN;
		else if (window > ZENITH_PREDICT_UP_WINDOW_MAX)
			window = ZENITH_PREDICT_UP_WINDOW_MAX;

		if (eff_hispeed && freq < eff_hispeed &&
		    z_policy->peak_starve_count == 0 &&
		    now_ns >= z_policy->peak_rescue_until_ns &&
		    z_policy->util_history_count >= window) {
			unsigned int idx = z_policy->util_history_idx;
			unsigned int newest_idx =
				(idx + ZENITH_PREDICT_UP_WINDOW_MAX - 1) %
				ZENITH_PREDICT_UP_WINDOW_MAX;
			unsigned int oldest_idx =
				(idx + ZENITH_PREDICT_UP_WINDOW_MAX - window) %
				ZENITH_PREDICT_UP_WINDOW_MAX;
			unsigned long newest = z_policy->util_history[newest_idx];
			unsigned long oldest = z_policy->util_history[oldest_idx];

			if (newest > oldest) {
				unsigned long delta = newest - oldest;
				unsigned int delta_x256 = (unsigned int)
					((delta * 256) / max_cap);

				if (delta_x256 >=
				    z_policy->tunables->predict_up_thresh) {
					freq = eff_hispeed;
					tp_path = "predict_up";
					/* Patch D: arm the peer cluster
					 * for cross-cluster IPC chains.
					 * No-op when peer_ramp_window_ms
					 * is 0 or this is LITTLE.
					 */
					zenith_peer_ramp_arm(z_policy, now_ns);
				}
			}

			/* Patch C3: PELT rising-edge tier.  Catches a
			 * sharp single-sample slope-up that the
			 * rolling-window delta above dilutes.  Reuses
			 * newest_idx (already computed) and pulls the
			 * sample one position older so the slope is
			 * (newest - prev) / max_cap.  Only fires if
			 * predict_up did not already lift this tick
			 * (freq still below eff_hispeed) AND the slope
			 * test passes AND the absolute level guard
			 * (pelt_rising_edge_min_pct) is satisfied so we
			 * do not chase noise from a low base.
			 */
			if (freq < eff_hispeed &&
			    z_policy->tunables->pelt_rising_edge_thresh) {
				unsigned int prev_idx;
				unsigned long prev;

				prev_idx = (idx + ZENITH_PREDICT_UP_WINDOW_MAX - 2)
					% ZENITH_PREDICT_UP_WINDOW_MAX;
				prev = z_policy->util_history[prev_idx];

				if (newest > prev) {
					unsigned long edge = newest - prev;
					unsigned int edge_x256 = (unsigned int)
						((edge * 256) / max_cap);
					unsigned int newest_pct = (unsigned int)
						((newest * 100) / max_cap);

					if (edge_x256 >=
					    z_policy->tunables->pelt_rising_edge_thresh &&
					    newest_pct >=
					    z_policy->tunables->pelt_rising_edge_min_pct) {
						freq = eff_hispeed;
						tp_path = "pelt_edge";
						zenith_peer_ramp_arm(z_policy,
								     now_ns);
					}
				}
			}
		}
	}

	/* 2b. Hispeed floor — intermediate snap tier.
	 *
	 * When load has crossed hispeed_load but is still below
	 * up_threshold, we are in an "active but not saturated" regime.
	 * Rather than letting proportional math pick a freq based on a
	 * noisy PELT signal, floor the target at hispeed_freq so the
	 * cluster is at least at its "fast but efficient" bin. Above
	 * up_threshold we already went straight to policy->max in
	 * step 1, so this tier never competes with brutality.
	 *
	 * hispeed_freq=0 disables the tier.
	 */
	{
		unsigned int eff_hispeed = zenith_eff_hispeed_freq(z_policy);

		if (eff_hispeed && max_cap) {
			unsigned int load_pct = (util * 100) / max_cap;
			unsigned int entry = z_policy->tunables->hispeed_load;
			unsigned int hyst  = z_policy->tunables->hispeed_hyst_pct;
			unsigned int exit  = hyst < entry ? entry - hyst : 0;
			unsigned int streak =
				z_policy->tunables->hispeed_entry_streak;

			/* Entry-side streak hysteresis.  When streak == 0
			 * the historical immediate-flip behaviour is
			 * preserved (hispeed_entry_count crosses 1 > 0 on
			 * the very first qualifying sample).  When
			 * streak == N>0, require load_pct >= entry to
			 * hold for N+1 consecutive ticks before flipping.
			 * Counter is reset whenever load_pct drops below
			 * entry, and saturates at the streak cap so it
			 * cannot wrap.
			 */
			if (load_pct >= entry) {
				if (z_policy->hispeed_entry_count <
				    ZENITH_HISPEED_ENTRY_STREAK_MAX)
					z_policy->hispeed_entry_count++;
			} else {
				z_policy->hispeed_entry_count = 0;
			}

			if (!z_policy->hispeed_active &&
			    load_pct >= entry &&
			    z_policy->hispeed_entry_count > streak)
				z_policy->hispeed_active = true;
			else if (z_policy->hispeed_active && load_pct < exit)
				z_policy->hispeed_active = false;

			if (z_policy->hispeed_active && freq < eff_hispeed) {
				freq = eff_hispeed;
				tp_path = "hispeed";
			}
		} else {
			/* Tier disabled or max_cap == 0: drop the sticky bit
			 * so we don't carry it across a disable/enable cycle.
			 * Reset the streak counter too so we don't carry
			 * partial entry credit across a disable.
			 */
			z_policy->hispeed_active = false;
			z_policy->hispeed_entry_count = 0;
		}
	}

	/* 2b'. Peak-headroom pre-arm.  Soft early intervention that
	 * lifts the cluster up to eff_hispeed_freq while the
	 * starvation streak is accumulating but has not yet crossed
	 * peak_headroom_starve_streak.  See peak_headroom_prearm in
	 * struct zenith_tunables for the full rationale.
	 *
	 * Trigger: gate is on, rescue gate is on (so peak_starve_count
	 * is being maintained at all), max_cap and policy->max non-
	 * zero, pin_to_target is false, eff_hispeed is configured,
	 * peak_starve_count > 0 (last sample was starving), current
	 * freq is below eff_hispeed, and lifting to eff_hispeed would
	 * actually escape the starvation floor (eff_hispeed >=
	 * floor_freq -- otherwise the rescue tier 2c would just
	 * re-flag the cluster as starving on the next sample, the
	 * hispeed pull would be wasted energy, and the rescue would
	 * be delayed).
	 *
	 * Effect: a one-shot per-episode graduated response.  Once the
	 * pull lands and freq >= floor_freq, 2c's starve check
	 * resolves false and peak_starve_count resets to 0, which is
	 * also the gate that lets this pre-arm fire, so the
	 * intervention won't repeat until a fresh starvation episode
	 * begins.  If eff_hispeed wasn't enough (peak_starve_count
	 * keeps climbing past the streak), the rescue tier in 2c
	 * fires as normal.
	 */
	if (z_policy->tunables->peak_headroom_rescue &&
	    z_policy->tunables->peak_headroom_prearm &&
	    max_cap && policy->max && !pin_to_target &&
	    z_policy->peak_starve_count > 0) {
		unsigned int eff_hispeed = zenith_eff_hispeed_freq(z_policy);
		unsigned int floor_pct =
			z_policy->tunables->peak_headroom_freq_floor_pct;
		unsigned int floor_freq =
			(policy->max / 100) * floor_pct;

		if (eff_hispeed && eff_hispeed >= floor_freq &&
		    freq < eff_hispeed) {
			freq = eff_hispeed;
			tp_path = "peak_prearm";
			/* Patch D: arm the peer cluster's deadline
			 * so its next eval picks up a soft floor.
			 */
			zenith_peer_ramp_arm(z_policy, ktime_get_ns());
		}
	}

	/* 2c. Peak-headroom rescue.  Watchdog tier that lifts the
	 * cluster off a sustained-high-util / sub-peak floor.  See
	 * ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE for the full rationale.
	 *
	 * Skipped when:
	 *   - The tunable gate is off (peak_headroom_rescue == 0).
	 *   - max_cap or policy->max is 0 (init / no freq table).
	 *   - pin_to_target is true (input_boost / brutality already
	 *     pinned the cluster, no rescue needed).
	 *
	 * Streak-and-hold-down design mirrors the hispeed and brutality
	 * tiers above (hispeed_entry_count / brutal_entry_count) so
	 * single-sample noise can't fire a rescue, and a fired rescue
	 * can't restack on the very next tick before the cpufreq driver
	 * has a chance to apply the previous request.
	 */
	if (z_policy->tunables->peak_headroom_rescue &&
	    max_cap && policy->max && !pin_to_target) {
		unsigned int load_pct = (util * 100) / max_cap;
		unsigned int starve_load =
			z_policy->tunables->peak_headroom_starve_load_pct;
		unsigned int floor_pct =
			z_policy->tunables->peak_headroom_freq_floor_pct;
		unsigned int streak =
			z_policy->tunables->peak_headroom_starve_streak;
		unsigned int jump_pct =
			z_policy->tunables->peak_headroom_jump_pct;
		unsigned int hold_ms =
			zenith_batt_scaled(
				z_policy->tunables->peak_headroom_hold_ms,
				z_policy->tunables->batt_hold_scale_pct);
		unsigned int floor_freq =
			(policy->max / 100) * floor_pct;
		bool starving = (load_pct >= starve_load) &&
				(freq < floor_freq);

		if (starving) {
			if (z_policy->peak_starve_count <
			    ZENITH_PEAK_HEADROOM_STREAK_MAX)
				z_policy->peak_starve_count++;
		} else {
			z_policy->peak_starve_count = 0;
		}

		if (z_policy->peak_starve_count > streak) {
			u64 now_ns = ktime_get_ns();

			if (now_ns >= z_policy->peak_rescue_until_ns) {
				unsigned int rescue_freq =
					(policy->max / 100) * jump_pct;

				if (rescue_freq > policy->max || !jump_pct)
					rescue_freq = policy->max;
				if (freq < rescue_freq) {
					freq = rescue_freq;
					tp_path = "peak_rescue";
					z_policy->peak_rescue_until_ns = now_ns +
						(u64)hold_ms * NSEC_PER_MSEC;
					/* Patch D: arm peer cluster.  Same
					 * now_ns the rescue computed its
					 * own hold-down off of, so the
					 * deadlines are aligned.
					 */
					zenith_peer_ramp_arm(z_policy, now_ns);
				}
			}
		}
	} else {
		/* Tunable disabled, max_cap == 0, or pin_to_target
		 * already covers the cluster: drop streak credit so we
		 * don't carry partial entry across a disable cycle or a
		 * boost-pin window.
		 */
		z_policy->peak_starve_count = 0;
	}

	/* 3. Powersave Bias.
	 *
	 * Only apply when current util is below bias_load_threshold so
	 * heavy work is not penalised. A threshold of 100 keeps the legacy
	 * "always-bias" behaviour; 0 disables the bias entirely without
	 * having to also write powersave_bias=0.
	 *
	 * 3a. Screen-on bias softening.  When the screen is on
	 * (tunables->screen_state == 1), scale dynamic_bias down by
	 * screen_on_bias_pct / 100.  See ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT
	 * for the rationale.  No-op when:
	 *   - screen_state != 1 (screen-off and uclamp-meaningful screen-
	 *     off paths are unaffected; the screen-off override at
	 *     dynamic_bias = 500 has already taken effect upstream).
	 *   - dynamic_bias == 0 (bias was already cleared upstream).
	 *   - screen_on_bias_pct >= 100 (no softening configured).
	 * The defensive >= 100 short-circuit also covers the case where
	 * userspace bypasses the sysfs store validator and writes a
	 * bogus value above 100.
	 */
	if (z_policy->tunables->screen_state == 1 && dynamic_bias &&
	    z_policy->tunables->screen_on_bias_pct < 100) {
		dynamic_bias = (dynamic_bias *
				z_policy->tunables->screen_on_bias_pct) / 100;
	}

	if (dynamic_bias && max_cap &&
	    (util * 100) / max_cap < z_policy->tunables->bias_load_threshold) {
		margin = freq * dynamic_bias / 1000;
		freq = freq - margin;
	}

	/* 3b. uclamp_min floor (ADPF PerformanceHint).
	 *
	 * After every other tier has had its say, guarantee that the
	 * chosen freq is at least what uclamp_min would imply via
	 * map_util_freq().  Catches three concrete cases schedutil's
	 * RQ-aggregate path alone would miss:
	 *
	 *   - light_load_freq cap undershooting an ADPF hint;
	 *   - powersave_bias shaving the proportional freq below the hint;
	 *   - cached_raw_freq short-circuit below returning a rate-limited
	 *     stale value while uclamp_min has been raised in between ticks.
	 *
	 * Applied regardless of screen_state: the boolean above has
	 * already picked whether the screen-off override suppression
	 * fires, but the floor is useful in screen-on paths too (e.g.
	 * a light-load cap would otherwise clip a meaningful hint).
	 */
	if (uclamp_min && max_cap) {
		unsigned int uclamp_floor = map_util_freq(uclamp_min,
							  policy->cpuinfo.max_freq,
							  max_cap);
		if (freq < uclamp_floor) {
			freq = uclamp_floor;
			tp_path = "uclamp_min_floor";
		}
	}

	/* 3c. Input-boost decay floor.  When a boost is in its trailing
	 * decay window, the ramp floor computed in step 0 overrides
	 * anything lower the eval tiers produced.  Applied last so it
	 * doesn't short-circuit the normal load-demand logic when the
	 * load genuinely calls for more than the ramp allows.
	 */
	if (input_boost_floor && freq < input_boost_floor) {
		freq = input_boost_floor;
		tp_path = "input_boost_decay";
	}

	/* 3c0. Boot-boost floor.  When boot_boost_ms != 0 and we are
	 * still inside the boot_boost window (measured from
	 * ktime_get_boottime_ns()), pin the freq to policy->max.  Gated
	 * by screen_state and reuses the input_boost_big_only topology
	 * gate so the small cluster does not wake to max during boot.
	 */
	if (z_policy->tunables->boot_boost_ms &&
	    z_policy->tunables->screen_state &&
	    (!z_policy->tunables->input_boost_big_only ||
	     z_policy->is_big_cluster)) {
		u64 boot_now = ktime_get_boottime_ns();
		u64 deadline_ns = (u64)z_policy->tunables->boot_boost_ms *
				  NSEC_PER_MSEC;

		/* boot_complete latch (sysfs-driven or in-kernel calm-detect).
		 * When raised, snap the deadline forward to the latch
		 * timestamp so the cluster transitions from Phase 1
		 * (pin to max) to Phase 2 (decay via boot_boost_decay_ms)
		 * immediately, instead of cliff-cutting to load-derived
		 * the way "write boot_boost_ms 0" would.
		 */
		if (atomic_read(&zenith_boot_complete)) {
			u64 latch_ns = READ_ONCE(zenith_boot_complete_ns);

			if (latch_ns && latch_ns < deadline_ns)
				deadline_ns = latch_ns;
		}

		if (boot_now < deadline_ns) {
			if (freq < policy->max) {
				freq = policy->max;
				tp_path = "boot_boost";
			}
		} else {
			/* Boot-boost decay tail.  Mirror of input_boost's
			 * decay phase but pinned to wall-clock boottime so
			 * the ramp shape is independent of how often
			 * zenith_get_next_freq() runs.  Linearly drops a
			 * floor from policy->max to policy->min across
			 * boot_boost_decay_ms after the hard pin window
			 * expires; eliminates the cliff that otherwise
			 * dumps boot freq from policy->max to load-derived
			 * the moment boot_boost_ms passes.
			 */
			unsigned int decay_ms = zenith_glide_value(z_policy,
				z_policy->tunables->boot_boost_decay_ms,
				z_policy->at_local_boot_boost_decay_ms);
			u64 decay_ns = (u64)decay_ms * NSEC_PER_MSEC;
			u64 elapsed_post = boot_now - deadline_ns;

			if (decay_ns && elapsed_post < decay_ns) {
				u64 span = policy->max - policy->min;
				u64 dropped = div64_u64(span * elapsed_post,
							decay_ns);
				unsigned int floor_freq;

				if (dropped >= span)
					floor_freq = policy->min;
				else
					floor_freq = policy->max -
						     (unsigned int)dropped;
				if (freq < floor_freq) {
					freq = floor_freq;
					tp_path = "boot_boost_decay";
				}
			}
		}
	}

	/* 3c''. Adaptive frame-budget floor.  Userspace-driven; sees
	 * the current vblank period via tunables->frame_budget_us and
	 * the calibrated 60 Hz baseline floor via
	 * tunables->frame_pace_floor_pct.  The effective floor scales
	 * inversely with the budget so 90 / 120 / 144 Hz refresh rates
	 * automatically lift the floor.  See the comment block at the
	 * top of the file for the formula.
	 */
	/* Read both tunables exactly once.  Without READ_ONCE the
	 * compiler is free to re-fetch frame_budget_us between the
	 * gating check and the divide below; if userspace writes 0 in
	 * that window the divide oopses.  Same reasoning for base_pct
	 * (a 0 here just skips the floor, but a torn read past the
	 * upper bound check could yield eff_pct overflow).
	 */
	{
		unsigned int anchor = cpumask_first(policy->cpus);
		unsigned int budget_us = (anchor < NR_CPUS) ?
			READ_ONCE(z_policy->tunables->
				frame_budget_us_per_policy[anchor]) : 0;
		unsigned int base_pct =
			zenith_tunable_or_local(z_policy,
				READ_ONCE(z_policy->tunables->frame_pace_floor_pct),
				z_policy->at_effective_frame_pace_floor_pct);

		/* Per-policy override of zero falls through to the
		 * global frame_budget_us.  See ZENITH_DEFAULT_FRAME_
		 * BUDGET_US per-policy comment block.
		 */
		if (!budget_us)
			budget_us =
			  READ_ONCE(z_policy->tunables->frame_budget_us);

		/* frame_budget_us_auto: when 1, prefer the drm-side
		 * cached vblank period over the userspace-set value.
		 * Drivers populate it via zenith_set_drm_vblank_us().
		 * The auto path falls back to budget_us silently when
		 * the cache is empty (e.g. boot before drm has made
		 * its first commit), so the floor still works on
		 * stock-tuned systems where userspace writes the rate.
		 */
		if (zenith_glide_value(z_policy,
				READ_ONCE(z_policy->tunables->
					  frame_budget_us_auto),
				z_policy->at_local_frame_budget_us_auto)) {
			unsigned int auto_us = (unsigned int)
				atomic_read(&zenith_drm_vblank_us);

			if (auto_us)
				budget_us = auto_us;
		}

		if (budget_us && base_pct) {
			unsigned int eff_pct;
			unsigned int fp_floor;

			eff_pct = (base_pct *
				   ZENITH_FRAME_PACE_BASE_BUDGET_US) /
				  budget_us;
			if (eff_pct > 100)
				eff_pct = 100;
			fp_floor = (policy->max * eff_pct) / 100;
			if (fp_floor > policy->max)
				fp_floor = policy->max;
			if (trace_zenith_frame_pace_enabled())
				trace_zenith_frame_pace(
					cpumask_first(policy->cpus),
					budget_us, eff_pct, fp_floor);
			if (freq < fp_floor) {
				freq = fp_floor;
				tp_path = "frame_pace";
			}
		}
	}

	/* 3c''. Audio low-jitter floor.  When audio_aware=1 and any CPU
	 * in this policy is currently running a known audio thread
	 * (AudioOut_*, audioserver, MediaCodec_*, ...), apply a freq
	 * floor of (policy->max * audio_floor_pct / 100).  Caches the
	 * comm walk for ZENITH_AUDIO_CACHE_TTL_NS.  The audio cap_pct
	 * companion is applied separately below, just before the
	 * uclamp_max final cap, so an explicit ADPF hint can still
	 * walk it down further.  audio_floor_pct=0 leaves the comm
	 * walk running (for tracepoint visibility) but applies no
	 * floor.
	 */
	if (ZENITH_FEATURE_ENABLED(audio_aware)) {
		bool has_audio = zenith_policy_has_audio(z_policy);
		unsigned int af_pct = z_policy->tunables->audio_floor_pct;
		unsigned int ac_pct = z_policy->tunables->audio_cap_pct;
		unsigned int af = af_pct ? (policy->max * af_pct) / 100 : 0;
		unsigned int ac = ac_pct ? (policy->max * ac_pct) / 100 : 0;

		if (af > policy->max)
			af = policy->max;
		if (ac > policy->max)
			ac = policy->max;
		if (trace_zenith_audio_band_enabled())
			trace_zenith_audio_band(
				cpumask_first(policy->cpus),
				has_audio, af_pct, ac_pct,
				has_audio ? af : 0,
				has_audio ? ac : 0);
		if (has_audio && af && freq < af) {
			freq = af;
			tp_path = "audio_floor";
		}
	}

	/* Wave A charger-aware floor.  When charger_aware == 1 AND the
	 * AC-vs-battery cache reports !on_battery, apply a freq floor of
	 * (policy->max * charger_floor_pct / 100).  The AC-vs-battery
	 * cache is updated lazily once per ZENITH_AUTO_TUNE_PERIOD by
	 * zenith_auto_tune_work() via power_supply_is_system_supplied(),
	 * so the floor follows the cable within roughly one auto_eval_ms
	 * window of plug / unplug.
	 *
	 * Both knobs default 0 so the tier is opt-in; thermal still wins
	 * downstream because auto_thermal_cap / thermal_state run after
	 * this site and can walk the floor back down if the SoC heats.
	 */
	if (z_policy->tunables->charger_aware &&
	    z_policy->tunables->charger_floor_pct &&
	    !atomic_read(&zenith_on_battery)) {
		unsigned int cf = (policy->max *
				   z_policy->tunables->charger_floor_pct) /
				  100;

		if (cf > policy->max)
			cf = policy->max;
		if (freq < cf) {
			freq = cf;
			tp_path = "charger_floor";
		}
	}

	/* 3c'. Render-thread / display-pipeline floor.  When
	 * render_aware=1 and any CPU in this policy is currently running
	 * a known render / display-pipeline thread (RenderThread,
	 * surfaceflinger, ...), apply a freq floor of
	 * (policy->max * render_floor_pct / 100).  Caches the comm walk
	 * for ZENITH_RENDER_CACHE_TTL_NS to keep the hot path cheap.
	 * Floor is still capped by the uclamp_max tier below.
	 *
	 * Patch B: debounce the floor by render_floor_min_runtime_ms.
	 * Stamp the first-seen time on the false->true transition, clear
	 * on any sample where has_render is false.  Apply the floor only
	 * when the render thread has been observed for at least the
	 * debounce window; this filters one-shot SurfaceFlinger flushes
	 * and idle RenderEngine wakes that would otherwise bounce the
	 * cluster up to render_floor_pct for a single sample.
	 */
	if (ZENITH_FEATURE_ENABLED(render_aware) &&
	    z_policy->tunables->render_floor_pct) {
		bool has_render = zenith_policy_has_render(z_policy);
		unsigned int rf = (policy->max *
				   z_policy->tunables->render_floor_pct) /
				  100;
		unsigned int debounce_ms =
			READ_ONCE(z_policy->tunables->render_floor_min_runtime_ms);
		bool debounce_ok = true;
		u64 now_ns_render;

		if (rf > policy->max)
			rf = policy->max;

		if (has_render) {
			now_ns_render = ktime_get_ns();
			if (z_policy->render_first_seen_ns == 0)
				z_policy->render_first_seen_ns = now_ns_render;
			if (debounce_ms) {
				u64 thresh = (u64)debounce_ms * NSEC_PER_MSEC;

				if (now_ns_render -
				    z_policy->render_first_seen_ns < thresh)
					debounce_ok = false;
			}
		} else {
			z_policy->render_first_seen_ns = 0;
		}

		if (trace_zenith_render_floor_enabled())
			trace_zenith_render_floor(
				cpumask_first(policy->cpus),
				has_render,
				z_policy->tunables->render_floor_pct,
				(has_render && debounce_ok) ? rf : 0);
		if (has_render && debounce_ok && freq < rf) {
			freq = rf;
			tp_path = "render_floor";
		}
	}

	/* Wave A render-thread util tracker.  More selective sibling
	 * of render_floor: applies a separate (typically higher) floor
	 * only when the matched render thread's PELT util_avg is at or
	 * above render_thread_util_thresh.  This filters out false
	 * positives where RenderThread is observed but is sitting idle
	 * in its main loop (paused video, static UI), where the
	 * unconditional render_floor over-floors.
	 *
	 * Reuses the zenith_policy_has_render() cache; the helper
	 * stores the matched task's util_avg in
	 * z_policy->render_matched_util_avg.  All three knobs default
	 * 0 so the tier is opt-in.  Requires render_aware=1 (otherwise
	 * the comm-walk does not run and util_avg is not observed).
	 */
	if (ZENITH_FEATURE_ENABLED(render_aware) &&
	    z_policy->tunables->render_thread_util_aware &&
	    z_policy->tunables->render_thread_util_thresh &&
	    z_policy->tunables->render_thread_util_floor_pct &&
	    zenith_policy_has_render(z_policy) &&
	    z_policy->render_matched_util_avg >=
	    z_policy->tunables->render_thread_util_thresh) {
		unsigned int rtuf =
			(policy->max *
			 z_policy->tunables->render_thread_util_floor_pct) /
			100;

		if (rtuf > policy->max)
			rtuf = policy->max;
		if (freq < rtuf) {
			freq = rtuf;
			tp_path = "render_thread_util_floor";
		}
	}

	/* Wave B PMU IPC tracker.  Apply a freq floor when measured
	 * IPC across the policy's CPUs is at or above pmu_ipc_thresh.
	 * The IPC cache is refreshed once per auto_tune window
	 * (zenith_pmu_sample_cpu() is called from zenith_auto_tune_-
	 * work()), so the floor follows workload-mix changes within
	 * roughly one auto_eval_ms after a transition.  Both knobs
	 * default 0 so the tier is opt-in; on CONFIG_PERF_EVENTS=n
	 * the helper returns a constant zero and the floor never
	 * applies.  See the comment block above ZENITH_DEFAULT_PMU_-
	 * AWARE for the full rationale.
	 */
	if (z_policy->tunables->pmu_aware &&
	    z_policy->tunables->pmu_ipc_thresh &&
	    z_policy->tunables->pmu_ipc_floor_pct &&
	    zenith_policy_max_ipc_pct(z_policy) >=
	    z_policy->tunables->pmu_ipc_thresh) {
		unsigned int pf =
			(policy->max *
			 z_policy->tunables->pmu_ipc_floor_pct) /
			100;

		if (pf > policy->max)
			pf = policy->max;
		if (freq < pf) {
			freq = pf;
			tp_path = "pmu_ipc_floor";
		}
	}

	/* Wave B EAS energy-knee floor.  When em_aware is on AND the
	 * cpufreq driver has registered an Energy Model for this
	 * policy AND em_floor_pct is non-zero, raise the freq floor to
	 * (em_knee_freq * em_floor_pct / 100).  The energy-knee is
	 * the OPP that minimises joules per instruction for sustained
	 * load -- below the knee, voltage scaling stops paying off
	 * and the policy wastes time without saving much energy.
	 * Floor still subject to the policy->max clamp downstream.
	 * See the comment block above ZENITH_DEFAULT_EM_AWARE for
	 * the full rationale.
	 */
	if (z_policy->tunables->em_aware &&
	    z_policy->tunables->em_floor_pct) {
		unsigned int knee = zenith_em_knee_freq(z_policy);

		if (knee) {
			unsigned int ef =
				(knee *
				 z_policy->tunables->em_floor_pct) /
				100;

			if (ef > policy->max)
				ef = policy->max;
			if (freq < ef) {
				freq = ef;
				tp_path = "em_floor";
			}
		}
	}

	/* Audit fix F3: V2-gated render-thread RT-priority floor.
	 *
	 * Per-policy uclamp-min-style boost.  When the tunable is enabled
	 * AND ZENITH_AT_FLAG_RENDER is currently active AND V2 has already
	 * committed to LATENCY or SUSTAINED_PERF, raise the freq floor to
	 * (policy->max * auto_tune_render_rt_floor_pct / 100).
	 *
	 * Logically a stricter sibling of render_floor_pct that fires only
	 * in the V2 states where a single CFS preemption of RenderThread
	 * is user-visible as a frame stutter.  See the
	 * ZENITH_DEFAULT_AT_RENDER_RT_FLOOR_PCT comment block for full
	 * semantics.  Default tunable is 0 (off); when enabled, this floor
	 * always wins over render_floor_pct because it is applied last in
	 * the floor chain.
	 */
	if (z_policy->tunables->auto_tune_render_rt_floor_pct &&
	    (z_policy->at_last_flags & ZENITH_AT_FLAG_RENDER) &&
	    (z_policy->at_last_state == ZENITH_AT_STATE_LATENCY ||
	     z_policy->at_last_state == ZENITH_AT_STATE_SUSTAINED_PERF)) {
		unsigned int rrf =
			(policy->max *
			 z_policy->tunables->auto_tune_render_rt_floor_pct) /
			100;

		if (rrf > policy->max)
			rrf = policy->max;
		if (freq < rrf) {
			freq = rrf;
			tp_path = "render_rt_floor";
		}
	}

	/* 3c''''. Camera capture-pipeline floor.  When camera_aware=1
	 * and either (camera_active=force-on) or (camera_active=auto
	 * && comm walk finds a known camera HAL/framework thread on
	 * the policy), apply a freq floor of
	 * (policy->max * camera_floor_pct / 100).  No companion cap:
	 * camera bursts benefit from full freq headroom; capping
	 * costs frame-rate.  Floor is still subject to the uclamp_max
	 * downstream cap.
	 *
	 * When camera_active=force-off, the comm walk is skipped and
	 * no floor is applied even if the table would have matched.
	 * The override exists because vendor camera HALs sometimes
	 * use thread names that don't match the table.
	 */
	if (ZENITH_FEATURE_ENABLED(camera_aware)) {
		unsigned int override = z_policy->tunables->camera_active;
		bool auto_match = false;
		bool active;
		unsigned int cf_pct = z_policy->tunables->camera_floor_pct;
		unsigned int cf;

		if (override == ZENITH_CAMERA_OVERRIDE_FORCE_OFF) {
			active = false;
		} else if (override == ZENITH_CAMERA_OVERRIDE_FORCE_ON) {
			active = true;
		} else {
			auto_match = zenith_policy_has_camera(z_policy);
			active = auto_match;
		}

		cf = cf_pct ? (policy->max * cf_pct) / 100 : 0;
		if (cf > policy->max)
			cf = policy->max;

		if (trace_zenith_camera_floor_enabled())
			trace_zenith_camera_floor(
				cpumask_first(policy->cpus),
				active, auto_match, override, cf_pct,
				active ? cf : 0);

		if (active && cf && freq < cf) {
			freq = cf;
			tp_path = "camera_floor";
		}
	}

	/* Wave A cgroup-aware top-app floor.  When top_app_aware == 1
	 * AND any CPU in the policy is currently running a task in the
	 * cpuset cgroup named "top-app", apply a freq floor of
	 * (policy->max * top_app_floor_pct / 100).  Cached for
	 * ZENITH_TOP_APP_CACHE_TTL_NS via
	 * zenith_policy_has_top_app() to keep the hot path cheap.
	 *
	 * Comm-walk floors (audio / render / camera / charger) all run
	 * before this site, so top_app_floor only fires when none of
	 * the more specific signals already lifted freq above
	 * top_app_floor_pct.  Both knobs default 0 so the tier is
	 * opt-in.  Requires CONFIG_CPUSETS=y; on CONFIG_CPUSETS=n the
	 * helper short-circuits to false and the floor never applies.
	 */
	if (z_policy->tunables->top_app_aware &&
	    z_policy->tunables->top_app_floor_pct &&
	    zenith_policy_has_top_app(z_policy)) {
		unsigned int taf = (policy->max *
				    z_policy->tunables->top_app_floor_pct) /
				   100;

		if (taf > policy->max)
			taf = policy->max;
		if (freq < taf) {
			freq = taf;
			tp_path = "top_app_floor";
		}
	}

	/* Patch K: game / sustained-high-load performance burst floor.
	 * When the FSM (evaluated up-stream from this function, near
	 * the game_auto tick) is ARMED, lift freq to the operator-
	 * configured floor.  COOLDOWN linearly steps the floor down to
	 * 0 across cooldown_ms.  zenith_gpb_floor() returns 0 when:
	 *   - master tunable is 0
	 *   - FSM is IDLE (no detected game / sustained load)
	 *   - thermal guardrail is engaged (skin temp >= ceiling)
	 * so the call is a no-op outside the active windows.  The
	 * pin_to_target gate skips the floor while a higher-priority
	 * pin (input boost / brutality) is in charge.
	 */
	if (!pin_to_target &&
	    ZENITH_FEATURE_ENABLED(game_perf_burst) &&
	    READ_ONCE(z_policy->tunables->game_perf_burst)) {
		unsigned int gpb = zenith_gpb_floor(z_policy, policy->max);

		if (gpb && freq < gpb) {
			freq = gpb;
			tp_path = "game_perf_burst_floor";
		}
	}

	/* Patch C6: SCHED_DEADLINE awareness floor.  Walks the
	 * policy CPU mask and looks for any rq with a non-zero
	 * dl.dl_nr_running.  When found, lift freq to
	 * (policy->max * dl_task_floor_pct / 100).  Race on
	 * dl_nr_running is harmless: read is a single load,
	 * decision is a heuristic responsiveness lift on top of
	 * the bandwidth math that schedutil_cpu_util already does
	 * for correctness.
	 *
	 * Tunable 0 (cold-boot default) disables; profile bakes
	 * default it on for PERFORMANCE / GAMING (100) and AUDIO
	 * (80).  pin_to_target skip avoids stacking under a
	 * higher pin from input_boost / brutality.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->dl_task_floor_pct) {
		int cpu_iter;
		bool dl_present = false;

		for_each_cpu(cpu_iter, policy->cpus) {
			if (READ_ONCE(cpu_rq(cpu_iter)->dl.dl_nr_running)) {
				dl_present = true;
				break;
			}
		}
		if (dl_present) {
			unsigned int dlf = (policy->max *
					    z_policy->tunables->dl_task_floor_pct) /
					   100;

			if (dlf > policy->max)
				dlf = policy->max;
			if (freq < dlf) {
				freq = dlf;
				tp_path = "dl_floor";
			}
		}
	}

	/* Patch C9: io_floor hysteresis sticky-floor.  When the
	 * iowait_boost path armed within the last io_floor_hyst_ms
	 * (zenith_iowait_boost() stamped io_floor_until_ns), lift
	 * freq to (policy->max * io_floor_hyst_pct / 100).  Both
	 * tunables 0 disables; pin_to_target paths skip (already
	 * pinned higher).
	 *
	 * Why this exists: iowait_boost decays to 0 within a few
	 * PELT periods after the last SCHED_CPUFREQ_IOWAIT, but
	 * sustained block IO (sqlite WAL replay, ext4 commit, media
	 * transcode) often goes through a small idle gap and then
	 * resumes; the boost has decayed and the level signal alone
	 * pins a low freq because the worker is mostly D-state.
	 * Holding the floor closes the gap.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->io_floor_hyst_ms &&
	    z_policy->tunables->io_floor_hyst_pct) {
		u64 until = z_policy->io_floor_until_ns;
		u64 now_ns = ktime_get_ns();

		if (until && now_ns < until) {
			unsigned int iof = (policy->max *
					    z_policy->tunables->io_floor_hyst_pct) /
					   100;

			if (iof > policy->max)
				iof = policy->max;
			if (freq < iof) {
				freq = iof;
				tp_path = "io_floor";
			}
		}
	}

	/* 3c'''''. Peer-ramp soft floor (Patch D).
	 *
	 * If the peer cluster (BIG <-> PRIME) ramped to peak in the
	 * recent past via predict_up / peak_prearm / peak_rescue, it
	 * stamped a deadline on this cluster's slot.  While that
	 * deadline has not expired, hold a soft floor at
	 * peer_ramp_floor_pct of policy->max so the IPC chain doesn't
	 * spend its first few samples stalled at idle freq waiting
	 * for the level signal to land.
	 *
	 * The atomic is a single ktime_get_ns() compare; both the
	 * window and floor knobs short-circuit when 0.  pin_to_target
	 * paths bypass: input_boost / brutality already pin the
	 * cluster higher, so layering a floor under them is wasted
	 * arithmetic.  LITTLE has no peer atomic so the lookup
	 * returns NULL and the tier is a single branch on that
	 * cluster.
	 */
	if (!pin_to_target && policy->max &&
	    zenith_peer_ramp_effective_window_ms(z_policy->tunables) &&
	    z_policy->tunables->peer_ramp_floor_pct) {
		atomic64_t *self =
			zenith_peer_ramp_self_atomic(z_policy->cluster_class);

		if (self) {
			u64 until = (u64)atomic64_read(self);
			u64 now_ns = ktime_get_ns();

			if (now_ns < until) {
				struct zenith_tunables *t = z_policy->tunables;
				unsigned int eff_pct = t->peer_ramp_floor_pct;
				unsigned int floor;

				/* Patch M2: optionally fold uclamp_min
				 * into the peer_ramp floor.  Helper returns
				 * 0 when no task has uclamp_min set, so
				 * the max() is a no-op in the common case.
				 */
				if (READ_ONCE(t->peer_ramp_uclamp_min_respect)) {
					unsigned int umin_pct =
						zenith_uclamp_min_pct_of_max(z_policy);

					if (umin_pct > eff_pct)
						eff_pct = umin_pct;
				}
				floor = (policy->max * eff_pct) / 100;
				if (floor > policy->max)
					floor = policy->max;
				if (freq < floor) {
					freq = floor;
					tp_path = "peer_ramp";
				}
			}
		}
	}

	/* 3c''''''-pre. Cluster-wake-pulse soft floor (Patch 1.3).
	 *
	 * Mirrors the migration_floor mechanic below: if the arm block
	 * at the top of zenith_get_next_freq() detected a >= cluster_-
	 * wake_pulse_idle_ms gap since the last eval, it stamped a
	 * deadline on cluster_wake_pulse_until_ns.  While that
	 * deadline has not expired, hold a soft floor at
	 * cluster_wake_pulse_floor_pct of policy->max so the freshly-
	 * woken cluster runs above PELT-cold-start freq for the
	 * pulse window.
	 *
	 * Bypassed when pin_to_target (input_boost / brutality already
	 * own freq above any plausible pulse floor) or when the floor
	 * percentage is 0 (knob stamps but suppresses, mirroring
	 * peer_ramp / migration_floor).  Runs before the migration_-
	 * floor tier so a cluster that wakes AND receives an inbound
	 * migrating task picks the higher of the two floors.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->cluster_wake_pulse_floor_pct) {
		u64 until = z_policy->cluster_wake_pulse_until_ns;

		if (until && ktime_get_ns() < until) {
			unsigned int floor =
				(policy->max / 100) *
				z_policy->tunables->cluster_wake_pulse_floor_pct;

			if (floor > policy->max)
				floor = policy->max;
			if (freq < floor) {
				freq = floor;
				tp_path = "cluster_wake_pulse";
			}
		}
	}

	/* 3c''''''-pre-fg. Foreground-transition pulse soft floor
	 * (Patch 1.9).  Mirrors the cluster_wake_pulse mechanic:
	 * the sched_wakeup_new probe stamps fg_transition_pulse_-
	 * until_ns whenever a foreground task is woken for the
	 * first time after fork() on a CPU belonging to this policy.
	 * While the deadline has not expired, hold a soft floor at
	 * fg_transition_pulse_pct of policy->max so the freshly-
	 * forked top-app task picks up above PELT cold-start freq.
	 *
	 * Bypassed when pin_to_target (input_boost / brutality
	 * already at or above any plausible pulse floor) or when
	 * the floor percentage is 0 (knob stamps but suppresses).
	 * Reads the deadline with READ_ONCE since the writer is the
	 * sched_wakeup_new probe in arbitrary scheduler context;
	 * see the comment block above zenith_probe_wakeup_new for
	 * the full safety argument.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->fg_transition_pulse_pct) {
		u64 until = READ_ONCE(z_policy->fg_transition_pulse_until_ns);

		if (until && ktime_get_ns() < until) {
			unsigned int floor =
				(policy->max / 100) *
				z_policy->tunables->fg_transition_pulse_pct;

			if (floor > policy->max)
				floor = policy->max;
			if (freq < floor) {
				freq = floor;
				tp_path = "fg_transition_pulse";
			}
		}
	}

	/* 3c''''''. Migration-arrival soft floor (Patch K1).
	 *
	 * If a per-CPU update_util tick observed a util jump
	 * exceeding migration_jump_pct of max_capacity, it stamped a
	 * deadline on this policy's migration_in_until_ns.  While
	 * that deadline has not expired, hold a soft floor at
	 * migration_floor_pct of policy->max so the destination
	 * cluster runs at a sensible freq during the inbound task's
	 * PELT warm-up rather than picking idle freq from the
	 * stale-low aggregate util signal.
	 *
	 * Same pin_to_target bypass as peer_ramp: input_boost /
	 * brutality already pin freq higher, so layering a soft
	 * floor under them is wasted arithmetic.  Both knobs
	 * short-circuit when 0 (jump_pct == 0 means stamping is also
	 * off; floor_pct == 0 keeps stamping but suppresses the
	 * floor here so userspace can correlate stats vs. effect).
	 */
	{
		/* Patch L: gate the K1 read through the V2 tier
		 * accessor.  In LATENCY / FRAME / GAME states the
		 * tier mask returns the profile-set jump_pct /
		 * floor_pct unchanged; in EFFICIENCY / BALANCED /
		 * THERMAL_RECOVERY states the V2 worker has cleared
		 * the migration tier bit and the accessor returns 0,
		 * which short-circuits the floor below.  User sysfs
		 * overrides bypass the V2 gate per zenith_tier_value().
		 */
		unsigned int eff_jump = zenith_tier_value(z_policy,
				z_policy->tunables->migration_jump_pct,
				ZENITH_AT_OVERRIDE_MIGRATION_JUMP,
				ZENITH_AT_TIER_MIGRATION);
		unsigned int eff_floor_pct = zenith_tier_value(z_policy,
				z_policy->tunables->migration_floor_pct,
				ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_PCT,
				ZENITH_AT_TIER_MIGRATION);

		if (!pin_to_target && policy->max && eff_jump &&
		    eff_floor_pct) {
			u64 until = z_policy->migration_in_until_ns;

			if (until && ktime_get_ns() < until) {
				struct zenith_tunables *t = z_policy->tunables;
				unsigned int eff_pct = eff_floor_pct;
				unsigned int floor;

				/* Patch M2: optionally fold uclamp_min into
				 * the migration_floor.  Same shape as the
				 * peer_ramp variant above; the helper
				 * returns 0 when no task on the policy has
				 * uclamp_min set, so max() is a no-op.
				 */
				if (READ_ONCE(t->migration_floor_uclamp_min_respect)) {
					unsigned int umin_pct =
						zenith_uclamp_min_pct_of_max(z_policy);

					if (umin_pct > eff_pct)
						eff_pct = umin_pct;
				}
				floor = (policy->max * eff_pct) / 100;
				if (floor > policy->max)
					floor = policy->max;
				if (freq < floor) {
					freq = floor;
					tp_path = "migration_floor";
				}
			}
		}
	}

	/* 3c'''''''. PSI-CPU sustained-pressure floor (Patch K2).
	 *
	 * When the system-wide PSI_CPU_SOME 10s EWMA is at or above
	 * tunables->psi_cpu_floor_thresh and a hispeed freq is
	 * configured, lift to it.  This addresses the workload class
	 * where aggregate util sits below hispeed-entry threshold
	 * but PSI shows lots of queueing -- multi-app multitasking,
	 * gaming + background sync, screen-record + foreground app
	 * -- which the existing util-driven tiers don't catch
	 * because util is "just running" rather than "running with
	 * waiters".
	 *
	 * Tier short-circuits in three places: feature-gate off,
	 * tunable == 0, eff_hispeed_freq() == 0.  pin_to_target
	 * paths bypass for the same reason as the other floor tiers
	 * (input_boost / brutality already pin higher).  The
	 * 10s-EWMA smoothing of the underlying signal is by design:
	 * predict_up / peak_rescue cover the sub-second case;
	 * this tier covers the steady-state queueing case.
	 */
	{
		/* Patch L: V2-gated K2 read.  Armed in LATENCY only
		 * (queueing during latency-sensitive workloads is
		 * exactly the case the floor was designed for); not
		 * armed during FRAME / GAME because the 10s EWMA
		 * smoothing would be a cross-talk signal during
		 * gameplay.
		 */
		unsigned int eff_thresh = zenith_tier_value(z_policy,
				z_policy->tunables->psi_cpu_floor_thresh,
				ZENITH_AT_OVERRIDE_PSI_CPU_FLOOR,
				ZENITH_AT_TIER_PSI_CPU_FLOOR);

		if (!pin_to_target &&
		    ZENITH_FEATURE_ENABLED(psi_aware) &&
		    eff_thresh) {
			if (zenith_psi_cpu_some_pct() >= eff_thresh) {
				unsigned int floor =
					zenith_eff_hispeed_freq(z_policy);

				if (floor && floor > policy->max)
					floor = policy->max;
				if (floor && freq < floor) {
					freq = floor;
					tp_path = "psi_cpu_floor";
				}
			}
		}
	}

	/* 3c''''''''. Frame-overrun rescue (Patch K3).
	 *
	 * If zenith_drm_vblank_event() observed a vblank gap wider
	 * than the configured budget, it stamped the governor-wide
	 * zenith_frame_overrun_until_ns deadline.  While that
	 * deadline holds, every cluster's eval lifts to a soft
	 * floor at frame_overrun_floor_pct of policy->max.
	 *
	 * Symmetric to peer_ramp -- the producer is governor-wide
	 * (frame events are observed at the display layer above
	 * the cluster partition), the read happens on every
	 * cluster's eval, and either of the per-policy knobs being
	 * 0 short-circuits the read.  The slack_us cache being 0
	 * means no overrun event ever stamped the deadline, so the
	 * read is effectively a no-op anyway -- the explicit
	 * floor_pct gate just avoids the atomic_read in that case.
	 */
	{
		/* Patch L: V2-gated K3 read.  Armed in FRAME / GAME
		 * states (where vblank-driven floors actually make
		 * sense); disarmed elsewhere.  Note the producer
		 * (zenith_drm_vblank_event()) is governor-wide and
		 * keeps stamping the deadline regardless -- the V2
		 * gate is only on the consumer side.  An overrun
		 * stamped during a non-FRAME state simply does not
		 * lift this cluster's freq for the deadline window.
		 */
		unsigned int eff_slack = zenith_tier_value(z_policy,
				z_policy->tunables->frame_overrun_slack_us,
				ZENITH_AT_OVERRIDE_FRAME_OVR_SLACK,
				ZENITH_AT_TIER_FRAME_OVERRUN);
		unsigned int eff_floor_pct = zenith_tier_value(z_policy,
				z_policy->tunables->frame_overrun_floor_pct,
				ZENITH_AT_OVERRIDE_FRAME_OVR_FLOOR,
				ZENITH_AT_TIER_FRAME_OVERRUN);

		if (!pin_to_target && policy->max && eff_slack &&
		    eff_floor_pct) {
			u64 until =
			      (u64)atomic64_read(&zenith_frame_overrun_until_ns);

			if (until && ktime_get_ns() < until) {
				unsigned int floor_pct = eff_floor_pct;
				unsigned int deep_streak =
					READ_ONCE(z_policy->tunables->frame_overrun_deep_streak);
				unsigned int floor;

				/* Patch M5: deep tier.  After deep_streak
				 * consecutive overruns the floor escalates
				 * to frame_overrun_deep_floor_pct (default
				 * 100%).  deep_streak == 0 short-circuits
				 * the comparison and the streak atomic is
				 * never read.  The lower bound of max(...)
				 * with eff_floor_pct guarantees the deep
				 * tier never produces a *lower* floor than
				 * the standard K3 floor would.
				 */
				if (deep_streak &&
				    (unsigned int)atomic_read(&zenith_frame_overrun_streak) >=
				    deep_streak) {
					unsigned int deep_pct = READ_ONCE(
						z_policy->tunables->frame_overrun_deep_floor_pct);

					if (deep_pct > floor_pct)
						floor_pct = deep_pct;
				}
				floor = (policy->max * floor_pct) / 100;
				if (floor > policy->max)
					floor = policy->max;
				if (freq < floor) {
					freq = floor;
					tp_path = "frame_overrun";
				}
			}
		}
	}

	/* 3c'''. Audio low-jitter cap.  Companion to the audio floor
	 * tier above: when audio_aware=1, an audio thread is enqueued
	 * on the policy, and audio_cap_pct > 0, cap freq at
	 * (policy->max * audio_cap_pct / 100).  Applied after every
	 * floor tier so it can pull the freq down even when render_floor
	 * / boot_boost / input_boost would otherwise hold it higher.
	 * Applied *before* the uclamp_max final cap so an explicit ADPF
	 * power-efficiency hint can still walk it down further.  When
	 * the user sets audio_cap_pct < audio_floor_pct, the cap takes
	 * precedence (the floor block ran earlier; this block then
	 * pulls back).
	 */
	if (ZENITH_FEATURE_ENABLED(audio_aware) &&
	    z_policy->tunables->audio_cap_pct) {
		unsigned int ac = (policy->max *
				   z_policy->tunables->audio_cap_pct) / 100;

		if (ac > policy->max)
			ac = policy->max;
		if (z_policy->audio_active && freq > ac) {
			freq = ac;
			tp_path = "audio_cap";
		}
	}

apply_uclamp_max_cap:
	/* 3d. uclamp_max final-freq cap.  Applied after every other tier
	 * so that brutality snap, hispeed floor, input boost, and the
	 * uclamp_min / input_boost_decay floors can't walk over an
	 * explicit power-efficiency hint.  A uclamp_min floor higher
	 * than the uclamp_max cap wins by construction (floor applies
	 * first, cap would clamp it down below uclamp_min only when the
	 * two hints disagree, and per-task uclamp validation already
	 * prevents that at the scheduler layer).
	 *
	 * Reachable both from the natural fall-through path AND from
	 * the input_boost / brutality "goto" sites, so an ADPF
	 * uclamp_max hint can walk down even an explicitly-pinned
	 * boost target (the user's "save power on this thread"
	 * intent should beat the governor's "this is interactive"
	 * heuristic).
	 */
	if (uclamp_max < SCHED_CAPACITY_SCALE && max_cap) {
		unsigned int uclamp_cap = map_util_freq(uclamp_max,
							policy->cpuinfo.max_freq,
							max_cap);
		if (freq > uclamp_cap) {
			freq = uclamp_cap;
			tp_path = "uclamp_max_cap";
		}
	}

	/* 3e'. PSI-mem light cap (Patch M1).  Sits one tier above the
	 * existing PSI-cap-at-hispeed (3e below).  When psi_aware=1
	 * AND the V2 PSI_MEM_CAP tier is armed (or the user has
	 * sysfs-overridden the thresh) AND
	 * zenith_psi_mem_some_pct() >= psi_mem_cap_thresh, stamp the
	 * per-policy psi_mem_cap_until_ns deadline.  While the
	 * deadline holds, cap final freq at psi_mem_cap_pct of
	 * policy->max.  Once the EWMA falls and the window lapses,
	 * the cap releases without further hysteresis.
	 *
	 * Cheap when off: psi_aware == 0 short-circuits via
	 * ZENITH_FEATURE_ENABLED; eff_thresh == 0 from V2 disarm
	 * short-circuits the EWMA read; pin_to_target paths skip
	 * the entire block.  The deadline read is a single field
	 * compare per eval, identical pattern to migration_floor.
	 */
	{
		unsigned int eff_thresh = zenith_tier_value(z_policy,
				z_policy->tunables->psi_mem_cap_thresh,
				ZENITH_AT_OVERRIDE_PSI_MEM_CAP_THRESH,
				ZENITH_AT_TIER_PSI_MEM_CAP);

		if (!pin_to_target &&
		    ZENITH_FEATURE_ENABLED(psi_aware) &&
		    eff_thresh && policy->max) {
			unsigned int cap_pct = READ_ONCE(
				z_policy->tunables->psi_mem_cap_pct);
			unsigned int win_ms = READ_ONCE(
				z_policy->tunables->psi_mem_cap_window_ms);
			u64 now_ns = ktime_get_ns();

			/* Arm path: stamp the deadline whenever the
			 * EWMA is currently above thresh.  Subsequent
			 * ticks while the EWMA stays above thresh keep
			 * pushing the deadline forward; once it drops,
			 * the deadline holds the cap until win_ms has
			 * lapsed.  Stamping before the cap apply means
			 * a cap that just released and a new spike both
			 * land cleanly.
			 */
			if (zenith_psi_mem_some_pct() >= eff_thresh)
				z_policy->psi_mem_cap_until_ns =
					now_ns + (u64)win_ms * NSEC_PER_MSEC;

			if (cap_pct >= ZENITH_PSI_MEM_CAP_PCT_MIN &&
			    z_policy->psi_mem_cap_until_ns &&
			    now_ns < z_policy->psi_mem_cap_until_ns) {
				unsigned int psi_cap =
					(policy->max * cap_pct) / 100;

				if (psi_cap < policy->min)
					psi_cap = policy->min;
				if (psi_cap > policy->max)
					psi_cap = policy->max;
				if (freq > psi_cap) {
					freq = psi_cap;
					tp_path = "psi_mem_cap_light";
				}
			}
		}
	}

	/* 3e. PSI memory-pressure cap.  When psi_aware=1 and the system
	 * is over the configured 10s memory-pressure threshold, cap the
	 * final freq at the effective hispeed floor (or policy->max as
	 * fallback when the hispeed tier is disabled).  Rationale: under
	 * heavy memstall, going above hispeed mostly burns energy on
	 * cycles that stall waiting for memory.  Boot-boost (3c0) sits
	 * higher in the chain so the boot window is preserved even with
	 * psi_aware=1.  pin_to_target=true (input_boost / brutality)
	 * skips the PSI cap so a touch-driven boost wins even under
	 * memstall; the uclamp_max cap above is still authoritative.
	 */
	if (!pin_to_target &&
	    ZENITH_FEATURE_ENABLED(psi_aware) &&
	    (z_policy->tunables->psi_mem_thresh ||
	     z_policy->tunables->psi_cpu_thresh ||
	     z_policy->tunables->psi_io_thresh)) {
		const char *psi_tag = NULL;

		if (z_policy->tunables->psi_mem_thresh &&
		    zenith_psi_mem_some_pct() >=
		    z_policy->tunables->psi_mem_thresh)
			psi_tag = "psi_mem_cap";
		else if (z_policy->tunables->psi_cpu_thresh &&
			 zenith_psi_cpu_some_pct() >=
			 z_policy->tunables->psi_cpu_thresh)
			psi_tag = "psi_cpu_cap";
		else if (z_policy->tunables->psi_io_thresh &&
			 zenith_psi_io_some_pct() >=
			 z_policy->tunables->psi_io_thresh)
			psi_tag = "psi_io_cap";

		if (psi_tag) {
			unsigned int psi_cap =
				zenith_eff_hispeed_freq(z_policy);

			if (!psi_cap)
				psi_cap = policy->max;
			if (freq > psi_cap) {
				freq = psi_cap;
				tp_path = psi_tag;
			}
		}
	}

	/* 3f. Quiet-hours cap (Patch 1.10).
	 *
	 * Hard freq cap inside the user-configured nightly window.
	 * pin_to_target tiers (input_boost / brutality) bypass the
	 * cap so a user explicitly poking the device mid-window still
	 * gets full responsiveness; passive evaluation is what gets
	 * throttled.  The screen_off_only gate is the second guard:
	 * with the default of 1, the cap only fires while
	 * tunables->screen_state == 0, so a quiet-hours window that
	 * accidentally overlaps an active call or alarm doesn't drag
	 * the cluster down.
	 *
	 * cap_pct is bounded floor 50 % (sysfs); cap_pct == 100 makes
	 * the tier a no-op and is the default, so without an explicit
	 * profile / sysfs override the tier is invisible.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->quiet_hours_cap_pct &&
	    z_policy->tunables->quiet_hours_cap_pct < 100 &&
	    (!z_policy->tunables->quiet_hours_screen_off_only ||
	     !READ_ONCE(z_policy->tunables->screen_state)) &&
	    zenith_in_quiet_hours(z_policy->tunables)) {
		unsigned int qh_cap = (policy->max / 100) *
			z_policy->tunables->quiet_hours_cap_pct;

		if (qh_cap < policy->min)
			qh_cap = policy->min;
		if (qh_cap > policy->max)
			qh_cap = policy->max;
		if (freq > qh_cap) {
			freq = qh_cap;
			tp_path = "quiet_hours_cap";
		}
	}

	/* X. Peak-return hysteresis (Patch E).
	 *
	 * If the previous evaluation pinned the cluster at peak class
	 * (cached_raw_freq >= ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT
	 * of policy->max) and the current freq wants to drop below the
	 * soft floor (prev * peak_step_down_pct / 100), hold the soft
	 * floor for the next peak_hysteresis_streak samples then
	 * release.  pin_to_target paths (input_boost full-pin,
	 * brutality, climb_step) bypass this tier so the user-
	 * experience tier wins.
	 *
	 * Cost in the disabled path is one branch; in the enabled
	 * path it's a few unsigned multiplies and one streak compare.
	 */
	freq = zenith_apply_peak_hysteresis(z_policy, policy, freq,
					    pin_to_target, &tp_path);

	/* Patch H: sleeper-tail shaving.  When the cluster has been
	 * idle for at least sleeper_tail_thresh_us microseconds and
	 * the freq we'd otherwise pick is above policy->min, shave
	 * the freq by sleeper_tail_pct / 100 (clamped at
	 * policy->min).  Bypassed when pin_to_target is set so user-
	 * experience boosts always win.
	 *
	 * thresh_us == 0 disables the tier; pct == 100 makes the
	 * shave a no-op so we early-out to avoid the multiply on
	 * the common case.
	 */
	if (!pin_to_target && freq > policy->min) {
		unsigned int thresh_us =
			READ_ONCE(z_policy->tunables->sleeper_tail_thresh_us);
		unsigned int shave_pct =
			READ_ONCE(z_policy->tunables->sleeper_tail_pct);

		if (thresh_us && shave_pct &&
		    shave_pct < ZENITH_SLEEPER_TAIL_PCT_MAX) {
			u64 now_ns = ktime_get_ns();
			u64 idle_ns = now_ns - z_policy->last_runnable_ns;
			u64 thresh_ns = (u64)thresh_us * NSEC_PER_USEC;

			if (idle_ns >= thresh_ns) {
				unsigned int shaved =
					(freq / 100) * shave_pct;

				if (shaved < policy->min)
					shaved = policy->min;
				if (shaved < freq) {
					freq = shaved;
					tp_path = "sleeper_tail";
				}
			}
		}
	}

	/* cached_raw_freq shortcut: when the pre-resolve freq matches
	 * the value we cached on the previous tick AND nothing has
	 * marked need_freq_update, the post-resolve tiers (ladder,
	 * light_cap, EM, sampling-down) all produced the same answer
	 * last tick, so we can return the cached final value.
	 *
	 * EXCEPTION: if the efficient_freq ladder has any armed bin
	 * deadline (eff_unlock_at_ns[i] != 0), we MUST run the ladder
	 * loop again so deadlines can release on schedule.  Otherwise a
	 * sustained sub-up_threshold load that holds freq steady would
	 * latch the ladder at its current bin forever, because the
	 * cache hit short-circuits past the loop that consumes them.
	 */
	if (freq == z_policy->cached_raw_freq && !z_policy->need_freq_update &&
	    !zenith_ladder_pending(z_policy)) {
		z_policy->stats[ZENITH_STAT_DECISIONS]++;
		z_policy->stats[ZENITH_STAT_CACHE_HITS]++;
		/* Patch J: stamp the per-policy last decision tag on
		 * the cache-hit path too so the sysfs node reflects
		 * the most recent path even when the cache shortcut
		 * wins.  WRITE_ONCE pairs with READ_ONCE in the show
		 * handler.
		 */
		WRITE_ONCE(z_policy->last_decision_path, tp_path);
		/* Emit the same summary tracepoint on cache-hit so a
		 * trace consumer sees a continuous record of decisions
		 * rather than gaps every time the cache shortcut wins.
		 * Gated on trace_zenith_decision_enabled() so the cache
		 * hot path stays free when tracing is off.
		 */
		if (trace_zenith_decision_enabled()) {
			unsigned int kc_pct = per_cpu(zenith_cpu, policy->cpu)
						.kc_filtered_busy_pct;
			trace_zenith_decision(policy->cpu, tp_path, util,
					      max_cap, tp_load_pct, freq,
					      z_policy->next_freq, kc_pct,
					      z_policy->cached_uclamp_min,
					      z_policy->cached_uclamp_max,
					      true);
		}
		/*
		 * Hikari floor application on the early-return path.
		 * Raises the cached freq if Hikari has a published
		 * wake-demand floor that exceeds it.  No-op when no
		 * floor is published or when Hikari is off.
		 */
		{
			unsigned int hf = zenith_hikari_policy_floor(policy);

			if (hf && hf > z_policy->next_freq)
				return hf;
		}
		return z_policy->next_freq;
	}

	z_policy->cached_raw_freq = freq;
	{
		unsigned int _vh_idx, _vh_l_freq, _vh_h_freq;

		_vh_l_freq = cpufreq_driver_resolve_freq(policy, freq);
		_vh_idx = cpufreq_frequency_table_target(policy, freq,
							 CPUFREQ_RELATION_H);
		_vh_h_freq = policy->freq_table[_vh_idx].frequency;
		_vh_h_freq = clamp(_vh_h_freq, policy->min, policy->max);
		if (_vh_l_freq <= _vh_h_freq || _vh_l_freq == policy->min)
			target_freq = _vh_l_freq;
		else if (mult_frac(100, freq - _vh_h_freq,
				   _vh_l_freq - _vh_h_freq) < 20)
			target_freq = _vh_h_freq;
		else
			target_freq = _vh_l_freq;
	}

	/* 4. Efficient-frequency ladder (soft cap).
	 *
	 * The ladder is an array of (efficient_freq, up_delay_us) pairs
	 * sorted ascending by freq. For each bin i that the requested
	 * target_freq wants to cross, the governor holds at eff_freq[i]
	 * until the request has been sustained for eff_delay_us[i]. If
	 * target drops back below eff_freq[i] the corresponding
	 * deadline is cleared, so transient bursts do not accumulate
	 * climbing progress.
	 *
	 * eff_nr == 0 disables the ladder (identical to pre-ladder
	 * efficient_freq=0).  pin_to_target=true (input_boost full-pin,
	 * brutality snap_max / brutal_hold, climb_step) skips the
	 * ladder so the user-experience tier wins.
	 */
	if (!pin_to_target && z_policy->tunables->eff_nr) {
		unsigned int nr = z_policy->tunables->eff_nr;
		u64 now = ktime_get_ns();
		int i;

		if (nr > ZENITH_EFF_BINS_MAX)
			nr = ZENITH_EFF_BINS_MAX;

		for (i = 0; i < nr; i++) {
			unsigned int bin_freq = z_policy->tunables->eff_freq[i];
			u64 delay_ns = (u64)z_policy->tunables->eff_delay_us[i] *
				       NSEC_PER_USEC;

			/* Clamp the bin to this policy's max.  The
			 * efficient_freq table is set on the global
			 * tunables and may serve multiple policies with
			 * different policy->max values.  Bins above
			 * policy->max would otherwise be permanently
			 * unreachable (target_freq is already bounded
			 * by policy->max upstream), silently disabling
			 * the upper rungs of the ladder for the smaller
			 * cluster.  Collapsing to policy->max gives the
			 * operator the intuitive behaviour: "the table
			 * extends through this cluster's max".
			 */
			if (bin_freq > policy->max)
				bin_freq = policy->max;

			if (target_freq <= bin_freq) {
				/* Target is at or below this bin.  With
				 * hysteresis enabled, only release the
				 * bin (clear its and every higher bin's
				 * wait-deadline) if target dropped past
				 * bin_freq * (100 - hyst_pct) / 100.
				 * Otherwise the deadline is preserved
				 * but target_freq is left as-is, so a
				 * re-cross of bin_freq doesn't have to
				 * re-arm the bin.  See ZENITH_DEFAULT_
				 * EFF_BIN_HYST_PCT for the why.
				 */
				unsigned int hyst = READ_ONCE(
					z_policy->tunables->eff_bin_hyst_pct);

				if (hyst) {
					unsigned int margin =
						(bin_freq * hyst) / 100;
					unsigned int release = bin_freq -
						margin;

					if (target_freq > release)
						break;
				}
				{
					int j;

					for (j = i; j < nr; j++)
						z_policy->eff_unlock_at_ns[j] = 0;
				}
				break;
			}

			/* Target wants to cross bin i. Arm its deadline
			 * on first sight, hold at bin_freq until the
			 * sustained time expires.
			 */
			if (!z_policy->eff_unlock_at_ns[i]) {
				z_policy->eff_unlock_at_ns[i] = now + delay_ns;
				target_freq = bin_freq;
				break;
			}
			if (now < z_policy->eff_unlock_at_ns[i]) {
				target_freq = bin_freq;
				break;
			}
			/* Bin already unlocked; try the next one. */
		}
	}

	/* 5. Light-load hard cap.
	 *
	 * Independent of the efficient_freq soft cap (which gates climbs):
	 * when current util is below light_load_threshold, hard-clamp the
	 * resolved target_freq down to light_load_freq. Saves power on
	 * idle-ish workloads (background sync, screen-on hold) where PELT
	 * jitter would otherwise push us into a mid bin we do not need.
	 * pin_to_target=true skips this cap for the same reason as the
	 * efficient-freq ladder above.
	 */
	if (!pin_to_target &&
	    z_policy->tunables->light_load_freq && max_cap &&
	    (util * 100) / max_cap < z_policy->tunables->light_load_threshold &&
	    target_freq > z_policy->tunables->light_load_freq) {
		target_freq = z_policy->tunables->light_load_freq;
		tp_path = "light_cap";
	}

	/* 6. Energy Model Validation */
	{
		unsigned int em_in = target_freq;

		target_freq = zenith_em_cap_freq(z_policy, target_freq);
		if (target_freq != em_in)
			tp_path = "em_cap";
	}

	/* 6a. auto_thermal_cap (Path B): final-stage hard cap on
	 * target_freq when sustained thermal pressure exceeds the
	 * configured threshold.  Default off (auto_thermal_cap == 0);
	 * a single READ_ONCE short-circuits the pressure read on the
	 * fast path.  Applied AFTER em_cap so the energy model has
	 * already validated the freq; this tier just clamps the upper
	 * bound to policy->max * auto_thermal_cap_freq_pct / 100 when
	 * the pressure threshold is met.  See ZENITH_DEFAULT_AUTO_-
	 * THERMAL_CAP comment block.  Wrapped by the thermal_aware
	 * master gate (static-key) so the entire pressure read +
	 * threshold compare folds away when the master gate is off.
	 */
	if (ZENITH_FEATURE_ENABLED(thermal_aware) &&
	    READ_ONCE(z_policy->tunables->auto_thermal_cap)) {
		unsigned int p_pct =
			zenith_policy_thermal_pressure_pct(z_policy);
		unsigned int thresh = READ_ONCE(z_policy->tunables->
					auto_thermal_cap_pressure_pct);
		unsigned int cap_pct = READ_ONCE(z_policy->tunables->
					auto_thermal_cap_freq_pct);

		if (p_pct >= thresh && cap_pct < 100) {
			unsigned long cap_freq =
				((unsigned long)policy->max * cap_pct) /
				100UL;

			if (cap_freq && target_freq > cap_freq) {
				target_freq = cap_freq;
				tp_path = "auto_thermal_cap";
			}
		}
	}

	/* 7. Sampling-down multiplier: extend the down-rate delay by
	 * sampling_down_factor while we are either (a) sitting at
	 * policy->max, or (b) within one stretched down-rate window of
	 * a recently-exited input boost (boost_exit_extend).  The latter
	 * smooths the gesture tail: after the input-boost full-pin phase
	 * ends, normal eval may briefly pick a much lower freq while
	 * PELT catches up to the post-boost workload, and dropping the
	 * sampling multiplier the instant target falls below max
	 * produces a perceptible undershoot.  Reset the multiplier (and
	 * clear the latch) the moment both conditions are false.
	 */
	{
		unsigned int sdf = max(z_policy->tunables->sampling_down_factor,
					1U);
		bool boost_exit_active = false;

		if (z_policy->tunables->boost_exit_extend &&
		    z_policy->boost_active_until_ns) {
			u64 now_ns = ktime_get_ns();
			u64 stretch_ns = (u64)z_policy->tunables->down_rate_limit_us *
					 NSEC_PER_USEC * sdf;

			if (now_ns < z_policy->boost_active_until_ns +
				     stretch_ns)
				boost_exit_active = true;
			else
				z_policy->boost_active_until_ns = 0;
		}

		if (target_freq >= policy->max || boost_exit_active)
			z_policy->down_rate_mult = sdf;
		else
			z_policy->down_rate_mult = 1;
	}

	if (trace_zenith_decision_enabled()) {
		/* Report the leader CPU's filtered kcpustat busy% so a
		 * trace consumer can tell at a glance whether the
		 * decision was lifted by the kcpustat blend.  Reads 0
		 * when the feature is off (sampler is gated by
		 * kcpustat_hispeed_enable in update_util) or when no
		 * recent activity has populated the sampler.
		 */
		unsigned int kc_pct =
			per_cpu(zenith_cpu, policy->cpu).kc_filtered_busy_pct;

		trace_zenith_decision(policy->cpu, tp_path, util, max_cap,
				      tp_load_pct, freq, target_freq, kc_pct,
				      z_policy->cached_uclamp_min,
				      z_policy->cached_uclamp_max,
				      false);
	}

	z_policy->stats[ZENITH_STAT_DECISIONS]++;
	z_policy->stats[zenith_path_to_bucket(tp_path)]++;

	/* Patch 1.4: decision-latency histogram.  Single ktime read
	 * + subtract + bucket increment.  3 fixed thresholds (10 us
	 * / 50 us / 100 us) cover the practical range of zenith eval
	 * costs on a 5.10 kernel; an eval that lands above 100 us is
	 * already pathological and the >=100us bucket is intentionally
	 * a fire-and-forget marker for those.
	 *
	 * Patch B7-2 piggy-backs on the same lat_ns sample to feed
	 * the decision-ring entry below; pulled out of the inner
	 * scope so dec_ring stamping can reuse it without re-reading
	 * ktime_get_ns().
	 */
	{
		u64 lat_ns = ktime_get_ns() - dec_eval_start_ns;
		unsigned int head;

		if (lat_ns < 10000ULL)
			z_policy->dec_lat_buckets[0]++;
		else if (lat_ns < 50000ULL)
			z_policy->dec_lat_buckets[1]++;
		else if (lat_ns < 100000ULL)
			z_policy->dec_lat_buckets[2]++;
		else
			z_policy->dec_lat_buckets[3]++;

		/* Patch B7-2: append (path, lat_ns) to the per-policy
		 * decision ring.  Clamp lat_ns to u32_max so the field
		 * truncation matches the storage type without wrapping.
		 * head advances under the same update_lock that gates
		 * this whole tail; readers use READ_ONCE on path and
		 * accept the corresponding lat_ns torn-write window
		 * (worst case: a brief mismatch resolved on the next
		 * read, perfectly fine for an observability ring).
		 */
		head = z_policy->dec_ring_head & ZENITH_DEC_RING_MASK;
		z_policy->dec_ring[head].lat_ns =
			(lat_ns > U32_MAX) ? U32_MAX : (u32)lat_ns;
		WRITE_ONCE(z_policy->dec_ring[head].path, tp_path);
		WRITE_ONCE(z_policy->dec_ring_head, head + 1);
	}
	/* Patch J: stamp the per-policy last decision tag.  Pairs
	 * with READ_ONCE in the last_decision_path sysfs handler.
	 */
	WRITE_ONCE(z_policy->last_decision_path, tp_path);

	/* Update the variance EWMA used by the up_threshold_adaptive
	 * shaping at the top of the next eval.  Uses tp_load_pct as the
	 * input signal (computed earlier in this function).  Cheap: one
	 * abs-diff, one shift, one add.  See ZENITH_DEFAULT_UP_THRESHOLD_
	 * ADAPTIVE for what consumes load_var_ewma_x256.  Updated even
	 * when up_threshold_adaptive is 0 so flipping the tunable on
	 * doesn't see a stale-zero variance for the first 8 samples.
	 */
	{
		unsigned int prev = z_policy->last_load_pct;
		unsigned int delta = tp_load_pct > prev ?
				tp_load_pct - prev : prev - tp_load_pct;
		z_policy->load_var_ewma_x256 =
			(z_policy->load_var_ewma_x256 * 7 + delta * 256) / 8;
		z_policy->last_load_pct = tp_load_pct;
	}

	/* Push the current util sample into the predict_up trend ring
	 * so the next eval can compare a window of samples and decide
	 * whether to fire tier 2a'.  Done unconditionally (not gated
	 * on tunables->predict_up_thresh) so flipping the tunable on
	 * after a quiet period doesn't observe a ring of unwritten
	 * zeroes.  util_history_count saturates at the ring size so
	 * the value is a clean "are we warmed up" gate.
	 */
	{
		unsigned int idx = z_policy->util_history_idx;

		z_policy->util_history[idx] = util;
		z_policy->util_history_idx =
			(idx + 1) % ZENITH_PREDICT_UP_WINDOW_MAX;
		if (z_policy->util_history_count <
		    ZENITH_PREDICT_UP_WINDOW_MAX)
			z_policy->util_history_count++;
	}

	/*
	 * Hikari floor application on the main return path.  Raises
	 * target_freq if Hikari has a published wake-demand floor
	 * that exceeds it.  No-op when no floor is published or when
	 * Hikari is off.  Applied AFTER all in-governor decision
	 * tiers so the floor acts as a hard, additive lower bound on
	 * the wake-time freq -- it can lift Zenith's choice but
	 * never lower it.
	 */
	{
		unsigned int hf = zenith_hikari_policy_floor(policy);

		if (hf && hf > target_freq)
			target_freq = hf;
	}

	return target_freq;
}

static void zenith_execute_switch(struct zenith_policy *z_policy, u64 time, unsigned int next_freq)
{
	if (z_policy->need_freq_update) {
		z_policy->need_freq_update = false;
		if (z_policy->next_freq == next_freq &&
		    !cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS))
			return;
	} else if (z_policy->next_freq == next_freq) {
		return;
	}

	if (next_freq < z_policy->next_freq) {
		unsigned int margin_pct =
			READ_ONCE(z_policy->tunables->freq_stability_margin_pct);

		if (margin_pct) {
			unsigned int margin;

			if (margin_pct > ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX)
				margin_pct = ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX;
			margin = (z_policy->policy->max * margin_pct) / 100;
			if (z_policy->next_freq - next_freq <= margin)
				return;
		}
	}

	if (zenith_up_down_rate_limit(z_policy, time, next_freq))
		return;

	z_policy->next_freq = next_freq;
	z_policy->last_freq_update_time = time;

	if (z_policy->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(z_policy->policy, next_freq);
	} else if (!z_policy->work_in_progress) {
		z_policy->work_in_progress = true;
		irq_work_queue(&z_policy->irq_work);
	}
}

/************************ Scheduler Hooks ***********************/

static void zenith_update_single(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct zenith_cpu *z_cpu = container_of(hook, struct zenith_cpu, update_util);
	struct zenith_policy *z_policy = z_cpu->z_policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long util, max_cap;
	unsigned int next_f;

	zenith_iowait_boost(z_cpu, time, flags, tunables->io_is_busy);
	z_cpu->last_update = time;

	zenith_ignore_dl_rate_limit(z_cpu, z_policy);

	if (!zenith_should_update_freq(z_policy, time))
		return;

	util = zenith_get_util(z_cpu);
	max_cap = z_cpu->max_capacity;

	util = zenith_iowait_apply(z_cpu, time, util, max_cap);
	zenith_migration_arrival_check(z_cpu, util, max_cap, z_policy);
	if (READ_ONCE(tunables->wakeup_boost) && max_cap) {
		unsigned int cur_pct = (unsigned int)((util * 100) / max_cap);
		unsigned int prev_pct = z_cpu->wakeup_prev_util ?
			(unsigned int)((z_cpu->wakeup_prev_util * 100) /
				       max_cap) : 0;

		if (prev_pct < ZENITH_WAKEUP_IDLE_THRESH_PCT &&
		    cur_pct >= ZENITH_WAKEUP_BUSY_THRESH_PCT) {
			unsigned int ms = zenith_glide_value(z_policy,
				READ_ONCE(tunables->wakeup_boost_ms),
				z_policy->at_local_wakeup_boost_ms);

			z_cpu->wakeup_boost_ticks = ZENITH_WAKEUP_BOOST_TICKS;
			if (ms) {
				if (ms > ZENITH_WAKEUP_BOOST_MS_MAX)
					ms = ZENITH_WAKEUP_BOOST_MS_MAX;
				z_cpu->wakeup_boost_until_ns =
					ktime_get_ns() +
					(u64)ms * NSEC_PER_MSEC;
			}
		}
	}
	z_cpu->wakeup_prev_util = util;

	if (tunables->kcpustat_hispeed_enable) {
		zenith_kcpustat_sample(z_cpu,
				       tunables->kcpustat_window_us,
				       tunables->kcpustat_filter_shift,
				       time);
		util = zenith_kcpustat_blend(z_cpu, util, max_cap, time);
	}

	z_policy->nice_pct = tunables->ignore_nice_load ?
		zenith_sample_nice_pct(z_cpu, time) : 0;

	next_f = zenith_get_next_freq(z_policy, util, max_cap);

	if (z_policy->policy->fast_switch_enabled) {
		zenith_execute_switch(z_policy, time, next_f);
	} else {
		raw_spin_lock(&z_policy->update_lock);
		zenith_execute_switch(z_policy, time, next_f);
		raw_spin_unlock(&z_policy->update_lock);
	}
}

static void zenith_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct zenith_cpu *z_cpu = container_of(hook, struct zenith_cpu, update_util);
	struct zenith_policy *z_policy = z_cpu->z_policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long util = 0, max_cap = 1;
	unsigned int next_f, j;

	raw_spin_lock(&z_policy->update_lock);

	zenith_iowait_boost(z_cpu, time, flags, tunables->io_is_busy);
	z_cpu->last_update = time;

	zenith_ignore_dl_rate_limit(z_cpu, z_policy);

	if (zenith_should_update_freq(z_policy, time)) {
		unsigned int nice_pct_max = 0;

		for_each_cpu(j, z_policy->policy->cpus) {
			struct zenith_cpu *j_z_cpu = &per_cpu(zenith_cpu, j);
			unsigned long j_util, j_max;

			/*
			 * Skip siblings whose zenith_start() has not yet
			 * populated j_z_cpu->z_policy.  Without this guard,
			 * the first update tick that fires on a policy CPU
			 * before all sibling CPUs in the same policy have
			 * completed their per-CPU zenith_start() iteration
			 * dereferences NULL->tunables for the not-yet-
			 * initialized sibling and faults at
			 * NULL+offsetof(struct zenith_policy, tunables).
			 * The race window is closed by the two-loop ordering
			 * in zenith_start(), but keep the runtime check as
			 * belt-and-suspenders for any future code path that
			 * might transiently leave z_policy NULL.
			 */
			if (unlikely(!READ_ONCE(j_z_cpu->z_policy)))
				continue;

			j_util = zenith_get_util(j_z_cpu);
			j_max = j_z_cpu->max_capacity;
			j_util = zenith_iowait_apply(j_z_cpu, time, j_util, j_max);
			zenith_migration_arrival_check(j_z_cpu, j_util,
						       j_max, z_policy);
			if (READ_ONCE(tunables->wakeup_boost) && j_max) {
				unsigned int cur_pct =
					(unsigned int)((j_util * 100) / j_max);
				unsigned int prev_pct =
					j_z_cpu->wakeup_prev_util ?
					(unsigned int)((j_z_cpu->wakeup_prev_util *
							100) / j_max) : 0;

				if (prev_pct < ZENITH_WAKEUP_IDLE_THRESH_PCT &&
				    cur_pct >= ZENITH_WAKEUP_BUSY_THRESH_PCT) {
					unsigned int ms = zenith_glide_value(
						z_policy,
						READ_ONCE(
						 tunables->wakeup_boost_ms),
						z_policy->
						 at_local_wakeup_boost_ms);

					j_z_cpu->wakeup_boost_ticks =
						ZENITH_WAKEUP_BOOST_TICKS;
					if (ms) {
						if (ms > ZENITH_WAKEUP_BOOST_MS_MAX)
							ms = ZENITH_WAKEUP_BOOST_MS_MAX;
						j_z_cpu->wakeup_boost_until_ns =
							ktime_get_ns() +
							(u64)ms * NSEC_PER_MSEC;
					}
				}
			}
			j_z_cpu->wakeup_prev_util = j_util;

			if (tunables->kcpustat_hispeed_enable) {
				zenith_kcpustat_sample(j_z_cpu,
					tunables->kcpustat_window_us,
					tunables->kcpustat_filter_shift,
					time);
				j_util = zenith_kcpustat_blend(j_z_cpu, j_util,
							       j_max, time);
			}

			if (tunables->ignore_nice_load) {
				unsigned int p = zenith_sample_nice_pct(j_z_cpu, time);

				if (p > nice_pct_max)
					nice_pct_max = p;
			}

			if (j_util * max_cap > j_max * util) {
				util = j_util;
				max_cap = j_max;
			}
		}

		z_policy->nice_pct = tunables->ignore_nice_load ? nice_pct_max : 0;

		next_f = zenith_get_next_freq(z_policy, util, max_cap);
		zenith_execute_switch(z_policy, time, next_f);
	}

	raw_spin_unlock(&z_policy->update_lock);
}

/************************ Kthread Slow Path ***********************/

static void zenith_work(struct kthread_work *work)
{
	struct zenith_policy *z_policy = container_of(work, struct zenith_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&z_policy->update_lock, flags);
	freq = z_policy->next_freq;
	z_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&z_policy->update_lock, flags);

	mutex_lock(&z_policy->work_lock);
	__cpufreq_driver_target(z_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&z_policy->work_lock);
}

static void zenith_irq_work(struct irq_work *irq_work)
{
	struct zenith_policy *z_policy = container_of(irq_work, struct zenith_policy, irq_work);

	kthread_queue_work(&z_policy->worker, &z_policy->work);
}

/************************** Sysfs Interface & Tunables ************************/

static struct zenith_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct zenith_tunables *to_zenith_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct zenith_tunables, attr_set);
}

/* Recompute z_policy->min_rate_limit_ns from the current cached
 * up/down delays.  Reads use READ_ONCE so concurrent updaters of
 * either delay (sysfs writers, profile switches, auto_tune) cannot
 * tear our snapshot, and the result is published with WRITE_ONCE
 * so the hot-path reader (zenith_should_update_freq) observes a
 * coherent value.  The min_rate_lock mutex previously used here is
 * gone: readers and writers are all single-instruction loads/stores
 * of a 64-bit aligned scalar, and the worst-case stale read costs
 * exactly one rate-limit tick of latency (already bounded by the
 * hot path anyway).
 */
static void update_min_rate_limit_ns(struct zenith_policy *z_policy)
{
	s64 up_ns = READ_ONCE(z_policy->up_rate_delay_ns);
	s64 down_ns = READ_ONCE(z_policy->down_rate_delay_ns);

	WRITE_ONCE(z_policy->min_rate_limit_ns, min(up_ns, down_ns));
}

static void zenith_update_cluster_rate_scale(struct zenith_policy *z_policy)
{
	unsigned int little_thresh =
		(SCHED_CAPACITY_SCALE * ZENITH_CLUSTER_LITTLE_THRESH_PCT) / 100;
	unsigned int cluster_cap = 0;
	unsigned int big_cap = 0;
	unsigned int cpu;

	for_each_cpu(cpu, z_policy->policy->cpus) {
		unsigned int cap = arch_scale_cpu_capacity(cpu);

		if (cap > cluster_cap)
			cluster_cap = cap;
	}
	for_each_possible_cpu(cpu) {
		unsigned int cap = arch_scale_cpu_capacity(cpu);

		if (cap > big_cap)
			big_cap = cap;
	}
	if (cluster_cap < little_thresh)
		z_policy->cluster_class = ZENITH_CLUSTER_LITTLE;
	else if (big_cap && cluster_cap >= big_cap)
		z_policy->cluster_class = ZENITH_CLUSTER_PRIME;
	else
		z_policy->cluster_class = ZENITH_CLUSTER_BIG;
	if (z_policy->tunables->rate_limit_cluster_scale &&
	    cluster_cap < little_thresh) {
		z_policy->up_rate_scale = 2;
		z_policy->down_rate_scale_shift = 1;
	} else {
		z_policy->up_rate_scale = 1;
		z_policy->down_rate_scale_shift = 0;
	}
}

static void zenith_update_rate_delay_ns(struct zenith_policy *z_policy)
{
	struct zenith_tunables *t = z_policy->tunables;
	unsigned int up_us = zenith_tunable_or_local(z_policy,
		t->up_rate_limit_us, z_policy->at_effective_up_rate_limit_us);
	unsigned int down_us = zenith_tunable_or_local(z_policy,
		t->down_rate_limit_us,
		z_policy->at_effective_down_rate_limit_us);
	s64 up_ns = (u64)up_us * NSEC_PER_USEC;
	s64 down_ns = (u64)down_us * NSEC_PER_USEC;

	up_ns *= max(z_policy->up_rate_scale, 1U);
	down_ns >>= z_policy->down_rate_scale_shift;
	WRITE_ONCE(z_policy->up_rate_delay_ns, up_ns);
	WRITE_ONCE(z_policy->down_rate_delay_ns, down_ns);
	update_min_rate_limit_ns(z_policy);
}

/* Force the next zenith_get_next_freq() call on every policy sharing
 * this tunables set to recompute from scratch, bypassing the
 * cached_raw_freq shortcut. Call this from any sysfs _store that
 * changes a value which feeds into the freq decision, so the user's
 * write takes effect on the very next scheduler tick rather than
 * waiting for pre-resolve freq to drift. Must be called with the
 * attr_set->update_lock held (governor_store always takes it).
 */
static void zenith_invalidate_cache(struct gov_attr_set *attr_set)
{
	struct zenith_policy *z_pol;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		z_pol->need_freq_update = true;
}

/* Refresh the per-policy {up,down}_rate_delay_ns caches on every
 * policy sharing this tunables set after a profile change has
 * mutated the {up,down}_rate_limit_us fields.  Without this, the
 * hot path keeps reading the previous profile's cached delays
 * (zenith_up_down_rate_limit() reads z_policy->up_rate_delay_ns,
 * not tunables->up_rate_limit_us).  Caller must hold the
 * attr_set->update_lock; sysfs profile_store always does.
 */
static void zenith_refresh_rate_delays(struct gov_attr_set *attr_set)
{
	struct zenith_policy *z_pol;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		zenith_update_cluster_rate_scale(z_pol);
		zenith_update_rate_delay_ns(z_pol);
	}
}

/* Update the per-policy prefer_silver hit-rate snapshot (dead --
 * CONFIG_SCHED_PREFER_SILVER was removed, the #ifdef block below
 * is never compiled).  Was called once per V1 classifier window
 * from zenith_auto_tune_work(); the no-op #else stub keeps the
 * call site from breaking.
 */
static void zenith_at_update_prefer_silver_rate(struct zenith_policy *z_policy,
						struct zenith_tunables *t,
						unsigned int *flags)
{
#ifdef CONFIG_SCHED_PREFER_SILVER
	unsigned int hit_now = 0, miss_now = 0;
	unsigned int hit_delta, miss_delta, total_delta;
	unsigned int rate;

	/*
	 * Disable transition fast path: when sysctl_prefer_silver was
	 * flipped to 0 mid-run, the global hit / miss counters stop
	 * incrementing.  The (!total_delta) branch below would still
	 * correctly zero the cached rate -- but only on the next
	 * worker window, leaving the previous "hot" reading active
	 * for one full classifier cycle's worth of zenith_get_next_freq()
	 * calls.  Drop the cache immediately on observing the disable
	 * so the up-threshold bump path also flips off in the same
	 * window the user expected.
	 *
	 * The previous-snapshot fields are zeroed too so re-enabling
	 * prefer_silver later does not see a stale baseline that would
	 * make the next window's delta artificially large.
	 */
	if (!READ_ONCE(sysctl_prefer_silver)) {
		z_policy->ps_hit_rate_pct = 0;
		z_policy->ps_prev_hit     = 0;
		z_policy->ps_prev_miss    = 0;
		return;
	}

	prefer_silver_get_hit_miss(&hit_now, &miss_now);

	hit_delta  = hit_now  - z_policy->ps_prev_hit;
	miss_delta = miss_now - z_policy->ps_prev_miss;
	total_delta = hit_delta + miss_delta;

	z_policy->ps_prev_hit  = hit_now;
	z_policy->ps_prev_miss = miss_now;

	if (!total_delta) {
		/* No prefer_silver activity in this window.  Decay the
		 * cached rate towards zero so a brief idle period
		 * cannot leave a stale "hot" reading behind.
		 */
		z_policy->ps_hit_rate_pct = 0;
		return;
	}

	rate = (hit_delta * 100U) / total_delta;
	if (rate > 100)
		rate = 100;
	z_policy->ps_hit_rate_pct = rate;

	if (t->prefer_silver_aware &&
	    rate >= t->prefer_silver_hot_threshold_pct)
		*flags |= ZENITH_AT_FLAG_PREFER_SILVER_HOT;
#else
	(void)z_policy;
	(void)t;
	(void)flags;
#endif
}

/* Push a one-shot sample into the per-policy auto-tune ring buffer.
 *
 * Called from the tail of zenith_auto_tune_work() after the V1
 * classifier has resolved a target and the V2 state machine has
 * settled on a state.  Reads the freshly-populated at_last_* mirrors
 * on z_policy so the caller does not have to redundantly thread the
 * same dozen-odd values; only from_state is taken as a parameter
 * because at_last_state has already been overwritten with the new
 * state by the time we get here.
 *
 * The ring is a simple head-advances-then-wraps circular buffer.
 * The worker is the only writer and it is single-shot per policy
 * (a delayed_work, not a timer with overlapping fire), so no
 * locking is needed on the writer side.  Sysfs readers walk the
 * ring under no extra lock and accept tearing on the wrap window;
 * the on-disk semantics are "last N samples observed by the worker",
 * which is exactly what diagnostics want.
 */
static void zenith_at_log_push(struct zenith_policy *z_policy,
			       unsigned int from_state,
			       unsigned int v1_target,
			       bool emergency)
{
	struct zenith_at_log_entry *e;
	unsigned int slot;

	slot = z_policy->at_log_head;
	if (slot >= ZENITH_AT_LOG_NR)
		slot = 0;
	e = &z_policy->at_log[slot];

	e->ts_ns		= ktime_get_ns();
	e->flags		= z_policy->at_last_flags;
	e->var_x256		= z_policy->at_last_var_x256;
	e->thermal_pressure	= z_policy->at_last_thermal_pressure;
	e->sat_pct		= z_policy->at_last_sat_pct;
	e->events_rate_x2	= z_policy->at_last_events_rate_x2;
	e->thermal_slope	= z_policy->at_last_thermal_slope;
	e->v1_target		= (u8)v1_target;
	e->v2_from_state	= (u8)from_state;
	e->v2_to_state		= (u8)z_policy->at_last_state;
	e->reason		= (u8)z_policy->at_last_reason;
	e->emergency		= emergency ? 1 : 0;

	z_policy->at_log_head = (slot + 1) % ZENITH_AT_LOG_NR;
	if (z_policy->at_log_count < ZENITH_AT_LOG_NR)
		z_policy->at_log_count++;
}

/* V3 self-calibration tail.
 *
 * Called from zenith_auto_tune_work() once per V1 window when V3 is
 * enabled (zenith_auto_tune_v3_key TRUE) and the auto_tune_v3 scalar
 * is non-zero (defended against a momentary tear during a sysfs
 * store).  Walks the per-policy at_log ring, counts V2 state
 * transitions, and -- if the wall-clock interval has elapsed since
 * the last calibration -- updates at_v3_hyst_offset / at_v3_cool_offset
 * within the bounded range [ZENITH_AT_V3_OFFSET_MIN,
 * ZENITH_AT_V3_OFFSET_MAX].
 *
 * Mode 1 (OBSERVE) updates at_v3_last_transitions and exposes the
 * value via auto_tune_v3_state but never adjusts the offsets.  Mode 2
 * (APPLY) does both.
 *
 * Cost: at most ZENITH_AT_LOG_NR (=16) loads and a small bounded
 * amount of arithmetic, gated by the wall-clock interval check
 * (default 60 s).  Single-threaded with the v2 worker so no locking
 * is required for the per-policy fields; sysfs *_show readers are
 * serialised under the gov_attr_set rwsem.
 */
static void zenith_at_v3_calibrate(struct zenith_policy *z_policy,
				   unsigned int mode)
{
	struct zenith_tunables *t = z_policy->tunables;
	u64 now = ktime_get_boottime_ns();
	u64 interval_ns;
	unsigned int interval_ms;
	unsigned int transitions = 0;
	unsigned int prev_state;
	unsigned int idx, count, head;
	bool has_prev = false;
	/* Patch J: snapshot of the live offsets before any APPLY-mode
	 * nudge below; recorded in the calibration ring at the tail of
	 * this function so userspace can audit V3 drift.
	 */
	signed char hyst_before, cool_before;
	struct zenith_at_v3_calib_log_entry *log_entry;
	unsigned int log_slot;

	interval_ms = READ_ONCE(t->auto_tune_v3_interval_ms);
	if (interval_ms < ZENITH_AT_V3_INTERVAL_MIN_MS)
		interval_ms = ZENITH_AT_V3_INTERVAL_MIN_MS;
	if (interval_ms > ZENITH_AT_V3_INTERVAL_MAX_MS)
		interval_ms = ZENITH_AT_V3_INTERVAL_MAX_MS;
	interval_ns = (u64)interval_ms * NSEC_PER_MSEC;

	if (z_policy->at_v3_last_calib_ns &&
	    now - z_policy->at_v3_last_calib_ns < interval_ns)
		return;

	count = z_policy->at_log_count;
	head = z_policy->at_log_head;
	prev_state = 0;

	/* Walk oldest -> newest. */
	for (idx = 0; idx < count; idx++) {
		unsigned int slot;
		struct zenith_at_log_entry *e;

		if (count < ZENITH_AT_LOG_NR)
			slot = idx;
		else
			slot = (head + idx) % ZENITH_AT_LOG_NR;

		e = &z_policy->at_log[slot];
		if (!has_prev) {
			prev_state = e->v2_to_state;
			has_prev = true;
			continue;
		}
		if (e->v2_to_state != prev_state) {
			transitions++;
			prev_state = e->v2_to_state;
		}
	}

	z_policy->at_v3_last_transitions = transitions;
	z_policy->at_v3_last_calib_ns = now;

	hyst_before = z_policy->at_v3_hyst_offset;
	cool_before = z_policy->at_v3_cool_offset;

	/* Apply bounded nudge to offsets only in APPLY mode.  OBSERVE
	 * mode still falls through to the calibration-ring push below
	 * with before == after so userspace can see "V3 ran but did
	 * not nudge because mode is OBSERVE".
	 */
	if (mode == ZENITH_AT_V3_MODE_APPLY) {
		if (transitions >= ZENITH_AT_V3_THRASH_HI) {
			if (z_policy->at_v3_hyst_offset < ZENITH_AT_V3_OFFSET_MAX)
				z_policy->at_v3_hyst_offset++;
			if (z_policy->at_v3_cool_offset < ZENITH_AT_V3_OFFSET_MAX)
				z_policy->at_v3_cool_offset++;
		} else if (transitions <= ZENITH_AT_V3_THRASH_LO) {
			if (z_policy->at_v3_hyst_offset > ZENITH_AT_V3_OFFSET_MIN)
				z_policy->at_v3_hyst_offset--;
			if (z_policy->at_v3_cool_offset > ZENITH_AT_V3_OFFSET_MIN)
				z_policy->at_v3_cool_offset--;
		}
	}

	/* Patch J: push a calibration record on every tick (both
	 * OBSERVE and APPLY) so userspace can audit V3 drift without
	 * ftrace.  Single-writer ring -- no locking needed against
	 * sysfs readers, who walk the ring under the gov_attr_set
	 * rwsem and accept the same wrap-window tearing the existing
	 * at_log path tolerates.
	 */
	log_slot = z_policy->at_v3_calib_log_head;
	if (log_slot >= ZENITH_AT_V3_CALIB_LOG_NR)
		log_slot = 0;
	log_entry = &z_policy->at_v3_calib_log[log_slot];
	log_entry->ts_ns	= now;
	log_entry->transitions	= transitions;
	log_entry->mode		= (u8)mode;
	log_entry->hyst_before	= hyst_before;
	log_entry->hyst_after	= z_policy->at_v3_hyst_offset;
	log_entry->cool_before	= cool_before;
	log_entry->cool_after	= z_policy->at_v3_cool_offset;
	z_policy->at_v3_calib_log_head =
		(log_slot + 1) % ZENITH_AT_V3_CALIB_LOG_NR;
	if (z_policy->at_v3_calib_log_count < ZENITH_AT_V3_CALIB_LOG_NR)
		z_policy->at_v3_calib_log_count++;
}

/* Effective hysteresis_windows / cooldown_windows after V3 nudge.
 *
 * Returns the unmodified base when V3 is OFF or in OBSERVE mode.  When
 * V3 mode is APPLY (== 2), adds the per-policy signed offset and clamps
 * the result to [1, ZENITH_AT_*_WINDOWS_MAX].  >=1 floor preserves the
 * V2 state machine invariant that at least one window of agreement is
 * required before a state change commits.
 */
static inline unsigned int
zenith_at_eff_hyst_windows(struct zenith_policy *z_policy, unsigned int base)
{
	int v;

	if (!static_branch_likely(&zenith_auto_tune_v3_key))
		return base;
	if (READ_ONCE(z_policy->tunables->auto_tune_v3) !=
	    ZENITH_AT_V3_MODE_APPLY)
		return base;

	v = (int)base + (int)z_policy->at_v3_hyst_offset;
	if (v < 1)
		v = 1;
	if (v > ZENITH_AT_HYSTERESIS_WINDOWS_MAX)
		v = ZENITH_AT_HYSTERESIS_WINDOWS_MAX;
	return (unsigned int)v;
}

static inline unsigned int
zenith_at_eff_cool_windows(struct zenith_policy *z_policy, unsigned int base)
{
	int v;

	if (!static_branch_likely(&zenith_auto_tune_v3_key))
		return base;
	if (READ_ONCE(z_policy->tunables->auto_tune_v3) !=
	    ZENITH_AT_V3_MODE_APPLY)
		return base;

	v = (int)base + (int)z_policy->at_v3_cool_offset;
	if (v < 1)
		v = 1;
	if (v > ZENITH_AT_COOLDOWN_WINDOWS_MAX)
		v = ZENITH_AT_COOLDOWN_WINDOWS_MAX;
	return (unsigned int)v;
}

/* Reset all observability counters on a single policy.
 *
 * Clears both the per-policy zenith_stats[] array and the auto-tune
 * classifier ring.  Used by the zenith_stats_reset sysfs node so an
 * operator can mark a "start of measurement" point before a benchmark
 * without having to re-init the governor.  The decision-tier counters
 * are racy with the fast path (the per-CPU update path increments
 * them with a plain ++), but that race is bounded to a few lost
 * counts on the boundary which is acceptable for diagnostics.
 */
static void zenith_policy_observability_reset(struct zenith_policy *z_policy)
{
	memset(z_policy->stats, 0, sizeof(z_policy->stats));
	memset(z_policy->dec_lat_buckets, 0,
	       sizeof(z_policy->dec_lat_buckets));
	memset(z_policy->at_log, 0, sizeof(z_policy->at_log));
	z_policy->at_log_head = 0;
	z_policy->at_log_count = 0;
	/* Patch J: clear the V3 calibration audit ring on the same
	 * sysfs reset path.  Same rationale as the at_log clear above:
	 * keep all observability surfaces aligned so an operator can
	 * read a clean baseline from any of them on the next eval tick.
	 */
	memset(z_policy->at_v3_calib_log, 0,
	       sizeof(z_policy->at_v3_calib_log));
	z_policy->at_v3_calib_log_head = 0;
	z_policy->at_v3_calib_log_count = 0;
	/* Patch B7-2: clear the per-policy decision ring on the same
	 * sysfs reset path that clears stats / dec_lat_buckets.  Keeps
	 * the three observability surfaces aligned so an operator can
	 * "echo 1 > zenith_stats_reset" and read a clean baseline from
	 * any of them on the next eval tick.
	 */
	memset(z_policy->dec_ring, 0, sizeof(z_policy->dec_ring));
	z_policy->dec_ring_head = 0;
}

static void zenith_reset_local_actions(struct zenith_policy *z_policy)
{
	z_policy->at_effective_up_rate_limit_us = z_policy->tunables->up_rate_limit_us;
	z_policy->at_effective_down_rate_limit_us =
		z_policy->tunables->down_rate_limit_us;
	z_policy->at_effective_up_threshold = z_policy->tunables->up_threshold;
	z_policy->at_effective_down_threshold =
		z_policy->tunables->down_threshold;
	z_policy->at_effective_input_boost_ms =
		z_policy->tunables->input_boost_ms;
	z_policy->at_effective_input_boost_cap_pct =
		z_policy->tunables->input_boost_cap_pct;
	z_policy->at_effective_down_rate_adaptive =
		z_policy->tunables->down_rate_adaptive;
	z_policy->at_effective_down_threshold_adaptive =
		z_policy->tunables->down_threshold_adaptive;
	z_policy->at_effective_frame_pace_floor_pct =
		z_policy->tunables->frame_pace_floor_pct;
	z_policy->at_effective_game_mode = z_policy->tunables->game_mode;
	z_policy->at_local_actions = false;
	z_policy->at_local_glides_active = false;
	z_policy->at_local_tiers_active = false;
	zenith_update_rate_delay_ns(z_policy);
}

/* Single-policy variant of zenith_refresh_rate_delays() for the
 * auto_tune classifier worker.  The worker runs from delayed_work
 * context and does NOT hold attr_set->update_lock, so iterating
 * policy_list there would race with concurrent gov_attr_set_get/put.
 * The per-policy z_policy that owns the worker is guaranteed live
 * (zenith_exit() cancels the worker before unlinking).  Other
 * policies sharing the same tunables pick up the new delays on
 * their own next auto_tune tick.
 */
static void zenith_refresh_rate_delays_one(struct zenith_policy *z_policy)
{
	zenith_update_cluster_rate_scale(z_policy);
	zenith_update_rate_delay_ns(z_policy);
}

static const char *zenith_profile_name(unsigned int profile)
{
	switch (profile) {
	case ZENITH_PROFILE_PERFORMANCE:
		return "performance";
	case ZENITH_PROFILE_BALANCED:
		return "balanced";
	case ZENITH_PROFILE_BATTERY:
		return "battery";
	case ZENITH_PROFILE_LEGACY:
		return "legacy";
	case ZENITH_PROFILE_GAMING:
		return "gaming";
	case ZENITH_PROFILE_AUDIO:
		return "audio";
	case ZENITH_PROFILE_CUSTOM:
	default:
		return "custom";
	}
}

/* Patch L: human-readable zenith logging helpers.  All three are
 * gated on READ_ONCE(t->verbose_log) so production builds (where
 * verbose_log defaults to 0) pay only the cost of one load + one
 * branch on the cold profile_store / master-store paths.  The
 * pr_info() format strings deliberately match across the three
 * helpers so a userspace log scraper can grep "zenith: " and parse
 * a stable structure.
 *
 * Patch M: emit via pr_info_ratelimited() so a pathological caller
 * (auto_tune flapping the profile, or a userspace tool fanning out
 * master-switch stores) cannot spam dmesg.  Default rate-limit is
 * 10 messages per 5 seconds (DEFAULT_RATELIMIT_BURST /
 * DEFAULT_RATELIMIT_INTERVAL); excess events are coalesced with a
 * "callbacks suppressed" trailer so the trail is still meaningful.
 *
 * profile_change: emitted by profile_store on the user-write path
 *                 *before* the bake runs, so the dmesg trail reads
 *                 "switching X -> Y" / "applied Y profile: ..." in
 *                 chronological order.
 * profile_applied: emitted at the tail of zenith_apply_profile()
 *                  with a one-line summary of the major bake
 *                  results.  Fires for AUTO-driven applies too,
 *                  giving the operator visibility into the auto
 *                  selector's decisions.
 * master_flip:    emitted by each of the seven master-switch stores
 *                 (audio_aware / render_aware / camera_aware /
 *                 psi_aware / game_auto / auto_tune_v3 /
 *                 thermal_aware) after a value change is committed,
 *                 with old / new values both included so the trail
 *                 is meaningful even under fast successive flips.
 */
static inline void zenith_log_profile_change(struct zenith_tunables *t,
					     unsigned int old_prof,
					     unsigned int new_prof)
{
	if (!READ_ONCE(t->verbose_log))
		return;
	pr_info_ratelimited("zenith: switching profile %s -> %s\n",
		zenith_profile_name(old_prof),
		zenith_profile_name(new_prof));
}

static inline void zenith_log_profile_applied(struct zenith_tunables *t,
					      unsigned int prof)
{
	if (!READ_ONCE(t->verbose_log))
		return;
	pr_info_ratelimited("zenith: applied %s profile: hispeed_freq_pct=%u up_threshold=%u down_threshold=%u climb_mode=%u freq_step_pct=%u powersave_bias=%u up_rate_limit_us=%u down_rate_limit_us=%u wakeup_boost=%u down_threshold_adaptive=%u\n",
		zenith_profile_name(prof),
		t->hispeed_freq_pct, t->up_threshold, t->down_threshold,
		t->climb_mode, t->freq_step_pct, t->powersave_bias,
		t->up_rate_limit_us, t->down_rate_limit_us,
		t->wakeup_boost, t->down_threshold_adaptive);
}

static inline void zenith_log_master_flip(struct zenith_tunables *t,
					  const char *name,
					  unsigned int old_val,
					  unsigned int new_val)
{
	if (!READ_ONCE(t->verbose_log))
		return;
	pr_info_ratelimited("zenith: master %s %u -> %u\n",
		name, old_val, new_val);
}

static const char *zenith_at_state_name(unsigned int state)
{
	switch (state) {
	case ZENITH_AT_STATE_EFFICIENCY:
		return "efficiency";
	case ZENITH_AT_STATE_BALANCED:
		return "balanced";
	case ZENITH_AT_STATE_LATENCY:
		return "latency";
	case ZENITH_AT_STATE_SUSTAINED_PERF:
		return "sustained_perf";
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		return "thermal_recovery";
	default:
		return "unknown";
	}
}

static const char *zenith_at_reason_name(unsigned int reason)
{
	switch (reason) {
	case ZENITH_AT_REASON_CLASSIFIER:
		return "classifier";
	case ZENITH_AT_REASON_CAMERA_RENDER:
		return "camera_render";
	case ZENITH_AT_REASON_MEMSTALL:
		return "memstall";
	case ZENITH_AT_REASON_AUDIO:
		return "audio";
	case ZENITH_AT_REASON_VARIANCE:
		return "variance";
	case ZENITH_AT_REASON_THERMAL:
		return "thermal";
	case ZENITH_AT_REASON_COOLDOWN:
		return "cooldown";
	case ZENITH_AT_REASON_HYSTERESIS:
		return "hysteresis";
	case ZENITH_AT_REASON_SCREEN:
		return "screen";
	case ZENITH_AT_REASON_PSI:
		return "psi";
	case ZENITH_AT_REASON_FRAME:
		return "frame";
	case ZENITH_AT_REASON_GAME:
		return "game";
	case ZENITH_AT_REASON_THERMAL_SLOPE:
		return "thermal_slope";
	default:
		return "unknown";
	}
}

static unsigned int zenith_at_clamp(unsigned int val, unsigned int lo,
				    unsigned int hi)
{
	if (val < lo)
		return lo;
	if (val > hi)
		return hi;
	return val;
}

struct zenith_at_guardrails {
	unsigned int up_rate_min;
	unsigned int up_rate_max;
	unsigned int down_rate_min;
	unsigned int down_rate_max;
	unsigned int up_threshold_min;
	unsigned int up_threshold_max;
	unsigned int down_threshold_min;
	unsigned int down_threshold_max;
	unsigned int input_boost_min;
	unsigned int input_boost_max;
	unsigned int input_cap_min;
	unsigned int input_cap_max;
	unsigned int down_adaptive_min;
	unsigned int down_adaptive_max;
	unsigned int down_thresh_adaptive_min;
	unsigned int down_thresh_adaptive_max;
	unsigned int frame_floor_min;
	unsigned int frame_floor_max;
	unsigned int game_mode_min;
	unsigned int game_mode_max;
};

static void zenith_at_get_guardrails(unsigned int profile,
				     struct zenith_at_guardrails *g)
{
	switch (profile) {
	case ZENITH_PROFILE_PERFORMANCE:
	case ZENITH_PROFILE_GAMING:
		*g = (struct zenith_at_guardrails) {
			.up_rate_min = 0, .up_rate_max = 250,
			.down_rate_min = 4000, .down_rate_max = 12000,
			.up_threshold_min = 55, .up_threshold_max = 75,
			.down_threshold_min = 35, .down_threshold_max = 60,
			.input_boost_min = 100, .input_boost_max = 220,
			.input_cap_min = 0, .input_cap_max = 100,
			.down_adaptive_min = 1, .down_adaptive_max = 1,
			.down_thresh_adaptive_min = 5,
			.down_thresh_adaptive_max = 15,
			.frame_floor_min = 25, .frame_floor_max = 70,
			.game_mode_min = 0, .game_mode_max = 2,
		};
		break;
	case ZENITH_PROFILE_BATTERY:
		*g = (struct zenith_at_guardrails) {
			.up_rate_min = 250, .up_rate_max = 1000,
			.down_rate_min = 1000, .down_rate_max = 4000,
			.up_threshold_min = 78, .up_threshold_max = 95,
			.down_threshold_min = 35, .down_threshold_max = 55,
			.input_boost_min = 0, .input_boost_max = 80,
			.input_cap_min = 50, .input_cap_max = 75,
			.down_adaptive_min = 0, .down_adaptive_max = 1,
			.down_thresh_adaptive_min = 0,
			.down_thresh_adaptive_max = 5,
			.frame_floor_min = 0, .frame_floor_max = 35,
			.game_mode_min = 0, .game_mode_max = 1,
		};
		break;
	case ZENITH_PROFILE_LEGACY:
		*g = (struct zenith_at_guardrails) {
			.up_rate_min = 1000, .up_rate_max = 4000,
			.down_rate_min = 2000, .down_rate_max = 8000,
			.up_threshold_min = 75, .up_threshold_max = 90,
			.down_threshold_min = 70, .down_threshold_max = 90,
			.input_boost_min = 0, .input_boost_max = 0,
			.input_cap_min = 0, .input_cap_max = 0,
			.down_adaptive_min = 0, .down_adaptive_max = 0,
			.down_thresh_adaptive_min = 0,
			.down_thresh_adaptive_max = 0,
			.frame_floor_min = 0, .frame_floor_max = 0,
			.game_mode_min = 0, .game_mode_max = 0,
		};
		break;
	case ZENITH_PROFILE_CUSTOM:
	case ZENITH_PROFILE_BALANCED:
	case ZENITH_PROFILE_AUDIO:
	default:
		*g = (struct zenith_at_guardrails) {
			.up_rate_min = 50, .up_rate_max = 500,
			.down_rate_min = 2500, .down_rate_max = 7000,
			.up_threshold_min = 65, .up_threshold_max = 85,
			.down_threshold_min = 45, .down_threshold_max = 70,
			.input_boost_min = 60, .input_boost_max = 160,
			.input_cap_min = 65, .input_cap_max = 90,
			.down_adaptive_min = 0, .down_adaptive_max = 1,
			.down_thresh_adaptive_min = 0,
			.down_thresh_adaptive_max = 10,
			.frame_floor_min = 15, .frame_floor_max = 55,
			.game_mode_min = 0, .game_mode_max = 2,
		};
		break;
	}
}

static void zenith_at_get_policy_guardrails(struct zenith_policy *z_policy,
					    unsigned int profile,
					    struct zenith_at_guardrails *g)
{
	zenith_at_get_guardrails(profile, g);
	if (!z_policy->tunables->auto_tune_cluster_aware)
		return;

	switch (z_policy->cluster_class) {
	case ZENITH_CLUSTER_LITTLE:
		g->up_rate_min = max(g->up_rate_min,
				     (g->up_rate_min + g->up_rate_max) / 2);
		g->input_boost_max = min(g->input_boost_max,
					 (g->input_boost_min +
					  g->input_boost_max) / 2);
		g->frame_floor_max = min(g->frame_floor_max,
					 (g->frame_floor_min +
					  g->frame_floor_max) / 2);
		g->game_mode_max = min(g->game_mode_max, 1U);
		break;
	case ZENITH_CLUSTER_PRIME:
		g->up_rate_max = min(g->up_rate_max,
				     (g->up_rate_min + g->up_rate_max) / 2);
		break;
	case ZENITH_CLUSTER_BIG:
	default:
		break;
	}
}

static unsigned int zenith_at_state_to_profile(unsigned int state)
{
	switch (state) {
	case ZENITH_AT_STATE_EFFICIENCY:
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		return ZENITH_PROFILE_BATTERY;
	case ZENITH_AT_STATE_LATENCY:
	case ZENITH_AT_STATE_SUSTAINED_PERF:
		return ZENITH_PROFILE_PERFORMANCE;
	case ZENITH_AT_STATE_BALANCED:
	default:
		return ZENITH_PROFILE_BALANCED;
	}
}

static unsigned int zenith_profile_to_at_state(unsigned int profile)
{
	switch (profile) {
	case ZENITH_PROFILE_PERFORMANCE:
	case ZENITH_PROFILE_GAMING:
		return ZENITH_AT_STATE_LATENCY;
	case ZENITH_PROFILE_BATTERY:
		return ZENITH_AT_STATE_EFFICIENCY;
	case ZENITH_PROFILE_BALANCED:
	case ZENITH_PROFILE_LEGACY:
	case ZENITH_PROFILE_AUDIO:
	case ZENITH_PROFILE_CUSTOM:
	default:
		return ZENITH_AT_STATE_BALANCED;
	}
}

static const char *zenith_at_cluster_name(unsigned int cluster)
{
	switch (cluster) {
	case ZENITH_CLUSTER_LITTLE:
		return "little";
	case ZENITH_CLUSTER_BIG:
		return "big";
	case ZENITH_CLUSTER_PRIME:
		return "prime";
	default:
		return "unknown";
	}
}

static unsigned int zenith_at_profile_for_state(struct zenith_policy *z_policy,
						unsigned int state)
{
	if (!z_policy->tunables->auto_tune_cluster_aware)
		return zenith_at_state_to_profile(state);

	switch (z_policy->cluster_class) {
	case ZENITH_CLUSTER_LITTLE:
		if (state == ZENITH_AT_STATE_LATENCY ||
		    state == ZENITH_AT_STATE_SUSTAINED_PERF)
			return ZENITH_PROFILE_BALANCED;
		return zenith_at_state_to_profile(state);
	case ZENITH_CLUSTER_PRIME:
		if (state == ZENITH_AT_STATE_BALANCED)
			return ZENITH_PROFILE_PERFORMANCE;
		return zenith_at_state_to_profile(state);
	case ZENITH_CLUSTER_BIG:
	default:
		return zenith_at_state_to_profile(state);
	}
}

static void zenith_at_mark_override(struct zenith_tunables *t,
				    unsigned long bit)
{
	t->auto_tune_override_mask |= bit;
}

static bool zenith_at_set_uint(struct zenith_tunables *t, unsigned long bit,
			       unsigned int *field, unsigned int val,
			       unsigned int lo, unsigned int hi)
{
	if (t->auto_tune_override_mask & bit)
		return false;
	val = zenith_at_clamp(val, lo, hi);
	if (*field == val)
		return false;
	*field = val;
	return true;
}

static unsigned int zenith_tunable_or_local(struct zenith_policy *z_policy,
					    unsigned int tunable,
					    unsigned int local)
{
	return z_policy->at_local_actions ? local : tunable;
}

/* zenith_glide_value - auto_tune_v2_glides accessor for round-U-z10 knobs.
 *
 * Semantics (different from zenith_tunable_or_local!):
 *   - If user wrote a non-zero tunable value, that wins outright --
 *     always, regardless of whether glides are active.
 *   - If the user value is 0 (the default for all seven glide knobs)
 *     AND auto_tune_v2_glides is on AND the V2 worker has run at
 *     least once (at_local_glides_active == true), return the
 *     V2-derived value.
 *   - Otherwise return 0 (== legacy behaviour: the consumer's
 *     "feature off" path).
 *
 * Read in the freq-update hot path; the non-zero short-circuit
 * keeps it one branch + one load on the common case (user has
 * tuned the knob explicitly).
 */
static unsigned int zenith_glide_value(struct zenith_policy *z_policy,
				       unsigned int tunable,
				       unsigned int local)
{
	if (tunable)
		return tunable;
	if (READ_ONCE(z_policy->tunables->auto_tune_v2_glides) &&
	    z_policy->at_local_glides_active)
		return local;
	return 0;
}

/* zenith_tier_value - V2 tier-classifier accessor for Patch L knobs.
 *
 * Different shape from zenith_glide_value():
 *
 *   1. User sysfs override always wins.  When the matching bit is
 *      set in tunables->auto_tune_override_mask the read returns
 *      the tunable value verbatim and skips all V2 logic.  Profile
 *      changes clear the entire mask (zenith_apply_profile() at
 *      line ~10630), so a profile flip rearms V2 even after the
 *      user has poked individual tiers.
 *
 *   2. When auto_tune_v2_tiers is 0 OR at_local_tiers_active is
 *      false the read returns the tunable verbatim -- this is the
 *      pre-Patch-L code path, byte-identical.
 *
 *   3. Otherwise (V2 active, tier mask current) the read returns
 *      the tunable iff the matching tier bit is set in the V2's
 *      armed mask, else 0.  "Disarmed" means "treat as off for
 *      this V2 window" -- a knob the user enabled via profile but
 *      that V2 has decided is not appropriate for the current
 *      classified state.
 *
 * Read in the eval hot path; the override-mask short-circuit is
 * one branch + one load on the common case (no override).
 */
static unsigned int zenith_tier_value(struct zenith_policy *z_policy,
				      unsigned int tunable,
				      unsigned long override_bit,
				      unsigned long tier_bit)
{
	struct zenith_tunables *t = z_policy->tunables;

	if (t->auto_tune_override_mask & override_bit)
		return tunable;
	if (!READ_ONCE(t->auto_tune_v2_tiers))
		return tunable;
	if (!z_policy->at_local_tiers_active)
		return tunable;
	return (z_policy->at_local_tier_armed_mask & tier_bit) ?
		tunable : 0;
}

static void zenith_at_write_effective(struct zenith_policy *z_policy,
				      struct zenith_at_guardrails *g,
				      unsigned int up_rate,
				      unsigned int down_rate,
				      unsigned int up_th,
				      unsigned int down_th,
				      unsigned int boost_ms,
				      unsigned int boost_cap,
				      unsigned int down_adapt,
				      unsigned int down_th_adapt,
				      unsigned int frame_floor,
				      unsigned int game_mode)
{
	struct zenith_tunables *t = z_policy->tunables;

	z_policy->at_effective_up_rate_limit_us =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_UP_RATE) ?
		t->up_rate_limit_us : zenith_at_clamp(up_rate,
						      g->up_rate_min,
						      g->up_rate_max);
	z_policy->at_effective_down_rate_limit_us =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_DOWN_RATE) ?
		t->down_rate_limit_us : zenith_at_clamp(down_rate,
							g->down_rate_min,
							g->down_rate_max);
	z_policy->at_effective_up_threshold =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_UP_THRESHOLD) ?
		t->up_threshold : zenith_at_clamp(up_th,
						  g->up_threshold_min,
						  g->up_threshold_max);
	z_policy->at_effective_down_threshold =
		(t->auto_tune_override_mask &
		 ZENITH_AT_OVERRIDE_DOWN_THRESHOLD) ?
		t->down_threshold : zenith_at_clamp(down_th,
						    g->down_threshold_min,
						    g->down_threshold_max);
	z_policy->at_effective_input_boost_ms =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_INPUT_BOOST_MS) ?
		t->input_boost_ms : zenith_at_clamp(boost_ms,
						    g->input_boost_min,
						    g->input_boost_max);
	z_policy->at_effective_input_boost_cap_pct =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP) ?
		t->input_boost_cap_pct : zenith_at_clamp(boost_cap,
							 g->input_cap_min,
							 g->input_cap_max);
	z_policy->at_effective_down_rate_adaptive =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE) ?
		t->down_rate_adaptive : zenith_at_clamp(down_adapt,
							g->down_adaptive_min,
							g->down_adaptive_max);
	z_policy->at_effective_down_threshold_adaptive =
		(t->auto_tune_override_mask &
		 ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE) ?
		t->down_threshold_adaptive :
		zenith_at_clamp(down_th_adapt, g->down_thresh_adaptive_min,
				g->down_thresh_adaptive_max);
	z_policy->at_effective_frame_pace_floor_pct =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_FRAME_PACE) ?
		t->frame_pace_floor_pct : zenith_at_clamp(frame_floor,
							  g->frame_floor_min,
							  g->frame_floor_max);
	z_policy->at_effective_game_mode =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_GAME_MODE) ?
		t->game_mode : zenith_at_clamp(game_mode, g->game_mode_min,
					       g->game_mode_max);
	z_policy->at_local_actions = true;
	zenith_update_rate_delay_ns(z_policy);
	z_policy->need_freq_update = true;
}

static bool zenith_at_apply_actions(struct zenith_policy *z_policy,
				    unsigned int state)
{
	struct zenith_tunables *t = z_policy->tunables;
	struct zenith_at_guardrails g;
	unsigned int up_rate, down_rate, up_th, down_th;
	unsigned int boost_ms, boost_cap, down_adapt, down_th_adapt;
	unsigned int frame_floor = 0;
	unsigned int game_mode = 0;
	bool rate_changed = false;
	bool changed = false;

	zenith_at_get_policy_guardrails(z_policy, t->auto_tune_cluster_aware ?
					zenith_at_profile_for_state(z_policy,
								    state) :
					t->active_profile, &g);
	switch (state) {
	case ZENITH_AT_STATE_EFFICIENCY:
		up_rate = g.up_rate_max;
		down_rate = g.down_rate_min;
		up_th = g.up_threshold_max;
		down_th = g.down_threshold_min;
		boost_ms = g.input_boost_min;
		boost_cap = g.input_cap_min;
		down_adapt = g.down_adaptive_min;
		down_th_adapt = g.down_thresh_adaptive_min;
		frame_floor = g.frame_floor_min;
		game_mode = g.game_mode_min;
		break;
	case ZENITH_AT_STATE_LATENCY:
		up_rate = g.up_rate_min;
		down_rate = (g.down_rate_min + g.down_rate_max) / 2;
		up_th = g.up_threshold_min;
		down_th = g.down_threshold_min;
		boost_ms = g.input_boost_max;
		boost_cap = g.input_cap_max;
		down_adapt = g.down_adaptive_max;
		down_th_adapt = g.down_thresh_adaptive_max;
		frame_floor = (g.frame_floor_min + g.frame_floor_max) / 2;
		game_mode = min_t(unsigned int, g.game_mode_max, 1);
		break;
	case ZENITH_AT_STATE_SUSTAINED_PERF:
		up_rate = g.up_rate_min;
		down_rate = g.down_rate_max;
		up_th = g.up_threshold_min;
		down_th = g.down_threshold_max;
		boost_ms = g.input_boost_max;
		boost_cap = g.input_cap_max;
		down_adapt = g.down_adaptive_max;
		down_th_adapt = g.down_thresh_adaptive_max;
		frame_floor = g.frame_floor_max;
		game_mode = g.game_mode_max;
		break;
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		up_rate = g.up_rate_max;
		down_rate = g.down_rate_min;
		up_th = g.up_threshold_max;
		down_th = g.down_threshold_min;
		boost_ms = g.input_boost_min;
		boost_cap = g.input_cap_min;
		down_adapt = g.down_adaptive_min;
		down_th_adapt = g.down_thresh_adaptive_min;
		frame_floor = g.frame_floor_min;
		game_mode = g.game_mode_min;
		break;
	case ZENITH_AT_STATE_BALANCED:
	default:
		up_rate = (g.up_rate_min + g.up_rate_max) / 2;
		down_rate = (g.down_rate_min + g.down_rate_max) / 2;
		up_th = (g.up_threshold_min + g.up_threshold_max) / 2;
		down_th = (g.down_threshold_min + g.down_threshold_max) / 2;
		boost_ms = (g.input_boost_min + g.input_boost_max) / 2;
		boost_cap = (g.input_cap_min + g.input_cap_max) / 2;
		down_adapt = g.down_adaptive_max;
		down_th_adapt = (g.down_thresh_adaptive_min +
				  g.down_thresh_adaptive_max) / 2;
		frame_floor = (g.frame_floor_min + g.frame_floor_max) / 2;
		game_mode = min_t(unsigned int, g.game_mode_max, 1);
		break;
	}

	if (t->auto_tune_cluster_aware) {
		if (z_policy->cluster_class == ZENITH_CLUSTER_LITTLE) {
			up_rate = max(up_rate, (g.up_rate_min + g.up_rate_max) / 2);
			down_rate = g.down_rate_min;
			boost_ms = min(boost_ms,
				       (g.input_boost_min + g.input_boost_max) / 2);
			frame_floor = min(frame_floor,
					  (g.frame_floor_min + g.frame_floor_max) / 2);
			game_mode = min(game_mode, 1U);
		} else if (z_policy->cluster_class == ZENITH_CLUSTER_PRIME) {
			up_rate = g.up_rate_min;
			if (state == ZENITH_AT_STATE_LATENCY ||
			    state == ZENITH_AT_STATE_SUSTAINED_PERF) {
				boost_ms = g.input_boost_max;
				frame_floor = g.frame_floor_max;
			}
		}
	}

	if (t->auto_tune_frame_pacing && z_policy->at_last_frame_budget_us &&
	    (state == ZENITH_AT_STATE_LATENCY ||
	     state == ZENITH_AT_STATE_SUSTAINED_PERF))
		frame_floor = g.frame_floor_max;
	if (t->auto_tune_sustained_gaming &&
	    state == ZENITH_AT_STATE_SUSTAINED_PERF)
		game_mode = g.game_mode_max;
	if (t->auto_tune_thermal_slope &&
	    (z_policy->at_last_flags & ZENITH_AT_FLAG_THERMAL_SLOPE)) {
		down_rate = g.down_rate_min;
		down_th = g.down_threshold_min;
		boost_ms = g.input_boost_min;
		boost_cap = g.input_cap_min;
		frame_floor = g.frame_floor_min;
		game_mode = g.game_mode_min;
	}

	if (t->auto_tune_cluster_aware) {
		zenith_at_write_effective(z_policy, &g, up_rate, down_rate,
					  up_th, down_th, boost_ms, boost_cap,
					  down_adapt, down_th_adapt,
					  frame_floor, game_mode);
		return true;
	}

	rate_changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_UP_RATE,
					   &t->up_rate_limit_us, up_rate,
					   g.up_rate_min, g.up_rate_max);
	rate_changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_DOWN_RATE,
					   &t->down_rate_limit_us, down_rate,
					   g.down_rate_min, g.down_rate_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_UP_THRESHOLD,
				      &t->up_threshold, up_th,
				      g.up_threshold_min,
				      g.up_threshold_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_DOWN_THRESHOLD,
				      &t->down_threshold, down_th,
				      g.down_threshold_min,
				      g.down_threshold_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_INPUT_BOOST_MS,
				      &t->input_boost_ms, boost_ms,
				      g.input_boost_min, g.input_boost_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP,
				      &t->input_boost_cap_pct, boost_cap,
				      g.input_cap_min, g.input_cap_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE,
				      &t->down_rate_adaptive, down_adapt,
				      g.down_adaptive_min, g.down_adaptive_max);
	changed |= zenith_at_set_uint(t,
				      ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE,
				      &t->down_threshold_adaptive,
				      down_th_adapt,
				      g.down_thresh_adaptive_min,
				      g.down_thresh_adaptive_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_FRAME_PACE,
				      &t->frame_pace_floor_pct, frame_floor,
				      g.frame_floor_min, g.frame_floor_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_GAME_MODE,
				      &t->game_mode, game_mode,
				      g.game_mode_min, g.game_mode_max);
	if (rate_changed)
		zenith_refresh_rate_delays_one(z_policy);
	if (changed || rate_changed) {
		WRITE_ONCE(zenith_input_boost_active_ms, t->input_boost_ms);
		z_policy->need_freq_update = true;
	}
	return changed || rate_changed;
}

/* zenith_at_apply_glides - V2 driver for the round-U-z10 glide knobs.
 *
 * Called from the V2 worker tail (just after zenith_at_apply_actions)
 * when auto_tune_v2 && auto_tune_v2_glides.  Populates per-policy
 * effective copies of the seven glide knobs from the current state +
 * the at_last_flags bitmap so the freq-update hot path can fall
 * through to them when the user-set tunable is 0.
 *
 * Mapping (state -> effective values):
 *
 *   ZENITH_AT_STATE_LATENCY:
 *     brutal_decay_ms       = ZENITH_AT_GLIDE_BRUTAL_DECAY_MS
 *     wakeup_boost_ms       = ZENITH_AT_GLIDE_WAKEUP_BOOST_MS
 *
 *   ZENITH_AT_STATE_THERMAL_RECOVERY:
 *     thermal_pressure_continuous = 1
 *
 *   ZENITH_AT_STATE_EFFICIENCY / BALANCED:
 *     prefer_silver_aware    = 1 (when ps hit-rate >= configured
 *                                 threshold; gated by the existing
 *                                 ps_hit_rate_pct snapshot)
 *
 *   any state with (flags & ZENITH_AT_FLAG_FRAME) || game_mode:
 *     wakeup_boost_ms        = ZENITH_AT_GLIDE_WAKEUP_BOOST_MS
 *     frame_budget_us_auto   = 1 (when zenith_drm_vblank_us != 0)
 *
 *   pressure_pct >= ZENITH_AT_GLIDE_THERMAL_PRESSURE_PCT:
 *     thermal_pressure_continuous = 1
 *
 *   always (independent of state):
 *     screen_off_glide_ms    = ZENITH_AT_GLIDE_SCREEN_OFF_MS
 *     boot_boost_decay_ms    = ZENITH_AT_GLIDE_BOOT_BOOST_DECAY_MS
 *     (these are arm-time / one-shot knobs, not state-dependent)
 *
 * Sets at_local_glides_active to gate consumer fall-through.
 * Cheap: O(1) and writes scalar fields with no locking (single
 * writer in the V2 worker, single readers in the consumer paths
 * under update_lock; staleness cost is bounded by the V2 tick
 * cadence).
 */
static void zenith_at_apply_glides(struct zenith_policy *z_policy,
				   unsigned int state)
{
	unsigned int flags = z_policy->at_last_flags;
	unsigned int pressure_pct = z_policy->at_last_thermal_pressure;
	unsigned int brutal_ms = 0;
	unsigned int wakeup_ms = 0;
	unsigned int thermal_continuous = 0;
	unsigned int prefer_silver_aware_v = 0;
	unsigned int frame_auto_v = 0;

	switch (state) {
	case ZENITH_AT_STATE_LATENCY:
		brutal_ms = ZENITH_AT_GLIDE_BRUTAL_DECAY_MS;
		wakeup_ms = ZENITH_AT_GLIDE_WAKEUP_BOOST_MS;
		break;
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		thermal_continuous = 1;
		break;
	case ZENITH_AT_STATE_EFFICIENCY:
	case ZENITH_AT_STATE_BALANCED:
		if (z_policy->ps_hit_rate_pct >=
		    READ_ONCE(z_policy->tunables->
			      prefer_silver_hot_threshold_pct))
			prefer_silver_aware_v = 1;
		break;
	default:
		break;
	}

	if ((flags & ZENITH_AT_FLAG_FRAME) || (flags & ZENITH_AT_FLAG_GAME)) {
		if (!wakeup_ms)
			wakeup_ms = ZENITH_AT_GLIDE_WAKEUP_BOOST_MS;
		if (atomic_read(&zenith_drm_vblank_us))
			frame_auto_v = 1;
	}

	if (pressure_pct >= ZENITH_AT_GLIDE_THERMAL_PRESSURE_PCT)
		thermal_continuous = 1;

	z_policy->at_local_brutal_decay_ms = brutal_ms;
	z_policy->at_local_wakeup_boost_ms = wakeup_ms;
	z_policy->at_local_boot_boost_decay_ms =
		ZENITH_AT_GLIDE_BOOT_BOOST_DECAY_MS;
	z_policy->at_local_screen_off_glide_ms =
		ZENITH_AT_GLIDE_SCREEN_OFF_MS;
	z_policy->at_local_thermal_pressure_continuous = thermal_continuous;
	z_policy->at_local_prefer_silver_aware = prefer_silver_aware_v;
	z_policy->at_local_frame_budget_us_auto = frame_auto_v;
	z_policy->at_local_glides_active = true;
}

/* Patch L: V2-classifier tier-armer.  Sibling of
 * zenith_at_apply_glides().  Computes which Stage-4 K1/K2/K3
 * floor tiers should be armed for the current state / flag set
 * and stamps the result into z_policy->at_local_tier_armed_mask.
 *
 * Mapping rationale (also documented at the ZENITH_AT_TIER_*
 * defines, kept in sync with zenith_at_apply_glides()'s state
 * switch):
 *
 *   LATENCY: this is the "we want fast wakeups" V2 state.  K1
 *   compensates PELT migration lag on inbound tasks; K2 lifts
 *   to hispeed when sustained CPU pressure says queueing is
 *   real.  Both are fits.  K3 needs vblank events that LATENCY
 *   doesn't necessarily imply -- gated separately on the FRAME
 *   flag below.
 *
 *   FRAME / GAME flags (independent of state): K1 catches the
 *   render-thread scheduling shuffle that scaling between
 *   clusters tends to trigger; K3 catches missed frames once
 *   the panel driver wires zenith_drm_vblank_event().  K2 is
 *   intentionally NOT armed here -- frame work doesn't
 *   correlate with sustained PSI pressure, and the 10s EWMA
 *   would be a cross-talk signal we don't want during gameplay.
 *
 *   EFFICIENCY / BALANCED: the "be conservative on freq" V2
 *   states.  Disarming all three tiers preserves the user's
 *   intent (battery / mid-line behaviour) even when the
 *   profile selected was PERFORMANCE/BALANCED.
 *
 *   THERMAL_RECOVERY: the V2 actions path already pulls down
 *   freq with thermal_pressure_continuous; layering K1/K2/K3
 *   floors on top would fight that.  Disarmed.
 *
 *   SUSTAINED_PERF: the V2 actions path already pins freq
 *   high; K1/K2/K3 floors on top are double-counting (the
 *   resolved freq is already at-or-above any of the floor_pct
 *   ceilings).  Disarmed -- not because it would be wrong but
 *   because it would be redundant.
 *
 * Cheap: O(1), one bitmask write under the policy lock.  Single
 * writer (V2 worker), single reader (eval path under
 * update_lock).
 */
static void zenith_at_apply_tiers(struct zenith_policy *z_policy,
				  unsigned int state)
{
	unsigned int flags = z_policy->at_last_flags;
	unsigned long armed = 0;

	switch (state) {
	case ZENITH_AT_STATE_LATENCY:
		armed |= ZENITH_AT_TIER_MIGRATION;
		armed |= ZENITH_AT_TIER_PSI_CPU_FLOOR;
		break;
	case ZENITH_AT_STATE_EFFICIENCY:
	case ZENITH_AT_STATE_BALANCED:
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		/* Patch M1: PSI-mem light cap.  Three states where
		 * "back off on memstall" is the policy's job, not a
		 * regression: EFFICIENCY explicitly trades freq for
		 * energy; BALANCED is the all-rounder default;
		 * THERMAL_RECOVERY is already in cool-down.
		 */
		armed |= ZENITH_AT_TIER_PSI_MEM_CAP;
		break;
	default:
		break;
	}

	if (flags & (ZENITH_AT_FLAG_FRAME | ZENITH_AT_FLAG_GAME)) {
		armed |= ZENITH_AT_TIER_MIGRATION;
		armed |= ZENITH_AT_TIER_FRAME_OVERRUN;
		/* Patch M1: frame-pacing and game overrides need full
		 * headroom on the cap side, so unconditionally clear
		 * the PSI-mem cap bit even if the underlying state
		 * had armed it.  (BALANCED + FRAME flag is the common
		 * case here.)  An explicit clear, not a "reset armed",
		 * so other tiers stay armed.
		 */
		armed &= ~ZENITH_AT_TIER_PSI_MEM_CAP;
	}

	z_policy->at_local_tier_armed_mask = armed;
	z_policy->at_local_tiers_active = true;
}


/* Audit fix M6.  Bounded variant of ZENITH_TUNABLE_UINT.  Max value
 * is taken from a user-supplied compile-time constant so the cap is
 * documented in the same place the macro is invoked.  Compare to
 * the open-coded "kstrtouint(...) || val > FOO" pattern used by
 * iowait_boost_min, iowait_stack_pct, etc.
 */
#define ZENITH_TUNABLE_UINT_MAX(_name, _max) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	return sysfs_emit(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val) || val > (_max)) \
		return -EINVAL; \
	t->_name = val; \
	return count; \
} \
static struct governor_attr _name = __ATTR_RW(_name)

/* Same as ZENITH_TUNABLE_UINT but invalidates the per-policy freq
 * cache after the write so the new value takes effect on the next
 * scheduler tick. Use this for fields that feed into the
 * zenith_get_next_freq() decision.
 */

/* Audit fix M6.  Bounded variant of ZENITH_TUNABLE_UINT_INVAL for
 * boolean-style tunables (0 or 1 only).  Without the bound the
 * previous template would happily store UINT_MAX into a "screen on?"
 * field; downstream code uses the value with `if (t->screen_state)`
 * so any nonzero won the test, but writing a large number out via
 * the show() path then re-reading produced a confusing audit trail.
 */
#define ZENITH_TUNABLE_UINT_BOOL_INVAL(_name) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	return sysfs_emit(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val) || val > 1) \
		return -EINVAL; \
	t->_name = val; \
	zenith_invalidate_cache(attr_set); \
	return count; \
} \
static struct governor_attr _name = __ATTR_RW(_name)

/* Combined ZENITH_TUNABLE_UINT_MAX + ZENITH_TUNABLE_UINT_INVAL: clamp
 * the stored value at _max and invalidate the per-policy freq cache
 * so the new value takes effect on the next scheduler tick.  Use for
 * fields that have a non-trivial upper bound *and* feed into the
 * zenith_get_next_freq() decision (e.g. hispeed_freq, light_load_freq,
 * climb_mode, powersave_bias).
 */
#define ZENITH_TUNABLE_UINT_MAX_INVAL(_name, _max) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	return sysfs_emit(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val) || val > (_max)) \
		return -EINVAL; \
	t->_name = val; \
	zenith_invalidate_cache(attr_set); \
	return count; \
} \
static struct governor_attr _name = __ATTR_RW(_name)
static unsigned int auto_profile_predict __read_mostly;

/* auto_profile_predict is a file-scope global; sysfs show/store not wired. */
ZENITH_TUNABLE_UINT_BOOL_INVAL(io_is_busy);

static ssize_t iowait_boost_min_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->iowait_boost_min);
}

static ssize_t iowait_boost_min_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	/* 0..1000 permille of SCHED_CAPACITY_SCALE. >1000 would overshoot
	 * capacity on the first arm, which is never what we want.
	 */
	if (kstrtouint(buf, 10, &val) || val > 1000)
		return -EINVAL;
	t->iowait_boost_min = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr iowait_boost_min = __ATTR_RW(iowait_boost_min);

ZENITH_TUNABLE_UINT_MAX(iowait_stack_pct, 100);

static ssize_t iowait_backoff_after_ms_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->iowait_backoff_after_ms);
}

static ssize_t iowait_backoff_after_ms_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	/* 0 disables the backoff.  Cap at 60_000 ms (1 minute): an
	 * iowait episode lasting that long without a single quiet tick
	 * is exotic enough that the user almost certainly wants the
	 * backoff to kick in well before then; values above the cap
	 * are rejected so a typo doesn't silently disable the feature
	 * for hours.
	 */
	if (kstrtouint(buf, 10, &val) || val > 60000)
		return -EINVAL;
	t->iowait_backoff_after_ms = val;
	return count;
}
static struct governor_attr iowait_backoff_after_ms =
	__ATTR_RW(iowait_backoff_after_ms);

ZENITH_TUNABLE_UINT_BOOL_INVAL(ignore_nice_load);

/* Apply one of the preset recipes to all tunables in-place. Leaves
 * light_load_freq, hispeed_freq, efficient_freq ladder and other
 * device-specific frequencies untouched because their correct values
 * depend on the SoC's actual freq table. The user can layer those on
 * top after picking a profile.
 *
 * Profile design intent (re-derived from the four preset tables
 * below; all values that don't appear in the per-profile table fall
 * back to the module-level ZENITH_DEFAULT_* values).  This block is
 * the one place that documents each profile's stance per tier;
 * individual tunable comments and tracepoints describe the mechanic,
 * but the per-profile *values* belong here so a reader picking a
 * profile can see the whole budget at once.
 *
 *   PERFORMANCE  Aggressive everywhere.  up_threshold=65, hispeed
 *                tier engages at load 55, climb mode SNAP, freq_step
 *                15 %% per tick, no powersave bias, input boost
 *                150 ms with full decay tail, peak-headroom rescue
 *                AND prearm both on, render-floor and frame-overrun
 *                tiers tuned tighter, peer-ramp window 80 ms with
 *                screen-off variant disabled, predictive up tier
 *                tighter, PSI floors armed but with hispeed-headroom
 *                guards.  Tradeoff: best burst latency, highest
 *                steady-state power.  Preferred for scenario flips
 *                triggered by camera_aware / render_aware / sustained
 *                game-mode.
 *
 *   BALANCED     The default.  up_threshold ~75, hispeed tier at
 *                load 65, climb mode SNAP, freq_step 10 %%, mild
 *                powersave bias, input boost 100 ms with shorter
 *                decay, peak-headroom rescue on but prearm gated by
 *                hysteresis streak, peer-ramp window 50 ms with a
 *                short screen-off variant, frame-overrun deep tier
 *                gated by streak.  Tradeoff: best 95th-pct latency
 *                while keeping idle power close to BATTERY.
 *                Preferred for the audio-detect scenario flip.
 *
 *   BATTERY      Conservative everywhere it matters for residency.
 *                up_threshold raised, climb mode STEP not SNAP,
 *                freq_step ~5-7 %%, larger powersave bias, input
 *                boost shorter, peak-headroom rescue off (only
 *                prearm fires), render-floor gated by min runtime,
 *                peer-ramp window ~30 ms (just long enough to catch
 *                a real cluster wake without paying for noise),
 *                frame-overrun off, PSI memstall cap armed
 *                aggressively to throttle when memstall is the
 *                bottleneck.  Tradeoff: lowest screen-off and idle
 *                power, deeper drops on bursty foreground work.
 *                Preferred for memstall-detected scenario flip
 *                (audio + memstall + screen-off all push toward
 *                BATTERY).
 *
 *   LEGACY       Bug-for-bug compatible with pre-V4 zenith.  All
 *                post-V4 tiers (peak-headroom prearm, peer-ramp,
 *                migration-floor, frame-overrun, PSI floors,
 *                render-floor, predict-up) gated to 0 so the
 *                runtime path matches the original schedutil-plus-
 *                ondemand-plus-reflex hybrid.  Useful as a
 *                regression baseline and for users who have an
 *                existing tuning workflow that depends on the V4
 *                tiers being silent.
 *
 *   CUSTOM       (not in this table)  No profile applied.  Every
 *                tunable carries whatever sysfs left it at, which
 *                in practice is the ZENITH_DEFAULT_* values from
 *                the top of this file unless the user overwrote
 *                them.  This is the value reported by the profile
 *                sysfs node when no preset has ever been applied.
 *
 * Adding a new tunable that should track the profile: add a field
 * to struct zenith_profile_defaults below, fill in the value in all
 * four preset tables, and copy it from p->foo to t->foo at the end
 * of this function.  Forgetting any of those three steps will leave
 * the field at its module default for that profile, which is
 * almost always wrong.
 *
 * Profile-baked vs. global-only tunable split (audited 2026-05-07,
 * zenith-tunables-audit).  struct zenith_tunables exposes ~180
 * tunables; this function bakes the subset whose semantics differ
 * by user-picked profile.  Everything else is "global-only": set
 * once at zenith_create_tunables_data() time from the
 * ZENITH_DEFAULT_* constants and only mutated by direct sysfs
 * writes (no profile path touches them).
 *
 * BAKED (81 fields, in the same order as struct
 * zenith_profile_defaults below).  Adding a field here without
 * updating all four preset tables triggers the warning above; a
 * static assert is not feasible because the struct is anonymous
 * inside this function.
 *
 *   V1 base (legacy schedutil-plus tier, 24 fields):
 *     up_rate_limit_us, down_rate_limit_us, up_threshold,
 *     down_threshold, hispeed_freq_pct, hispeed_load, climb_mode,
 *     freq_step_pct, powersave_bias, bias_load_threshold,
 *     ignore_nice_load, input_boost_ms, input_boost_decay_ms,
 *     input_boost_cap_pct, light_load_threshold,
 *     sampling_down_factor, thermal_auto, screen_auto,
 *     util_math_v2, kcpustat_hispeed_enable, down_rate_adaptive,
 *     wakeup_boost, down_threshold_adaptive,
 *     rate_limit_cluster_scale.
 *
 *   Stage 1/2 (peak-headroom + scenario shaping, 17 fields):
 *     peak_headroom_rescue, peak_headroom_prearm,
 *     peak_headroom_starve_load_pct,
 *     peak_headroom_freq_floor_pct,
 *     peak_headroom_starve_streak, peak_headroom_jump_pct,
 *     peak_headroom_hold_ms, batt_hold_scale_pct,
 *     cluster_wake_pulse_ms, cluster_wake_pulse_idle_ms,
 *     cluster_wake_pulse_floor_pct, quiet_hours_cap_pct,
 *     quiet_hours_screen_off_only, fg_transition_pulse_ms,
 *     fg_transition_pulse_pct, screen_on_bias_pct,
 *     input_boost_down_rate_mult_pct.
 *
 *   Stage 4 / Stage 5 (predict + rising-edge + DL + IO + render
 *   + peak-hyst + boost-idle + bg-util + sleeper + peer-ramp +
 *   migration-floor + PSI-CPU floor + frame-overrun + PSI-mem
 *   cap, 32 fields):
 *     predict_up_thresh, predict_up_window, pelt_rising_edge_thresh,
 *     pelt_rising_edge_min_pct, dl_task_floor_pct,
 *     io_floor_hyst_ms, io_floor_hyst_pct, render_floor_pct,
 *     render_floor_min_runtime_ms, input_boost_touchdown_extra_ms,
 *     peak_hysteresis_streak, peak_step_down_pct,
 *     boost_idle_thresh, boost_idle_streak, bg_util_scale_pct,
 *     sleeper_tail_thresh_us, sleeper_tail_pct,
 *     peer_ramp_window_ms, peer_ramp_floor_pct,
 *     peer_ramp_window_off_ms, migration_jump_pct,
 *     migration_floor_window_ms, migration_floor_pct,
 *     psi_cpu_floor_thresh, frame_overrun_slack_us,
 *     frame_overrun_window_ms, frame_overrun_floor_pct,
 *     frame_overrun_deep_streak, frame_overrun_deep_floor_pct,
 *     psi_mem_cap_thresh, psi_mem_cap_pct, psi_mem_cap_window_ms.
 *
 *   Audio + vendor-hooks (7 fields):
 *     audio_hyst_ms, vh_arch_freq_scale_enable,
 *     vh_uclamp_observer_enable, vh_cpu_idle_enable,
 *     vh_freq_qos_enable, vh_sched_move_task_enable,
 *     vh_scheduler_tick_enable.
 *
 *   Patch B10-3 (1 field):
 *     psi_cgroup_path.
 *
 * GLOBAL-ONLY (~100 fields).  Not enumerated individually; the
 * categories are:
 *
 *   - User intent / opt-in flags whose semantics are device-wide
 *     and not per-profile: audio_aware, render_aware, camera_aware,
 *     psi_aware, game_auto, auto_tune_v2,
 *     auto_tune_v3 (all of the seven static-key-gated aware-flags
 *     and tier switches plus auto_tune_cluster_aware,
 *     auto_tune_v2_signals, auto_tune_frame_pacing,
 *     auto_tune_sustained_gaming).
 *
 *   - Mode-style scalars whose semantics are device-wide:
 *     thermal_state, thermal_active, thermal_aware, screen_state,
 *     freq_step_adaptive, climb_mode_brutal_*, profile (the
 *     reported profile node itself), profile_auto.
 *
 *   - Hardware-shaped freqs that depend on the SoC's freq table,
 *     not the user's intent: hispeed_freq, light_load_freq,
 *     efficient_freq[N], eff_bin_*, up_delay_us.
 *
 *   - Auto-tune V2/V3 internals that the auto-tune state machine
 *     manages directly (auto_tune_eval_ms,
 *     auto_tune_hysteresis_ms, auto_tune_hysteresis_windows,
 *     auto_tune_cooldown_windows, auto_tune_v2_var_promote_thresh,
 *     auto_tune_util_rising_thresh_pct,
 *     auto_tune_render_rt_floor_pct, auto_tune_v3_interval_ms,
 *     auto_tune_v3_state, auto_tune_thermal_slope,
 *     auto_tune_thermal_pressure_pct,
 *     auto_tune_thermal_slope_pct, auto_tune_sat_load_pct,
 *     auto_tune_hi_sat_pct, auto_tune_lo_sat_pct,
 *     auto_tune_hi_events_x2, auto_tune_lo_events_x2,
 *     auto_tune_scenario, all of the at_log_* sysfs RO mirrors).
 *
 *   - PSI thresholds and adaptive-up internals that already key off
 *     other profile-baked tiers: psi_mem_thresh, psi_cpu_thresh,
 *     psi_io_thresh, up_threshold_adaptive (the adaptive value, as
 *     opposed to the on/off knob which is profile-baked).
 *
 *   - Boot-time and frame-budget knobs that are global by design:
 *     boot_boost_ms, boot_boost_decay_ms, boot_complete_auto,
 *     frame_budget_us, frame_budget_us_auto, frame_pace_floor_pct,
 *     fg_*_pct ratios that are not in the bake set above.
 *
 *   - Boost / wakeup secondary knobs whose primary on/off lives in
 *     the bake set: input_boost_decay_curve, input_boost_big_only,
 *     wakeup_boost_ms, brutal_decay_ms, hispeed_*_streak,
 *     hispeed_hyst_pct, hispeed_entry_streak, freq_stability_-
 *     margin_pct, sampling_down_factor's helpers (already-baked
 *     factor itself is the bake-set entry), boost_exit_extend.
 *
 *   - Stats / observability: zenith_stats_*, *_active mirrors
 *     written from the runtime path, prefer_silver_*_pct, and the
 *     prefer_silver_hot_threshold_pct / prefer_silver_hot_bump_pct
 *     pair (the on/off prefer_silver_aware lives in the aware-flag
 *     opt-in group above).
 *
 * Rule of thumb for new tunables: if the right value depends on
 * what the user picked in /sys/.../zenith/profile, add it to the
 * bake set below.  If the right value depends on the device's
 * hardware (freq table, cluster topology) or on the user's intent
 * to enable an opt-in detection path (an aware-flag), keep it
 * global-only.  Default values for both groups belong in the
 * ZENITH_DEFAULT_* block at the top of this file; the four preset
 * tables only need to override values that differ from the
 * BALANCED row, but the current convention is to fill every cell
 * for readability.
 */

static void zenith_auto_predict_tick(struct zenith_policy *z_policy);
static void zenith_apply_profile(struct zenith_tunables *t, unsigned int prof);

/*
 * Lightweight workload classifier
 * Called from the governor tick when auto_profile_predict is set.
 * Reads the max-cluster util_avg, classifies into a band, and
 * switches profiles after 4 consecutive same-band samples.
 * Experimental: off by default, enable via sysfs.
 */
static void zenith_auto_predict_tick(struct zenith_policy *z_policy)
{
	unsigned long max_util = 0;
	int cpu;
	int target_profile;
	static unsigned int settle_count;
	static int last_band = -1;
	int band;

	if (unlikely(!READ_ONCE(auto_profile_predict)))
		return;
	if (unlikely(!z_policy))
		return;

	/* Find max util across all CPUs in the policy. */
	for_each_cpu(cpu, z_policy->policy->cpus) {
		struct rq *rq = cpu_rq(cpu);
		unsigned long util = READ_ONCE(rq->cfs.avg.util_avg);

		if (util > max_util)
			max_util = util;
	}

	/* Classify into a band. */
	if (max_util < SCHED_CAPACITY_SCALE * 35 / 100)
		band = 0;  /* idle/light */
	else if (max_util <= SCHED_CAPACITY_SCALE * 65 / 100)
		band = 1;  /* medium */
	else if (max_util <= SCHED_CAPACITY_SCALE * 85 / 100)
		band = 2;  /* heavy */
	else
		band = 3;  /* sustained */

	/* Update settle count. */
	if (band == last_band) {
		if (settle_count < 4)
			settle_count++;
	} else {
		last_band = band;
		settle_count = 1;
		return;
	}

	if (settle_count < 4)
		return;

	/* Map band to profile. */
	switch (band) {
	case 0:
		target_profile = ZENITH_PROFILE_BATTERY;
		break;
	case 1:
		target_profile = ZENITH_PROFILE_BALANCED;
		break;
	case 2:
		target_profile = ZENITH_PROFILE_PERFORMANCE;
		break;
	case 3:
		target_profile = ZENITH_PROFILE_GAMING;
		break;
	default:
		return;
	}

	if (target_profile != READ_ONCE(z_policy->tunables->active_profile)) {
		zenith_apply_profile(z_policy->tunables, target_profile);
		pr_info_ratelimited("zenith: auto-predict switched to profile %d\n",
			    target_profile);
	}
}

static void zenith_apply_profile(struct zenith_tunables *t, unsigned int prof)
{
	struct zenith_profile_defaults {
		unsigned int profile;
		unsigned int up_rate_limit_us;
		unsigned int down_rate_limit_us;
		unsigned int up_threshold;
		unsigned int down_threshold;
		unsigned int hispeed_freq_pct;
		unsigned int hispeed_load;
		unsigned int climb_mode;
		unsigned int freq_step_pct;
		unsigned int powersave_bias;
		unsigned int bias_load_threshold;
		unsigned int ignore_nice_load;
		unsigned int input_boost_ms;
		unsigned int input_boost_decay_ms;
		unsigned int input_boost_cap_pct;
		unsigned int light_load_threshold;
		unsigned int sampling_down_factor;
		unsigned int thermal_auto;
		unsigned int screen_auto;
		unsigned int util_math_v2;
		unsigned int kcpustat_hispeed_enable;
		unsigned int down_rate_adaptive;
		unsigned int wakeup_boost;
		unsigned int down_threshold_adaptive;
		unsigned int rate_limit_cluster_scale;
		/* Stage 1 / Stage 2 additions, profile-driven so users
		 * never have to touch them: scenario-detected profile
		 * flips (camera/render -> PERFORMANCE, memstall ->
		 * BATTERY, audio -> BALANCED, game-mode sustained ->
		 * PERFORMANCE, thermal/screen-off/PSI -> ...) all
		 * re-apply this table via zenith_apply_profile().
		 */
		unsigned int peak_headroom_rescue;
		unsigned int peak_headroom_prearm;
		unsigned int peak_headroom_starve_load_pct;
		unsigned int peak_headroom_freq_floor_pct;
		unsigned int peak_headroom_starve_streak;
		unsigned int peak_headroom_jump_pct;
		unsigned int peak_headroom_hold_ms;
		unsigned int batt_hold_scale_pct;
		unsigned int cluster_wake_pulse_ms;
		unsigned int cluster_wake_pulse_idle_ms;
		unsigned int cluster_wake_pulse_floor_pct;
		unsigned int quiet_hours_cap_pct;
		unsigned int quiet_hours_screen_off_only;
		unsigned int fg_transition_pulse_ms;
		unsigned int fg_transition_pulse_pct;
		unsigned int screen_on_bias_pct;
		unsigned int input_boost_down_rate_mult_pct;
		unsigned int predict_up_thresh;
		unsigned int predict_up_window;
		unsigned int pelt_rising_edge_thresh;
		unsigned int pelt_rising_edge_min_pct;
		unsigned int dl_task_floor_pct;
		unsigned int io_floor_hyst_ms;
		unsigned int io_floor_hyst_pct;
		unsigned int render_floor_pct;
		unsigned int render_floor_min_runtime_ms;
		unsigned int input_boost_touchdown_extra_ms;
		unsigned int peak_hysteresis_streak;
		unsigned int peak_step_down_pct;
		unsigned int boost_idle_thresh;
		unsigned int boost_idle_streak;
		unsigned int bg_util_scale_pct;
		unsigned int sleeper_tail_thresh_us;
		unsigned int sleeper_tail_pct;
		unsigned int peer_ramp_window_ms;
		unsigned int peer_ramp_floor_pct;
		unsigned int peer_ramp_window_off_ms;
		unsigned int migration_jump_pct;
		unsigned int migration_floor_window_ms;
		unsigned int migration_floor_pct;
		unsigned int psi_cpu_floor_thresh;
		unsigned int frame_overrun_slack_us;
		unsigned int frame_overrun_window_ms;
		unsigned int frame_overrun_floor_pct;
		unsigned int frame_overrun_deep_streak;
		unsigned int frame_overrun_deep_floor_pct;
		unsigned int psi_mem_cap_thresh;
		unsigned int psi_mem_cap_pct;
		unsigned int psi_mem_cap_window_ms;
		unsigned int audio_hyst_ms;
		unsigned int vh_arch_freq_scale_enable;
		unsigned int vh_uclamp_observer_enable;
		unsigned int vh_cpu_idle_enable;
		unsigned int vh_freq_qos_enable;
		unsigned int vh_sched_move_task_enable;
		unsigned int vh_scheduler_tick_enable;
		/* Patch B10-3: per-profile cgroup-v2 path the
		 * zenith_psi_*_some_pct() helpers should read from.
		 * NULL or "" means "use system-wide PSI" (the safe
		 * default and the pre-B10 behaviour); a non-empty
		 * cgroup-v2 path is resolved + cached at apply time
		 * via zenith_psi_cgroup_apply().  Profile-bake here
		 * is overridable per policy via the psi_cgroup_path
		 * sysfs node.
		 */
		const char *psi_cgroup_path;
	};
	static const struct zenith_profile_defaults profiles[] = {
		{
			.profile = ZENITH_PROFILE_PERFORMANCE,
			.up_rate_limit_us = 0,
			.down_rate_limit_us = 8000,
			.up_threshold = 65,
			.down_threshold = 45,
			.hispeed_freq_pct = 60,
			.hispeed_load = 55,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = 15,
			.powersave_bias = 0,
			.bias_load_threshold = 50,
			.ignore_nice_load = 0,
			.input_boost_ms = 150,
			.input_boost_decay_ms = 50,
			.input_boost_cap_pct = 0,
			.light_load_threshold = 15,
			.sampling_down_factor = 4,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 1,
			.down_rate_adaptive = 1,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 10,
			.rate_limit_cluster_scale = 1,
			/* Stage 1/2: aggressive rescue, no bias on
			 * screen, 3x down-rate during input boost.
			 */
			.peak_headroom_rescue = 1,
			.peak_headroom_prearm = 1,
			.peak_headroom_starve_load_pct = 88,
			.peak_headroom_freq_floor_pct = 80,
			.peak_headroom_starve_streak = 2,
			.peak_headroom_jump_pct = 100,
			.peak_headroom_hold_ms = 25,
			.batt_hold_scale_pct = 100,
			.cluster_wake_pulse_ms = 80,
			.cluster_wake_pulse_idle_ms = 60,
			.cluster_wake_pulse_floor_pct = 70,
			.quiet_hours_cap_pct = 100,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms = 50,
			.fg_transition_pulse_pct = 75,
			.screen_on_bias_pct = 0,
			.input_boost_down_rate_mult_pct = 300,
			/* Stage 4 / Patch A: PERFORMANCE wants eager
			 * prediction.  Lower thresh (48 vs 64 default)
			 * fires the lift on smaller rises; window stays
			 * at the default (4) so the trend is computed
			 * over the same warm-up period.
			 */
			.predict_up_thresh = 48,
			.predict_up_window = 3,
			/* Patch C3: PERFORMANCE wants the rising-edge
			 * tier to fire on smaller per-sample slopes
			 * (24 vs 32 default) and from a lower absolute
			 * level (40%% of max_cap vs 50%% default), so
			 * a fresh foreground task barely starts the
			 * rise and we already have eff_hispeed under
			 * it.  Effort budget: bigger battery hit if a
			 * jitter sample fires the lift, but PERF is
			 * the profile that pays that cost willingly.
			 */
			.pelt_rising_edge_thresh = 24,
			.pelt_rising_edge_min_pct = 40,
			/* Patch C6: PERFORMANCE pins to policy->max
			 * the moment any DL task is detected.  Frame-
			 * work / kernel timers using SCHED_DEADLINE
			 * (e.g. PipeWire RT, V4L2 streamer threads on
			 * the BIG cluster) get a hard freq floor so
			 * the first wake doesn't miss its deadline.
			 */
			.dl_task_floor_pct = 100,
			/* Patch C9: PERFORMANCE holds a 70%% floor for
			 * 500 ms past the last iowait sample.  Block
			 * IO bursts on PERF (e.g. apt-update, large
			 * file copies) carry the freq instead of
			 * collapsing into the level signal between
			 * batches.
			 */
			.io_floor_hyst_ms = 500,
			.io_floor_hyst_pct = 70,
			/* Stage 4 / Patch B: PERFORMANCE keeps the
			 * render floor strong (80%% of max, vs 70%%
			 * default) and tightens the debounce to 20 ms
			 * so frame deadlines aren't lost to a slow
			 * floor-arm on transient render activity.
			 */
			.render_floor_pct = 80,
			.render_floor_min_runtime_ms = 20,
			/* Stage 4 / Patch C: PERFORMANCE adds an extra
			 * 80 ms on touchdown -- the user-perceived
			 * latency from finger-down to first frame is
			 * what makes a phone "feel snappy".
			 */
			.input_boost_touchdown_extra_ms = 80,
			/* Stage 4 / Patch E: PERFORMANCE pins the soft
			 * floor higher (97% of anchor) and holds it
			 * for 4 samples.  The cluster paid for the
			 * peak; an extra few ms of high freq is cheap
			 * compared to bouncing through a frame deadline
			 * because EAS plunged on a single low sample.
			 */
			.peak_hysteresis_streak = 4,
			.peak_step_down_pct = 97,
			/* Stage 4 / Patch F: PERFORMANCE disables the
			 * boost early-exit.  The whole point of the
			 * profile is to honour every UX boost in full;
			 * a few ticks of below-threshold load inside a
			 * boost window is not a reason to drop the
			 * cluster off the boost ceiling.
			 */
			.boost_idle_thresh = 0,
			.boost_idle_streak = 0,
			/* Stage 4 / Patch G: PERFORMANCE keeps full util
			 * even when the screen is off.  This profile is
			 * for users who want maximum responsiveness on
			 * unlock; trimming background util would slow
			 * the device's recovery from a deep-sleep wake.
			 */
			.bg_util_scale_pct = 100,
			/* Stage 4 / Patch H: PERFORMANCE disables sleeper-
			 * tail shaving so wake-up freq is unshaved.
			 */
			.sleeper_tail_thresh_us = 0,
			.sleeper_tail_pct = 100,
			/* Stage 4 / Patch D: PERFORMANCE widens the peer-
			 * ramp window to 40 ms and lifts the floor to 70%%
			 * of policy->max.  IPC chains under this profile
			 * are typically the latency-sensitive kind --
			 * binder hops on app launch, render -> compositor
			 * -> display -- so giving the peer cluster a
			 * bigger and slightly higher pre-arm is worth the
			 * energy.
			 */
			.peer_ramp_window_ms = 40,
			.peer_ramp_floor_pct = 70,
			/* Stage 5 / Patch M3: PERFORMANCE suppresses
			 * peer_ramp once the screen is off.  The IPC
			 * chains the screen-on PERF override widens for
			 * (compositor / render / input) are inactive
			 * with the display blanked, so even the most
			 * aggressive profile gives back the screen-off
			 * energy.  Set non-zero by sysfs to keep peer-
			 * arming warm during e.g. audio playback.
			 */
			.peer_ramp_window_off_ms = 0,
			/* Stage 4 / Patch K1: PERFORMANCE drops the
			 * jump threshold to 15%% (one of every six
			 * util-percent points instead of every five)
			 * so smaller migration arrivals still trip the
			 * floor, and widens the floor to 70%% over a
			 * 35 ms window.  Symmetric reasoning to the
			 * peer-ramp PERF override above: latency-
			 * sensitive workloads benefit more from
			 * absorbing the PELT-warm-up cost than from
			 * the small energy saving of letting the freq
			 * drift down.
			 */
			.migration_jump_pct = 15,
			.migration_floor_window_ms = 35,
			.migration_floor_pct = 70,
			/* Stage 4 / Patch K2: PERFORMANCE arms the
			 * PSI-CPU sustained-pressure floor at 40%%.
			 * The 10s EWMA at 40+%% means the system has
			 * been queueing for a sustained stretch -- on
			 * a PERF profile that is unambiguously a sign
			 * we should be at hispeed regardless of
			 * what aggregate util is saying.
			 */
			.psi_cpu_floor_thresh = 40,
			/* Stage 4 / Patch K3: PERFORMANCE tightens
			 * the slack to 3000 us (~18%% of a 60 Hz
			 * frame) so smaller misses still trip the
			 * floor, and lifts the floor to 90%% of
			 * policy->max over a 60 ms window.  PERF is
			 * the profile where chasing missed frames is
			 * actually useful; energy spent on a recovery
			 * floor is justified.
			 */
			.frame_overrun_slack_us = 3000,
			.frame_overrun_window_ms = 60,
			.frame_overrun_floor_pct = 90,
			/* Stage 5 / Patch M5: PERFORMANCE arms the K3
			 * deep tier at 2 / 100%%.  Two consecutive
			 * 60 Hz vblank gaps wider than 3 ms slack is
			 * ~33 ms of stutter -- well outside any
			 * plausible single-shot jitter explanation.
			 * Lifting the floor to 100%% of policy->max
			 * for the recovery window is the right
			 * trade for a profile the user has explicitly
			 * picked for max responsiveness.
			 */
			.frame_overrun_deep_streak = 2,
			.frame_overrun_deep_floor_pct = 100,
			/* Stage 5 / Patch M1: PERFORMANCE keeps the
			 * PSI-mem cap off (thresh = 0).  The whole
			 * point of PERF is full headroom; backing off
			 * on memstall is the wrong call here.  The
			 * pct/window are populated with sane values
			 * for forward compatibility if the user
			 * overrides thresh via sysfs.
			 */
			.psi_mem_cap_thresh = 0,
			.psi_mem_cap_pct = 90,
			.psi_mem_cap_window_ms = 1000,
			.audio_hyst_ms = 250,
			.vh_arch_freq_scale_enable = 1,
			.vh_uclamp_observer_enable = 1,
			.vh_cpu_idle_enable = 1,
			.vh_freq_qos_enable = 1,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
		},
		{
			.profile = ZENITH_PROFILE_BALANCED,
			.up_rate_limit_us = ZENITH_DEFAULT_UP_RATE_LIMIT_US,
			.down_rate_limit_us = ZENITH_DEFAULT_DOWN_RATE_LIMIT_US,
			.up_threshold = ZENITH_DEFAULT_UP_THRESHOLD,
			.down_threshold = ZENITH_DEFAULT_DOWN_THRESHOLD,
			.hispeed_freq_pct = ZENITH_DEFAULT_HISPEED_FREQ_PCT,
			.hispeed_load = ZENITH_DEFAULT_HISPEED_LOAD,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = ZENITH_DEFAULT_FREQ_STEP_PCT,
			.powersave_bias = 50,
			.bias_load_threshold = 40,
			.ignore_nice_load = 1,
			.input_boost_ms = ZENITH_DEFAULT_INPUT_BOOST_MS,
			.input_boost_decay_ms = ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS,
			.input_boost_cap_pct = ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT,
			.light_load_threshold = ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD,
			.sampling_down_factor = ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 1,
			.down_rate_adaptive = 1,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 5,
			.rate_limit_cluster_scale = 1,
			/* Stage 1/2: same compile-time defaults as
			 * ZENITH_DEFAULT_*; this is the cold-boot
			 * baseline.
			 */
			.peak_headroom_rescue =
				ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE,
			.peak_headroom_prearm =
				ZENITH_DEFAULT_PEAK_HEADROOM_PREARM,
			.peak_headroom_starve_load_pct =
				ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_LOAD_PCT,
			.peak_headroom_freq_floor_pct =
				ZENITH_DEFAULT_PEAK_HEADROOM_FREQ_FLOOR_PCT,
			.peak_headroom_starve_streak =
				ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_STREAK,
			.peak_headroom_jump_pct =
				ZENITH_DEFAULT_PEAK_HEADROOM_JUMP_PCT,
			.peak_headroom_hold_ms =
				ZENITH_DEFAULT_PEAK_HEADROOM_HOLD_MS,
			.batt_hold_scale_pct = 120,
			.cluster_wake_pulse_ms =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS,
			.cluster_wake_pulse_idle_ms =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_IDLE_MS,
			.cluster_wake_pulse_floor_pct =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_FLOOR_PCT,
			.quiet_hours_cap_pct = 70,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms =
				ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS,
			.fg_transition_pulse_pct =
				ZENITH_DEFAULT_FG_TRANSITION_PULSE_PCT,
			.screen_on_bias_pct =
				ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT,
			.input_boost_down_rate_mult_pct =
				ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT,
			/* Stage 4 / Patch A: BALANCED matches the cold-
			 * boot ZENITH_DEFAULT_* values exactly so a
			 * plain compile produces the same behaviour as
			 * an explicit echo balanced > profile.
			 */
			.predict_up_thresh =
				ZENITH_DEFAULT_PREDICT_UP_THRESH,
			.predict_up_window = ZENITH_DEFAULT_PREDICT_UP_WINDOW,
			/* Patch C3: BALANCED matches cold-boot defaults
			 * (32 thresh, 50%% level gate).
			 */
			.pelt_rising_edge_thresh =
				ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH,
			.pelt_rising_edge_min_pct =
				ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT,
			/* Patch C6: BALANCED matches cold-boot default
			 * (off).  The DL bandwidth math via
			 * schedutil_cpu_util() handles average-throughput
			 * for DL tasks; the floor is an opt-in
			 * responsiveness boost reserved for the
			 * profiles that explicitly want it.
			 */
			.dl_task_floor_pct =
				ZENITH_DEFAULT_DL_TASK_FLOOR_PCT,
			/* Patch C9: BALANCED holds a 50%% floor for
			 * 200 ms.  Default-magnitude responsiveness
			 * for moderate IO without paying the energy
			 * frame of PERFORMANCE.
			 */
			.io_floor_hyst_ms = 200,
			.io_floor_hyst_pct = 50,
			/* Stage 4 / Patch B: BALANCED matches cold-
			 * boot defaults (70%% floor, 50 ms debounce).
			 */
			.render_floor_pct =
				ZENITH_DEFAULT_RENDER_FLOOR_PCT,
			.render_floor_min_runtime_ms =
				ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS,
			/* Stage 4 / Patch C: BALANCED matches cold-boot
			 * default (50 ms touchdown extra).
			 */
			.input_boost_touchdown_extra_ms =
				ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS,
			/* Stage 4 / Patch E: BALANCED matches cold-boot
			 * defaults (3-sample streak, 95% step-down).
			 */
			.peak_hysteresis_streak =
				ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK,
			.peak_step_down_pct =
				ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT,
			/* Stage 4 / Patch F: BALANCED matches cold-boot
			 * defaults (15% threshold, 3-tick streak).  This
			 * is the energy-optimisation profile, so trimming
			 * the boost tail when load actually drained makes
			 * sense as a default.
			 */
			.boost_idle_thresh =
				ZENITH_DEFAULT_BOOST_IDLE_THRESH,
			.boost_idle_streak =
				ZENITH_DEFAULT_BOOST_IDLE_STREAK,
			/* Stage 4 / Patch G: BALANCED scales screen-off
			 * util to 75% so background sync work runs on a
			 * lower freq tier while the device is locked.
			 */
			.bg_util_scale_pct = 75,
			/* Stage 4 / Patch H: BALANCED arms sleeper-tail
			 * shaving with a 20 ms idle threshold and a 90%
			 * shave.  Mild trim on the wake-up tick.
			 */
			.sleeper_tail_thresh_us = 20000,
			.sleeper_tail_pct = 90,
			/* Stage 4 / Patch D: BALANCED matches cold-boot
			 * defaults (25 ms window, 60%% floor).  Mild
			 * pre-arm: enough to shave warm-up latency on
			 * common cross-cluster wakes without paying for
			 * a full hispeed pin on every peer event.
			 */
			.peer_ramp_window_ms =
				ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS,
			.peer_ramp_floor_pct =
				ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT,
			/* Stage 5 / Patch M3: BALANCED takes the cold-
			 * boot screen-off default (suppress peer_ramp).
			 * BALANCED is the all-day profile and the screen
			 * spends most of that day off.
			 */
			.peer_ramp_window_off_ms =
				ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS,
			/* Stage 4 / Patch K1: BALANCED matches cold-
			 * boot defaults (20%% jump, 30 ms window, 60%%
			 * floor).  Sensible middle ground: catches the
			 * common one-task-arrived migration without
			 * over-firing on routine util oscillation.
			 */
			.migration_jump_pct =
				ZENITH_DEFAULT_MIGRATION_JUMP_PCT,
			.migration_floor_window_ms =
				ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS,
			.migration_floor_pct =
				ZENITH_DEFAULT_MIGRATION_FLOOR_PCT,
			/* Stage 4 / Patch K2: BALANCED leaves the
			 * PSI-CPU floor disabled (the cold-boot
			 * default).  The energy cost of a hispeed pin
			 * triggered by a smoothed signal isn't worth
			 * paying on the everyday profile -- predict_up
			 * and the existing peak tiers cover the cases
			 * a balanced workload actually needs.
			 */
			.psi_cpu_floor_thresh =
				ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH,
			/* Stage 4 / Patch K3: BALANCED arms the
			 * frame-overrun rescue with 4000 us slack
			 * (~24%% of a 60 Hz frame), 50 ms window,
			 * 80%% floor.  Mid-range slack catches
			 * actual misses without tripping on routine
			 * driver jitter; mid-range floor balances
			 * recovery latency against energy cost.
			 */
			.frame_overrun_slack_us = 4000,
			.frame_overrun_window_ms =
				ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS,
			.frame_overrun_floor_pct =
				ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT,
			/* Stage 5 / Patch M5: BALANCED keeps the deep
			 * tier at the cold-boot off default.  K3 alone
			 * is sufficient for the BALANCED energy /
			 * latency trade-off; users who want the deep
			 * escalation can opt in via sysfs without
			 * switching to PERFORMANCE.
			 */
			.frame_overrun_deep_streak =
				ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK,
			.frame_overrun_deep_floor_pct =
				ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT,
			/* Stage 5 / Patch M1: BALANCED keeps the
			 * PSI-mem cap at the cold-boot off default.
			 * BALANCED's existing tier mapping arms the
			 * tier bit, so a sysfs flip of thresh > 0 is
			 * sufficient to opt in without flipping the
			 * profile.
			 */
			.psi_mem_cap_thresh =
				ZENITH_DEFAULT_PSI_MEM_CAP_THRESH,
			.psi_mem_cap_pct =
				ZENITH_DEFAULT_PSI_MEM_CAP_PCT,
			.psi_mem_cap_window_ms =
				ZENITH_DEFAULT_PSI_MEM_CAP_WINDOW_MS,
			.audio_hyst_ms = ZENITH_DEFAULT_AUDIO_HYST_MS,
			.vh_arch_freq_scale_enable =
				ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE,
			.vh_uclamp_observer_enable =
				ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE,
			.vh_cpu_idle_enable =
				ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE,
			/* See ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE for the
			 * BALANCED-vs-cold-boot rationale: the cold-boot
			 * default is 0 (opt-in), but the BALANCED bake is
			 * explicitly 1 because BALANCED is the AUTO
			 * engine's default resting state, and the AUTO
			 * pivot to PERFORMANCE on QoS pressure is what
			 * makes this observer useful in the first place.
			 */
			.vh_freq_qos_enable = 1,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
		},
		{
			.profile = ZENITH_PROFILE_BATTERY,
			.up_rate_limit_us = 500,
			.down_rate_limit_us = 2000,
			.up_threshold = 85,
			.down_threshold = 40,
			.hispeed_freq_pct = 0,
			.hispeed_load = 75,
			.climb_mode = ZENITH_CLIMB_MODE_STEP,
			.freq_step_pct = 8,
			.powersave_bias = 150,
			.bias_load_threshold = 35,
			.ignore_nice_load = 1,
			.input_boost_ms = 40,
			.input_boost_decay_ms = 10,
			.input_boost_cap_pct = 60,
			.light_load_threshold = 30,
			.sampling_down_factor = 1,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 0,
			.down_rate_adaptive = 0,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 0,
			.rate_limit_cluster_scale = 1,
			/* Stage 1/2: keep the rescue safety net (the
			 * tester's "never reaches peak" complaint
			 * matters even on BATTERY) but disable the
			 * pre-arm and bound the rescue to 90%% of max
			 * to save the peak-freq energy slope.  Looser
			 * starve_streak (5) and longer hold (100ms)
			 * stop rescue churn from waking the cluster
			 * unnecessarily.  Light bias softening (80%%)
			 * keeps screen-on responsive without paying
			 * the BALANCED energy cost.
			 */
			.peak_headroom_rescue = 1,
			.peak_headroom_prearm = 0,
			.peak_headroom_starve_load_pct = 92,
			.peak_headroom_freq_floor_pct = 90,
			.peak_headroom_starve_streak = 5,
			.peak_headroom_jump_pct = 90,
			.peak_headroom_hold_ms = 100,
			.batt_hold_scale_pct = 180,
			.cluster_wake_pulse_ms = 0,
			.cluster_wake_pulse_idle_ms = 0,
			.cluster_wake_pulse_floor_pct = 0,
			.quiet_hours_cap_pct = 55,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms = 0,
			.fg_transition_pulse_pct = 0,
			.screen_on_bias_pct = 80,
			.input_boost_down_rate_mult_pct = 150,
			/* Stage 4 / Patch A: BATTERY disables prediction.
			 * The pre-shift would burn frame-edge energy that
			 * the rest of the BATTERY profile is specifically
			 * trying to avoid, and the level-triggered
			 * hispeed tier is good enough at this freq cap.
			 */
			.predict_up_thresh = 0,
			.predict_up_window = 6,
			/* Patch C3: BATTERY also disables the rising-
			 * edge tier; same energy-frame argument as
			 * predict_up.  The hispeed level tier alone is
			 * the right speed/energy trade for this profile.
			 */
			.pelt_rising_edge_thresh = 0,
			.pelt_rising_edge_min_pct =
				ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT,
			/* Patch C6: BATTERY keeps the floor off; DL
			 * bandwidth math is sufficient for the energy
			 * frame this profile targets.
			 */
			.dl_task_floor_pct = 0,
			/* Patch C9: BATTERY disables IO floor
			 * hysteresis — energy frame.
			 */
			.io_floor_hyst_ms = 0,
			.io_floor_hyst_pct = 0,
			/* Stage 4 / Patch B: BATTERY softens the render
			 * floor to 50%% and stretches the debounce to
			 * 100 ms.  Render activity still floors the
			 * cluster but only after the workload is sticky;
			 * one-shot SurfaceFlinger flushes don't pay the
			 * floor.
			 */
			.render_floor_pct = 50,
			.render_floor_min_runtime_ms = 100,
			/* Stage 4 / Patch C: BATTERY drops the touchdown
			 * extra to 30 ms.  Touchdown latency still gets
			 * a small bonus, but the energy cost of an
			 * 80 ms extra-pin tail is too high for the
			 * battery profile.
			 */
			.input_boost_touchdown_extra_ms = 30,
			/* Stage 4 / Patch E: BATTERY shortens the
			 * streak to 2 samples and steepens the step
			 * down to 90% so the cluster falls off peak
			 * faster.  Hysteresis is still useful (one
			 * sample of low load isn't enough to confirm
			 * the workload is gone) but we don't want to
			 * pay 4 samples of extra freq.
			 */
			.peak_hysteresis_streak = 2,
			.peak_step_down_pct = 90,
			/* Stage 4 / Patch F: BATTERY uses an aggressive
			 * early-exit (25% threshold, 2-tick streak) so
			 * the boost ceiling is dropped as soon as the
			 * launch animation visibly drains.  Combined
			 * with the lower input_boost_ms in this profile
			 * the energy tail of a tap-and-release gesture
			 * is closer to a hard cliff than a 60 ms decay.
			 */
			.boost_idle_thresh = 25,
			.boost_idle_streak = 2,
			/* Stage 4 / Patch G: BATTERY scales screen-off
			 * util to 60% -- aggressive but appropriate
			 * for the battery profile where the user has
			 * already opted in to slower performance.
			 */
			.bg_util_scale_pct = 60,
			/* Stage 4 / Patch H: BATTERY uses an aggressive
			 * 10 ms threshold and 80% shave for maximum
			 * leakage savings.
			 */
			.sleeper_tail_thresh_us = 10000,
			.sleeper_tail_pct = 80,
			/* Stage 4 / Patch D: BATTERY disables peer-ramp.
			 * The user has opted in to slower performance,
			 * and the energy cost of holding the peer at
			 * a 60%%+ floor for 25 ms after every BIG/PRIME
			 * peak is exactly the kind of overhead this
			 * profile exists to avoid.
			 */
			.peer_ramp_window_ms = 0,
			.peer_ramp_floor_pct = 0,
			/* Stage 5 / Patch M3: BATTERY mirrors the
			 * peer_ramp_window_ms = 0 stance for the screen-
			 * off path; redundant given peer_ramp is already
			 * off, but kept explicit so a sysfs tweak that
			 * lifts peer_ramp_window_ms in this profile does
			 * not silently un-suppress the screen-off arm.
			 */
			.peer_ramp_window_off_ms = 0,
			/* Stage 4 / Patch K1: BATTERY disables the
			 * migration-arrival floor for the same reason
			 * peer-ramp is off here.  PELT warm-up
			 * latency is exactly the cost the user is
			 * trading away when picking this profile.
			 */
			.migration_jump_pct = 0,
			.migration_floor_window_ms = 0,
			.migration_floor_pct = 0,
			/* Stage 4 / Patch K2: BATTERY keeps the PSI-CPU
			 * floor disabled.  Sustained pressure under
			 * BATTERY is exactly when the user wants the
			 * governor to *not* lift -- the queueing is
			 * the price of running at conservative freq.
			 */
			.psi_cpu_floor_thresh = 0,
			/* Stage 4 / Patch K3: BATTERY disables the
			 * frame-overrun rescue.  Same trade as
			 * peer-ramp / migration-arrival above:
			 * recovery freq is exactly the cost the
			 * user is opting out of when picking BATTERY.
			 */
			.frame_overrun_slack_us = 0,
			.frame_overrun_window_ms = 0,
			.frame_overrun_floor_pct = 0,
			/* Stage 5 / Patch M5: BATTERY keeps the deep
			 * tier off for the same reason K3 itself is
			 * off here.  Redundant given K3's outer gate
			 * already short-circuits, but kept explicit so
			 * a sysfs tweak that lifts frame_overrun_*
			 * does not silently bring deep along.
			 */
			.frame_overrun_deep_streak = 0,
			.frame_overrun_deep_floor_pct = 100,
			/* Stage 5 / Patch M1: BATTERY arms the PSI-
			 * mem cap aggressively (thresh = 50%, cap to
			 * 70% of policy->max for 1.5 s).  Battery is
			 * the profile where backing off on memstall
			 * matches the user's stated preference: spend
			 * less power on cycles that would just stall
			 * on mm.  The wider window captures full
			 * kswapd / lowmem-killer bursts.
			 */
			.psi_mem_cap_thresh = 50,
			.psi_mem_cap_pct = 70,
			.psi_mem_cap_window_ms = 1500,
			.audio_hyst_ms = 100,
			.vh_arch_freq_scale_enable = 0,
			.vh_uclamp_observer_enable = 0,
			.vh_cpu_idle_enable = 1,
			.vh_freq_qos_enable = 0,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
		},
		{
			.profile = ZENITH_PROFILE_LEGACY,
			.up_rate_limit_us = 2000,
			.down_rate_limit_us = 4000,
			.up_threshold = 80,
			.down_threshold = 80,
			.hispeed_freq_pct = 0,
			.hispeed_load = 90,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = 5,
			.powersave_bias = 0,
			.bias_load_threshold = 50,
			.ignore_nice_load = 1,
			.input_boost_ms = 0,
			.input_boost_decay_ms = 0,
			.input_boost_cap_pct = 0,
			.light_load_threshold = 20,
			.sampling_down_factor = 1,
			.thermal_auto = 0,
			.screen_auto = 0,
			.util_math_v2 = 0,
			.kcpustat_hispeed_enable = 0,
			.down_rate_adaptive = 0,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 0,
			.rate_limit_cluster_scale = 1,
			/* Stage 1/2: LEGACY restores pre-Stage-1
			 * behaviour wholesale.  Rescue and pre-arm
			 * disabled (rescue gate off cancels the entire
			 * peak-headroom path; pre-arm follows for
			 * defence in depth).  screen_on_bias_pct = 100
			 * disables the bias softening (effective bias
			 * == raw bias).  input_boost_down_rate_mult_-
			 * pct = 100 disables the down-rate extension
			 * (effective down_delay == raw down_delay
			 * during boost).  The diagnostic /
			 * sub-rescue knobs are populated with the
			 * loosest values for forward compatibility
			 * (in case rescue is turned back on by sysfs).
			 */
			.peak_headroom_rescue = 0,
			.peak_headroom_prearm = 0,
			.peak_headroom_starve_load_pct = 95,
			.peak_headroom_freq_floor_pct = 95,
			.peak_headroom_starve_streak = 16,
			.peak_headroom_jump_pct = 100,
			.peak_headroom_hold_ms = 200,
			.batt_hold_scale_pct = 100,
			.cluster_wake_pulse_ms = 0,
			.cluster_wake_pulse_idle_ms = 0,
			.cluster_wake_pulse_floor_pct = 0,
			.quiet_hours_cap_pct = 100,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms = 0,
			.fg_transition_pulse_pct = 0,
			.screen_on_bias_pct = 100,
			.input_boost_down_rate_mult_pct = 100,
			/* Stage 4 / Patch A: LEGACY disables prediction
			 * (no Stage 4 features in the historical-
			 * compatibility profile).
			 */
			.predict_up_thresh = 0,
			.predict_up_window =
				ZENITH_DEFAULT_PREDICT_UP_WINDOW,
			/* Patch C3: LEGACY disables the rising-edge
			 * tier as well -- the historical-compat profile
			 * keeps every Patch-C3-and-later tier dormant.
			 */
			.pelt_rising_edge_thresh = 0,
			.pelt_rising_edge_min_pct =
				ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT,
			/* Patch C6: LEGACY also disables the DL floor
			 * (historical-compat profile).
			 */
			.dl_task_floor_pct = 0,
			/* Patch C9: LEGACY off (historical-compat). */
			.io_floor_hyst_ms = 0,
			.io_floor_hyst_pct = 0,
			/* Stage 4 / Patch B: LEGACY disables the floor
			 * outright (render_floor_pct=0) since the floor
			 * is a Stage-1+ feature.  The debounce knob is
			 * left at 0 (no debounce) for forward
			 * compatibility if the user re-enables the
			 * floor through sysfs.
			 */
			.render_floor_pct = 0,
			.render_floor_min_runtime_ms = 0,
			/* Stage 4 / Patch C: LEGACY disables the
			 * touchdown extra (legacy boost cadence
			 * unchanged).
			 */
			.input_boost_touchdown_extra_ms = 0,
			/* Stage 4 / Patch E: LEGACY disables peak-
			 * return hysteresis entirely so the descent
			 * shape is identical to the pre-Stage-4
			 * governor.
			 */
			.peak_hysteresis_streak = 0,
			.peak_step_down_pct = 0,
			/* Stage 4 / Patch F: LEGACY disables the boost
			 * early-exit; boosts are honoured to their
			 * configured deadline.
			 */
			.boost_idle_thresh = 0,
			.boost_idle_streak = 0,
			/* Stage 4 / Patch G: LEGACY keeps full util
			 * regardless of screen state (legacy governor
			 * had no notion of screen-off util scaling).
			 */
			.bg_util_scale_pct = 100,
			/* Stage 4 / Patch H: LEGACY disables sleeper-tail
			 * shaving (legacy governor had no notion).
			 */
			.sleeper_tail_thresh_us = 0,
			.sleeper_tail_pct = 100,
			/* Stage 4 / Patch D: LEGACY disables peer-ramp.
			 * Pre-Stage-1 governor had no cross-cluster
			 * coordination; LEGACY preserves that behaviour
			 * end-to-end.
			 */
			.peer_ramp_window_ms = 0,
			.peer_ramp_floor_pct = 0,
			/* Stage 5 / Patch M3: LEGACY likewise leaves the
			 * screen-off shadow at 0.  Pre-Stage-1 governor
			 * had no peer_ramp at all, screen-on or screen-
			 * off, so this preserves that absence end-to-
			 * end.
			 */
			.peer_ramp_window_off_ms = 0,
			/* Stage 4 / Patch K1: LEGACY disables the
			 * migration-arrival floor.  Pre-Stage-1
			 * governor had no PELT-warm-up compensation;
			 * LEGACY preserves that.
			 */
			.migration_jump_pct = 0,
			.migration_floor_window_ms = 0,
			.migration_floor_pct = 0,
			/* Stage 4 / Patch K2: LEGACY disables the
			 * PSI-CPU floor.  Pre-Stage-1 governor had no
			 * pressure-aware lifts.  LEGACY preserves
			 * that.
			 */
			.psi_cpu_floor_thresh = 0,
			/* Stage 4 / Patch K3: LEGACY disables the
			 * frame-overrun rescue.  Pre-Stage-1 governor
			 * had no display-layer awareness; LEGACY
			 * preserves that.
			 */
			.frame_overrun_slack_us = 0,
			.frame_overrun_window_ms = 0,
			.frame_overrun_floor_pct = 0,
			/* Stage 5 / Patch M5: LEGACY mirrors the K3
			 * disable for the deep tier.  Pre-Stage-1
			 * governor had no concept of consecutive-
			 * overrun amplification; LEGACY preserves that
			 * absence end-to-end.
			 */
			.frame_overrun_deep_streak = 0,
			.frame_overrun_deep_floor_pct = 100,
			/* Stage 5 / Patch M1: LEGACY keeps the PSI-mem
			 * cap fully off.  Pre-Stage-1 governor had no
			 * concept of memstall-driven freq capping;
			 * LEGACY preserves that.
			 */
			.psi_mem_cap_thresh = 0,
			.psi_mem_cap_pct = 80,
			.psi_mem_cap_window_ms = 1000,
			.audio_hyst_ms = 0,
			.vh_arch_freq_scale_enable = 0,
			.vh_uclamp_observer_enable = 0,
			.vh_cpu_idle_enable = 1,
			.vh_freq_qos_enable = 0,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
		},
		{
			/* Patch 4.1: GAMING profile.
			 *
			 * Purpose: maximum frame stability under
			 * sustained foreground game load.  Branches
			 * from PERFORMANCE with stronger frame-overrun
			 * tiering, deeper cluster-wake / fg pulses,
			 * full screen-on background util, and AC-vs-
			 * battery hold scaling pinned at identity (the
			 * profile is for plugged-in / docked play; the
			 * user explicitly opted into the energy cost).
			 *
			 * Auto-tune state: LATENCY (mapped via
			 * zenith_profile_to_at_state).  Guardrails:
			 * shared with PERFORMANCE (mapped via
			 * zenith_at_get_guardrails).
			 */
			.profile = ZENITH_PROFILE_GAMING,
			.up_rate_limit_us = 0,
			.down_rate_limit_us = 8000,
			.up_threshold = 60,
			.down_threshold = 45,
			.hispeed_freq_pct = 65,
			.hispeed_load = 50,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = 18,
			.powersave_bias = 0,
			.bias_load_threshold = 50,
			.ignore_nice_load = 0,
			.input_boost_ms = 180,
			.input_boost_decay_ms = 60,
			.input_boost_cap_pct = 0,
			.light_load_threshold = 12,
			.sampling_down_factor = 4,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 1,
			.down_rate_adaptive = 1,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 12,
			.rate_limit_cluster_scale = 1,
			/* Aggressive peak-headroom: lower starve_load_-
			 * pct (85 vs PERF 88), shorter streak (1 vs 2),
			 * higher floor (90 vs PERF 80), longer hold
			 * (35 ms vs PERF 25).  A single below-floor
			 * sample inside a sustained game load is a
			 * frame at risk; recover before EAS sees it.
			 */
			.peak_headroom_rescue = 1,
			.peak_headroom_prearm = 1,
			.peak_headroom_starve_load_pct = 85,
			.peak_headroom_freq_floor_pct = 90,
			.peak_headroom_starve_streak = 1,
			.peak_headroom_jump_pct = 100,
			.peak_headroom_hold_ms = 35,
			.batt_hold_scale_pct = 100,
			.cluster_wake_pulse_ms = 100,
			.cluster_wake_pulse_idle_ms = 50,
			.cluster_wake_pulse_floor_pct = 80,
			.quiet_hours_cap_pct = 100,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms = 60,
			.fg_transition_pulse_pct = 80,
			.screen_on_bias_pct = 0,
			.input_boost_down_rate_mult_pct = 350,
			.predict_up_thresh = 40,
			.predict_up_window = 2,
			/* Patch C3: GAMING is the most aggressive
			 * profile for the rising-edge tier (20 / 35).
			 * Slope as small as ~8%% per sample fires the
			 * lift, and the level gate drops to 35%% of
			 * max_cap so a freshly-foregrounded game frame
			 * snaps to eff_hispeed before the level-trig
			 * tier even sees it.
			 */
			.pelt_rising_edge_thresh = 20,
			.pelt_rising_edge_min_pct = 35,
			/* Patch C6: GAMING pins to policy->max on DL
			 * task presence -- some game engines pin a
			 * compositor-pacer thread to SCHED_DEADLINE
			 * to lock vblank cadence; the floor guarantees
			 * its first wake hits target frequency.
			 */
			.dl_task_floor_pct = 100,
			/* Patch C9: GAMING 300 ms / 60%% — game asset
			 * loading and save/restore bursts keep the
			 * freq elevated through batch gaps.
			 */
			.io_floor_hyst_ms = 300,
			.io_floor_hyst_pct = 60,
			.render_floor_pct = 85,
			.render_floor_min_runtime_ms = 15,
			.input_boost_touchdown_extra_ms = 100,
			.peak_hysteresis_streak = 5,
			.peak_step_down_pct = 98,
			.boost_idle_thresh = 0,
			.boost_idle_streak = 0,
			.bg_util_scale_pct = 100,
			.sleeper_tail_thresh_us = 0,
			.sleeper_tail_pct = 100,
			.peer_ramp_window_ms = 50,
			.peer_ramp_floor_pct = 75,
			/* Keep peer_ramp warm even with screen blanked.
			 * Game-mode often suspends rendering briefly
			 * (loading screens / menu transitions) where
			 * the IPC chain is still active; suppressing
			 * peer_ramp the moment the panel reports
			 * screen-off would defeat the profile.
			 */
			.peer_ramp_window_off_ms = 30,
			.migration_jump_pct = 12,
			.migration_floor_window_ms = 40,
			.migration_floor_pct = 75,
			.psi_cpu_floor_thresh = 35,
			/* Frame-overrun tiering tightened across the
			 * board.  Slack 2500 us = ~15%% of a 60 Hz
			 * frame; window 60 ms; floor 95%%.  Deep tier
			 * arms on a single consecutive overrun (vs
			 * PERF's 2) at full 100%%.
			 */
			.frame_overrun_slack_us = 2500,
			.frame_overrun_window_ms = 60,
			.frame_overrun_floor_pct = 95,
			.frame_overrun_deep_streak = 1,
			.frame_overrun_deep_floor_pct = 100,
			/* PSI-mem cap stays off (same reasoning as
			 * PERFORMANCE).  Forward-compatible defaults
			 * for the pct/window if a sysfs override
			 * arms thresh.
			 */
			.psi_mem_cap_thresh = 0,
			.psi_mem_cap_pct = 90,
			.psi_mem_cap_window_ms = 1000,
			.audio_hyst_ms = 250,
			.vh_arch_freq_scale_enable = 1,
			.vh_uclamp_observer_enable = 1,
			.vh_cpu_idle_enable = 1,
			/* Patch B9-3+: GAMING already provides aggressive
			 * headroom; QoS-pressure pivot to PERFORMANCE is
			 * redundant.  Bake off.
			 */
			.vh_freq_qos_enable = 0,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
		},
		{
			/* Patch 4.2: AUDIO profile.
			 *
			 * Purpose: pinhole-tight wake-cadence guarantees
			 * for ALSA / aaudio callback threads (typical
			 * pcm period ~ 5..20 ms).  Branches from
			 * BALANCED with the screen-off / sleeper-tail /
			 * peer-ramp-off knobs all softened so the
			 * cluster doesn't drift into states that cost
			 * an audio underrun on the first wake of every
			 * period.  Frame-overrun stays at BALANCED
			 * (audio doesn't care about vblank).  Render
			 * floor disabled (audio threads aren't
			 * RENDER_PRIO).  PSI-mem cap stays off.
			 *
			 * Auto-tune state: BALANCED (mapped via
			 * zenith_profile_to_at_state).  Guardrails:
			 * shared with BALANCED / CUSTOM.
			 */
			.profile = ZENITH_PROFILE_AUDIO,
			.up_rate_limit_us = 250,
			.down_rate_limit_us = 8000,
			.up_threshold = 70,
			.down_threshold = 50,
			.hispeed_freq_pct = ZENITH_DEFAULT_HISPEED_FREQ_PCT,
			.hispeed_load = ZENITH_DEFAULT_HISPEED_LOAD,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = ZENITH_DEFAULT_FREQ_STEP_PCT,
			.powersave_bias = 30,
			.bias_load_threshold = 45,
			.ignore_nice_load = 1,
			.input_boost_ms = 50,
			.input_boost_decay_ms = 30,
			.input_boost_cap_pct = 70,
			.light_load_threshold = 18,
			.sampling_down_factor = 4,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 1,
			.down_rate_adaptive = 1,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 5,
			.rate_limit_cluster_scale = 1,
			/* Mild peak-headroom: lift the floor slightly
			 * over BALANCED so the audio worker cluster
			 * recovers fast on a brief peak, but keep
			 * starve_load_pct higher so we don't fire on
			 * routine cadence.
			 */
			.peak_headroom_rescue =
				ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE,
			.peak_headroom_prearm =
				ZENITH_DEFAULT_PEAK_HEADROOM_PREARM,
			.peak_headroom_starve_load_pct = 90,
			.peak_headroom_freq_floor_pct = 75,
			.peak_headroom_starve_streak = 3,
			.peak_headroom_jump_pct = 100,
			.peak_headroom_hold_ms = 30,
			.batt_hold_scale_pct = 110,
			.cluster_wake_pulse_ms =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS,
			.cluster_wake_pulse_idle_ms =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_IDLE_MS,
			.cluster_wake_pulse_floor_pct = 65,
			.quiet_hours_cap_pct = 85,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms =
				ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS,
			.fg_transition_pulse_pct =
				ZENITH_DEFAULT_FG_TRANSITION_PULSE_PCT,
			.screen_on_bias_pct =
				ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT,
			.input_boost_down_rate_mult_pct =
				ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT,
			.predict_up_thresh =
				ZENITH_DEFAULT_PREDICT_UP_THRESH,
			.predict_up_window = 3,
			/* Patch C3: AUDIO matches BALANCED defaults.
			 * Audio worker bursts are predictable enough
			 * that the rolling-window predict_up handles
			 * them without the rising-edge tier needing to
			 * over-react.  Keeping the knob default-armed
			 * means a transient game/UI burst alongside
			 * audio playback is still caught.
			 */
			.pelt_rising_edge_thresh =
				ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH,
			.pelt_rising_edge_min_pct =
				ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT,
			/* Patch C6: AUDIO uses an 80%% DL floor.  ALSA
			 * RT/DL paths and JACK-style audio servers
			 * pin a per-period worker on SCHED_DEADLINE
			 * with sub-ms periods; the 80%% floor gives
			 * those threads guaranteed headroom without
			 * pinning the cluster to absolute max for
			 * everything else on the device.
			 */
			.dl_task_floor_pct = 80,
			/* Patch C9: AUDIO 500 ms / 60%% — sustained
			 * DAC ring-buffer writes are iowait-heavy but
			 * util-light (D-state dominant); the floor
			 * prevents freq drops between DMA periods.
			 */
			.io_floor_hyst_ms = 500,
			.io_floor_hyst_pct = 60,
			/* Render floor off: audio worker is not the
			 * RENDER_PRIO thread that renderer-floor is
			 * scoped to.  Keep the floor knob populated
			 * for forward-compat in case sysfs flips it
			 * during an audio + game session, but cold-
			 * start the AUDIO profile with it off.
			 */
			.render_floor_pct = 0,
			.render_floor_min_runtime_ms =
				ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS,
			.input_boost_touchdown_extra_ms = 30,
			.peak_hysteresis_streak =
				ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK,
			.peak_step_down_pct =
				ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT,
			.boost_idle_thresh =
				ZENITH_DEFAULT_BOOST_IDLE_THRESH,
			.boost_idle_streak =
				ZENITH_DEFAULT_BOOST_IDLE_STREAK,
			/* Background-util scale stays high under AUDIO.
			 * The ALSA / aaudio worker keeps running with
			 * the screen off; trimming it to the BALANCED
			 * 75%% bg_util_scale risks dropping the
			 * cluster into a slower tier on the very
			 * cadence the worker depends on.
			 */
			.bg_util_scale_pct = 90,
			/* Sleeper-tail shaving disabled.  Audio
			 * callbacks wake on a tight period; shaving
			 * the wake-tick freq is exactly the kind of
			 * micro-saving that turns into audible
			 * underruns.
			 */
			.sleeper_tail_thresh_us = 0,
			.sleeper_tail_pct = 100,
			/* Peer-ramp: keep both windows armed.  Audio
			 * worker often migrates LITTLE -> BIG on a
			 * spike (resampler / mixer), and the IPC chain
			 * to the audio HAL stays warm with the screen
			 * off.  Suppressing peer_ramp_off here would
			 * defeat the profile.
			 */
			.peer_ramp_window_ms =
				ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS,
			.peer_ramp_floor_pct = 65,
			.peer_ramp_window_off_ms = 25,
			.migration_jump_pct =
				ZENITH_DEFAULT_MIGRATION_JUMP_PCT,
			.migration_floor_window_ms =
				ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS,
			.migration_floor_pct =
				ZENITH_DEFAULT_MIGRATION_FLOOR_PCT,
			.psi_cpu_floor_thresh =
				ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH,
			/* Frame-overrun unchanged from BALANCED;
			 * audio cadence is independent of vblank.
			 */
			.frame_overrun_slack_us = 4000,
			.frame_overrun_window_ms =
				ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS,
			.frame_overrun_floor_pct =
				ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT,
			.frame_overrun_deep_streak = 0,
			.frame_overrun_deep_floor_pct = 100,
			/* PSI-mem cap off: capping the cluster on
			 * memstall risks underruns the same way
			 * sleeper-tail shaving would.  pct/window
			 * left at sensible defaults for forward
			 * compat with sysfs overrides.
			 */
			.psi_mem_cap_thresh = 0,
			.psi_mem_cap_pct = 80,
			.psi_mem_cap_window_ms = 1000,
			.audio_hyst_ms = 750,
			.vh_arch_freq_scale_enable = 0,
			.vh_uclamp_observer_enable = 0,
			.vh_cpu_idle_enable = 1,
			.vh_freq_qos_enable = 0,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
		},
	};
	const struct zenith_profile_defaults *p = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(profiles); i++) {
		if (profiles[i].profile == prof) {
			p = &profiles[i];
			break;
		}
	}
	if (!p)
		return;

	t->up_rate_limit_us	= p->up_rate_limit_us;
	t->down_rate_limit_us	= p->down_rate_limit_us;
	t->up_threshold		= p->up_threshold;
	t->down_threshold	= p->down_threshold;
	t->hispeed_freq_pct	= p->hispeed_freq_pct;
	t->hispeed_load		= p->hispeed_load;
	t->climb_mode		= p->climb_mode;
	t->freq_step_pct	= p->freq_step_pct;
	t->powersave_bias	= p->powersave_bias;
	t->bias_load_threshold	= p->bias_load_threshold;
	t->ignore_nice_load	= p->ignore_nice_load;
	t->input_boost_ms	= p->input_boost_ms;
	t->input_boost_decay_ms	= p->input_boost_decay_ms;
	t->input_boost_cap_pct	= p->input_boost_cap_pct;
	t->light_load_threshold	= p->light_load_threshold;
	t->sampling_down_factor	= p->sampling_down_factor;
	t->thermal_auto		= p->thermal_auto;
	t->screen_auto		= p->screen_auto;
	t->util_math_v2		= p->util_math_v2;
	t->kcpustat_hispeed_enable = p->kcpustat_hispeed_enable;
	t->down_rate_adaptive	= p->down_rate_adaptive;
	t->wakeup_boost		= p->wakeup_boost;
	t->down_threshold_adaptive = p->down_threshold_adaptive;
	t->rate_limit_cluster_scale = p->rate_limit_cluster_scale;
	t->peak_headroom_rescue	= p->peak_headroom_rescue;
	t->peak_headroom_prearm	= p->peak_headroom_prearm;
	t->peak_headroom_starve_load_pct =
		p->peak_headroom_starve_load_pct;
	t->peak_headroom_freq_floor_pct =
		p->peak_headroom_freq_floor_pct;
	t->peak_headroom_starve_streak =
		p->peak_headroom_starve_streak;
	t->peak_headroom_jump_pct = p->peak_headroom_jump_pct;
	t->peak_headroom_hold_ms = p->peak_headroom_hold_ms;
	t->batt_hold_scale_pct	= p->batt_hold_scale_pct;
	t->cluster_wake_pulse_ms = p->cluster_wake_pulse_ms;
	t->cluster_wake_pulse_idle_ms = p->cluster_wake_pulse_idle_ms;
	t->cluster_wake_pulse_floor_pct = p->cluster_wake_pulse_floor_pct;
	t->quiet_hours_cap_pct	= p->quiet_hours_cap_pct;
	t->quiet_hours_screen_off_only = p->quiet_hours_screen_off_only;
	t->fg_transition_pulse_ms = p->fg_transition_pulse_ms;
	t->fg_transition_pulse_pct = p->fg_transition_pulse_pct;
	t->screen_on_bias_pct	= p->screen_on_bias_pct;
	t->input_boost_down_rate_mult_pct =
		p->input_boost_down_rate_mult_pct;
	WRITE_ONCE(t->predict_up_thresh, p->predict_up_thresh);
	WRITE_ONCE(t->predict_up_window, p->predict_up_window);
	WRITE_ONCE(t->pelt_rising_edge_thresh, p->pelt_rising_edge_thresh);
	WRITE_ONCE(t->pelt_rising_edge_min_pct, p->pelt_rising_edge_min_pct);
	WRITE_ONCE(t->dl_task_floor_pct, p->dl_task_floor_pct);

	/* Boot-time hardware calibration: clamp to calibrated minimum */
	if (READ_ONCE(zenith_calibrate_enable)) {
		if (t->up_rate_limit_us < READ_ONCE(zenith_calibrated_up_us))
			t->up_rate_limit_us = READ_ONCE(zenith_calibrated_up_us);
		if (t->down_rate_limit_us < READ_ONCE(zenith_calibrated_down_us))
			t->down_rate_limit_us = READ_ONCE(zenith_calibrated_down_us);
	}

	WRITE_ONCE(t->io_floor_hyst_ms, p->io_floor_hyst_ms);
	WRITE_ONCE(t->io_floor_hyst_pct, p->io_floor_hyst_pct);
	t->render_floor_pct	= p->render_floor_pct;
	WRITE_ONCE(t->render_floor_min_runtime_ms,
		   p->render_floor_min_runtime_ms);
	WRITE_ONCE(t->input_boost_touchdown_extra_ms,
		   p->input_boost_touchdown_extra_ms);
	WRITE_ONCE(t->peak_hysteresis_streak,
		   p->peak_hysteresis_streak);
	WRITE_ONCE(t->peak_step_down_pct, p->peak_step_down_pct);
	WRITE_ONCE(t->boost_idle_thresh, p->boost_idle_thresh);
	WRITE_ONCE(t->boost_idle_streak, p->boost_idle_streak);
	WRITE_ONCE(t->bg_util_scale_pct, p->bg_util_scale_pct);
	WRITE_ONCE(t->sleeper_tail_thresh_us,
		   p->sleeper_tail_thresh_us);
	WRITE_ONCE(t->sleeper_tail_pct, p->sleeper_tail_pct);
	WRITE_ONCE(t->peer_ramp_window_ms, p->peer_ramp_window_ms);
	WRITE_ONCE(t->peer_ramp_floor_pct, p->peer_ramp_floor_pct);
	WRITE_ONCE(t->peer_ramp_window_off_ms,
		   p->peer_ramp_window_off_ms);
	WRITE_ONCE(t->migration_jump_pct, p->migration_jump_pct);
	WRITE_ONCE(t->migration_floor_window_ms,
		   p->migration_floor_window_ms);
	WRITE_ONCE(t->migration_floor_pct, p->migration_floor_pct);
	WRITE_ONCE(t->psi_cpu_floor_thresh, p->psi_cpu_floor_thresh);
	WRITE_ONCE(t->frame_overrun_slack_us, p->frame_overrun_slack_us);
	WRITE_ONCE(t->frame_overrun_window_ms,
		   p->frame_overrun_window_ms);
	WRITE_ONCE(t->frame_overrun_floor_pct,
		   p->frame_overrun_floor_pct);
	WRITE_ONCE(t->frame_overrun_deep_streak,
		   p->frame_overrun_deep_streak);
	WRITE_ONCE(t->frame_overrun_deep_floor_pct,
		   p->frame_overrun_deep_floor_pct);
	WRITE_ONCE(t->psi_mem_cap_thresh, p->psi_mem_cap_thresh);
	WRITE_ONCE(t->psi_mem_cap_pct, p->psi_mem_cap_pct);
	WRITE_ONCE(t->psi_mem_cap_window_ms,
		   p->psi_mem_cap_window_ms);
	WRITE_ONCE(t->audio_hyst_ms, p->audio_hyst_ms);
	WRITE_ONCE(t->vh_arch_freq_scale_enable,
		   p->vh_arch_freq_scale_enable);
	WRITE_ONCE(t->vh_uclamp_observer_enable,
		   p->vh_uclamp_observer_enable);
	WRITE_ONCE(t->vh_cpu_idle_enable,
		   p->vh_cpu_idle_enable);
	WRITE_ONCE(t->vh_freq_qos_enable,
		   p->vh_freq_qos_enable);
	WRITE_ONCE(t->vh_sched_move_task_enable,
		   p->vh_sched_move_task_enable);
	WRITE_ONCE(t->vh_scheduler_tick_enable,
		   p->vh_scheduler_tick_enable);
	WRITE_ONCE(zenith_frame_overrun_slack_us_cache,
		   p->frame_overrun_slack_us);
	WRITE_ONCE(zenith_frame_overrun_window_ms_cache,
		   p->frame_overrun_window_ms);
	/* Patch B10-3: copy the profile's cgroup-v2 PSI path into
	 * the per-tunables buffer (NULL bake -> empty string =
	 * system-wide PSI = pre-B10 behaviour) and refresh the
	 * file-scope cached cgroup pointer.  Identical-path
	 * applies are no-ops inside zenith_psi_cgroup_apply() so
	 * profile-bake from multiple policies doesn't churn the
	 * cgroup ref.
	 */
	{
		const char *cg_path = p->psi_cgroup_path ?: "";

		strscpy(t->psi_cgroup_path, cg_path,
			sizeof(t->psi_cgroup_path));
		zenith_psi_cgroup_apply(cg_path);
	}

	/* Mirror input_boost_ms and input_boost_touchdown_extra_ms to
	 * the governor-wide caches used by the input handler fast
	 * path so a profile flip is picked up on the next event.
	 */
	WRITE_ONCE(zenith_input_boost_active_ms, t->input_boost_ms);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache,
		   t->input_boost_touchdown_extra_ms);

	/* Patch L: emit a one-line summary of the bake result so the
	 * operator can confirm in dmesg / logcat which profile is
	 * live and what the major tunables were set to.  Fires on
	 * every apply path -- user-driven profile_store, AUTO-driven
	 * worker, and the early-balanced-then-AUTO bootstrap inside
	 * profile_store's AUTO branch.  Gated on verbose_log inside
	 * the helper.
	 */
	zenith_log_profile_applied(t, prof);

	/* Propagate the profile to the thermal stack so Kasumi's
	 * dampening window and Iyashi's performance floor track
	 * the governor's intent.  BALANCED bakes the compile-time
	 * defaults in both subsystems so this is a no-op on the
	 * cold-boot path.
	 */
	kasumi_apply_profile(prof);
	iyashi_apply_profile(prof);
	hikari_apply_profile(prof);
	equilibrium_apply_profile(prof);
	nocturne_apply_profile(prof);
}

/* Patch B-AUTO-4: auto-selector classifier (priority cascade).
 *
 * Returns the concrete profile zenith should run while
 * active_profile == ZENITH_PROFILE_AUTO.  Reads governor-wide
 * signals plus a per-policy walk of attr_set->policy_list and
 * picks one of:
 *
 *   AUDIO       any process holds an open ALSA fd (atomic
 *               refcount maintained by the snd_pcm_open /
 *               snd_pcm_release vendor hooks).  Highest priority
 *               -- audio glitches are the most user-visible
 *               regression and AUDIO bake is the lowest-jitter
 *               profile.
 *   GAMING      any policy currently sees a game-engine thread
 *               on a runqueue (Unity / Unreal / Cocos2d / etc;
 *               see zenith_game_auto_comms[]) OR has render-
 *               thread saturation while the v4l2 fd refcount is
 *               zero (camera takes precedence over render via
 *               PERFORMANCE).  Game profile prioritises sustained
 *               throughput + frame-pacing over efficiency.
 *   PERFORMANCE recent input event (< 1500 ms) AND screen on,
 *               OR camera fd open (capture pipelines need
 *               headroom but not the GAMING bake).
 *   BATTERY     screen off AND running on battery.  Most
 *               aggressive efficiency bake; safe because the
 *               user is by definition not interacting.
 *   BALANCED    fallback when no signal fires.  Sane default
 *               that handles screen-on idle, charger-attached
 *               idle, and any state the more specific
 *               classifiers do not catch.
 *
 * LEGACY and CUSTOM are never picked -- they are explicit opt-
 * out paths.
 *
 * The classifier is read-only with respect to tunables and z_-
 * policy state (no field is mutated; no lock is taken beyond the
 * list-walk RCU-equivalent under attr_set->update_lock that the
 * caller already holds).  All atomic / shared-state reads are
 * READ_ONCE / atomic_read / atomic64_read and racing values are
 * tolerated -- the worst case is a single 500 ms eval window
 * delayed pick, debounced again by auto_hysteresis_ms.
 *
 * Recency window for PERFORMANCE: 1500 ms.  Long enough to ride
 * through a single scroll gesture's quiet phase, short enough
 * that a finished interaction does not pin PERFORMANCE forever.
 */
#define ZENITH_AUTO_INPUT_RECENT_NS	(1500ULL * NSEC_PER_MSEC)

static unsigned int zenith_auto_classify(struct zenith_tunables *t)
{
	struct zenith_policy *z_policy;
	bool audio;
	bool camera;
	bool game = false;
	bool render = false;
	bool input_recent;
	bool screen_on;
	bool on_battery;
	u64 now_ns;
	u64 last_input_ns;

	audio = atomic_read(&zenith_alsa_active_fds) > 0;
	if (audio)
		return ZENITH_PROFILE_AUDIO;

	camera = atomic_read(&zenith_v4l2_active_fds) > 0;
	on_battery = atomic_read(&zenith_on_battery) > 0;
	screen_on = READ_ONCE(t->screen_state) != 0;
	now_ns = ktime_get_ns();
	last_input_ns = atomic64_read(&zenith_input_last_event_ns);
	input_recent = last_input_ns &&
		       (now_ns - last_input_ns) < ZENITH_AUTO_INPUT_RECENT_NS;

	/* Game / render walk over attr_set->policy_list.  The caller
	 * (zenith_auto_eval_work_fn) holds attr_set->update_lock so
	 * the policy_list is stable here; the per-policy detectors
	 * are already cache-TTL'd so a 2 - 4 policy walk costs at
	 * most one strncmp loop per per-policy cache miss (sub-second
	 * window).  Render is OR'd into game when no camera fd is
	 * open -- a saturated render thread without a camera capture
	 * happening is dominated by the GAMING bake; with a camera
	 * fd open the camera takes precedence and the cascade lands
	 * on PERFORMANCE.
	 */
	list_for_each_entry(z_policy, &t->attr_set.policy_list,
			    tunables_hook) {
		if (zenith_policy_has_game_auto(z_policy)) {
			game = true;
			break;
		}
		if (!camera && zenith_policy_has_render(z_policy))
			render = true;
	}
	if (game || render)
		return ZENITH_PROFILE_GAMING;

	if (camera)
		return ZENITH_PROFILE_PERFORMANCE;

	/* Patch B9-3+: high-pressure FREQ_QOS_MIN raised by a vendor /
	 * thermal / ADPF requester within the last
	 * ZENITH_VH_FREQ_QOS_WINDOW_MS.  Tunable-gated
	 * (vh_freq_qos_enable) so the consumer side mirrors the
	 * probe's own write-side gate.  pressure_until_ns is 0 on
	 * cold boot (kzalloc); the s64 comparison treats 0 < now_ns
	 * correctly without wrap.
	 */
	if (READ_ONCE(t->vh_freq_qos_enable) &&
	    atomic64_read(&t->vh_freq_qos_pressure_until_ns) > (s64)now_ns)
		return ZENITH_PROFILE_PERFORMANCE;

	if (input_recent && screen_on)
		return ZENITH_PROFILE_PERFORMANCE;

	if (!screen_on && on_battery)
		return ZENITH_PROFILE_BATTERY;

	return ZENITH_PROFILE_BALANCED;
}

/* Patch B-AUTO-3: auto-selector worker.
 *
 * Runs on the system unbound deferrable workqueue at
 * t->auto_eval_ms cadence whenever active_profile ==
 * ZENITH_PROFILE_AUTO.  See the eval_work comment in struct
 * zenith_tunables for the design notes; this is the worker body
 * itself.
 *
 * Off-switch path:
 *   - active_profile != AUTO  -> return without rearming.  This
 *     is the path that fires when the user writes "balanced" /
 *     "performance" / etc to the profile sysfs node.
 *     profile_store will subsequently cancel_delayed_work_sync
 *     to drain any in-flight invocation.
 *
 * On-path:
 *   - target = zenith_auto_classify(t)  (B-AUTO-3 stub: BALANCED;
 *                                        B-AUTO-4: cascade)
 *   - hysteresis: if target == auto_target the engine is already
 *     converged; clear pending state and rearm.  Otherwise the
 *     pending target's first_seen timestamp gates the commit
 *     until auto_hysteresis_ms has elapsed.  When the timer
 *     fires we commit -- zenith_apply_profile under
 *     attr_set->update_lock plus zenith_refresh_rate_delays /
 *     zenith_invalidate_cache so cached per-policy state for the
 *     old profile does not leak forward.
 *   - active_profile *stays* at AUTO across commits; only
 *     auto_target reflects the engine's current pick.
 *
 * Cadence-zero path:
 *   - if auto_eval_ms == 0 the engine is paused.  The worker
 *     still rearms (default cadence 500 ms) so a userspace write
 *     to a non-zero auto_eval_ms re-enters the active path
 *     promptly, but the classifier is skipped and no profile
 *     mutation runs.  This makes auto_eval_ms = 0 a runtime
 *     enable/disable toggle without changing the work struct
 *     lifecycle.
 *
 * Concurrency:
 *   - active_profile is read once outside the lock as an early-
 *     out so the cancel-in-progress path (profile_store on AUTO
 *     exit) does not deadlock on update_lock waiting for
 *     cancel_delayed_work_sync to drain.
 *   - all profile-bake mutation paths take attr_set->update_lock
 *     (mutex, sleepable) just like profile_store does, so the
 *     two writers serialise cleanly.
 */
static void zenith_auto_eval_work_fn(struct work_struct *w)
{
	struct zenith_tunables *t = container_of(to_delayed_work(w),
						 struct zenith_tunables,
						 eval_work);
	unsigned int eval_ms;
	unsigned int hyst_ms;
	unsigned int target;
	unsigned int current_target;
	unsigned int user_eval_ms;

	/* Off-switch: read active_profile lock-free; if userspace has
	 * already pivoted to a manual profile we exit immediately
	 * without rearming so cancel_delayed_work_sync converges.
	 */
	if (READ_ONCE(t->active_profile) != ZENITH_PROFILE_AUTO)
		return;

	user_eval_ms = READ_ONCE(t->auto_eval_ms);
	hyst_ms = READ_ONCE(t->auto_hysteresis_ms);
	eval_ms = user_eval_ms ? user_eval_ms : ZENITH_DEFAULT_AUTO_EVAL_MS;

	/* Cadence-zero gate: auto_eval_ms == 0 is the runtime pause
	 * switch.  We still rearm at the default cadence so a future
	 * userspace write to a non-zero auto_eval_ms re-enters the
	 * active path promptly, but we skip the classifier and the
	 * profile commit so no tunable mutation happens while paused.
	 */
	if (user_eval_ms == 0)
		goto rearm;

	target = zenith_auto_classify(t);

	mutex_lock(&t->attr_set.update_lock);

	/* Re-check active_profile under the lock to close the race
	 * between the lock-free early-out above and a concurrent
	 * profile_store that took the lock first to switch us to a
	 * manual profile.  When the race fires we drop the lock and
	 * exit without rearming -- profile_store's
	 * cancel_delayed_work_sync follow-up will then converge.
	 */
	if (t->active_profile != ZENITH_PROFILE_AUTO) {
		mutex_unlock(&t->attr_set.update_lock);
		return;
	}

	current_target = READ_ONCE(t->auto_target);

	if (target == current_target) {
		/* Already converged on this target; clear any pending
		 * hysteresis tracking so a fresh divergence starts
		 * with a clean first_seen timestamp.
		 */
		t->auto_pending_target = current_target;
		t->auto_pending_first_seen_ns = 0;
		goto unlock;
	}

	if (t->auto_pending_target != target) {
		t->auto_pending_target = target;
		t->auto_pending_first_seen_ns = ktime_get_ns();
		goto unlock;
	}

	/* Pending target unchanged across this window; check whether
	 * the hysteresis timer has elapsed.
	 */
	if (hyst_ms) {
		u64 now = ktime_get_ns();
		u64 first_seen = t->auto_pending_first_seen_ns;
		u64 hyst_ns = (u64)hyst_ms * NSEC_PER_MSEC;

		if (!first_seen || now - first_seen < hyst_ns)
			goto unlock;
	}

	/* Hysteresis satisfied -- commit the new profile bake.
	 * active_profile stays at AUTO so the engine keeps running.
	 */
	zenith_apply_profile(t, target);
	WRITE_ONCE(t->auto_target, target);
	t->auto_pending_target = target;
	t->auto_pending_first_seen_ns = 0;

	zenith_refresh_rate_delays(&t->attr_set);
	zenith_invalidate_cache(&t->attr_set);

unlock:
	mutex_unlock(&t->attr_set.update_lock);

rearm:
	/* Final guard against the cancel_delayed_work_sync race --
	 * if active_profile was flipped out of AUTO while we held
	 * the lock above, do not rearm.
	 */
	if (READ_ONCE(t->active_profile) == ZENITH_PROFILE_AUTO)
		schedule_delayed_work(&t->eval_work,
				      msecs_to_jiffies(eval_ms));
}

/* early_param("zenith.profile", ...) — accepts one of the canonical
 * preset names (performance / balanced / battery / legacy / custom).
 * Anything else is ignored and leaves zenith_cmdline_profile at CUSTOM.
 *
 * Returning 1 tells the early-param core "consumed, do not append to
 * the residual cmdline"; returning 0 would leak the value into init
 * env via the unknown-param fallback.
 */
static int __init zenith_setup_profile(char *s)
{
	if (!s)
		return 1;
	if (!strcmp(s, "performance")) {
		zenith_cmdline_profile = ZENITH_PROFILE_PERFORMANCE;
	} else if (!strcmp(s, "balanced")) {
		zenith_cmdline_profile = ZENITH_PROFILE_BALANCED;
	} else if (!strcmp(s, "battery")) {
		zenith_cmdline_profile = ZENITH_PROFILE_BATTERY;
	} else if (!strcmp(s, "legacy")) {
		zenith_cmdline_profile = ZENITH_PROFILE_LEGACY;
	} else if (!strcmp(s, "gaming")) {
		zenith_cmdline_profile = ZENITH_PROFILE_GAMING;
	} else if (!strcmp(s, "audio")) {
		zenith_cmdline_profile = ZENITH_PROFILE_AUDIO;
	} else if (!strcmp(s, "custom")) {
		zenith_cmdline_profile = ZENITH_PROFILE_CUSTOM;
	} else {
		/* Unknown preset.  Reset explicitly to CUSTOM so a
		 * subsequent zenith.profile= on the cmdline (or a
		 * future caller chaining into this routine) cannot
		 * leak a previously-parsed value.  Belt and braces:
		 * the variable is already file-static and starts at
		 * CUSTOM, but pinning the reset on every unknown-input
		 * branch removes the entire class of "is this still
		 * the default?" questions.
		 */
		zenith_cmdline_profile = ZENITH_PROFILE_CUSTOM;
		pr_warn("zenith.profile=%s: unknown preset, ignored\n", s);
	}
	return 1;
}
early_param("zenith.profile", zenith_setup_profile);

/* Parse a single "performance|balanced|battery|legacy|custom" name
 * into a profile id.  Returns ZENITH_PROFILE_CUSTOM on unknown
 * input so callers can use the result as a tri-state ("apply" /
 * "explicit-custom" / "ignore") without another strcmp pass.
 */
static unsigned int __init zenith_parse_profile_name(const char *s)
{
	if (!strcmp(s, "performance"))
		return ZENITH_PROFILE_PERFORMANCE;
	if (!strcmp(s, "balanced"))
		return ZENITH_PROFILE_BALANCED;
	if (!strcmp(s, "battery"))
		return ZENITH_PROFILE_BATTERY;
	if (!strcmp(s, "legacy"))
		return ZENITH_PROFILE_LEGACY;
	if (!strcmp(s, "gaming"))
		return ZENITH_PROFILE_GAMING;
	if (!strcmp(s, "audio"))
		return ZENITH_PROFILE_AUDIO;
	if (!strcmp(s, "custom"))
		return ZENITH_PROFILE_CUSTOM;
	return ZENITH_PROFILE_CUSTOM;
}

/* early_param("zenith.policy_profile", ...) -- asymmetric preset
 * list.  Accepts a comma-separated list of "CPU:name" pairs where
 * CPU is the anchor-cpu of a cpufreq policy (cpumask_first) and
 * name is any of the canonical profile names accepted by
 * zenith.profile.  Example on a 4+3+1 big.LITTLE:
 *
 *   zenith.policy_profile=0:battery,4:balanced,7:performance
 *
 * Silently skips pairs with an out-of-range CPU index or an
 * unknown name.  Tokenising is done in-place against a stable
 * early_param scratch buffer (the cmdline is already copied by
 * the boot allocator).  Any slot not mentioned stays at CUSTOM
 * and therefore falls through to the global zenith.profile= (or
 * to the unprofiled default when that too is CUSTOM).
 */
static int __init zenith_setup_policy_profile(char *s)
{
	char *p, *next;

	if (!s)
		return 1;

	for (p = s; p && *p; p = next) {
		unsigned int cpu, prof;
		char *colon;

		next = strchr(p, ',');
		if (next)
			*next++ = '\0';

		colon = strchr(p, ':');
		if (!colon)
			continue;
		*colon++ = '\0';

		if (kstrtouint(p, 10, &cpu) || cpu >= NR_CPUS)
			continue;

		prof = zenith_parse_profile_name(colon);
		/* CUSTOM from the parser means either "user wrote
		 * custom" (explicit) or "unknown name" (skipped).  In
		 * both cases leaving the slot at its CUSTOM default
		 * is correct: CUSTOM means "no override".  Distinguish
		 * the explicit case only if we later want to force a
		 * CUSTOM override over the global profile; today we
		 * don't, so both collapse to the same behaviour.
		 */
		zenith_cmdline_policy_profile[cpu] = (u8)prof;
	}
	return 1;
}
early_param("zenith.policy_profile", zenith_setup_policy_profile);

/************************ Auto-tune observer *****************************/

/* Classify the workload seen since the last pass and pick a profile.
 * Runs from a delayed_work context on an arbitrary CPU. The per-policy
 * sample counters are atomics, so no lock is needed to sample+reset
 * them here even though zenith_update_single() increments them without
 * holding update_lock.
 */
static void zenith_auto_tune_work(struct work_struct *w)
{
	struct zenith_policy *z_policy =
		container_of(to_delayed_work(w), struct zenith_policy, at_work);
	struct zenith_tunables *t = z_policy->tunables;
	unsigned int total, saturated, sat_pct;
	u64 events_now, events_delta;
	unsigned int events_rate_x2;
	unsigned int target;
	unsigned int state;
	unsigned int reason = ZENITH_AT_REASON_CLASSIFIER;
	unsigned int flags = 0;
	bool audio = false;
	bool camera = false;
	bool render = false;
	bool memstall = false;
	bool psi_cpu = false;
	bool psi_io = false;
	bool frame_active = false;
	bool screen_off = false;
	bool thermal_slope = false;
	bool thermal = false;
	unsigned int psi_mem_pct = 0;
	unsigned int psi_cpu_pct = 0;
	unsigned int psi_io_pct = 0;
	unsigned int thermal_pressure = 0;
	unsigned int thermal_delta = 0;
	unsigned int frame_budget_us = 0;
	unsigned int at_from_state = z_policy->at_last_state;
	bool at_emergency = false;

	if (!t->auto_tune)
		return;	/* tunable turned off; stop the chain */

	/* Patch 1.2: refresh the AC-vs-battery cache once per
	 * auto_tune window.  Cheap (a single power_supply iterator
	 * walk per 10 s), keeps the hot path lock-free, and is the
	 * earliest point in the periodic chain where it makes sense
	 * to update -- the per-policy worker is the only periodic
	 * timer in the governor and it already runs only when
	 * auto_tune is on.  When no power supply driver is
	 * registered the helper returns -ENODEV / -ENOSYS and the
	 * cache stays at 0 (AC), so default behaviour is preserved.
	 */
	{
		int psy = power_supply_is_system_supplied();

		if (psy >= 0)
			atomic_set(&zenith_on_battery, psy ? 0 : 1);
	}

	/* Wave B PMU IPC tracker.  Sample per-CPU instructions /
	 * cycles perf_events once per auto_tune window.  Cheap
	 * (perf_event_read_value() walks one IPI per CPU; total cost
	 * for a 4-CPU policy is roughly 4 IPIs per ZENITH_AUTO_TUNE_-
	 * PERIOD_MS, 10 s by default).  Updates the per-CPU ipc_pct
	 * cache that zenith_get_next_freq()'s pmu_ipc_floor block
	 * reads via zenith_policy_max_ipc_pct().  When pmu_aware is 0
	 * the sampling still runs but the result is unused (always
	 * cheap to compute, and keeps the cache fresh in case the
	 * tunable is flipped on at runtime).  Compiles out on
	 * CONFIG_PERF_EVENTS=n via the #else branch in the helper.
	 */
	{
		unsigned int cpu;

		for_each_cpu(cpu, z_policy->policy->cpus)
			zenith_pmu_sample_cpu(cpu);
	}

	total = (unsigned int)atomic_xchg(&z_policy->at_samples_total, 0);
	saturated = (unsigned int)atomic_xchg(&z_policy->at_samples_saturated, 0);

	events_now = atomic64_read(&zenith_auto_input_events);
	events_delta = events_now - z_policy->at_last_events;
	z_policy->at_last_events = events_now;

	sat_pct = total ? (saturated * 100 / total) : 0;
	/* events per 2s, i.e. half-events/s * 2, kept integer-friendly:
	 * ZENITH_AUTO_TUNE_HI_EVENTS_X2=4 corresponds to > 2.0/s, and
	 * ZENITH_AUTO_TUNE_LO_EVENTS_X2=2 corresponds to < 1.0/s, over
	 * the ZENITH_AUTO_TUNE_PERIOD_MS window (10s by default).
	 */
	events_rate_x2 = (unsigned int)((events_delta * 2000) /
					ZENITH_AUTO_TUNE_PERIOD_MS);

	if (sat_pct >= t->auto_tune_hi_sat_pct &&
	    events_rate_x2 >= t->auto_tune_hi_events_x2)
		target = ZENITH_PROFILE_PERFORMANCE;
	else if (sat_pct <= t->auto_tune_lo_sat_pct &&
		 events_rate_x2 <= t->auto_tune_lo_events_x2)
		target = ZENITH_PROFILE_BATTERY;
	else
		target = ZENITH_PROFILE_BALANCED;
	state = zenith_profile_to_at_state(target);

	if (trace_zenith_auto_tune_enabled())
		trace_zenith_auto_tune(z_policy->policy->cpu, sat_pct,
				       events_rate_x2, t->active_profile,
				       target);

	/* Audit fix F2: PELT util-rising trend.  Sum per-cpu PELT
	 * util_avg across this policy and compare against the previous
	 * window's value.  Raise ZENITH_AT_FLAG_UTIL_RISING when the
	 * delta exceeds the configured percentage.  Done above the V2
	 * signal block so the flag is available to the state machine.
	 *
	 * Math: pct_growth = ((cur - prev) * 100) / max(prev, 1).
	 * Zero out the previous-sample on the first window after init
	 * so the first comparison doesn't see 100% growth from 0->N.
	 */
	{
		unsigned int cpu;
		unsigned long util_sum = 0;

		for_each_cpu(cpu, z_policy->policy->cpus) {
			struct rq *rq = cpu_rq(cpu);

			util_sum += READ_ONCE(rq->cfs.avg.util_avg);
		}

		if (z_policy->at_last_util_sum &&
		    t->auto_tune_util_rising_thresh_pct) {
			unsigned long prev = z_policy->at_last_util_sum;
			unsigned long delta = util_sum > prev ?
				util_sum - prev : 0;
			unsigned int pct = prev ?
				(unsigned int)((delta * 100) / prev) : 0;

			if (pct >= t->auto_tune_util_rising_thresh_pct)
				flags |= ZENITH_AT_FLAG_UTIL_RISING;
		}
		z_policy->at_last_util_sum = util_sum;
	}

	if (t->auto_tune_v2 && t->auto_tune_v2_signals) {
		unsigned int anchor = cpumask_first(z_policy->policy->cpus);

		screen_off = !READ_ONCE(t->screen_state);
		if (screen_off)
			flags |= ZENITH_AT_FLAG_SCREEN_OFF;

		if (t->psi_mem_thresh) {
			psi_mem_pct = zenith_psi_mem_some_pct();
			memstall = psi_mem_pct >= t->psi_mem_thresh;
		}
		if (t->psi_cpu_thresh) {
			psi_cpu_pct = zenith_psi_cpu_some_pct();
			psi_cpu = psi_cpu_pct >= t->psi_cpu_thresh;
		}
		if (t->psi_io_thresh) {
			psi_io_pct = zenith_psi_io_some_pct();
			psi_io = psi_io_pct >= t->psi_io_thresh;
		}
		if (psi_cpu)
			flags |= ZENITH_AT_FLAG_PSI_CPU;
		if (psi_io)
			flags |= ZENITH_AT_FLAG_PSI_IO;

		if (t->auto_tune_frame_pacing) {
			if (cpu_possible(anchor))
				frame_budget_us = READ_ONCE(
					t->frame_budget_us_per_policy[anchor]);
			if (!frame_budget_us)
				frame_budget_us = READ_ONCE(t->frame_budget_us);
			/* Mirror the hot-path frame_budget_us_auto override
			 * so the V2 classifier sees the same effective
			 * budget as the floor itself (which already prefers
			 * the drm-side cache).  Without this, V2 would
			 * miss frame_active on auto setups whose userspace
			 * frame_budget_us is left at 0.
			 */
			if (zenith_glide_value(z_policy,
				READ_ONCE(t->frame_budget_us_auto),
				z_policy->at_local_frame_budget_us_auto)) {
				unsigned int auto_us = (unsigned int)
					atomic_read(&zenith_drm_vblank_us);

				if (auto_us)
					frame_budget_us = auto_us;
			}
			frame_active = frame_budget_us &&
				       READ_ONCE(t->frame_pace_floor_pct);
			if (frame_active)
				flags |= ZENITH_AT_FLAG_FRAME;
		}

		if (t->auto_tune_sustained_gaming && READ_ONCE(t->game_mode)) {
			flags |= ZENITH_AT_FLAG_GAME;
			if (state == ZENITH_AT_STATE_LATENCY) {
				state = ZENITH_AT_STATE_SUSTAINED_PERF;
				reason = ZENITH_AT_REASON_GAME;
			}
		}
	}

	/* Scenario overlay.  V2 samples the same signals even when the
	 * legacy overlay gate is off, because diagnostics and state choice
	 * both benefit from knowing why a policy was held back or boosted.
	 */
	if (t->auto_tune_scenario || t->auto_tune_v2) {
		unsigned int cam_override = t->camera_active;
		unsigned int prev = target;

		audio = zenith_policy_has_audio(z_policy);
		render = zenith_policy_has_render(z_policy);
		if (cam_override == ZENITH_CAMERA_OVERRIDE_FORCE_ON)
			camera = true;
		else if (cam_override == ZENITH_CAMERA_OVERRIDE_FORCE_OFF)
			camera = false;
		else
			camera = zenith_policy_has_camera(z_policy);

		if (!psi_mem_pct && t->psi_mem_thresh)
			psi_mem_pct = zenith_psi_mem_some_pct();
		if (t->psi_mem_thresh)
			memstall = psi_mem_pct >= t->psi_mem_thresh;

		if (camera || render)
			target = ZENITH_PROFILE_PERFORMANCE;
		else if (memstall)
			target = ZENITH_PROFILE_BATTERY;
		else if (audio)
			target = ZENITH_PROFILE_BALANCED;
		if (camera)
			flags |= ZENITH_AT_FLAG_CAMERA;
		if (render)
			flags |= ZENITH_AT_FLAG_RENDER;
		if (audio)
			flags |= ZENITH_AT_FLAG_AUDIO;
		if (memstall)
			flags |= ZENITH_AT_FLAG_MEMSTALL;
		/* Reason reflects the strongest currently active scenario
		 * signal, not just the cause of the last target change.
		 * Without this, a long-held scenario (e.g. camera viewfinder
		 * pinned at PERFORMANCE for 30s) would show reason=classifier
		 * because the target stayed steady; that is misleading for
		 * post-hoc telemetry.  Severity order matches the target
		 * picker above (camera/render > memstall > audio).  Thermal,
		 * PSI, frame, screen, variance still override below; this
		 * is only the scenario-block default.
		 */
		if (camera || render)
			reason = ZENITH_AT_REASON_CAMERA_RENDER;
		else if (memstall)
			reason = ZENITH_AT_REASON_MEMSTALL;
		else if (audio)
			reason = ZENITH_AT_REASON_AUDIO;
		if (!(t->auto_tune_v2 && t->auto_tune_sustained_gaming &&
		      state == ZENITH_AT_STATE_SUSTAINED_PERF))
			state = zenith_profile_to_at_state(target);

		if (trace_zenith_auto_tune_scenario_enabled())
			trace_zenith_auto_tune_scenario(
				z_policy->policy->cpu, audio, camera,
				render, memstall, prev, target);
	}

	/* Master gate covers both the level signal (thermal_state) and
	 * the slope signal (auto_tune_thermal_slope) the V2 evaluator
	 * uses to drive the THERMAL_RECOVERY state.  When thermal_aware
	 * == 0 we pin both inputs to 0 here so the THERMAL_RECOVERY
	 * branches below never fire; the screen / PSI / frame /
	 * variance branches are unaffected because they don't depend
	 * on either signal.  See ZENITH_DEFAULT_THERMAL_AWARE comment
	 * block.
	 */
	if (ZENITH_FEATURE_ENABLED(thermal_aware)) {
		thermal = READ_ONCE(t->thermal_state);
		if (t->auto_tune_v2 && t->auto_tune_thermal_slope) {
			thermal_pressure =
				zenith_policy_thermal_pressure_pct(z_policy);
			if (thermal_pressure > z_policy->at_last_thermal_pressure)
				thermal_delta = thermal_pressure -
					z_policy->at_last_thermal_pressure;
			thermal_slope = thermal_pressure >=
					t->auto_tune_thermal_pressure_pct ||
				thermal_delta >= t->auto_tune_thermal_slope_pct;
			if (thermal_slope)
				flags |= ZENITH_AT_FLAG_THERMAL_SLOPE;
		}
	}
	if (thermal) {
		flags |= ZENITH_AT_FLAG_THERMAL;
		if (t->auto_tune_v2) {
			state = ZENITH_AT_STATE_THERMAL_RECOVERY;
			target = zenith_at_profile_for_state(z_policy, state);
			reason = ZENITH_AT_REASON_THERMAL;
		}
	} else if (t->auto_tune_v2 && thermal_slope) {
		state = ZENITH_AT_STATE_THERMAL_RECOVERY;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_THERMAL_SLOPE;
	} else if (t->auto_tune_v2 && screen_off) {
		state = ZENITH_AT_STATE_EFFICIENCY;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_SCREEN;
	} else if (t->auto_tune_v2 && (psi_cpu || psi_io)) {
		state = psi_cpu ? ZENITH_AT_STATE_SUSTAINED_PERF :
			ZENITH_AT_STATE_EFFICIENCY;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_PSI;
	} else if (t->auto_tune_v2 && frame_active &&
		   state == ZENITH_AT_STATE_LATENCY) {
		state = ZENITH_AT_STATE_SUSTAINED_PERF;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_FRAME;
	} else if (t->auto_tune_v2 &&
		   t->auto_tune_v2_var_promote_thresh &&
		   z_policy->load_var_ewma_x256 >=
				t->auto_tune_v2_var_promote_thresh &&
		   state == ZENITH_AT_STATE_LATENCY) {
		state = ZENITH_AT_STATE_SUSTAINED_PERF;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_VARIANCE;
	}

	zenith_at_update_prefer_silver_rate(z_policy, t, &flags);

	z_policy->at_last_total = total;
	z_policy->at_last_saturated = saturated;
	z_policy->at_last_sat_pct = sat_pct;
	z_policy->at_last_events_rate_x2 = events_rate_x2;
	z_policy->at_last_target = target;
	z_policy->at_last_flags = flags;
	z_policy->at_last_var_x256 = z_policy->load_var_ewma_x256;
	z_policy->at_last_psi_cpu = psi_cpu_pct;
	z_policy->at_last_psi_io = psi_io_pct;
	z_policy->at_last_psi_mem = psi_mem_pct;
	z_policy->at_last_thermal_slope = thermal_delta;
	z_policy->at_last_frame_budget_us = frame_budget_us;
	z_policy->at_last_thermal_pressure = thermal_pressure;

	if (t->auto_tune_v2) {
		unsigned int need = zenith_at_eff_hyst_windows(z_policy,
						t->auto_tune_hysteresis_windows);
		bool emergency = state == ZENITH_AT_STATE_THERMAL_RECOVERY ||
				 state == ZENITH_AT_STATE_SUSTAINED_PERF;
		/* Audit fix M3: rising-edge fast-path commit.
		 *
		 * Compute the bits that turned ON in this window's flag
		 * bitmap (~at_last_flags & flags).  If any of the
		 * UI-perceptible / urgent scenarios just-armed -- camera,
		 * render, frame, thermal_slope, game, psi_cpu, memstall --
		 * treat the resulting V2 state as emergency for both the
		 * cooldown bypass and the hysteresis gate.  Without this,
		 * a 2-window default hysteresis means a camera flag that
		 * fires at t=0 has to wait until t=2*1.25 s = 2.5 s before
		 * V2 actually commits the new state, which is visible to
		 * users (camera open feels sluggish for the first second).
		 *
		 * The mask is intentionally narrow.  Plain saturation /
		 * variance promotions are NOT fast-pathed because those
		 * are exactly the workloads where stable hysteresis pays
		 * off (avoid bouncing between LATENCY and SUSTAINED_PERF
		 * on a single-tick spike).  Only signals from explicit
		 * producers (input, drm, scenario detectors, PSI) get the
		 * fast lane.
		 */
		unsigned int rising = flags & ~z_policy->at_last_flags;
		const unsigned int fastpath_mask =
			ZENITH_AT_FLAG_CAMERA |
			ZENITH_AT_FLAG_RENDER |
			ZENITH_AT_FLAG_FRAME  |
			ZENITH_AT_FLAG_THERMAL_SLOPE |
			ZENITH_AT_FLAG_GAME   |
			ZENITH_AT_FLAG_PSI_CPU |
			ZENITH_AT_FLAG_MEMSTALL;
		bool rising_fastpath = (rising & fastpath_mask) &&
			state != z_policy->at_last_state;

		if (need > ZENITH_AT_HYSTERESIS_WINDOWS_MAX)
			need = ZENITH_AT_HYSTERESIS_WINDOWS_MAX;
		if (z_policy->at_pending_state != state) {
			z_policy->at_pending_state = state;
			z_policy->at_pending_windows = 1;
		} else if (z_policy->at_pending_windows < need) {
			z_policy->at_pending_windows++;
		}
		if (rising_fastpath) {
			emergency = true;
			z_policy->at_pending_windows = need;
		}
		if (!emergency && z_policy->at_cooldown_left) {
			z_policy->at_cooldown_left--;
			z_policy->at_last_reason = ZENITH_AT_REASON_COOLDOWN;
			goto rearm;
		}
		if (z_policy->at_pending_windows < need) {
			z_policy->at_last_reason = ZENITH_AT_REASON_HYSTERESIS;
			goto rearm;
		}
		if (state != z_policy->at_last_state) {
			unsigned int old_state = z_policy->at_last_state;
			u64 now_ns = ktime_get_ns();
			unsigned int hslot;

			/* M1: accumulate residency for the outgoing
			 * state.  The denominator can be 0 on the very
			 * first transition (last_change_ns hasn't been
			 * stamped yet) -- guard against that, otherwise
			 * a u64 underflow would inject ~146 years into
			 * residency[old_state].
			 */
			if (z_policy->at_state_last_change_ns &&
			    now_ns > z_policy->at_state_last_change_ns &&
			    old_state < ARRAY_SIZE(z_policy->at_state_residency_ns))
				z_policy->at_state_residency_ns[old_state] +=
					now_ns - z_policy->at_state_last_change_ns;
			z_policy->at_state_last_change_ns = now_ns;

			/* M2: push a history entry.  Lock-free single-
			 * writer ring (the V2 worker is the only writer
			 * for a given z_policy), reader (sysfs show)
			 * tolerates one entry of tearing on wrap which
			 * is acceptable for a diagnostic surface.
			 */
			hslot = z_policy->at_history_head;
			if (hslot >= ZENITH_AT_HISTORY_NR)
				hslot = 0;
			z_policy->at_history[hslot].ts_ns = now_ns;
			z_policy->at_history[hslot].flags = flags;
			z_policy->at_history[hslot].from = (u8)old_state;
			z_policy->at_history[hslot].to = (u8)state;
			z_policy->at_history[hslot].reason = (u8)reason;
			z_policy->at_history_head =
				(hslot + 1) % ZENITH_AT_HISTORY_NR;
			if (z_policy->at_history_count < ZENITH_AT_HISTORY_NR)
				z_policy->at_history_count++;

			z_policy->at_last_state = state;
			z_policy->at_cooldown_left =
				min_t(unsigned int,
				      zenith_at_eff_cool_windows(z_policy,
						t->auto_tune_cooldown_windows),
				      ZENITH_AT_COOLDOWN_WINDOWS_MAX);
			at_emergency = emergency;
			if (!t->auto_tune_cluster_aware &&
			    target != t->active_profile) {
				zenith_apply_profile(t, target);
				t->active_profile = target;
				zenith_refresh_rate_delays_one(z_policy);
			}
			if (trace_zenith_auto_tune_v2_enabled())
				trace_zenith_auto_tune_v2(
					z_policy->policy->cpu,
					z_policy->cluster_class, old_state,
					state, reason, flags, target);
		}
		z_policy->at_last_reason = reason;
		zenith_at_apply_actions(z_policy, state);
		/* Record the cluster-aware-demoted state for telemetry.
		 * applied_state mirrors at_last_state when cluster_aware
		 * is off, but reflects the LATENCY->BALANCED / BALANCED->
		 * LATENCY demotions when on.  Same expression the action
		 * picker uses internally (zenith_at_profile_for_state),
		 * mapped back via zenith_profile_to_at_state so the field
		 * is a state ID rather than a profile ID.
		 */
		z_policy->at_last_applied_state =
			zenith_profile_to_at_state(
				zenith_at_profile_for_state(z_policy, state));
		/* Populate the round-U-z10 glide knobs (brutal_decay_ms,
		 * wakeup_boost_ms, ...) from the just-resolved V2 state
		 * when auto_tune_v2_glides is on.  Cheap; gated so it
		 * remains free on systems that opt out.
		 */
		if (READ_ONCE(t->auto_tune_v2_glides))
			zenith_at_apply_glides(z_policy, state);
		else
			z_policy->at_local_glides_active = false;

		/* Patch L: arm / disarm Stage-4 K1/K2/K3 tiers based on
		 * the just-resolved V2 state and flag set.  Same gating
		 * shape as the glides above -- the tunable is its own
		 * master switch separate from auto_tune_v2.
		 */
		if (READ_ONCE(t->auto_tune_v2_tiers))
			zenith_at_apply_tiers(z_policy, state);
		else
			z_policy->at_local_tiers_active = false;

		/* Boot-complete calm detector.  Only runs while the
		 * latch is still down and the auto arm is enabled.
		 * Counts consecutive committed-EFFICIENCY windows on
		 * this policy; the first policy to reach the threshold
		 * past the boottime grace period raises the latch
		 * globally.  Subsequent policies / windows short-circuit
		 * on the atomic_read.
		 */
		if (!atomic_read(&zenith_boot_complete) &&
		    READ_ONCE(t->boot_complete_auto)) {
			if (state == ZENITH_AT_STATE_EFFICIENCY)
				z_policy->at_boot_calm_streak++;
			else
				z_policy->at_boot_calm_streak = 0;

			if (z_policy->at_boot_calm_streak >=
			    ZENITH_BOOT_COMPLETE_CALM_WINDOWS) {
				u64 now_ns = ktime_get_boottime_ns();

				if (now_ns >= ZENITH_BOOT_COMPLETE_GRACE_NS) {
					WRITE_ONCE(zenith_boot_complete_ns,
						   now_ns);
					atomic_set(&zenith_boot_complete, 1);
					pr_info("zenith: boot_complete latched (auto, calm=%u windows)\n",
						z_policy->at_boot_calm_streak);
				}
			}
		}
		goto rearm;
	}

	if (target != t->active_profile) {
		zenith_apply_profile(t, target);
		t->active_profile = target;
		z_policy->at_local_actions = false;
		z_policy->at_local_glides_active = false;
		z_policy->at_local_tiers_active = false;
		/* Profile mutated tunables->{up,down}_rate_limit_us;
		 * refresh the per-policy rate-delay cache for *this*
		 * policy so the new limits take effect on the next tick.
		 * Other policies sharing the same tunables pick up the
		 * change on their own next auto_tune tick.  We cannot
		 * iterate attr_set->policy_list from this context
		 * (no lock held) without racing gov_attr_set_get/put.
		 */
		zenith_refresh_rate_delays_one(z_policy);
	}
	z_policy->at_last_state = zenith_profile_to_at_state(t->active_profile);
	z_policy->at_last_reason = reason;

	/* Re-arm for the next classification window. */
rearm:
	zenith_at_log_push(z_policy, at_from_state, z_policy->at_last_target,
			   at_emergency);

	/* V3 self-calibration tail.  Gated by the static branch
	 * (FALSE while auto_tune_v3 = 0) and the live tunables scalar
	 * (defends against a momentary tear during a sysfs store; the
	 * branch can be true while the scalar transitions back to 0).
	 * Internal cadence gate (auto_tune_v3_interval_ms) limits the
	 * actual work to once per 10..600 s.  See ZENITH_DEFAULT_AUTO_TUNE_V3
	 * comment block.
	 */
	if (static_branch_likely(&zenith_auto_tune_v3_key)) {
		unsigned int v3_mode = READ_ONCE(t->auto_tune_v3);

		if (v3_mode != ZENITH_AT_V3_MODE_OFF)
			zenith_at_v3_calibrate(z_policy, v3_mode);
	}

	{
		/* F1: pick the faster reschedule cadence whenever a
		 * scenario flag is active in the just-completed window.
		 * The check uses at_last_flags (set above by the
		 * decision path), so a scenario that ended IN this
		 * window still gets one fast follow-up window before
		 * we settle back to the slow cadence -- catches the
		 * case where the burst leaves residual variance worth
		 * a quick re-eval.
		 */
		const unsigned int fast_mask =
			ZENITH_AT_FLAG_CAMERA |
			ZENITH_AT_FLAG_RENDER |
			ZENITH_AT_FLAG_FRAME  |
			ZENITH_AT_FLAG_GAME   |
			ZENITH_AT_FLAG_MEMSTALL |
			ZENITH_AT_FLAG_THERMAL_SLOPE |
			ZENITH_AT_FLAG_PSI_CPU;
		unsigned int period =
			(z_policy->at_last_flags & fast_mask) ?
			ZENITH_AUTO_TUNE_FAST_PERIOD_MS :
			ZENITH_AUTO_TUNE_PERIOD_MS;

		schedule_delayed_work(&z_policy->at_work,
				      msecs_to_jiffies(period));
	}
}

static ssize_t auto_tune_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->auto_tune);
}

static ssize_t auto_tune_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_policy;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	if (t->auto_tune == val)
		return count;
	t->auto_tune = val;

	list_for_each_entry(z_policy, &t->attr_set.policy_list, tunables_hook) {
		if (val) {
			zenith_at_v_reset_window(z_policy);
			z_policy->at_cooldown_left = 0;
			schedule_delayed_work(&z_policy->at_work,
				msecs_to_jiffies(ZENITH_AUTO_TUNE_PERIOD_MS));
		} else {
			cancel_delayed_work_sync(&z_policy->at_work);
		}
	}

	return count;
}
static struct governor_attr auto_tune = __ATTR_RW(auto_tune);

ZENITH_TUNABLE_UINT_MAX(auto_tune_v2, 1);

/* auto_tune_v2_glides sysfs knob.  Boolean (0/1).  See
 * ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES.  Master gate for V2-driven
 * population of the round-U-z10 glide / coordination knobs;
 * defaults to 1 so the new soft-glide behaviour is on out of the
 * box without forcing operators to write seven separate sysfs
 * entries.  Per-knob user writes still take precedence.
 */
ZENITH_TUNABLE_UINT_MAX(auto_tune_v2_glides, 1);

/* auto_tune_v2_tiers sysfs knob (Patch L).  Boolean (0/1).  See
 * ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS.  Master gate for V2-driven
 * arming of the Stage-4 K1/K2/K3 floor tiers; defaults to 1 so
 * the V2 classifier shapes those tiers per state out of the box.
 * Set to 0 to lock all three tiers back to pure profile-driven
 * behaviour without disabling the rest of the V2 classifier.
 * Per-knob user sysfs writes still take precedence either way
 * (via the auto_tune_override_mask path).
 */
ZENITH_TUNABLE_UINT_MAX(auto_tune_v2_tiers, 1);

static ssize_t auto_tune_hysteresis_windows_show(struct gov_attr_set *attr_set,
						 char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_hysteresis_windows);
}

static ssize_t auto_tune_hysteresis_windows_store(struct gov_attr_set *attr_set,
						  const char *buf,
						  size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_HYSTERESIS_WINDOWS_MAX)
		return -EINVAL;
	t->auto_tune_hysteresis_windows = val;
	return count;
}
static struct governor_attr auto_tune_hysteresis_windows =
	__ATTR_RW(auto_tune_hysteresis_windows);

static ssize_t auto_tune_cooldown_windows_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_cooldown_windows);
}

static ssize_t auto_tune_cooldown_windows_store(struct gov_attr_set *attr_set,
						const char *buf,
						size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_COOLDOWN_WINDOWS_MAX)
		return -EINVAL;
	t->auto_tune_cooldown_windows = val;
	return count;
}
static struct governor_attr auto_tune_cooldown_windows =
	__ATTR_RW(auto_tune_cooldown_windows);

static ssize_t auto_tune_v2_var_promote_thresh_show(struct gov_attr_set *attr_set,
						    char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_v2_var_promote_thresh);
}

static ssize_t auto_tune_v2_var_promote_thresh_store(struct gov_attr_set *attr_set,
						     const char *buf,
						     size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_V2_VAR_PROMOTE_THRESH_MAX)
		return -EINVAL;
	t->auto_tune_v2_var_promote_thresh = val;
	return count;
}

static struct governor_attr auto_tune_v2_var_promote_thresh =
	__ATTR_RW(auto_tune_v2_var_promote_thresh);

/* Audit fix F2: PELT util-rising trend threshold sysfs knob.
 *
 * Read-only show / write-with-clamp store.  See the
 * ZENITH_DEFAULT_AT_UTIL_RISING_THRESH_PCT comment block for
 * semantics.  0 disables the signal cleanly.
 */
static ssize_t auto_tune_util_rising_thresh_pct_show(struct gov_attr_set *attr_set,
						     char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->auto_tune_util_rising_thresh_pct);
}

static ssize_t auto_tune_util_rising_thresh_pct_store(struct gov_attr_set *attr_set,
						      const char *buf,
						      size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_UTIL_RISING_THRESH_PCT_MAX)
		return -EINVAL;
	t->auto_tune_util_rising_thresh_pct = val;
	return count;
}

static struct governor_attr auto_tune_util_rising_thresh_pct =
	__ATTR_RW(auto_tune_util_rising_thresh_pct);

/* Audit fix F3: render-thread RT-priority floor (uclamp-min-style)
 * sysfs knob.  Range 0..100; 0 disables the floor cleanly.  See the
 * ZENITH_DEFAULT_AT_RENDER_RT_FLOOR_PCT comment block for semantics.
 */
static ssize_t auto_tune_render_rt_floor_pct_show(struct gov_attr_set *attr_set,
						  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->auto_tune_render_rt_floor_pct);
}

static ssize_t auto_tune_render_rt_floor_pct_store(struct gov_attr_set *attr_set,
						   const char *buf,
						   size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_RENDER_RT_FLOOR_PCT_MAX)
		return -EINVAL;
	t->auto_tune_render_rt_floor_pct = val;
	return count;
}

static struct governor_attr auto_tune_render_rt_floor_pct =
	__ATTR_RW(auto_tune_render_rt_floor_pct);

/* auto_tune_v3 sysfs knob (RW).  See ZENITH_DEFAULT_AUTO_TUNE_V3
 * comment block for full semantics.  Three accepted values:
 *
 *   0  off (default)
 *   1  observe-only (collect telemetry, do not adjust)
 *   2  apply       (collect telemetry AND adjust V2 hyst/cool windows)
 *
 * Store-side: clamps to [0, ZENITH_AT_V3_MODE_MAX], syncs the
 * zenith_auto_tune_v3_key static branch, and -- on a transition to 0
 * -- clears any accumulated offsets so a subsequent re-enable starts
 * from a clean baseline.
 */
static ssize_t auto_tune_v3_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->auto_tune_v3);
}

static ssize_t auto_tune_v3_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_policy;
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_AT_V3_MODE_MAX)
		return -EINVAL;
	old = t->auto_tune_v3;
	t->auto_tune_v3 = val;
	zenith_set_static_key(&zenith_auto_tune_v3_key, val);
	if (val == ZENITH_AT_V3_MODE_OFF) {
		list_for_each_entry(z_policy, &attr_set->policy_list,
				    tunables_hook) {
			z_policy->at_v3_hyst_offset = 0;
			z_policy->at_v3_cool_offset = 0;
			z_policy->at_v3_last_calib_ns = 0;
			z_policy->at_v3_last_transitions = 0;
			/* Patch J: clear the calibration audit ring on the
			 * V3 master-disable transition so a subsequent
			 * re-enable starts the audit trail from a clean
			 * baseline, matching the rest of the V3 fields
			 * cleared here.
			 */
			memset(z_policy->at_v3_calib_log, 0,
			       sizeof(z_policy->at_v3_calib_log));
			z_policy->at_v3_calib_log_head = 0;
			z_policy->at_v3_calib_log_count = 0;
		}
	}
	/* V3 mode flips alter the offsets fed into
	 * zenith_at_eff_hyst_windows() / zenith_at_eff_cool_windows()
	 * on the freq-decision path.  Drop any cached effective
	 * windows so the next tick recomputes on the new mode --
	 * matters most on MODE_APPLY -> MODE_OFF and MODE_DRY ->
	 * MODE_APPLY transitions where the offset surface changes
	 * shape under the cache.
	 */
	zenith_invalidate_cache(attr_set);
	if (old != val)
		zenith_log_master_flip(t, "auto_tune_v3", old, val);
	return count;
}
static struct governor_attr auto_tune_v3 = __ATTR_RW(auto_tune_v3);

/* auto_tune_v3_interval_ms sysfs knob (RW).  Calibration period in
 * milliseconds.  Default ZENITH_DEFAULT_AT_V3_INTERVAL_MS (60000);
 * clamped on store to [ZENITH_AT_V3_INTERVAL_MIN_MS,
 * ZENITH_AT_V3_INTERVAL_MAX_MS].
 */
static ssize_t auto_tune_v3_interval_ms_show(struct gov_attr_set *attr_set,
					     char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_v3_interval_ms);
}

static ssize_t auto_tune_v3_interval_ms_store(struct gov_attr_set *attr_set,
					      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_AT_V3_INTERVAL_MIN_MS)
		val = ZENITH_AT_V3_INTERVAL_MIN_MS;
	if (val > ZENITH_AT_V3_INTERVAL_MAX_MS)
		val = ZENITH_AT_V3_INTERVAL_MAX_MS;
	t->auto_tune_v3_interval_ms = val;
	return count;
}
static struct governor_attr auto_tune_v3_interval_ms =
	__ATTR_RW(auto_tune_v3_interval_ms);

/* auto_tune_v3_state sysfs knob (RO).  One line per online policy on
 * which V3 has been observed at least once:
 *
 *   policy<N>: transitions=<n> hyst_offset=<-1..+4> cool_offset=<-1..+4>
 *
 * transitions is the V2 state-transition count from the most recent
 * calibration window; the offsets are the live signed nudges (only
 * actually applied when auto_tune_v3 == 2).
 */
static ssize_t auto_tune_v3_state_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	struct zenith_policy *z_policy;
	ssize_t pos = 0;

	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		pos += sysfs_emit_at(buf, pos,
			"policy%u: transitions=%u hyst_offset=%d cool_offset=%d\n",
			z_policy->policy ? z_policy->policy->cpu : 0,
			z_policy->at_v3_last_transitions,
			(int)z_policy->at_v3_hyst_offset,
			(int)z_policy->at_v3_cool_offset);
		if (pos >= PAGE_SIZE - 80)
			break;
	}
	return pos;
}
static struct governor_attr auto_tune_v3_state =
	__ATTR_RO(auto_tune_v3_state);

/* Patch J: auto_tune_v3_calib_log RO sysfs.  Dumps the per-policy
 * V3 calibration audit ring -- one line per calibration tick,
 * oldest first, capped at ZENITH_AT_V3_CALIB_LOG_NR entries.
 * Format chosen to fit comfortably under PAGE_SIZE for the common
 * HMP topology (2 policies x 8 entries) while remaining grep
 * friendly: every key is name=value with no quoted strings or
 * commas.
 *
 * Mode is reported as the integer ZENITH_AT_V3_MODE_* value so a
 * scraper can index into it directly; hyst/cool deltas are emitted
 * as (before -> after) pairs so the operator can see at a glance
 * which entries actually moved the offsets and which were OBSERVE
 * passes or rail-clamped APPLY passes.
 */
static ssize_t auto_tune_v3_calib_log_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		unsigned int count = z_pol->at_v3_calib_log_count;
		unsigned int head = z_pol->at_v3_calib_log_head;
		unsigned int start, i;

		if (count > ZENITH_AT_V3_CALIB_LOG_NR)
			count = ZENITH_AT_V3_CALIB_LOG_NR;
		start = (count == ZENITH_AT_V3_CALIB_LOG_NR) ? head : 0;

		len += sysfs_emit_at(buf, len,
				 "policy%u: %u entries\n",
				 z_pol->policy ? z_pol->policy->cpu : 0,
				 count);
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < count; i++) {
			struct zenith_at_v3_calib_log_entry *e =
				&z_pol->at_v3_calib_log[
					(start + i) %
					ZENITH_AT_V3_CALIB_LOG_NR];

			len += sysfs_emit_at(buf, len,
				"  ts_ns=%llu mode=%u trans=%u hyst=%d->%d cool=%d->%d\n",
				(unsigned long long)e->ts_ns,
				(unsigned int)e->mode,
				e->transitions,
				(int)e->hyst_before, (int)e->hyst_after,
				(int)e->cool_before, (int)e->cool_after);
			if (len >= PAGE_SIZE)
				break;
		}
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}
static struct governor_attr auto_tune_v3_calib_log =
	__ATTR_RO(auto_tune_v3_calib_log);

static ssize_t auto_tune_cluster_aware_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_cluster_aware);
}

static ssize_t auto_tune_cluster_aware_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_cluster_aware = val;
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_reset_local_actions(z_pol);
	zenith_refresh_rate_delays(attr_set);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr auto_tune_cluster_aware =
	__ATTR_RW(auto_tune_cluster_aware);

ZENITH_TUNABLE_UINT_MAX(auto_tune_v2_signals, 1);

ZENITH_TUNABLE_UINT_MAX(auto_tune_thermal_slope, 1);

static ssize_t auto_tune_thermal_pressure_pct_show(struct gov_attr_set *attr_set,
						   char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->
		       auto_tune_thermal_pressure_pct);
}

static ssize_t auto_tune_thermal_pressure_pct_store(struct gov_attr_set *attr_set,
						    const char *buf,
						    size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_AT_THERMAL_PCT_MAX)
		return -EINVAL;
	t->auto_tune_thermal_pressure_pct = val;
	return count;
}
static struct governor_attr auto_tune_thermal_pressure_pct =
	__ATTR_RW(auto_tune_thermal_pressure_pct);

static ssize_t auto_tune_thermal_slope_pct_show(struct gov_attr_set *attr_set,
						char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->
		       auto_tune_thermal_slope_pct);
}

static ssize_t auto_tune_thermal_slope_pct_store(struct gov_attr_set *attr_set,
						 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_AT_THERMAL_PCT_MAX)
		return -EINVAL;
	t->auto_tune_thermal_slope_pct = val;
	return count;
}
static struct governor_attr auto_tune_thermal_slope_pct =
	__ATTR_RW(auto_tune_thermal_slope_pct);

ZENITH_TUNABLE_UINT_MAX(auto_tune_frame_pacing, 1);

static ssize_t auto_tune_sustained_gaming_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->
		       auto_tune_sustained_gaming);
}

static ssize_t auto_tune_sustained_gaming_store(struct gov_attr_set *attr_set,
						const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_sustained_gaming = val;
	return count;
}
static struct governor_attr auto_tune_sustained_gaming =
	__ATTR_RW(auto_tune_sustained_gaming);

/* auto_tune_* threshold tunables. The three *_pct fields are clamped to
 * the 0..100 range; the events_x2 fields accept any uint but only
 * values that can realistically occur in the 10 s observation window
 * are meaningful (events_x2 = events_per_2s, so 10 == 5 events/s).
 */
#define ZENITH_AT_PCT_STORE(_name) \
static ssize_t _name##_store(struct gov_attr_set *attr_set, \
			     const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val) || val > 100) \
		return -EINVAL; \
	t->_name = val; \
	return count; \
}

#define ZENITH_AT_PCT_SHOW(_name) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->_name); \
}

#define ZENITH_AT_PCT_TUNABLE(_name) \
	ZENITH_AT_PCT_SHOW(_name) \
	ZENITH_AT_PCT_STORE(_name) \
	static struct governor_attr _name = __ATTR_RW(_name)

ZENITH_AT_PCT_TUNABLE(auto_tune_sat_load_pct);
ZENITH_AT_PCT_TUNABLE(auto_tune_hi_sat_pct);
ZENITH_AT_PCT_TUNABLE(auto_tune_lo_sat_pct);

/* Input event rate thresholds, doubled.  Practical max in normal
 * use is a few hundred (touch event stream is 50-200 / s, doubled
 * gives 100-400); 65535 is a generous upper bound that still keeps
 * the field well inside u16 range so future packing into the V2
 * status struct stays free.
 */
ZENITH_TUNABLE_UINT_MAX(auto_tune_hi_events_x2, 65535);
ZENITH_TUNABLE_UINT_MAX(auto_tune_lo_events_x2, 65535);

/* auto_tune_scenario sysfs knob.  Strict 0/1 boolean; non-zero
 * values normalised to 1 on store.  Effective only when auto_tune=1.
 * See ZENITH_DEFAULT_AUTO_TUNE_SCENARIO comment block for the
 * detection logic and scenario precedence.
 */
ZENITH_TUNABLE_UINT_MAX(auto_tune_scenario, 1);

static ssize_t profile_show(struct gov_attr_set *attr_set, char *buf)
{
	switch (to_zenith_tunables(attr_set)->active_profile) {
	case ZENITH_PROFILE_PERFORMANCE:	return sysfs_emit(buf, "performance\n");
	case ZENITH_PROFILE_BALANCED:		return sysfs_emit(buf, "balanced\n");
	case ZENITH_PROFILE_BATTERY:		return sysfs_emit(buf, "battery\n");
	case ZENITH_PROFILE_LEGACY:		return sysfs_emit(buf, "legacy\n");
	case ZENITH_PROFILE_GAMING:		return sysfs_emit(buf, "gaming\n");
	case ZENITH_PROFILE_AUDIO:		return sysfs_emit(buf, "audio\n");
	/* Patch B-AUTO-2: AUTO is the meta-profile that engages the
	 * auto-selector engine.  Userspace sees "auto"; the concrete
	 * profile the engine has applied is exposed separately via
	 * the auto_target RO sysfs node.
	 */
	case ZENITH_PROFILE_AUTO:		return sysfs_emit(buf, "auto\n");
	case ZENITH_PROFILE_CUSTOM:
	default:				return sysfs_emit(buf, "custom\n");
	}
}

static ssize_t profile_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_policy;
	unsigned int prof;

	/* Accept the canonical name with optional trailing whitespace. */
	if (sysfs_streq(buf, "performance"))
		prof = ZENITH_PROFILE_PERFORMANCE;
	else if (sysfs_streq(buf, "balanced"))
		prof = ZENITH_PROFILE_BALANCED;
	else if (sysfs_streq(buf, "battery"))
		prof = ZENITH_PROFILE_BATTERY;
	else if (sysfs_streq(buf, "legacy"))
		prof = ZENITH_PROFILE_LEGACY;
	else if (sysfs_streq(buf, "gaming"))
		prof = ZENITH_PROFILE_GAMING;
	else if (sysfs_streq(buf, "audio"))
		prof = ZENITH_PROFILE_AUDIO;
	else if (sysfs_streq(buf, "custom"))
		prof = ZENITH_PROFILE_CUSTOM;
	/* Patch B-AUTO-2: "auto" is the meta-profile that engages the
	 * auto-selector engine (B-AUTO-3 / B-AUTO-4).  On entry we
	 * apply the BALANCED bake immediately so the device runs on a
	 * known-safe baseline until the engine's first eval lands;
	 * active_profile then becomes AUTO so the engine knows it is
	 * free to pick a concrete target.
	 */
	else if (sysfs_streq(buf, "auto"))
		prof = ZENITH_PROFILE_AUTO;
	else
		return -EINVAL;

	/* Idempotent early-return: writing the currently-active
	 * profile is a no-op.  Skipping zenith_apply_profile() avoids
	 * re-stomping the tunables (which would lose any per-knob
	 * userspace tweaks the operator layered on top of the
	 * preset), and skipping zenith_refresh_rate_delays() avoids a
	 * tunables-list walk under attr_set->update_lock for nothing.
	 * The active_profile field is already correct by definition.
	 */
	if (t->active_profile == prof)
		return count;

	/* Patch L: log the profile transition before the bake runs so
	 * the dmesg trail reads chronologically against the
	 * subsequent "applied" summary line emitted by
	 * zenith_apply_profile().  Gated on verbose_log inside the
	 * helper.
	 */
	zenith_log_profile_change(t, t->active_profile, prof);

	/* Patch B-AUTO-2: AUTO is a meta-profile, not a tunables bake.
	 * Apply the BALANCED preset immediately (so the device is on a
	 * known-safe baseline before the auto-selector engine's first
	 * eval lands), then mark active_profile = AUTO and stamp the
	 * auto_target so the engine has a starting point.  The engine
	 * itself (B-AUTO-3 / B-AUTO-4) will refine the target on its
	 * 500 ms eval cadence with 2000 ms hysteresis.
	 */
	if (prof == ZENITH_PROFILE_AUTO) {
		zenith_apply_profile(t, ZENITH_PROFILE_BALANCED);
		WRITE_ONCE(t->auto_target, ZENITH_PROFILE_BALANCED);
		t->auto_pending_target = ZENITH_PROFILE_BALANCED;
		t->auto_pending_first_seen_ns = 0;
	} else {
		zenith_apply_profile(t, prof);
	}
	t->active_profile = prof;
	t->auto_tune_override_mask = 0;
	/* Patch B-AUTO-3: AUTO entry scheduling.  On entry we kick
	 * the eval worker so the first classifier run lands after
	 * one auto_eval_ms window rather than waiting for some
	 * external tick.  schedule_delayed_work is idempotent if the
	 * worker was already pending.
	 *
	 * On AUTO exit we *do not* call cancel_delayed_work_sync from
	 * here -- governor_store holds attr_set->update_lock across
	 * this entire path, the worker also acquires that lock to
	 * apply profile mutations, and a sync cancel while the worker
	 * is waiting on the same lock would deadlock.  Instead we
	 * rely on the worker's own lock-free
	 * READ_ONCE(active_profile) early-out: at most one stale
	 * worker invocation runs after the profile flip, observes
	 * active_profile != AUTO, and self-cancels (no rearm).  The
	 * synchronous drain happens later in zenith_tunables_free,
	 * which runs from the kobject release path with no lock
	 * held.
	 */
	if (prof == ZENITH_PROFILE_AUTO)
		schedule_delayed_work(&t->eval_work,
				      msecs_to_jiffies(t->auto_eval_ms ?
						       t->auto_eval_ms :
						       ZENITH_DEFAULT_AUTO_EVAL_MS));
	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		zenith_reset_local_actions(z_policy);
		z_policy->at_last_state = ZENITH_AT_STATE_BALANCED;
		z_policy->at_last_applied_state = ZENITH_AT_STATE_BALANCED;
		z_policy->at_pending_state = ZENITH_AT_STATE_BALANCED;
		z_policy->at_pending_windows = 0;
		z_policy->at_cooldown_left = 0;
	}
	/* Profile may have mutated tunables->{up,down}_rate_limit_us;
	 * refresh the per-policy rate-delay cache on every policy
	 * sharing this tunables set so the new limits take effect on
	 * the next tick rather than persisting the previous profile's
	 * cached delays.  attr_set->update_lock is held by the
	 * governor_store wrapper, so iterating policy_list is safe.
	 */
	zenith_refresh_rate_delays(attr_set);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr profile = __ATTR_RW(profile);

/* Patch L: verbose_log RW sysfs.  Plain 0/1 boolean.  When 1, the
 * three helpers (zenith_log_profile_change, zenith_log_profile_-
 * applied, zenith_log_master_flip) emit "zenith:" prefixed
 * pr_info() lines on the user-driven sysfs write paths.  Default 0
 * so production builds aren't spammed.  Read via READ_ONCE inside
 * the helpers; written via WRITE_ONCE here.
 */
static ssize_t verbose_log_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->verbose_log);
}

static ssize_t verbose_log_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->verbose_log, val);
	return count;
}
static struct governor_attr verbose_log = __ATTR_RW(verbose_log);

/* Patch K: game_perf_burst master sysfs.  0/1 boolean.  See the
 * long ZENITH_DEFAULT_GAME_PERF_BURST comment block for the full
 * mechanism.  Toggling this also flips the matching static branch
 * so the FSM tick + floor application drop off the hot path entirely
 * when the master is 0.  Verbose-log gated dmesg trail emitted on
 * every flip (Patch L integration).
 */
static ssize_t game_perf_burst_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->game_perf_burst);
}

static ssize_t game_perf_burst_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->game_perf_burst;
	WRITE_ONCE(t->game_perf_burst, val);
	zenith_set_static_key(&zenith_game_perf_burst_key, val);
	if (old != val)
		zenith_log_master_flip(t, "game_perf_burst", old, val);
	return count;
}
static struct governor_attr game_perf_burst = __ATTR_RW(game_perf_burst);

/* Patch K: game_perf_burst_floor_pct sysfs.  Clamped to
 * [ZENITH_GAME_PERF_BURST_FLOOR_PCT_MIN .. _MAX].  WRITE_ONCE'd
 * because the value is read on the per-tick hot path inside the
 * FSM floor helper.
 */
static ssize_t game_perf_burst_floor_pct_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->game_perf_burst_floor_pct);
}

static ssize_t game_perf_burst_floor_pct_store(struct gov_attr_set *attr_set,
					       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_GAME_PERF_BURST_FLOOR_PCT_MIN)
		val = ZENITH_GAME_PERF_BURST_FLOOR_PCT_MIN;
	if (val > ZENITH_GAME_PERF_BURST_FLOOR_PCT_MAX)
		val = ZENITH_GAME_PERF_BURST_FLOOR_PCT_MAX;
	WRITE_ONCE(t->game_perf_burst_floor_pct, val);
	return count;
}
static struct governor_attr game_perf_burst_floor_pct =
	__ATTR_RW(game_perf_burst_floor_pct);

/* Patch K: game_perf_burst_thermal_ceiling_dc sysfs.  Millidegrees C
 * (matches the kernel thermal subsystem's unit, so this knob plumbs
 * straight through to thermal_zone_get_temp() comparisons).  Clamp
 * range 40..60 dC keeps the user from shooting themselves in the
 * foot with absurd values; the user requested a 45..50 dC operating
 * range so the default 48000 sits at the midpoint.
 */
static ssize_t game_perf_burst_thermal_ceiling_dc_show(
	struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
				game_perf_burst_thermal_ceiling_dc);
}

static ssize_t game_perf_burst_thermal_ceiling_dc_store(
	struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MIN)
		val = ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MIN;
	if (val > ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MAX)
		val = ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MAX;
	WRITE_ONCE(t->game_perf_burst_thermal_ceiling_dc, val);
	return count;
}
static struct governor_attr game_perf_burst_thermal_ceiling_dc =
	__ATTR_RW(game_perf_burst_thermal_ceiling_dc);

/* Patch K: game_perf_burst_disarm_grace_ms sysfs.  Hold time after
 * Signal B drops before transitioning ARMED -> COOLDOWN.  Default
 * 1000 keeps Alt+Tab snappy.  Clamp [0, 10000] (10s upper bound is
 * already absurd for "Alt+Tab grace"; anything more should be a
 * cooldown_ms tweak instead).
 */
static ssize_t game_perf_burst_disarm_grace_ms_show(
	struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
				game_perf_burst_disarm_grace_ms);
}

static ssize_t game_perf_burst_disarm_grace_ms_store(
	struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_GAME_PERF_BURST_DISARM_GRACE_MS_MAX)
		val = ZENITH_GAME_PERF_BURST_DISARM_GRACE_MS_MAX;
	WRITE_ONCE(t->game_perf_burst_disarm_grace_ms, val);
	return count;
}
static struct governor_attr game_perf_burst_disarm_grace_ms =
	__ATTR_RW(game_perf_burst_disarm_grace_ms);

/* Patch K: game_perf_burst_cooldown_ms sysfs.  Length of the
 * COOLDOWN-state linear floor glide back to 0.  Clamp [0, 60000]
 * (1 minute upper bound; longer than that is effectively a stuck
 * floor).  0 disables the glide -- floor drops to 0 immediately on
 * disarm, which is rarely what you want but is supported.
 */
static ssize_t game_perf_burst_cooldown_ms_show(struct gov_attr_set *attr_set,
						char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
				game_perf_burst_cooldown_ms);
}

static ssize_t game_perf_burst_cooldown_ms_store(struct gov_attr_set *attr_set,
						 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_GAME_PERF_BURST_COOLDOWN_MS_MAX)
		val = ZENITH_GAME_PERF_BURST_COOLDOWN_MS_MAX;
	WRITE_ONCE(t->game_perf_burst_cooldown_ms, val);
	return count;
}
static struct governor_attr game_perf_burst_cooldown_ms =
	__ATTR_RW(game_perf_burst_cooldown_ms);

/* Patch K: game_perf_burst_state RO sysfs.  Live FSM state across
 * all attached policies.  Walks the attr_set policy list and prints
 * one line per policy: "<cluster_first_cpu> <state>".  When all
 * policies are IDLE this returns a single "idle" line so userspace
 * tooling has a stable shape to grep / parse.
 *
 * Stable tokens (zenith_gpb_state_name): "idle" / "ARMED" /
 * "COOLDOWN".  Order-stable per policy attach order.
 */
static ssize_t game_perf_burst_state_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	struct zenith_policy *z_policy;
	ssize_t off = 0;
	bool any_active = false;

	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		int first_cpu = cpumask_first(z_policy->policy->cpus);
		const char *name = zenith_gpb_state_name(z_policy->gpb_state);

		off += sysfs_emit_at(buf, off, "%d %s\n",
				 first_cpu, name);
		if (z_policy->gpb_state != ZENITH_GPB_STATE_IDLE)
			any_active = true;
		if (off >= PAGE_SIZE - 32)
			break;
	}
	if (!any_active && off == 0)
		off = sysfs_emit(buf, "idle\n");
	return off;
}
static struct governor_attr game_perf_burst_state =
	__ATTR_RO(game_perf_burst_state);

/* Patch M: game_perf_burst_stats RO sysfs.  Per-policy lifetime
 * counters and the last ARMED -> COOLDOWN reason, exposed for
 * empirical tuning of disarm_grace_ms / cooldown_ms.  One line per
 * attached policy:
 *
 *   "<cluster_first_cpu> state=<name> arm=<n> disarm=<n>
 *    idle=<n> last_disarm=<token>\n"
 *
 * Token grammar:
 *   state         "idle" | "ARMED" | "COOLDOWN"
 *   last_disarm   "none" | "fast" | "sustained"
 *
 * arm:    IDLE -> ARMED + COOLDOWN -> ARMED transitions
 * disarm: ARMED -> COOLDOWN transitions
 * idle:   COOLDOWN -> IDLE transitions (full-glide completions;
 *         arm > idle means we have re-armed during cooldown).
 *
 * Zero-initialised per attach (zenith_start) so a governor switch
 * resets the counters and userspace can compute rates over a known
 * baseline.  Order-stable per policy attach order.
 */
static ssize_t game_perf_burst_stats_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	struct zenith_policy *z_policy;
	ssize_t off = 0;

	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		int first_cpu = cpumask_first(z_policy->policy->cpus);
		const char *state_name =
			zenith_gpb_state_name(z_policy->gpb_state);
		const char *disarm_name =
			zenith_gpb_disarm_name(
				z_policy->gpb_last_disarm_reason);

		off += sysfs_emit_at(buf, off,
				 "%d state=%s arm=%u disarm=%u idle=%u last_disarm=%s\n",
				 first_cpu, state_name,
				 z_policy->gpb_arm_count,
				 z_policy->gpb_disarm_count,
				 z_policy->gpb_idle_count,
				 disarm_name);
		if (off >= PAGE_SIZE - 64)
			break;
	}
	return off;
}
static struct governor_attr game_perf_burst_stats =
	__ATTR_RO(game_perf_burst_stats);

/* Patch B-AUTO-2: auto_target RO sysfs.  When active_profile ==
 * ZENITH_PROFILE_AUTO this prints the concrete profile the auto-
 * selector engine has currently applied (BALANCED, PERFORMANCE,
 * BATTERY, GAMING, AUDIO).  When active_profile != AUTO this still
 * returns the last value the engine wrote -- it is a debug-only
 * window into the engine state.  Read with READ_ONCE so a torn
 * write from the engine worker cannot produce a malformed string.
 */
static ssize_t auto_target_show(struct gov_attr_set *attr_set, char *buf)
{
	unsigned int target = READ_ONCE(to_zenith_tunables(attr_set)->auto_target);

	switch (target) {
	case ZENITH_PROFILE_PERFORMANCE:	return sysfs_emit(buf, "performance\n");
	case ZENITH_PROFILE_BALANCED:		return sysfs_emit(buf, "balanced\n");
	case ZENITH_PROFILE_BATTERY:		return sysfs_emit(buf, "battery\n");
	case ZENITH_PROFILE_GAMING:		return sysfs_emit(buf, "gaming\n");
	case ZENITH_PROFILE_AUDIO:		return sysfs_emit(buf, "audio\n");
	default:				return sysfs_emit(buf, "balanced\n");
	}
}
static struct governor_attr auto_target = __ATTR_RO(auto_target);

/* Bump ZENITH_AT_STATUS_FORMAT_VERSION whenever the layout of
 * auto_tune_status changes (new fields, reordering, renaming) so
 * userspace parsers can opt in / fail soft on unknown versions.
 * Field additions in trailing positions stay backwards-compatible
 * within the same major version.
 */
#define ZENITH_AT_STATUS_FORMAT_VERSION		1

static ssize_t auto_tune_status_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	len += sysfs_emit_at(buf, len,
			 "version=%u\n", ZENITH_AT_STATUS_FORMAT_VERSION);
	len += sysfs_emit_at(buf, len,
			 "auto_tune=%u\n", t->auto_tune);
	len += sysfs_emit_at(buf, len,
			 "auto_tune_v2=%u glides=%u tiers=%u\n",
			 t->auto_tune_v2, t->auto_tune_v2_glides,
			 t->auto_tune_v2_tiers);
	len += sysfs_emit_at(buf, len,
			 "v2_knobs=cluster:%u signals:%u thermal_slope:%u frame:%u gaming:%u\n",
			 t->auto_tune_cluster_aware,
			 t->auto_tune_v2_signals,
			 t->auto_tune_thermal_slope,
			 t->auto_tune_frame_pacing,
			 t->auto_tune_sustained_gaming);
	len += sysfs_emit_at(buf, len, "profile=%s\n",
			 zenith_profile_name(t->active_profile));
	len += sysfs_emit_at(buf, len,
			 "override_mask=0x%lx\n", t->auto_tune_override_mask);
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		len += sysfs_emit_at(buf, len,
				 "policy%u(%s): state=%s applied_state=%s pending=%s pending_windows=%u cooldown=%u reason=%s target=%s samples=%u saturated=%u sat_pct=%u events_x2=%u flags=0x%x var_x256=%u psi=%u/%u/%u thermal=%u+%u frame_us=%u local=%u eff_rate=%u/%u eff_thresh=%u/%u eff_boost=%u/%u eff_frame=%u eff_game=%u\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 zenith_at_state_name(z_pol->at_last_state),
				 zenith_at_state_name(z_pol->at_last_applied_state),
				 zenith_at_state_name(z_pol->at_pending_state),
				 z_pol->at_pending_windows,
				 z_pol->at_cooldown_left,
				 zenith_at_reason_name(z_pol->at_last_reason),
				 zenith_profile_name(z_pol->at_last_target),
				 z_pol->at_last_total,
				 z_pol->at_last_saturated,
				 z_pol->at_last_sat_pct,
				 z_pol->at_last_events_rate_x2,
				 z_pol->at_last_flags,
				 z_pol->at_last_var_x256,
				 z_pol->at_last_psi_cpu,
				 z_pol->at_last_psi_io,
				 z_pol->at_last_psi_mem,
				 z_pol->at_last_thermal_pressure,
				 z_pol->at_last_thermal_slope,
				 z_pol->at_last_frame_budget_us,
				 z_pol->at_local_actions,
				 z_pol->at_effective_up_rate_limit_us,
				 z_pol->at_effective_down_rate_limit_us,
				 z_pol->at_effective_up_threshold,
				 z_pol->at_effective_down_threshold,
				 z_pol->at_effective_input_boost_ms,
				 z_pol->at_effective_input_boost_cap_pct,
				 z_pol->at_effective_frame_pace_floor_pct,
				 z_pol->at_effective_game_mode);
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}
static struct governor_attr auto_tune_status = __ATTR_RO(auto_tune_status);

/* M1: time-in-state RO sysfs.  Format: one line per (policy, state)
 * tuple:
 *
 *   policy<cpu>(<cluster>) state=<state> residency_ns=<ns>
 *
 * Values are cumulative nanoseconds since governor start.  The
 * still-current state's counter is "live" -- it does not include
 * the time elapsed since the last commit (that delta is implicit
 * in the difference between sum-of-counters and uptime).
 * Userspace tools should treat this as monotonically non-decreasing
 * and compute differences across two reads to derive percent-time-
 * in-state for an arbitrary window.
 *
 * One line per state keeps each scnprintf() narrow enough to fit
 * in 100 columns and matches the at_log / state_history layout that
 * other zenith RO surfaces use, so awk / cut pipelines can be
 * reused.
 *
 * No reset hook -- a fresh boot zeroes the values, and
 * profile_store() does not zap residency (so cross-profile
 * comparisons stay legible).
 */
static ssize_t auto_tune_state_residency_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	ssize_t len = 0;
	unsigned int s;

	list_for_each_entry(z_pol, &t->attr_set.policy_list, tunables_hook) {
		for (s = 0; s < ARRAY_SIZE(z_pol->at_state_residency_ns); s++) {
			len += sysfs_emit_at(buf, len,
					 "policy%u(%s) state=%s residency_ns=%llu\n",
					 z_pol->policy->cpu,
					 zenith_at_cluster_name(
						z_pol->cluster_class),
					 zenith_at_state_name(s),
					 z_pol->at_state_residency_ns[s]);
			if (len >= PAGE_SIZE)
				return len;
		}
	}
	return len;
}

static struct governor_attr auto_tune_state_residency =
	__ATTR_RO(auto_tune_state_residency);

/* M2: V2 state-transition history RO sysfs.  One line per recorded
 * transition, most recent first, format:
 *
 *   policy<cpu>(<cluster>) ts=<ns> from=<state> to=<state> reason=<r> flags=0x<f>
 *
 * Bounded to ZENITH_AT_HISTORY_NR entries per policy.  The output
 * may exceed PAGE_SIZE on a fully-populated 8-cluster system; the
 * scnprintf early-out below truncates cleanly.
 *
 * Lock-free single-writer ring (V2 worker), reader (this show) may
 * see one torn entry on wrap.  Acceptable for diagnostics.
 */
static ssize_t auto_tune_state_history_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	ssize_t len = 0;
	unsigned int i, slot, head, count;

	list_for_each_entry(z_pol, &t->attr_set.policy_list, tunables_hook) {
		head = z_pol->at_history_head;
		count = z_pol->at_history_count;
		for (i = 0; i < count; i++) {
			/* Walk newest -> oldest.  Newest entry is at
			 * (head - 1) mod NR; subtract i more to step
			 * back through the ring.
			 */
			slot = (head + ZENITH_AT_HISTORY_NR - 1 - i) %
			       ZENITH_AT_HISTORY_NR;
			len += sysfs_emit_at(buf, len,
					 "policy%u(%s) ts=%llu from=%s to=%s reason=%s flags=0x%x\n",
					 z_pol->policy->cpu,
					 zenith_at_cluster_name(z_pol->cluster_class),
					 z_pol->at_history[slot].ts_ns,
					 zenith_at_state_name(z_pol->at_history[slot].from),
					 zenith_at_state_name(z_pol->at_history[slot].to),
					 zenith_at_reason_name(z_pol->at_history[slot].reason),
					 z_pol->at_history[slot].flags);
			if (len >= PAGE_SIZE)
				return len;
		}
	}
	return len;
}

static struct governor_attr auto_tune_state_history =
	__ATTR_RO(auto_tune_state_history);

static ssize_t auto_tune_reset_overrides_store(struct gov_attr_set *attr_set,
					       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	if (val)
		t->auto_tune_override_mask = 0;
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_reset_local_actions(z_pol);
	return count;
}
static struct governor_attr auto_tune_reset_overrides =
	__ATTR_WO(auto_tune_reset_overrides);

/* profile_values: readonly dump of the hardcoded preset tables.
 *
 * Invokes zenith_apply_profile() against a stack-scratch tunables
 * struct for each preset (performance, balanced, battery, legacy)
 * and prints the resulting knob values in "name=value" form,
 * one line per profile.  The CUSTOM profile is intentionally
 * skipped: it is a marker ("no preset has been applied") and has
 * no canonical values.
 *
 * zenith_apply_profile() has one side effect -- it stamps
 * zenith_input_boost_active_ms with the preset's input_boost_ms.
 * Save / restore that mirror around the loop so reading this node
 * never perturbs the running boost window.  The scratch struct
 * itself is stack-local so there is no tunables-race window.
 */
static ssize_t profile_values_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables scratch;
	u32 saved_active_ms = READ_ONCE(zenith_input_boost_active_ms);
	u32 saved_touchdown_extra_ms =
		READ_ONCE(zenith_input_boost_touchdown_extra_ms_cache);
	ssize_t len = 0;
	int i;
	static const struct {
		unsigned int id;
		const char *name;
	} profs[] = {
		{ ZENITH_PROFILE_PERFORMANCE, "performance" },
		{ ZENITH_PROFILE_BALANCED,    "balanced" },
		{ ZENITH_PROFILE_BATTERY,     "battery" },
		{ ZENITH_PROFILE_LEGACY,      "legacy" },
	};

	for (i = 0; i < ARRAY_SIZE(profs); i++) {
		memset(&scratch, 0, sizeof(scratch));
		zenith_apply_profile(&scratch, profs[i].id);
		len += sysfs_emit_at(buf, len,
			"%s: up_rate_limit_us=%u down_rate_limit_us=%u "
			"up_threshold=%u down_threshold=%u "
			"hispeed_freq_pct=%u hispeed_load=%u "
			"climb_mode=%u freq_step_pct=%u "
			"powersave_bias=%u bias_load_threshold=%u "
			"ignore_nice_load=%u input_boost_ms=%u "
			"input_boost_decay_ms=%u input_boost_cap_pct=%u "
			"light_load_threshold=%u sampling_down_factor=%u "
			"thermal_auto=%u screen_auto=%u util_math_v2=%u "
			"kcpustat_hispeed_enable=%u\n",
			profs[i].name,
			scratch.up_rate_limit_us, scratch.down_rate_limit_us,
			scratch.up_threshold, scratch.down_threshold,
			scratch.hispeed_freq_pct, scratch.hispeed_load,
			scratch.climb_mode, scratch.freq_step_pct,
			scratch.powersave_bias, scratch.bias_load_threshold,
			scratch.ignore_nice_load, scratch.input_boost_ms,
			scratch.input_boost_decay_ms,
			scratch.input_boost_cap_pct,
			scratch.light_load_threshold,
			scratch.sampling_down_factor,
			scratch.thermal_auto, scratch.screen_auto,
			scratch.util_math_v2,
			scratch.kcpustat_hispeed_enable);
		len += sysfs_emit_at(buf, len, "%s-extra: ",
				 profs[i].name);
		len += sysfs_emit_at(buf, len,
				 "down_rate_adaptive=%u ",
				 scratch.down_rate_adaptive);
		len += sysfs_emit_at(buf, len,
				 "wakeup_boost=%u ", scratch.wakeup_boost);
		len += sysfs_emit_at(buf, len,
				 "down_threshold_adaptive=%u ",
				 scratch.down_threshold_adaptive);
		len += sysfs_emit_at(buf, len,
				 "rate_limit_cluster_scale=%u\n",
				 scratch.rate_limit_cluster_scale);
	}

	/* Restore the boost-active and touchdown-extra mirrors that
	 * zenith_apply_profile() stamps on every call.  Use WRITE_ONCE
	 * to match the writer semantics elsewhere in the file.
	 */
	WRITE_ONCE(zenith_input_boost_active_ms, saved_active_ms);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache,
		   saved_touchdown_extra_ms);
	return len;
}
static struct governor_attr profile_values = __ATTR_RO(profile_values);

/* zenith_stats: readonly per-policy decision-stat dump.  One line
 * per bucket, "name=count", emitted in enum order so userspace
 * scrapers can read field-by-name without depending on a fixed
 * column count.  Counters reset on zenith_start() so values reflect
 * the current attach cycle.  See enum zenith_stat_idx for what each
 * bucket includes.
 *
 * Each policy carries its own gov_attr_set, so this attribute is
 * per-policy automatically; we walk attr_set->policy_list for the
 * (currently always one) z_policy that owns the values.  The first
 * entry's stats are reported; if multiple policies were ever to
 * share a tunables instance, later entries are summed in.
 */
static ssize_t zenith_stats_show(struct gov_attr_set *attr_set, char *buf)
{
	static const char * const zenith_stat_names[] = {
		[ZENITH_STAT_DECISIONS]		= "decisions",
		[ZENITH_STAT_CACHE_HITS]	= "cache_hits",
		[ZENITH_STAT_INPUT_BOOST]	= "input_boost",
		[ZENITH_STAT_BRUTAL]		= "brutal",
		[ZENITH_STAT_HISPEED]		= "hispeed",
		[ZENITH_STAT_FRAME_PACE]	= "frame_pace",
		[ZENITH_STAT_AUDIO]		= "audio",
		[ZENITH_STAT_RENDER_CAMERA]	= "render_camera",
		[ZENITH_STAT_UCLAMP]		= "uclamp",
		[ZENITH_STAT_PSI]		= "psi",
		[ZENITH_STAT_BOOT_BOOST]	= "boot_boost",
		[ZENITH_STAT_LIGHT_CAP]		= "light_cap",
		[ZENITH_STAT_EM_CAP]		= "em_cap",
		[ZENITH_STAT_EAS]		= "eas",
		[ZENITH_STAT_OTHER]		= "other",
		[ZENITH_STAT_PREDICT_UP]	= "predict_up",
		[ZENITH_STAT_PEAK_PREARM]	= "peak_prearm",
		[ZENITH_STAT_PEAK_RESCUE]	= "peak_rescue",
		[ZENITH_STAT_PEAK_HYST]		= "peak_hyst",
		[ZENITH_STAT_PEER_RAMP]		= "peer_ramp",
		[ZENITH_STAT_MIGRATION_FLOOR]	= "migration_floor",
		[ZENITH_STAT_PSI_CPU_FLOOR]	= "psi_cpu_floor",
		[ZENITH_STAT_FRAME_OVERRUN]	= "frame_overrun",
		[ZENITH_STAT_AUTO_THERMAL_CAP]	= "auto_thermal_cap",
		[ZENITH_STAT_QUIET_HOURS_CAP]	= "quiet_hours_cap",
	};
	unsigned long sum[ZENITH_STAT_NR] = { 0 };
	struct zenith_policy *z_pol;
	ssize_t len = 0;
	unsigned int i;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		for (i = 0; i < ZENITH_STAT_NR; i++)
			sum[i] += z_pol->stats[i];
	}

	for (i = 0; i < ZENITH_STAT_NR; i++) {
		len += sysfs_emit_at(buf, len, "%s=%lu\n",
				 zenith_stat_names[i], sum[i]);
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}
static struct governor_attr zenith_stats = __ATTR_RO(zenith_stats);

/* Reset all per-policy observability counters: zenith_stats[] and the
 * auto-tune classifier ring (at_log).  Write any non-zero value to
 * trigger; writing 0 is a no-op so a misfired "echo > stats_reset"
 * can't accidentally erase the data the operator was about to read.
 */
static ssize_t zenith_stats_reset_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (!val)
		return count;
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_policy_observability_reset(z_pol);
	return count;
}
static struct governor_attr zenith_stats_reset =
	__ATTR_WO(zenith_stats_reset);

/* Stage 4 / Patch I -- governor-wide input observability sysfs node.
 *
 * Read-only; emits one "name=value" line per atomic counter so
 * userspace can scrape by field name.  All counters are
 * monotonic-since-boot atomic64s so the reader's job is to subtract
 * a stored snapshot to get a rate.  See the block comment above
 * zenith_in_events_total for what each counter records.
 *
 * The counters are governor-wide (one set, not per-policy) because
 * input events are observed once per dispatch and applied to all
 * policies that have boosting enabled.  Putting the node on the
 * gov_attr_set means it appears under every cpufreq policy
 * directory but reads the same global atomics.
 */
static ssize_t zenith_input_stats_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sysfs_emit(buf,
		"events_total=%llu\n"
		"boosts_armed=%llu\n"
		"boosts_quiet_extended=%llu\n"
		"boosts_skipped_disabled=%llu\n"
		"boosts_early_exit=%llu\n",
		(unsigned long long)atomic64_read(&zenith_in_events_total),
		(unsigned long long)atomic64_read(&zenith_in_boosts_armed),
		(unsigned long long)atomic64_read(
			&zenith_in_boosts_quiet_extended),
		(unsigned long long)atomic64_read(
			&zenith_in_boosts_skipped_disabled),
		(unsigned long long)atomic64_read(
			&zenith_in_boosts_early_exit));
}

static struct governor_attr zenith_input_stats =
	__ATTR_RO(zenith_input_stats);

/* Read-only dump of the per-policy auto-tune classifier ring buffer.
 *
 * One line per entry, oldest first, capped at ZENITH_AT_LOG_NR per
 * policy.  Format chosen to fit comfortably under PAGE_SIZE for the
 * common HMP topology (2 policies × 16 entries) while remaining
 * grep-friendly: every key is name=value with no quoted strings or
 * commas.
 */
static ssize_t at_log_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		unsigned int count = z_pol->at_log_count;
		unsigned int head = z_pol->at_log_head;
		unsigned int start, i;

		if (count > ZENITH_AT_LOG_NR)
			count = ZENITH_AT_LOG_NR;
		start = (count == ZENITH_AT_LOG_NR) ? head : 0;

		len += sysfs_emit_at(buf, len,
				 "policy%u(%s): %u entries\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 count);
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < count; i++) {
			struct zenith_at_log_entry *e =
				&z_pol->at_log[(start + i) % ZENITH_AT_LOG_NR];

			len += sysfs_emit_at(buf, len,
				"  ts_ns=%llu reason=%s from=%s to=%s target=%s sat=%u evx2=%u thp=%u slope=%u var=%u flags=0x%x emerg=%u\n",
				(unsigned long long)e->ts_ns,
				zenith_at_reason_name(e->reason),
				zenith_at_state_name(e->v2_from_state),
				zenith_at_state_name(e->v2_to_state),
				zenith_profile_name(e->v1_target),
				e->sat_pct,
				e->events_rate_x2,
				e->thermal_pressure,
				e->thermal_slope,
				e->var_x256,
				e->flags,
				e->emergency);
			if (len >= PAGE_SIZE)
				break;
		}
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}
static struct governor_attr at_log = __ATTR_RO(at_log);

/* last_decision_path sysfs node (Patch J).  Read-only.  Dumps
 * one line per policy in attr_set->policy_list with the most
 * recent tp_path tag chosen by zenith_get_next_freq() on that
 * policy.  Format:
 *
 *   policy<cpu>(<cluster>): <tag>
 *
 * Tag pointers are .rodata literals stamped via WRITE_ONCE in
 * the eval path; reads use READ_ONCE so a torn pointer is
 * impossible.  Initial value before the first eval tick is
 * "init", set in zenith_init().
 */
static ssize_t last_decision_path_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		const char *tag = READ_ONCE(z_pol->last_decision_path);

		if (!tag)
			tag = "init";
		len += sysfs_emit_at(buf, len,
				 "policy%u(%s): %s\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 tag);
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}

static struct governor_attr last_decision_path =
	__ATTR_RO(last_decision_path);

/* Patch B7-2: decision_ring sysfs node.  Read-only.  Dumps the
 * per-policy ring of the last ZENITH_DEC_RING_NR (path, lat_us)
 * pairs newest-first.  Format:
 *
 *   policy<cpu>(<cluster>):
 *     <tag> <lat_us>
 *     ...
 *
 * The ring is a power-of-two circular buffer; entries with a NULL
 * path are uninitialised (the ring has not yet wrapped through
 * those slots since the policy was created) and are skipped.
 *
 * Reader is single-shot per sysfs read; the eval path keeps
 * advancing concurrently, so the snapshot is best-effort.
 * READ_ONCE on the path pointer makes the read torn-write-safe;
 * lat_ns is sampled without strict ordering relative to path
 * which can occasionally pair a path with the lat_ns from the
 * neighbouring slot (tolerable for an observability dump).
 */
static ssize_t decision_ring_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		unsigned int head = READ_ONCE(z_pol->dec_ring_head);
		unsigned int i;

		len += sysfs_emit_at(buf, len,
				 "policy%u(%s):\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class));
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < ZENITH_DEC_RING_NR; i++) {
			unsigned int idx =
				(head - 1 - i) & ZENITH_DEC_RING_MASK;
			const char *p = READ_ONCE(z_pol->dec_ring[idx].path);
			u32 lat_ns = z_pol->dec_ring[idx].lat_ns;

			if (!p)
				continue;
			len += sysfs_emit_at(buf, len,
					 "  %s %u\n", p, lat_ns / 1000);
			if (len >= PAGE_SIZE)
				break;
		}
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}

static struct governor_attr decision_ring = __ATTR_RO(decision_ring);

/* Patch C8: decision_confidence sysfs node.  Read-only.  Walks the
 * per-policy dec_ring (ZENITH_DEC_RING_NR == 32 entries) and tallies
 * how many of the last 32 evals were won by each tp_path.  Output
 * is one line per distinct path actually seen, sorted by descending
 * count, with absolute count and percent-of-window:
 *
 *   policy0(little):
 *     hispeed       18 56%
 *     predict_up     6 18%
 *     pelt_edge      4 12%
 *     dl_floor       2  6%
 *     ...
 *
 * Useful for diagnosing which tier is doing the work in a given
 * load profile -- e.g. confirming pelt_edge is firing on cold-wake
 * scrolling, or checking that dl_floor is dormant on BALANCED.
 *
 * Cost: one PAGE_SIZE buffer pass per policy on read.  No locking
 * (READ_ONCE on path; tally is on stack).  Pointer-compare is
 * sufficient because tp_path is always a string literal -- gcc
 * pools all literals so the same path always points to the same
 * address; if a future tier writes a non-literal, strcmp would be
 * needed but the existing dec_ring sample logic stores the literal
 * pointer as well, so any path captured into the ring is also a
 * literal.
 */
#define ZENITH_CONF_TALLY_MAX	16

static ssize_t
decision_confidence_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		struct {
			const char	*path;
			unsigned int	count;
		} tally[ZENITH_CONF_TALLY_MAX];
		unsigned int n_tally = 0;
		unsigned int total = 0;
		unsigned int head = READ_ONCE(z_pol->dec_ring_head);
		unsigned int i, j;

		memset(tally, 0, sizeof(tally));

		for (i = 0; i < ZENITH_DEC_RING_NR; i++) {
			unsigned int idx =
				(head - 1 - i) & ZENITH_DEC_RING_MASK;
			const char *p = READ_ONCE(z_pol->dec_ring[idx].path);

			if (!p)
				continue;
			total++;

			for (j = 0; j < n_tally; j++) {
				if (tally[j].path == p) {
					tally[j].count++;
					break;
				}
			}
			if (j == n_tally && n_tally < ZENITH_CONF_TALLY_MAX) {
				tally[n_tally].path = p;
				tally[n_tally].count = 1;
				n_tally++;
			}
		}

		for (i = 0; i + 1 < n_tally; i++) {
			unsigned int max_j = i;

			for (j = i + 1; j < n_tally; j++) {
				if (tally[j].count > tally[max_j].count)
					max_j = j;
			}
			if (max_j != i) {
				const char *tp = tally[i].path;
				unsigned int tc = tally[i].count;

				tally[i].path = tally[max_j].path;
				tally[i].count = tally[max_j].count;
				tally[max_j].path = tp;
				tally[max_j].count = tc;
			}
		}

		len += sysfs_emit_at(buf, len,
				 "policy%u(%s) total=%u:\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 total);
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < n_tally; i++) {
			unsigned int pct = total ?
				(tally[i].count * 100) / total : 0;

			len += sysfs_emit_at(buf, len,
					 "  %-16s %3u %3u%%\n",
					 tally[i].path, tally[i].count, pct);
			if (len >= PAGE_SIZE)
				break;
		}
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}

static struct governor_attr decision_confidence =
	__ATTR_RO(decision_confidence);

ZENITH_TUNABLE_UINT_BOOL_INVAL(screen_state);

/* screen_off_glide_ms sysfs knob.  Range
 * 0..ZENITH_SCREEN_OFF_GLIDE_MS_MAX.  See struct zenith_tunables for
 * semantics.  0 keeps the legacy hard cliff at the screen-state 1->0
 * edge; non-zero arms a linear ramp on both dynamic_up_thresh and
 * dynamic_bias from the natural up_threshold / powersave_bias to
 * the cliff targets across the configured window.
 */
static ssize_t screen_off_glide_ms_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->screen_off_glide_ms);
}

static ssize_t screen_off_glide_ms_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_SCREEN_OFF_GLIDE_MS_MAX)
		return -EINVAL;
	t->screen_off_glide_ms = val;
	return count;
}
static struct governor_attr screen_off_glide_ms =
	__ATTR_RW(screen_off_glide_ms);

ZENITH_TUNABLE_UINT_MAX(screen_auto, 1);
ZENITH_TUNABLE_UINT_BOOL_INVAL(thermal_state);

ZENITH_TUNABLE_UINT_MAX(thermal_auto, 1);

/* thermal_pressure_continuous sysfs knob.  Strict 0/1 boolean.  When
 * 1, dynamic_up_thresh ramps linearly from the policy's normal
 * up_threshold (at 0 %% thermal pressure) to 90 (at 100 %% thermal
 * pressure) instead of cliff-jumping to 90 the instant
 * zenith_thermal_active() flips true.  See the field comment on
 * struct zenith_tunables for the rationale.
 */
ZENITH_TUNABLE_UINT_BOOL_INVAL(thermal_pressure_continuous);

/* thermal_aware sysfs knob.  Master gate over the cluster of
 * thermal-driven freq adjustments (thermal_util_derate, the
 * thermal_pressure_continuous up_thresh ramp, auto_thermal_cap, the
 * V2 THERMAL_RECOVERY transitions).  Strict 0/1 boolean.  Mirrors
 * the value into zenith_thermal_aware_key so the gated branches
 * fold to no-ops in the off case.  See ZENITH_DEFAULT_THERMAL_AWARE
 * for rationale.
 */
static ssize_t thermal_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->thermal_aware);
}

static ssize_t thermal_aware_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->thermal_aware;
	t->thermal_aware = val;
	zenith_set_static_key(&zenith_thermal_aware_key, val);
	/* Master-switch flips alter the freq-decision surface fed by
	 * the thermal cluster (thermal_util_derate, the
	 * thermal_pressure_continuous up_thresh ramp, auto_thermal_cap,
	 * V2 THERMAL_RECOVERY).  Drop any cached snapshots so the
	 * next tick recomputes on the new gate state, matching the
	 * symmetry of thermal_pressure_continuous_store above.
	 */
	zenith_invalidate_cache(attr_set);
	if (old != val)
		zenith_log_master_flip(t, "thermal_aware", old, val);
	return count;
}
static struct governor_attr thermal_aware = __ATTR_RW(thermal_aware);

/* thermal_active sysfs knob.  Read-only mirror of
 * zenith_thermal_active(): 1 when the governor currently believes
 * thermal pressure is at or above the threshold, 0 otherwise.
 * Written by the kernel from zenith_thermal_active() on every call.
 * Always 0 when thermal_aware is 0.  Read with READ_ONCE because
 * the writes from per-policy update paths can race with sysfs
 * reads, and tearing the unsigned int read across CPUs is benign
 * but observable.
 */
static ssize_t thermal_active_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       READ_ONCE(to_zenith_tunables(attr_set)->thermal_active));
}
static struct governor_attr thermal_active = __ATTR_RO(thermal_active);

/* thermal_util_derate sysfs knob.  Strict 0/1 boolean.  See the
 * ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block for semantics.
 */
ZENITH_TUNABLE_UINT_MAX(thermal_util_derate, 1);

ZENITH_TUNABLE_UINT_MAX(thermal_derate_rate_pct, 100);

static ssize_t auto_thermal_cap_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_thermal_cap);
}

static ssize_t auto_thermal_cap_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_thermal_cap = val;
	/* Cap master switch governs whether the auto_thermal_cap_*_pct
	 * pair feeds the freq decision; flip needs a cache drop so the
	 * next tick recomputes the headroom on the new gate state.
	 */
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr auto_thermal_cap =
	__ATTR_RW(auto_thermal_cap);

static ssize_t auto_thermal_cap_pressure_pct_show(struct gov_attr_set *attr_set,
						  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
			       auto_thermal_cap_pressure_pct);
}

static ssize_t auto_thermal_cap_pressure_pct_store(struct gov_attr_set *attr_set,
						   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MIN ||
	    val > ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MAX)
		return -EINVAL;
	t->auto_thermal_cap_pressure_pct = val;
	return count;
}
static struct governor_attr auto_thermal_cap_pressure_pct =
	__ATTR_RW(auto_thermal_cap_pressure_pct);

static ssize_t auto_thermal_cap_freq_pct_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_thermal_cap_freq_pct);
}

static ssize_t auto_thermal_cap_freq_pct_store(struct gov_attr_set *attr_set,
					       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MIN ||
	    val > ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MAX)
		return -EINVAL;
	t->auto_thermal_cap_freq_pct = val;
	return count;
}
static struct governor_attr auto_thermal_cap_freq_pct =
	__ATTR_RW(auto_thermal_cap_freq_pct);

static ssize_t freq_stability_margin_pct_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
				freq_stability_margin_pct);
}

static ssize_t freq_stability_margin_pct_store(struct gov_attr_set *attr_set,
					       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX)
		return -EINVAL;
	t->freq_stability_margin_pct = val;
	return count;
}
static struct governor_attr freq_stability_margin_pct =
	__ATTR_RW(freq_stability_margin_pct);

static ssize_t down_rate_adaptive_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->down_rate_adaptive);
}

static ssize_t down_rate_adaptive_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->down_rate_adaptive = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE);
	return count;
}
static struct governor_attr down_rate_adaptive =
	__ATTR_RW(down_rate_adaptive);

ZENITH_TUNABLE_UINT_MAX(wakeup_boost, 1);

/* wakeup_boost_ms sysfs knob.  Range 0..ZENITH_WAKEUP_BOOST_MS_MAX.
 * 0 disables the wall-clock bypass and leaves only the legacy
 * tick-based ZENITH_WAKEUP_BOOST_TICKS countdown.  Non-zero arms a
 * deadline at the detection sites; the up-rate bypass holds until
 * either the tick counter expires or the deadline lapses.
 */
ZENITH_TUNABLE_UINT_MAX(wakeup_boost_ms, ZENITH_WAKEUP_BOOST_MS_MAX);

static ssize_t rate_limit_cluster_scale_show(struct gov_attr_set *attr_set,
					     char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->rate_limit_cluster_scale);
}

static ssize_t rate_limit_cluster_scale_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->rate_limit_cluster_scale = val;
	zenith_refresh_rate_delays(attr_set);
	return count;
}
static struct governor_attr rate_limit_cluster_scale =
	__ATTR_RW(rate_limit_cluster_scale);

static ssize_t input_boost_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->input_boost_ms);
}

static ssize_t input_boost_ms_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1000)
		return -EINVAL;
	t->input_boost_ms = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_INPUT_BOOST_MS);
	WRITE_ONCE(zenith_input_boost_active_ms, val);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr input_boost_ms = __ATTR_RW(input_boost_ms);

ZENITH_TUNABLE_UINT_MAX_INVAL(input_boost_decay_ms, 1000);

/* input_boost_touchdown_extra_ms sysfs knob (Patch C).
 *
 * Extends the input-boost active window by an extra
 * input_boost_touchdown_extra_ms ms on EV_KEY/BTN_TOUCH press.
 * 0 disables the touchdown extra.  Capped at
 * ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX so a runaway echo
 * can't pin the cluster up indefinitely.
 *
 * Mirrors the stored value to
 * zenith_input_boost_touchdown_extra_ms_cache so the input fast
 * path doesn't have to walk the tunables list.
 */
static ssize_t
input_boost_touchdown_extra_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_touchdown_extra_ms);
}

static ssize_t
input_boost_touchdown_extra_ms_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->input_boost_touchdown_extra_ms, val);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache, val);
	return count;
}

static struct governor_attr input_boost_touchdown_extra_ms =
	__ATTR_RW(input_boost_touchdown_extra_ms);

/* input_boost_decay_curve sysfs knob.  0 = linear (legacy),
 * 1 = cubic ease-in.  See ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE.
 */
ZENITH_TUNABLE_UINT_MAX(input_boost_decay_curve, 1);

ZENITH_TUNABLE_UINT_MAX(input_boost_big_only, 1);

static ssize_t input_boost_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_cap_pct);
}

static ssize_t input_boost_cap_pct_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->input_boost_cap_pct = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP);
	return count;
}
static struct governor_attr input_boost_cap_pct =
	__ATTR_RW(input_boost_cap_pct);

/* input_boost_down_rate_mult_pct sysfs knob.  Range
 * 100..ZENITH_INPUT_BOOST_DOWN_RATE_MULT_PCT_MAX (1000).  See
 * ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT for full semantics.
 */
static ssize_t input_boost_down_rate_mult_pct_show(struct gov_attr_set *attr_set,
						   char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_down_rate_mult_pct);
}

static ssize_t input_boost_down_rate_mult_pct_store(struct gov_attr_set *attr_set,
						    const char *buf,
						    size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val < 100 ||
	    val > ZENITH_INPUT_BOOST_DOWN_RATE_MULT_PCT_MAX)
		return -EINVAL;
	t->input_boost_down_rate_mult_pct = val;
	return count;
}

static struct governor_attr input_boost_down_rate_mult_pct =
	__ATTR_RW(input_boost_down_rate_mult_pct);

/* Parse up to ZENITH_EFF_BINS_MAX unsigned ints separated by whitespace
 * into out[], returning the number parsed. Extra tokens are ignored.
 * Returns -EINVAL if any token fails kstrtouint or if no tokens parse.
 */
static int zenith_parse_uint_list(const char *buf, unsigned int *out,
				  unsigned int max)
{
	char tmp[16];
	const char *p = buf;
	unsigned int nr = 0;
	int ret;

	while (*p && nr < max) {
		size_t len = 0;
		unsigned int val;

		while (*p == ' ' || *p == '\t' || *p == '\n')
			p++;
		if (!*p)
			break;

		while (p[len] && p[len] != ' ' && p[len] != '\t' &&
		       p[len] != '\n' && len < sizeof(tmp) - 1)
			len++;
		if (!len)
			break;
		memcpy(tmp, p, len);
		tmp[len] = '\0';
		p += len;

		ret = kstrtouint(tmp, 10, &val);
		if (ret)
			return -EINVAL;
		out[nr++] = val;
	}

	return nr ? (int)nr : -EINVAL;
}

static ssize_t efficient_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int i;
	ssize_t len = 0;

	if (!t->eff_nr)
		return sysfs_emit(buf, "0\n");

	for (i = 0; i < t->eff_nr; i++)
		len += sysfs_emit_at(buf, len, "%u%c",
			       t->eff_freq[i],
			       (i + 1 == t->eff_nr) ? '\n' : ' ');
	return len;
}

static ssize_t efficient_freq_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int parsed[ZENITH_EFF_BINS_MAX];
	int n, i;

	n = zenith_parse_uint_list(buf, parsed, ZENITH_EFF_BINS_MAX);
	if (n < 0)
		return n;

	/* "efficient_freq=0" disables the ladder. */
	if (n == 1 && parsed[0] == 0) {
		t->eff_nr = 0;
		t->efficient_freq = 0;
		zenith_invalidate_cache(attr_set);
		return count;
	}

	/* Require strictly ascending sort — the ladder walks from low
	 * to high, and equal / decreasing entries would create
	 * unreachable bins.
	 */
	for (i = 1; i < n; i++) {
		if (parsed[i] <= parsed[i - 1])
			return -EINVAL;
	}

	for (i = 0; i < n; i++)
		t->eff_freq[i] = parsed[i];

	/* Broadcast the existing scalar up_delay_us to any new bins
	 * that don't yet have a delay. The user can then write a
	 * matching list to up_delay_us to override per-bin.
	 */
	for (i = 0; i < n; i++)
		if (!t->eff_delay_us[i])
			t->eff_delay_us[i] = t->up_delay_us;

	t->eff_nr = n;
	t->efficient_freq = t->eff_freq[0];
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr efficient_freq = __ATTR_RW(efficient_freq);

static ssize_t eff_bin_hyst_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->eff_bin_hyst_pct);
}

static ssize_t eff_bin_hyst_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_EFF_BIN_HYST_PCT_MAX)
		return -EINVAL;
	t->eff_bin_hyst_pct = val;
	return count;
}
static struct governor_attr eff_bin_hyst_pct = __ATTR_RW(eff_bin_hyst_pct);

static ssize_t up_delay_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int i;
	ssize_t len = 0;

	if (!t->eff_nr)
		return sysfs_emit(buf, "%u\n", t->up_delay_us);

	for (i = 0; i < t->eff_nr; i++)
		len += sysfs_emit_at(buf, len, "%u%c",
			       t->eff_delay_us[i],
			       (i + 1 == t->eff_nr) ? '\n' : ' ');
	return len;
}

static ssize_t up_delay_us_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int parsed[ZENITH_EFF_BINS_MAX];
	int n, i;

	n = zenith_parse_uint_list(buf, parsed, ZENITH_EFF_BINS_MAX);
	if (n < 0)
		return n;

	for (i = 0; i < n; i++)
		if (parsed[i] > 1000000)
			return -EINVAL;

	if (n == 1) {
		/* Scalar write: broadcast to every existing bin and
		 * keep the scalar shadow up-to-date.
		 */
		t->up_delay_us = parsed[0];
		for (i = 0; i < ZENITH_EFF_BINS_MAX; i++)
			t->eff_delay_us[i] = parsed[0];
		zenith_invalidate_cache(attr_set);
		return count;
	}

	/* Vector write: must match eff_nr exactly. */
	if (!t->eff_nr || (unsigned int)n != t->eff_nr)
		return -EINVAL;

	for (i = 0; i < n; i++)
		t->eff_delay_us[i] = parsed[i];
	t->up_delay_us = parsed[0];
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr up_delay_us = __ATTR_RW(up_delay_us);

ZENITH_TUNABLE_UINT_MAX_INVAL(light_load_freq, ZENITH_LIGHT_LOAD_FREQ_MAX);

ZENITH_TUNABLE_UINT_MAX_INVAL(light_load_threshold, 100);

static ssize_t sampling_down_factor_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->sampling_down_factor);
}

static ssize_t sampling_down_factor_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 ||
	    val > ZENITH_MAX_SAMPLING_DOWN_FACTOR)
		return -EINVAL;
	t->sampling_down_factor = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr sampling_down_factor = __ATTR_RW(sampling_down_factor);

ZENITH_TUNABLE_UINT_MAX(boost_exit_extend, 1);

ZENITH_TUNABLE_UINT_MAX_INVAL(bias_load_threshold, 100);

static ssize_t up_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->up_threshold);
}

static ssize_t up_threshold_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->up_threshold = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_UP_THRESHOLD);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr up_threshold = __ATTR_RW(up_threshold);

static ssize_t up_threshold_adaptive_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->up_threshold_adaptive);
}

static ssize_t up_threshold_adaptive_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_UP_THRESHOLD_ADAPTIVE_MAX)
		return -EINVAL;
	t->up_threshold_adaptive = val;
	return count;
}
static struct governor_attr up_threshold_adaptive =
	__ATTR_RW(up_threshold_adaptive);

static ssize_t up_threshold_hispeed_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->up_threshold_hispeed);
}

static ssize_t up_threshold_hispeed_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	/* 0 = disabled (fall back to up_threshold). >100 is nonsense for
	 * a load percentage; up_threshold_hispeed < up_threshold is also
	 * nonsense because it would widen the hispeed capture zone
	 * rather than narrow it, but the user is free to do that if they
	 * really want to.
	 */
	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->up_threshold_hispeed = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr up_threshold_hispeed =
	__ATTR_RW(up_threshold_hispeed);

static ssize_t down_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->down_threshold);
}

static ssize_t down_threshold_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->down_threshold = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_DOWN_THRESHOLD);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr down_threshold = __ATTR_RW(down_threshold);

static ssize_t down_threshold_adaptive_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->down_threshold_adaptive);
}

static ssize_t down_threshold_adaptive_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX)
		return -EINVAL;
	t->down_threshold_adaptive = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr down_threshold_adaptive =
	__ATTR_RW(down_threshold_adaptive);

ZENITH_TUNABLE_UINT_MAX_INVAL(hispeed_freq, ZENITH_HISPEED_FREQ_MAX);

ZENITH_TUNABLE_UINT_MAX_INVAL(hispeed_freq_pct, 100);

static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->hispeed_load);
}

static ssize_t hispeed_load_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->hispeed_load = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr hispeed_load = __ATTR_RW(hispeed_load);

ZENITH_TUNABLE_UINT_MAX(hispeed_hyst_pct, 100);

/* hispeed_entry_streak sysfs knob.  See ZENITH_DEFAULT_HISPEED_ENTRY_STREAK
 * for semantics.  Capped to ZENITH_HISPEED_ENTRY_STREAK_MAX on store
 * so the per-policy u8 streak counter cannot overflow.
 */
static ssize_t hispeed_entry_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->hispeed_entry_streak);
}

static ssize_t hispeed_entry_streak_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_HISPEED_ENTRY_STREAK_MAX)
		return -EINVAL;
	t->hispeed_entry_streak = val;
	return count;
}
static struct governor_attr hispeed_entry_streak =
	__ATTR_RW(hispeed_entry_streak);

/* brutal_entry_streak sysfs knob.  See ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK
 * for semantics.  Capped to ZENITH_BRUTAL_ENTRY_STREAK_MAX on store
 * so the per-policy u8 streak counter cannot overflow.
 */
static ssize_t brutal_entry_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->brutal_entry_streak);
}

static ssize_t brutal_entry_streak_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_BRUTAL_ENTRY_STREAK_MAX)
		return -EINVAL;
	t->brutal_entry_streak = val;
	return count;
}
static struct governor_attr brutal_entry_streak =
	__ATTR_RW(brutal_entry_streak);

/* peak_headroom_rescue sysfs knob.  Boolean gate for the watchdog
 * tier that lifts the cluster off a sustained-high-util / sub-peak
 * floor.  See ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE for the full
 * rationale.  Accepts 0 (disabled) or 1 (enabled, default); any
 * other value is rejected.
 */
ZENITH_TUNABLE_UINT_MAX(peak_headroom_rescue, 1);

/* peak_headroom_starve_load_pct sysfs knob.  Minimum cluster
 * load_pct (0..100, util / max_cap * 100) at which a sample counts
 * as "starving" for the peak-headroom rescue tier.  Accepts 1..100;
 * 0 is rejected (rescue would fire on every sample).
 */
static ssize_t peak_headroom_starve_load_pct_show(struct gov_attr_set *attr_set,
						  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_starve_load_pct);
}

static ssize_t peak_headroom_starve_load_pct_store(struct gov_attr_set *attr_set,
						   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->peak_headroom_starve_load_pct = val;
	return count;
}

static struct governor_attr peak_headroom_starve_load_pct =
	__ATTR_RW(peak_headroom_starve_load_pct);

/* peak_headroom_freq_floor_pct sysfs knob.  Maximum fraction of
 * policy->max (1..100) below which the cluster freq must sit before
 * a sample counts as "starving" for the rescue tier.  100 means
 * "any time freq < policy->max"; smaller values demand a deeper
 * sub-peak gap before triggering.  0 is rejected (any positive freq
 * would be above 0% of policy->max so the freq guard would never
 * fire).
 */
static ssize_t peak_headroom_freq_floor_pct_show(struct gov_attr_set *attr_set,
						 char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_freq_floor_pct);
}

static ssize_t peak_headroom_freq_floor_pct_store(struct gov_attr_set *attr_set,
						  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->peak_headroom_freq_floor_pct = val;
	return count;
}

static struct governor_attr peak_headroom_freq_floor_pct =
	__ATTR_RW(peak_headroom_freq_floor_pct);

/* peak_headroom_starve_streak sysfs knob.  Number of consecutive
 * starving samples required before the rescue may fire.  Accepts
 * 0..PEAK_HEADROOM_STREAK_MAX (16) since the underlying counter is
 * a u8 saturating at that value.  0 fires on the very first
 * starving sample.
 */
ZENITH_TUNABLE_UINT_MAX(peak_headroom_starve_streak, ZENITH_PEAK_HEADROOM_STREAK_MAX);

/* peak_headroom_jump_pct sysfs knob.  Target as percentage of
 * policy->max for the rescue freq.  Accepts 1..100; 100 pins to
 * policy->max (the default), 50 rescues to half of policy->max,
 * etc.  0 is rejected (rescue with target 0 would never raise
 * freq).
 */
static ssize_t peak_headroom_jump_pct_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_jump_pct);
}

static ssize_t peak_headroom_jump_pct_store(struct gov_attr_set *attr_set,
					    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->peak_headroom_jump_pct = val;
	return count;
}

static struct governor_attr peak_headroom_jump_pct =
	__ATTR_RW(peak_headroom_jump_pct);

/* peak_headroom_hold_ms sysfs knob.  Minimum gap between two
 * rescue fires, in milliseconds.  Accepts
 * 0..PEAK_HEADROOM_HOLD_MS_MAX (1000); 0 disables the hold-down
 * (rescue can fire on every starving sample past the streak),
 * larger values rate-limit more aggressively.
 */
static ssize_t peak_headroom_hold_ms_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_hold_ms);
}

static ssize_t peak_headroom_hold_ms_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEAK_HEADROOM_HOLD_MS_MAX)
		return -EINVAL;
	t->peak_headroom_hold_ms = val;
	return count;
}

static struct governor_attr peak_headroom_hold_ms =
	__ATTR_RW(peak_headroom_hold_ms);

/* batt_hold_scale_pct sysfs knob (Patch 1.2).  Percentage applied
 * to peak-rescue hold-down deadlines when the system is on
 * battery (zenith_on_battery == 1).  Profile-baked: PERFORMANCE
 * keeps 100 (no scaling), BALANCED extends to 120, BATTERY to
 * 180, LEGACY 100.  Accepts 50..300.
 */
static ssize_t batt_hold_scale_pct_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->batt_hold_scale_pct);
}

static ssize_t batt_hold_scale_pct_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_BATT_HOLD_SCALE_PCT_MIN ||
	    val > ZENITH_BATT_HOLD_SCALE_PCT_MAX)
		return -EINVAL;
	t->batt_hold_scale_pct = val;
	return count;
}

static struct governor_attr batt_hold_scale_pct =
	__ATTR_RW(batt_hold_scale_pct);

/* Wave A charger-aware floor knobs.  charger_aware is a 0/1 gate;
 * charger_floor_pct is the floor as a percentage of policy->max,
 * applied when the gate is on AND the AC-vs-battery cache reports
 * !on_battery.  See the comment block above
 * ZENITH_DEFAULT_CHARGER_AWARE for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(charger_aware, 1);
ZENITH_TUNABLE_UINT_MAX(charger_floor_pct, ZENITH_CHARGER_FLOOR_PCT_MAX);

/* Wave A cgroup-aware top-app floor knobs.  top_app_aware is a 0/1
 * gate; top_app_floor_pct is the floor as a percentage of
 * policy->max, applied when the gate is on AND any CPU in the
 * policy is currently running a task in the cpuset cgroup named
 * "top-app".  See the comment block above
 * ZENITH_DEFAULT_TOP_APP_AWARE for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(top_app_aware, 1);
ZENITH_TUNABLE_UINT_MAX(top_app_floor_pct, ZENITH_TOP_APP_FLOOR_PCT_MAX);

/* Wave A render-thread util tracker knobs.  render_thread_util_aware
 * is a 0/1 gate; render_thread_util_thresh is the util_avg threshold
 * (1/SCHED_CAPACITY_SCALE units, 0..1024); render_thread_util_floor_pct
 * is the floor as a percentage of policy->max applied when the gate
 * is on AND a render thread is observed AND its util_avg >= thresh.
 * See the comment block above ZENITH_DEFAULT_RENDER_THREAD_UTIL_AWARE
 * for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(render_thread_util_aware, 1);
ZENITH_TUNABLE_UINT_MAX(render_thread_util_thresh,
			ZENITH_RENDER_THREAD_UTIL_THRESH_MAX);
ZENITH_TUNABLE_UINT_MAX(render_thread_util_floor_pct,
			ZENITH_RENDER_THREAD_UTIL_FLOOR_PCT_MAX);

/* Wave B PMU IPC tracker knobs.  pmu_aware is a 0/1 gate; pmu_ipc_-
 * thresh is the IPC threshold in percent (100 = 1.0 IPC, default
 * 100, max 1000); pmu_ipc_floor_pct is the floor as a percentage of
 * policy->max applied when the gate is on AND the policy's max
 * sampled IPC across CPUs is >= thresh.  See the comment block above
 * ZENITH_DEFAULT_PMU_AWARE for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(pmu_aware, 1);
ZENITH_TUNABLE_UINT_MAX(pmu_ipc_thresh, ZENITH_PMU_IPC_THRESH_MAX);
ZENITH_TUNABLE_UINT_MAX(pmu_ipc_floor_pct, ZENITH_PMU_IPC_FLOOR_PCT_MAX);

/* Wave B EAS energy-knee floor knobs.  em_aware is a 0/1 gate;
 * em_floor_pct is the floor as a percentage of the policy's energy-
 * knee freq, capped at 200% to allow the user to express "a bit
 * above the knee for safety margin".  See the comment block above
 * ZENITH_DEFAULT_EM_AWARE for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(em_aware, 1);
ZENITH_TUNABLE_UINT_MAX(em_floor_pct, ZENITH_EM_FLOOR_PCT_MAX);

/* on_battery sysfs read-only diagnostic (Patch 1.2).  Reports the
 * current AC-vs-battery cache state (0 = AC / system-supplied, 1
 * = on battery).  Updated lazily once per ZENITH_AUTO_TUNE_PERIOD
 * by zenith_auto_tune_work().  Useful for an operator wondering
 * why a battery-aware tunable is or isn't engaging.
 */
static ssize_t on_battery_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       (unsigned int)atomic_read(&zenith_on_battery));
}

static struct governor_attr on_battery = __ATTR_RO(on_battery);

/* cluster_wake_pulse_ms sysfs knob (Patch 1.3).  Width of the soft
 * floor armed when the cluster wakes from a >= cluster_wake_pulse_-
 * idle_ms gap.  Accepts 0 (disable the tier) up to ZENITH_CLUSTER_-
 * WAKE_PULSE_MS_MAX (200 ms).
 */
static ssize_t cluster_wake_pulse_ms_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->cluster_wake_pulse_ms);
}

static ssize_t cluster_wake_pulse_ms_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_CLUSTER_WAKE_PULSE_MS_MAX)
		return -EINVAL;
	t->cluster_wake_pulse_ms = val;
	return count;
}

static struct governor_attr cluster_wake_pulse_ms =
	__ATTR_RW(cluster_wake_pulse_ms);

/* cluster_wake_pulse_idle_ms sysfs knob.  Minimum gap between
 * consecutive evals required before the wake-pulse arms.  Bounded
 * to 0..ZENITH_CLUSTER_WAKE_PULSE_IDLE_MS_MAX (1000 ms).  0 means
 * "any gap qualifies", which combined with cluster_wake_pulse_ms
 * non-zero would arm the pulse on every eval; users wanting that
 * effect should set both knobs explicitly.
 */
static ssize_t cluster_wake_pulse_idle_ms_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->cluster_wake_pulse_idle_ms);
}

static ssize_t cluster_wake_pulse_idle_ms_store(struct gov_attr_set *attr_set,
						const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_CLUSTER_WAKE_PULSE_IDLE_MS_MAX)
		return -EINVAL;
	t->cluster_wake_pulse_idle_ms = val;
	return count;
}

static struct governor_attr cluster_wake_pulse_idle_ms =
	__ATTR_RW(cluster_wake_pulse_idle_ms);

/* cluster_wake_pulse_floor_pct sysfs knob.  Floor as percentage of
 * policy->max held for the wake-pulse window.  Accepts 0..100;
 * 0 stamps the deadline but suppresses the floor application,
 * mirroring the peer_ramp / migration_floor knob shape.
 */
ZENITH_TUNABLE_UINT_MAX(cluster_wake_pulse_floor_pct, 100);

/* quiet_hours_start_min / quiet_hours_end_min sysfs knobs (Patch
 * 1.10).  Both accept 0..1439 (minutes since 00:00 UTC).  When
 * start == end, the tier is disabled.  When start > end, the
 * window wraps midnight.  See zenith_in_quiet_hours().
 */
static ssize_t quiet_hours_start_min_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->quiet_hours_start_min);
}

static ssize_t quiet_hours_start_min_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_QUIET_HOURS_MINUTE_MAX)
		return -EINVAL;
	t->quiet_hours_start_min = val;
	return count;
}

static struct governor_attr quiet_hours_start_min =
	__ATTR_RW(quiet_hours_start_min);

static ssize_t quiet_hours_end_min_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->quiet_hours_end_min);
}

static ssize_t quiet_hours_end_min_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_QUIET_HOURS_MINUTE_MAX)
		return -EINVAL;
	t->quiet_hours_end_min = val;
	return count;
}

static struct governor_attr quiet_hours_end_min =
	__ATTR_RW(quiet_hours_end_min);

/* quiet_hours_cap_pct: 50..100, profile-baked.  100 disables the
 * cap (no-op, default).  Smaller values cap freq harder during
 * the window.  Floor of 50 mirrors batt_hold_scale_pct's lower
 * bound and avoids surprising users with a near-min cap.
 */
static ssize_t quiet_hours_cap_pct_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->quiet_hours_cap_pct);
}

static ssize_t quiet_hours_cap_pct_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_QUIET_HOURS_CAP_PCT_MIN || val > 100)
		return -EINVAL;
	t->quiet_hours_cap_pct = val;
	return count;
}

static struct governor_attr quiet_hours_cap_pct =
	__ATTR_RW(quiet_hours_cap_pct);

/* quiet_hours_screen_off_only: 0 / 1.  Default 1 -- the cap only
 * fires while the screen is off, so a window that overlaps an
 * active call / alarm doesn't drag the cluster down.
 */
ZENITH_TUNABLE_UINT_MAX(quiet_hours_screen_off_only, 1);

/* Patch 1.4: decision_latency_hist sysfs node.
 *
 * Read-only.  Prints four numbers separated by spaces, terminated
 * with a newline:
 *
 *   <count_lt10us> <count_10_50us> <count_50_100us> <count_ge100us>
 *
 * Each count is the lifetime number of zenith_get_next_freq()
 * evals whose end-to-end cost (from function entry to commit
 * point) landed in the corresponding bucket on the policy that
 * owns this attribute set.  Reset on policy attach (zenith_start),
 * so the values reflect the current attach cycle only.
 *
 * Single-line space-separated format keeps userspace parsers
 * trivial (awk '{print $1, $2, $3, $4}'); no need to walk a
 * multi-line table.  A separate decisions counter (the existing
 * stats[] / decisions_total node) lets userspace compute the per-
 * eval cost distribution.
 */
static ssize_t decision_latency_hist_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	struct zenith_policy *z_policy;
	unsigned long b0 = 0, b1 = 0, b2 = 0, b3 = 0;

	/* Same single-leader-cpu policy iteration pattern used by
	 * the existing stats sysfs handlers (e.g. decisions_total).
	 * The list is stable while the gov_attr_set is alive (held
	 * by the sysfs read), so a plain list_for_each_entry without
	 * an extra lock is correct.
	 */
	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		b0 += z_policy->dec_lat_buckets[0];
		b1 += z_policy->dec_lat_buckets[1];
		b2 += z_policy->dec_lat_buckets[2];
		b3 += z_policy->dec_lat_buckets[3];
	}
	return sysfs_emit(buf, "%lu %lu %lu %lu\n", b0, b1, b2, b3);
}

static struct governor_attr decision_latency_hist =
	__ATTR_RO(decision_latency_hist);

/* Patch 1.9 fg-transition pulse sysfs knobs (RW, profile-baked).
 *
 * fg_transition_pulse_ms:
 *   Pulse duration in milliseconds.  0 disables the producer
 *   (the sched_wakeup_new probe never stamps a deadline) and is
 *   the BATTERY / LEGACY default.  Bounded to 0..200 to keep an
 *   accidentally-large value from holding the floor for an
 *   unreasonable stretch.
 *
 * fg_transition_pulse_pct:
 *   Floor depth in percent of policy->max.  0..100; 0 disables
 *   the consumer (deadline still gets stamped, but no floor is
 *   applied).
 */
static ssize_t fg_transition_pulse_ms_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->fg_transition_pulse_ms);
}

static ssize_t fg_transition_pulse_ms_store(struct gov_attr_set *attr_set,
					    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FG_TRANSITION_PULSE_MS_MAX)
		return -EINVAL;
	t->fg_transition_pulse_ms = val;
	return count;
}

static struct governor_attr fg_transition_pulse_ms =
	__ATTR_RW(fg_transition_pulse_ms);

static ssize_t fg_transition_pulse_pct_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->fg_transition_pulse_pct);
}

static ssize_t fg_transition_pulse_pct_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FG_TRANSITION_PULSE_PCT_MAX)
		return -EINVAL;
	t->fg_transition_pulse_pct = val;
	return count;
}

static struct governor_attr fg_transition_pulse_pct =
	__ATTR_RW(fg_transition_pulse_pct);

/* peak_headroom_prearm sysfs knob.  Boolean gate for the soft early
 * intervention tier (2b') that lifts the cluster to eff_hispeed_freq
 * while the starvation streak is accumulating but has not yet
 * crossed peak_headroom_starve_streak.  Accepts 0 or 1 only.
 */
ZENITH_TUNABLE_UINT_MAX(peak_headroom_prearm, 1);

/* predict_up_thresh sysfs knob.  Trend threshold for the
 * predictive up-shift tier (2a') in 256ths of max_cap; see the
 * block comment above ZENITH_DEFAULT_PREDICT_UP_THRESH for what
 * the unit means and how the trigger is gated.  0 disables the
 * tier, max ZENITH_PREDICT_UP_THRESH_MAX (255).
 */
static ssize_t predict_up_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->predict_up_thresh);
}

static ssize_t predict_up_thresh_store(struct gov_attr_set *attr_set,
				       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_PREDICT_UP_THRESH_MAX)
		return -EINVAL;
	WRITE_ONCE(t->predict_up_thresh, val);
	return count;
}

static struct governor_attr predict_up_thresh =
	__ATTR_RW(predict_up_thresh);

/* predict_up_window sysfs knob.  Window size for the predict_up
 * trend ring, in samples.  Range
 * [ZENITH_PREDICT_UP_WINDOW_MIN .. ZENITH_PREDICT_UP_WINDOW_MAX]
 * (2..8).  Out-of-range values are rejected.  Larger windows
 * smooth PELT noise out of the trend signal at the cost of one
 * more tick of warm-up after a cold attach.
 */
static ssize_t predict_up_window_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->predict_up_window);
}

static ssize_t predict_up_window_store(struct gov_attr_set *attr_set,
				       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_PREDICT_UP_WINDOW_MIN ||
	    val > ZENITH_PREDICT_UP_WINDOW_MAX)
		return -EINVAL;
	WRITE_ONCE(t->predict_up_window, val);
	return count;
}

static struct governor_attr predict_up_window =
	__ATTR_RW(predict_up_window);

/* pelt_rising_edge_thresh sysfs knob (Patch C3).  See block
 * comment above ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH for the
 * full semantics.  0 disables the tier; max
 * ZENITH_PELT_RISING_EDGE_THRESH_MAX (255).
 */
static ssize_t
pelt_rising_edge_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->pelt_rising_edge_thresh);
}

static ssize_t
pelt_rising_edge_thresh_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PELT_RISING_EDGE_THRESH_MAX)
		return -EINVAL;
	WRITE_ONCE(t->pelt_rising_edge_thresh, val);
	return count;
}

static struct governor_attr pelt_rising_edge_thresh =
	__ATTR_RW(pelt_rising_edge_thresh);

/* pelt_rising_edge_min_pct sysfs knob (Patch C3).  Absolute-level
 * gate for the rising-edge tier; the tier only fires when the
 * newest util sample is at least this percent of max_cap.  Range
 * 0..ZENITH_PELT_RISING_EDGE_MIN_PCT_MAX (100).
 */
static ssize_t
pelt_rising_edge_min_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->pelt_rising_edge_min_pct);
}

static ssize_t
pelt_rising_edge_min_pct_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PELT_RISING_EDGE_MIN_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->pelt_rising_edge_min_pct, val);
	return count;
}

static struct governor_attr pelt_rising_edge_min_pct =
	__ATTR_RW(pelt_rising_edge_min_pct);

/* dl_task_floor_pct sysfs knob (Patch C6).  Range 0..100.  When
 * any CPU in the policy has a SCHED_DEADLINE task, lift freq to
 * (policy->max * dl_task_floor_pct / 100).  0 disables the floor;
 * see ZENITH_DEFAULT_DL_TASK_FLOOR_PCT for the full block comment.
 */
static ssize_t
dl_task_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->dl_task_floor_pct);
}

static ssize_t
dl_task_floor_pct_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_DL_TASK_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->dl_task_floor_pct, val);
	return count;
}

static struct governor_attr dl_task_floor_pct =
	__ATTR_RW(dl_task_floor_pct);

/* io_floor_hyst_ms sysfs knob (Patch C9).  Range 0..2000.  When
 * non-zero, every iowait_boost arming stamps a deadline that
 * keeps the io_floor tier active for io_floor_hyst_ms past the
 * last arming sample.  See ZENITH_DEFAULT_IO_FLOOR_HYST_MS for
 * the full block comment.
 */
static ssize_t
io_floor_hyst_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->io_floor_hyst_ms);
}

static ssize_t
io_floor_hyst_ms_store(struct gov_attr_set *attr_set,
		       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_IO_FLOOR_HYST_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->io_floor_hyst_ms, val);
	return count;
}

static struct governor_attr io_floor_hyst_ms =
	__ATTR_RW(io_floor_hyst_ms);

/* io_floor_hyst_pct sysfs knob (Patch C9).  Range 0..100.  Floor
 * value as a percentage of policy->max while the io_floor_until_ns
 * deadline has not expired.  0 disables the floor effect (a
 * non-zero deadline still gets stamped but no lift happens).
 */
static ssize_t
io_floor_hyst_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->io_floor_hyst_pct);
}

static ssize_t
io_floor_hyst_pct_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_IO_FLOOR_HYST_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->io_floor_hyst_pct, val);
	return count;
}

static struct governor_attr io_floor_hyst_pct =
	__ATTR_RW(io_floor_hyst_pct);

/* peak_hysteresis_streak sysfs knob (Patch E).
 * Range 0..ZENITH_PEAK_HYSTERESIS_STREAK_MAX.  Number of
 * consecutive samples after a peak-class previous freq for
 * which the soft floor is held; 0 disables the hysteresis tier.
 */
static ssize_t
peak_hysteresis_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_hysteresis_streak);
}

static ssize_t
peak_hysteresis_streak_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEAK_HYSTERESIS_STREAK_MAX)
		return -EINVAL;
	WRITE_ONCE(t->peak_hysteresis_streak, val);
	return count;
}

static struct governor_attr peak_hysteresis_streak =
	__ATTR_RW(peak_hysteresis_streak);

/* peak_step_down_pct sysfs knob (Patch E).
 * Range 0..100.  Soft-floor freq for the hysteresis tier as a
 * percentage of the peak-anchor freq; 0 disables the tier.  100
 * makes the floor identical to the anchor (the cluster won't
 * descend at all during the streak).  95 (default) gives a 5%
 * stair-step descent.
 */
static ssize_t
peak_step_down_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_step_down_pct);
}

static ssize_t
peak_step_down_pct_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	WRITE_ONCE(t->peak_step_down_pct, val);
	return count;
}

static struct governor_attr peak_step_down_pct =
	__ATTR_RW(peak_step_down_pct);

/* boost_idle_thresh sysfs knob (Patch F).
 * Range 0..100.  load_pct below which a boost-active tick is
 * counted toward the persistent-idle streak.  0 disables the
 * boost early-exit (boost is always honoured to its deadline).
 */
static ssize_t
boost_idle_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boost_idle_thresh);
}

static ssize_t
boost_idle_thresh_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	WRITE_ONCE(t->boost_idle_thresh, val);
	return count;
}

static struct governor_attr boost_idle_thresh =
	__ATTR_RW(boost_idle_thresh);

/* boost_idle_streak sysfs knob (Patch F).
 * Range 0..ZENITH_BOOST_IDLE_STREAK_MAX.  Number of consecutive
 * idle ticks (load_pct < boost_idle_thresh) inside an active
 * boost window before the policy preempts the boost on its
 * tier-0 path.  0 disables the boost early-exit.
 */
static ssize_t
boost_idle_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boost_idle_streak);
}

static ssize_t
boost_idle_streak_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_BOOST_IDLE_STREAK_MAX)
		return -EINVAL;
	WRITE_ONCE(t->boost_idle_streak, val);
	return count;
}

static struct governor_attr boost_idle_streak =
	__ATTR_RW(boost_idle_streak);

/* bg_util_scale_pct sysfs knob (Patch G).
 * Range ZENITH_BG_UTIL_SCALE_PCT_MIN..100.  Scales the util
 * signal returned by zenith_get_util() down to this percent
 * when the display is off.  100 is a pass-through (default).
 * 0 is rejected -- this is a scale knob, not a gate.
 */
static ssize_t
bg_util_scale_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->bg_util_scale_pct);
}

static ssize_t
bg_util_scale_pct_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_BG_UTIL_SCALE_PCT_MIN || val > 100)
		return -EINVAL;
	WRITE_ONCE(t->bg_util_scale_pct, val);
	return count;
}

static struct governor_attr bg_util_scale_pct =
	__ATTR_RW(bg_util_scale_pct);

/* sleeper_tail_thresh_us sysfs knob (Patch H).
 * Range 0..ZENITH_SLEEPER_TAIL_THRESH_US_MAX.  0 disables.
 */
static ssize_t
sleeper_tail_thresh_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->sleeper_tail_thresh_us);
}

static ssize_t
sleeper_tail_thresh_us_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_SLEEPER_TAIL_THRESH_US_MAX)
		return -EINVAL;
	WRITE_ONCE(t->sleeper_tail_thresh_us, val);
	return count;
}

static struct governor_attr sleeper_tail_thresh_us =
	__ATTR_RW(sleeper_tail_thresh_us);

/* sleeper_tail_pct sysfs knob (Patch H).
 * Range ZENITH_SLEEPER_TAIL_PCT_MIN..ZENITH_SLEEPER_TAIL_PCT_MAX.
 */
static ssize_t
sleeper_tail_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->sleeper_tail_pct);
}

static ssize_t
sleeper_tail_pct_store(struct gov_attr_set *attr_set,
		       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_SLEEPER_TAIL_PCT_MIN ||
	    val > ZENITH_SLEEPER_TAIL_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->sleeper_tail_pct, val);
	return count;
}

static struct governor_attr sleeper_tail_pct =
	__ATTR_RW(sleeper_tail_pct);

/* peer_ramp_window_ms sysfs knob (Patch D).
 * Range 0..ZENITH_PEER_RAMP_WINDOW_MS_MAX (100).  0 disables the
 * peer-ramp coordination on both sides (no arming writes from
 * the peak tiers, no floor reads).  Non-zero values are the
 * post-peak window during which the peer cluster applies the
 * soft floor after this cluster ramps.
 */
static ssize_t
peer_ramp_window_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peer_ramp_window_ms);
}

static ssize_t
peer_ramp_window_ms_store(struct gov_attr_set *attr_set,
			  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEER_RAMP_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->peer_ramp_window_ms, val);
	return count;
}

static struct governor_attr peer_ramp_window_ms =
	__ATTR_RW(peer_ramp_window_ms);

/* peer_ramp_window_off_ms sysfs knob (Patch M3).
 * Range 0..ZENITH_PEER_RAMP_WINDOW_MS_MAX (100).  Screen-state-
 * aware shadow of peer_ramp_window_ms: when tunables->screen_state
 * is 0 the peer-ramp arming and floor-eval paths use this value
 * instead.  0 (default) suppresses peer_ramp entirely while the
 * screen is off.  Set equal to peer_ramp_window_ms to restore the
 * pre-Stage-5 always-on behaviour byte-identically.
 */
static ssize_t
peer_ramp_window_off_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peer_ramp_window_off_ms);
}

static ssize_t
peer_ramp_window_off_ms_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEER_RAMP_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->peer_ramp_window_off_ms, val);
	return count;
}

static struct governor_attr peer_ramp_window_off_ms =
	__ATTR_RW(peer_ramp_window_off_ms);

/* peer_ramp_floor_pct sysfs knob (Patch D).
 * Range 0..ZENITH_PEER_RAMP_FLOOR_PCT_MAX (100).  Soft floor as
 * a percent of policy->max applied while a peer-ramp deadline
 * is active.  0 leaves the deadlines being stamped (visible to
 * trace consumers) but suppresses the floor itself.
 */
static ssize_t
peer_ramp_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peer_ramp_floor_pct);
}

static ssize_t
peer_ramp_floor_pct_store(struct gov_attr_set *attr_set,
			  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEER_RAMP_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->peer_ramp_floor_pct, val);
	return count;
}

static struct governor_attr peer_ramp_floor_pct =
	__ATTR_RW(peer_ramp_floor_pct);

/* migration_jump_pct (Patch K1).  Range 0..100.  0 disables both
 * the per-CPU stamping and the floor read.
 */
static ssize_t
migration_jump_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->migration_jump_pct);
}

static ssize_t
migration_jump_pct_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_MIGRATION_JUMP_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->migration_jump_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_MIGRATION_JUMP);
	return count;
}

static struct governor_attr migration_jump_pct =
	__ATTR_RW(migration_jump_pct);

/* migration_floor_window_ms (Patch K1).  Range 0..100.  Length of
 * the post-arrival window in milliseconds.  0 disables the
 * stamping (the detector still updates migration_prev_util but
 * does not arm a deadline).
 */
static ssize_t
migration_floor_window_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       migration_floor_window_ms);
}

static ssize_t
migration_floor_window_ms_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_MIGRATION_FLOOR_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->migration_floor_window_ms, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_WIN);
	return count;
}

static struct governor_attr migration_floor_window_ms =
	__ATTR_RW(migration_floor_window_ms);

/* migration_floor_pct (Patch K1).  Range 0..100.  Soft floor as
 * a percent of policy->max while a migration deadline is active.
 * 0 leaves the deadline arming visible to stats / trace but
 * suppresses the freq adjustment.
 */
static ssize_t
migration_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->migration_floor_pct);
}

static ssize_t
migration_floor_pct_store(struct gov_attr_set *attr_set,
			  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_MIGRATION_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->migration_floor_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_PCT);
	return count;
}

static struct governor_attr migration_floor_pct =
	__ATTR_RW(migration_floor_pct);

/* psi_cpu_floor_thresh (Patch K2).  Range 0..100.  0 disables.
 * When >= this percent of system-wide PSI_CPU_SOME 10s EWMA is
 * observed, the eval lifts to zenith_eff_hispeed_freq().  Mirror
 * of psi_cpu_thresh on the floor side.  See the
 * ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH comment block for the full
 * rationale.
 */
static ssize_t
psi_cpu_floor_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_cpu_floor_thresh);
}

static ssize_t
psi_cpu_floor_thresh_store(struct gov_attr_set *attr_set,
			   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PSI_CPU_FLOOR_THRESH_MAX)
		return -EINVAL;
	WRITE_ONCE(t->psi_cpu_floor_thresh, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_PSI_CPU_FLOOR);
	return count;
}

static struct governor_attr psi_cpu_floor_thresh =
	__ATTR_RW(psi_cpu_floor_thresh);

/* frame_overrun_slack_us (Patch K3).  Range 0..16667 us.
 * Tolerance beyond the cached vblank period before treating a
 * gap as an overrun.  0 disables stamping (the producer still
 * updates last_vblank_ns).  Mirrored into the governor-wide
 * cache so the producer can read it without a tunables lookup.
 */
static ssize_t
frame_overrun_slack_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->frame_overrun_slack_us);
}

static ssize_t
frame_overrun_slack_us_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_SLACK_US_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_slack_us, val);
	WRITE_ONCE(zenith_frame_overrun_slack_us_cache, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_FRAME_OVR_SLACK);
	return count;
}

static struct governor_attr frame_overrun_slack_us =
	__ATTR_RW(frame_overrun_slack_us);

/* frame_overrun_window_ms (Patch K3).  Range 0..200 ms.  How
 * long the soft floor holds after an overrun is detected.  0
 * suppresses stamping but still updates last_vblank_ns.  Also
 * mirrored to the governor-wide cache.
 */
static ssize_t
frame_overrun_window_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       frame_overrun_window_ms);
}

static ssize_t
frame_overrun_window_ms_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_window_ms, val);
	WRITE_ONCE(zenith_frame_overrun_window_ms_cache, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_FRAME_OVR_WINDOW);
	return count;
}

static struct governor_attr frame_overrun_window_ms =
	__ATTR_RW(frame_overrun_window_ms);

/* frame_overrun_floor_pct (Patch K3).  Range 0..100.  Soft
 * floor as a percent of policy->max while the overrun deadline
 * is active.  0 leaves stamping in place but suppresses the
 * floor.
 */
static ssize_t
frame_overrun_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       frame_overrun_floor_pct);
}

static ssize_t
frame_overrun_floor_pct_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_floor_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_FRAME_OVR_FLOOR);
	return count;
}

static struct governor_attr frame_overrun_floor_pct =
	__ATTR_RW(frame_overrun_floor_pct);

/* frame_overrun_deep_streak sysfs knob (Patch M5).  Range
 * 0..ZENITH_FRAME_OVERRUN_DEEP_STREAK_MAX (16).  When non-zero,
 * after this many consecutive overruns the K3 floor escalates
 * from frame_overrun_floor_pct to frame_overrun_deep_floor_pct.
 * 0 disables the deep tier (the consumer never reads the streak
 * atomic).  See the macro block above struct zenith_tunables
 * for the full design rationale.
 */
static ssize_t
frame_overrun_deep_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       frame_overrun_deep_streak);
}

static ssize_t
frame_overrun_deep_streak_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_DEEP_STREAK_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_deep_streak, val);
	return count;
}

static struct governor_attr frame_overrun_deep_streak =
	__ATTR_RW(frame_overrun_deep_streak);

/* frame_overrun_deep_floor_pct sysfs knob (Patch M5).  Range
 * 0..ZENITH_FRAME_OVERRUN_DEEP_FLOOR_PCT_MAX (100).  Floor as a
 * percent of policy->max applied while the K3 deadline is active
 * AND the consecutive-overrun streak has crossed
 * frame_overrun_deep_streak.  Effective floor is
 * max(deep_floor_pct, frame_overrun_floor_pct) so the deep tier
 * never produces a lower floor than the standard K3 floor.
 */
static ssize_t
frame_overrun_deep_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       frame_overrun_deep_floor_pct);
}

static ssize_t
frame_overrun_deep_floor_pct_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_DEEP_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_deep_floor_pct, val);
	return count;
}

static struct governor_attr frame_overrun_deep_floor_pct =
	__ATTR_RW(frame_overrun_deep_floor_pct);

/* psi_mem_cap_thresh / psi_mem_cap_pct / psi_mem_cap_window_ms
 * sysfs knobs (Patch M1).  thresh ranges 0..100 (0 = off);
 * pct ranges 50..100; window_ms ranges 100..5000.  All three
 * mark their override bit on store so the V2 worker stops
 * touching the value until the next profile flip clears the
 * mask.  See the macro block above for the full design.
 */
static ssize_t
psi_mem_cap_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_mem_cap_thresh);
}

static ssize_t
psi_mem_cap_thresh_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PSI_MEM_CAP_THRESH_MAX)
		return -EINVAL;
	WRITE_ONCE(t->psi_mem_cap_thresh, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_PSI_MEM_CAP_THRESH);
	return count;
}

static struct governor_attr psi_mem_cap_thresh =
	__ATTR_RW(psi_mem_cap_thresh);

static ssize_t
psi_mem_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_mem_cap_pct);
}

static ssize_t
psi_mem_cap_pct_store(struct gov_attr_set *attr_set,
		      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_PSI_MEM_CAP_PCT_MIN ||
	    val > ZENITH_PSI_MEM_CAP_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->psi_mem_cap_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_PSI_MEM_CAP_PCT);
	return count;
}

static struct governor_attr psi_mem_cap_pct =
	__ATTR_RW(psi_mem_cap_pct);

static ssize_t
psi_mem_cap_window_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_mem_cap_window_ms);
}

static ssize_t
psi_mem_cap_window_ms_store(struct gov_attr_set *attr_set,
			    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_PSI_MEM_CAP_WINDOW_MS_MIN ||
	    val > ZENITH_PSI_MEM_CAP_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->psi_mem_cap_window_ms, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_PSI_MEM_CAP_WINDOW);
	return count;
}

static struct governor_attr psi_mem_cap_window_ms =
	__ATTR_RW(psi_mem_cap_window_ms);

/* brutal_decay_ms sysfs knob.  Range 0..ZENITH_BRUTAL_DECAY_MS_MAX.
 * 0 disables the tail-glide and restores the legacy hard cliff
 * exit; non-zero arms a linear ramp from policy->max down to the
 * EAS-computed freq across the configured window on every
 * brutal-hold cliff exit.  See struct zenith_tunables for details.
 */
ZENITH_TUNABLE_UINT_MAX(brutal_decay_ms, ZENITH_BRUTAL_DECAY_MS_MAX);

ZENITH_TUNABLE_UINT_MAX_INVAL(climb_mode, ZENITH_CLIMB_MODE_STEP);

static ssize_t freq_step_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->freq_step_pct);
}

static ssize_t freq_step_pct_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->freq_step_pct = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr freq_step_pct = __ATTR_RW(freq_step_pct);

/* freq_step_adaptive sysfs knob.  0/1 only.  See
 * ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE for semantics.
 */
ZENITH_TUNABLE_UINT_MAX(freq_step_adaptive, 1);

ZENITH_TUNABLE_UINT_MAX_INVAL(powersave_bias, 1000);

/* screen_on_bias_pct sysfs knob.  See struct zenith_tunables doc and
 * ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT for full semantics.  Range
 * 0..100 (percent).  100 disables the softening (legacy behaviour);
 * 50 (the default) halves the configured powersave_bias whenever
 * the screen is on; 0 zeroes the bias on screen-on.
 */
ZENITH_TUNABLE_UINT_MAX_INVAL(screen_on_bias_pct, 100);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->up_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_RATE_LIMIT_US_MAX)
		return -EINVAL;
	t->up_rate_limit_us = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_UP_RATE);

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_update_rate_delay_ns(z_pol);
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->down_rate_limit_us);
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_RATE_LIMIT_US_MAX)
		return -EINVAL;
	t->down_rate_limit_us = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_DOWN_RATE);

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_update_rate_delay_ns(z_pol);
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

/* kcpustat tunables. _store paths invalidate the freq cache so a
 * userspace write takes effect on the very next scheduler tick rather
 * than waiting for the prev_freq cache shortcut to drift. window_us is
 * clamped to a sane range; filter_shift is capped to keep the >>shift
 * idiom well-defined; hispeed_enable is a strict boolean.
 */
static ssize_t kcpustat_window_us_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->kcpustat_window_us);
}

static ssize_t kcpustat_window_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_KCPUSTAT_WINDOW_MIN_US ||
	    val > ZENITH_KCPUSTAT_WINDOW_MAX_US)
		return -EINVAL;
	t->kcpustat_window_us = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr kcpustat_window_us =
	__ATTR_RW(kcpustat_window_us);

static ssize_t kcpustat_filter_shift_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->kcpustat_filter_shift);
}

static ssize_t kcpustat_filter_shift_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_KCPUSTAT_FILTER_SHIFT_MAX)
		return -EINVAL;
	t->kcpustat_filter_shift = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr kcpustat_filter_shift =
	__ATTR_RW(kcpustat_filter_shift);

ZENITH_TUNABLE_UINT_BOOL_INVAL(kcpustat_hispeed_enable);

/* Strict-bool tunable selecting v1 (legacy cpu_util_cfs()) vs v2
 * (6.x-style runnable-aware util) input to schedutil_cpu_util in
 * zenith_get_util().  Invalidates the prev_freq cache so toggles
 * take effect on the next tick.
 */
ZENITH_TUNABLE_UINT_BOOL_INVAL(util_math_v2);

/* predict_util_pct sysfs knob.  See ZENITH_DEFAULT_PREDICT_UTIL_PCT
 * comment block at the top of the file for semantics.  Range
 * 0..ZENITH_PREDICT_UTIL_PCT_MAX; values above the cap are rejected
 * outright rather than silently clamped, so userspace gets a clear
 * EINVAL on out-of-range writes.  The freq cache is invalidated so
 * a toggle takes effect on the very next zenith_update tick.
 */
static ssize_t predict_util_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->predict_util_pct);
}

static ssize_t predict_util_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_PREDICT_UTIL_PCT_MAX)
		return -EINVAL;
	t->predict_util_pct = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr predict_util_pct = __ATTR_RW(predict_util_pct);

/* predict_util_smooth sysfs knob.  0/1 only.  See
 * ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH for semantics.
 */
ZENITH_TUNABLE_UINT_MAX(predict_util_smooth, 1);

/* render_aware sysfs knob.  Strict 0/1 boolean; non-zero values are
 * normalised to 1 on store so userspace can echo any truthy integer.
 * No cache invalidation is required: tunables->render_aware is read
 * fresh on every zenith_get_next_freq() call, so the next scheduler
 * tick already sees the new value.
 */
static ssize_t render_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->render_aware);
}

static ssize_t render_aware_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->render_aware;
	t->render_aware = val;
	zenith_set_static_key(&zenith_render_aware_key, val);
	if (old != val)
		zenith_log_master_flip(t, "render_aware", old, val);
	return count;
}
static struct governor_attr render_aware = __ATTR_RW(render_aware);

/* render_floor_pct sysfs knob.  Range 0..100; 0 leaves the comm
 * walk running but applies no floor.  Out-of-range values rejected
 * with EINVAL so userspace gets a clear error.
 */
static ssize_t render_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->render_floor_pct);
}

static ssize_t render_floor_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->render_floor_pct = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_RENDER_FLOOR_PCT);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr render_floor_pct = __ATTR_RW(render_floor_pct);

/* render_floor_min_runtime_ms sysfs knob.  Debounce window in
 * milliseconds for the render-thread floor; 0 disables the
 * debounce so the floor fires the moment a render thread is
 * picked up.  See ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS for
 * details.  Capped at ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX.
 */
static ssize_t
render_floor_min_runtime_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->render_floor_min_runtime_ms);
}

static ssize_t
render_floor_min_runtime_ms_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->render_floor_min_runtime_ms, val);
	return count;
}

static struct governor_attr render_floor_min_runtime_ms =
	__ATTR_RW(render_floor_min_runtime_ms);

/* audio_aware sysfs knob.  Strict 0/1 boolean; non-zero values are
 * normalised to 1 on store.  No cache invalidation: tunables->audio_aware
 * is read fresh on every zenith_get_next_freq() call.  Toggling from 1
 * to 0 leaves the per-policy audio_active cache stale, but its TTL
 * (ZENITH_AUDIO_CACHE_TTL_NS) is short and the cap/floor are gated on
 * tunables->audio_aware first so the stale state is unreachable.
 */
static ssize_t audio_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_aware);
}

static ssize_t audio_aware_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->audio_aware;
	t->audio_aware = val;
	zenith_set_static_key(&zenith_audio_aware_key, val);
	if (old != val)
		zenith_log_master_flip(t, "audio_aware", old, val);
	return count;
}
static struct governor_attr audio_aware = __ATTR_RW(audio_aware);

/* audio_floor_pct sysfs knob.  Range 0..100; 0 leaves the comm walk
 * running (when audio_aware=1) but applies no floor.  Out-of-range
 * values rejected with EINVAL.  audio_floor_pct does NOT have to be
 * <= audio_cap_pct: if the user inverts them, the cap still wins
 * because it runs after the floor in zenith_get_next_freq().
 */
static ssize_t audio_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_floor_pct);
}

static ssize_t audio_floor_pct_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->audio_floor_pct = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_AUDIO_FLOOR_PCT);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr audio_floor_pct = __ATTR_RW(audio_floor_pct);

/* audio_cap_pct sysfs knob.  Range 0..100; 0 disables the cap (the
 * floor side of the band can still apply alone).  Applied before the
 * uclamp_max final cap so ADPF power-efficiency hints still win.
 */
static ssize_t audio_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_cap_pct);
}

static ssize_t audio_cap_pct_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->audio_cap_pct = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_AUDIO_CAP_PCT);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr audio_cap_pct = __ATTR_RW(audio_cap_pct);

/* Patch B7-1: audio_hyst_ms sysfs knob.  Range 0..2000 (capped via
 * ZENITH_AUDIO_HYST_MS_MAX); 0 disables the sticky window so the
 * helper falls back to the cache-TTL-only behaviour.  Profile-baked
 * to a sane value per profile (PERFORMANCE/BALANCED/GAMING 250 ms,
 * BATTERY 100 ms, AUDIO 750 ms, LEGACY 0).
 */
static ssize_t audio_hyst_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_hyst_ms);
}

static ssize_t audio_hyst_ms_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_AUDIO_HYST_MS_MAX)
		return -EINVAL;
	t->audio_hyst_ms = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_AUDIO_HYST_MS);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr audio_hyst_ms = __ATTR_RW(audio_hyst_ms);

/* Patch B9-1: vh_arch_freq_scale_enable sysfs knob.  Strict 0/1
 * boolean; gates the android_vh_arch_set_freq_scale vendor-hook
 * observer (see ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE for the
 * full semantics and profile bakes).  Stored via plain assignment;
 * the probe reads it with READ_ONCE so a torn write would only
 * delay the gate flip by one realisation event.
 */
static ssize_t
vh_arch_freq_scale_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_arch_freq_scale_enable);
}

static ssize_t
vh_arch_freq_scale_enable_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_arch_freq_scale_enable, val);
	return count;
}
static struct governor_attr vh_arch_freq_scale_enable =
	__ATTR_RW(vh_arch_freq_scale_enable);

/* Patch B9-2: vh_uclamp_observer_enable sysfs knob.  Strict 0/1
 * boolean; gates the android_vh_setscheduler_uclamp vendor-hook
 * observer (see ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE for full
 * semantics and profile bakes).  Stored via WRITE_ONCE; the probe
 * reads it with READ_ONCE so a torn write would only delay the
 * gate flip by one ADPF write.
 */
static ssize_t
vh_uclamp_observer_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_uclamp_observer_enable);
}

static ssize_t
vh_uclamp_observer_enable_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_uclamp_observer_enable, val);
	return count;
}
static struct governor_attr vh_uclamp_observer_enable =
	__ATTR_RW(vh_uclamp_observer_enable);

/* Patch B9-3: vh_cpu_idle_enable sysfs knob.  Strict 0/1 boolean;
 * gates the android_vh_cpu_idle_enter / android_vh_cpu_idle_exit
 * vendor-hook observer pair (see ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE
 * for full semantics and profile bakes).  Stored via WRITE_ONCE;
 * both probes and the cwp arm-site reader use READ_ONCE so a torn
 * write would at worst delay the gate flip by one cpuidle exit /
 * one eval window.
 */
static ssize_t
vh_cpu_idle_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_cpu_idle_enable);
}

static ssize_t
vh_cpu_idle_enable_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_cpu_idle_enable, val);
	return count;
}

static struct governor_attr vh_cpu_idle_enable =
	__ATTR_RW(vh_cpu_idle_enable);

/* Patch B9-3+: vh_freq_qos_enable sysfs knob.  Strict 0/1 boolean;
 * gates the android_vh_freq_qos_update_request vendor-hook observer
 * (see ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE for full semantics and
 * profile bakes).  Stored via WRITE_ONCE; the probe and the auto-
 * classify consumer both read with READ_ONCE so a torn write would
 * at worst delay the gate flip by one QoS update / one auto-eval
 * window.
 */
static ssize_t
vh_freq_qos_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_freq_qos_enable);
}

static ssize_t
vh_freq_qos_enable_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_freq_qos_enable, val);
	return count;
}

static struct governor_attr vh_freq_qos_enable =
	__ATTR_RW(vh_freq_qos_enable);

/* Patch B9-5: vh_sched_move_task_enable sysfs knob.  Strict 0/1
 * boolean; gates the android_vh_sched_move_task vendor-hook observer
 * (see ZENITH_DEFAULT_VH_SCHED_MOVE_TASK_ENABLE for full semantics
 * and profile bakes).  Stored via WRITE_ONCE; the probe reads with
 * READ_ONCE so a torn write would at worst delay the gate flip by
 * one cgroup move.
 */
static ssize_t
vh_sched_move_task_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_sched_move_task_enable);
}

static ssize_t
vh_sched_move_task_enable_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_sched_move_task_enable, val);
	return count;
}

static struct governor_attr vh_sched_move_task_enable =
	__ATTR_RW(vh_sched_move_task_enable);

/* Patch B9-4: vh_scheduler_tick_enable sysfs knob.  Strict 0/1
 * boolean; gates the android_vh_scheduler_tick vendor-hook observer
 * (see ZENITH_DEFAULT_VH_SCHEDULER_TICK_ENABLE for full semantics
 * and profile bakes).  Stored via WRITE_ONCE; the probe reads with
 * READ_ONCE so a torn write would at worst delay the gate flip by
 * one tick (4 ms at HZ=250).
 */
static ssize_t
vh_scheduler_tick_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_scheduler_tick_enable);
}

static ssize_t
vh_scheduler_tick_enable_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_scheduler_tick_enable, val);
	return count;
}

static struct governor_attr vh_scheduler_tick_enable =
	__ATTR_RW(vh_scheduler_tick_enable);

/* Patch B-AUTO-3: auto_eval_ms RW sysfs.  Cadence at which the
 * auto-selector engine runs its classifier when active_profile ==
 * ZENITH_PROFILE_AUTO.  Bounded by ZENITH_AUTO_EVAL_MS_{MIN,MAX}.
 * 0 is the runtime pause (worker still rearms but does not
 * classify or commit).  Reads / writes are READ_ONCE / WRITE_ONCE
 * because the worker reads this lock-free.
 */
static ssize_t auto_eval_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       READ_ONCE(to_zenith_tunables(attr_set)->auto_eval_ms));
}

static ssize_t auto_eval_ms_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val && val < ZENITH_AUTO_EVAL_MS_MIN)
		return -EINVAL;
	if (val > ZENITH_AUTO_EVAL_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->auto_eval_ms, val);
	return count;
}
static struct governor_attr auto_eval_ms = __ATTR_RW(auto_eval_ms);

/* Patch B-AUTO-3: auto_hysteresis_ms RW sysfs.  How long the
 * auto-selector classifier's chosen target must hold before
 * zenith commits the profile switch.  Bounded by
 * ZENITH_AUTO_HYSTERESIS_MS_{MIN,MAX}; 0 disables hysteresis (the
 * classifier's pick lands on the very next eval window).
 */
static ssize_t auto_hysteresis_ms_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       READ_ONCE(to_zenith_tunables(attr_set)->auto_hysteresis_ms));
}

static ssize_t auto_hysteresis_ms_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_AUTO_HYSTERESIS_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->auto_hysteresis_ms, val);
	return count;
}
static struct governor_attr auto_hysteresis_ms =
	__ATTR_RW(auto_hysteresis_ms);

/* camera_aware sysfs knob.  Strict 0/1 boolean; non-zero values
 * normalised to 1 on store.
 */
static ssize_t camera_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->camera_aware);
}

static ssize_t camera_aware_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->camera_aware;
	t->camera_aware = val;
	zenith_set_static_key(&zenith_camera_aware_key, val);
	if (old != val)
		zenith_log_master_flip(t, "camera_aware", old, val);
	return count;
}
static struct governor_attr camera_aware = __ATTR_RW(camera_aware);

/* render_comms / audio_comms / camera_comms sysfs knobs.  CSV of
 * comma-separated comm prefixes.  See the zenith_comm_table comment
 * block above zenith_render_comms[] for the RCU semantics and
 * defaults.  Empty write resets to the in-tree default list.
 */
static ssize_t render_comms_show(struct gov_attr_set *attr_set, char *buf)
{
	return zenith_show_comm_table(&zenith_render_table, buf);
}

static ssize_t render_comms_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	return zenith_store_comm_table(&zenith_render_table,
				       zenith_render_comms,
				       ARRAY_SIZE(zenith_render_comms),
				       buf, count);
}
static struct governor_attr render_comms = __ATTR_RW(render_comms);

static ssize_t audio_comms_show(struct gov_attr_set *attr_set, char *buf)
{
	return zenith_show_comm_table(&zenith_audio_table, buf);
}

static ssize_t audio_comms_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	return zenith_store_comm_table(&zenith_audio_table,
				       zenith_audio_comms,
				       ARRAY_SIZE(zenith_audio_comms),
				       buf, count);
}
static struct governor_attr audio_comms = __ATTR_RW(audio_comms);

static ssize_t camera_comms_show(struct gov_attr_set *attr_set, char *buf)
{
	return zenith_show_comm_table(&zenith_camera_table, buf);
}

static ssize_t camera_comms_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	return zenith_store_comm_table(&zenith_camera_table,
				       zenith_camera_comms,
				       ARRAY_SIZE(zenith_camera_comms),
				       buf, count);
}
static struct governor_attr camera_comms = __ATTR_RW(camera_comms);

static ssize_t game_auto_comms_show(struct gov_attr_set *attr_set, char *buf)
{
	return zenith_show_comm_table(&zenith_game_auto_table, buf);
}

static ssize_t game_auto_comms_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	return zenith_store_comm_table(&zenith_game_auto_table,
				       zenith_game_auto_comms,
				       ARRAY_SIZE(zenith_game_auto_comms),
				       buf, count);
}
static struct governor_attr game_auto_comms = __ATTR_RW(game_auto_comms);

/* camera_active sysfs knob.  Tri-state override:
 *   0  ZENITH_CAMERA_OVERRIDE_AUTO        consult comm table
 *   1  ZENITH_CAMERA_OVERRIDE_FORCE_ON    floor always applied
 *   2  ZENITH_CAMERA_OVERRIDE_FORCE_OFF   floor never applied
 * Out-of-range values (>=3) rejected with EINVAL.
 */
static ssize_t camera_active_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->camera_active);
}

static ssize_t camera_active_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_CAMERA_OVERRIDE_FORCE_OFF)
		return -EINVAL;
	t->camera_active = val;
	return count;
}
static struct governor_attr camera_active = __ATTR_RW(camera_active);

/* camera_floor_pct sysfs knob.  Range 0..100; 0 leaves the comm
 * walk running (when camera_aware=1) but applies no floor.
 */
static ssize_t camera_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->camera_floor_pct);
}

static ssize_t camera_floor_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->camera_floor_pct = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_CAMERA_FLOOR_PCT);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr camera_floor_pct = __ATTR_RW(camera_floor_pct);

/* game_mode sysfs knob.  Strict 0/1 boolean.  See
 * ZENITH_DEFAULT_GAME_MODE comment block for the per-tier overlays
 * that flip behaviour when this is set.  No cache invalidation
 * required: the value is consumed inline in the freq path.
 */
static ssize_t game_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->game_mode);
}

static ssize_t game_mode_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;
	unsigned int prev;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_GAME_MODE_MAX)
		return -EINVAL;
	prev = t->game_mode;
	t->game_mode = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_GAME_MODE);
	if (prev != t->game_mode) {
		if (trace_zenith_game_mode_enabled())
			trace_zenith_game_mode(smp_processor_id(), t->game_mode);
		atomic_notifier_call_chain(&zenith_game_mode_nh,
					 val ? 1 : 0, NULL);
	}
	return count;
}
static struct governor_attr game_mode = __ATTR_RW(game_mode);

/* game_auto sysfs knob.  Master gate for the in-kernel game
 * detector.  See the ZENITH_DEFAULT_GAME_AUTO comment block.  Strict
 * 0/1 boolean.  Syncs the zenith_game_auto_key static branch and,
 * on disable, clears the global zenith_game_auto_active_until_ns
 * latch so a stale "game active" state does not survive game_auto =
 * 0 (otherwise a write of 0 would leave the helper still returning
 * 1 until the existing TTL expired).
 */
static ssize_t game_auto_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->game_auto);
}

static ssize_t game_auto_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->game_auto;
	t->game_auto = val;
	zenith_set_static_key(&zenith_game_auto_key, val);
	if (!val) {
		u64 prev_latch = READ_ONCE(zenith_game_auto_active_until_ns);
		WRITE_ONCE(zenith_game_auto_active_until_ns, 0);
		if (prev_latch)
			atomic_notifier_call_chain(&zenith_game_mode_nh, 0, NULL);
	}
	if (old != val)
		zenith_log_master_flip(t, "game_auto", old, val);
	return count;
}
static struct governor_attr game_auto = __ATTR_RW(game_auto);

/* game_auto_state sysfs knob.  Read-only view of the global latch
 * state.  Returns 1 when the in-kernel detector currently considers
 * a game active (i.e. a fresh detection landed within
 * ZENITH_GAME_AUTO_ACTIVE_TTL_NS), 0 otherwise.  Useful for tooling
 * that wants to confirm the detector fired, distinct from a manual
 * game_mode write.
 */
static ssize_t game_auto_state_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", zenith_game_auto_active() ? 1 : 0);
}
static struct governor_attr game_auto_state = __ATTR_RO(game_auto_state);

/* psi_aware sysfs knob.  Strict 0/1 boolean.  See ZENITH_DEFAULT_PSI_AWARE
 * comment block for semantics.  No cache invalidation -- the value is
 * consumed inline at zenith_get_next_freq() time.
 */
static ssize_t psi_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->psi_aware);
}

static ssize_t psi_aware_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->psi_aware;
	t->psi_aware = val;
	zenith_set_static_key(&zenith_psi_aware_key, val);
	if (old != val)
		zenith_log_master_flip(t, "psi_aware", old, val);
	return count;
}
static struct governor_attr psi_aware = __ATTR_RW(psi_aware);

/* psi_mem_thresh sysfs knob.  Range 0..100 (integer percentage of
 * the PSI 10s some-stall average).  0 disables the cap even with
 * psi_aware=1, useful for tracing the helper without changing freq.
 */
static ssize_t psi_mem_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_mem_thresh);
}

static ssize_t psi_mem_thresh_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->psi_mem_thresh = val;
	return count;
}
static struct governor_attr psi_mem_thresh = __ATTR_RW(psi_mem_thresh);

/* psi_cpu_thresh / psi_io_thresh sysfs knobs.  Same shape as
 * psi_mem_thresh: 0..100 integer percent, 0 disables that dimension's
 * cap.  See ZENITH_DEFAULT_PSI_AWARE comment block for semantics.
 */
static ssize_t psi_cpu_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_cpu_thresh);
}

static ssize_t psi_cpu_thresh_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->psi_cpu_thresh = val;
	return count;
}
static struct governor_attr psi_cpu_thresh = __ATTR_RW(psi_cpu_thresh);

static ssize_t psi_io_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_io_thresh);
}

static ssize_t psi_io_thresh_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->psi_io_thresh = val;
	return count;
}
static struct governor_attr psi_io_thresh = __ATTR_RW(psi_io_thresh);

/* psi_cgroup_path sysfs knob (Patch B10-3).
 *
 * Read returns the per-tunables psi_cgroup_path string (the path
 * most recently stored on this attr_set, profile-baked or sysfs-
 * written).  Newline-terminated, like every other zenith string knob.
 *
 * Write copies the supplied string into t->psi_cgroup_path (after
 * stripping a trailing newline) and calls zenith_psi_cgroup_apply()
 * to refresh the file-scope cached cgroup pointer.  Empty string
 * (just "\n" or "") drops to NULL = system-wide PSI = pre-B10
 * behaviour.  -EINVAL on a too-long path; resolution failures
 * (cgroup-v2 not mounted, missing cgroup) are silently demoted to
 * the empty/system-wide path inside zenith_psi_cgroup_apply().
 */
static ssize_t psi_cgroup_path_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%s\n",
		       to_zenith_tunables(attr_set)->psi_cgroup_path);
}

static ssize_t psi_cgroup_path_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	char path[ZENITH_PSI_CGROUP_PATH_MAX];
	ssize_t len;

	len = strscpy(path, buf, sizeof(path));
	if (len < 0)
		return -EINVAL;

	/* Strip trailing newline, if any. */
	if (len > 0 && path[len - 1] == '\n')
		path[len - 1] = '\0';

	strscpy(t->psi_cgroup_path, path, sizeof(t->psi_cgroup_path));
	zenith_psi_cgroup_apply(path);

	return count;
}
static struct governor_attr psi_cgroup_path = __ATTR_RW(psi_cgroup_path);

/* boot_boost_ms sysfs knob.  See ZENITH_DEFAULT_BOOT_BOOST_MS comment
 * block for semantics.  Range 0..ZENITH_BOOT_BOOST_MAX_MS;
 * out-of-range values rejected with EINVAL so userspace gets a clear
 * error rather than a silent clamp.  No cache invalidation: the value
 * is consumed inline by the eval path on every tick.
 */
static ssize_t boot_boost_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boot_boost_ms);
}

static ssize_t boot_boost_ms_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_BOOT_BOOST_MAX_MS)
		return -EINVAL;
	t->boot_boost_ms = val;
	return count;
}
static struct governor_attr boot_boost_ms = __ATTR_RW(boot_boost_ms);

/* boot_boost_decay_ms sysfs knob.  Range
 * 0..ZENITH_BOOT_BOOST_DECAY_MS_MAX.  See struct zenith_tunables for
 * semantics.  Mirror of input_boost_decay_ms but for the boot-pin
 * window: 0 keeps the legacy hard cliff at boot_boost_ms, non-zero
 * applies a linear floor ramp from policy->max to policy->min after
 * the pin expires.
 */
static ssize_t boot_boost_decay_ms_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boot_boost_decay_ms);
}

static ssize_t boot_boost_decay_ms_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_BOOT_BOOST_DECAY_MS_MAX)
		return -EINVAL;
	t->boot_boost_decay_ms = val;
	return count;
}
static struct governor_attr boot_boost_decay_ms =
	__ATTR_RW(boot_boost_decay_ms);

/* boot_complete sysfs knob.  See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO
 * comment block.  Read returns the global latch state shared across
 * all policies (any policy's node is equivalent).  Write 1 to raise
 * the latch from userspace -- typical use is from
 * on property:sys.boot_completed=1 in init.zenith.rc.  Write 0 to
 * lower the latch (useful for testing the boot-boost path on a
 * running system without a reboot).  Values >1 are rejected.
 *
 * Raising the latch (write 1) sets zenith_boot_complete_ns to
 * ktime_get_boottime_ns() so the boost path snaps the deadline to
 * the latch timestamp.  Lowering (write 0) clears both the atomic
 * and the timestamp; the boost behaves as if the latch had never
 * been raised, except that the wall-clock has advanced -- a
 * post-deadline boot would simply re-enter the existing
 * boot_boost_decay_ms tail.
 */
static ssize_t boot_complete_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", atomic_read(&zenith_boot_complete));
}

static ssize_t boot_complete_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	if (val) {
		WRITE_ONCE(zenith_boot_complete_ns,
			   ktime_get_boottime_ns());
		atomic_set(&zenith_boot_complete, 1);
	} else {
		atomic_set(&zenith_boot_complete, 0);
		WRITE_ONCE(zenith_boot_complete_ns, 0);
	}
	return count;
}
static struct governor_attr boot_complete = __ATTR_RW(boot_complete);

/* boot_complete_auto sysfs knob.  See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO
 * comment block.  Per-tunables (per-attr-set), unlike the global
 * boot_complete latch above.  Default 1: the auto-tune worker may
 * raise the latch on the first policy that observes
 * ZENITH_BOOT_COMPLETE_CALM_WINDOWS consecutive EFFICIENCY windows
 * past the grace period.  Set to 0 to require an explicit userspace
 * write to boot_complete; the calm streak counter still increments
 * but never raises the latch.
 */
ZENITH_TUNABLE_UINT_MAX(boot_complete_auto, 1);

/* frame_budget_us sysfs knob.  Range 0..ZENITH_FRAME_BUDGET_US_MAX
 * (50 ms).  Userspace writes the current vblank period in
 * microseconds whenever the panel changes refresh rate.  0 disables
 * the adaptive frame-budget floor outright.  See the
 * ZENITH_DEFAULT_FRAME_BUDGET_US comment block at the top of the file
 * for typical values per refresh rate.
 */
static ssize_t frame_budget_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->frame_budget_us);
}

static ssize_t frame_budget_us_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_FRAME_BUDGET_US_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_budget_us, val);
	return count;
}
static struct governor_attr frame_budget_us = __ATTR_RW(frame_budget_us);

/* frame_budget_us_auto sysfs knob.  Boolean (0/1).  When 1, the
 * adaptive frame-budget floor and the auto-tune V2 classifier both
 * prefer the drm-side cached vblank period (zenith_drm_vblank_us)
 * over the userspace-set frame_budget_us.  Falls back to the
 * userspace value when the cache is empty so existing tunings keep
 * working on systems whose drm driver doesn't yet call
 * zenith_set_drm_vblank_us().
 */
static ssize_t frame_budget_us_auto_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->frame_budget_us_auto);
}

static ssize_t frame_budget_us_auto_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->frame_budget_us_auto, val);
	return count;
}
static struct governor_attr frame_budget_us_auto =
	__ATTR_RW(frame_budget_us_auto);

/* drm_vblank_us read-only debug knob.  Mirrors the cached
 * zenith_drm_vblank_us atomic so userspace can verify the drm
 * driver actually called zenith_set_drm_vblank_us().  Read-only:
 * userspace tuning should write frame_budget_us instead.
 */
static ssize_t drm_vblank_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       (unsigned int)atomic_read(&zenith_drm_vblank_us));
}
static struct governor_attr drm_vblank_us = __ATTR_RO(drm_vblank_us);

/* frame_budget_us_per_policy sysfs knob.  CSV "cpu:budget_us[,...]"
 * with the per-policy override semantics described in the comment
 * block at the top of the file.  Empty write clears all overrides.
 */
static ssize_t frame_budget_us_per_policy_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	ssize_t len = 0;
	unsigned int cpu;
	bool first = true;

	/* Iterate only over CPUs that actually exist on this machine.
	 * Slots in [nr_cpu_ids, NR_CPUS) are kept at zero by the store
	 * side (input is rejected with -EINVAL above nr_cpu_ids), so
	 * skipping them here is functionally equivalent and avoids a
	 * read of zero-initialized storage we know cannot be non-zero.
	 * sysfs_emit_at is the bounded form of sprintf for sysfs show
	 * handlers: identical formatted output, but it cannot overrun
	 * the PAGE_SIZE buffer the sysfs core hands us.
	 */
	for_each_possible_cpu(cpu) {
		unsigned int v = t->frame_budget_us_per_policy[cpu];

		if (!v)
			continue;
		len += sysfs_emit_at(buf, len, "%s%u:%u",
				     first ? "" : ",", cpu, v);
		first = false;
	}
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static ssize_t frame_budget_us_per_policy_store(struct gov_attr_set *attr_set,
						const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int parsed[NR_CPUS] = { 0 };
	const char *p = buf;
	const char *end = buf + count;
	unsigned int cpu;

	/* Empty write (just "\n" or "") clears all overrides. */
	while (p < end && (*p == ' ' || *p == '\t' || *p == '\n'))
		p++;
	if (p == end)
		goto commit;

	while (p < end) {
		unsigned int anchor;
		unsigned int val;
		char *colon;
		char *comma;
		char token[32];
		size_t tlen;

		comma = strnchr(p, end - p, ',');
		tlen = comma ? (size_t)(comma - p) : (size_t)(end - p);
		if (tlen >= sizeof(token))
			return -EINVAL;
		memcpy(token, p, tlen);
		token[tlen] = '\0';
		/* Strip trailing whitespace / newline. */
		while (tlen && (token[tlen - 1] == ' ' ||
				token[tlen - 1] == '\t' ||
				token[tlen - 1] == '\n'))
			token[--tlen] = '\0';
		if (!tlen) {
			p = comma ? comma + 1 : end;
			continue;
		}

		colon = strchr(token, ':');
		if (!colon)
			return -EINVAL;
		*colon = '\0';
		if (kstrtouint(token, 10, &anchor))
			return -EINVAL;
		if (kstrtouint(colon + 1, 10, &val))
			return -EINVAL;
		/* Reject input that targets a CPU index above the number
		 * of CPUs that actually exist on this machine.  The array
		 * is still sized [NR_CPUS], but slots in [nr_cpu_ids,
		 * NR_CPUS) are dead storage -- accepting them here would
		 * silently mask a typo / misconfigured boot string.
		 */
		if (anchor >= nr_cpu_ids || val > ZENITH_FRAME_BUDGET_US_MAX)
			return -EINVAL;
		parsed[anchor] = val;
		p = comma ? comma + 1 : end;
	}

commit:
	/* Only write to slots for CPUs that actually exist.  Slots in
	 * [nr_cpu_ids, NR_CPUS) stay at zero (the kzalloc value) for
	 * the lifetime of the tunables, because the bounds check above
	 * never lets the store path write a non-zero value into them.
	 */
	for_each_possible_cpu(cpu)
		WRITE_ONCE(t->frame_budget_us_per_policy[cpu], parsed[cpu]);
	return count;
}
static struct governor_attr frame_budget_us_per_policy =
	__ATTR_RW(frame_budget_us_per_policy);

/* frame_pace_floor_pct sysfs knob.  Range 0..100; the value is the
 * 60 Hz baseline floor as a percent of policy->max.  The kernel
 * scales this inversely with frame_budget_us, so the same value
 * gives a higher effective floor at higher refresh rates.  0
 * disables the floor while leaving the tracepoint live.
 */
static ssize_t frame_pace_floor_pct_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->frame_pace_floor_pct);
}

static ssize_t frame_pace_floor_pct_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	WRITE_ONCE(t->frame_pace_floor_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_FRAME_PACE);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr frame_pace_floor_pct =
	__ATTR_RW(frame_pace_floor_pct);

/*
 * uclamp_min_respect sysfs knob.  See the ZENITH_DEFAULT_UCLAMP_MIN_RESPECT
 * comment block at the top of this file for full semantics.  Normalised to
 * 0/1 on store.  No cache invalidation required -- the value is re-read
 * from tunables on every zenith_get_next_freq() call.
 */
ZENITH_TUNABLE_UINT_MAX(uclamp_min_respect, 1);

/* peer_ramp_uclamp_min_respect sysfs knob (Patch M2).  Range
 * 0..1.  When set, the peer_ramp floor is computed as
 * max(peer_ramp_floor_pct, uclamp_min_pct).  When 0, the
 * floor uses peer_ramp_floor_pct verbatim (Stage 4 behaviour).
 * Independent of the master uclamp_min_respect knob.
 */
static ssize_t
peer_ramp_uclamp_min_respect_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       peer_ramp_uclamp_min_respect);
}

static ssize_t
peer_ramp_uclamp_min_respect_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->peer_ramp_uclamp_min_respect, val);
	return count;
}

static struct governor_attr peer_ramp_uclamp_min_respect =
	__ATTR_RW(peer_ramp_uclamp_min_respect);

/* migration_floor_uclamp_min_respect sysfs knob (Patch M2).
 * Range 0..1.  Same shape as peer_ramp_uclamp_min_respect above
 * but for the migration_floor read site.  Independent of both
 * the master uclamp_min_respect knob and the peer_ramp variant.
 */
static ssize_t
migration_floor_uclamp_min_respect_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       migration_floor_uclamp_min_respect);
}

static ssize_t
migration_floor_uclamp_min_respect_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->migration_floor_uclamp_min_respect, val);
	return count;
}

static struct governor_attr migration_floor_uclamp_min_respect =
	__ATTR_RW(migration_floor_uclamp_min_respect);

/*
 * uclamp_max_respect sysfs knob.  See the ZENITH_DEFAULT_UCLAMP_MAX_RESPECT
 * comment block at the top of this file for full semantics.  Normalised to
 * 0/1 on store.
 */
ZENITH_TUNABLE_UINT_MAX(uclamp_max_respect, 1);


/*
 * calibrate_enable: toggle boot-time hardware calibration.
 * When disabled, the compile-time DEFAULT_UP/DOWN_RATE_LIMIT_US
 * values are used unchanged.
 */
static ssize_t calibrate_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(zenith_calibrate_enable));
}

static ssize_t calibrate_enable_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	unsigned int val;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;
	WRITE_ONCE(zenith_calibrate_enable, val);
	return count;
}

static struct governor_attr calibrate_enable =
	__ATTR(calibrate_enable, 0644, calibrate_enable_show,
	       calibrate_enable_store);

static struct attribute *zenith_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&up_threshold.attr,
	&up_threshold_adaptive.attr,
	&up_threshold_hispeed.attr,
	&down_threshold.attr,
	&hispeed_freq.attr,
	&hispeed_freq_pct.attr,
	&hispeed_load.attr,
	&hispeed_hyst_pct.attr,
	&hispeed_entry_streak.attr,
	&brutal_entry_streak.attr,
	&peak_headroom_rescue.attr,
	&peak_headroom_starve_load_pct.attr,
	&peak_headroom_freq_floor_pct.attr,
	&peak_headroom_starve_streak.attr,
	&peak_headroom_jump_pct.attr,
	&peak_headroom_hold_ms.attr,
	&batt_hold_scale_pct.attr,
	&on_battery.attr,
	&charger_aware.attr,
	&charger_floor_pct.attr,
	&top_app_aware.attr,
	&top_app_floor_pct.attr,
	&render_thread_util_aware.attr,
	&render_thread_util_thresh.attr,
	&render_thread_util_floor_pct.attr,
	&pmu_aware.attr,
	&pmu_ipc_thresh.attr,
	&pmu_ipc_floor_pct.attr,
	&em_aware.attr,
	&em_floor_pct.attr,
	&cluster_wake_pulse_ms.attr,
	&cluster_wake_pulse_idle_ms.attr,
	&cluster_wake_pulse_floor_pct.attr,
	&quiet_hours_start_min.attr,
	&quiet_hours_end_min.attr,
	&quiet_hours_cap_pct.attr,
	&quiet_hours_screen_off_only.attr,
	&decision_latency_hist.attr,
	&fg_transition_pulse_ms.attr,
	&fg_transition_pulse_pct.attr,
	&peak_headroom_prearm.attr,
	&predict_up_thresh.attr,
	&predict_up_window.attr,
	&pelt_rising_edge_thresh.attr,
	&pelt_rising_edge_min_pct.attr,
	&dl_task_floor_pct.attr,
	&io_floor_hyst_ms.attr,
	&io_floor_hyst_pct.attr,
	&peak_hysteresis_streak.attr,
	&peak_step_down_pct.attr,
	&boost_idle_thresh.attr,
	&boost_idle_streak.attr,
	&bg_util_scale_pct.attr,
	&sleeper_tail_thresh_us.attr,
	&sleeper_tail_pct.attr,
	&peer_ramp_window_ms.attr,
	&peer_ramp_floor_pct.attr,
	&peer_ramp_window_off_ms.attr,
	&migration_jump_pct.attr,
	&migration_floor_window_ms.attr,
	&migration_floor_pct.attr,
	&psi_cpu_floor_thresh.attr,
	&frame_overrun_slack_us.attr,
	&frame_overrun_window_ms.attr,
	&frame_overrun_floor_pct.attr,
	&frame_overrun_deep_streak.attr,
	&frame_overrun_deep_floor_pct.attr,
	&psi_mem_cap_thresh.attr,
	&psi_mem_cap_pct.attr,
	&psi_mem_cap_window_ms.attr,
	&brutal_decay_ms.attr,
	&climb_mode.attr,
	&freq_step_pct.attr,
	&freq_step_adaptive.attr,
	&profile.attr,
	&verbose_log.attr,
	&game_perf_burst.attr,
	&game_perf_burst_floor_pct.attr,
	&game_perf_burst_thermal_ceiling_dc.attr,
	&game_perf_burst_disarm_grace_ms.attr,
	&game_perf_burst_cooldown_ms.attr,
	&game_perf_burst_state.attr,
	&game_perf_burst_stats.attr,
	&auto_target.attr,
	&auto_eval_ms.attr,
	&auto_hysteresis_ms.attr,
	&profile_values.attr,
	&zenith_stats.attr,
	&zenith_stats_reset.attr,
	&zenith_input_stats.attr,
	&at_log.attr,
	&last_decision_path.attr,
	&decision_ring.attr,
	&decision_confidence.attr,
	&auto_tune_status.attr,
	&auto_tune_state_residency.attr,
	&auto_tune_state_history.attr,
	&auto_tune_reset_overrides.attr,
	&auto_tune.attr,
	&auto_tune_v2.attr,
	&auto_tune_v2_glides.attr,
	&auto_tune_v2_tiers.attr,
	&auto_tune_hysteresis_windows.attr,
	&auto_tune_cooldown_windows.attr,
	&auto_tune_v2_var_promote_thresh.attr,
	&auto_tune_util_rising_thresh_pct.attr,
	&auto_tune_render_rt_floor_pct.attr,
	&auto_tune_v3.attr,
	&auto_tune_v3_interval_ms.attr,
	&auto_tune_v3_state.attr,
	&auto_tune_v3_calib_log.attr,
	&auto_tune_cluster_aware.attr,
	&auto_tune_v2_signals.attr,
	&auto_tune_thermal_slope.attr,
	&auto_tune_thermal_pressure_pct.attr,
	&auto_tune_thermal_slope_pct.attr,
	&auto_tune_frame_pacing.attr,
	&auto_tune_sustained_gaming.attr,
	&auto_tune_sat_load_pct.attr,
	&auto_tune_hi_sat_pct.attr,
	&auto_tune_lo_sat_pct.attr,
	&auto_tune_hi_events_x2.attr,
	&auto_tune_lo_events_x2.attr,
	&auto_tune_scenario.attr,
	&powersave_bias.attr,
	&screen_on_bias_pct.attr,
	&io_is_busy.attr,
	&iowait_boost_min.attr,
	&iowait_stack_pct.attr,
	&iowait_backoff_after_ms.attr,
	&ignore_nice_load.attr,
	&screen_state.attr,
	&screen_off_glide_ms.attr,
	&screen_auto.attr,
	&thermal_state.attr,
	&thermal_auto.attr,
	&thermal_aware.attr,
	&thermal_active.attr,
	&thermal_pressure_continuous.attr,
	&thermal_util_derate.attr,
	&thermal_derate_rate_pct.attr,
	&auto_thermal_cap.attr,
	&auto_thermal_cap_pressure_pct.attr,
	&auto_thermal_cap_freq_pct.attr,
	&freq_stability_margin_pct.attr,
	&down_rate_adaptive.attr,
	&wakeup_boost.attr,
	&wakeup_boost_ms.attr,
	&down_threshold_adaptive.attr,
	&rate_limit_cluster_scale.attr,
	&input_boost_ms.attr,
	&input_boost_decay_ms.attr,
	&input_boost_touchdown_extra_ms.attr,
	&input_boost_decay_curve.attr,
	&input_boost_big_only.attr,
	&input_boost_cap_pct.attr,
	&input_boost_down_rate_mult_pct.attr,
	&efficient_freq.attr,
	&eff_bin_hyst_pct.attr,
	&up_delay_us.attr,
	&light_load_freq.attr,
	&light_load_threshold.attr,
	&sampling_down_factor.attr,
	&boost_exit_extend.attr,
	&bias_load_threshold.attr,
	&kcpustat_window_us.attr,
	&kcpustat_filter_shift.attr,
	&kcpustat_hispeed_enable.attr,
	&util_math_v2.attr,
	&uclamp_min_respect.attr,
	&peer_ramp_uclamp_min_respect.attr,
	&migration_floor_uclamp_min_respect.attr,
	&uclamp_max_respect.attr,
	&predict_util_pct.attr,
	&predict_util_smooth.attr,
	&render_aware.attr,
	&render_comms.attr,
	&render_floor_pct.attr,
	&render_floor_min_runtime_ms.attr,
	&audio_aware.attr,
	&audio_comms.attr,
	&audio_floor_pct.attr,
	&audio_cap_pct.attr,
	&audio_hyst_ms.attr,
	&vh_arch_freq_scale_enable.attr,
	&vh_uclamp_observer_enable.attr,
	&vh_cpu_idle_enable.attr,
	&vh_freq_qos_enable.attr,
	&vh_sched_move_task_enable.attr,
	&vh_scheduler_tick_enable.attr,
	&camera_aware.attr,
	&camera_comms.attr,
	&camera_active.attr,
	&camera_floor_pct.attr,
	&game_mode.attr,
	&psi_aware.attr,
	&psi_mem_thresh.attr,
	&psi_cpu_thresh.attr,
	&psi_io_thresh.attr,
	&psi_cgroup_path.attr,
	&boot_boost_ms.attr,
	&boot_boost_decay_ms.attr,
	&boot_complete.attr,
	&boot_complete_auto.attr,
	&game_auto.attr,
	&game_auto_state.attr,
	&game_auto_comms.attr,
	&frame_budget_us.attr,
	&frame_budget_us_auto.attr,
	&drm_vblank_us.attr,
	&frame_budget_us_per_policy.attr,
	&frame_pace_floor_pct.attr,
	&calibrate_enable.attr,
	NULL
};
ATTRIBUTE_GROUPS(zenith);

static void zenith_tunables_free(struct kobject *kobj)
{
	struct zenith_tunables *t =
		to_zenith_tunables(container_of(kobj, struct gov_attr_set,
						kobj));

	/* Patch B-AUTO-3: drain the auto-selector worker before
	 * freeing the tunables container.  The worker re-arms itself
	 * unconditionally while active_profile == AUTO so we must
	 * flip active_profile to a non-AUTO value first (the kobject
	 * is going away, so any value will do); cancel_delayed_work_-
	 * sync then returns once the in-flight invocation has
	 * observed the new value via the rearm-side READ_ONCE guard.
	 */
	WRITE_ONCE(t->active_profile, ZENITH_PROFILE_CUSTOM);
	cancel_delayed_work_sync(&t->eval_work);

	kfree(t);
}

static struct kobj_type zenith_tunables_ktype = {
	.default_groups = zenith_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &zenith_tunables_free,
};

/********************** Lifecycle & Registration *********************/

static int zenith_kthread_create(struct zenith_policy *z_policy)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAG_SUGOV,
		.sched_nice	= 0,
		.sched_priority	= 0,
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
	int ret;

	if (z_policy->policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&z_policy->work, zenith_work);
	kthread_init_worker(&z_policy->worker);
	thread = kthread_create(kthread_worker_fn, &z_policy->worker, "zenith:%d",
				cpumask_first(z_policy->policy->related_cpus));
	if (IS_ERR(thread))
		return PTR_ERR(thread);

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		return ret;
	}

	z_policy->thread = thread;
	if (!z_policy->policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, z_policy->policy->related_cpus);

	init_irq_work(&z_policy->irq_work, zenith_irq_work);
	mutex_init(&z_policy->work_lock);
	wake_up_process(thread);

	return 0;
}

/* Wave B PMU IPC tracker.  Allocate the per-CPU instructions and
 * cycles perf_events on this CPU.  Idempotent: if the events are
 * already allocated (re-attach on the same CPU after a governor
 * cycle), return success without re-allocating.  Allocation failures
 * (PMU not exposed by the SoC, perf locked down, OOM) leave the
 * pointers NULL; subsequent zenith_pmu_sample_cpu() calls return
 * early and the floor never applies on this CPU.  See the comment
 * block above ZENITH_DEFAULT_PMU_AWARE for the full rationale.
 */
#if IS_ENABLED(CONFIG_PERF_EVENTS)
static int zenith_pmu_init_cpu(unsigned int cpu)
{
	struct zenith_pmu_state *st = per_cpu_ptr(&zenith_pmu, cpu);
	struct perf_event_attr inst_attr = {
		.type		= PERF_TYPE_HARDWARE,
		.config		= PERF_COUNT_HW_INSTRUCTIONS,
		.size		= sizeof(inst_attr),
		.pinned		= 1,
		.disabled	= 0,
		.exclude_idle	= 1,
	};
	struct perf_event_attr cycle_attr = inst_attr;

	cycle_attr.config = PERF_COUNT_HW_CPU_CYCLES;

	if (st->inst_event && st->cycle_event)
		return 0;

	if (!st->inst_event) {
		struct perf_event *e =
			perf_event_create_kernel_counter(&inst_attr, cpu,
							 NULL, NULL, NULL);

		if (IS_ERR(e)) {
			st->inst_event = NULL;
			return PTR_ERR(e);
		}
		st->inst_event = e;
	}
	if (!st->cycle_event) {
		struct perf_event *e =
			perf_event_create_kernel_counter(&cycle_attr, cpu,
							 NULL, NULL, NULL);

		if (IS_ERR(e)) {
			perf_event_release_kernel(st->inst_event);
			st->inst_event = NULL;
			st->cycle_event = NULL;
			return PTR_ERR(e);
		}
		st->cycle_event = e;
	}
	st->last_inst = 0;
	st->last_cycles = 0;
	st->ipc_pct = 0;
	return 0;
}

static void zenith_pmu_exit_cpu(unsigned int cpu)
{
	struct zenith_pmu_state *st = per_cpu_ptr(&zenith_pmu, cpu);

	if (st->inst_event) {
		perf_event_release_kernel(st->inst_event);
		st->inst_event = NULL;
	}
	if (st->cycle_event) {
		perf_event_release_kernel(st->cycle_event);
		st->cycle_event = NULL;
	}
	st->last_inst = 0;
	st->last_cycles = 0;
	st->ipc_pct = 0;
}

/* Sample the per-CPU instructions and cycles counters and compute
 * IPC as a percentage.  Called from zenith_auto_tune_work() (process
 * context); perf_event_read_value() uses smp_call_function_single()
 * internally to read the counter on the target CPU, so this is safe
 * to call on any CPU regardless of which CPU owns the event.  Stores
 * the latest IPC in st->ipc_pct via WRITE_ONCE() so the lock-free
 * reader in zenith_policy_max_ipc_pct() sees a coherent value.
 */
static void zenith_pmu_sample_cpu(unsigned int cpu)
{
	struct zenith_pmu_state *st = per_cpu_ptr(&zenith_pmu, cpu);
	u64 enabled, running, inst, cycles, di, dc;

	if (!st->inst_event || !st->cycle_event)
		return;

	inst   = perf_event_read_value(st->inst_event,  &enabled, &running);
	cycles = perf_event_read_value(st->cycle_event, &enabled, &running);

	di = inst   - st->last_inst;
	dc = cycles - st->last_cycles;
	st->last_inst   = inst;
	st->last_cycles = cycles;

	if (dc) {
		u64 r = div64_u64(di * 100, dc);

		if (r > ZENITH_PMU_IPC_THRESH_MAX)
			r = ZENITH_PMU_IPC_THRESH_MAX;
		WRITE_ONCE(st->ipc_pct, (unsigned int)r);
	}
}

static unsigned int zenith_policy_max_ipc_pct(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	unsigned int max_ipc = 0;

	for_each_cpu(cpu, policy->cpus) {
		struct zenith_pmu_state *st = per_cpu_ptr(&zenith_pmu, cpu);
		unsigned int ipc = READ_ONCE(st->ipc_pct);

		if (ipc > max_ipc)
			max_ipc = ipc;
	}
	return max_ipc;
}
#else
static inline int zenith_pmu_init_cpu(unsigned int cpu)
{
	return 0;
}
static inline void zenith_pmu_exit_cpu(unsigned int cpu)
{
}
static inline void zenith_pmu_sample_cpu(unsigned int cpu)
{
}
static inline unsigned int
zenith_policy_max_ipc_pct(struct zenith_policy *z_policy)
{
	return 0;
}
#endif

/* Wave B EAS / Energy Model integration.  See the comment block
 * above ZENITH_DEFAULT_EM_AWARE for the full rationale.  On
 * CONFIG_ENERGY_MODEL=n, em_cpu_get() returns NULL unconditionally
 * (header-defined) and this function returns 0; the caller treats
 * 0 as "no EM" and the em_floor never applies.  No #if guard is
 * required at this site because the header itself stubs the API.
 */
static unsigned int zenith_em_knee_freq(struct zenith_policy *z_policy)
{
	struct em_perf_domain *em;
	unsigned int cpu = cpumask_first(z_policy->policy->cpus);
	unsigned long best_cost = ULONG_MAX;
	unsigned int knee_freq = 0;
	int i;

	if (z_policy->em_knee_freq)
		return z_policy->em_knee_freq;

	em = em_cpu_get(cpu);
	if (!em || em->nr_perf_states <= 0 || !em->table)
		return 0;

	for (i = 0; i < em->nr_perf_states; i++) {
		unsigned long c = em->table[i].cost;

		if (c && c < best_cost) {
			best_cost = c;
			knee_freq = (unsigned int)em->table[i].frequency;
		}
	}

	/* Bad EM (all costs zero, or single-OPP table): fall back to
	 * the lowest registered freq, which is at least a meaningful
	 * lower bound for the knee.  Caller still scales by
	 * em_floor_pct so a 0% effective floor is fine.
	 */
	if (!knee_freq)
		knee_freq = (unsigned int)em->table[0].frequency;

	z_policy->em_knee_freq = knee_freq;
	return knee_freq;
}

/*
 * Boot-time hardware calibration: reads the cpufreq driver transition
 * latency and computes optimal up/down rate limits for this SoC.
 * Called once from zenith_init() after the tunables are allocated.
 */
static void zenith_calibrate(struct cpufreq_policy *policy)
{
	static bool calibrated_done;
	unsigned int latency_ns;
	unsigned int cal_up, cal_down;

	if (!READ_ONCE(zenith_calibrate_enable))
		return;
	if (calibrated_done)
		return;

	latency_ns = policy->cpuinfo.transition_latency;
	if (!latency_ns || latency_ns > 10000000)
		return;

	/* At least 2x transition latency, floor at 100 us */
	cal_up = max(latency_ns / 500, ZENITH_DEFAULT_UP_RATE_LIMIT_US);
	if (cal_up > 5000)
		cal_up = 5000;

	/* Down rate preserves the 40:1 ratio from the defaults */
	cal_down = max(cal_up * 40, ZENITH_DEFAULT_DOWN_RATE_LIMIT_US);
	if (cal_down > 50000)
		cal_down = 50000;

	WRITE_ONCE(zenith_calibrated_up_us, cal_up);
	calibrated_done = true;
	WRITE_ONCE(zenith_calibrated_down_us, cal_down);

	pr_info("zenith: hw calibration: transition_latency=%uns up=%uus down=%uus\n"
,		latency_ns, cal_up, cal_down);
}

static int zenith_init(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy;
	struct zenith_tunables *tunables;
	int ret;

	zenith_calibrate(policy);
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	z_policy = kzalloc(sizeof(*z_policy), GFP_KERNEL);
	if (!z_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	z_policy->policy = policy;
	raw_spin_lock_init(&z_policy->update_lock);
	INIT_DELAYED_WORK(&z_policy->at_work, zenith_auto_tune_work);
	/* Patch J: prime the per-policy decision tag so the sysfs
	 * node returns a meaningful value before the first eval.
	 */
	z_policy->last_decision_path = "init";

	ret = zenith_kthread_create(z_policy);
	if (ret)
		goto free_z_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		tunables = global_tunables;
		gov_attr_set_get(&tunables->attr_set, &z_policy->tunables_hook);
		goto out;
	}

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (!tunables) {
		ret = -ENOMEM;
		goto unlock;
	}

	gov_attr_set_init(&tunables->attr_set, &z_policy->tunables_hook);

	tunables->up_rate_limit_us	= READ_ONCE(zenith_calibrated_up_us);
	tunables->down_rate_limit_us	= READ_ONCE(zenith_calibrated_down_us);
	tunables->up_threshold		= ZENITH_DEFAULT_UP_THRESHOLD;
	tunables->up_threshold_adaptive	= ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE;
	tunables->up_threshold_hispeed	= ZENITH_DEFAULT_UP_THRESHOLD_HISPEED;
	tunables->down_threshold	= ZENITH_DEFAULT_DOWN_THRESHOLD;
	tunables->hispeed_freq		= ZENITH_DEFAULT_HISPEED_FREQ;
	tunables->hispeed_freq_pct	= ZENITH_DEFAULT_HISPEED_FREQ_PCT;
	tunables->hispeed_load		= ZENITH_DEFAULT_HISPEED_LOAD;
	tunables->hispeed_hyst_pct	= ZENITH_DEFAULT_HISPEED_HYST_PCT;
	tunables->hispeed_entry_streak	= ZENITH_DEFAULT_HISPEED_ENTRY_STREAK;
	tunables->brutal_entry_streak	= ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK;
	tunables->peak_headroom_rescue	= ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE;
	tunables->peak_headroom_starve_load_pct =
		ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_LOAD_PCT;
	tunables->peak_headroom_freq_floor_pct =
		ZENITH_DEFAULT_PEAK_HEADROOM_FREQ_FLOOR_PCT;
	tunables->peak_headroom_starve_streak =
		ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_STREAK;
	tunables->peak_headroom_jump_pct =
		ZENITH_DEFAULT_PEAK_HEADROOM_JUMP_PCT;
	tunables->peak_headroom_hold_ms =
		ZENITH_DEFAULT_PEAK_HEADROOM_HOLD_MS;
	tunables->peak_headroom_prearm =
		ZENITH_DEFAULT_PEAK_HEADROOM_PREARM;
	tunables->batt_hold_scale_pct =
		ZENITH_DEFAULT_BATT_HOLD_SCALE_PCT;
	tunables->charger_aware =
		ZENITH_DEFAULT_CHARGER_AWARE;
	tunables->charger_floor_pct =
		ZENITH_DEFAULT_CHARGER_FLOOR_PCT;
	tunables->top_app_aware =
		ZENITH_DEFAULT_TOP_APP_AWARE;
	tunables->top_app_floor_pct =
		ZENITH_DEFAULT_TOP_APP_FLOOR_PCT;
	tunables->render_thread_util_aware =
		ZENITH_DEFAULT_RENDER_THREAD_UTIL_AWARE;
	tunables->render_thread_util_thresh =
		ZENITH_DEFAULT_RENDER_THREAD_UTIL_THRESH;
	tunables->render_thread_util_floor_pct =
		ZENITH_DEFAULT_RENDER_THREAD_UTIL_FLOOR_PCT;
	tunables->pmu_aware =
		ZENITH_DEFAULT_PMU_AWARE;
	tunables->pmu_ipc_thresh =
		ZENITH_DEFAULT_PMU_IPC_THRESH;
	tunables->pmu_ipc_floor_pct =
		ZENITH_DEFAULT_PMU_IPC_FLOOR_PCT;
	tunables->em_aware =
		ZENITH_DEFAULT_EM_AWARE;
	tunables->em_floor_pct =
		ZENITH_DEFAULT_EM_FLOOR_PCT;
	tunables->cluster_wake_pulse_ms =
		ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS;
	tunables->cluster_wake_pulse_idle_ms =
		ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_IDLE_MS;
	tunables->cluster_wake_pulse_floor_pct =
		ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_FLOOR_PCT;
	tunables->quiet_hours_start_min =
		ZENITH_DEFAULT_QUIET_HOURS_START_MIN;
	tunables->quiet_hours_end_min =
		ZENITH_DEFAULT_QUIET_HOURS_END_MIN;
	tunables->quiet_hours_cap_pct =
		ZENITH_DEFAULT_QUIET_HOURS_CAP_PCT;
	tunables->quiet_hours_screen_off_only =
		ZENITH_DEFAULT_QUIET_HOURS_SCREEN_OFF_ONLY;
	tunables->fg_transition_pulse_ms =
		ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS;
	tunables->fg_transition_pulse_pct =
		ZENITH_DEFAULT_FG_TRANSITION_PULSE_PCT;
	tunables->predict_up_thresh	= ZENITH_DEFAULT_PREDICT_UP_THRESH;
	tunables->predict_up_window	= ZENITH_DEFAULT_PREDICT_UP_WINDOW;
	tunables->pelt_rising_edge_thresh =
		ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH;
	tunables->pelt_rising_edge_min_pct =
		ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT;
	tunables->dl_task_floor_pct	= ZENITH_DEFAULT_DL_TASK_FLOOR_PCT;
	tunables->io_floor_hyst_ms	= ZENITH_DEFAULT_IO_FLOOR_HYST_MS;
	tunables->io_floor_hyst_pct	= ZENITH_DEFAULT_IO_FLOOR_HYST_PCT;
	tunables->vh_arch_freq_scale_enable =
		ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE;
	tunables->vh_uclamp_observer_enable =
		ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE;
	tunables->vh_cpu_idle_enable =
		ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE;
	tunables->vh_freq_qos_enable =
		ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE;
	tunables->vh_sched_move_task_enable =
		ZENITH_DEFAULT_VH_SCHED_MOVE_TASK_ENABLE;
	tunables->vh_scheduler_tick_enable =
		ZENITH_DEFAULT_VH_SCHEDULER_TICK_ENABLE;
	/* vh_freq_qos_pressure_until_ns is already 0 from kzalloc;
	 * 0 < any future ktime_get_ns() so the auto-classify check
	 * starts disarmed.  No explicit atomic64_set needed.
	 */
	/* Patch B-AUTO-2: seed auto_target so the auto_target sysfs
	 * node never reads 0 / "balanced" by accident on a fresh
	 * tunables alloc.  The actual cold-boot active_profile flip to
	 * ZENITH_PROFILE_AUTO lives in B-AUTO-5; here we just ensure
	 * the meta-state field is well-defined when a user writes
	 * "auto" to the profile sysfs node before any auto eval has
	 * landed.
	 */
	tunables->auto_target		= ZENITH_PROFILE_BALANCED;
	/* Patch B-AUTO-3: cadence + hysteresis defaults for the auto
	 * selector engine.  The eval_work itself is initialised below
	 * after the sysfs attr_set publication, but we need the
	 * tunables to carry sane values from this point so a
	 * subsequent profile_store("auto") schedules at the intended
	 * cadence rather than at jiffies-now (msecs_to_jiffies(0) is
	 * 0 jiffies, i.e. "run immediately on the next tick").
	 */
	tunables->auto_eval_ms		= ZENITH_DEFAULT_AUTO_EVAL_MS;
	tunables->auto_hysteresis_ms	= ZENITH_DEFAULT_AUTO_HYSTERESIS_MS;
	tunables->auto_pending_target	= ZENITH_PROFILE_BALANCED;
	tunables->auto_pending_first_seen_ns = 0;
	INIT_DEFERRABLE_WORK(&tunables->eval_work, zenith_auto_eval_work_fn);
	tunables->peak_hysteresis_streak =
		ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK;
	tunables->peak_step_down_pct	= ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT;
	tunables->boost_idle_thresh	= ZENITH_DEFAULT_BOOST_IDLE_THRESH;
	tunables->boost_idle_streak	= ZENITH_DEFAULT_BOOST_IDLE_STREAK;
	tunables->bg_util_scale_pct	= ZENITH_DEFAULT_BG_UTIL_SCALE_PCT;
	tunables->sleeper_tail_thresh_us =
		ZENITH_DEFAULT_SLEEPER_TAIL_THRESH_US;
	tunables->sleeper_tail_pct	= ZENITH_DEFAULT_SLEEPER_TAIL_PCT;
	tunables->peer_ramp_window_ms	=
		ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS;
	tunables->peer_ramp_floor_pct	=
		ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT;
	tunables->peer_ramp_window_off_ms =
		ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS;
	tunables->migration_jump_pct	=
		ZENITH_DEFAULT_MIGRATION_JUMP_PCT;
	tunables->migration_floor_window_ms =
		ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS;
	tunables->migration_floor_pct	=
		ZENITH_DEFAULT_MIGRATION_FLOOR_PCT;
	tunables->psi_cpu_floor_thresh	=
		ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH;
	tunables->frame_overrun_slack_us =
		ZENITH_DEFAULT_FRAME_OVERRUN_SLACK_US;
	tunables->frame_overrun_window_ms =
		ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS;
	tunables->frame_overrun_floor_pct =
		ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT;
	tunables->frame_overrun_deep_streak =
		ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK;
	tunables->frame_overrun_deep_floor_pct =
		ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT;
	tunables->psi_mem_cap_thresh =
		ZENITH_DEFAULT_PSI_MEM_CAP_THRESH;
	tunables->psi_mem_cap_pct =
		ZENITH_DEFAULT_PSI_MEM_CAP_PCT;
	tunables->psi_mem_cap_window_ms =
		ZENITH_DEFAULT_PSI_MEM_CAP_WINDOW_MS;
	tunables->climb_mode		= ZENITH_DEFAULT_CLIMB_MODE;
	tunables->freq_step_pct		= ZENITH_DEFAULT_FREQ_STEP_PCT;
	tunables->freq_step_adaptive	= ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE;
	tunables->active_profile	= ZENITH_PROFILE_CUSTOM;
	tunables->auto_tune		= 1;
	tunables->auto_tune_sat_load_pct = ZENITH_DEFAULT_AT_SAT_LOAD_PCT;
	tunables->auto_tune_hi_sat_pct	= ZENITH_DEFAULT_AT_HI_SAT_PCT;
	tunables->auto_tune_lo_sat_pct	= ZENITH_DEFAULT_AT_LO_SAT_PCT;
	tunables->auto_tune_hi_events_x2 = ZENITH_DEFAULT_AT_HI_EVENTS_X2;
	tunables->auto_tune_lo_events_x2 = ZENITH_DEFAULT_AT_LO_EVENTS_X2;
	tunables->auto_tune_v2		= ZENITH_DEFAULT_AUTO_TUNE_V2;
	tunables->auto_tune_v2_glides	= ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES;
	tunables->auto_tune_v2_tiers	= ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS;
	tunables->auto_tune_hysteresis_windows =
		ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS;
	tunables->auto_tune_cooldown_windows =
		ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS;
	tunables->auto_tune_v2_var_promote_thresh =
		ZENITH_DEFAULT_AT_V2_VAR_PROMOTE_THRESH;
	tunables->auto_tune_util_rising_thresh_pct =
		ZENITH_DEFAULT_AT_UTIL_RISING_THRESH_PCT;
	tunables->auto_tune_render_rt_floor_pct =
		ZENITH_DEFAULT_AT_RENDER_RT_FLOOR_PCT;
	tunables->auto_tune_v3 = ZENITH_DEFAULT_AUTO_TUNE_V3;
	tunables->auto_tune_v3_interval_ms =
		ZENITH_DEFAULT_AT_V3_INTERVAL_MS;
	tunables->auto_tune_cluster_aware =
		ZENITH_DEFAULT_AT_CLUSTER_AWARE;
	tunables->auto_tune_v2_signals =
		ZENITH_DEFAULT_AT_V2_SIGNALS;
	tunables->auto_tune_thermal_slope =
		ZENITH_DEFAULT_AT_THERMAL_SLOPE;
	tunables->auto_tune_thermal_pressure_pct =
		ZENITH_DEFAULT_AT_THERMAL_PRESSURE_PCT;
	tunables->auto_tune_thermal_slope_pct =
		ZENITH_DEFAULT_AT_THERMAL_SLOPE_PCT;
	tunables->auto_tune_frame_pacing =
		ZENITH_DEFAULT_AT_FRAME_PACING;
	tunables->auto_tune_sustained_gaming =
		ZENITH_DEFAULT_AT_SUSTAINED_GAMING;
	tunables->auto_tune_scenario	= ZENITH_DEFAULT_AUTO_TUNE_SCENARIO;
	tunables->powersave_bias	= ZENITH_DEFAULT_POWERSAVE_BIAS;
	tunables->screen_on_bias_pct	= ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT;
	tunables->io_is_busy		= ZENITH_DEFAULT_IO_IS_BUSY;
	tunables->iowait_boost_min	= ZENITH_DEFAULT_IOWAIT_BOOST_MIN;
	tunables->iowait_stack_pct	= ZENITH_DEFAULT_IOWAIT_STACK_PCT;
	tunables->iowait_backoff_after_ms = ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS;
	tunables->ignore_nice_load	= 0;
	tunables->screen_state		= 1;
	tunables->screen_off_glide_ms	= ZENITH_DEFAULT_SCREEN_OFF_GLIDE_MS;
	tunables->screen_auto		= 1;
	tunables->thermal_state		= 0;
	tunables->thermal_auto		= ZENITH_DEFAULT_THERMAL_AUTO;
	tunables->thermal_aware		= ZENITH_DEFAULT_THERMAL_AWARE;
	tunables->thermal_active	= 0;
	tunables->thermal_pressure_continuous =
		ZENITH_DEFAULT_THERMAL_PRESSURE_CONTINUOUS;
		ZENITH_DEFAULT_PREFER_SILVER_HOT_BUMP_PCT;
	tunables->brutal_decay_ms	= ZENITH_DEFAULT_BRUTAL_DECAY_MS;
	tunables->thermal_util_derate	= ZENITH_DEFAULT_THERMAL_UTIL_DERATE;
	tunables->thermal_derate_rate_pct = ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT;
	tunables->auto_thermal_cap	= ZENITH_DEFAULT_AUTO_THERMAL_CAP;
	tunables->auto_thermal_cap_pressure_pct =
		ZENITH_DEFAULT_AUTO_THERMAL_CAP_PRESSURE_PCT;
	tunables->auto_thermal_cap_freq_pct =
		ZENITH_DEFAULT_AUTO_THERMAL_CAP_FREQ_PCT;
	tunables->freq_stability_margin_pct = ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT;
	tunables->down_rate_adaptive	= ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE;
	tunables->wakeup_boost		= ZENITH_DEFAULT_WAKEUP_BOOST;
	tunables->wakeup_boost_ms	= ZENITH_DEFAULT_WAKEUP_BOOST_MS;
	tunables->down_threshold_adaptive = ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE;
	tunables->rate_limit_cluster_scale = ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE;
	tunables->input_boost_ms	= ZENITH_DEFAULT_INPUT_BOOST_MS;
	tunables->input_boost_decay_ms	= ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS;
	tunables->input_boost_touchdown_extra_ms =
		ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS;
	tunables->input_boost_decay_curve = ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE;
	tunables->input_boost_big_only	= ZENITH_DEFAULT_INPUT_BOOST_BIG_ONLY;
	tunables->input_boost_cap_pct	= ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT;
	tunables->input_boost_down_rate_mult_pct =
		ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT;
	tunables->efficient_freq	= ZENITH_DEFAULT_EFFICIENT_FREQ;
	tunables->eff_bin_hyst_pct	= ZENITH_DEFAULT_EFF_BIN_HYST_PCT;
	tunables->up_delay_us		= ZENITH_DEFAULT_UP_DELAY_US;
	tunables->light_load_freq	= ZENITH_DEFAULT_LIGHT_LOAD_FREQ;
	tunables->light_load_threshold	= ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD;
	tunables->sampling_down_factor	= ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR;
	tunables->boost_exit_extend	= ZENITH_DEFAULT_BOOST_EXIT_EXTEND;
	tunables->bias_load_threshold	= ZENITH_DEFAULT_BIAS_LOAD_THRESHOLD;
	tunables->kcpustat_window_us	= ZENITH_DEFAULT_KCPUSTAT_WINDOW_US;
	tunables->kcpustat_filter_shift	= ZENITH_DEFAULT_KCPUSTAT_FILTER_SHIFT;
	tunables->kcpustat_hispeed_enable = ZENITH_DEFAULT_KCPUSTAT_HISPEED_ENABLE;
	tunables->util_math_v2		= ZENITH_DEFAULT_UTIL_MATH_V2;
	tunables->uclamp_min_respect	= ZENITH_DEFAULT_UCLAMP_MIN_RESPECT;
	tunables->peer_ramp_uclamp_min_respect =
		ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT;
	tunables->migration_floor_uclamp_min_respect =
		ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT;
	tunables->uclamp_max_respect	= ZENITH_DEFAULT_UCLAMP_MAX_RESPECT;
	tunables->predict_util_pct	= ZENITH_DEFAULT_PREDICT_UTIL_PCT;
	tunables->predict_util_smooth	= ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH;
	tunables->render_aware		= ZENITH_DEFAULT_RENDER_AWARE;
	tunables->render_floor_pct	= ZENITH_DEFAULT_RENDER_FLOOR_PCT;
	tunables->render_floor_min_runtime_ms =
		ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS;
	tunables->audio_aware		= ZENITH_DEFAULT_AUDIO_AWARE;
	tunables->audio_floor_pct	= ZENITH_DEFAULT_AUDIO_FLOOR_PCT;
	tunables->audio_cap_pct		= ZENITH_DEFAULT_AUDIO_CAP_PCT;
	tunables->audio_hyst_ms		= ZENITH_DEFAULT_AUDIO_HYST_MS;
	tunables->camera_aware		= ZENITH_DEFAULT_CAMERA_AWARE;
	tunables->camera_active		= ZENITH_DEFAULT_CAMERA_ACTIVE;
	tunables->camera_floor_pct	= ZENITH_DEFAULT_CAMERA_FLOOR_PCT;
	tunables->game_mode		= ZENITH_DEFAULT_GAME_MODE;
	tunables->game_auto		= ZENITH_DEFAULT_GAME_AUTO;
	tunables->psi_aware		= ZENITH_DEFAULT_PSI_AWARE;
	tunables->psi_mem_thresh	= ZENITH_DEFAULT_PSI_MEM_THRESH;
	tunables->psi_cpu_thresh	= ZENITH_DEFAULT_PSI_CPU_THRESH;
	tunables->psi_io_thresh		= ZENITH_DEFAULT_PSI_IO_THRESH;
	tunables->boot_boost_ms		= ZENITH_DEFAULT_BOOT_BOOST_MS;
	tunables->boot_boost_decay_ms	= ZENITH_DEFAULT_BOOT_BOOST_DECAY_MS;
	tunables->boot_complete_auto	= ZENITH_DEFAULT_BOOT_COMPLETE_AUTO;
	tunables->frame_budget_us	= ZENITH_DEFAULT_FRAME_BUDGET_US;
	tunables->frame_budget_us_auto	= ZENITH_DEFAULT_FRAME_BUDGET_US_AUTO;
	tunables->frame_pace_floor_pct	= ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT;
	tunables->verbose_log		= ZENITH_DEFAULT_VERBOSE_LOG;
	/* Patch K: game_perf_burst defaults.  See the
	 * ZENITH_DEFAULT_GAME_PERF_BURST comment block for the
	 * rationale on each default.
	 */
	tunables->game_perf_burst	= ZENITH_DEFAULT_GAME_PERF_BURST;
	tunables->game_perf_burst_floor_pct =
		ZENITH_DEFAULT_GAME_PERF_BURST_FLOOR_PCT;
	tunables->game_perf_burst_thermal_ceiling_dc =
		ZENITH_DEFAULT_GAME_PERF_BURST_THERMAL_CEILING_DC;
	tunables->game_perf_burst_disarm_grace_ms =
		ZENITH_DEFAULT_GAME_PERF_BURST_DISARM_GRACE_MS;
	tunables->game_perf_burst_cooldown_ms =
		ZENITH_DEFAULT_GAME_PERF_BURST_COOLDOWN_MS;
	WRITE_ONCE(zenith_input_boost_active_ms, ZENITH_DEFAULT_INPUT_BOOST_MS);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache,
		   ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS);

	/* Sync the audio_aware / render_aware / camera_aware /
	 * psi_aware / game_auto / auto_tune_v3 / thermal_aware static
	 * keys against their default scalars.  See the comment above
	 * DEFINE_STATIC_KEY_FALSE for the invariant: scalars whose
	 * default is non-zero need an explicit init-time key enable.
	 * audio_aware / render_aware were flipped to 1 in wave-2;
	 * game_auto was flipped to 1 and auto_tune_v3 to 2 in wave-7;
	 * camera_aware and psi_aware were flipped to 1 in the
	 * auto-defaults round mirroring audio/render so all six
	 * detector branches ship live by default; thermal_aware
	 * defaults to 1 as the master gate over the thermal-mechanism
	 * cluster.  Idempotent across re-attaches:
	 * zenith_set_static_key() is a no-op if the key is already
	 * in the requested state.  zenith_set_static_key() coerces
	 * non-zero scalars (including the auto_tune_v3 = 2 APPLY
	 * mode) to TRUE, which is the correct branch state for any
	 * non-OFF mode.
	 */
	zenith_set_static_key(&zenith_audio_aware_key,
			      tunables->audio_aware);
	zenith_set_static_key(&zenith_render_aware_key,
			      tunables->render_aware);
	zenith_set_static_key(&zenith_camera_aware_key,
			      tunables->camera_aware);
	zenith_set_static_key(&zenith_psi_aware_key,
			      tunables->psi_aware);
	zenith_set_static_key(&zenith_game_auto_key,
			      tunables->game_auto);
	zenith_set_static_key(&zenith_auto_tune_v3_key,
			      tunables->auto_tune_v3);
	zenith_set_static_key(&zenith_thermal_aware_key,
			      tunables->thermal_aware);
	/* Patch K: same invariant for game_perf_burst.  Default = 1
	 * (master ON, "all automatic"); the FSM evaluator + floor
	 * application both sit inside ZENITH_FEATURE_ENABLED checks
	 * so this sync is what unlocks the hot-path body when the
	 * scalar is non-zero.  Idempotent across re-attaches.
	 */
	zenith_set_static_key(&zenith_game_perf_burst_key,
			      tunables->game_perf_burst);

	/* Apply a cmdline-picked preset before the sysfs attr set is
	 * published, so userspace sees the cmdline-picked preset as the
	 * initial state of the profile node.
	 *
	 * Precedence:
	 *   1. zenith.policy_profile=N:prof wins for the matching
	 *      anchor cpu (cpumask_first(policy->cpus)) -- asymmetric
	 *      big.LITTLE presets, one policy at a time.
	 *   2. zenith.profile= applies to every unmatched policy
	 *      (the historical global behaviour).
	 *   3. No cmdline override -> CUSTOM (historical default).
	 */
	{
		unsigned int anchor = cpumask_first(policy->cpus);
		unsigned int chosen = ZENITH_PROFILE_CUSTOM;

		if (anchor < NR_CPUS &&
		    zenith_cmdline_policy_profile[anchor] !=
		    ZENITH_PROFILE_CUSTOM)
			chosen = zenith_cmdline_policy_profile[anchor];
		else if (zenith_cmdline_profile != ZENITH_PROFILE_CUSTOM)
			chosen = zenith_cmdline_profile;

		if (chosen != ZENITH_PROFILE_CUSTOM) {
			zenith_apply_profile(tunables, chosen);
			tunables->active_profile = chosen;
		} else {
			/* Patch B-AUTO-5: cold-boot default = AUTO.
			 *
			 * No cmdline override and no per-policy
			 * override pinned this tunables container,
			 * so engage the auto-selector engine
			 * immediately.  The BALANCED bake is already
			 * in place from the kzalloc + tunables-init
			 * defaults further up; we simply flip
			 * active_profile to AUTO and stamp
			 * auto_target so the engine sees a known
			 * starting point.  schedule_delayed_work is
			 * deferred until after attr_set publication
			 * (below) -- the eval_work struct must be
			 * fully initialised before it can be
			 * scheduled, and zenith_tunables_alloc has
			 * already INIT_DEFERRABLE_WORK'd it.
			 *
			 * Users who explicitly want manual control
			 * still have every escape hatch:
			 *   - boot with zenith.profile=balanced
			 *     (cmdline override above wins)
			 *   - boot with zenith.profile.policyN=...
			 *     (per-policy override above wins)
			 *   - echo balanced > .../zenith/profile
			 *     (sysfs profile_store disengages auto)
			 *   - echo custom > .../zenith/profile
			 *     (CUSTOM is auto-immune by design)
			 *
			 * LEGACY and CUSTOM remain manual-only
			 * targets: the classifier never picks them,
			 * so a user's explicit "echo custom >
			 * profile" is preserved across the eval
			 * cadence.
			 */
			tunables->active_profile = ZENITH_PROFILE_AUTO;
			WRITE_ONCE(tunables->auto_target,
				   ZENITH_PROFILE_BALANCED);
			tunables->auto_pending_target =
				ZENITH_PROFILE_BALANCED;
			tunables->auto_pending_first_seen_ns = 0;
		}
	}

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &zenith_tunables_ktype,
				   get_governor_parent_kobj(policy),
				   "zenith");
	if (ret) {
		/* kobject_init_and_add() always initialises the kobject
		 * refcount; on failure we must drop that reference via
		 * kobject_put(), which invokes zenith_tunables_free() and
		 * kfrees the backing tunables. Calling kfree() directly
		 * would bypass the release callback and leak any resources
		 * later attached to the ktype.
		 */
		kobject_put(&tunables->attr_set.kobj);
		goto unlock;
	}

	global_tunables = tunables;

	/* Patch B-AUTO-5: arm the auto-selector worker if this fresh
	 * tunables container booted into AUTO (cold-boot default,
	 * see the chosen-selection block above).  schedule_delayed_-
	 * work runs first eval after one auto_eval_ms window so the
	 * device has time to settle on the BALANCED bake before the
	 * classifier starts steering.  Subsequent policies that
	 * attach to this tunables container do not re-schedule --
	 * the worker is one-per-tunables.
	 */
	if (tunables->active_profile == ZENITH_PROFILE_AUTO)
		schedule_delayed_work(&tunables->eval_work,
				      msecs_to_jiffies(tunables->auto_eval_ms ?
						       tunables->auto_eval_ms :
						       ZENITH_DEFAULT_AUTO_EVAL_MS));

out:
	mutex_unlock(&global_tunables_lock);
	z_policy->tunables = tunables;
	zenith_update_cluster_rate_scale(z_policy);
	zenith_reset_local_actions(z_policy);
	policy->governor_data = z_policy;
	return 0;

unlock:
	mutex_unlock(&global_tunables_lock);
	if (!policy->fast_switch_enabled && z_policy->thread) {
		kthread_stop(z_policy->thread);
		mutex_destroy(&z_policy->work_lock);
	}
free_z_policy:
	kfree(z_policy);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	return ret;
}

static void zenith_exit(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	struct zenith_tunables *tunables = z_policy->tunables;

	/* Remove from the shared tunables' policy_list first so a concurrent
	 * sysfs store (e.g. auto_tune=1) can no longer iterate this policy
	 * and re-schedule at_work against it. Only after the list unlink
	 * is it safe to cancel the delayed work and free z_policy.
	 */
	mutex_lock(&global_tunables_lock);
	if (!gov_attr_set_put(&tunables->attr_set, &z_policy->tunables_hook))
		global_tunables = NULL;
	mutex_unlock(&global_tunables_lock);

	cancel_delayed_work_sync(&z_policy->at_work);

	if (!policy->fast_switch_enabled) {
		kthread_flush_worker(&z_policy->worker);
		kthread_stop(z_policy->thread);
		mutex_destroy(&z_policy->work_lock);
	}

	/*
	 * Publish the NULL so every subsequent vendor-hook probe
	 * (vh_arch_set_freq_scale, vh_cpu_idle_*, vh_setscheduler_uclamp,
	 * vh_freq_qos_update_request, vh_sched_move_task,
	 * vh_scheduler_tick) sees it and bails out immediately.
	 *
	 * The probes are registered at module level (zenith_gov_init)
	 * and fire from tracepoint callbacks which execute inside an
	 * RCU-sched read-side critical section (preempt_disable /
	 * rcu_read_lock_sched).  A probe that loaded z_policy before
	 * we NULLed governor_data is still referencing valid memory
	 * here -- the kfree has not happened yet.  synchronize_rcu()
	 * below guarantees that every such in-flight callback has
	 * returned before we free z_policy, closing the
	 * load-then-use-after-free window.
	 *
	 * Cost: one grace period (~few ms) on the teardown path,
	 * which only runs on governor switch -- not on the hot path.
	 */
	policy->governor_data = NULL;

	synchronize_rcu();

	cpufreq_disable_fast_switch(policy);
	kfree(z_policy);
}

static int zenith_start(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	unsigned int cpu;

	z_policy->last_freq_update_time = 0;
	z_policy->next_freq = 0;
	z_policy->work_in_progress = false;
	z_policy->limits_changed = false;
	z_policy->cached_raw_freq = 0;
	z_policy->need_freq_update = cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS);

	/* Cache the cluster-topology bit used by input_boost_big_only.
	 * SCHED_CAPACITY_SCALE is the normalised top capacity; any CPU
	 * in the policy hitting that value means this policy belongs
	 * to the system's highest-capacity cluster(s).
	 */
	z_policy->is_big_cluster = false;
	for_each_cpu(cpu, policy->cpus) {
		if (arch_scale_cpu_capacity(cpu) >= SCHED_CAPACITY_SCALE) {
			z_policy->is_big_cluster = true;
			break;
		}
	}
	zenith_update_cluster_rate_scale(z_policy);
	zenith_reset_local_actions(z_policy);

	/* Zero the uclamp cache so zenith_policy_uclamp_{min,max} refresh
	 * on the first eval after start rather than returning stale
	 * zeros cached from a previous attach cycle.
	 */
	z_policy->cached_uclamp_min = 0;
	z_policy->cached_uclamp_max = SCHED_CAPACITY_SCALE;
	z_policy->uclamp_cache_stamp_ns = 0;

	memset(z_policy->stats, 0, sizeof(z_policy->stats));
	memset(z_policy->dec_lat_buckets, 0,
	       sizeof(z_policy->dec_lat_buckets));
	z_policy->at_last_state =
		zenith_profile_to_at_state(z_policy->tunables->active_profile);
	z_policy->at_pending_state = z_policy->at_last_state;
	z_policy->at_pending_windows = 0;
	z_policy->at_cooldown_left = 0;
	z_policy->at_last_target = z_policy->tunables->active_profile;

	/*
	 * Two-pass attach to close the per-CPU init race.
	 *
	 * Pass 1 zeroes per-CPU state and publishes z_policy for every
	 * CPU in the policy *before* any update-util hook is registered.
	 * Pass 2 then registers the hooks.  Once the first hook is live,
	 * the scheduler is free to fire zenith_update_{single,shared} on
	 * any policy CPU; with z_policy already published on every
	 * sibling, the per-CPU iteration in zenith_update_shared cannot
	 * see a NULL z_policy.
	 *
	 * Previously this was a single loop, so the moment
	 * cpufreq_add_update_util_hook() ran for the first CPU the hook
	 * could fire on that CPU and iterate over a sibling whose
	 * z_policy had not yet been assigned -- causing a NULL deref at
	 * zenith_get_util+0x6c (read at NULL+offsetof(tunables)).  The
	 * race window was widened by any change that shifted CFS tick
	 * timing (e.g. removing BORE) and was observed at boot on MT6768
	 * (8-core, two clusters, shared policy per cluster).
	 */
	for_each_cpu(cpu, policy->cpus) {
		struct zenith_cpu *z_cpu = &per_cpu(zenith_cpu, cpu);

		memset(z_cpu, 0, sizeof(*z_cpu));
		z_cpu->cpu = cpu;
		/*
		 * Pair with READ_ONCE in zenith_update_shared.  Ensures
		 * the z_policy assignment is visible before any later
		 * cpufreq_add_update_util_hook() lets the scheduler
		 * observe the per-CPU state on another CPU.
		 */
		WRITE_ONCE(z_cpu->z_policy, z_policy);
	}

	for_each_cpu(cpu, policy->cpus) {
		struct zenith_cpu *z_cpu = &per_cpu(zenith_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &z_cpu->update_util,
			policy_is_shared(policy) ? zenith_update_shared : zenith_update_single);

		/* Wave B PMU IPC tracker.  Allocate per-CPU
		 * instructions / cycles perf_events.  Idempotent and
		 * failure-tolerant: errors leave the per-CPU pointers
		 * NULL and the floor never applies on this CPU.  See
		 * the comment block above ZENITH_DEFAULT_PMU_AWARE.
		 */
		(void)zenith_pmu_init_cpu(cpu);
	}

	/* Patch K: reset the game_perf_burst FSM and resolve the
	 * per-cluster thermal zone for the guardrail.
	 *
	 * State reset: zero the FSM scalars so a re-attach (governor
	 * switch / suspend resume) starts in IDLE rather than picking
	 * up a stale ARMED/COOLDOWN from the prior attach cycle.
	 *
	 * Zone resolution: ask the kernel thermal subsystem for
	 * "cpu<N>-thermal" where N == cpumask_first(policy->cpus).
	 * Zuma DT ships cpu0/cpu4/cpu6 zones (anchored at the first
	 * CPU of each cluster) so this lands on big-cluster zone for
	 * the policy that owns the big cluster.  IS_ERR / NULL means
	 * the zone was not registered yet (boot ordering: thermal
	 * subsystem registers after some cpufreq governors come up
	 * on certain SoCs) or this is a foreign SoC; the helper
	 * falls back to arch_scale_thermal_pressure-derived dC so
	 * the guardrail still works.  Patch M closes the boot-ordering
	 * gap by retrying resolution lazily from the hot-path helper
	 * (rate limited via gpb_tzd_retry_at_ns) so a thermal-core
	 * that registers after this point still binds eventually.
	 */
	z_policy->gpb_state = ZENITH_GPB_STATE_IDLE;
	z_policy->gpb_state_entry_ns = 0;
	z_policy->gpb_b_arm_first_seen_ns = 0;
	z_policy->gpb_b_disarm_first_seen_ns = 0;
	z_policy->gpb_tzd_retry_at_ns = 0;
	z_policy->gpb_arm_count = 0;
	z_policy->gpb_disarm_count = 0;
	z_policy->gpb_idle_count = 0;
	z_policy->gpb_last_disarm_reason = ZENITH_GPB_DISARM_NONE;
	{
		char zone_name[16];
		struct thermal_zone_device *tzd;
		unsigned int first_cpu = cpumask_first(policy->cpus);

		scnprintf(zone_name, sizeof(zone_name), "cpu%u-thermal",
			  first_cpu);
		tzd = thermal_zone_get_zone_by_name(zone_name);
		if (IS_ERR(tzd))
			tzd = NULL;
		z_policy->gpb_tzd = tzd;
		if (!tzd)
			z_policy->gpb_tzd_retry_at_ns =
				ktime_get_ns() +
				ZENITH_GPB_TZD_RETRY_INTERVAL_NS;
	}

	/* Arm the auto-tune classifier when the tunable is enabled.  The
	 * default tunables_init() sets auto_tune=1, but the only place
	 * that schedules at_work is auto_tune_store() on a 0->1
	 * transition.  Without this hook, the classifier silently never
	 * runs on stock defaults; userspace has to write 0 then 1 to
	 * arm it.  Mirror the body of the val=1 branch in
	 * auto_tune_store() so start-time and runtime behaviour agree.
	 */
	if (z_policy->tunables->auto_tune) {
		zenith_at_v_reset_window(z_policy);
		z_policy->at_cooldown_left = 0;
		schedule_delayed_work(&z_policy->at_work,
			msecs_to_jiffies(ZENITH_AUTO_TUNE_PERIOD_MS));
	}

	return 0;
}

static void zenith_stop(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus) {
		cpufreq_remove_update_util_hook(cpu);
		/* Wave B PMU IPC tracker.  Release per-CPU perf_events
		 * before the per-CPU update hook is fully removed so
		 * any in-flight sample_cpu() (which uses
		 * smp_call_function_single() under the hood) completes
		 * against valid pointers.
		 */
		zenith_pmu_exit_cpu(cpu);
	}
	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&z_policy->irq_work);
		kthread_cancel_work_sync(&z_policy->work);
	}
}

static void zenith_limits(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&z_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&z_policy->work_lock);
	}
	/* Release-store the flag so the subsequent acquire-load in
	 * zenith_should_update_freq() observes every write to
	 * policy->{min,max} that cpufreq_policy_apply_limits() made
	 * above.  smp_wmb() previously used here is a write-write
	 * barrier that does NOT establish a happens-before edge with
	 * the read side; release/acquire is the documented idiom.
	 */
	smp_store_release(&z_policy->limits_changed, true);
}

static struct cpufreq_governor zenith_gov = {
	.name       = "zenith",
	.init       = zenith_init,
	.exit       = zenith_exit,
	.start      = zenith_start,
	.stop       = zenith_stop,
	.limits     = zenith_limits,
	.owner      = THIS_MODULE,
	.flags      = CPUFREQ_GOV_DYNAMIC_SWITCHING,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ZENITH
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &zenith_gov;
}
#endif

/************************ Input Boost ***********************/

static void zenith_input_event(struct input_handle *handle, unsigned int type,
			       unsigned int code, int value)
{
	unsigned int active = READ_ONCE(zenith_input_boost_active_ms);
	u64 now_ns, last_ns, deadline;
	unsigned int effective_ms;
	bool was_quiet = false;
	unsigned int gap_ms = 0;
	unsigned int source = 0;

	if (type != EV_KEY && type != EV_ABS && type != EV_REL)
		return;

	/* Always bump the auto-tune counter so a policy that enables
	 * auto_tune mid-session has recent data. Cheap atomic inc.
	 */
	atomic64_inc(&zenith_auto_input_events);
	atomic64_inc(&zenith_in_events_total);

	if (!active) {
		atomic64_inc(&zenith_in_boosts_skipped_disabled);
		return;
	}

	now_ns = ktime_get_ns();
	last_ns = (u64)atomic64_read(&zenith_input_last_event_ns);
	atomic64_set(&zenith_input_last_event_ns, now_ns);

	/* Quiet-period extension.  When the gap since the previous
	 * event exceeds ZENITH_INPUT_QUIET_THRESHOLD_MS (or this is the
	 * very first event after boot, last_ns == 0), widen the
	 * full-pin window by ZENITH_INPUT_QUIET_BOOST_MULT_PCT and clip
	 * at ZENITH_INPUT_QUIET_BOOST_MAX_MS.  Sustained-interaction
	 * events (gap < threshold) keep the original active duration.
	 */
	effective_ms = active;
	if (!last_ns ||
	    now_ns - last_ns >=
	    (u64)ZENITH_INPUT_QUIET_THRESHOLD_MS * NSEC_PER_MSEC) {
		unsigned int extended = (active *
					 ZENITH_INPUT_QUIET_BOOST_MULT_PCT) /
					100;

		if (extended > ZENITH_INPUT_QUIET_BOOST_MAX_MS)
			extended = ZENITH_INPUT_QUIET_BOOST_MAX_MS;
		if (extended > effective_ms)
			effective_ms = extended;
		was_quiet = true;
	}

	/* Touchdown detection (Patch C).  Only the EV_KEY/BTN_TOUCH
	 * press counts as a touchdown.  Coordinate-stream EV_ABS and
	 * BTN_TOUCH release (value == 0) take the unmodified path.
	 * The extra is added on top of any quiet-period extension --
	 * a touchdown after a long quiet gap gets both bonuses, which
	 * is exactly what the user feels (cold start of a gesture).
	 */
	if (type == EV_KEY && code == BTN_TOUCH && value == 1) {
		unsigned int extra =
			READ_ONCE(zenith_input_boost_touchdown_extra_ms_cache);

		if (extra) {
			u64 widened64 = (u64)effective_ms + extra;

			if (widened64 > U32_MAX)
				widened64 = U32_MAX;
			effective_ms = (unsigned int)widened64;
		}
	}

	deadline = now_ns + (u64)effective_ms * NSEC_PER_MSEC;
	atomic64_set(&zenith_input_boost_until_ns, deadline);
	atomic64_inc(&zenith_in_boosts_armed);
	if (effective_ms > active)
		atomic64_inc(&zenith_in_boosts_quiet_extended);

	/* Observability: emit a tracepoint capturing the boost arming
	 * decision.  Default-disabled; consumers (testers /
	 * developers) opt in via
	 *
	 *   echo 1 > /sys/kernel/debug/tracing/events/cpufreq_zenith/zenith_input_boost/enable
	 *
	 * The tracepoint is gated by trace_zenith_input_boost_enabled()
	 * so the cost in the disabled case is a single conditional
	 * branch on a static-key.  All emit-side computations
	 * (gap_ms, source) are guarded by the same gate so they
	 * cannot show up in the hot path when nobody is tracing.
	 */
	if (trace_zenith_input_boost_enabled()) {
		u64 gap_ns;

		if (last_ns && now_ns > last_ns)
			gap_ns = now_ns - last_ns;
		else
			gap_ns = 0;
		if (gap_ns) {
			u64 gap_ms_u64 = div_u64(gap_ns, NSEC_PER_MSEC);

			gap_ms = gap_ms_u64 > U32_MAX ? U32_MAX :
				 (unsigned int)gap_ms_u64;
		}

		switch (type) {
		case EV_ABS:
			source = 1; /* touchscreen */
			break;
		case EV_KEY:
			source = 2; /* key */
			break;
		default:
			source = 0; /* generic / EV_REL et al */
			break;
		}

		trace_zenith_input_boost(effective_ms, active,
					 effective_ms > active,
					 was_quiet, gap_ms, source);
	}
}

static int zenith_input_connect(struct input_handler *handler,
				struct input_dev *dev,
				const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "zenith";

	ret = input_register_handle(handle);
	if (ret)
		goto err_free;

	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void zenith_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id zenith_input_ids[] = {
	/* Multitouch screens */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
				BIT_MASK(ABS_MT_POSITION_X) },
	},
	/* Touchpads */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] = BIT_MASK(ABS_X) },
	},
	/* Keyboards */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler zenith_input_handler = {
	.event		= zenith_input_event,
	.connect	= zenith_input_connect,
	.disconnect	= zenith_input_disconnect,
	.name		= "zenith",
	.id_table	= zenith_input_ids,
};

/************************ FB blank notifier (screen_auto) ********************/

/* Common back-end shared by every panel-event source.
 *
 * Both the legacy fb_notifier callback and the optional
 * drm_panel_notifier callback funnel here so the screen_state write
 * happens in exactly one place.  Splitting them into a helper also
 * makes it cheap for vendor / out-of-tree drivers to deliver panel
 * events directly without registering a notifier (e.g. a vendor
 * mode-set handler can call this from its own ioctl path; the
 * function has no module-level dependencies).
 *
 * __maybe_unused is required because the only in-tree callers live
 * inside CONFIG_FB_NOTIFY and CONFIG_DRM_PANEL_NOTIFY blocks; when
 * both are disabled the function has no in-tree caller and a -Werror
 * build would otherwise fail with -Wunused-function.  Out-of-tree
 * consumers (vendor mode-set / panel ioctl handlers) still get the
 * helper they need.
 */
static void __maybe_unused
zenith_panel_blank_event(int blank, unsigned int unblank_value)
{
	unsigned int new_state = (blank == (int)unblank_value) ? 1 : 0;

	/* All zenith policies share one global_tunables (per-cluster
	 * clones hold a reference to the same struct), so a single write
	 * propagates everywhere.
	 */
	mutex_lock(&global_tunables_lock);
	if (global_tunables && global_tunables->screen_auto)
		WRITE_ONCE(global_tunables->screen_state, new_state);
	mutex_unlock(&global_tunables_lock);
}

#ifdef CONFIG_FB_NOTIFY
static int zenith_fb_notifier_cb(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct fb_event *evdata = data;
	int blank;

	/* FB_EVENT_BLANK is the only blank event defined in
	 * android-common-5.10. Some older trees also ship
	 * FB_EARLY_EVENT_BLANK, but this one does not — including
	 * the symbol there breaks the build when CONFIG_FB_NOTIFY=y.
	 */
	if (action != FB_EVENT_BLANK)
		return NOTIFY_OK;
	if (!evdata || !evdata->data)
		return NOTIFY_OK;

	blank = *(int *)evdata->data;
	zenith_panel_blank_event(blank, FB_BLANK_UNBLANK);

	return NOTIFY_OK;
}

static struct notifier_block zenith_fb_notifier = {
	.notifier_call	= zenith_fb_notifier_cb,
	.priority	= 0,
};
#endif /* CONFIG_FB_NOTIFY */

#ifdef CONFIG_DRM_PANEL_NOTIFY
/* drm_panel_notifier callback.
 *
 * Called by the vendor drm panel notifier chain on display blank
 * transitions.  The vendor convention (Qualcomm and most adopters) is
 * to deliver a struct drm_panel_notifier whose .data field points to
 * an int holding DRM_PANEL_BLANK_UNBLANK, DRM_PANEL_BLANK_POWERDOWN
 * or DRM_PANEL_BLANK_LP.  Treat anything that is not UNBLANK as a
 * powered-off panel so screen_state goes to 0.
 */
static int zenith_drm_panel_notifier_cb(struct notifier_block *nb,
					unsigned long action, void *data)
{
	struct drm_panel_notifier *evdata = data;
	int blank;

	if (action != DRM_PANEL_EVENT_BLANK)
		return NOTIFY_OK;
	if (!evdata || !evdata->data)
		return NOTIFY_OK;

	blank = *(int *)evdata->data;
	zenith_panel_blank_event(blank, DRM_PANEL_BLANK_UNBLANK);

	return NOTIFY_OK;
}

static struct notifier_block zenith_drm_notifier = {
	.notifier_call	= zenith_drm_panel_notifier_cb,
	.priority	= 0,
};
#endif /* CONFIG_DRM_PANEL_NOTIFY */

/* Patch 1.9 fg-transition pulse: sched_wakeup_new tracepoint
 * probe.  Fires once per fork(), the very first time the new
 * task is woken (wake_up_new_task -> trace_sched_wakeup_new).
 *
 * Safety / context:
 *   - The probe runs in arbitrary scheduler context with the
 *     rq lock potentially held.  No sleeping primitives.
 *   - cpufreq_cpu_get_raw() is a per_cpu pointer load with a
 *     cpumask_test_cpu() check; no locks, no RCU writes.
 *   - policy->governor_data is a regular pointer and is set in
 *     zenith_start() / cleared in zenith_stop().  We read it
 *     with READ_ONCE; if zenith_stop() concurrently NULLs it
 *     after our load, the worst case is a write into a struct
 *     about to be freed -- but cpufreq_register_governor's
 *     teardown path is synchronous with respect to ongoing
 *     governor_data accesses (unregister_governor blocks until
 *     all in-flight callbacks complete, which is symmetric for
 *     the tracepoint hook because we unregister the probe at
 *     module_exit / on the cpufreq_register_governor failure
 *     rollback path).
 *
 * Foreground proxy: uclamp_eff_value(p, UCLAMP_MIN) > 0.
 *   - On Android 12 the top-app cgroup sets a non-zero
 *     uclamp.min on the cgroup itself; the freshly-forked task
 *     inherits that effective value at fork time.
 *   - On a kernel without CONFIG_UCLAMP_TASK the inline returns
 *     0 unconditionally and the probe degenerates to a per-fork
 *     no-op (one branch, no work).
 */
static void zenith_probe_wakeup_new(void *data, struct task_struct *p)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned int cpu;
	unsigned int pulse_ms;

	if (unlikely(!p))
		return;
	if (!uclamp_eff_value(p, UCLAMP_MIN))
		return;

	cpu = task_cpu(p);
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t)
		return;
	pulse_ms = READ_ONCE(t->fg_transition_pulse_ms);
	if (!pulse_ms)
		return;

	WRITE_ONCE(z_policy->fg_transition_pulse_until_ns,
		   ktime_get_ns() +
		   (u64)pulse_ms * NSEC_PER_MSEC);
}

/* Patch B9-1: android_vh_arch_set_freq_scale observer.
 *
 * The hook fires from arch_set_freq_scale() in drivers/base/
 * arch_topology.c whenever the scheduler caches a new per-cluster
 * frequency-scale value (used downstream for capacity_orig_of() /
 * cpu_util_*() accounting).  The signal is unique compared to the
 * tracepoints zenith already consumes in two ways:
 *
 *   1. It fires after the freq write has *taken effect*, not after
 *      zenith decided to write it.  On platforms that route the
 *      actual freq change through firmware / SCMI / a separate fast
 *      switch, drift between decision and realisation is real.
 *   2. It also fires for clusters zenith does *not* drive (e.g. on
 *      a hetero SoC where the BIG cluster is on schedutil and the
 *      LITTLE on zenith).  This gives zenith cross-cluster
 *      activity awareness without coupling to either governor's
 *      internal state.
 *
 * Use is opt-in via tunables->vh_arch_freq_scale_enable (default
 * 0).  When the gate is off the probe is a single READ_ONCE plus a
 * branch -- the cost on hot platforms (where this fires per
 * fast-switch) is dominated by the policy lookup.  When on:
 *
 *   - cache the realised scale in z_policy->vh_arch_freq_scale_-
 *     last (WRITE_ONCE; readers see torn-write-safe values)
 *   - if the scale jumped by >= ZENITH_VH_ARCH_FREQ_SCALE_STEP
 *     compared to the prior cached value (~5%% of SCHED_CAPACITY_-
 *     SCALE; filters governor-noise re-evaluations of the same
 *     OPP), arm the peer cluster's peer_ramp window so a peer
 *     governor's lift pre-warms us before the next eval window.
 *
 * Concurrency:
 *   - The probe runs in arbitrary scheduler context.  No sleeping
 *     primitives.  cpufreq_cpu_get_raw() is per_cpu pointer load
 *     plus a cpumask_test_cpu() check; no locks, no RCU writes.
 *   - z_policy->governor_data is the same READ_ONCE pattern the
 *     existing zenith_probe_wakeup_new() uses (see comment block
 *     above that function for the unregister-vs-free ordering
 *     argument; identical reasoning applies here because we
 *     register / unregister via the same trace-point lifecycle in
 *     zenith_gov_init()).
 *   - zenith_peer_ramp_arm() takes no locks; it reads the
 *     tunables window via READ_ONCE and writes a static atomic64
 *     via atomic64_set.  Safe to call from this context.
 */
static void
zenith_probe_arch_set_freq_scale(void *data, const struct cpumask *cpus,
				 unsigned long freq, unsigned long max,
				 unsigned long *scale)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned long new_scale;
	unsigned long prev_scale;
	unsigned int cpu;

	if (!cpus || !max)
		return;
	cpu = cpumask_first(cpus);
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_arch_freq_scale_enable))
		return;

	/* Prefer the scheduler's own normalised value when available;
	 * fall back to (freq / max) * SCHED_CAPACITY_SCALE if the
	 * caller didn't supply a destination pointer (defensive --
	 * the in-tree caller always passes one).
	 */
	if (scale)
		new_scale = *scale;
	else
		new_scale = (freq * SCHED_CAPACITY_SCALE) / max;

	prev_scale = READ_ONCE(z_policy->vh_arch_freq_scale_last);
	WRITE_ONCE(z_policy->vh_arch_freq_scale_last, new_scale);

	if (new_scale > prev_scale &&
	    new_scale - prev_scale >= ZENITH_VH_ARCH_FREQ_SCALE_STEP)
		zenith_peer_ramp_arm(z_policy, ktime_get_ns());
}

/* Patch B9-2: android_vh_setscheduler_uclamp observer.
 *
 * Fires from sched_setattr() / sched_setscheduler() / set_user_-
 * uclamp() when userspace assigns SCHED_FLAG_KEEP_PARAMS |
 * SCHED_FLAG_UTIL_CLAMP for a task.  Android Dynamic Performance
 * Framework (ADPF) uses this path heavily: a foreground game raises
 * uclamp_min on its render-critical thread to express "this thread
 * needs more headroom" which the cpufreq governor is supposed to
 * respect.  Schedutil reads task uclamp at every eval and reacts;
 * zenith reads task uclamp at every eval too -- but the *peer*
 * cluster only sees the raise once PELT propagation pushes the
 * clamped task's util upwards, which on a hot game thread can take
 * 4..32 ms.
 *
 * This probe gives us a synchronous notification: the moment
 * userspace writes the new uclamp_min, we look up the task's
 * current CPU's policy and arm peer_ramp on its peer cluster.  No
 * PELT lag.
 *
 * Use is opt-in via tunables->vh_uclamp_observer_enable (default
 * 0).  When the gate is off the probe is a single READ_ONCE plus a
 * branch.  The probe also short-circuits on:
 *
 *   - clamp_id != UCLAMP_MIN (uclamp_max raises do not justify a
 *     peer-cluster arm; they only cap the task's own cluster).
 *   - value == 0 (a clear, not a raise).
 *   - tsk == NULL or task_cpu(tsk) out of range (defensive).
 *
 * Concurrency: same reasoning as zenith_probe_arch_set_freq_scale.
 * Hook fires in arbitrary scheduler context; cpufreq_cpu_get_raw +
 * READ_ONCE on governor_data is the established no-lock pattern;
 * zenith_peer_ramp_arm is lock-free.
 */
static void
zenith_probe_setscheduler_uclamp(void *data, struct task_struct *tsk,
				 int clamp_id, unsigned int value)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned int cpu;

	if (!tsk || clamp_id != UCLAMP_MIN || !value)
		return;
	cpu = task_cpu(tsk);
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_uclamp_observer_enable))
		return;

	zenith_peer_ramp_arm(z_policy, ktime_get_ns());
}

/* Patch B9-3: android_vh_cpu_idle_enter probe.  Stamps a per-CPU
 * ktime_get_ns() timestamp on the zenith_cpu container of the CPU
 * going idle, gated by the master vh_cpu_idle_enable tunable.  Read-
 * only observer: the cpuidle hook permits mutating *state but we
 * leave it untouched -- cpuidle's own state selection is not our
 * concern here.
 *
 * Hook fires in regular kernel context on the local CPU, before
 * rcu_idle_enter().  cpufreq_cpu_get_raw + READ_ONCE on
 * governor_data is the established no-lock pattern (same as B9-1
 * and B9-2 above).  No governor lock is taken; the per-cpu
 * timestamp is naturally single-writer (only the local CPU enters
 * idle on itself) so no atomicity constraints beyond WRITE_ONCE.
 */
static void
zenith_probe_cpu_idle_enter(void *data, int *state,
			    struct cpuidle_device *dev)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	struct zenith_cpu *z_cpu;
	unsigned int cpu;

	if (!dev)
		return;
	cpu = (unsigned int)dev->cpu;
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_cpu_idle_enable))
		return;

	z_cpu = &per_cpu(zenith_cpu, cpu);
	WRITE_ONCE(z_cpu->vh_cpu_idle_last_enter_ns, ktime_get_ns());
}

/* Patch B9-3: android_vh_cpu_idle_exit probe.  Computes residency
 * (now_ns - per_cpu enter_ns) and stamps the per-policy aggregate
 * vh_cpu_idle_last_residency_ns, which the cluster_wake_pulse arm
 * site in zenith_get_next_freq() uses to suppress wasteful pulse
 * floors after a deep idle.  enter_ns == 0 means "no enter
 * observed" (e.g. enter probe fired before vh_cpu_idle_enable was
 * flipped on, or this CPU has not entered cpuidle since boot); the
 * residency stamp is skipped in that case.  ktime monotonicity
 * guard (now_ns <= enter_ns) preserves the same skip if the clock
 * source went backwards across the idle period.
 *
 * Last-writer-wins across CPUs in the cluster is acceptable: the
 * cwp gate cares about whether *some* CPU in the cluster recently
 * had a deep idle, and the next exit on any CPU resets the field.
 *
 * Same lock-free / no-governor-lock contract as the enter probe.
 */
static void
zenith_probe_cpu_idle_exit(void *data, int state,
			   struct cpuidle_device *dev)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	struct zenith_cpu *z_cpu;
	u64 enter_ns;
	u64 now_ns;
	unsigned int cpu;

	if (!dev)
		return;
	cpu = (unsigned int)dev->cpu;
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_cpu_idle_enable))
		return;

	z_cpu = &per_cpu(zenith_cpu, cpu);
	enter_ns = READ_ONCE(z_cpu->vh_cpu_idle_last_enter_ns);
	if (!enter_ns)
		return;
	now_ns = ktime_get_ns();
	if (now_ns <= enter_ns)
		return;
	WRITE_ONCE(z_policy->vh_cpu_idle_last_residency_ns,
		   now_ns - enter_ns);
}

/* Patch B9-3+: android_vh_freq_qos_update_request observer.
 *
 * Fires from kernel/power/qos.c::freq_qos_update_request() before
 * freq_qos_apply() so the call sees the new requested value but the
 * aggregated freq_constraints have not yet rolled forward.  Used by
 * thermal manager / battery saver / ADPF / vendor power HAL to drive
 * cpufreq min/max constraints; we only consume FREQ_QOS_MIN raises
 * here -- a vendor module asking for sustained high-min freq is the
 * deliberate "I want headroom" signal we want the AUTO selector to
 * pivot on.
 *
 * Lookup path: req->qos is a `struct freq_constraints *` which may
 * be embedded in a cpufreq_policy (the cpufreq freq-QoS path) or in
 * a per-device dev_pm_qos block.  We walk possible CPUs and match
 * by &policy->constraints; non-cpufreq freq_qos requests yield no
 * match and are silently ignored.  Bounded by num_possible_cpus();
 * fires only on QoS update so cost is fine.
 *
 * Pressure stamp: a hit at or above ZENITH_VH_FREQ_QOS_MIN_PCT of
 * cpuinfo.max_freq sets vh_freq_qos_pressure_until_ns to now +
 * ZENITH_VH_FREQ_QOS_WINDOW_MS so the AUTO selector's consumer
 * (zenith_auto_classify) sees the pressure for the next 2 s.
 *
 * Concurrency: same lock-free contract as B9-1 / B9-2 / B9-3.  Hook
 * fires in process / softirq context (preemptible at the qos.c call
 * site, no rq lock).  cpufreq_cpu_get_raw + READ_ONCE on
 * governor_data is the established pattern; the timestamp write is
 * atomic64_set so no governor lock is required against the
 * auto-eval consumer's atomic64_read.
 */
static void
zenith_probe_freq_qos_update_request(void *data,
				     struct freq_qos_request *req,
				     int value)
{
	struct cpufreq_policy *policy = NULL;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned int max_freq;
	unsigned int thresh;
	unsigned int cpu;

	if (!req || req->type != FREQ_QOS_MIN || value <= 0)
		return;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *p = cpufreq_cpu_get_raw(cpu);

		if (p && &p->constraints == req->qos) {
			policy = p;
			break;
		}
	}
	if (!policy || policy->governor != &zenith_gov)
		return;

	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_freq_qos_enable))
		return;

	max_freq = policy->cpuinfo.max_freq;
	if (!max_freq)
		return;

	thresh = (max_freq * ZENITH_VH_FREQ_QOS_MIN_PCT) / 100;
	if ((unsigned int)value < thresh)
		return;

	atomic64_set(&t->vh_freq_qos_pressure_until_ns,
		     ktime_get_ns() +
		     (u64)ZENITH_VH_FREQ_QOS_WINDOW_MS * NSEC_PER_MSEC);
}

/* Patch B9-5: android_vh_sched_move_task probe.  Stamps a per-policy
 * jiffies timestamp on z_policy->vh_sched_move_task_last_jiffies on
 * every cgroup move that lands a task on a CPU belonging to a
 * zenith-driven policy, gated by tunables->vh_sched_move_task_enable.
 *
 * Read-only observer: the hook permits inspection but not mutation
 * of the task; we only stamp the timestamp.  The tracepoint fires
 * from sched_move_task() in kernel/sched/core.c, called by the
 * cgroup attach / migration paths -- preemptible kernel context, no
 * rq lock held at the trace-point.
 *
 * Concurrency: same lock-free contract as B9-1 / B9-2 / B9-3 /
 * B9-3+.  cpufreq_cpu_get_raw + READ_ONCE on governor_data is the
 * established pattern; the timestamp write is WRITE_ONCE on a
 * per-policy unsigned long.  Multiple concurrent moves landing on
 * different CPUs of the same policy are last-writer-wins, which is
 * acceptable: any consumer only cares about the recency of the
 * most recent move, and a torn jiffies write would only delay the
 * stamp by one move.  No governor lock is taken.
 *
 * Filtering: a NULL tsk is impossible at the trace site (the kernel
 * always passes a real task to sched_move_task) but we defensively
 * check anyway so the probe is robust to future refactors.  Tasks
 * whose task_cpu() is not in a zenith-driven policy are silently
 * dropped, mirroring the B9-2 / B9-3+ behaviour.
 */
static void
zenith_probe_sched_move_task(void *data, struct task_struct *tsk)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned int cpu;

	if (!tsk)
		return;
	cpu = task_cpu(tsk);
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_sched_move_task_enable))
		return;

	WRITE_ONCE(z_policy->vh_sched_move_task_last_jiffies, jiffies);
}

/* Patch B9-4: android_vh_scheduler_tick probe.  Stamps a per-CPU
 * ktime_get_ns() timestamp on z_cpu->vh_scheduler_tick_last_ns and
 * bumps z_cpu->vh_scheduler_tick_count once on every scheduler-tick
 * fire on a CPU belonging to a zenith-driven policy, gated by
 * tunables->vh_scheduler_tick_enable.
 *
 * Hot-path contract: this probe runs from the scheduler tick path
 * (HZ * num_present_cpus calls/s, ~2000/s on a typical Android
 * arm64 board) -- the strictest hot path of any zenith vendor-hook
 * observer.  Mandatory contract: cpufreq_cpu_get_raw + READ_ONCE
 * on governor_data and the tunable gate, single ktime_get_ns(),
 * two WRITE_ONCEs to per-CPU fields, no mutex, no spinlock, no
 * atomic_*.
 *
 * Storage: per-CPU (struct zenith_cpu) rather than per-policy
 * because the tick fires on the local CPU and a remote-CPU stamp
 * would mis-attribute the residency.  Single writer per CPU (the
 * local CPU's tick handler) so the WRITE_ONCEs do not race; remote
 * readers use READ_ONCE.
 *
 * Filtering: the hook fires under preempt_disable from
 * scheduler_tick() in kernel/sched/core.c, after rq_unlock and
 * trigger_load_balance.  rq is guaranteed non-NULL by the trace
 * point but we defensively check anyway.  cpu_of(rq) is the local
 * CPU; we range-check vs nr_cpu_ids before indexing the per-CPU
 * area to be robust against a future cpu_of() that yields a stale
 * value during cpu hotplug teardown.
 */
static void
zenith_probe_scheduler_tick(void *data, struct rq *rq)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	struct zenith_cpu *z_cpu;
	unsigned int cpu;
	u64 now;

	if (!rq)
		return;
	cpu = cpu_of(rq);
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_scheduler_tick_enable))
		return;

	now = ktime_get_ns();
	z_cpu = &per_cpu(zenith_cpu, cpu);
	WRITE_ONCE(z_cpu->vh_scheduler_tick_last_ns, now);
	WRITE_ONCE(z_cpu->vh_scheduler_tick_count,
		   READ_ONCE(z_cpu->vh_scheduler_tick_count) + 1);
}

/*
 * Hikari wake-time hint receiver.  The callback runs in atomic
 * context (from atomic_notifier_call_chain) and must therefore
 * not sleep, not block, and must avoid touching state that
 * requires cpufreq's locks.
 *
 * The floor value itself is stored per-CPU on the Hikari side
 * and read out by zenith_get_next_freq() via
 * hikari_get_floor_khz().  The receiver here is intentionally
 * minimal: it validates the hint and ACKs.  The next natural
 * scheduler tick on the hint's target CPU (sub-millisecond on a
 * busy CPU, up to one tick on an idle one) will see the floor
 * via hikari_get_floor_khz() and apply it.
 *
 * We deliberately do NOT call cpufreq_update_util() from here.
 * That would either need the target CPU's rq lock (we may be on
 * a different CPU) or risk re-entering the governor with
 * unexpected locking.  The added latency from waiting one tick
 * is acceptable given the floor TTL is on the order of 50ms.
 */
static int zenith_hikari_freq_hint_cb(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	struct hikari_freq_hint *hint = data;
	struct zenith_cpu *z_cpu;
	struct zenith_policy *z_policy;

	if (event != HIKARI_NOTIFIER_WAKE_DEMAND)
		return NOTIFY_DONE;
	if (!hint)
		return NOTIFY_DONE;
	if (hint->cpu >= nr_cpu_ids)
		return NOTIFY_DONE;

	z_cpu = &per_cpu(zenith_cpu, hint->cpu);
	z_policy = READ_ONCE(z_cpu->z_policy);
	if (!z_policy)
		return NOTIFY_DONE;
	if (!z_policy->policy)
		return NOTIFY_DONE;

	return NOTIFY_OK;
}

static struct notifier_block zenith_hikari_nb = {
	.notifier_call = zenith_hikari_freq_hint_cb,
};

/* ------------------------------------------------------------ */
/* Selene game detection integration                            */
/* ------------------------------------------------------------ */
static int zenith_selene_nb_fn(struct notifier_block *nb,
                               unsigned long event, void *data)
{
	switch (event) {
	case SELENE_EVENT_GAME_START:
		WRITE_ONCE(zenith_game_auto_active_until_ns,
			   ktime_get_ns() + 30ULL * NSEC_PER_SEC);
		atomic_notifier_call_chain(&zenith_game_mode_nh, 1, NULL);
		break;
	case SELENE_EVENT_GAME_STOP:
		WRITE_ONCE(zenith_game_auto_active_until_ns, 0);
		atomic_notifier_call_chain(&zenith_game_mode_nh, 0, NULL);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block zenith_selene_nb = {
	.notifier_call = zenith_selene_nb_fn,
};

/*
 * Walk the policy's CPUs, take the max Hikari floor across them,
 * and clamp to the policy's [min..max] range.  Returns 0 if no
 * floor is currently published on any CPU in the policy.  Cheap
 * when Hikari is off (hikari_get_floor_khz early-returns 0 on a
 * single READ_ONCE inside hikari_enabled()).
 */
static unsigned int zenith_hikari_policy_floor(struct cpufreq_policy *policy)
{
	unsigned int floor = 0;
	unsigned int f;
	int cpu;

	if (!policy)
		return 0;

	for_each_cpu(cpu, policy->cpus) {
		f = hikari_get_floor_khz(cpu);
		if (f > floor)
			floor = f;
	}

	if (!floor)
		return 0;

	if (floor < policy->min)
		floor = policy->min;
	if (floor > policy->max)
		floor = policy->max;
	return floor;
}

static int __init zenith_gov_init(void)
{
	int ret;
	bool input_registered = false;
	bool fg_pulse_registered = false;
	bool vh_arch_freq_scale_registered = false;
	bool vh_uclamp_observer_registered = false;
	bool vh_cpu_idle_enter_registered = false;
	bool vh_cpu_idle_exit_registered = false;
	bool vh_freq_qos_registered = false;
	bool vh_sched_move_task_registered = false;
	bool vh_scheduler_tick_registered = false;
#ifdef CONFIG_FB_NOTIFY
	bool fb_registered = false;
#endif
#ifdef CONFIG_DRM_PANEL_NOTIFY
	bool drm_registered = false;
#endif

	/* Boot banner.  Printed once at governor init, picked up by
	 * `dmesg | grep -E 'Zenith :'`.  Pure ASCII for the latin art
	 * (so dmesg viewers that pick the East-Asian-Ambiguous wide-cell
	 * width for U+2665 etc. would still align, but we use only safe
	 * glyphs in the art panels); a handful of unambiguously wide CJK
	 * characters (頂点 / 光 / 霞 / 癒し) appear in the chapter
	 * titles and render the same in narrow and wide-cell viewers.
	 *
	 * Structure: an opening over the dark sky, then chapters for
	 * each of the four subsystems (Zenith / Hikari / Kasumi /
	 * Iyashi) with a big FIGlet per chapter, inter-chapter
	 * interludes, a final stacked-FIGlet roll, and a closing.
	 * Every line carries the "Zenith :" prefix so the whole
	 * multi-line banner survives a grep, and so the verify
	 * script's `grep -E 'Zenith :'` finds the banner intact.
	 *
	 * Small per-subsystem signatures are also emitted from
	 * hikari_init / kasumi_sysfs_init / iyashi_init with their own
	 * "Hikari :" / "Kasumi :" / "Iyashi :" prefixes, so each
	 * subsystem is individually grep-able.
	 */
	#ifdef CONFIG_ZENITH_DEBUG_MSG
	pr_info("Zenith: v4 hybrid cpufreq governor initialized (Zenith/Hikari/Kasumi/Iyashi)\n");
	#endif /* CONFIG_ZENITH_DEBUG_MSG */

	/* Allocate the initial RCU comm tables from the in-tree default
	 * arrays.  Failure here is non-fatal: zenith_policy_has_X()
	 * checks for a NULL table on the read side, so the awareness
	 * features simply skip the comm walk and fall through to the
	 * existing override paths (camera_active, render override, etc.)
	 * until userspace populates the tables via the sysfs nodes.
	 */
	rcu_assign_pointer(zenith_render_table,
		zenith_alloc_comm_table_from_defaults(zenith_render_comms,
			ARRAY_SIZE(zenith_render_comms)));
	rcu_assign_pointer(zenith_audio_table,
		zenith_alloc_comm_table_from_defaults(zenith_audio_comms,
			ARRAY_SIZE(zenith_audio_comms)));
	rcu_assign_pointer(zenith_camera_table,
		zenith_alloc_comm_table_from_defaults(zenith_camera_comms,
			ARRAY_SIZE(zenith_camera_comms)));
	rcu_assign_pointer(zenith_game_auto_table,
		zenith_alloc_comm_table_from_defaults(zenith_game_auto_comms,
			ARRAY_SIZE(zenith_game_auto_comms)));

	ret = input_register_handler(&zenith_input_handler);
	if (ret)
		pr_warn("Zenith: input handler register failed (%d), boost disabled\n",
			ret);
	else
		input_registered = true;

	/* Patch 1.9 fg-transition pulse: register the sched_-
	 * wakeup_new tracepoint probe.  Failure is non-fatal; the
	 * pulse just becomes a no-op (no deadline ever stamped).
	 */
	ret = register_trace_sched_wakeup_new(zenith_probe_wakeup_new, NULL);
	if (ret)
		pr_warn("Zenith: sched_wakeup_new probe register failed (%d), fg_transition_pulse disabled\n",
			ret);
	else
		fg_pulse_registered = true;

	/* Patch B9-1: register the android_vh_arch_set_freq_scale
	 * vendor-hook probe.  Failure is non-fatal; the freq-scale
	 * realisation observer simply remains silent and zenith falls
	 * back to its decision-time peer_ramp arming alone.  The
	 * tunables->vh_arch_freq_scale_enable gate is the runtime
	 * switch; this register call only makes the probe *available*
	 * for the gate to flip on.
	 */
	ret = register_trace_android_vh_arch_set_freq_scale(
		zenith_probe_arch_set_freq_scale, NULL);
	if (ret)
		pr_warn("Zenith: vh_arch_set_freq_scale probe register failed (%d), vh_arch_freq_scale_enable will be a no-op\n",
			ret);
	else
		vh_arch_freq_scale_registered = true;

	/* Patch B9-2: register the android_vh_setscheduler_uclamp
	 * vendor-hook probe.  Failure is non-fatal; the ADPF
	 * synchronous-arm path simply remains silent and zenith falls
	 * back to seeing the uclamp raise after PELT propagation
	 * (existing behaviour).  The tunables->vh_uclamp_observer_-
	 * enable gate is the runtime switch.
	 */
	ret = register_trace_android_vh_setscheduler_uclamp(
		zenith_probe_setscheduler_uclamp, NULL);
	if (ret)
		pr_warn("Zenith: vh_setscheduler_uclamp probe register failed (%d), vh_uclamp_observer_enable will be a no-op\n",
			ret);
	else
		vh_uclamp_observer_registered = true;

	/* Patch B9-3: register the android_vh_cpu_idle_enter / _exit
	 * vendor-hook probe pair.  Failure on either is non-fatal and
	 * independent: if only the enter probe registers, the exit
	 * probe will treat per-cpu enter_ns == 0 as "no enter
	 * observed" and skip the residency stamp; if only the exit
	 * probe registers, no enter_ns is ever stamped so residency
	 * stays 0 and the cwp gate never fires.  In both partial
	 * failure cases vh_cpu_idle_enable becomes effectively a
	 * no-op.  The tunables->vh_cpu_idle_enable gate is the
	 * runtime switch (default 0).
	 */
	ret = register_trace_android_vh_cpu_idle_enter(
		zenith_probe_cpu_idle_enter, NULL);
	if (ret)
		pr_warn("Zenith: vh_cpu_idle_enter probe register failed (%d), vh_cpu_idle_enable will be a no-op\n",
			ret);
	else
		vh_cpu_idle_enter_registered = true;

	ret = register_trace_android_vh_cpu_idle_exit(
		zenith_probe_cpu_idle_exit, NULL);
	if (ret)
		pr_warn("Zenith: vh_cpu_idle_exit probe register failed (%d), vh_cpu_idle_enable will be a no-op\n",
			ret);
	else
		vh_cpu_idle_exit_registered = true;

	/* Patch B9-3+: register the android_vh_freq_qos_update_request
	 * vendor-hook probe.  Failure is non-fatal; the freq-QoS
	 * pressure observer simply remains silent and the auto-selector
	 * falls back to its existing camera / input-recent / battery
	 * cascade alone.  The tunables->vh_freq_qos_enable gate is the
	 * runtime switch; this register call only makes the probe
	 * *available* for the gate to flip on.
	 */
	ret = register_trace_android_vh_freq_qos_update_request(
		zenith_probe_freq_qos_update_request, NULL);
	if (ret)
		pr_warn("Zenith: vh_freq_qos_update_request probe register failed (%d), vh_freq_qos_enable will be a no-op\n",
			ret);
	else
		vh_freq_qos_registered = true;

	/* Patch B9-5: register the android_vh_sched_move_task vendor-
	 * hook probe.  Failure is non-fatal; the cgroup-move observer
	 * simply remains silent and z_policy->vh_sched_move_task_-
	 * last_jiffies stays at 0 across the lifetime of the policy.
	 * The tunables->vh_sched_move_task_enable gate is the runtime
	 * switch; this register call only makes the probe *available*
	 * for the gate to flip on.
	 */
	ret = register_trace_android_vh_sched_move_task(
		zenith_probe_sched_move_task, NULL);
	if (ret)
		pr_warn("Zenith: vh_sched_move_task probe register failed (%d), vh_sched_move_task_enable will be a no-op\n",
			ret);
	else
		vh_sched_move_task_registered = true;

	/* Patch B9-4: register the android_vh_scheduler_tick vendor-
	 * hook probe.  Failure is non-fatal; the per-CPU tick observer
	 * simply remains silent and z_cpu->vh_scheduler_tick_last_ns /
	 * _count stay at 0 across the lifetime of the policy.  The
	 * tunables->vh_scheduler_tick_enable gate is the runtime
	 * switch; this register call only makes the probe *available*
	 * for the gate to flip on.
	 */
	ret = register_trace_android_vh_scheduler_tick(
		zenith_probe_scheduler_tick, NULL);
	if (ret)
		pr_warn("Zenith: vh_scheduler_tick probe register failed (%d), vh_scheduler_tick_enable will be a no-op\n",
			ret);
	else
		vh_scheduler_tick_registered = true;

	/* Panel-state delivery: register the drm_panel_notifier path first
	 * (preferred when available because the fb notifier chain is
	 * deprecated upstream and absent on most modern vendor builds), and
	 * fall back to fb_register_client() if drm is unavailable or its
	 * registration fails.  Registering both at once would cause every
	 * blank/unblank event to write screen_state twice, so the fb
	 * registration is skipped when drm is live.
	 */
#ifdef CONFIG_DRM_PANEL_NOTIFY
	ret = drm_panel_notifier_register(&zenith_drm_notifier);
	if (ret)
		pr_warn("Zenith: drm panel notifier register failed (%d), trying fb fallback\n",
			ret);
	else
		drm_registered = true;
#endif
#ifdef CONFIG_FB_NOTIFY
	if (
#ifdef CONFIG_DRM_PANEL_NOTIFY
	    !drm_registered &&
#endif
	    1) {
		ret = fb_register_client(&zenith_fb_notifier);
		if (ret)
			pr_warn("Zenith: fb notifier register failed (%d), screen_auto disabled\n",
				ret);
		else
			fb_registered = true;
	}
#endif
#if !defined(CONFIG_FB_NOTIFY) && !defined(CONFIG_DRM_PANEL_NOTIFY)
	/* screen_auto stores still accept 0/1 (the field is plain
	 * bookkeeping for userspace introspection) but no notifier
	 * will ever flip screen_state.  Print once at init so an
	 * operator wondering "why is screen_auto=1 not affecting the
	 * frequency" can grep dmesg and find the answer immediately
	 * rather than chasing the runtime path.
	 */
	pr_info("Zenith: CONFIG_FB_NOTIFY=n and CONFIG_DRM_PANEL_NOTIFY=n, screen_auto is bookkeeping-only (no panel events delivered)\n");
#endif
#ifdef CONFIG_DRM_PANEL_NOTIFY
	if (drm_registered)
		pr_info("Zenith: screen_auto wired through drm panel notifier\n");
#endif

	ret = cpufreq_register_governor(&zenith_gov);
	if (ret) {
		/* Roll back the input + fb / drm hooks we successfully
		 * registered above so that a probe failure here
		 * leaves no dangling notifier / handler bound to a
		 * governor that does not exist.  Previously the
		 * function returned the error and silently leaked
		 * both registrations; subsequent module-style
		 * insmod/rmmod cycles would double-register.
		 */
#ifdef CONFIG_DRM_PANEL_NOTIFY
		if (drm_registered)
			drm_panel_notifier_unregister(&zenith_drm_notifier);
#endif
#ifdef CONFIG_FB_NOTIFY
		if (fb_registered)
			fb_unregister_client(&zenith_fb_notifier);
#endif
		if (input_registered)
			input_unregister_handler(&zenith_input_handler);
		if (fg_pulse_registered)
			unregister_trace_sched_wakeup_new(
				zenith_probe_wakeup_new, NULL);
		if (vh_arch_freq_scale_registered)
			unregister_trace_android_vh_arch_set_freq_scale(
				zenith_probe_arch_set_freq_scale, NULL);
		if (vh_uclamp_observer_registered)
			unregister_trace_android_vh_setscheduler_uclamp(
				zenith_probe_setscheduler_uclamp, NULL);
		if (vh_cpu_idle_enter_registered)
			unregister_trace_android_vh_cpu_idle_enter(
				zenith_probe_cpu_idle_enter, NULL);
		if (vh_cpu_idle_exit_registered)
			unregister_trace_android_vh_cpu_idle_exit(
				zenith_probe_cpu_idle_exit, NULL);
		if (vh_freq_qos_registered)
			unregister_trace_android_vh_freq_qos_update_request(
				zenith_probe_freq_qos_update_request, NULL);
		if (vh_sched_move_task_registered)
			unregister_trace_android_vh_sched_move_task(
				zenith_probe_sched_move_task, NULL);
		if (vh_scheduler_tick_registered)
			unregister_trace_android_vh_scheduler_tick(
				zenith_probe_scheduler_tick, NULL);
		pr_err("Zenith: cpufreq_register_governor failed (%d)\n", ret);
		return ret;
	}

	/*
	 * Subscribe to Hikari's wake-time hint chain.  Failure here is
	 * non-fatal: Zenith works fine without Hikari hints, and the
	 * direct hikari_get_floor_khz() read in zenith_get_next_freq()
	 * is still functional with or without subscription.  Log the
	 * outcome for observability.
	 */
	{
		int hret = hikari_register_cpufreq_notifier(&zenith_hikari_nb);

		if (hret)
			pr_warn("Zenith: hikari notifier register failed (%d), continuing without subscription\n",
				hret);
		else
			pr_info("Zenith: subscribed to hikari wake-demand chain\n");
	}

	/*
	 * Subscribe to Selene\'s game detection chain.  On game start
	 * we set a 30-second game-mode auto latch so zenith\'s hot-path
	 * overlays (higher freq targets, lower rate limits) activate.
	 * On game stop the latch is cleared.
	 */
	{
		int sret = selene_register_notifier(&zenith_selene_nb);

		if (sret)
			pr_warn("Zenith: selene notifier register failed (%d), continuing without game detection\n",
				sret);
		else
			pr_info("Zenith: subscribed to selene game detection chain\n");
	}

	return 0;
}
fs_initcall(zenith_gov_init);
