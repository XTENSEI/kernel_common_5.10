/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public kernel API for the zenith cpufreq governor.
 *
 * Display-side hooks:
 *
 *   zenith_set_drm_vblank_us() publishes the active panel vblank
 *   period in microseconds (e.g. on a 60 -> 120 Hz mode switch).
 *
 *   zenith_drm_vblank_event() is called once per vblank from the
 *   display driver / drm-panel bridge so the governor can detect
 *   frame budget overruns and apply a recovery floor.  When the
 *   panel driver does not call this, the overrun-detection path
 *   is a no-op -- the rest of the governor behaves as before.
 *
 * Scheduler-side hooks:
 *
 *   zenith_signal_wake_demand() is called by the Hikari wake-
 *   latency vruntime shift (see Documentation/scheduler/sched-
 *   hikari.rst) when a task's wait-time EWMA saturates the shift
 *   cap on a CPU belonging to a zenith-governed policy.  Zenith
 *   arms a soft frequency floor at cluster_wake_pulse_floor_pct
 *   of policy->max for cluster_wake_pulse_ms, reusing the existing
 *   cluster_wake_pulse tier rather than introducing a new one.
 *   No-op when cluster_wake_pulse is disabled or when the CPU is
 *   not governed by zenith.
 *
 * All hooks stub out cleanly when zenith is not built so callers
 * can stay unconditional.
 */
#ifndef _LINUX_CPUFREQ_ZENITH_H
#define _LINUX_CPUFREQ_ZENITH_H

#include <linux/types.h>

#if IS_ENABLED(CONFIG_CPU_FREQ_GOV_ZENITH)
extern void zenith_set_drm_vblank_us(unsigned int us);
void zenith_drm_vblank_event(void);
void zenith_signal_wake_demand(int cpu);
#else
static inline void zenith_set_drm_vblank_us(unsigned int us) { }
static inline void zenith_drm_vblank_event(void) { }
static inline void zenith_signal_wake_demand(int cpu) { }
#endif

#endif /* _LINUX_CPUFREQ_ZENITH_H */
