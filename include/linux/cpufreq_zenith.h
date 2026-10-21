/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public kernel API for the zenith cpufreq governor.
 *
 * Two display-side hooks live here:
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
 * Both stub out cleanly when zenith is not built so callers can
 * stay unconditional.
 */
#ifndef _LINUX_CPUFREQ_ZENITH_H
#define _LINUX_CPUFREQ_ZENITH_H

#include <linux/types.h>
#include <linux/notifier.h>

#if IS_ENABLED(CONFIG_CPU_FREQ_GOV_ZENITH)
extern void zenith_set_drm_vblank_us(unsigned int us);
void zenith_drm_vblank_event(void);
void zenith_gpu_load_event(unsigned int gpu_load_pct);
void zenith_gpu_freq_event(unsigned int freq_pct);
bool zenith_is_game_mode_active(void);
int zenith_register_game_mode_notifier(struct notifier_block *nb);
int zenith_unregister_game_mode_notifier(struct notifier_block *nb);
#else
static inline void zenith_set_drm_vblank_us(unsigned int us) { }
static inline void zenith_drm_vblank_event(void) { }
static inline void zenith_gpu_load_event(unsigned int gpu_load_pct) { }
static inline void zenith_gpu_freq_event(unsigned int freq_pct) { }
static inline bool zenith_is_game_mode_active(void) { return false; }
static inline int zenith_register_game_mode_notifier(struct notifier_block *nb) { return 0; }
static inline int zenith_unregister_game_mode_notifier(struct notifier_block *nb) { return 0; }
#endif

#endif /* _LINUX_CPUFREQ_ZENITH_H */
