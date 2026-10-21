/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Iyashi (癒し) thermal performance floor -- XTENSEI
 *
 * Internal interface between the thermal core and iyashi.c.
 * iyashi_clamp_target() runs from thermal_cdev_update() right before
 * the cooling device's set_cur_state callback fires, so it gets a
 * final chance to substitute a more permissive target before the OPP
 * cap is applied.
 *
 * When CONFIG_IYASHI=n the hook compiles to a no-op and contributes
 * zero text/data to thermal_sys.o.
 */

#ifndef __IYASHI_H__
#define __IYASHI_H__

#include <linux/thermal.h>

#ifdef CONFIG_IYASHI
unsigned long iyashi_clamp_target(struct thermal_cooling_device *cdev,
				  unsigned long target);
void iyashi_apply_profile(unsigned int profile);
/*
 * Called from thermal_charger_guard.c to layer an external
 * suppression flag on top of the user's /sys/kernel/iyashi/enabled
 * knob.  Effective active state is (enabled && !charger_suppressed);
 * this does not modify the user-visible `enabled` value.
 */
void iyashi_set_charger_suppressed(bool suppressed);
#else
static inline unsigned long
iyashi_clamp_target(struct thermal_cooling_device *cdev,
		    unsigned long target)
{
	return target;
}
static inline void iyashi_apply_profile(unsigned int profile) { }
static inline void iyashi_set_charger_suppressed(bool suppressed) { }
#endif

#endif /* __IYASHI_H__ */
