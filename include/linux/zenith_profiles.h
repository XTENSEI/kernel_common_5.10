/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * include/linux/zenith_profiles.h - Profile ID definitions shared
 * between Zenith, Hikari, IYASHI, and Kasumi.
 *
 * The numeric values must match the ZENITH_PROFILE_* defines in
 * kernel/sched/cpufreq_zenith.c.  Adding or renumbering a profile
 * requires updating both locations.
 */
#ifndef _LINUX_ZENITH_PROFILES_H
#define _LINUX_ZENITH_PROFILES_H

#define ZENITH_PROFILE_CUSTOM		0
#define ZENITH_PROFILE_PERFORMANCE	1
#define ZENITH_PROFILE_BALANCED		2
#define ZENITH_PROFILE_BATTERY		3
#define ZENITH_PROFILE_LEGACY		4
#define ZENITH_PROFILE_GAMING		5
#define ZENITH_PROFILE_AUDIO		6
#define ZENITH_PROFILE_AUTO		7

/**
 * zenith_resolve_profile - map a raw profile ID to a concrete profile
 * @profile: raw profile number
 *
 * Meta-profiles (CUSTOM, LEGACY, AUTO) and out-of-range values
 * are resolved to BALANCED.  Concrete profiles are returned unchanged.
 *
 * Note: AUDIO (6) is NOT resolved here because Hikari treats it as
 * a concrete profile with specific values.  Subsystems that treat
 * AUDIO as a meta-profile (IYASHI, Kasumi) handle it naturally via
 * their own array bounds checks.
 */
static inline unsigned int zenith_resolve_profile(unsigned int profile)
{
	if (profile == ZENITH_PROFILE_CUSTOM ||
	    profile == ZENITH_PROFILE_LEGACY ||
	    profile >= ZENITH_PROFILE_AUTO)
		return ZENITH_PROFILE_BALANCED;
	return profile;
}

#endif /* _LINUX_ZENITH_PROFILES_H */
