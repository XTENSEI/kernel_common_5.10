// SPDX-License-Identifier: GPL-2.0
/*
 * Iyashi (癒し) thermal performance floor -- XTENSEI
 *
 * "Iyashi" -- the healing of unnecessary throttling.  Sits between the
 * thermal governor's chosen cooling target and the cooling device's
 * set_cur_state() callback.  When the system is far from any active
 * trip point, Iyashi clamps the cooling target so the CPU is held at
 * a configurable performance floor (default 90% of max OPP).  When
 * the closest bound trip is within near_limit_offset_c degrees,
 * Iyashi steps aside and lets the thermal framework throttle normally.
 *
 * Decision per thermal_cdev_update() call:
 *
 *   1. If iyashi_enabled == 0, passthrough.
 *   2. If cdev->type doesn't match the cdev filter (default:
 *      "thermal-cpufreq-" prefix), passthrough.  This keeps GPU and
 *      other cooling devices out of Iyashi's scope.
 *   3. Walk cdev->thermal_instances, read each bound zone's current
 *      temperature and its trip-point temperature, and compute the
 *      minimum (trip - current) headroom across all instances.
 *   4. If min_headroom_c < near_limit_offset_c, cooldown is imminent
 *      -- passthrough so the framework can throttle.
 *   5. Otherwise clamp the target to a floor_state so the CPU never
 *      drops below floor_pct of max OPP.
 *
 * Reads tz->temperature directly under the cdev->lock that
 * thermal_cdev_update() already holds.  Calls tz->ops->get_trip_temp
 * outside of tz->lock (the trip-temp callbacks are documented to be
 * lockless and side-effect free in the of_thermal backend, which is
 * what every platform driver we ship uses).
 *
 * All shared state is __read_mostly + READ_ONCE / WRITE_ONCE; the
 * hot path takes no locks.  The cdev_filter string is protected by
 * iyashi_filter_lock and snapshotted into a stack-local buffer.
 *
 * Disabled via static_branch when iyashi_enabled = 0, so the hook
 * is literally one branch on the cooling-update fast path while off.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/hikari.h>
#include <linux/zenith_profiles.h>
#include <linux/selene.h>
#include <linux/init.h>
#include <linux/jump_label.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/pm_qos.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>

#define CREATE_TRACE_POINTS
#include <trace/events/iyashi.h>
#undef CREATE_TRACE_POINTS

#include "iyashi.h"
#include "thermal_core.h"

/* --------------------------------------------------------------- *
 * Tunables (R/W from sysfs)                                       *
 * --------------------------------------------------------------- */

static unsigned int iyashi_enabled            __read_mostly = 1;
static unsigned int iyashi_floor_pct          __read_mostly = 90;
static unsigned int iyashi_near_limit_offset_c __read_mostly = 5;

/*
 * External suppression flag driven by thermal_charger_guard.c.  The
 * effective active state is (iyashi_enabled && !iyashi_charger_suppressed);
 * iyashi_recompute_active() flips the static branch to mirror it.  The
 * user-visible `enabled` sysfs node is left untouched by the charger
 * guard -- the suppression flag is exposed read-only as a sibling.
 */
static unsigned int iyashi_charger_suppressed __read_mostly;

/*
 * Optional frequency-units floor (only meaningful for cpufreq cooling).
 * 0 = off; the floor is taken from iyashi_floor_pct in cooling-state
 * units.  Non-zero = compute the deepest cooling state whose target
 * frequency is still >= (pct% of policy->cpuinfo.max_freq) and use the
 * MORE PERMISSIVE of {state-units floor, freq-units floor} -- i.e. the
 * one that keeps cpufreq running faster.
 *
 * Why both?  state-units floor_pct is well-defined for non-cpufreq
 * cooling devices (GPU, fan, etc.); min_freq_pct is only meaningful
 * for cpufreq cooling but is the knob users actually think in.
 */
static unsigned int iyashi_min_freq_pct       __read_mostly;
static unsigned int iyashi_enforce_min        __read_mostly;

/*
 * Hikari cross-link tunables.  When hikari_aware is non-zero AND
 * Hikari published a wake-demand event within the last
 * hikari_window_ms milliseconds, Iyashi temporarily raises the
 * effective floor_pct by hikari_boost_pct percentage points.
 *
 * Default: all off (hikari_aware=0).  PERFORMANCE/GAMING profiles
 * enable it via iyashi_apply_profile().
 */
static unsigned int iyashi_hikari_aware       __read_mostly;
static unsigned int iyashi_hikari_window_ms   __read_mostly = 50;
static unsigned int iyashi_hikari_boost_pct   __read_mostly = 5;

/*
 * Predictive thermal throttling.  When the temperature is rising fast
 * enough that a trip point could be hit within the prediction window,
 * Iyashi voluntarily reduces the performance floor proactively so the
 * eventual throttle is gradual rather than a sudden cliff.
 *
 *   predict_enable    — 0=off, 1=on (default: on)
 *   predict_window_ms — look-ahead window in milliseconds (default: 200)
 *   predict_slope_mc_per_s  — rate-of-change threshold in mC/s.
 *                       When the temperature is rising faster than this
 *                       and headroom is below predict_window_ms away at
 *                       current slope, pre-throttle by reducing floor.
 *                       Default 5000 mC/s = 5 C/s.
 *
 * Tracking uses jiffy-granularity timestamps (cheap, no hrtimer needed)
 * and the already-read zone temperature from the clamp_target path.
 */
static unsigned int iyashi_predict_enable       __read_mostly = 1;
static unsigned int iyashi_predict_window_ms    __read_mostly = 200;
static unsigned int iyashi_predict_slope_mc_per_s __read_mostly = 5000;

/* Per-cdev state for slope tracking.  Indexed by cdev->type string hash. */
struct iyashi_predict_state {
	int		last_temp_mc;
	unsigned long	last_jiffies;
};

/*
 * We keep a small hash table keyed by cdev type string so that
 * independent cooling devices (CPU cluster 0, CPU cluster 1, GPU)
 * each get their own slope tracker.  A fixed-size array of 8 entries
 * covers any reasonable mobile SoC topology.
 */
#define IYASHI_PREDICT_HASH_BITS 4
#define IYASHI_PREDICT_NR_ENT    (1 << IYASHI_PREDICT_HASH_BITS)

static struct iyashi_predict_state
iyashi_predict_tbl[IYASHI_PREDICT_NR_ENT];
static DEFINE_SPINLOCK(iyashi_predict_lock);

/* Jenkins one-at-a-time hash of a string, truncated to table bits. */
static unsigned int iyashi_predict_hash(const char *s)
{
	unsigned int hash = 0;

	if (!s)
		return 0;
	while (*s) {
		hash += *s++;
		hash += (hash << 10);
		hash ^= (hash >> 6);
	}
	hash += (hash << 3);
	hash ^= (hash >> 11);
	hash += (hash << 15);
	return hash & (IYASHI_PREDICT_NR_ENT - 1);
}

/*
 * Compute a predictive penalty percentage (0..100) that reflects how
 * close the zone is to hitting a trip at the current warming rate.
 * Returns 0 when prediction is disabled, slope is flat/cooling, or
 * headroom is large enough to absorb the window.
 */
static unsigned int iyashi_predict_penalty(const char *cdev_type,
					   int zone_temp_mc,
					   int closest_trip_mc,
					   int min_headroom_c)
{
	unsigned int idx;
	struct iyashi_predict_state *st;
	unsigned long now = jiffies;
	unsigned int slope_mc_per_s;
	int headroom_mc;
	unsigned long dt;
	int dt_mc;

	if (!READ_ONCE(iyashi_predict_enable))
		return 0;
	if (min_headroom_c == INT_MAX || min_headroom_c <= 0)
		return 0;
	if (!zone_temp_mc || !closest_trip_mc)
		return 0;

	idx = iyashi_predict_hash(cdev_type);
	st = &iyashi_predict_tbl[idx];

	spin_lock(&iyashi_predict_lock);
	dt = jiffies - st->last_jiffies;
	dt_mc = zone_temp_mc - st->last_temp_mc;
	st->last_temp_mc = zone_temp_mc;
	st->last_jiffies = now;
	spin_unlock(&iyashi_predict_lock);

	/* Need at least 2 samples to compute slope */
	if (!dt || dt_mc <= 0)
		return 0;

	slope_mc_per_s = (unsigned int)dt_mc * HZ / (unsigned int)dt;

	if (slope_mc_per_s < READ_ONCE(iyashi_predict_slope_mc_per_s))
		return 0;

	/* How much headroom in mC? */
	headroom_mc = (closest_trip_mc - zone_temp_mc);
	if (headroom_mc <= 0)
		return 0;

	/*
	 * How long until we hit the trip at current slope?
	 * If the trip would be reached within predict_window_ms,
	 * compute a penalty proportional to how imminent it is.
	 * Penalty ranges from 0 (barely inside window) to
	 * 50 (at the trip edge).
	 */
	{
		unsigned int window = READ_ONCE(iyashi_predict_window_ms);
		unsigned long time_to_trip_ms =
			(unsigned long)headroom_mc * 1000 /
			max(slope_mc_per_s, 1U);
		unsigned int penalty;

		if (time_to_trip_ms > window)
			return 0;

		/*
		 * Linear penalty from 0 (time_to_trip==window) to
		 * 50 (time_to_trip==0).  This reduces the effective
		 * floor_pct temporarily, pre-throttling gradually.
		 */
		penalty = 50 - (50 * time_to_trip_ms / window);
		return min(penalty, 50U);
	}
}

/*
 * Static key gating.  Flipped by enabled_store() so the disabled
 * fast path is a single unlikely-branch.
 */
static DEFINE_STATIC_KEY_FALSE(iyashi_active_key);

static void iyashi_qos_attach_all(void);
static void iyashi_qos_update_all(void);

/*
 * Recompute the static-branch state from the AND of (iyashi_enabled,
 * !iyashi_charger_suppressed).  Both writers (enabled_store and
 * iyashi_set_charger_suppressed) call this after their own WRITE_ONCE
 * so the fast path's single static-branch read is always consistent
 * with the most recent userspace + charger state.
 */
static void iyashi_recompute_active(void)
{
	bool active = READ_ONCE(iyashi_enabled) &&
		      !READ_ONCE(iyashi_charger_suppressed);

	if (active)
		static_branch_enable(&iyashi_active_key);
	else
		static_branch_disable(&iyashi_active_key);
}

/* --------------------------------------------------------------- *
 * Observability latches (R/O from sysfs)                          *
 * --------------------------------------------------------------- */

static atomic64_t iyashi_clamped_count;
static atomic64_t iyashi_passthrough_count;
static atomic64_t iyashi_freq_floor_used_count;

static int iyashi_last_min_headroom_c  __read_mostly;
static int iyashi_last_trip_temp_mc    __read_mostly;
static int iyashi_last_zone_temp_mc    __read_mostly;
static unsigned long iyashi_last_target_in   __read_mostly;
static unsigned long iyashi_last_target_out  __read_mostly;

/* --------------------------------------------------------------- *
 * cdev filter -- which cooling devices Iyashi acts on             *
 *                                                                 *
 * Default: "thermal-cpufreq-,thermal-devfreq-,thermal-gpufreq-"    *
 * (prefix match).  Covers every cpufreq_cooling.c-registered cdev *
 * (name = thermal-cpufreq-N), every devfreq_cooling.c-registered  *
 * GPU/DDR cdev (name = thermal-devfreq-N), and any vendor          *
 * thermal-gpufreq-* cdev some downstream trees ship.               *
 *                                                                 *
 * The freq-units floor only applies meaningfully to cpufreq cdevs *
 * (cpufreq_cooling_floor_state_for_pct returns 0 elsewhere), so   *
 * widening the filter just lets the state-units floor_pct floor   *
 * also gate GPU/DDR cooling -- it does not implicitly enable      *
 * min_freq_pct on those devices.                                  *
 *                                                                 *
 * Override via sysfs.  Comma-separated list of prefixes.  A leading*
 * '-' inverts to blacklist mode.                                  *
 * --------------------------------------------------------------- */

#define IYASHI_FILTER_LEN 256
#define IYASHI_DEFAULT_FILTER \
	"thermal-cpufreq-,thermal-devfreq-,thermal-gpufreq-"

static char iyashi_cdev_filter_buf[IYASHI_FILTER_LEN] = IYASHI_DEFAULT_FILTER;
static DEFINE_SPINLOCK(iyashi_filter_lock);

static bool iyashi_cdev_allowed(const char *cdev_type)
{
	char snapshot[IYASHI_FILTER_LEN];
	char *p, *tok;
	bool blacklist;
	unsigned long flags;

	if (!cdev_type)
		return false;

	spin_lock_irqsave(&iyashi_filter_lock, flags);
	strscpy(snapshot, iyashi_cdev_filter_buf, sizeof(snapshot));
	spin_unlock_irqrestore(&iyashi_filter_lock, flags);

	if (snapshot[0] == '\0')
		return false;	/* empty filter = match nothing */

	blacklist = (snapshot[0] == '-');
	p = snapshot + (blacklist ? 1 : 0);

	while ((tok = strsep(&p, ",")) != NULL) {
		size_t len = strlen(tok);

		while (len > 0 &&
		       (tok[len - 1] == ' ' ||
			tok[len - 1] == '\t' ||
			tok[len - 1] == '\n' ||
			tok[len - 1] == '\r'))
			tok[--len] = '\0';
		while (*tok == ' ' || *tok == '\t')
			tok++;
		if (*tok == '\0')
			continue;
		if (!strncmp(cdev_type, tok, strlen(tok)))
			return !blacklist;
	}

	return blacklist;
}

/* --------------------------------------------------------------- *
 * Core hook -- called from thermal_cdev_update()                  *
 * --------------------------------------------------------------- */

unsigned long iyashi_clamp_target(struct thermal_cooling_device *cdev,
				  unsigned long target)
{
	struct thermal_instance *inst;
	int min_headroom_c = INT_MAX;
	int closest_trip_mc = 0;
	int closest_zone_mc = 0;
	unsigned int near_c, floor_pct;
	unsigned long max_state = 0, floor_state;
	bool hikari_boosted = false;

	if (!static_branch_unlikely(&iyashi_active_key))
		return target;

	if (!iyashi_cdev_allowed(cdev->type))
		return target;

	if (cdev->ops->get_max_state &&
	    cdev->ops->get_max_state(cdev, &max_state))
		return target;
	if (!max_state)
		return target;

	near_c    = READ_ONCE(iyashi_near_limit_offset_c);
	floor_pct = READ_ONCE(iyashi_floor_pct);

	/*
	 * Hikari cross-link: if the scheduler is seeing sustained
	 * wake-demand (bursty foreground load), temporarily raise
	 * the performance floor so thermal doesn't pull freq down
	 * during the burst.  Tunable-gated: hikari_aware == 0 on
	 * BALANCED means this block is never entered.
	 */
	if (READ_ONCE(iyashi_hikari_aware)) {
		unsigned int window = READ_ONCE(iyashi_hikari_window_ms);
		unsigned long last = hikari_get_last_demand_jiffies();

		if (last && time_is_after_jiffies(last + msecs_to_jiffies(window))) {
			unsigned int boost = READ_ONCE(iyashi_hikari_boost_pct);

			if (floor_pct + boost <= 100)
				floor_pct += boost;
			else
				floor_pct = 100;
			hikari_boosted = true;
		}
	}

	/*
	 * thermal_cdev_update() already holds cdev->lock when we are
	 * called, so cdev->thermal_instances is safe to walk.
	 */
	list_for_each_entry(inst, &cdev->thermal_instances, cdev_node) {
		struct thermal_zone_device *tz = inst->tz;
		int trip_temp = 0;
		int zone_temp;
		int headroom_c;

		if (!tz || !tz->ops || !tz->ops->get_trip_temp)
			continue;
		if (inst->target == THERMAL_NO_TARGET)
			continue;
		if (tz->ops->get_trip_temp(tz, inst->trip, &trip_temp))
			continue;

		zone_temp = tz->temperature;
		if (zone_temp <= 0)
			continue;

		headroom_c = (trip_temp - zone_temp) / 1000;
		if (headroom_c < min_headroom_c) {
			min_headroom_c  = headroom_c;
			closest_trip_mc = trip_temp;
			closest_zone_mc = zone_temp;
		}
	}

	/*
	 * Predictive thermal pre-throttling.  If the temperature is rising
	 * fast and close to a trip, compute a penalty that reduces the
	 * effective floor_pct so the eventual throttle is gradual.
	 */
	if (min_headroom_c != INT_MAX && closest_trip_mc && closest_zone_mc) {
		unsigned int pred = iyashi_predict_penalty(cdev->type,
					closest_zone_mc, closest_trip_mc,
					min_headroom_c);
		if (pred && pred <= 50) {
			if (pred > floor_pct)
				floor_pct = 0;
			else
				floor_pct -= pred;
		}
	}

	/* Latches: visible from /sys/kernel/iyashi/last_* */
	WRITE_ONCE(iyashi_last_min_headroom_c,
		   min_headroom_c == INT_MAX ? 0 : min_headroom_c);
	WRITE_ONCE(iyashi_last_trip_temp_mc, closest_trip_mc);
	WRITE_ONCE(iyashi_last_zone_temp_mc, closest_zone_mc);
	WRITE_ONCE(iyashi_last_target_in,    target);

	/*
	 * Cooldown imminent: at least one trip is within near_c of
	 * the bound zone's current temperature.  Surrender control.
	 */
	if (min_headroom_c != INT_MAX && min_headroom_c < (int)near_c) {
		atomic64_inc(&iyashi_passthrough_count);
		WRITE_ONCE(iyashi_last_target_out, target);
		if (trace_iyashi_clamp_enabled())
			trace_iyashi_clamp(cdev->type, target, target,
					   floor_pct,
					   min_headroom_c == INT_MAX ? 0 : min_headroom_c,
					   false, hikari_boosted);
		return target;
	}

	/*
	 * Far from any trip: enforce performance floor.  floor_state
	 * is the deepest cooling step we tolerate, in cooling-device
	 * units where 0 = no mitigation and max_state = fully cold.
	 */
	floor_state = max_state * (100 - floor_pct) / 100;

	/*
	 * Optional freq-units override: if the user expressed the floor
	 * as "% of cpuinfo_max_freq", translate it through the cpufreq
	 * cooling helper and prefer it when it is more permissive (i.e.
	 * its state index is lower, meaning a higher actual frequency).
	 * cpufreq_cooling_floor_state_for_pct() returns 0 for non-cpufreq
	 * cdevs or when the helper cannot decide, which we treat as
	 * "no override".
	 */
	{
		unsigned int min_freq_pct = READ_ONCE(iyashi_min_freq_pct);

		if (min_freq_pct) {
			unsigned long freq_floor =
				cpufreq_cooling_floor_state_for_pct(cdev,
								    min_freq_pct);

			if (freq_floor && freq_floor < floor_state) {
				floor_state = freq_floor;
				atomic64_inc(&iyashi_freq_floor_used_count);
			}
		}
	}

	if (target > floor_state) {
		atomic64_inc(&iyashi_clamped_count);
		WRITE_ONCE(iyashi_last_target_out, floor_state);
		if (trace_iyashi_clamp_enabled())
			trace_iyashi_clamp(cdev->type, target, floor_state,
					   floor_pct,
					   min_headroom_c == INT_MAX ? 0 : min_headroom_c,
					   true, hikari_boosted);
		return floor_state;
	}

	atomic64_inc(&iyashi_passthrough_count);
	WRITE_ONCE(iyashi_last_target_out, target);
	if (trace_iyashi_clamp_enabled())
		trace_iyashi_clamp(cdev->type, target, target,
				   floor_pct,
				   min_headroom_c == INT_MAX ? 0 : min_headroom_c,
				   false, hikari_boosted);
	return target;
}
EXPORT_SYMBOL_GPL(iyashi_clamp_target);

/*
 * Profile-aware Iyashi tuning.  Called from Zenith's
 * zenith_apply_profile() when the active profile changes.
 *
 * Profile IDs mirror Zenith's ZENITH_PROFILE_* defines:
 *   0 = CUSTOM, 1 = PERFORMANCE, 2 = BALANCED, 3 = BATTERY,
 *   4 = LEGACY, 5 = GAMING, 6 = AUDIO, 7 = AUTO.
 *
 * BALANCED / CUSTOM / LEGACY / AUTO write the compile-time
 * defaults (floor_pct=90, near_limit=5) so the cold-boot
 * baseline is preserved byte-for-byte.
 *
 * PERFORMANCE / GAMING raise the floor and widen the headroom
 * margin so cpufreq stays higher for longer during sustained
 * foreground load.
 *
 * BATTERY lowers the floor and narrows the margin so thermal
 * responses arrive sooner and the cooling devices save power.
 *
 * AUDIO (6) lifts the floor to 92% (vs BALANCED's 90%) and
 * widens the near-limit margin to 6 C (vs 5 C) to reduce the
 * chance of a thermal-triggered frequency dip glitching an
 * active audio stream.  Hikari cross-link is left off because
 * audio threads are steady-state, not bursty.
 *
 * Does NOT touch enforce_min or cdev_filter -- those are
 * topology/policy-specific and belong in userspace init.
 */
static struct notifier_block iyashi_selene_nb;

void iyashi_apply_profile(unsigned int profile)
{
	struct iyashi_profile_vals {
		unsigned int floor_pct;
		unsigned int near_limit_offset_c;
		unsigned int min_freq_pct;
		unsigned int hikari_aware;
	};

	static const struct iyashi_profile_vals profiles[] = {
		/* PERFORMANCE (1): higher floor, wider margin, hikari cross-link */
		[1] = {
			.floor_pct          = 95,
			.near_limit_offset_c = 8,
			.min_freq_pct       = 0,
			.hikari_aware       = 1,
		},
		/* BALANCED (2): compile-time defaults, hikari cross-link off */
		[2] = {
			.floor_pct          = 90,
			.near_limit_offset_c = 5,
			.min_freq_pct       = 0,
			.hikari_aware       = 0,
		},
		/* BATTERY (3): lower floor, tighter margin */
		[3] = {
			.floor_pct          = 60,
			.near_limit_offset_c = 3,
			.min_freq_pct       = 0,
			.hikari_aware       = 0,
		},
		/* GAMING (5): most aggressive floor, hikari cross-link */
		[5] = {
			.floor_pct          = 97,
			.near_limit_offset_c = 10,
			.min_freq_pct       = 0,
			.hikari_aware       = 1,
		},
		/* AUDIO (6): moderate floor, protects against buffer underruns */
		[6] = {
			.floor_pct          = 92,
			.near_limit_offset_c = 6,
			.min_freq_pct       = 0,
			.hikari_aware       = 0,
		},
	};

	const struct iyashi_profile_vals *v;

	profile = zenith_resolve_profile(profile);
	if (profile >= ARRAY_SIZE(profiles))
		profile = ZENITH_PROFILE_BALANCED;

	v = &profiles[profile];
	if (!v->floor_pct)
		v = &profiles[2];

	WRITE_ONCE(iyashi_floor_pct, v->floor_pct);
	WRITE_ONCE(iyashi_near_limit_offset_c, v->near_limit_offset_c);
	WRITE_ONCE(iyashi_min_freq_pct, v->min_freq_pct);
	WRITE_ONCE(iyashi_hikari_aware, v->hikari_aware);

	if (READ_ONCE(iyashi_enforce_min)) {
		if (v->min_freq_pct)
			iyashi_qos_attach_all();
		iyashi_qos_update_all();
	}

	if (trace_iyashi_profile_enabled())
		trace_iyashi_profile(profile, v->floor_pct,
				     v->near_limit_offset_c,
				     v->hikari_aware);

	#ifdef CONFIG_IYASHI_DEBUG_MSG
	pr_info_ratelimited("iyashi: profile %u applied (floor=%u%% near_limit=%uC min_freq=%u%% hikari_aware=%u)\n",
			    profile, v->floor_pct, v->near_limit_offset_c,
			    v->min_freq_pct, v->hikari_aware);
#endif /* CONFIG_IYASHI_DEBUG_MSG */
}

/* --------------------------------------------------------------- *
 * Per-cpufreq-policy freq_qos MIN enforcement (enforce_min)        *
 *                                                                 *
 * The clamp_target path above keeps cpufreq cooling devices from  *
 * pulling cpufreq below the floor.  But "consistent performance"  *
 * also wants cpufreq to stay high when there is no thermal cap   *
 * at all (e.g. light wake-bursts).  Iyashi can opt-in to register *
 * a FREQ_QOS_MIN request per policy so the scheduler is told the  *
 * floor through cpufreq's official channel.                       *
 *                                                                 *
 * Lifecycle:                                                       *
 *   - enforce_min flips 0->1: walk online cpus, attach qos per pol*
 *   - enforce_min flips 1->0: walk list, detach + free each entry *
 *   - min_freq_pct changes while enforce_min=1: update every req  *
 *   - CPUFREQ_CREATE_POLICY: attach if enforce_min=1              *
 *   - CPUFREQ_REMOVE_POLICY: detach matching entry if present     *
 *                                                                 *
 * Concurrency: iyashi_qos_lock (mutex) guards iyashi_qos_list and *
 * the enforce_min state transitions.  The freq_qos API takes its  *
 * own internal locks; iyashi_qos_lock is the order on top.        *
 * --------------------------------------------------------------- */

struct iyashi_qos_entry {
	struct cpufreq_policy	*policy;
	struct freq_qos_request	 req;
	struct list_head	 node;
};

static LIST_HEAD(iyashi_qos_list);
static DEFINE_MUTEX(iyashi_qos_lock);

static unsigned int iyashi_compute_min_freq(struct cpufreq_policy *policy,
					    unsigned int pct)
{
	u64 v;

	if (!policy || !policy->cpuinfo.max_freq)
		return 0;
	if (pct == 0 || pct > 100)
		return 0;

	v = (u64)policy->cpuinfo.max_freq * pct;
	do_div(v, 100);
	if (v > policy->cpuinfo.max_freq)
		v = policy->cpuinfo.max_freq;
	return (unsigned int)v;
}

/* iyashi_qos_lock must be held. */
static struct iyashi_qos_entry *
iyashi_qos_find_entry(struct cpufreq_policy *policy)
{
	struct iyashi_qos_entry *e;

	list_for_each_entry(e, &iyashi_qos_list, node) {
		if (e->policy == policy)
			return e;
	}
	return NULL;
}

/* iyashi_qos_lock must be held.  Adds an entry if one is not present. */
static int iyashi_qos_attach_locked(struct cpufreq_policy *policy)
{
	struct iyashi_qos_entry *entry;
	unsigned int freq;
	int ret;

	if (iyashi_qos_find_entry(policy))
		return 0;

	freq = iyashi_compute_min_freq(policy,
				       READ_ONCE(iyashi_min_freq_pct));
	if (!freq)
		return 0;	/* min_freq_pct == 0 -> nothing to enforce */

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->policy = policy;
	ret = freq_qos_add_request(&policy->constraints, &entry->req,
				   FREQ_QOS_MIN, freq);
	if (ret < 0) {
		kfree(entry);
		return ret;
	}

	list_add(&entry->node, &iyashi_qos_list);
	return 0;
}

/* iyashi_qos_lock must be held. */
static void iyashi_qos_detach_entry_locked(struct iyashi_qos_entry *entry)
{
	list_del(&entry->node);
	freq_qos_remove_request(&entry->req);
	kfree(entry);
}

/* Attach a request to every online cpu's policy.  enforce_min=1 path. */
static void iyashi_qos_attach_all(void)
{
	struct cpufreq_policy *policy;
	int cpu;

	mutex_lock(&iyashi_qos_lock);
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		/* Only attach once per policy (multiple cpus per policy). */
		if (!iyashi_qos_find_entry(policy))
			(void)iyashi_qos_attach_locked(policy);
		cpufreq_cpu_put(policy);
	}
	mutex_unlock(&iyashi_qos_lock);
}

/* Detach every request.  enforce_min=0 path. */
static void iyashi_qos_detach_all(void)
{
	struct iyashi_qos_entry *e, *tmp;

	mutex_lock(&iyashi_qos_lock);
	list_for_each_entry_safe(e, tmp, &iyashi_qos_list, node)
		iyashi_qos_detach_entry_locked(e);
	mutex_unlock(&iyashi_qos_lock);
}

/* Update every active request after min_freq_pct changes. */
static void iyashi_qos_update_all(void)
{
	struct iyashi_qos_entry *e;
	unsigned int freq;
	unsigned int pct = READ_ONCE(iyashi_min_freq_pct);

	mutex_lock(&iyashi_qos_lock);
	list_for_each_entry(e, &iyashi_qos_list, node) {
		freq = iyashi_compute_min_freq(e->policy, pct);
		if (!freq)
			freq = FREQ_QOS_MIN_DEFAULT_VALUE;
		(void)freq_qos_update_request(&e->req, (s32)freq);
	}
	mutex_unlock(&iyashi_qos_lock);
}

static int iyashi_cpufreq_policy_notify(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (!READ_ONCE(iyashi_enforce_min))
		return NOTIFY_OK;

	switch (event) {
	case CPUFREQ_CREATE_POLICY:
		mutex_lock(&iyashi_qos_lock);
		(void)iyashi_qos_attach_locked(policy);
		mutex_unlock(&iyashi_qos_lock);
		break;
	case CPUFREQ_REMOVE_POLICY:
		mutex_lock(&iyashi_qos_lock);
		{
			struct iyashi_qos_entry *e =
				iyashi_qos_find_entry(policy);
			if (e)
				iyashi_qos_detach_entry_locked(e);
		}
		mutex_unlock(&iyashi_qos_lock);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block iyashi_cpufreq_policy_nb = {
	.notifier_call = iyashi_cpufreq_policy_notify,
};

/* --------------------------------------------------------------- *
 * sysfs                                                           *
 * --------------------------------------------------------------- */

#define IYASHI_ATTR_RW(_name, _var, _validate)				\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%u\n", READ_ONCE(_var));		\
}									\
static ssize_t _name##_store(struct kobject *kobj,			\
			     struct kobj_attribute *attr,		\
			     const char *buf, size_t count)		\
{									\
	unsigned int val;						\
	if (kstrtouint(buf, 0, &val))					\
		return -EINVAL;						\
	if (!(_validate))						\
		return -ERANGE;						\
	WRITE_ONCE(_var, val);						\
	return count;							\
}									\
static struct kobj_attribute iyashi_##_name##_attr =			\
	__ATTR(_name, 0644, _name##_show, _name##_store)

static ssize_t enabled_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_enabled));
}

static ssize_t enabled_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (val > 1)
		return -ERANGE;

	if (val == READ_ONCE(iyashi_enabled))
		return count;

	WRITE_ONCE(iyashi_enabled, val);
	iyashi_recompute_active();

	return count;
}

static struct kobj_attribute iyashi_enabled_attr =
	__ATTR(enabled, 0644, enabled_show, enabled_store);

void iyashi_set_charger_suppressed(bool suppressed)
{
	unsigned int val = suppressed ? 1 : 0;

	if (val == READ_ONCE(iyashi_charger_suppressed))
		return;

	WRITE_ONCE(iyashi_charger_suppressed, val);
	iyashi_recompute_active();
}

static ssize_t charger_suppressed_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  READ_ONCE(iyashi_charger_suppressed));
}

static struct kobj_attribute iyashi_charger_suppressed_attr =
	__ATTR(charger_suppressed, 0444, charger_suppressed_show, NULL);

IYASHI_ATTR_RW(floor_pct,           iyashi_floor_pct,           val >= 50 && val <= 100);
IYASHI_ATTR_RW(near_limit_offset_c, iyashi_near_limit_offset_c, val >= 1  && val <= 15);

/* Predictive thermal throttling knobs */
IYASHI_ATTR_RW(predict_enable,          iyashi_predict_enable,          val <= 1);
IYASHI_ATTR_RW(predict_window_ms,       iyashi_predict_window_ms,       val >= 10  && val <= 5000);
IYASHI_ATTR_RW(predict_slope_mc_per_s,  iyashi_predict_slope_mc_per_s,  val >= 100 && val <= 100000);

static ssize_t min_freq_pct_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_min_freq_pct));
}

static ssize_t min_freq_pct_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (!(val == 0 || (val >= 50 && val <= 100)))
		return -ERANGE;

	WRITE_ONCE(iyashi_min_freq_pct, val);

	/*
	 * Re-apply on every attached policy if enforce_min is active.
	 * If enforce_min was enabled while min_freq_pct was 0, no
	 * requests were attached yet; attach them now before updating.
	 */
	if (READ_ONCE(iyashi_enforce_min)) {
		if (val)
			iyashi_qos_attach_all();
		iyashi_qos_update_all();
	}

	return count;
}

static struct kobj_attribute iyashi_min_freq_pct_attr =
	__ATTR(min_freq_pct, 0644, min_freq_pct_show, min_freq_pct_store);

static ssize_t enforce_min_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_enforce_min));
}

static ssize_t enforce_min_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (val > 1)
		return -ERANGE;

	if (val == READ_ONCE(iyashi_enforce_min))
		return count;

	WRITE_ONCE(iyashi_enforce_min, val);
	if (val)
		iyashi_qos_attach_all();
	else
		iyashi_qos_detach_all();

	return count;
}

static struct kobj_attribute iyashi_enforce_min_attr =
	__ATTR(enforce_min, 0644, enforce_min_show, enforce_min_store);

#define IYASHI_ATTR_RO_INT(_name, _var)					\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%d\n", READ_ONCE(_var));		\
}									\
static struct kobj_attribute iyashi_##_name##_attr =			\
	__ATTR(_name, 0444, _name##_show, NULL)

#define IYASHI_ATTR_RO_ULONG(_name, _var)				\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%lu\n", READ_ONCE(_var));		\
}									\
static struct kobj_attribute iyashi_##_name##_attr =			\
	__ATTR(_name, 0444, _name##_show, NULL)

#define IYASHI_ATTR_RO_ATOMIC64(_name, _var)				\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%lld\n", (long long)atomic64_read(&_var)); \
}									\
static struct kobj_attribute iyashi_##_name##_attr =			\
	__ATTR(_name, 0444, _name##_show, NULL)

IYASHI_ATTR_RO_INT     (last_min_headroom_c,    iyashi_last_min_headroom_c);
IYASHI_ATTR_RO_INT     (last_trip_temp_mc,      iyashi_last_trip_temp_mc);
IYASHI_ATTR_RO_INT     (last_zone_temp_mc,      iyashi_last_zone_temp_mc);
IYASHI_ATTR_RO_ULONG   (last_target_in,         iyashi_last_target_in);
IYASHI_ATTR_RO_ULONG   (last_target_out,        iyashi_last_target_out);
IYASHI_ATTR_RO_ATOMIC64(clamped_count,          iyashi_clamped_count);
IYASHI_ATTR_RO_ATOMIC64(passthrough_count,      iyashi_passthrough_count);
IYASHI_ATTR_RO_ATOMIC64(freq_floor_used_count,  iyashi_freq_floor_used_count);

static ssize_t cdev_filter_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;
	ssize_t ret;

	spin_lock_irqsave(&iyashi_filter_lock, flags);
	ret = sysfs_emit(buf, "%s\n", iyashi_cdev_filter_buf);
	spin_unlock_irqrestore(&iyashi_filter_lock, flags);
	return ret;
}

static ssize_t cdev_filter_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long flags;
	size_t len = count;

	if (len >= IYASHI_FILTER_LEN)
		return -E2BIG;

	spin_lock_irqsave(&iyashi_filter_lock, flags);
	memcpy(iyashi_cdev_filter_buf, buf, len);
	iyashi_cdev_filter_buf[len] = '\0';
	if (len > 0 && iyashi_cdev_filter_buf[len - 1] == '\n')
		iyashi_cdev_filter_buf[len - 1] = '\0';
	spin_unlock_irqrestore(&iyashi_filter_lock, flags);
	return count;
}

static struct kobj_attribute iyashi_cdev_filter_attr =
	__ATTR(cdev_filter, 0644, cdev_filter_show, cdev_filter_store);

/* --------------------------------------------------------------- *
 * Hikari cross-link sysfs tunables                                *
 * --------------------------------------------------------------- */

static ssize_t hikari_aware_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_hikari_aware));
}

static ssize_t hikari_aware_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	WRITE_ONCE(iyashi_hikari_aware, !!val);
	return count;
}

static struct kobj_attribute iyashi_hikari_aware_attr =
	__ATTR(hikari_aware, 0644, hikari_aware_show, hikari_aware_store);

static ssize_t hikari_window_ms_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_hikari_window_ms));
}

static ssize_t hikari_window_ms_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1000)
		return -EINVAL;
	WRITE_ONCE(iyashi_hikari_window_ms, val);
	return count;
}

static struct kobj_attribute iyashi_hikari_window_ms_attr =
	__ATTR(hikari_window_ms, 0644, hikari_window_ms_show,
	       hikari_window_ms_store);

static ssize_t hikari_boost_pct_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_hikari_boost_pct));
}

static ssize_t hikari_boost_pct_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 50)
		return -EINVAL;
	WRITE_ONCE(iyashi_hikari_boost_pct, val);
	return count;
}

static struct kobj_attribute iyashi_hikari_boost_pct_attr =
	__ATTR(hikari_boost_pct, 0644, hikari_boost_pct_show,
	       hikari_boost_pct_store);

static ssize_t version_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "iyashi-v1\n");
}

static struct kobj_attribute iyashi_version_attr =
	__ATTR(version, 0444, version_show, NULL);

static struct attribute *iyashi_attrs[] = {
	&iyashi_version_attr.attr,
	&iyashi_enabled_attr.attr,
	&iyashi_charger_suppressed_attr.attr,
	&iyashi_floor_pct_attr.attr,
	&iyashi_min_freq_pct_attr.attr,
	&iyashi_enforce_min_attr.attr,
	&iyashi_near_limit_offset_c_attr.attr,
	&iyashi_cdev_filter_attr.attr,
	&iyashi_hikari_aware_attr.attr,
	&iyashi_hikari_window_ms_attr.attr,
	&iyashi_predict_enable_attr.attr,
	&iyashi_predict_window_ms_attr.attr,
	&iyashi_predict_slope_mc_per_s_attr.attr,
	&iyashi_hikari_boost_pct_attr.attr,
	&iyashi_clamped_count_attr.attr,
	&iyashi_passthrough_count_attr.attr,
	&iyashi_freq_floor_used_count_attr.attr,
	&iyashi_last_min_headroom_c_attr.attr,
	&iyashi_last_trip_temp_mc_attr.attr,
	&iyashi_last_zone_temp_mc_attr.attr,
	&iyashi_last_target_in_attr.attr,
	&iyashi_last_target_out_attr.attr,
	NULL,
};

static struct attribute_group iyashi_attr_group = {
	.attrs = iyashi_attrs,
};

static struct kobject *iyashi_kobj;

/* --------------------------------------------------------------- *
 * Safety self-test                                                *
 * --------------------------------------------------------------- */

/*
 * Verify the core floor math invariants so a misconfigured
 * floor_pct can never produce a nonsensical floor_state.
 * Called once at init, before exposing sysfs.
 */
#define IYASHI_SELFTEST_MAX 100

static int __init iyashi_safety_self_test(void)
{
	struct {
		unsigned int floor_pct;
		unsigned long max_state;
		unsigned long expected;
	} const cases[] = {
		{  0, IYASHI_SELFTEST_MAX, IYASHI_SELFTEST_MAX },
		{100, IYASHI_SELFTEST_MAX, 0 },
		{ 50, IYASHI_SELFTEST_MAX, IYASHI_SELFTEST_MAX / 2 },
		{ 90, IYASHI_SELFTEST_MAX, IYASHI_SELFTEST_MAX * 10 / 100 },
		{ 75, IYASHI_SELFTEST_MAX, IYASHI_SELFTEST_MAX * 25 / 100 },
		{ 95, IYASHI_SELFTEST_MAX, IYASHI_SELFTEST_MAX * 5 / 100 },
		{ 50, 0,                 0 },
		{ 90, 1,                 0 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		unsigned long floor_state =
			cases[i].max_state * (100 - cases[i].floor_pct) / 100;

		if (floor_state != cases[i].expected) {
			pr_err("iyashi: safety self-test FAILED "
			       "[max=%lu pct=%u]: got %lu, expected %lu\n",
			       cases[i].max_state, cases[i].floor_pct,
			       floor_state, cases[i].expected);
			return -EIO;
		}
	}

#ifdef CONFIG_IYASHI_DEBUG_MSG
	pr_info("iyashi: safety self-test passed (%u cases)\n",
		(unsigned int)ARRAY_SIZE(cases));
#endif /* CONFIG_IYASHI_DEBUG_MSG */
	return 0;
}

/* --------------------------------------------------------------- *
 * Init                                                            *
 * --------------------------------------------------------------- */


/* ------------------------------------------------------------ */
/* Selene game detection integration                            */
/* ------------------------------------------------------------ */
static int iyashi_selene_nb_fn(struct notifier_block *nb,
                               unsigned long event, void *data)
{
	switch (event) {
	case SELENE_EVENT_GAME_START:
		iyashi_apply_profile(ZENITH_PROFILE_GAMING);
		break;
	case SELENE_EVENT_GAME_STOP:
		iyashi_apply_profile(ZENITH_PROFILE_BALANCED);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block iyashi_selene_nb = {
	.notifier_call = iyashi_selene_nb_fn,
};

static int __init iyashi_init(void)
{
	int ret;

	ret = iyashi_safety_self_test();
	if (ret) {
		pr_err("iyashi: refusing to register sysfs (self-test failed)\n");
		return ret;
	}

	iyashi_kobj = kobject_create_and_add("iyashi", kernel_kobj);
	if (!iyashi_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(iyashi_kobj, &iyashi_attr_group);
	if (ret) {
		kobject_put(iyashi_kobj);
		iyashi_kobj = NULL;
		return ret;
	}

	if (READ_ONCE(iyashi_enabled))
		iyashi_recompute_active();

	/*
	 * Register a cpufreq policy notifier so the enforce_min path can
	 * attach/detach a FREQ_QOS_MIN request when policies come and go
	 * (CPU hot-plug etc.).  The notifier itself early-outs when
	 * enforce_min == 0, so it is cheap even with the feature disabled.
	 */
	ret = cpufreq_register_notifier(&iyashi_cpufreq_policy_nb,
					CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		pr_warn("Iyashi: cpufreq_register_notifier failed: %d (enforce_min will not survive hotplug)\n",
			ret);

	/* Subscribe to Selene game detection once at init */
	{
		int sret = selene_register_notifier(&iyashi_selene_nb);

		if (sret)
			pr_warn("iyashi: selene notifier register failed (%d)\n", sret);
		else
			pr_info("iyashi: subscribed to selene game detection chain\n");
	}

	#ifdef CONFIG_IYASHI_DEBUG_MSG
	pr_info("iyashi: initialized (floor=%u%% near_limit=%uC hikari_aware=%u)\n",
		READ_ONCE(iyashi_floor_pct),
		READ_ONCE(iyashi_near_limit_offset_c),
		READ_ONCE(iyashi_hikari_aware));
	#endif /* CONFIG_IYASHI_DEBUG_MSG */

	return 0;
}
late_initcall_sync(iyashi_init);
