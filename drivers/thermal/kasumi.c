// SPDX-License-Identifier: GPL-2.0
/*
 * Kasumi (霞) thermal dampening -- XTENSEI
 *
 * Delays thermal throttling by subtracting a configurable offset from
 * the reported temperature.  A linear ramp between ramp_start and
 * ceiling smoothly reduces the offset to zero so the real temperature
 * is reported once the safety ceiling is reached.
 *
 * Originally part of thermal_helpers.c; broken out into its own
 * compilation unit for clarity.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/minmax.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/zenith_profiles.h>
#include <linux/selene.h>
#include <linux/sysfs.h>
#include <linux/timekeeping.h>

#define CREATE_TRACE_POINTS
#include <trace/events/kasumi.h>
#undef CREATE_TRACE_POINTS

/*
 *   real < ramp_start : reported = real - offset        (full offset)
 *   ramp_start <= real < ceiling : offset tapers to 0   (smooth ramp)
 *   real >= ceiling   : reported = real                 (safety)
 */
static unsigned int kasumi_enable     __read_mostly = 1;
static unsigned int kasumi_offset_mc  __read_mostly = 5000;  /* 5 C  */
static unsigned int kasumi_ramp_mc    __read_mostly = 85000;  /* 85 C  */
static unsigned int kasumi_ceiling_mc __read_mostly = 95000;  /* 95 C  */

/*
 * External suppression flag driven by thermal_charger_guard.c.  Layered
 * orthogonally to kasumi_enable: the fast path treats
 * (kasumi_enable && !kasumi_charger_suppressed) as the effective active
 * state, so userspace's `enabled` knob keeps its declared meaning and
 * is never mutated by the charger guard.  Exposed read-only at
 * /sys/kernel/kasumi/charger_suppressed for observability.
 */
static unsigned int kasumi_charger_suppressed __read_mostly;

/*
 * Shape of the offset taper inside the [ramp_mc, ceiling_mc) window.
 *
 *   KASUMI_RAMP_LINEAR    (0, default)
 *       Existing behaviour: offset tapers linearly from full at
 *       ramp_mc to zero at ceiling_mc.
 *
 *   KASUMI_RAMP_QUADRATIC (1)
 *       offset tapers as (remaining/range)^2.  Gentler at the start
 *       of the ramp (close to ramp_mc), steeper near the ceiling.
 *       Buys a bit more illusion in the warm-but-not-hot region.
 *
 *   KASUMI_RAMP_STEP      (2)
 *       No taper.  Full offset all the way up to the ceiling, then
 *       a hard cliff to zero at >= ceiling_mc.  Useful for testing
 *       and for scenarios where the framework needs the cliff to
 *       trigger throttling on a single threshold crossing.
 *
 * Default 0 preserves the previous shape exactly, so this knob is
 * back-compatible with every existing tunables.cfg.
 */
enum {
	KASUMI_RAMP_LINEAR    = 0,
	KASUMI_RAMP_QUADRATIC = 1,
	KASUMI_RAMP_STEP      = 2,
	KASUMI_RAMP_MAX       = KASUMI_RAMP_STEP,
};
static unsigned int kasumi_ramp_shape __read_mostly = KASUMI_RAMP_LINEAR;

static const char * const kasumi_ramp_shape_names[] = {
	[KASUMI_RAMP_LINEAR]    = "linear",
	[KASUMI_RAMP_QUADRATIC] = "quadratic",
	[KASUMI_RAMP_STEP]      = "step",
};

/*
 * Boot-time warmup.  The very first seconds after a cold boot are
 * thermally noisy -- the framework can see a transient spike from
 * brief userspace init bursts before the platform has settled into
 * its idle baseline, then trip a guard temperature and pin a cooling
 * device.  Kasumi already smooths this with offset_mc, but on devices
 * with aggressive thermal trips the default 15 C is sometimes not
 * enough.
 *
 *   warmup_secs        -- seconds since boot during which the warmup
 *                         offset takes precedence over offset_mc.
 *                         Default 0 disables the feature entirely;
 *                         the dampening branch is the same as before.
 *
 *   warmup_offset_mc   -- the offset used while inside the warmup
 *                         window.  Substitutes for offset_mc; the
 *                         existing ramp / ceiling / shape logic
 *                         applies on top of it unchanged.
 *
 * The check uses ktime_get_boottime_seconds() which already accounts
 * for suspend so a device that boots, suspends for hours, and resumes
 * still observes "boot+30s" not "wall+hours".  Cheap enough on the
 * read path -- single per-CPU timer read, no locks.
 */
static unsigned int kasumi_warmup_secs       __read_mostly;        /* off */
static unsigned int kasumi_warmup_offset_mc  __read_mostly = 25000; /* 25 C */

/*
 * hot_threshold_mc / hot_extra_offset_mc -- temperature-dependent
 * dampening boost.  When real >= hot_threshold_mc and we are *not*
 * already above the cliff (real >= ceiling), add hot_extra_offset_mc
 * to the effective offset.  The reported temperature seen by the
 * thermal framework therefore lags the real temperature by an even
 * larger amount once things are hot, which suppresses cpufreq /
 * GPU / cooling-device throttling decisions that would otherwise
 * fire while the device is still well below the ceiling.
 *
 * Disabled by default (hot_threshold_mc == 0).  Capped so that even
 * a misconfigured pair (threshold below ramp, extra > ceiling)
 * cannot cause underflow or absurd reported values -- the cliff at
 * ceiling_mc still wins so real overheats still expose real temps.
 *
 * Observability: kasumi_hot_active_count increments on every dampen
 * call that actually applied the extra offset.
 */
static unsigned int kasumi_hot_threshold_mc  __read_mostly;        /* off */
static unsigned int kasumi_hot_extra_offset_mc __read_mostly = 10000; /* 10 C */
static atomic64_t   kasumi_hot_active_count = ATOMIC64_INIT(0);

/*
 * Observability latches.  Updated at the end of every kasumi_dampen()
 * call; readable through /sys/kernel/kasumi/last_*.  R/O so userspace
 * cannot fabricate state.  WRITE_ONCE / READ_ONCE for tearing safety.
 */
static int kasumi_last_real_mc        __read_mostly;
static int kasumi_last_reported_mc    __read_mostly;
static int kasumi_applied_offset_mc   __read_mostly;

/*
 * Zone-type filter.  Comma-separated list of thermal_zone types.
 *   ""                       (default)  apply to all zones
 *   "mtktscpu,mtktspmic"     whitelist  apply only to listed zones
 *   "-battery,mtktsbattery"  blacklist  apply to all EXCEPT listed zones
 *
 * Protected by kasumi_filter_lock.  The hot path takes a stack-local
 * snapshot under the lock and parses without it.
 */
#define KASUMI_FILTER_LEN 256
static char kasumi_zone_filter_buf[KASUMI_FILTER_LEN];
static DEFINE_SPINLOCK(kasumi_filter_lock);

/*
 * Per-zone offset override.  Comma-separated list of "zone_type=mc"
 * pairs that override kasumi_offset_mc for that specific zone, e.g.
 *
 *   "mtktscpu=20000,mtktspmic=10000"
 *
 * means dampen mtktscpu by 20 C and mtktspmic by 10 C while every
 * other zone keeps the global offset_mc.  Empty string (default)
 * means "no per-zone overrides, use offset_mc everywhere".  A zone
 * type appearing in this list still has to pass zone_filter; the
 * filter runs first and shorts the whole dampening path before the
 * override is even consulted.
 *
 * Protected by the same spinlock as zone_filter -- both are
 * stack-snapshotted on the hot path and parsed without the lock,
 * so concurrent writes never see a torn buffer.
 *
 * Bounded length to keep the stack snapshot in kasumi_dampen()
 * small.  A 256-byte budget fits ~16 typical zone names at a
 * generous 16-byte avg + 6-byte offset.
 */
#define KASUMI_ZONE_OFFSETS_LEN 256
static char kasumi_zone_offsets_buf[KASUMI_ZONE_OFFSETS_LEN];

/*
 * Look up a per-zone offset override for zone_type, in millideg C.
 * Returns true when a zone-specific value was found; the override may
 * intentionally be 0 so the normal offset path does not fall back to
 * the global offset.  Caller passes a stack-local snapshot of
 * kasumi_zone_offsets_buf so the parser does not block writers.
 */
static bool kasumi_zone_offset_lookup(const char *snapshot,
				      const char *zone_type,
				      unsigned int *offset)
{
	char tok[KASUMI_ZONE_OFFSETS_LEN];
	char *cur, *sep;
	unsigned int val;
	size_t zlen;

	if (!zone_type || !snapshot || snapshot[0] == '\0' || !offset)
		return false;

	zlen = strlen(zone_type);

	/*
	 * Copy snapshot to a local mutable buffer because strsep
	 * needs to overwrite separators.  Snapshot itself lives on
	 * the dampen-path's stack so this is one stack-to-stack
	 * memcpy with strscpy semantics.
	 */
	strscpy(tok, snapshot, sizeof(tok));
	cur = tok;

	while ((sep = strsep(&cur, ",")) != NULL) {
		char *eq;

		while (*sep == ' ' || *sep == '\t')
			sep++;
		eq = strchr(sep, '=');
		if (!eq)
			continue;
		*eq = '\0';
		if (strlen(sep) != zlen || strcmp(sep, zone_type) != 0)
			continue;
		if (kstrtouint(eq + 1, 0, &val))
			continue;
		*offset = val;
		return true;
	}
	return false;
}

bool kasumi_zone_allowed(const char *zone_type)
{
	char snapshot[KASUMI_FILTER_LEN];
	char *p, *tok;
	bool blacklist;
	unsigned long flags;

	if (!zone_type)
		return true;

	spin_lock_irqsave(&kasumi_filter_lock, flags);
	strscpy(snapshot, kasumi_zone_filter_buf, sizeof(snapshot));
	spin_unlock_irqrestore(&kasumi_filter_lock, flags);

	if (snapshot[0] == '\0')
		return true;

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
		if (!strcmp(tok, zone_type))
			return !blacklist;
	}

	return blacklist;
}

int kasumi_dampen(int real, const char *zone_type)
{
	unsigned int offset, ramp, ceiling;
	unsigned int per_zone;
	int dampened;

	if (!READ_ONCE(kasumi_enable) ||
	    READ_ONCE(kasumi_charger_suppressed) ||
	    real <= 0)
		return real;

	offset  = READ_ONCE(kasumi_offset_mc);
	ramp    = READ_ONCE(kasumi_ramp_mc);
	ceiling = READ_ONCE(kasumi_ceiling_mc);

	/*
	 * Per-zone offset override.  Takes precedence over offset_mc
	 * but is itself overridden by the boot warmup substitute
	 * below.  Snapshot the global buffer once under the lock and
	 * parse on the stack so concurrent writes do not have to
	 * wait on dampen.
	 */
	if (zone_type) {
		char snapshot[KASUMI_ZONE_OFFSETS_LEN];
		unsigned long flags;

		spin_lock_irqsave(&kasumi_filter_lock, flags);
		strscpy(snapshot, kasumi_zone_offsets_buf, sizeof(snapshot));
		spin_unlock_irqrestore(&kasumi_filter_lock, flags);

		if (kasumi_zone_offset_lookup(snapshot, zone_type,
					      &per_zone))
			offset = per_zone;
	}

	/*
	 * Boot warmup window.  Substitute warmup_offset_mc for the
	 * regular offset only while:
	 *   - warmup_secs is non-zero (admin opted in)
	 *   - warmup_offset_mc is non-zero (no point in 'use 0 instead')
	 *   - we are still within warmup_secs seconds of boot
	 * Past the window everything reverts to the regular offset_mc
	 * without any reconfiguration step.  Warmup wins over the
	 * per-zone override because the boot-time spike is platform-
	 * wide and not zone-specific.
	 */
	{
		unsigned int wsecs = READ_ONCE(kasumi_warmup_secs);
		unsigned int woff  = READ_ONCE(kasumi_warmup_offset_mc);

		if (wsecs && woff &&
		    ktime_get_boottime_seconds() < (time64_t)wsecs)
			offset = woff;
	}

	/*
	 * Hot-zone extra dampening.  Strictly below the ceiling cliff
	 * so a real overheat still hits the cliff branch immediately
	 * below and exposes raw temp to the framework.  Saturating
	 * add: cap effective offset at half ceiling to avoid wrapping
	 * a careless misconfiguration into a sub-zero "dampened" temp.
	 */
	{
		unsigned int hot_th    = READ_ONCE(kasumi_hot_threshold_mc);
		unsigned int hot_extra = READ_ONCE(kasumi_hot_extra_offset_mc);

		if (hot_th && hot_extra &&
		    real >= (int)hot_th && real < (int)ceiling) {
			unsigned int cap = ceiling / 2;
			unsigned int sum = offset + hot_extra;

			offset = (sum > cap) ? cap : sum;
			atomic64_inc(&kasumi_hot_active_count);
		}
	}

	if (real >= (int)ceiling) {
		WRITE_ONCE(kasumi_last_real_mc,      real);
		WRITE_ONCE(kasumi_last_reported_mc,  real);
		WRITE_ONCE(kasumi_applied_offset_mc, 0);
		return real;
	}

	if (real < (int)ramp) {
		dampened = real - (int)offset;
	} else {
		int range     = (int)ceiling - (int)ramp;
		int remaining = (int)ceiling - real;
		unsigned int shape = READ_ONCE(kasumi_ramp_shape);

		if (range <= 0) {
			dampened = real;
		} else {
			switch (shape) {
			case KASUMI_RAMP_STEP:
				/*
				 * No taper -- full offset all the way up
				 * to the ceiling, then the existing cliff
				 * branch above takes over at real >= ceiling.
				 */
				dampened = real - (int)offset;
				break;
			case KASUMI_RAMP_QUADRATIC: {
				/*
				 * dampened = real - offset * (1 - (progress/range)^2)
				 * where progress = real - ramp.  Equivalent form
				 * with one division:
				 *     offset_applied = offset *
				 *                      (range^2 - progress^2)
				 *                      / range^2
				 *
				 * Result: offset drops gently while real is just
				 * above ramp_mc (preserves more dampening), then
				 * accelerates toward zero near the ceiling.  At
				 * midway through the ramp window the applied
				 * offset is 0.75 * offset vs linear's 0.5 *
				 * offset -- gentler at first.
				 *
				 * Computed in 64-bit because range can be up
				 * to ~10000 mC; range^2 = 1e8 fits in u32 but
				 * the (offset * (range^2 - progress^2)) product
				 * needs u64 head-room.
				 */
				int progress = real - (int)ramp;
				u64 num = (u64)offset *
					  ((u64)range * range -
					   (u64)progress * progress);
				dampened = real - (int)(num /
							((u64)range * range));
				break;
			}
			case KASUMI_RAMP_LINEAR:
			default:
				dampened = real - (int)offset * remaining / range;
				break;
			}
		}
	}

	if (dampened < 0)
		dampened = 0;

	WRITE_ONCE(kasumi_last_real_mc,      real);
	WRITE_ONCE(kasumi_last_reported_mc,  dampened);
	WRITE_ONCE(kasumi_applied_offset_mc, real - dampened);

	if (trace_kasumi_filter_enabled())
		trace_kasumi_filter(zone_type ? zone_type : "(null)",
				    real, dampened, offset, ramp,
				    READ_ONCE(kasumi_ramp_shape));

	return dampened;
}

/*
 * Iyashi consumers read this; do not export to userspace through any
 * other channel than the /sys/kernel/kasumi/last_real_mc attribute.
 */
int kasumi_get_last_real_mc(void)
{
	return READ_ONCE(kasumi_last_real_mc);
}
EXPORT_SYMBOL_GPL(kasumi_get_last_real_mc);

/*
 * Return the offset that was actually applied during the most recent
 * kasumi_dampen() call, in millidegrees Celsius.  Hikari reads this
 * to detect when the dampening offset is so large that the frequency
 * floor and thermal dampening are conspiring to hide heat buildup.
 *
 * Safe from any context: returns a stale snapshot (last value written
 * by WRITE_ONCE in the dampen path) and never blocks.
 */
int kasumi_get_applied_offset_mc(void)
{
	return READ_ONCE(kasumi_applied_offset_mc);
}
EXPORT_SYMBOL_GPL(kasumi_get_applied_offset_mc);

static struct notifier_block kasumi_selene_nb;

/*
 * Profile-aware Kasumi tuning.  Called from Zenith's
 * zenith_apply_profile() when the active profile changes.
 *
 * Profile IDs mirror Zenith's ZENITH_PROFILE_* defines:
 *   0 = CUSTOM, 1 = PERFORMANCE, 2 = BALANCED, 3 = BATTERY,
 *   4 = LEGACY, 5 = GAMING, 6 = AUDIO, 7 = AUTO.
 *
 * BALANCED / CUSTOM / LEGACY / AUTO write the compile-time
 * defaults so the cold-boot baseline is preserved byte-for-byte.
 * PERFORMANCE / GAMING widen the dampening window (more offset,
 * higher ramp start, enable warmup + hot-zone boost) so the
 * framework throttles later during sustained foreground load.
 * BATTERY tightens the window so thermal responses arrive sooner
 * and the cooling devices can save power.
 * AUDIO (6) uses the same offset as BALANCED but lifts the
 * ramp start to 88 C so the dampening taper begins later,
 * preserving more headroom during sustained audio output
 * without triggering the aggressive quadratic cut that
 * PERFORMANCE/GAMING apply.
 *
 * Every tunable written here is already individually writable via
 * /sys/kernel/kasumi/; a subsequent sysfs write overrides the
 * profile bake until the next profile flip.
 */
void kasumi_apply_profile(unsigned int profile)
{
	struct kasumi_profile_vals {
		unsigned int offset_mc;
		unsigned int ramp_mc;
		unsigned int ceiling_mc;
		unsigned int ramp_shape;
		unsigned int warmup_secs;
		unsigned int warmup_offset_mc;
		unsigned int hot_threshold_mc;
		unsigned int hot_extra_offset_mc;
	};

	static const struct kasumi_profile_vals profiles[] = {
		/* PERFORMANCE (1): widen dampening for sustained load */
		[1] = {
			.offset_mc          = 20000,  /* 20 C */
			.ramp_mc            = 88000,  /* 88 C */
			.ceiling_mc         = 95000,  /* unchanged */
			.ramp_shape         = KASUMI_RAMP_QUADRATIC,
			.warmup_secs        = 30,
			.warmup_offset_mc   = 25000,
			.hot_threshold_mc   = 75000,  /* 75 C */
			.hot_extra_offset_mc = 10000, /* 10 C */
		},
		/* BALANCED (2): compile-time defaults */
		[2] = {
			.offset_mc          = 15000,
			.ramp_mc            = 85000,
			.ceiling_mc         = 95000,
			.ramp_shape         = KASUMI_RAMP_LINEAR,
			.warmup_secs        = 0,
			.warmup_offset_mc   = 25000,
			.hot_threshold_mc   = 0,
			.hot_extra_offset_mc = 0,
		},
		/* BATTERY (3): tighter dampening, save power */
		[3] = {
			.offset_mc          = 8000,   /* 8 C */
			.ramp_mc            = 80000,  /* 80 C */
			.ceiling_mc         = 95000,
			.ramp_shape         = KASUMI_RAMP_LINEAR,
			.warmup_secs        = 0,
			.warmup_offset_mc   = 25000,
			.hot_threshold_mc   = 0,
			.hot_extra_offset_mc = 10000,
		},		/* GAMING (5): most aggressive dampening */
		[5] = {
			.offset_mc          = 22000,	/* 22 C */
			.ramp_mc            = 90000,	/* 90 C */
			.ceiling_mc         = 95000,
			.ramp_shape         = KASUMI_RAMP_QUADRATIC,
			.warmup_secs        = 30,
			.warmup_offset_mc   = 25000,
			.hot_threshold_mc   = 70000,	/* 70 C */
			.hot_extra_offset_mc = 12000, /* 12 C */
		},
		/* AUDIO (6): moderate dampening, preserves audio jitter margins */
		[6] = {
			.offset_mc          = 15000,	/* 15 C — same as BALANCED */
			.ramp_mc            = 88000,	/* 88 C — later taper than BALANCED */
			.ceiling_mc         = 95000,
			.ramp_shape         = KASUMI_RAMP_LINEAR,
			.warmup_secs        = 0,
			.warmup_offset_mc   = 25000,
			.hot_threshold_mc   = 0,	/* off — audio doesn't push thermals */
			.hot_extra_offset_mc = 0,
		},
	};

	const struct kasumi_profile_vals *v;

	profile = zenith_resolve_profile(profile);
	if (profile >= ARRAY_SIZE(profiles))
		profile = ZENITH_PROFILE_BALANCED;

	v = &profiles[profile];
	if (!v->ceiling_mc)
		v = &profiles[2];

	WRITE_ONCE(kasumi_offset_mc, v->offset_mc);
	WRITE_ONCE(kasumi_ramp_mc, v->ramp_mc);
	WRITE_ONCE(kasumi_ceiling_mc, v->ceiling_mc);
	WRITE_ONCE(kasumi_ramp_shape, v->ramp_shape);
	WRITE_ONCE(kasumi_warmup_secs, v->warmup_secs);
	WRITE_ONCE(kasumi_warmup_offset_mc, v->warmup_offset_mc);
	WRITE_ONCE(kasumi_hot_threshold_mc, v->hot_threshold_mc);
	WRITE_ONCE(kasumi_hot_extra_offset_mc, v->hot_extra_offset_mc);

	if (trace_kasumi_profile_enabled())
		trace_kasumi_profile(profile, v->offset_mc, v->ramp_mc,
				     v->ramp_shape, v->warmup_secs);

	#ifdef CONFIG_KASUMI_DEBUG_MSG
	pr_info_ratelimited("kasumi: profile %u applied (offset=%u ramp=%u ceiling=%u shape=%u warmup=%us hot_thresh=%u)\n",
			    profile, v->offset_mc, v->ramp_mc, v->ceiling_mc,
			    v->ramp_shape, v->warmup_secs,
			    v->hot_threshold_mc);
#endif /* CONFIG_KASUMI_DEBUG_MSG */
}

/* ---- Kasumi sysfs interface (/sys/kernel/kasumi/) ---- */#define KASUMI_ATTR_RW(_name, _var, _validate)				\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{										\
	return sysfs_emit(buf, "%u\n", READ_ONCE(_var));		\
}										\
static ssize_t _name##_store(struct kobject *kobj,			\
			     struct kobj_attribute *attr,		\
			     const char *buf, size_t count)		\
{										\
	unsigned int val;						\
	if (kstrtouint(buf, 0, &val))					\
		return -EINVAL;						\
	if (!(_validate))						\
		return -ERANGE;						\
	WRITE_ONCE(_var, val);						\
	return count;							\
}										\
static struct kobj_attribute kasumi_##_name##_attr =			\
	__ATTR(_name, 0644, _name##_show, _name##_store)

#define KASUMI_ATTR_RO(_name, _var)					\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%d\n", READ_ONCE(_var));		\
}									\
static struct kobj_attribute kasumi_##_name##_attr =			\
	__ATTR(_name, 0444, _name##_show, NULL)

KASUMI_ATTR_RW(enabled,    kasumi_enable,            val <= 1);
KASUMI_ATTR_RO(charger_suppressed, kasumi_charger_suppressed);
KASUMI_ATTR_RW(offset_mc,  kasumi_offset_mc,          val <= 50000);
KASUMI_ATTR_RW(ramp_mc,    kasumi_ramp_mc,            val <= 120000);
KASUMI_ATTR_RW(ceiling_mc, kasumi_ceiling_mc,         val >= 50000 && val <= 120000);
KASUMI_ATTR_RW(warmup_secs,      kasumi_warmup_secs,           val <= 600);
KASUMI_ATTR_RW(warmup_offset_mc, kasumi_warmup_offset_mc,      val <= 50000);

KASUMI_ATTR_RW(hot_threshold_mc,    kasumi_hot_threshold_mc,       val <= 100000);
KASUMI_ATTR_RW(hot_extra_offset_mc, kasumi_hot_extra_offset_mc,    val <= 50000);

KASUMI_ATTR_RO(last_real_mc,       kasumi_last_real_mc);
KASUMI_ATTR_RO(last_reported_mc,   kasumi_last_reported_mc);
KASUMI_ATTR_RO(applied_offset_mc,  kasumi_applied_offset_mc);

static ssize_t hot_active_count_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%lld\n",
			  (long long)atomic64_read(&kasumi_hot_active_count));
}

static struct kobj_attribute kasumi_hot_active_count_attr =
	__ATTR(hot_active_count, 0444, hot_active_count_show, NULL);

static ssize_t zone_filter_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;
	ssize_t ret;

	spin_lock_irqsave(&kasumi_filter_lock, flags);
	ret = sysfs_emit(buf, "%s\n", kasumi_zone_filter_buf);
	spin_unlock_irqrestore(&kasumi_filter_lock, flags);
	return ret;
}

static ssize_t zone_filter_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long flags;
	size_t len = count;

	if (len >= KASUMI_FILTER_LEN)
		return -E2BIG;

	spin_lock_irqsave(&kasumi_filter_lock, flags);
	memcpy(kasumi_zone_filter_buf, buf, len);
	kasumi_zone_filter_buf[len] = '\0';
	/* strip a single trailing newline so 'echo "..." > zone_filter' DTRT */
	if (len > 0 && kasumi_zone_filter_buf[len - 1] == '\n')
		kasumi_zone_filter_buf[len - 1] = '\0';
	spin_unlock_irqrestore(&kasumi_filter_lock, flags);
	return count;
}

static struct kobj_attribute kasumi_zone_filter_attr =
	__ATTR(zone_filter, 0644, zone_filter_show, zone_filter_store);

/*
 * zone_offsets -- comma-separated "zone_type=offset_mc" pairs.
 * A matched zone may use offset 0 so the normal offset path does not
 * fall back to the global offset.
 */
static ssize_t zone_offsets_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;
	ssize_t ret;

	spin_lock_irqsave(&kasumi_filter_lock, flags);
	ret = sysfs_emit(buf, "%s\n", kasumi_zone_offsets_buf);
	spin_unlock_irqrestore(&kasumi_filter_lock, flags);
	return ret;
}

/*
 * Validate the zone_offsets string before storing.  Accepts the
 * empty string (clears all overrides) or one or more comma-separated
 * "<zone_type>=<offset_mc>" tokens.  Leading/trailing whitespace on
 * each token is tolerated; a trailing newline is stripped.  Anything
 * else returns -EINVAL so userspace gets a clear write failure rather
 * than a silent accept-but-ignore.
 */
static int kasumi_zone_offsets_validate(const char *s)
{
	char scratch[KASUMI_ZONE_OFFSETS_LEN];
	char *cur, *tok;
	unsigned int val;

	if (!s || s[0] == '\0')
		return 0;

	strscpy(scratch, s, sizeof(scratch));
	cur = scratch;

	while ((tok = strsep(&cur, ",")) != NULL) {
		char *eq, *name_end;

		while (*tok == ' ' || *tok == '\t')
			tok++;
		if (*tok == '\0')
			continue;

		eq = strchr(tok, '=');
		if (!eq || eq == tok)
			return -EINVAL;

		/* Trim trailing whitespace from the zone-name half. */
		name_end = eq;
		while (name_end > tok && (name_end[-1] == ' ' ||
					  name_end[-1] == '\t'))
			name_end--;
		if (name_end == tok)
			return -EINVAL;

		if (kstrtouint(eq + 1, 0, &val))
			return -EINVAL;
	}

	return 0;
}

static ssize_t zone_offsets_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long flags;
	char tmp[KASUMI_ZONE_OFFSETS_LEN];
	size_t len = count;
	int ret;

	if (len >= KASUMI_ZONE_OFFSETS_LEN)
		return -E2BIG;

	memcpy(tmp, buf, len);
	tmp[len] = '\0';
	if (len > 0 && tmp[len - 1] == '\n')
		tmp[len - 1] = '\0';

	ret = kasumi_zone_offsets_validate(tmp);
	if (ret)
		return ret;

	spin_lock_irqsave(&kasumi_filter_lock, flags);
	strscpy(kasumi_zone_offsets_buf, tmp, sizeof(kasumi_zone_offsets_buf));
	spin_unlock_irqrestore(&kasumi_filter_lock, flags);
	return count;
}

static struct kobj_attribute kasumi_zone_offsets_attr =
	__ATTR(zone_offsets, 0644, zone_offsets_show, zone_offsets_store);

/*
 * ramp_shape -- string-friendly RW attr.  Accepts on write:
 *   "linear" / "0"        -- KASUMI_RAMP_LINEAR    (default)
 *   "quadratic" / "1"     -- KASUMI_RAMP_QUADRATIC
 *   "step" / "2"          -- KASUMI_RAMP_STEP
 * On read, returns the canonical string name plus a parenthesised
 * numeric value:
 *   "linear (0)\n" | "quadratic (1)\n" | "step (2)\n"
 * which keeps both human and scripted readers happy.
 */
static ssize_t ramp_shape_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	unsigned int s = READ_ONCE(kasumi_ramp_shape);

	if (s > KASUMI_RAMP_MAX)
		return sysfs_emit(buf, "unknown (%u)\n", s);
	return sysfs_emit(buf, "%s (%u)\n", kasumi_ramp_shape_names[s], s);
}

static ssize_t ramp_shape_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	char tok[16];
	size_t n;
	unsigned int val, i;

	/* Strip trailing newline / whitespace into a stack-local token. */
	n = min_t(size_t, count, sizeof(tok) - 1);
	memcpy(tok, buf, n);
	tok[n] = '\0';
	while (n > 0 && (tok[n - 1] == '\n' || tok[n - 1] == '\r' ||
			 tok[n - 1] == ' '  || tok[n - 1] == '\t'))
		tok[--n] = '\0';

	for (i = 0; i <= KASUMI_RAMP_MAX; i++) {
		if (!strcmp(tok, kasumi_ramp_shape_names[i])) {
			WRITE_ONCE(kasumi_ramp_shape, i);
			return count;
		}
	}

	if (!kstrtouint(tok, 0, &val) && val <= KASUMI_RAMP_MAX) {
		WRITE_ONCE(kasumi_ramp_shape, val);
		return count;
	}

	return -EINVAL;
}

static struct kobj_attribute kasumi_ramp_shape_attr =
	__ATTR(ramp_shape, 0644, ramp_shape_show, ramp_shape_store);

static struct attribute *kasumi_attrs[] = {
	&kasumi_enabled_attr.attr,
	&kasumi_charger_suppressed_attr.attr,
	&kasumi_offset_mc_attr.attr,
	&kasumi_ramp_mc_attr.attr,
	&kasumi_ceiling_mc_attr.attr,
	&kasumi_ramp_shape_attr.attr,
	&kasumi_warmup_secs_attr.attr,
	&kasumi_warmup_offset_mc_attr.attr,
	&kasumi_hot_threshold_mc_attr.attr,
	&kasumi_hot_extra_offset_mc_attr.attr,
	&kasumi_hot_active_count_attr.attr,
	&kasumi_last_real_mc_attr.attr,
	&kasumi_last_reported_mc_attr.attr,
	&kasumi_applied_offset_mc_attr.attr,
	&kasumi_zone_filter_attr.attr,
	&kasumi_zone_offsets_attr.attr,
	NULL,
};

static struct attribute_group kasumi_attr_group = {
	.attrs = kasumi_attrs,
};

static struct kobject *kasumi_kobj;

/*
 * Safety self-test.  Runs once at init before any sysfs is exposed and
 * asserts the three contracts kasumi_dampen() makes:
 *
 *   1. at-or-above-ceiling input is returned unchanged (no dampening),
 *   2. below-ramp input gets exactly offset_mc subtracted,
 *   3. zero / negative input is passed through.
 *
 * If any of the three fails we refuse to expose /sys/kernel/kasumi at
 * all, which makes the failure loud (anyone scripting against the
 * sysfs files will see ENOENT) and prevents a misconfigured build
 * from quietly hiding throttling.  The dampening hook itself stays
 * active either way -- this is purely a paranoia check on the math.
 */
static int __init kasumi_safety_self_test(void)
{
	unsigned int ceiling = READ_ONCE(kasumi_ceiling_mc);
	unsigned int offset  = READ_ONCE(kasumi_offset_mc);
	unsigned int ramp    = READ_ONCE(kasumi_ramp_mc);
	unsigned int saved_enable;
	unsigned int saved_suppressed;
	int at_ceiling = (int)ceiling;
	int above_ceiling = (int)ceiling + 1000;
	int below_ramp = (int)ramp - 5000;
	int r1, r2, r3, r4;
	int ret = 0;

	/*
	 * Force kasumi_enable=1 and kasumi_charger_suppressed=0 for the
	 * duration of the test so the result is independent of the default
	 * we ship and of any charger that may have plugged in during the
	 * boot window.  Restore both on exit.
	 */
	saved_enable = READ_ONCE(kasumi_enable);
	saved_suppressed = READ_ONCE(kasumi_charger_suppressed);
	WRITE_ONCE(kasumi_enable, 1);
	WRITE_ONCE(kasumi_charger_suppressed, 0);

	/* 1. at-or-above-ceiling: must be returned unchanged. */
	r1 = kasumi_dampen(at_ceiling, NULL);
	r2 = kasumi_dampen(above_ceiling, NULL);
	if (r1 != at_ceiling || r2 != above_ceiling) {
		pr_err("kasumi: safety self-test FAILED: at_ceiling=%d->%d, above_ceiling=%d->%d (expected unchanged)\n",
		       at_ceiling, r1, above_ceiling, r2);
		ret = -EIO;
		goto out;
	}

	/* 2. below-ramp: must be exactly real - offset. */
	if (below_ramp > 0) {
		r3 = kasumi_dampen(below_ramp, NULL);
		if (r3 != below_ramp - (int)offset) {
			pr_err("kasumi: safety self-test FAILED: below_ramp=%d->%d (expected %d)\n",
			       below_ramp, r3, below_ramp - (int)offset);
			ret = -EIO;
			goto out;
		}
	}

	/* 3. zero / negative: must be passed through. */
	r4 = kasumi_dampen(0, NULL);
	if (r4 != 0) {
		pr_err("kasumi: safety self-test FAILED: zero input -> %d (expected 0)\n", r4);
		ret = -EIO;
		goto out;
	}

#ifdef CONFIG_KASUMI_DEBUG_MSG
	pr_info("kasumi: safety self-test passed (ceiling=%u offset=%u ramp=%u)\n",
		ceiling, offset, ramp);
#endif /* CONFIG_KASUMI_DEBUG_MSG */
out:
	WRITE_ONCE(kasumi_enable, saved_enable);
	WRITE_ONCE(kasumi_charger_suppressed, saved_suppressed);
	return ret;
}

void kasumi_set_charger_suppressed(bool suppressed)
{
	WRITE_ONCE(kasumi_charger_suppressed, suppressed ? 1 : 0);
}


/* ------------------------------------------------------------ */
/* Selene game detection integration                            */
/* ------------------------------------------------------------ */
static int kasumi_selene_nb_fn(struct notifier_block *nb,
                               unsigned long event, void *data)
{
	switch (event) {
	case SELENE_EVENT_GAME_START:
		kasumi_apply_profile(ZENITH_PROFILE_GAMING);
		printk(KERN_INFO "GrayRavens: kasumi: GAME_START: profile GAMING applied\n");
		break;
	case SELENE_EVENT_GAME_STOP:
		kasumi_apply_profile(ZENITH_PROFILE_BALANCED);
		printk(KERN_INFO "GrayRavens: kasumi: GAME_STOP: profile BALANCED applied\n");
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block kasumi_selene_nb = {
	.notifier_call = kasumi_selene_nb_fn,
};

static int __init kasumi_sysfs_init(void)
{
	int ret;

	ret = kasumi_safety_self_test();
	if (ret) {
		pr_err("kasumi: refusing to register sysfs (self-test failed)\n");
		return ret;
	}

	kasumi_kobj = kobject_create_and_add("kasumi", kernel_kobj);
	if (!kasumi_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(kasumi_kobj, &kasumi_attr_group);
	if (ret) {
		kobject_put(kasumi_kobj);
		kasumi_kobj = NULL;
		return ret;
	}

	/* Subscribe to Selene game detection once at init */
	{
		int sret = selene_register_notifier(&kasumi_selene_nb);

		if (sret)
			pr_warn("kasumi: selene notifier register failed (%d)\n", sret);
		else
			pr_info("kasumi: subscribed to selene game detection chain\n");
	}

	#ifdef CONFIG_KASUMI_DEBUG_MSG
	pr_info("kasumi: initialized (offset=%u ramp=%u ceiling=%u shape=%u)\n",
		READ_ONCE(kasumi_offset_mc),
		READ_ONCE(kasumi_ramp_mc),
		READ_ONCE(kasumi_ceiling_mc),
		READ_ONCE(kasumi_ramp_shape));
	#endif /* CONFIG_KASUMI_DEBUG_MSG */

	return 0;
}
late_initcall(kasumi_sysfs_init);
