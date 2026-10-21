// SPDX-License-Identifier: GPL-2.0-only
/*
 * kernel/sched/hikari.c - Hikari wake-time policy engine.
 *
 * Hikari observes per-task wake-to-run wait time and reacts via
 * three non-vruntime actuators:
 *
 *   1. uclamp_min boost on the waking task for a short window
 *   2. cpufreq frequency floor hint published via an atomic
 *      notifier chain (a governor like cpufreq_zenith may
 *      subscribe to it)
 *   3. big.LITTLE wake-up CPU placement steering
 *
 * It does NOT modify vruntime, prio, or any CFS fairness state.
 * BORE retains full authority over fairness.
 *
 * Default at runtime is disabled.  When enabled, only tasks that
 * have been opted in (via /proc/<pid>/hikari_enable, top-app
 * cgroup auto-opt-in, or the kernel API) see any actuator fire on
 * them.
 *
 * The hot-path entry points (hikari_on_enqueue / hikari_on_dequeue
 * / hikari_on_wake_up / hikari_select_cpu) early-return on a single
 * READ_ONCE of the master enable flag, so when Hikari is off the
 * cost is one predictable branch.
 *
 * Concurrency model:
 *   - Per-task fields (sched.h slots 2/3) are touched only while
 *     the task's rq lock is held (enqueue/dequeue/wake hooks all
 *     run under it), or while the task is current on its own CPU.
 *     READ_ONCE/WRITE_ONCE without atomics is sufficient.
 *   - Per-CPU state lives in DEFINE_PER_CPU and is touched only
 *     from the matching CPU's scheduler hot path, or under
 *     this_cpu_ptr()/per_cpu_ptr() with appropriate care from
 *     other CPUs for read-only observability reads.
 *   - Sysctl values are plain u32 read with READ_ONCE on hot
 *     paths.  No locking required.
 *   - Kill flag uses atomic_t for cross-CPU consistency.
 *
 * Init ordering (see hikari_init()) mirrors the Zenith hardening
 * pattern: every piece of state is initialised before the master
 * `init_complete` flag is set; every hot path reads `init_complete`
 * first.
 */

#define pr_fmt(fmt) "hikari: " fmt

#include <linux/atomic.h>
#include <linux/cgroup.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/hikari.h>
#include <linux/selene.h>
#include <linux/zenith_profiles.h>

/*
 * Kasumi cross-link: read the currently applied thermal dampening
 * offset to detect when the frequency floor and dampening are
 * hiding heat buildup from each other.  When the offset exceeds
 * 15 C (15000 mc) the force floor is reduced proportionally.
 *
 * Guarded with IS_ENABLED(CONFIG_KASUMI) at the call site so the
 * extern is only referenced when Kasumi is compiled in.
 */
extern int kasumi_get_applied_offset_mc(void) __maybe_unused;
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/jump_label.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/notifier.h>
#include <linux/percpu.h>
#include <linux/pid.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/signal.h>
#include <linux/sched/topology.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/sysctl.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "sched.h"

#define CREATE_TRACE_POINTS
#include <trace/events/hikari.h>
#undef CREATE_TRACE_POINTS

/*
 * Per-CPU state.  All u64 timestamps are jiffies values, compared
 * with time_after / time_before helpers so wraparound is handled.
 *
 * wake_demand_ewma_ns is a u32 nanoseconds EWMA, saturating at
 * U32_MAX.  ~4.29s upper bound -- any task waiting longer than
 * that on enqueue almost certainly isn't a task we want to chase.
 */
/*
 * Wake-time histogram bucket count.  Eight is enough headroom to
 * separate (sub-100us, common idle wakes) from (100us..250us, the
 * 'should boost' band) from (>10ms, contended).  See
 * hikari_wake_hist_edges_us[] below for the cuts.
 */
#define HIKARI_WAKE_HIST_BUCKETS	8

struct hikari_pcpu {
	u32		wake_demand_ewma_ns;
	unsigned int	wake_floor_khz;
	unsigned long	wake_floor_until_jiffies;
	unsigned long	audio_active_until_jiffies;
	atomic_t	boost_count;
	atomic_t	hint_count;
	/*
	 * Per-CPU wake-to-run wait-time histogram.  Updated from
	 * hikari_on_dequeue() with the delta we just measured for
	 * the task being picked.  atomic_long_t so a read across
	 * all CPUs sums cleanly without a lock; writes are always
	 * to the local CPU's slot so there is no cross-CPU
	 * cache-line bouncing.
	 */
	atomic_long_t	wake_hist[HIKARI_WAKE_HIST_BUCKETS];
};

/*
 * Open-upper bucket edges in microseconds.  A sample in nanoseconds
 * is divided by 1000 then placed into the first bucket whose edge
 * exceeds it; the final bucket (10ms+) catches everything else.
 *
 * Edges chosen to cover the bands hikari cares about:
 *   <100us   -- background idle wakes, nothing to chase
 *   100..250 -- audio-grade wakes, worth boosting
 *   250..500 -- foreground touch wakes
 *   500..1ms -- borderline, mostly contended
 *   1..2ms   -- around the default wake_threshold_us
 *   2..5ms   -- noticeable jank
 *   5..10ms  -- bad
 *   10ms+    -- catastrophic
 */
static const u32 hikari_wake_hist_edges_us[HIKARI_WAKE_HIST_BUCKETS - 1] = {
	100, 250, 500, 1000, 2000, 5000, 10000,
};

static DEFINE_PER_CPU(struct hikari_pcpu, hikari_pcpu);

/*
 * Per-task state side table.
 *
 * Android GKI freezes struct task_struct: any layout change shifts
 * the KMI CRC of every exported symbol whose type graph reaches
 * task_struct.  Hikari therefore keeps its four per-task values in
 * this statically allocated, spinlock-protected hash table keyed by
 * task pointer instead of in task_struct itself.
 *
 * Entries are handed out from a fixed pool on first touch (hot
 * paths never sleep and never allocate).  When the pool is
 * exhausted the least-recently-used entry is evicted and reused, so
 * the table size stays bounded and the feature can never silently
 * die.  Lookups validate the task pointer AND its pid so a recycled
 * task_struct address cannot inherit stale state from a dead task.
 */
#define HIKARI_TSTATE_MAX	1024

struct hikari_tstate {
	struct task_struct	*task;
	pid_t			pid;
	u32			flags;
	u32			wait_ewma_ns;
	u32			last_enqueue_ns;
	u32			boost_until_ns;
	struct hlist_node	hnode;
	struct list_head	free;
};

static DEFINE_SPINLOCK(hikari_tstate_lock);
static DEFINE_HASHTABLE(hikari_tstate_table, 8);
static LIST_HEAD(hikari_tstate_free);
static struct hikari_tstate hikari_tstate_pool[HIKARI_TSTATE_MAX];

static void __init hikari_tstate_init(void)
{
	int i;

	for (i = 0; i < HIKARI_TSTATE_MAX; i++)
		list_add(&hikari_tstate_pool[i].free, &hikari_tstate_free);
}

/*
 * Look up (or lazily create) the per-task state entry for @p.
 * Returns NULL only when the fixed pool is exhausted.  Safe to call
 * from scheduler hot paths: spinlock + hash probe, no sleeping.
 */
static struct hikari_tstate *hikari_state(struct task_struct *p)
{
	struct hikari_tstate *st, *victim = NULL;
	unsigned long flags;
	u32 oldest;
	int bkt;

	if (!p)
		return NULL;

	spin_lock_irqsave(&hikari_tstate_lock, flags);
	hash_for_each_possible(hikari_tstate_table, st, hnode,
			       (unsigned long)p) {
		if (st->task == p && st->pid == p->pid) {
			spin_unlock_irqrestore(&hikari_tstate_lock, flags);
			return st;
		}
	}
	st = list_first_entry_or_null(&hikari_tstate_free,
				      struct hikari_tstate, free);
	if (st) {
		list_del_init(&st->free);
	} else {
		/*
		 * Pool exhausted: evict the least-recently-used entry.
		 * Never-enqueued entries (last_enqueue_ns == 0) are
		 * preferred victims.  The evicted task loses its boost
		 * hints at worst -- graceful degradation.
		 */
		oldest = ~0U;
		hash_for_each(hikari_tstate_table, bkt, st, hnode) {
			if (st->last_enqueue_ns <= oldest) {
				oldest = st->last_enqueue_ns;
				victim = st;
			}
		}
		/*
		 * Free list empty AND table empty: pool not yet
		 * populated (pre-init call).  Never touch NULL.
		 */
		if (!victim) {
			spin_unlock_irqrestore(&hikari_tstate_lock, flags);
			return NULL;
		}
		st = victim;
		hash_del(&st->hnode);
	}
	memset(st, 0, sizeof(*st));
	st->task = p;
	st->pid = p->pid;
	hash_add(hikari_tstate_table, &st->hnode, (unsigned long)p);
	spin_unlock_irqrestore(&hikari_tstate_lock, flags);
	return st;
}

/*
 * Sysctl-backed tunables.  All u32, all proc_douintvec_minmax.
 *
 * The "_value" suffix on hikari_enable_value is intentional: the
 * accessor hikari_enabled() reads it via READ_ONCE.  Sysctl writes
 * pass through proc_douintvec_minmax which is a WRITE_ONCE-like
 * store on the slow path.
 */
static unsigned int hikari_enable_value = 1;
static unsigned int hikari_wake_threshold_us = 1000;
static unsigned int hikari_uclamp_boost_pct = 30;
static unsigned int hikari_uclamp_ttl_ms = 16;
/*
 * uclamp_max ceiling for HIKARI_FLAG_BACKGROUND tasks.  Expressed as
 * a percentage of SCHED_CAPACITY_SCALE (1024 on arm64); 0 means "no
 * ceiling, apply the platform uclamp_max as usual".  Sticky -- there
 * is no TTL because background-ness is a state, not an event.  A
 * task only sees this ceiling when its HIKARI_FLAG_BACKGROUND bit is
 * set; clearing the bit removes the ceiling on the next uclamp_eff
 * read.
 *
 * Bounded 0..100; the apply path clamps to SCHED_CAPACITY_SCALE
 * anyway for paranoia.
 */
static unsigned int hikari_uclamp_max_pct;
static unsigned int hikari_floor_khz_cluster0 = 800000;
static unsigned int hikari_floor_khz_cluster1 = 1200000;
static unsigned int hikari_floor_ttl_ms = 50;
/*
 * Per-cluster floor-TTL overrides.  Zero (default) means "fall back to
 * the shared hikari_floor_ttl_ms above".  Non-zero overrides the global
 * for that cluster only -- so an admin can hold the silvers' wake-floor
 * hint longer than the golds' (or vice-versa) without disturbing the
 * single-value semantics anyone already shipping a tunables.cfg
 * depends on.  Bounded 0..hikari_floor_ttl_max, validated the same
 * way the shared value is.
 */
static unsigned int hikari_floor_ttl_ms_big;
static unsigned int hikari_floor_ttl_ms_little;
/*
 * Always-on per-cluster floor expressed as a percentage of the
 * cluster's cpuinfo_max_freq.  Distinct from hikari_floor_khz_cluster*
 * (which is a flat kHz used only during a wake-floor TTL window): a
 * non-zero value here means "never let the published floor for this
 * cluster drop below pct% of cpuinfo_max_freq, regardless of whether
 * there was a recent wake event".  Zero (default) preserves the
 * pre-existing wake-only behaviour byte-for-byte.
 *
 * The percentage is converted to kHz lazily into the matching
 * hikari_force_floor_khz_* cache below, so the hot path
 * (hikari_get_floor_khz) does only an unsigned compare.
 */
static unsigned int hikari_force_floor_pct_big;
static unsigned int hikari_force_floor_pct_little;
static unsigned int hikari_force_floor_khz_big __read_mostly;
static unsigned int hikari_force_floor_khz_little __read_mostly;
static unsigned int hikari_placement_enable;

/*
 * Cross-subsystem signal: jiffies timestamp of the most recent
 * wake-demand publish across all CPUs.  Iyashi reads this to
 * know whether the scheduler is seeing sustained high-EWMA
 * tasks, and optionally raises the thermal performance floor
 * while wake-demand is active.  Lockless: updated with
 * WRITE_ONCE on the publish path, read with READ_ONCE.
 */
static unsigned long hikari_last_demand_jiffies __read_mostly;
/*
 * Per-task uclamp learning tunables.
 *
 * When a task's wake-to-run EWMA consistently exceeds the wake
 * threshold (hikari_wake_threshold_us), the task is promoted to
 * HIKARI_FLAG_LEARNED, which grants a persistent uclamp_min boost
 * (learn_boost_pct) as long as the EWMA stays high.  No userspace
 * intervention needed -- it learns and forgets automatically.
 *
 * learn_boost_pct:  uclamp_min percentage for learned tasks (0=off)
 * learn_forget_ratio: EWMA must drop below threshold/this to forget
 *                     (default 2 = threshold/2)
 */
static unsigned int hikari_learn_boost_pct = 20;
static unsigned int hikari_learn_forget_ratio = 2;
static unsigned int hikari_audio_intensify = 1;
static unsigned int hikari_topapp_auto_optin = 1;
static unsigned int hikari_topapp_auto_optout;

/* Selene game detection boost — set/cleared by selene notifier callback */
static bool hikari_game_selene_boost __read_mostly;
static unsigned int hikari_selene_floor_khz __read_mostly;
static unsigned int hikari_ewma_shift = 4;

static const unsigned int hikari_uint_zero = 0;
static const unsigned int hikari_uint_one  = 1;
static const unsigned int hikari_threshold_min = 100;
static const unsigned int hikari_threshold_max = 100000;
static const unsigned int hikari_boost_pct_max = 50;
static const unsigned int hikari_uclamp_ttl_min = 1;
static const unsigned int hikari_uclamp_ttl_max = 200;
static const unsigned int hikari_uclamp_max_pct_max = 100;
static const unsigned int hikari_floor_khz_max_c0 = 2000000;
static const unsigned int hikari_floor_khz_max_c1 = 3000000;
static const unsigned int hikari_floor_ttl_min = 1;
static const unsigned int hikari_floor_ttl_max = 500;
/*
 * Per-cluster override minimum is 0 (= "use the shared value") rather
 * than 1 like the shared value's, so an admin can clear the override
 * without disabling the floor entirely.
 */
static const unsigned int hikari_floor_ttl_override_min = 0;
static const unsigned int hikari_learn_boost_pct_max = 50;
static const unsigned int hikari_learn_forget_ratio_min = 2;
static const unsigned int hikari_learn_forget_ratio_max = 10;
static const unsigned int hikari_ewma_shift_min = 1;
static const unsigned int hikari_ewma_shift_max = 7;

/*
 * Master state.
 *
 * init_complete is set ONCE, after every piece of init has run
 * and is visible to all CPUs.  Hot paths read it first; reading a
 * stale "false" value just means we early-return early in boot.
 * Reading a stale "true" value never happens because the store is
 * the last init step.
 *
 * kill_flag is set if self-disable triggers.  Once set it is
 * never cleared without a reboot; the master enable sysctl can be
 * flipped back to 1 by an admin who wants to clear the lockout
 * explicitly (see hikari_enable_sysctl_handler).
 */
static bool			hikari_init_complete __read_mostly;
static atomic_t			hikari_kill_flag = ATOMIC_INIT(0);
static u8			hikari_disable_reason_global __read_mostly;

/*
 * Static key gate.  Mirrors hikari_enabled() exactly but compiles to a
 * single unlikely-branch on the hot path while disabled (init not
 * complete, kill flag set, or hikari_enable_value == 0).
 *
 * Flipped from three sites:
 *   - hikari_init() turns the key on after init_complete is set, if
 *     hikari_enable_value != 0 and kill_flag == 0.
 *   - hikari_self_disable() turns the key off (kill_flag now set).
 *   - hikari_enable_sysctl_handler() turns the key on/off to match
 *     the new value (and clears kill_flag if the admin re-enables).
 *
 * All three call sites run in process context, so the slow-path
 * static_branch_enable/disable() blockers (text_poke) are safe.
 */
static DEFINE_STATIC_KEY_FALSE(hikari_active_key);

static inline void hikari_active_key_sync(void)
{
	bool want = READ_ONCE(hikari_init_complete) &&
		    atomic_read(&hikari_kill_flag) == 0 &&
		    READ_ONCE(hikari_enable_value) != 0;

	if (want && !static_key_enabled(&hikari_active_key.key))
		static_branch_enable(&hikari_active_key);
	else if (!want && static_key_enabled(&hikari_active_key.key))
		static_branch_disable(&hikari_active_key);
}
static cpumask_t		hikari_big_cluster __read_mostly;
static cpumask_t		hikari_little_cluster __read_mostly;
static int			hikari_max_big_cpu __read_mostly = -1;

static ATOMIC_NOTIFIER_HEAD(hikari_cpufreq_chain);

static inline bool hikari_is_killed(void)
{
	return atomic_read(&hikari_kill_flag) != 0;
}

static inline void hikari_self_disable(u8 reason)
{
	/* Cmpxchg-ish: only the first failing path records the reason. */
	if (atomic_xchg(&hikari_kill_flag, 1) == 0) {
		hikari_disable_reason_global = reason;
		pr_err_once("self-disabled (reason=%u)\n", reason);
		/*
		 * static_branch_disable() needs process context, but this
		 * helper is called from hot paths (e.g. scheduler hooks).
		 * The kill flag itself is already authoritative -- the
		 * static key resync happens lazily next time the sysctl
		 * handler runs, or stays slightly stale until reboot.
		 * Hot-path consumers re-check hikari_enabled() anyway,
		 * which reads the kill flag directly.
		 */
#ifdef CONFIG_HIKARI_DEBUG
		BUG();
#endif
	}
}

bool hikari_enabled(void)
{
	if (!static_branch_unlikely(&hikari_active_key))
		return false;
	if (hikari_is_killed())
		return false;
	return READ_ONCE(hikari_enable_value) != 0;
}
EXPORT_SYMBOL_GPL(hikari_enabled);

/*
 * Truncate jiffies to u32 for storage in task_struct slot.
 * On HZ=250 this wraps every ~198 days, which exceeds any
 * reasonable boost TTL by orders of magnitude.
 */
static inline u32 hikari_now_token(void)
{
	return (u32)((unsigned long)jiffies);
}

static inline bool hikari_token_active(u32 expiry_token)
{
	u32 now = hikari_now_token();

	if (!expiry_token)
		return false;
	return (s32)(expiry_token - now) > 0;
}

static inline u32 hikari_token_add_ms(unsigned int ms)
{
	u32 ticks = (u32)msecs_to_jiffies(ms);
	u32 expiry = hikari_now_token() + ticks;

	/* avoid the "0 means unset" sentinel */
	if (!expiry)
		expiry = 1;
	return expiry;
}

/*
 * Forward declaration -- the setter API block (further down) defines
 * the cmpxchg-based update.  Used from the hot-path stamp sites in
 * hikari_on_dequeue() and hikari_select_cpu() so a /proc reader can
 * tell why Hikari skipped a specific task on its most recent call.
 */
static inline void hikari_set_skip_reason(struct task_struct *p, u32 reason);

static inline bool hikari_task_active(struct task_struct *p)
{
	struct hikari_tstate *st;
	u32 flags;

	if (!hikari_enabled())
		return false;
	st = hikari_state(p);
	if (!st)
		return false;
	flags = READ_ONCE(st->flags);
	return (flags & HIKARI_FLAG_OPT_IN) != 0;
}

static void hikari_set_flag(struct task_struct *p, u32 bit, bool on);

static inline bool hikari_in_top_app(struct task_struct *p)
{
	struct hikari_tstate *st = hikari_state(p);

	return st && (READ_ONCE(st->flags) & HIKARI_FLAG_FOREGROUND);
}

/*
 * Lazy top-app cgroup auto-opt-in.  Called from hikari_on_enqueue()
 * under rq_lock, where p->sched_task_group is stable.
 *
 * Walks task_group(p)->css.cgroup->kn->name and compares to
 * "top-app".  Cost: one pointer chain + 7-byte strcmp, only when
 * hikari_topapp_auto_optin is set.
 *
 * State transitions:
 *   enter top-app  -> set FOREGROUND + OPT_IN
 *   leave top-app  -> clear FOREGROUND only (OPT_IN may be
 *                     user-set via /proc and must survive)
 */
static inline void hikari_lazy_topapp_update(struct task_struct *p)
{
#ifdef CONFIG_CGROUP_SCHED
	struct task_group *tg;
	struct cgroup *cgrp;
	struct hikari_tstate *st;
	bool in_topapp;
	u32 flags;

	if (!READ_ONCE(hikari_topapp_auto_optin))
		return;

	tg = task_group(p);
	if (!tg)
		return;
	cgrp = tg->css.cgroup;
	if (!cgrp || !cgrp->kn || !cgrp->kn->name)
		return;

	st = hikari_state(p);
	if (!st)
		return;

	in_topapp = (strcmp(cgrp->kn->name, "top-app") == 0);
	flags = READ_ONCE(st->flags);

	if (in_topapp && (!(flags & HIKARI_FLAG_FOREGROUND) ||
			  (flags & HIKARI_FLAG_BACKGROUND)))
		hikari_mark_foreground(p, true);
	else if (!in_topapp && (flags & HIKARI_FLAG_FOREGROUND)) {
		u32 clear = HIKARI_FLAG_FOREGROUND;

		if (READ_ONCE(hikari_topapp_auto_optout))
			clear |= HIKARI_FLAG_OPT_IN;
		hikari_set_flag(p, clear, false);
	}
#endif
}

static inline unsigned int hikari_uclamp_boost_value(void)
{
	unsigned int pct = READ_ONCE(hikari_uclamp_boost_pct);

	if (pct > hikari_boost_pct_max)
		pct = hikari_boost_pct_max;
	return (SCHED_CAPACITY_SCALE * pct) / 100;
}

static inline unsigned int hikari_learn_boost_value(void)
{
	unsigned int pct = READ_ONCE(hikari_learn_boost_pct);

	if (pct > hikari_learn_boost_pct_max)
		pct = hikari_learn_boost_pct_max;
	return (SCHED_CAPACITY_SCALE * pct) / 100;
}

/*
 * Returns the additional uclamp_min that Hikari is currently
 * boosting on this task, or 0 if no boost is active.  Safe to call
 * from the uclamp_eff_get() hot path.
 */
/*
 * Per-task uclamp_max ceiling for HIKARI_FLAG_BACKGROUND tasks.
 * Returns the SCHED_CAPACITY-scaled clamp value, or 0 when no
 * ceiling should be applied.  Cheap on the hot path -- two reads
 * and a flag check; SCHED_CAPACITY_SCALE is the divisor so this
 * compiles to a simple shift+mul on most archs.
 *
 * The order of checks here matters: the percentage tunable is the
 * gate, so a 0 there short-circuits before we even read the
 * per-task flags.  Lets the master switch ('off') cost a single
 * READ_ONCE plus a branch.
 */
unsigned int hikari_uclamp_max_ceiling(struct task_struct *p)
{
	struct hikari_tstate *st;
	unsigned int pct;
	unsigned int ceiling;

	if (!hikari_enabled())
		return 0;
	if (!p)
		return 0;

	pct = READ_ONCE(hikari_uclamp_max_pct);
	if (!pct)
		return 0;
	if (pct > 100)
		pct = 100;

	st = hikari_state(p);
	if (!st || !(READ_ONCE(st->flags) & HIKARI_FLAG_BACKGROUND))
		return 0;

	ceiling = (SCHED_CAPACITY_SCALE * pct) / 100U;
	if (ceiling > SCHED_CAPACITY_SCALE)
		ceiling = SCHED_CAPACITY_SCALE;
	return ceiling;
}
EXPORT_SYMBOL_GPL(hikari_uclamp_max_ceiling);

unsigned int hikari_uclamp_boost_amount(struct task_struct *p)
{
	struct hikari_tstate *st;
	u32 until;

	if (!IS_ENABLED(CONFIG_HIKARI_UCLAMP))
		return 0;
	if (!hikari_enabled())
		return 0;
	if (!p)
		return 0;

	st = hikari_state(p);
	if (!st)
		return 0;

	/*
	 * Per-task uclamp learning: tasks that consistently show high
	 * wake-to-run latency get a persistent uclamp_min boost via
	 * HIKARI_FLAG_LEARNED.  Check this first -- it's the fastest
	 * path (one READ_ONCE + bit test) and returns immediately for
	 * flagged tasks regardless of TTL state.
	 */
	if (READ_ONCE(st->flags) & HIKARI_FLAG_LEARNED)
		return hikari_learn_boost_value();

	until = READ_ONCE(st->boost_until_ns);
	if (!hikari_token_active(until)) {
		/* Lazy clear: avoid storing through a hot read but on
		 * the next non-hot visit we'll zero it.
		 */
		return 0;
	}
	return hikari_uclamp_boost_value();
}
EXPORT_SYMBOL_GPL(hikari_uclamp_boost_amount);

static inline void hikari_apply_uclamp_boost(struct task_struct *p)
{
	struct hikari_tstate *st;

	if (!IS_ENABLED(CONFIG_HIKARI_UCLAMP))
		return;
	st = hikari_state(p);
	if (!st)
		return;
	WRITE_ONCE(st->boost_until_ns,
		   hikari_token_add_ms(READ_ONCE(hikari_uclamp_ttl_ms)));
	atomic_inc(&this_cpu_ptr(&hikari_pcpu)->boost_count);
}

/*
 * Bucket a wake-to-run delta into the per-CPU wake_hist[] array.
 * Called from hikari_on_dequeue() *after* the wraparound and zero
 * checks that already validate the delta -- no further bounds
 * checks are needed here.  Writes always land on the local CPU,
 * so atomic_long_inc() does not bounce a cache line; we use the
 * atomic variant so the cross-CPU read summing in wake_hist_show
 * is safe without a lock.
 */
static inline void hikari_wake_hist_record(u32 delta_ns)
{
	struct hikari_pcpu *pc = this_cpu_ptr(&hikari_pcpu);
	u32 us = delta_ns / 1000U;
	int i;

	for (i = 0; i < HIKARI_WAKE_HIST_BUCKETS - 1; i++) {
		if (us < hikari_wake_hist_edges_us[i]) {
			atomic_long_inc(&pc->wake_hist[i]);
			return;
		}
	}
	atomic_long_inc(&pc->wake_hist[HIKARI_WAKE_HIST_BUCKETS - 1]);
}

/*
 * Resolve the floor TTL for a specific CPU: per-cluster override if
 * set, otherwise the shared hikari_floor_ttl_ms.  Mirrors the
 * per-cluster floor_khz lookup directly above so both knobs travel
 * together.
 */
static inline unsigned int hikari_floor_ttl_ms_for_cpu(unsigned int cpu)
{
	unsigned int override;

	override = cpumask_test_cpu(cpu, &hikari_big_cluster)
		   ? READ_ONCE(hikari_floor_ttl_ms_big)
		   : READ_ONCE(hikari_floor_ttl_ms_little);
	if (override)
		return override;
	return READ_ONCE(hikari_floor_ttl_ms);
}

static inline void hikari_publish_freq_hint(unsigned int cpu, u32 demand_ns)
{
	struct hikari_freq_hint hint;
	unsigned int floor;

	if (!IS_ENABLED(CONFIG_HIKARI_ZENITH_HINT))
		return;
	if (cpu >= nr_cpu_ids)
		return;

	floor = cpumask_test_cpu(cpu, &hikari_big_cluster)
		? READ_ONCE(hikari_floor_khz_cluster1)
		: READ_ONCE(hikari_floor_khz_cluster0);

	if (!floor)
		return;

	hint.cpu       = cpu;
	hint.floor_khz = floor;
	hint.ttl_ms    = hikari_floor_ttl_ms_for_cpu(cpu);
	hint.demand_ns = demand_ns;

	atomic_inc(&per_cpu_ptr(&hikari_pcpu, cpu)->hint_count);
	WRITE_ONCE(per_cpu_ptr(&hikari_pcpu, cpu)->wake_floor_khz, floor);
	WRITE_ONCE(per_cpu_ptr(&hikari_pcpu, cpu)->wake_floor_until_jiffies,
		   jiffies + msecs_to_jiffies(hint.ttl_ms));

	WRITE_ONCE(hikari_last_demand_jiffies, jiffies);

	if (trace_hikari_freq_hint_enabled())
		trace_hikari_freq_hint(cpu, floor, hint.ttl_ms, demand_ns);

	atomic_notifier_call_chain(&hikari_cpufreq_chain,
				   HIKARI_NOTIFIER_WAKE_DEMAND, &hint);
}

/*
 * Resolve cpuinfo_max_freq for the named cluster by peeking at any
 * online CPU in it.  Returns 0 if cpufreq is not ready yet or the
 * cluster is empty.  Takes (and releases) a cpufreq policy ref, so
 * cheap-but-not-free -- only called from the writer path below.
 */
static unsigned int hikari_cluster_max_khz(const struct cpumask *cluster)
{
	struct cpufreq_policy *policy;
	unsigned int cpu, max_khz = 0;

	for_each_cpu(cpu, cluster) {
		if (!cpu_online(cpu))
			continue;
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		max_khz = policy->cpuinfo.max_freq;
		cpufreq_cpu_put(policy);
		if (max_khz)
			break;
	}
	return max_khz;
}

/*
 * Recompute hikari_force_floor_khz_{big,little} from the corresponding
 * _pct knobs and the cluster's current cpuinfo_max_freq.  Called from
 * the sysfs/sysctl store handlers when the pct changes, and once from
 * a deferred work item at first opt-in to handle the cpufreq-not-ready
 * boot race.  Cheap and idempotent.
 */
static void hikari_recompute_force_floors(void)
{
	unsigned int big_max, little_max, big_pct, little_pct;

	big_max    = hikari_cluster_max_khz(&hikari_big_cluster);
	little_max = hikari_cluster_max_khz(&hikari_little_cluster);
	big_pct    = READ_ONCE(hikari_force_floor_pct_big);
	little_pct = READ_ONCE(hikari_force_floor_pct_little);

	WRITE_ONCE(hikari_force_floor_khz_big,
		   big_max ? (unsigned int)((u64)big_max * big_pct / 100U) : 0);
	WRITE_ONCE(hikari_force_floor_khz_little,
		   little_max ? (unsigned int)((u64)little_max * little_pct / 100U) : 0);
}

unsigned int hikari_get_floor_khz(unsigned int cpu)
{
	struct hikari_pcpu *pc;
	unsigned long until;
	unsigned int khz = 0;
	unsigned int force_khz;
	bool big;

	if (!IS_ENABLED(CONFIG_HIKARI_ZENITH_HINT))
		return 0;
	if (!hikari_enabled())
		return 0;
	if (cpu >= nr_cpu_ids)
		return 0;

	pc = per_cpu_ptr(&hikari_pcpu, cpu);
	until = READ_ONCE(pc->wake_floor_until_jiffies);
	if (until && !time_after_eq(jiffies, until))
		khz = READ_ONCE(pc->wake_floor_khz);

	/*
	 * Always-on force-floor: independent of the wake-floor TTL.
	 * The cached kHz value is recomputed lazily by the writer path
	 * (hikari_recompute_force_floors), so the hot path is one
	 * compare + one max.
	 */
	big = cpumask_test_cpu(cpu, &hikari_big_cluster);
	force_khz = big ? READ_ONCE(hikari_force_floor_khz_big)
			: READ_ONCE(hikari_force_floor_khz_little);
	if (force_khz > khz)
		khz = force_khz;

	/* Selene game boost: apply floor when game is detected */
	if (READ_ONCE(hikari_game_selene_boost)) {
		unsigned int game_khz = READ_ONCE(hikari_selene_floor_khz);
		if (game_khz > khz)
			khz = game_khz;
	}

	return khz;
}
EXPORT_SYMBOL_GPL(hikari_get_floor_khz);

/*
 * Mark the CPU `p` is currently running on as audio-active for
 * the next ttl_ms (using the floor TTL for symmetry).  Called
 * from hikari_mark_audio() and from the scheduler when we
 * observe an audio-tagged task being enqueued.
 */
static inline void hikari_pcpu_mark_audio(unsigned int cpu)
{
	if (cpu >= nr_cpu_ids)
		return;
	WRITE_ONCE(per_cpu_ptr(&hikari_pcpu, cpu)->audio_active_until_jiffies,
		   jiffies + msecs_to_jiffies(hikari_floor_ttl_ms_for_cpu(cpu)));
}

static inline bool hikari_cpu_is_audio_active(unsigned int cpu)
{
	unsigned long until;

	if (cpu >= nr_cpu_ids)
		return false;
	until = READ_ONCE(per_cpu_ptr(&hikari_pcpu, cpu)->audio_active_until_jiffies);
	return until && time_before(jiffies, until);
}

/* ------------------------------------------------------------ */
/* Hot-path hooks called from the scheduler.                    */
/* ------------------------------------------------------------ */

void hikari_on_enqueue(struct task_struct *p, struct rq *rq)
{
	struct hikari_tstate *st;
	u32 now32;

	if (!hikari_enabled())
		return;
	if (!p || !rq)
		return;

	hikari_lazy_topapp_update(p);

	st = hikari_state(p);
	if (!st || !(READ_ONCE(st->flags) & HIKARI_FLAG_OPT_IN))
		return;

	now32 = (u32)rq_clock_task(rq);
	WRITE_ONCE(st->last_enqueue_ns, now32 ? now32 : 1);

	/*
	 * Lazy boost-expiry clear: if a stale "active" boost from
	 * before a long sleep is still set, clear it now.  This is
	 * the only safety against u32-jiffies wraparound for
	 * boost_until.
	 */
	if (st->boost_until_ns &&
	    !hikari_token_active(st->boost_until_ns))
		WRITE_ONCE(st->boost_until_ns, 0);

	if (READ_ONCE(st->flags) & HIKARI_FLAG_AUDIO_TAGGED)
		hikari_pcpu_mark_audio(task_cpu(p));
}
EXPORT_SYMBOL_GPL(hikari_on_enqueue);

void hikari_on_dequeue(struct task_struct *p, struct rq *rq)
{
	struct hikari_tstate *st;
	u32 last, now32, delta, ewma, threshold_ns;

	if (!hikari_task_active(p)) {
		/*
		 * Stamp why so /proc/<pid>/hikari_status can report it.
		 * hikari_task_active() returns false for three reasons; pick
		 * the most specific one we can tell apart cheaply.
		 */
		if (p) {
			if (!hikari_enabled())
				hikari_set_skip_reason(p,
					HIKARI_SKIP_GLOBAL_DISABLED);
			else
				hikari_set_skip_reason(p,
					HIKARI_SKIP_NOT_OPTED_IN);
		}
		return;
	}
	if (!rq)
		return;

	/*
	 * RT and deadline tasks already preempt CFS and ignore
	 * uclamp_min boosts, so there is nothing for Hikari to
	 * improve -- skip the measurement and actuators entirely.
	 */
	if (rt_task(p) || dl_task(p))
		return;

	st = hikari_state(p);
	if (!st)
		return;

	last = READ_ONCE(st->last_enqueue_ns);
	if (!last)
		return;

	now32 = (u32)rq_clock_task(rq);
	/*
	 * Signed wraparound-safe subtraction.  delta is "time
	 * spent waiting on the rq from enqueue to dequeue (pick)".
	 * If negative or zero, ignore (clock skew during migration
	 * or a degenerate sample).
	 */
	if ((s32)(now32 - last) <= 0)
		return;
	delta = now32 - last;

	/*
	 * EWMA with tunable alpha = 1/(1<<shift).  Default shift=3
	 * gives alpha=1/8: new = (7*old + delta) / 8.  Lower shift
	 * = more reactive, higher = lazier smoothing.
	 * Saturates at U32_MAX naturally because all values are u32.
	 */
	{
		unsigned int shift = READ_ONCE(hikari_ewma_shift);
		u32 weight;
		u64 sum;
		u64 next;

		if (shift < hikari_ewma_shift_min)
			shift = hikari_ewma_shift_min;
		if (shift > hikari_ewma_shift_max)
			shift = hikari_ewma_shift_max;
		weight = (1u << shift) - 1;

		ewma = READ_ONCE(st->wait_ewma_ns);
		if (unlikely(!ewma)) {
			/* Fast-start: first sample lands immediately instead of
			 * decaying from zero over ~8 wake events.
			 */
			ewma = delta > U32_MAX ? U32_MAX : delta;
		} else {
			sum = ((u64)ewma * weight) + delta;
			next = sum >> shift;
			ewma = next > U32_MAX ? U32_MAX : (u32)next;
		}
	}
	WRITE_ONCE(st->wait_ewma_ns, ewma);
	WRITE_ONCE(st->last_enqueue_ns, 0);

	/*
	 * Histogram the *current sample* (not the EWMA) so the
	 * shape of /sys/kernel/hikari/wake_hist reflects raw
	 * wake-to-run wait distribution and not the smoothing.
	 */
	hikari_wake_hist_record(delta);

	threshold_ns = READ_ONCE(hikari_wake_threshold_us);
	if (threshold_ns > U32_MAX / 1000)
		threshold_ns = U32_MAX / 1000;
	threshold_ns *= 1000;

if (ewma > threshold_ns) {
		if (IS_ENABLED(CONFIG_HIKARI_UCLAMP)) {
			hikari_apply_uclamp_boost(p);

			/*
			 * Per-task uclamp learning: if this task
			 * consistently exceeds the wake threshold,
			 * promote it to LEARNED for a persistent
			 * uclamp_min boost.  The EWMA itself is the
			 * gate -- no extra counter needed.  A task
			 * with sustained high latency will hit this
			 * branch on every dequeue; a bursty task
			 * will get the TTL-limited boost instead.
			 */
			if (READ_ONCE(hikari_learn_boost_pct) > 0 &&
			    !(READ_ONCE(st->flags) & HIKARI_FLAG_LEARNED))
				hikari_set_flag(p, HIKARI_FLAG_LEARNED, true);
		}

		hikari_publish_freq_hint(task_cpu(p), ewma);
		hikari_set_skip_reason(p, HIKARI_SKIP_ACTIONED);
	} else {
		/*
		 * Forgetting: when EWMA has dropped significantly
		 * (below threshold/forget_ratio), clear the learned
		 * flag so the task stops getting a permanent boost.
		 */
		if ((READ_ONCE(st->flags) & HIKARI_FLAG_LEARNED) &&
		    ewma < threshold_ns / max(READ_ONCE(hikari_learn_forget_ratio), 2u))
			hikari_set_flag(p, HIKARI_FLAG_LEARNED, false);

		hikari_set_skip_reason(p, HIKARI_SKIP_EWMA_LOW);
	}
}
EXPORT_SYMBOL_GPL(hikari_on_dequeue);

void hikari_on_wake_up(struct task_struct *p, int target_cpu)
{
	struct hikari_tstate *st;

	if (!hikari_task_active(p))
		return;
	if (target_cpu < 0 || target_cpu >= nr_cpu_ids)
		return;

	st = hikari_state(p);
	if (st && (READ_ONCE(st->flags) & HIKARI_FLAG_AUDIO_TAGGED))
		hikari_pcpu_mark_audio(target_cpu);
}
EXPORT_SYMBOL_GPL(hikari_on_wake_up);

/*
 * Big.LITTLE placement.  Engage condition:
 *
 *   placement_enable=1
 *   AND task is opted in
 *   AND task's recent EWMA exceeds the wake threshold
 *   AND (task is foreground OR a sibling CPU is audio-active)
 *
 * If engaged, returns a preferred CPU drawn from the big-cluster
 * intersected with the task's allowed mask.  If the intersection
 * is empty (task is pinned to little cores) returns -1.
 */
int hikari_select_cpu(struct task_struct *p, int prev_cpu, int wake_flags)
{
	struct hikari_tstate *st;
	u32 ewma, threshold_ns;
	int cpu;

	if (!IS_ENABLED(CONFIG_HIKARI_PLACEMENT))
		return -1;
	if (!READ_ONCE(hikari_placement_enable))
		return -1;
	if (!hikari_task_active(p))
		return -1;

	st = hikari_state(p);
	if (!st)
		return -1;

	ewma = READ_ONCE(st->wait_ewma_ns);
	threshold_ns = READ_ONCE(hikari_wake_threshold_us) * 1000;
	if (ewma <= threshold_ns) {
		hikari_set_skip_reason(p, HIKARI_SKIP_EWMA_LOW);
		return -1;
	}

	if (!(READ_ONCE(st->flags) & HIKARI_FLAG_FOREGROUND) &&
	    !hikari_cpu_is_audio_active(prev_cpu) &&
	    !(READ_ONCE(st->flags) & HIKARI_FLAG_AUDIO_TAGGED)) {
		hikari_set_skip_reason(p, HIKARI_SKIP_PLACEMENT_NOT_TOPAPP);
		return -1;
	}

	if (cpumask_empty(&hikari_big_cluster)) {
		hikari_set_skip_reason(p, HIKARI_SKIP_PLACEMENT_NO_BIG);
		return -1;
	}

	/*
	 * Two-pass selection over the big cluster:
	 *   1. Prefer a fully idle CPU (available_idle_cpu) -- avoids
	 *      packing wake-demand tasks onto an already-busy core.
	 *   2. Fall back to the first online CPU if none is idle.
	 *
	 * Within each pass we pick the first match; a proper
	 * least-loaded selection would need rq->nr_running which
	 * is scheduler-internal and not worth the coupling here.
	 */
	{
		int fallback = -1;

		for_each_cpu_and(cpu, &hikari_big_cluster, p->cpus_ptr) {
			if (!cpu_online(cpu))
				continue;
			if (available_idle_cpu(cpu)) {
				hikari_set_skip_reason(p, HIKARI_SKIP_ACTIONED);
				if (trace_hikari_placement_enabled())
					trace_hikari_placement(prev_cpu, cpu,
							       true, true);
				return cpu;
			}
			if (fallback < 0)
				fallback = cpu;
		}

		if (fallback >= 0) {
			hikari_set_skip_reason(p, HIKARI_SKIP_ACTIONED);
			if (trace_hikari_placement_enabled())
				trace_hikari_placement(prev_cpu, fallback,
						       false, true);
			return fallback;
		}
	}

	/*
	 * Big-cluster mask is non-empty but the task's allowed-CPU
	 * intersection with it is empty (pinned to little cores) or
	 * every big CPU is offline.  Tag the more informative reason.
	 */
	hikari_set_skip_reason(p, HIKARI_SKIP_PLACEMENT_PINNED);
	return -1;
}
EXPORT_SYMBOL_GPL(hikari_select_cpu);

/*
 * Profile-aware Hikari tuning.  Called from Zenith's
 * zenith_apply_profile() when the active profile changes.
 *
 * Profile IDs mirror Zenith's ZENITH_PROFILE_* defines:
 *   0 = CUSTOM, 1 = PERFORMANCE, 2 = BALANCED, 3 = BATTERY,
 *   4 = LEGACY, 5 = GAMING, 6 = AUDIO, 7 = AUTO.
 *
 * BALANCED / CUSTOM / LEGACY / AUTO write zero (the compile-time
 * default) so the cold-boot baseline is preserved byte-for-byte
 * — zero means "no always-on force-floor, wake-only behaviour".
 *
 * PERFORMANCE / GAMING set a non-zero force_floor_pct so cpufreq
 * never drops below that fraction of the cluster's max, even
 * when there is no recent wake event.
 *
 * BATTERY zeroes both to let the thermal stack pull freq down
 * freely.  AUDIO sets a modest little-cluster floor to stabilise
 * audio pipeline jitter without heating the big cluster.
 */
void hikari_apply_profile(unsigned int profile)
{
	struct hikari_floor_profile {
		unsigned int force_floor_pct_big;
		unsigned int force_floor_pct_little;
	};

	static const struct hikari_floor_profile profiles[] = {
		/* PERFORMANCE (1) */
		[1] = { .force_floor_pct_big = 15,
			.force_floor_pct_little = 10 },
		/* BALANCED (2): compile-time defaults (0/0) */
		[2] = { .force_floor_pct_big = 0,
			.force_floor_pct_little = 0 },
		/* BATTERY (3) */
		[3] = { .force_floor_pct_big = 0,
			.force_floor_pct_little = 0 },
		/* GAMING (5) */
		[5] = { .force_floor_pct_big = 15,
			.force_floor_pct_little = 10 },
		/* AUDIO (6) */
		[6] = { .force_floor_pct_big = 0,
			.force_floor_pct_little = 8 },
	};

	const struct hikari_floor_profile *v;

	profile = zenith_resolve_profile(profile);
	if (profile >= ARRAY_SIZE(profiles))
		profile = ZENITH_PROFILE_BALANCED;

	v = &profiles[profile];

	{
		unsigned int big = v->force_floor_pct_big;

		if (IS_ENABLED(CONFIG_KASUMI)) {
			int kasumi_offset = kasumi_get_applied_offset_mc();

			/*
			 * Kasumi cross-link: when the thermal dampening offset exceeds
			 * 15 C (15000 mc), the frequency floor and dampening can
			 * conspire to hide heat buildup.  Reduce the big floor
			 * proportionally: each 5 C over 15 C reduces the floor by
			 * 5 percentage points, with a floor of 0 (full surrender to
			 * the thermal framework).
			 *
			 * Example: offset=22 C, big=15%  -> reduction = 5%
			 *          effective floor = 15 - 5 = 10%
			 * Example: offset=30 C, big=15%  -> reduction = 15%
			 *          effective floor = 0%
			 */
			if (kasumi_offset > 15000 && big > 0) {
				unsigned int reduction = (kasumi_offset - 15000) / 5000 * 5;

				if (reduction >= big)
					big = 0;
				else
					big -= reduction;
			}
		}

		WRITE_ONCE(hikari_force_floor_pct_big, big);
		WRITE_ONCE(hikari_force_floor_pct_little, v->force_floor_pct_little);
		hikari_recompute_force_floors();
	}

	if (trace_hikari_profile_enabled())
		trace_hikari_profile(profile, v->force_floor_pct_big,
				     v->force_floor_pct_little);

#ifdef CONFIG_HIKARI_DEBUG_MSG
	pr_info_ratelimited("hikari: profile %u applied (force_floor big=%u%% little=%u%%)\n",
			    profile, v->force_floor_pct_big,
			    v->force_floor_pct_little);
#endif /* CONFIG_HIKARI_DEBUG_MSG */
}

/*
 * Return the jiffies timestamp of the most recent wake-demand
 * publish.  Iyashi uses this to decide whether scheduler-side
 * wake pressure is active and optionally raise the thermal
 * performance floor during bursty foreground load.
 */
unsigned long hikari_get_last_demand_jiffies(void)
{
	return READ_ONCE(hikari_last_demand_jiffies);
}

/* ------------------------------------------------------------ */
/* Setter API.                                                  */
/* ------------------------------------------------------------ */

static void hikari_set_flag(struct task_struct *p, u32 bit, bool on)
{
	struct hikari_tstate *st;
	u32 cur, new;

	st = hikari_state(p);
	if (!st)
		return;
	do {
		cur = READ_ONCE(st->flags);
		new = on ? (cur | bit) : (cur & ~bit);
		if (new == cur)
			return;
	} while (cmpxchg(&st->flags, cur, new) != cur);
}

/*
 * Stamp the per-task "last skip reason" into the upper byte of
 * hikari_flags.  Returns immediately (no atomic) if the value already
 * matches -- this is the common case for a steady-state task that
 * stays in the same opt-in / placement bucket across many enqueues.
 * Otherwise updates via cmpxchg so it cooperates with hikari_set_flag.
 */
static inline void hikari_set_skip_reason(struct task_struct *p, u32 reason)
{
	struct hikari_tstate *st;
	u32 cur, new;
	u32 want = (reason << HIKARI_SKIP_REASON_SHIFT) &
		   HIKARI_SKIP_REASON_MASK;

	st = hikari_state(p);
	if (!st)
		return;
	do {
		cur = READ_ONCE(st->flags);
		new = (cur & ~HIKARI_SKIP_REASON_MASK) | want;
		if (new == cur)
			return;
	} while (cmpxchg(&st->flags, cur, new) != cur);
}

void hikari_set_opt_in(struct task_struct *p, bool opt_in)
{
	hikari_set_flag(p, HIKARI_FLAG_OPT_IN, opt_in);
}
EXPORT_SYMBOL_GPL(hikari_set_opt_in);

void hikari_mark_audio(struct task_struct *p, bool tagged)
{
	hikari_set_flag(p, HIKARI_FLAG_AUDIO_TAGGED, tagged);
	if (tagged && p)
		hikari_pcpu_mark_audio(task_cpu(p));
}
EXPORT_SYMBOL_GPL(hikari_mark_audio);

void hikari_mark_foreground(struct task_struct *p, bool tagged)
{
	hikari_set_flag(p, HIKARI_FLAG_FOREGROUND, tagged);

	/*
	 * Top-app auto-opt-in: tagging foreground also opts the
	 * task in for Hikari if the auto-opt-in sysctl is set.
	 * Conversely, untagging foreground does NOT opt out --
	 * the user may have explicitly opted in via /proc.
	 */
	if (tagged && READ_ONCE(hikari_topapp_auto_optin))
		hikari_set_flag(p, HIKARI_FLAG_OPT_IN, true);

	/*
	 * Foreground and background are mutually exclusive states.
	 * Tagging foreground clears any existing background tag so a
	 * task that flips between the two cgroups never carries the
	 * stale ceiling.  Untagging foreground does NOT auto-set
	 * background -- userspace (or a vendor hook) is responsible
	 * for the explicit background tag.
	 */
	if (tagged)
		hikari_set_flag(p, HIKARI_FLAG_BACKGROUND, false);
}
EXPORT_SYMBOL_GPL(hikari_mark_foreground);

void hikari_mark_background(struct task_struct *p, bool tagged)
{
	hikari_set_flag(p, HIKARI_FLAG_BACKGROUND, tagged);

	/*
	 * Symmetric mutual-exclusion with hikari_mark_foreground:
	 * tagging background clears any foreground tag.
	 */
	if (tagged)
		hikari_set_flag(p, HIKARI_FLAG_FOREGROUND, false);
}
EXPORT_SYMBOL_GPL(hikari_mark_background);

/* ------------------------------------------------------------ */
/* Notifier registration.                                       */
/* ------------------------------------------------------------ */

int hikari_register_cpufreq_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&hikari_cpufreq_chain, nb);
}
EXPORT_SYMBOL_GPL(hikari_register_cpufreq_notifier);

int hikari_unregister_cpufreq_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&hikari_cpufreq_chain, nb);
}
EXPORT_SYMBOL_GPL(hikari_unregister_cpufreq_notifier);

/* ------------------------------------------------------------ */
/* Sysctls.                                                     */
/* ------------------------------------------------------------ */

static int hikari_enable_sysctl_handler(struct ctl_table *table, int write,
					void *buffer, size_t *lenp, loff_t *ppos)
{
	int ret = proc_douintvec_minmax(table, write, buffer, lenp, ppos);

	if (write && !ret) {
		if (READ_ONCE(hikari_enable_value) != 0)
			atomic_set(&hikari_kill_flag, 0);
		hikari_active_key_sync();
	}
	return ret;
}

static struct ctl_table hikari_sysctl_table[] = {
	{
		.procname	= "hikari_enable",
		.data		= &hikari_enable_value,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= hikari_enable_sysctl_handler,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{
		.procname	= "hikari_wake_threshold_us",
		.data		= &hikari_wake_threshold_us,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_threshold_min,
		.extra2		= (void *)&hikari_threshold_max,
	},
	{
		.procname	= "hikari_uclamp_boost_pct",
		.data		= &hikari_uclamp_boost_pct,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_boost_pct_max,
	},
	{
		.procname	= "hikari_uclamp_ttl_ms",
		.data		= &hikari_uclamp_ttl_ms,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uclamp_ttl_min,
		.extra2		= (void *)&hikari_uclamp_ttl_max,
	},
	{
		.procname	= "hikari_uclamp_max_pct",
		.data		= &hikari_uclamp_max_pct,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uclamp_max_pct_max,
	},
	{
		.procname	= "hikari_floor_khz_cluster0",
		.data		= &hikari_floor_khz_cluster0,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_floor_khz_max_c0,
	},
	{
		.procname	= "hikari_floor_khz_cluster1",
		.data		= &hikari_floor_khz_cluster1,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_floor_khz_max_c1,
	},
	{
		.procname	= "hikari_floor_ttl_ms",
		.data		= &hikari_floor_ttl_ms,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_floor_ttl_min,
		.extra2		= (void *)&hikari_floor_ttl_max,
	},
	{
		.procname	= "hikari_floor_ttl_ms_big",
		.data		= &hikari_floor_ttl_ms_big,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_floor_ttl_override_min,
		.extra2		= (void *)&hikari_floor_ttl_max,
	},
	{
		.procname	= "hikari_floor_ttl_ms_little",
		.data		= &hikari_floor_ttl_ms_little,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_floor_ttl_override_min,
		.extra2		= (void *)&hikari_floor_ttl_max,
	},
	{
		.procname	= "hikari_placement_enable",
		.data		= &hikari_placement_enable,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{
		.procname	= "hikari_audio_intensify",
		.data		= &hikari_audio_intensify,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{
		.procname	= "hikari_topapp_auto_optout",
		.data		= &hikari_topapp_auto_optout,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{
		.procname	= "hikari_ewma_shift",
		.data		= &hikari_ewma_shift,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_ewma_shift_min,
		.extra2		= (void *)&hikari_ewma_shift_max,
	},
	{
		.procname	= "hikari_learn_boost_pct",
		.data		= &hikari_learn_boost_pct,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_learn_boost_pct_max,
	},
	{
		.procname	= "hikari_learn_forget_ratio",
		.data		= &hikari_learn_forget_ratio,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_learn_forget_ratio_min,
		.extra2		= (void *)&hikari_learn_forget_ratio_max,
	},
	{
		.procname	= "hikari_topapp_auto_optin",
		.data		= &hikari_topapp_auto_optin,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{ }
};

/* ------------------------------------------------------------ */
/* Init.                                                        */
/* ------------------------------------------------------------ */

static void __init hikari_discover_clusters(void)
{
	unsigned long max_cap = 0, min_cap = ULONG_MAX;
	int cpu;

	cpumask_clear(&hikari_big_cluster);
	cpumask_clear(&hikari_little_cluster);

	for_each_possible_cpu(cpu) {
		unsigned long cap = arch_scale_cpu_capacity(cpu);

		if (cap > max_cap)
			max_cap = cap;
		if (cap < min_cap)
			min_cap = cap;
	}

	if (max_cap == min_cap) {
		/* uniprocessor / SMP-symmetric: no big cluster. */
		pr_info("symmetric capacity, big.LITTLE placement disabled\n");
		return;
	}

	for_each_possible_cpu(cpu) {
		unsigned long cap = arch_scale_cpu_capacity(cpu);

		if (cap == max_cap) {
			cpumask_set_cpu(cpu, &hikari_big_cluster);
			hikari_max_big_cpu = cpu;
		} else {
			cpumask_set_cpu(cpu, &hikari_little_cluster);
		}
	}

	pr_info("big cluster: %*pbl, little cluster: %*pbl\n",
		cpumask_pr_args(&hikari_big_cluster),
		cpumask_pr_args(&hikari_little_cluster));
}

/* ------------------------------------------------------------ */
/* Per-task /proc helpers (callable from fs/proc/base.c).       */
/* ------------------------------------------------------------ */

/*
 * Print a human-readable per-task stats blob to @m.  Output
 * format is one "key: value" line per attribute; the keys are
 * stable for parsing.
 */
static const char * const hikari_skip_reason_names[] = {
	[HIKARI_SKIP_NONE]			= "none",
	[HIKARI_SKIP_GLOBAL_DISABLED]		= "global_disabled",
	[HIKARI_SKIP_NOT_OPTED_IN]		= "not_opted_in",
	[HIKARI_SKIP_EWMA_LOW]			= "ewma_below_threshold",
	[HIKARI_SKIP_PLACEMENT_NOT_TOPAPP]	= "placement_not_topapp_or_audio",
	[HIKARI_SKIP_PLACEMENT_NO_BIG]		= "placement_no_big_cluster",
	[HIKARI_SKIP_PLACEMENT_PINNED]		= "placement_pinned_off_big",
	[HIKARI_SKIP_ACTIONED]			= "actioned",
};

void hikari_seq_print_stats(struct seq_file *m, struct task_struct *p)
{
	struct hikari_tstate *st;
	u32 flags, ewma, last_enq, boost_until, reason;
	const char *reason_name;

	if (!p) {
		seq_puts(m, "hikari: invalid task\n");
		return;
	}

	st = hikari_state(p);
	if (!st) {
		seq_puts(m, "hikari: no per-task state\n");
		return;
	}

	flags       = READ_ONCE(st->flags);
	ewma        = READ_ONCE(st->wait_ewma_ns);
	last_enq    = READ_ONCE(st->last_enqueue_ns);
	boost_until = READ_ONCE(st->boost_until_ns);
	reason      = (flags & HIKARI_SKIP_REASON_MASK) >>
		      HIKARI_SKIP_REASON_SHIFT;
	reason_name = (reason <= HIKARI_SKIP_REASON_MAX)
		      ? hikari_skip_reason_names[reason]
		      : "unknown";

	seq_printf(m, "hikari_enabled_global: %u\n",
		   hikari_enabled() ? 1U : 0U);
	seq_printf(m, "hikari_opt_in: %u\n",
		   (flags & HIKARI_FLAG_OPT_IN) ? 1U : 0U);
	seq_printf(m, "hikari_audio_tagged: %u\n",
		   (flags & HIKARI_FLAG_AUDIO_TAGGED) ? 1U : 0U);
	seq_printf(m, "hikari_foreground: %u\n",
		   (flags & HIKARI_FLAG_FOREGROUND) ? 1U : 0U);
	seq_printf(m, "hikari_background: %u\n",
		   (flags & HIKARI_FLAG_BACKGROUND) ? 1U : 0U);
	seq_printf(m, "hikari_learned: %u\n",
		   (flags & HIKARI_FLAG_LEARNED) ? 1U : 0U);
	seq_printf(m, "hikari_learn_boost_pct: %u\n",
		   READ_ONCE(hikari_learn_boost_pct));
	seq_printf(m, "hikari_uclamp_max_ceiling: %u\n",
		   hikari_uclamp_max_ceiling(p));
	seq_printf(m, "hikari_wait_ewma_ns: %u\n", ewma);
	seq_printf(m, "hikari_last_enqueue_ns: %u\n", last_enq);
	seq_printf(m, "hikari_boost_active: %u\n",
		   hikari_token_active(boost_until) ? 1U : 0U);
	seq_printf(m, "hikari_boost_until_jiffies: %u\n", boost_until);
	/*
	 * Per-task last-skip reason.  Set by the scheduler hot path on
	 * every call into hikari_on_dequeue() / hikari_select_cpu() so
	 * a /proc reader can tell why Hikari is (or isn't) firing for
	 * this specific PID.  See HIKARI_SKIP_* in <linux/hikari.h>.
	 */
	seq_printf(m, "hikari_skip_reason: %u %s\n", reason, reason_name);
}
EXPORT_SYMBOL_GPL(hikari_seq_print_stats);

/* Helper for /proc/<pid>/hikari_enable + /proc/<pid>/hikari_audio. */
u32 hikari_task_get_flag(struct task_struct *p, u32 bit)
{
	struct hikari_tstate *st;

	if (!p)
		return 0;
	st = hikari_state(p);
	return st && (READ_ONCE(st->flags) & bit) ? 1U : 0U;
}
EXPORT_SYMBOL_GPL(hikari_task_get_flag);

void hikari_task_set_flag(struct task_struct *p, u32 bit, bool on)
{
	if (bit == HIKARI_FLAG_FOREGROUND) {
		hikari_mark_foreground(p, on);
		return;
	}
	if (bit == HIKARI_FLAG_BACKGROUND) {
		hikari_mark_background(p, on);
		return;
	}

	hikari_set_flag(p, bit, on);
}
EXPORT_SYMBOL_GPL(hikari_task_set_flag);

/* ------------------------------------------------------------ */
/* /sys/kernel/hikari/ observability.                           */
/* ------------------------------------------------------------ */

static struct kobject *hikari_kobj;

static ssize_t enabled_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "%u\n", hikari_enabled() ? 1U : 0U);
}

static ssize_t disabled_reason_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  (unsigned int)READ_ONCE(hikari_disable_reason_global));
}

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "%s\n", "hikari-v2");
}

static ssize_t total_boost_count_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	unsigned long long sum = 0;
	int cpu;

	for_each_possible_cpu(cpu)
		sum += (unsigned long long)
			atomic_read(&per_cpu_ptr(&hikari_pcpu, cpu)->boost_count);
	return sysfs_emit(buf, "%llu\n", sum);
}

static ssize_t total_hint_count_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	unsigned long long sum = 0;
	int cpu;

	for_each_possible_cpu(cpu)
		sum += (unsigned long long)
			atomic_read(&per_cpu_ptr(&hikari_pcpu, cpu)->hint_count);
	return sysfs_emit(buf, "%llu\n", sum);
}

static ssize_t opted_in_count_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	struct hikari_tstate *st;
	unsigned int count = 0;
	unsigned long flags;
	int bkt;

	spin_lock_irqsave(&hikari_tstate_lock, flags);
	hash_for_each(hikari_tstate_table, bkt, st, hnode) {
		if (READ_ONCE(st->flags) & HIKARI_FLAG_OPT_IN)
			count++;
	}
	spin_unlock_irqrestore(&hikari_tstate_lock, flags);

	return sysfs_emit(buf, "%u\n", count);
}

static ssize_t big_cluster_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%*pbl\n",
			  cpumask_pr_args(&hikari_big_cluster));
}

static ssize_t little_cluster_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%*pbl\n",
			  cpumask_pr_args(&hikari_little_cluster));
}

/*
 * /sys/kernel/hikari/wake_hist  -- R/O
 *
 * Sums the per-CPU wake_hist[] buckets and emits one line per bucket
 * with the closed-open microsecond range, e.g.
 *
 *     0..100us:        12345
 *     100..250us:       4567
 *     ...
 *     10000+us:           42
 *     total:           17394
 *
 * The format is grep-able by humans and parseable by tools that split
 * on the first non-digit character.  Per-CPU summing tolerates
 * concurrent writers because each bucket is an atomic_long_t.
 */
static ssize_t wake_hist_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	unsigned long counts[HIKARI_WAKE_HIST_BUCKETS] = { 0 };
	unsigned long total = 0;
	ssize_t off = 0;
	int cpu, i;
	u32 lo, hi;

	for_each_possible_cpu(cpu) {
		struct hikari_pcpu *pc = per_cpu_ptr(&hikari_pcpu, cpu);

		for (i = 0; i < HIKARI_WAKE_HIST_BUCKETS; i++)
			counts[i] += (unsigned long)atomic_long_read(
				&pc->wake_hist[i]);
	}

	for (i = 0; i < HIKARI_WAKE_HIST_BUCKETS; i++) {
		lo = (i == 0) ? 0 : hikari_wake_hist_edges_us[i - 1];
		if (i == HIKARI_WAKE_HIST_BUCKETS - 1) {
			off += sysfs_emit_at(buf, off,
					     "%u+us: %lu\n",
					     lo, counts[i]);
		} else {
			hi = hikari_wake_hist_edges_us[i];
			off += sysfs_emit_at(buf, off,
					     "%u..%uus: %lu\n",
					     lo, hi, counts[i]);
		}
		total += counts[i];
	}
	off += sysfs_emit_at(buf, off, "total: %lu\n", total);
	return off;
}

/*
 * /sys/kernel/hikari/wake_hist_reset  -- W/O
 *
 * Any non-empty write zeroes every CPU's bucket array.  Useful
 * before running a benchmark so the captured shape reflects only
 * the benchmark window.  No-op if buf is empty.
 */
static ssize_t wake_hist_reset_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	int cpu, i;

	if (!count)
		return -EINVAL;

	for_each_possible_cpu(cpu) {
		struct hikari_pcpu *pc = per_cpu_ptr(&hikari_pcpu, cpu);

		for (i = 0; i < HIKARI_WAKE_HIST_BUCKETS; i++)
			atomic_long_set(&pc->wake_hist[i], 0);
	}
	return count;
}

static struct kobj_attribute hikari_attr_wake_hist =
	__ATTR(wake_hist, 0444, wake_hist_show, NULL);
static struct kobj_attribute hikari_attr_wake_hist_reset =
	__ATTR(wake_hist_reset, 0200, NULL, wake_hist_reset_store);

/*
 * R/W sysfs tunables -- mirrors of the /proc/sys/kernel/hikari_*
 * sysctls, exposed here so apps like Franco Kernel Manager can
 * discover them by scanning /sys/kernel/hikari/.
 */
#define HIKARI_TUNABLE_RW(_name, _var, _min, _max)			\
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
	if (kstrtouint(buf, 10, &val))					\
		return -EINVAL;						\
	if (val < (_min) || val > (_max))				\
		return -EINVAL;						\
	WRITE_ONCE(_var, val);						\
	return count;							\
}									\
static struct kobj_attribute hikari_attr_##_name =			\
	__ATTR(_name, 0644, _name##_show, _name##_store)

static ssize_t enable_show(struct kobject *kobj,
			   struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(hikari_enable_value));
}

static ssize_t enable_store(struct kobject *kobj,
			    struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;
	WRITE_ONCE(hikari_enable_value, val);
	if (val)
		atomic_set(&hikari_kill_flag, 0);
	hikari_active_key_sync();
	return count;
}

static struct kobj_attribute hikari_attr_enable =
	__ATTR(enable, 0644, enable_show, enable_store);

HIKARI_TUNABLE_RW(wake_threshold_us, hikari_wake_threshold_us,
		  100, 100000);
HIKARI_TUNABLE_RW(uclamp_boost_pct, hikari_uclamp_boost_pct, 0, 50);
HIKARI_TUNABLE_RW(uclamp_ttl_ms, hikari_uclamp_ttl_ms, 1, 200);
HIKARI_TUNABLE_RW(uclamp_max_pct, hikari_uclamp_max_pct, 0, 100);
HIKARI_TUNABLE_RW(floor_khz_cluster0, hikari_floor_khz_cluster0,
		  0, 2000000);
HIKARI_TUNABLE_RW(floor_khz_cluster1, hikari_floor_khz_cluster1,
		  0, 3000000);
HIKARI_TUNABLE_RW(floor_ttl_ms, hikari_floor_ttl_ms, 1, 500);
HIKARI_TUNABLE_RW(floor_ttl_ms_big, hikari_floor_ttl_ms_big, 0, 500);
HIKARI_TUNABLE_RW(floor_ttl_ms_little, hikari_floor_ttl_ms_little, 0, 500);

/*
 * force_floor_pct_{big,little}: pct of cluster cpuinfo_max_freq that
 * the published Zenith floor is held at, regardless of any recent
 * wake event.  Custom store handlers (rather than HIKARI_TUNABLE_RW)
 * because the kHz cache must be recomputed after every change.
 */
static ssize_t force_floor_pct_big_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(hikari_force_floor_pct_big));
}

static ssize_t force_floor_pct_big_store(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	WRITE_ONCE(hikari_force_floor_pct_big, val);
	hikari_recompute_force_floors();
	return count;
}

static struct kobj_attribute hikari_attr_force_floor_pct_big =
	__ATTR(force_floor_pct_big, 0644,
	       force_floor_pct_big_show, force_floor_pct_big_store);

static ssize_t force_floor_pct_little_show(struct kobject *kobj,
					   struct kobj_attribute *attr,
					   char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  READ_ONCE(hikari_force_floor_pct_little));
}

static ssize_t force_floor_pct_little_store(struct kobject *kobj,
					    struct kobj_attribute *attr,
					    const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	WRITE_ONCE(hikari_force_floor_pct_little, val);
	hikari_recompute_force_floors();
	return count;
}

static struct kobj_attribute hikari_attr_force_floor_pct_little =
	__ATTR(force_floor_pct_little, 0644,
	       force_floor_pct_little_show, force_floor_pct_little_store);

/*
 * Read-only mirror of the cached kHz values so userspace can verify
 * the cpuinfo_max_freq lookup landed (useful after boot or hotplug).
 */
static ssize_t force_floor_khz_big_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(hikari_force_floor_khz_big));
}

static struct kobj_attribute hikari_attr_force_floor_khz_big =
	__ATTR(force_floor_khz_big, 0444, force_floor_khz_big_show, NULL);

static ssize_t force_floor_khz_little_show(struct kobject *kobj,
					   struct kobj_attribute *attr,
					   char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  READ_ONCE(hikari_force_floor_khz_little));
}

static struct kobj_attribute hikari_attr_force_floor_khz_little =
	__ATTR(force_floor_khz_little, 0444,
	       force_floor_khz_little_show, NULL);

HIKARI_TUNABLE_RW(learn_boost_pct, hikari_learn_boost_pct, 0, 50);
HIKARI_TUNABLE_RW(learn_forget_ratio, hikari_learn_forget_ratio, 2, 10);
HIKARI_TUNABLE_RW(placement_enable, hikari_placement_enable, 0, 1);
HIKARI_TUNABLE_RW(audio_intensify, hikari_audio_intensify, 0, 1);
HIKARI_TUNABLE_RW(topapp_auto_optin, hikari_topapp_auto_optin, 0, 1);
HIKARI_TUNABLE_RW(topapp_auto_optout, hikari_topapp_auto_optout, 0, 1);
HIKARI_TUNABLE_RW(ewma_shift, hikari_ewma_shift, 1, 7);

static struct kobj_attribute hikari_attr_enabled =
	__ATTR(enabled, 0444, enabled_show, NULL);
static struct kobj_attribute hikari_attr_disabled_reason =
	__ATTR(disabled_reason, 0444, disabled_reason_show, NULL);
static struct kobj_attribute hikari_attr_version =
	__ATTR(version, 0444, version_show, NULL);
static struct kobj_attribute hikari_attr_total_boost_count =
	__ATTR(total_boost_count, 0444, total_boost_count_show, NULL);
static struct kobj_attribute hikari_attr_total_hint_count =
	__ATTR(total_hint_count, 0444, total_hint_count_show, NULL);
static struct kobj_attribute hikari_attr_opted_in_count =
	__ATTR(opted_in_count, 0444, opted_in_count_show, NULL);
static struct kobj_attribute hikari_attr_big_cluster =
	__ATTR(big_cluster, 0444, big_cluster_show, NULL);
static struct kobj_attribute hikari_attr_little_cluster =
	__ATTR(little_cluster, 0444, little_cluster_show, NULL);

static struct attribute *hikari_sysfs_attrs[] = {
	/* R/O status */
	&hikari_attr_enabled.attr,
	&hikari_attr_disabled_reason.attr,
	&hikari_attr_version.attr,
	&hikari_attr_total_boost_count.attr,
	&hikari_attr_total_hint_count.attr,
	&hikari_attr_opted_in_count.attr,
	&hikari_attr_big_cluster.attr,
	&hikari_attr_little_cluster.attr,
	&hikari_attr_wake_hist.attr,
	&hikari_attr_wake_hist_reset.attr,
	/* R/W tunables */
	&hikari_attr_enable.attr,
	&hikari_attr_wake_threshold_us.attr,
	&hikari_attr_uclamp_boost_pct.attr,
	&hikari_attr_uclamp_ttl_ms.attr,
	&hikari_attr_uclamp_max_pct.attr,
	&hikari_attr_floor_khz_cluster0.attr,
	&hikari_attr_floor_khz_cluster1.attr,
	&hikari_attr_floor_ttl_ms.attr,
	&hikari_attr_floor_ttl_ms_big.attr,
	&hikari_attr_floor_ttl_ms_little.attr,
	&hikari_attr_force_floor_pct_big.attr,
	&hikari_attr_force_floor_pct_little.attr,
	&hikari_attr_force_floor_khz_big.attr,
	&hikari_attr_force_floor_khz_little.attr,
	&hikari_attr_placement_enable.attr,
	&hikari_attr_audio_intensify.attr,
	&hikari_attr_topapp_auto_optin.attr,
	&hikari_attr_topapp_auto_optout.attr,
	&hikari_attr_learn_boost_pct.attr,
	&hikari_attr_learn_forget_ratio.attr,
	&hikari_attr_ewma_shift.attr,
	NULL,
};

static const struct attribute_group hikari_sysfs_group = {
	.attrs = hikari_sysfs_attrs,
};

static int __init hikari_sysfs_init(void)
{
	int ret;

	hikari_kobj = kobject_create_and_add("hikari", kernel_kobj);
	if (!hikari_kobj) {
		pr_warn("failed to create /sys/kernel/hikari\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(hikari_kobj, &hikari_sysfs_group);
	if (ret) {
		kobject_put(hikari_kobj);
		hikari_kobj = NULL;
		pr_warn("failed to create /sys/kernel/hikari attributes (%d)\n",
			ret);
		return ret;
	}

	return 0;
}


/* ------------------------------------------------------------ */
/* Selene game detection integration                            */
/* ------------------------------------------------------------ */
static int hikari_selene_nb_fn(struct notifier_block *nb,
                               unsigned long event, void *data)
{
	switch (event) {
	case SELENE_EVENT_GAME_START:
		WRITE_ONCE(hikari_game_selene_boost, true);
		printk(KERN_INFO "GrayRavens: hikari: GAME_START: game floor armed\n");
		break;
	case SELENE_EVENT_GAME_STOP:
		WRITE_ONCE(hikari_game_selene_boost, false);
		printk(KERN_INFO "GrayRavens: hikari: GAME_STOP: game floor cleared\n");
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block hikari_selene_nb = {
	.notifier_call = hikari_selene_nb_fn,
};

static int __init hikari_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct hikari_pcpu *pc = per_cpu_ptr(&hikari_pcpu, cpu);

		memset(pc, 0, sizeof(*pc));
		atomic_set(&pc->boost_count, 0);
		atomic_set(&pc->hint_count, 0);
	}

	hikari_tstate_init();
	hikari_discover_clusters();
	hikari_recompute_force_floors();

	if (!register_sysctl("kernel", hikari_sysctl_table)) {
		pr_err("failed to register sysctl entries\n");
		return -ENOMEM;
	}

	/*
	 * Sysfs init: non-fatal on failure.  /sys/kernel/hikari is
	 * observability-only; if it fails to create, the rest of
	 * Hikari is still functional.  The warning is logged.
	 */
	(void)hikari_sysfs_init();

	smp_wmb();
	WRITE_ONCE(hikari_init_complete, true);

	/*
	 * init_complete is now visible; flip the static key on if the
	 * master enable sysctl is non-zero.  Hot paths short-circuit to
	 * a single unlikely-branch when disabled.
	 */
	hikari_active_key_sync();

	/* Subscribe to Selene game detection */
	{
		int sret = selene_register_notifier(&hikari_selene_nb);

		if (sret)
			pr_warn("hikari: selene notifier register failed (%d)\n", sret);
		else
			pr_info("hikari: subscribed to selene game detection chain\n");
	}

	pr_info("initialised (master enable=%u, static_key=%s)\n",
		READ_ONCE(hikari_enable_value),
		static_key_enabled(&hikari_active_key.key) ? "on" : "off");

	#ifdef CONFIG_HIKARI_DEBUG_MSG
	pr_info("hikari: initialized v2 (enable=%u, static_key=%s)\n",
		READ_ONCE(hikari_enable_value),
		static_key_enabled(&hikari_active_key.key) ? "on" : "off");
	#endif /* CONFIG_HIKARI_DEBUG_MSG */

	return 0;
}
late_initcall(hikari_init);
