// SPDX-License-Identifier: GPL-2.0
/*
 * Hyperion CPUFreq governor (GrayRavens)
 *
 * A hybrid governor combining the instant responsiveness of the
 * interactive governor with the efficiency of schedutil PELT-based
 * scaling.  The frequency selection core is schedutil-identical
 * (including the 1.25x DVFS headroom); on top of it sits a
 * hispeed floor measured from real CPU busy% (kcpustat idle-time
 * counters), blended with PELT util using an exponential decay
 * matched to PELT's 32 ms half-life:
 *
 *   blended = pelt + (hispeed - pelt) >> half_lives
 *
 * After ~320 ms (10 half-lives) the hispeed contribution is
 * negligible and PELT proportional scaling takes full control.
 *
 * On top of the reflex-style core, hyperion carries three
 * GrayRavens additions:
 *
 *   - Input boost:   touch events immediately arm the hispeed floor
 *   - Game floor:    sustained high busy% holds a frequency floor
 *                    so frame-bursty workloads do not sag
 *   - Screen off:    suspend relaxes the hispeed blend and disables
 *                    the input boost
 *
 * Algorithm family: schedutil + reflex (firelzrd) hispeed blending,
 * adapted to android12-5.10 by GrayRavens.
 */

#define pr_fmt(fmt) "hyperion: " fmt

#include "sched.h"

#include <linux/cpufreq.h>
#include <linux/input.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/sched/cpufreq.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/tick.h>

#include <uapi/linux/sched/types.h>

/**************************************************************
 * Version information
 */
#define CPUFREQ_HYPERION_PROGNAME	"Hyperion CPUFreq Governor"
#define CPUFREQ_HYPERION_VERSION	"0.1.0"

/**************************************************************
 * Default tunables
 */
#define HYPERION_DEFAULT_HISPEED_WINDOW_US	4000
#define HYPERION_DEFAULT_HISPEED_FILTER_SHIFT	3
#define HYPERION_DEFAULT_INPUT_BOOST_PCT	80
#define HYPERION_DEFAULT_INPUT_BOOST_DURATION_US	8000
#define HYPERION_DEFAULT_SUSTAINED_THRESHOLD_PCT	60
#define HYPERION_DEFAULT_SUSTAINED_WINDOW_MS	200
#define HYPERION_DEFAULT_SUSTAINED_FLOOR_PCT	70
#define HYPERION_DEFAULT_SCREEN_OFF_CAP_PCT	40

#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

struct hyp_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		rate_limit_us;
	unsigned int		hispeed_window_us;
	unsigned int		hispeed_filter_shift;
	unsigned int		input_boost_pct;
	unsigned int		input_boost_duration_us;
	unsigned int		sustained_threshold_pct;
	unsigned int		sustained_window_ms;
	unsigned int		sustained_floor_pct;
	unsigned int		screen_off_cap_pct;
};

struct hyp_policy {
	struct cpufreq_policy	*policy;

	struct hyp_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			freq_update_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;

	/* Only needed if fast switch cannot be used: */
	struct irq_work		irq_work;
	struct kthread_work	work;
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;
};

struct hyp_cpu {
	struct update_util_data	update_util;
	struct hyp_policy	*hyp_policy;
	unsigned int		cpu;

	/* I/O wait boost (schedutil-compatible) */
	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	unsigned long		bw_dl;
	unsigned long		max;

	/* Idle-time accounting for hispeed decisions */
	u64			prev_idle_time;
	u64			prev_wall_time;
	unsigned int		busy_pct;
	unsigned int		filtered_busy_pct;
	bool			hispeed_active;
	u64			hispeed_start_ns;
	unsigned int		hispeed_idle_windows;

	/* Sustained (game) floor tracking */
	u64			sustained_since;
	bool			sustained;

	/* The field below is for single-CPU policies only: */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long		saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct hyp_cpu, hyp_cpu);

/* Input boost + screen state, written from IRQ/notifier context */
static u64			hyp_input_boost_until;
static unsigned int		hyp_boost_pct = HYPERION_DEFAULT_INPUT_BOOST_PCT;
static unsigned int		hyp_boost_duration_us =
				HYPERION_DEFAULT_INPUT_BOOST_DURATION_US;
static bool			hyp_screen_off;

/**************************************************************
 * Log-domain helpers for hispeed decay.
 *
 * The hispeed contribution decays in log-domain at 1 ms
 * granularity using PELT's 32 ms half-life (y = 2^(-1/32) per
 * ms), so the blend tracks PELT's own time constant exactly.
 */
#define HYP_LOG_OFP		26
#define HYP_LOG_0		S32_MIN
#define HYP_LOG_DECAY_PER_MS	(-2097152)	/* log2(0.97857206) in Q5.26 */
#define HYP_LOG_DECAY_MAX_MS	320		/* 10 half-lives */

static const u16 hyp_enc_corr_lut[256] = {
	    0,    89,   177,   264,   350,   436,   521,   606,   690,   773,   855,   937,  1018,  1098,  1178,  1257,
	 1335,  1413,  1489,  1565,  1641,  1716,  1790,  1863,  1936,  2008,  2079,  2150,  2219,  2289,  2357,  2425,
	 2492,  2558,  2624,  2689,  2753,  2817,  2880,  2942,  3004,  3065,  3125,  3184,  3243,  3301,  3358,  3415,
	 3471,  3526,  3581,  3635,  3688,  3740,  3792,  3843,  3894,  3943,  3992,  4041,  4088,  4135,  4182,  4227,
	 4272,  4316,  4360,  4402,  4444,  4486,  4526,  4566,  4606,  4644,  4682,  4719,  4756,  4792,  4827,  4861,
	 4895,  4928,  4960,  4992,  5023,  5053,  5083,  5112,  5140,  5167,  5194,  5220,  5245,  5270,  5294,  5317,
	 5340,  5362,  5383,  5404,  5423,  5443,  5461,  5479,  5496,  5512,  5528,  5543,  5557,  5570,  5583,  5596,
	 5607,  5618,  5628,  5637,  5646,  5654,  5661,  5668,  5674,  5679,  5683,  5687,  5690,  5693,  5695,  5696,
	 5696,  5696,  5695,  5693,  5690,  5687,  5683,  5679,  5674,  5668,  5661,  5654,  5646,  5637,  5628,  5618,
	 5607,  5596,  5583,  5570,  5557,  5543,  5528,  5512,  5496,  5479,  5461,  5443,  5423,  5404,  5383,  5362,
	 5340,  5317,  5294,  5270,  5245,  5220,  5194,  5167,  5140,  5112,  5083,  5053,  5023,  4992,  4960,  4928,
	 4895,  4861,  4827,  4792,  4756,  4719,  4682,  4644,  4606,  4566,  4526,  4486,  4444,  4402,  4360,  4316,
	 4272,  4227,  4182,  4135,  4088,  4041,  3992,  3943,  3894,  3843,  3792,  3740,  3688,  3635,  3581,  3526,
	 3471,  3415,  3358,  3301,  3243,  3184,  3125,  3065,  3004,  2942,  2880,  2817,  2753,  2689,  2624,  2558,
	 2492,  2425,  2357,  2289,  2219,  2150,  2079,  2008,  1936,  1863,  1790,  1716,  1641,  1565,  1489,  1413,
	 1335,  1257,  1178,  1098,  1018,   937,   855,   773,   690,   606,   521,   436,   350,   264,   177,    89,
};

static const u16 hyp_dec_corr_lut[256] = {
	    0,    88,   175,   261,   346,   431,   516,   599,   682,   764,   846,   926,  1006,  1086,  1165,  1243,
	 1320,  1397,  1473,  1548,  1622,  1696,  1770,  1842,  1914,  1985,  2056,  2125,  2194,  2263,  2331,  2398,
	 2464,  2530,  2595,  2659,  2722,  2785,  2848,  2909,  2970,  3030,  3090,  3148,  3206,  3264,  3321,  3377,
	 3432,  3487,  3541,  3594,  3646,  3698,  3750,  3800,  3850,  3899,  3948,  3995,  4042,  4089,  4135,  4180,
	 4224,  4268,  4311,  4353,  4394,  4435,  4476,  4515,  4554,  4592,  4630,  4666,  4702,  4738,  4773,  4807,
	 4840,  4873,  4905,  4936,  4966,  4996,  5026,  5054,  5082,  5109,  5136,  5161,  5186,  5211,  5235,  5258,
	 5280,  5302,  5323,  5343,  5362,  5381,  5400,  5417,  5434,  5450,  5466,  5480,  5494,  5508,  5521,  5533,
	 5544,  5555,  5565,  5574,  5582,  5590,  5598,  5604,  5610,  5615,  5620,  5623,  5626,  5629,  5631,  5632,
	 5632,  5632,  5631,  5629,  5626,  5623,  5620,  5615,  5610,  5604,  5598,  5590,  5582,  5574,  5565,  5555,
	 5544,  5533,  5521,  5508,  5494,  5480,  5466,  5450,  5434,  5417,  5400,  5381,  5362,  5343,  5323,  5302,
	 5280,  5258,  5235,  5211,  5186,  5161,  5136,  5109,  5082,  5053,  5026,  4996,  4966,  4936,  4905,  4873,
	 4840,  4807,  4773,  4738,  4702,  4666,  4630,  4592,  4554,  4515,  4476,  4435,  4394,  4353,  4311,  4268,
	 4224,  4180,  4135,  4089,  4042,  3995,  3948,  3899,  3850,  3800,  3750,  3698,  3646,  3594,  3541,  3487,
	 3432,  3377,  3321,  3264,  3206,  3148,  3090,  3030,  2970,  2909,  2848,  2785,  2722,  2659,  2595,  2530,
	 2464,  2398,  2331,  2263,  2194,  2125,  2056,  1985,  1914,  1842,  1770,  1696,  1622,  1548,  1473,  1397,
	 1320,  1243,  1165,  1086,  1006,   926,   846,   764,   682,   599,   516,   431,   346,   261,   175,    88,
};

/*
 * hyp_lin_to_log - convert u32 to corrected log32fpmax (Q5.26)
 */
static inline s32 hyp_lin_to_log(u32 v)
{
	u8 clz;
	u32 m, mf;
	u8 idx;

	if (!v)
		return HYP_LOG_0;

	clz = __builtin_clz(v);
	m = (v << clz) >> (32 - 1 - HYP_LOG_OFP);
	mf = m & ((1U << HYP_LOG_OFP) - 1);
	idx = (u8)(mf >> (HYP_LOG_OFP - 8));
	m += (u32)hyp_enc_corr_lut[idx] << (HYP_LOG_OFP - 16);

	return (s32)(((u32)(30 - clz) << HYP_LOG_OFP) + m);
}

/*
 * hyp_log_to_lin - convert corrected log32fpmax (Q5.26) back to u32
 */
static inline u32 hyp_log_to_lin(s32 v)
{
	bool negative;
	s32 e;
	u32 m, norm, mh;
	u8 idx;

	if (v == HYP_LOG_0)
		return 0;

	negative = v < 0;
	if (negative)
		v = -v;
	e = v >> HYP_LOG_OFP;
	if (negative)
		e = -e;

	if (e < 0)
		return 0;
	if (e >= 32)
		return U32_MAX;

	m = v & ((1U << HYP_LOG_OFP) - 1);
	norm = (1U << 31) | (m << (31 - HYP_LOG_OFP));
	mh = m << (31 - HYP_LOG_OFP);
	idx = (u8)(mh >> (31 - 8));
	norm -= (u32)hyp_dec_corr_lut[idx] << (31 - 16);

	return norm >> (31 - e);
}

/************************ Governor internals ***********************/

static bool hyp_should_update_freq(struct hyp_policy *hyp_pol, u64 time)
{
	s64 delta_ns;

	if (!cpufreq_this_cpu_can_update(hyp_pol->policy))
		return false;

	if (unlikely(READ_ONCE(hyp_pol->limits_changed))) {
		WRITE_ONCE(hyp_pol->limits_changed, false);
		hyp_pol->need_freq_update = true;

		/*
		 * The above limits_changed update must occur before the reads
		 * of policy limits in cpufreq_driver_resolve_freq() or a policy
		 * limits update might be missed, so use a memory barrier to
		 * ensure it.
		 *
		 * This pairs with the write memory barrier in hyp_limits().
		 */
		smp_mb();

		return true;
	} else if (hyp_pol->need_freq_update) {
		return true;
	}

	delta_ns = time - hyp_pol->last_freq_update_time;

	return delta_ns >= hyp_pol->freq_update_delay_ns;
}

static bool hyp_update_next_freq(struct hyp_policy *hyp_pol, u64 time,
				 unsigned int next_freq)
{
	if (hyp_pol->need_freq_update) {
		hyp_pol->need_freq_update = false;
		if (hyp_pol->next_freq == next_freq &&
		    !cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS))
			return false;
	} else if (hyp_pol->next_freq == next_freq) {
		return false;
	}

	hyp_pol->next_freq = next_freq;
	hyp_pol->last_freq_update_time = time;

	return true;
}

static void hyp_deferred_update(struct hyp_policy *hyp_pol, u64 time,
				unsigned int next_freq)
{
	if (!hyp_pol->work_in_progress) {
		hyp_pol->work_in_progress = true;
		irq_work_queue(&hyp_pol->irq_work);
	}
}

static unsigned int hyp_get_next_freq(struct hyp_policy *hyp_pol,
				      unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = hyp_pol->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : policy->cur;
	unsigned int idx, l_freq, h_freq;
	unsigned long next_freq = 0;

	freq = map_util_freq(util, freq, max);

	if (freq == hyp_pol->cached_raw_freq && !hyp_pol->need_freq_update)
		return hyp_pol->next_freq;

	hyp_pol->cached_raw_freq = freq;
	l_freq = cpufreq_driver_resolve_freq(policy, freq);
	idx = cpufreq_frequency_table_target(policy, freq, CPUFREQ_RELATION_H);
	h_freq = policy->freq_table[idx].frequency;
	h_freq = clamp(h_freq, policy->min, policy->max);
	if (l_freq <= h_freq || l_freq == policy->min)
		return l_freq;

	/*
	 * Use the frequency step below if the calculated frequency is <20%
	 * higher than it.
	 */
	if (mult_frac(100, freq - h_freq, l_freq - h_freq) < 20)
		return h_freq;

	return l_freq;
}

/*
 * Get CPU utilization via the scheduler, identical to schedutil.
 */
static unsigned long hyp_get_util(struct hyp_cpu *hyp_c)
{
	struct rq *rq = cpu_rq(hyp_c->cpu);
	unsigned long util = cpu_util_cfs(rq);
	unsigned long max = arch_scale_cpu_capacity(hyp_c->cpu);

	hyp_c->max = max;
	hyp_c->bw_dl = cpu_bw_dl(rq);

	return schedutil_cpu_util(hyp_c->cpu, util, max, FREQUENCY_UTIL, NULL);
}

/************************ Hispeed (idle-time accounting) ***********************/

/*
 * Update busy% using kernel idle-time accounting (kcpustat).
 * This gives the raw CPU busy ratio without PELT smoothing.
 */
static void hyp_update_busy_pct(struct hyp_cpu *hyp_c,
				unsigned int window_us,
				unsigned int filter_shift, u64 time,
				unsigned long max_cap)
{
	u64 cur_idle, cur_wall;
	unsigned int wall_delta, idle_delta;

	/*
	 * Fast path: hispeed not armed.  The hispeed window can only
	 * have expired if the wall clock advanced window_us since the
	 * last full read.
	 */
	if (!hyp_c->hispeed_active) {
		if (ktime_to_us(ktime_get()) - hyp_c->prev_wall_time < window_us)
			return;
	}

	cur_idle = get_cpu_idle_time(hyp_c->cpu, &cur_wall, 1);
	wall_delta = (unsigned int)(cur_wall - hyp_c->prev_wall_time);

	if (wall_delta >= window_us) {
		/*
		 * Phase 1: Window expired.  Reset busy_pct and request
		 * an immediate measurement on the next callback.
		 */
		hyp_c->busy_pct = 0;
		hyp_c->hispeed_active = true;
		hyp_c->prev_idle_time = cur_idle;
		hyp_c->prev_wall_time = cur_wall;
		return;
	}

	if (!hyp_c->hispeed_active)
		return;

	/* Phase 2: immediate post-reset measurement. */
	hyp_c->hispeed_active = false;

	if (cur_idle > hyp_c->prev_idle_time)
		idle_delta = (unsigned int)(cur_idle - hyp_c->prev_idle_time);
	else
		idle_delta = 0;

	if (wall_delta > idle_delta)
		hyp_c->busy_pct = 100 * (wall_delta - idle_delta) / wall_delta;
	else
		hyp_c->busy_pct = 0;

	hyp_c->prev_idle_time = cur_idle;
	hyp_c->prev_wall_time = cur_wall;

	/*
	 * Asymmetric EWMA filter on busy_pct:
	 *   Up:   instant tracking
	 *   Down: slow ramp-down
	 */
	if (!filter_shift || hyp_c->busy_pct >= hyp_c->filtered_busy_pct) {
		hyp_c->filtered_busy_pct = hyp_c->busy_pct;
	} else {
		unsigned int step = (hyp_c->filtered_busy_pct - hyp_c->busy_pct)
					>> filter_shift;
		if (step)
			hyp_c->filtered_busy_pct -= step;
		else
			hyp_c->filtered_busy_pct = hyp_c->busy_pct;
	}

	/*
	 * Hispeed decay timer tracking with one-window grace period.
	 * The decay timer is only reset after two consecutive idle
	 * windows, preventing idle/busy spinning from defeating decay.
	 */
	if (hyp_c->filtered_busy_pct > 0) {
		hyp_c->hispeed_idle_windows = 0;
		if (!hyp_c->hispeed_start_ns)
			hyp_c->hispeed_start_ns = time;
	} else {
		hyp_c->hispeed_idle_windows++;
		if (hyp_c->hispeed_idle_windows >= 2) {
			hyp_c->hispeed_start_ns = 0;
			hyp_c->filtered_busy_pct = 0;
		}
	}
}

/*
 * Blend PELT utilization with hispeed utilization using PELT-complementary
 * continuous exponential decay.
 *
 *   hispeed_decayed = 2^(log_hispeed + elapsed_ms x log2(y))
 *   blended = min(pelt + hispeed_decayed, hispeed_util)
 *
 * The hispeed level is the measured busy% floor; input boost forces
 * it up to the configured level for its duration, the sustained game
 * floor holds it, and screen-off relaxation caps it.
 */
static unsigned long hyp_blend_util(struct hyp_cpu *hyp_c,
				    unsigned long pelt_util,
				    unsigned long max_cap,
				    struct hyp_tunables *tunables,
				    u64 time)
{
	unsigned long hispeed_util, hispeed_decayed;
	unsigned int elapsed_ms;
	s32 log_hispeed, log_decayed;
	unsigned int pct = hyp_c->filtered_busy_pct;
	bool input_boost = time < READ_ONCE(hyp_input_boost_until);

	if (input_boost)
		pct = max(pct, tunables->input_boost_pct);
	if (hyp_c->sustained)
		pct = max(pct, tunables->sustained_floor_pct);
	if (hyp_screen_off)
		pct = min(pct, tunables->screen_off_cap_pct);

	if (!pct)
		return pelt_util;

	hispeed_util = max_cap * pct / 100;

	/* Game floor: hold the floor instead of decaying */
	if (hyp_c->sustained && hispeed_util > pelt_util)
		return hispeed_util;

	if (hispeed_util <= pelt_util)
		return pelt_util;

	log_hispeed = hyp_lin_to_log(hispeed_util);

	elapsed_ms = (unsigned int)((time - hyp_c->hispeed_start_ns)
				    / NSEC_PER_MSEC);
	if (elapsed_ms >= HYP_LOG_DECAY_MAX_MS)
		return pelt_util;

	log_decayed = log_hispeed + (s32)elapsed_ms * HYP_LOG_DECAY_PER_MS;
	hispeed_decayed = hyp_log_to_lin(log_decayed);

	if (hispeed_decayed <= pelt_util)
		return pelt_util;

	return min(pelt_util + hispeed_decayed, hispeed_util);
}

/************************ I/O wait boost ***********************/

static bool hyp_iowait_reset(struct hyp_cpu *hyp_c, u64 time,
			     bool set_iowait_boost)
{
	s64 delta_ns = time - hyp_c->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	hyp_c->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	hyp_c->iowait_boost_pending = set_iowait_boost;

	return true;
}

static void hyp_iowait_boost(struct hyp_cpu *hyp_c, u64 time,
			     unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	if (hyp_c->iowait_boost &&
	    hyp_iowait_reset(hyp_c, time, set_iowait_boost))
		return;

	if (!set_iowait_boost)
		return;

	if (hyp_c->iowait_boost_pending)
		return;
	hyp_c->iowait_boost_pending = true;

	if (hyp_c->iowait_boost) {
		hyp_c->iowait_boost =
			min_t(unsigned int, hyp_c->iowait_boost << 1,
			      SCHED_CAPACITY_SCALE);
		return;
	}

	hyp_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long hyp_iowait_apply(struct hyp_cpu *hyp_c, u64 time,
				      unsigned long util, unsigned long max)
{
	unsigned long boost;

	if (!hyp_c->iowait_boost)
		return util;

	if (hyp_iowait_reset(hyp_c, time, false))
		return util;

	if (!hyp_c->iowait_boost_pending) {
		hyp_c->iowait_boost >>= 1;
		if (hyp_c->iowait_boost < IOWAIT_BOOST_MIN) {
			hyp_c->iowait_boost = 0;
			return util;
		}
	}

	hyp_c->iowait_boost_pending = false;

	boost = (hyp_c->iowait_boost * max) >> SCHED_CAPACITY_SHIFT;
	boost = max(boost, util);
	boost = uclamp_rq_util_with(cpu_rq(hyp_c->cpu), boost, NULL);

	return boost;
}

/************************ Hold frequency ***********************/

#ifdef CONFIG_NO_HZ_COMMON
static bool hyp_cpu_is_busy(struct hyp_cpu *hyp_c)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(hyp_c->cpu);
	bool ret = idle_calls == hyp_c->saved_idle_calls;

	hyp_c->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool hyp_cpu_is_busy(struct hyp_cpu *hyp_c) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

static inline void hyp_ignore_dl_rate_limit(struct hyp_cpu *hyp_c)
{
	if (cpu_bw_dl(cpu_rq(hyp_c->cpu)) > hyp_c->bw_dl)
		WRITE_ONCE(hyp_c->hyp_policy->limits_changed, true);
}

/************************ Input boost ***********************/

/*
 * Arm the hispeed floor from an input (touch) event.  Called with
 * interrupts disabled, so only store the deadline.
 */
static void hyp_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	/* Touch contact edges only */
	if (type != EV_ABS && type != EV_KEY)
		return;
	if (type == EV_KEY && code != BTN_TOUCH)
		return;

	if (hyp_screen_off)
		return;

	WRITE_ONCE(hyp_input_boost_until,
		   ktime_get_ns() + (u64)hyp_boost_duration_us *
				    NSEC_PER_USEC);
}

static int hyp_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "hyperion";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;

err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void hyp_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

/*
 * Only attach to actual touchscreens/touchpads.  A bare EV_ABS match
 * would also catch accelerometers and other sensors, which report
 * EV_ABS and would otherwise spam the input boost path.
 */
static const struct input_device_id hyp_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
	},
	{ },
};

static struct input_handler hyp_input_handler = {
	.event		= hyp_input_event,
	.connect	= hyp_input_connect,
	.disconnect	= hyp_input_disconnect,
	.name		= "hyperion",
	.id_table	= hyp_input_ids,
};

/************************ Screen state ***********************/

static int hyp_pm_notifier(struct notifier_block *nb, unsigned long mode,
			   void *unused)
{
	switch (mode) {
	case PM_SUSPEND_PREPARE:
		WRITE_ONCE(hyp_screen_off, true);
		break;
	case PM_POST_SUSPEND:
		WRITE_ONCE(hyp_screen_off, false);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block hyp_pm_nb = {
	.notifier_call = hyp_pm_notifier,
};

/************************ Core update callbacks ***********************/

static void hyp_update_single(struct update_util_data *hook, u64 time,
			      unsigned int flags)
{
	struct hyp_cpu *hyp_c = container_of(hook, struct hyp_cpu, update_util);
	struct hyp_policy *hyp_pol = hyp_c->hyp_policy;
	struct hyp_tunables *tunables = hyp_pol->tunables;
	unsigned int cached_freq = hyp_pol->cached_raw_freq;
	unsigned long util, max, effective;
	unsigned int next_f;

	hyp_iowait_boost(hyp_c, time, flags);
	hyp_c->last_update = time;

	hyp_ignore_dl_rate_limit(hyp_c);

	if (!hyp_should_update_freq(hyp_pol, time))
		return;

	util = hyp_get_util(hyp_c);
	max = hyp_c->max;
	util = hyp_iowait_apply(hyp_c, time, util, max);

	hyp_update_busy_pct(hyp_c, tunables->hispeed_window_us,
			    tunables->hispeed_filter_shift, time, max);

	/* Touch boost arms the floor immediately */
	if (time < READ_ONCE(hyp_input_boost_until))
		hyp_c->hispeed_start_ns = time;

	/* Sustained game floor: hold a floor while busy% stays high */
	if (hyp_c->filtered_busy_pct >= tunables->sustained_threshold_pct) {
		if (!hyp_c->sustained_since)
			hyp_c->sustained_since = time;
		if (time - hyp_c->sustained_since >=
		    (u64)tunables->sustained_window_ms * NSEC_PER_MSEC)
			hyp_c->sustained = true;
	} else {
		hyp_c->sustained_since = 0;
		hyp_c->sustained = false;
	}

	effective = hyp_blend_util(hyp_c, util, max, tunables, time);
	next_f = hyp_get_next_freq(hyp_pol, effective, max);

	/*
	 * Do not reduce the frequency if the CPU has not been idle
	 * recently, as the reduction is likely to be premature then.
	 */
	if (hyp_cpu_is_busy(hyp_c) && next_f < hyp_pol->next_freq) {
		next_f = hyp_pol->next_freq;
		hyp_pol->cached_raw_freq = cached_freq;
	}

	if (!hyp_update_next_freq(hyp_pol, time, next_f))
		return;

	if (hyp_pol->policy->fast_switch_enabled)
		cpufreq_driver_fast_switch(hyp_pol->policy, next_f);
	else
		hyp_deferred_update(hyp_pol, time, next_f);
}

static unsigned int hyp_next_freq_shared(struct hyp_cpu *hyp_c, u64 time)
{
	struct hyp_policy *hyp_pol = hyp_c->hyp_policy;
	struct hyp_tunables *tunables = hyp_pol->tunables;
	struct cpufreq_policy *policy = hyp_pol->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct hyp_cpu *j_hyp_c = &per_cpu(hyp_cpu, j);
		unsigned long j_util, j_max, j_effective;

		j_util = hyp_get_util(j_hyp_c);
		j_max = j_hyp_c->max;
		j_util = hyp_iowait_apply(j_hyp_c, time, j_util, j_max);

		hyp_update_busy_pct(j_hyp_c, tunables->hispeed_window_us,
				    tunables->hispeed_filter_shift, time,
				    j_max);

		if (time < READ_ONCE(hyp_input_boost_until))
			j_hyp_c->hispeed_start_ns = time;

		if (j_hyp_c->filtered_busy_pct >=
		    tunables->sustained_threshold_pct) {
			if (!j_hyp_c->sustained_since)
				j_hyp_c->sustained_since = time;
			if (time - j_hyp_c->sustained_since >=
			    (u64)tunables->sustained_window_ms * NSEC_PER_MSEC)
				j_hyp_c->sustained = true;
		} else {
			j_hyp_c->sustained_since = 0;
			j_hyp_c->sustained = false;
		}

		j_effective = hyp_blend_util(j_hyp_c, j_util, j_max, tunables,
					     time);

		if (j_effective * max > util * j_max) {
			util = j_effective;
			max = j_max;
		}
	}

	return hyp_get_next_freq(hyp_pol, util, max);
}

static void
hyp_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct hyp_cpu *hyp_c = container_of(hook, struct hyp_cpu, update_util);
	struct hyp_policy *hyp_pol = hyp_c->hyp_policy;
	unsigned int next_f;

	raw_spin_lock(&hyp_pol->update_lock);

	hyp_iowait_boost(hyp_c, time, flags);
	hyp_c->last_update = time;

	hyp_ignore_dl_rate_limit(hyp_c);

	if (hyp_should_update_freq(hyp_pol, time)) {
		next_f = hyp_next_freq_shared(hyp_c, time);

		if (!hyp_update_next_freq(hyp_pol, time, next_f))
			goto unlock;

		if (hyp_pol->policy->fast_switch_enabled)
			cpufreq_driver_fast_switch(hyp_pol->policy, next_f);
		else
			hyp_deferred_update(hyp_pol, time, next_f);
	}
unlock:
	raw_spin_unlock(&hyp_pol->update_lock);
}

/************************ Kthread (slow path) ***********************/

static void hyp_work(struct kthread_work *work)
{
	struct hyp_policy *hyp_pol = container_of(work, struct hyp_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&hyp_pol->update_lock, flags);
	freq = hyp_pol->next_freq;
	hyp_pol->work_in_progress = false;
	raw_spin_unlock_irqrestore(&hyp_pol->update_lock, flags);

	mutex_lock(&hyp_pol->work_lock);
	__cpufreq_driver_target(hyp_pol->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&hyp_pol->work_lock);
}

static void hyp_irq_work(struct irq_work *irq_work)
{
	struct hyp_policy *hyp_pol;

	hyp_pol = container_of(irq_work, struct hyp_policy, irq_work);

	kthread_queue_work(&hyp_pol->worker, &hyp_pol->work);
}

/************************** sysfs interface ************************/

static struct hyp_tunables *hyp_global_tunables;
static DEFINE_MUTEX(hyp_global_tunables_lock);

static inline struct hyp_tunables *to_hyp_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct hyp_tunables, attr_set);
}

#define HYP_TUNABLE_UINT(name)						\
static ssize_t name##_show(struct gov_attr_set *attr_set, char *buf)	\
{									\
	struct hyp_tunables *t = to_hyp_tunables(attr_set);		\
	return sprintf(buf, "%u\n", t->name);				\
}									\
static ssize_t								\
name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{									\
	struct hyp_tunables *t = to_hyp_tunables(attr_set);		\
	unsigned int val;						\
	if (kstrtouint(buf, 10, &val))					\
		return -EINVAL;						\
	t->name = val;							\
	return count;							\
}									\
static struct governor_attr name = __ATTR_RW(name)

static ssize_t hyp_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct hyp_tunables *tunables = to_hyp_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->rate_limit_us);
}

static ssize_t
hyp_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf,
			size_t count)
{
	struct hyp_tunables *tunables = to_hyp_tunables(attr_set);
	struct hyp_policy *hyp_pol;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->rate_limit_us = rate_limit_us;

	list_for_each_entry(hyp_pol, &attr_set->policy_list, tunables_hook)
		hyp_pol->freq_update_delay_ns = rate_limit_us * NSEC_PER_USEC;

	return count;
}

static struct governor_attr hyp_rate_limit_us =
	__ATTR(rate_limit_us, 0644, hyp_rate_limit_us_show, hyp_rate_limit_us_store);

static ssize_t hyp_input_boost_pct_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct hyp_tunables *tunables = to_hyp_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->input_boost_pct);
}

static ssize_t hyp_input_boost_pct_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct hyp_tunables *tunables = to_hyp_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;

	tunables->input_boost_pct = val;
	WRITE_ONCE(hyp_boost_pct, val);

	return count;
}

static ssize_t hyp_input_boost_duration_us_show(struct gov_attr_set *attr_set,
						char *buf)
{
	struct hyp_tunables *tunables = to_hyp_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->input_boost_duration_us);
}

static ssize_t hyp_input_boost_duration_us_store(struct gov_attr_set *attr_set,
						 const char *buf, size_t count)
{
	struct hyp_tunables *tunables = to_hyp_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->input_boost_duration_us = val;
	WRITE_ONCE(hyp_boost_duration_us, val);

	return count;
}

static ssize_t version_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%s\n", CPUFREQ_HYPERION_VERSION);
}
static struct governor_attr version = __ATTR_RO(version);

HYP_TUNABLE_UINT(hispeed_window_us);
HYP_TUNABLE_UINT(hispeed_filter_shift);
HYP_TUNABLE_UINT(sustained_threshold_pct);
HYP_TUNABLE_UINT(sustained_window_ms);
HYP_TUNABLE_UINT(sustained_floor_pct);
HYP_TUNABLE_UINT(screen_off_cap_pct);

static struct governor_attr hyp_input_boost_pct =
	__ATTR(input_boost_pct, 0644, hyp_input_boost_pct_show,
	       hyp_input_boost_pct_store);
static struct governor_attr hyp_input_boost_duration_us =
	__ATTR(input_boost_duration_us, 0644, hyp_input_boost_duration_us_show,
	       hyp_input_boost_duration_us_store);

static struct attribute *hyp_attrs[] = {
	&version.attr,
	&hyp_rate_limit_us.attr,
	&hispeed_window_us.attr,
	&hispeed_filter_shift.attr,
	&hyp_input_boost_pct.attr,
	&hyp_input_boost_duration_us.attr,
	&sustained_threshold_pct.attr,
	&sustained_window_ms.attr,
	&sustained_floor_pct.attr,
	&screen_off_cap_pct.attr,
	NULL
};
ATTRIBUTE_GROUPS(hyp);

static void hyp_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);

	kfree(to_hyp_tunables(attr_set));
}

static struct kobj_type hyp_tunables_ktype = {
	.default_groups = hyp_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &hyp_tunables_free,
};

/********************** cpufreq governor interface *********************/

struct cpufreq_governor hyperion_gov;

static struct hyp_policy *hyp_policy_alloc(struct cpufreq_policy *policy)
{
	struct hyp_policy *hyp_pol;

	hyp_pol = kzalloc(sizeof(*hyp_pol), GFP_KERNEL);
	if (!hyp_pol)
		return NULL;

	hyp_pol->policy = policy;
	raw_spin_lock_init(&hyp_pol->update_lock);
	return hyp_pol;
}

static void hyp_policy_free(struct hyp_policy *hyp_pol)
{
	kfree(hyp_pol);
}

static int hyp_kthread_create(struct hyp_policy *hyp_pol)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAG_SUGOV,
		.sched_nice	= 0,
		.sched_priority	= 0,
		/*
		 * Fake (unused) bandwidth; workaround to "fix"
		 * priority inheritance.
		 */
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
	struct cpufreq_policy *policy = hyp_pol->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&hyp_pol->work, hyp_work);
	kthread_init_worker(&hyp_pol->worker);
	thread = kthread_create(kthread_worker_fn, &hyp_pol->worker,
				"hypgov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create hyperion thread: %ld\n",
		       PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_DEADLINE\n", __func__);
		return ret;
	}

	hyp_pol->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&hyp_pol->irq_work, hyp_irq_work);
	mutex_init(&hyp_pol->work_lock);

	wake_up_process(thread);

	return 0;
}

static void hyp_kthread_stop(struct hyp_policy *hyp_pol)
{
	/* kthread only required for slow path */
	if (hyp_pol->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&hyp_pol->worker);
	kthread_stop(hyp_pol->thread);
	mutex_destroy(&hyp_pol->work_lock);
}

static struct hyp_tunables *hyp_tunables_alloc(struct hyp_policy *hyp_pol)
{
	struct hyp_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &hyp_pol->tunables_hook);
		if (!have_governor_per_policy())
			hyp_global_tunables = tunables;
	}
	return tunables;
}

static void hyp_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		hyp_global_tunables = NULL;
}

static int hyp_init(struct cpufreq_policy *policy)
{
	struct hyp_policy *hyp_pol;
	struct hyp_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	hyp_pol = hyp_policy_alloc(policy);
	if (!hyp_pol) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = hyp_kthread_create(hyp_pol);
	if (ret)
		goto free_hyp_pol;

	mutex_lock(&hyp_global_tunables_lock);

	if (hyp_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = hyp_pol;
		hyp_pol->tunables = hyp_global_tunables;

		gov_attr_set_get(&hyp_global_tunables->attr_set,
				 &hyp_pol->tunables_hook);
		goto out;
	}

	tunables = hyp_tunables_alloc(hyp_pol);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	/* Default tunable values */
	tunables->rate_limit_us = cpufreq_policy_transition_delay_us(policy);
	tunables->hispeed_window_us = HYPERION_DEFAULT_HISPEED_WINDOW_US;
	tunables->hispeed_filter_shift = HYPERION_DEFAULT_HISPEED_FILTER_SHIFT;
	tunables->input_boost_pct = HYPERION_DEFAULT_INPUT_BOOST_PCT;
	tunables->input_boost_duration_us =
		HYPERION_DEFAULT_INPUT_BOOST_DURATION_US;
	tunables->sustained_threshold_pct =
		HYPERION_DEFAULT_SUSTAINED_THRESHOLD_PCT;
	tunables->sustained_window_ms = HYPERION_DEFAULT_SUSTAINED_WINDOW_MS;
	tunables->sustained_floor_pct = HYPERION_DEFAULT_SUSTAINED_FLOOR_PCT;
	tunables->screen_off_cap_pct = HYPERION_DEFAULT_SCREEN_OFF_CAP_PCT;

	policy->governor_data = hyp_pol;
	hyp_pol->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &hyp_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   hyperion_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&hyp_global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	hyp_clear_global_tunables();

stop_kthread:
	hyp_kthread_stop(hyp_pol);
	mutex_unlock(&hyp_global_tunables_lock);

free_hyp_pol:
	hyp_policy_free(hyp_pol);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void hyp_exit(struct cpufreq_policy *policy)
{
	struct hyp_policy *hyp_pol = policy->governor_data;
	struct hyp_tunables *tunables = hyp_pol->tunables;
	unsigned int count;

	mutex_lock(&hyp_global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &hyp_pol->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		hyp_clear_global_tunables();

	mutex_unlock(&hyp_global_tunables_lock);

	hyp_kthread_stop(hyp_pol);
	hyp_policy_free(hyp_pol);
	cpufreq_disable_fast_switch(policy);
}

static int hyp_start(struct cpufreq_policy *policy)
{
	struct hyp_policy *hyp_pol = policy->governor_data;
	unsigned int cpu;

	hyp_pol->freq_update_delay_ns	= hyp_pol->tunables->rate_limit_us *
					  NSEC_PER_USEC;
	hyp_pol->last_freq_update_time	= 0;
	hyp_pol->next_freq		= 0;
	hyp_pol->work_in_progress	= false;
	hyp_pol->limits_changed		= false;
	hyp_pol->cached_raw_freq	= 0;

	hyp_pol->need_freq_update =
		cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS);

	for_each_cpu(cpu, policy->cpus) {
		struct hyp_cpu *hyp_c = &per_cpu(hyp_cpu, cpu);

		memset(hyp_c, 0, sizeof(*hyp_c));
		hyp_c->cpu		= cpu;
		hyp_c->hyp_policy	= hyp_pol;
		/* Initialize idle-time baseline for hispeed busy% */
		hyp_c->prev_idle_time = get_cpu_idle_time(cpu,
					&hyp_c->prev_wall_time, 1);
	}

	for_each_cpu(cpu, policy->cpus) {
		struct hyp_cpu *hyp_c = &per_cpu(hyp_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &hyp_c->update_util,
					     policy_is_shared(policy) ?
						hyp_update_shared :
						hyp_update_single);
	}
	return 0;
}

static void hyp_stop(struct cpufreq_policy *policy)
{
	struct hyp_policy *hyp_pol = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&hyp_pol->irq_work);
		kthread_cancel_work_sync(&hyp_pol->work);
	}
}

static void hyp_limits(struct cpufreq_policy *policy)
{
	struct hyp_policy *hyp_pol = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&hyp_pol->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&hyp_pol->work_lock);
	}

	/*
	 * The limits_changed update below must take place before the updates
	 * of policy limits in cpufreq_set_policy() or a policy limits update
	 * might be missed, so use a memory barrier to ensure it.
	 *
	 * This pairs with the memory barrier in hyp_should_update_freq().
	 */
	smp_wmb();

	WRITE_ONCE(hyp_pol->limits_changed, true);
}

struct cpufreq_governor hyperion_gov = {
	.name			= "hyperion",
	.owner			= THIS_MODULE,
	.flags			= CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init			= hyp_init,
	.exit			= hyp_exit,
	.start			= hyp_start,
	.stop			= hyp_stop,
	.limits			= hyp_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_HYPERION
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &hyperion_gov;
}
#endif

static int __init hyperion_gov_init(void)
{
	return cpufreq_register_governor(&hyperion_gov);
}

/*
 * The input handler and pm notifier hook into subsystems that come up
 * after the governor itself, so register them in a later initcall.
 */
static int __init hyperion_extras_init(void)
{
	int ret;

	ret = input_register_handler(&hyp_input_handler);
	if (ret)
		return ret;

	register_pm_notifier(&hyp_pm_nb);

	pr_info("%s %s\n", CPUFREQ_HYPERION_PROGNAME,
		CPUFREQ_HYPERION_VERSION);

	return 0;
}

static void __exit hyperion_gov_exit(void)
{
	unregister_pm_notifier(&hyp_pm_nb);
	input_unregister_handler(&hyp_input_handler);
	cpufreq_unregister_governor(&hyperion_gov);
}

core_initcall(hyperion_gov_init);
late_initcall(hyperion_extras_init);
module_exit(hyperion_gov_exit);

MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION(CPUFREQ_HYPERION_PROGNAME);
MODULE_LICENSE("GPL");
