// SPDX-License-Identifier: GPL-2.0
/*
 * Hikari wake-latency vruntime shift.
 *
 * On wake-up, shift a task's vruntime backwards by a fraction of its
 * recent average wake-to-run wait time.  Tasks the scheduler has been
 * failing get a small boost on the next wake; tasks being served well
 * get nothing.
 *
 * The shift is bounded above by sysctl_sched_latency (so it never
 * exceeds the credit place_entity() itself can grant via
 * GENTLE_FAIR_SLEEPERS) and floored at min_vruntime -
 * sysctl_sched_latency (the same lower bound place_entity() enforces),
 * keeping the rb-tree entity_before() ordering well defined.
 *
 * No score, no reweight, no priority math -- pure vruntime nudge.
 *
 * See Documentation/scheduler/sched-hikari.rst for design notes,
 * including the dependency on runtime schedstats and the resulting
 * footgun when kernel.sched_schedstats=0.
 */
#include <linux/cpufreq_zenith.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/sched.h>
#include <linux/sched/sysctl.h>

#include "sched.h"

/*
 * Right-shift applied to the EWMA before subtracting from vruntime.
 *  0 disables Hikari at runtime (the hook becomes a no-op);
 *  higher values produce milder shifts.
 *
 * Default 4 means a steady 5 ms wait yields a ~0.3 ms shift, well
 * below sysctl_sched_latency, acting as a tiebreaker against equally
 * starved tasks rather than as a large preferential boost.
 *
 * With this default != 0 the kernel is intentionally NOT byte-for-byte
 * equivalent to vanilla CFS at boot.  Set the sysctl to 0 to restore
 * stock CFS placement.
 */
unsigned int sysctl_sched_hikari_shift __read_mostly = 4;

/*
 * EWMA smoothing constant.  new = (old * ((1 << s) - 1) + delta) >> s.
 *   1 -> nearly instantaneous;
 *   8 -> ~256-sample window.
 * Default 3 = ((old * 7) + delta) >> 3, mild smoothing.
 */
unsigned int sysctl_sched_hikari_ewma_shift __read_mostly = 3;

/*
 * Optional top-app shift multiplier.  When non-zero, tasks running in
 * the "top-app" cpuset cgroup get their shift left-shifted by this
 * value (still capped at sysctl_sched_latency), amplifying the
 * placement nudge for the foreground UI process group.
 *
 * Default 0 (disabled) so the helper has zero overhead unless the
 * user opts in.  Range 0..2; values above 2 saturate against the
 * sysctl_sched_latency ceiling on every meaningful EWMA.
 *
 * Stacks with zenith's existing top_app_floor_pct frequency floor;
 * tune both together if you turn this on.
 */
unsigned int sysctl_sched_hikari_topapp_boost __read_mostly;

void hikari_apply_wake_shift(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct task_struct *p = container_of(se, struct task_struct, se);
	unsigned int hshift = READ_ONCE(sysctl_sched_hikari_shift);
	unsigned int eshift = READ_ONCE(sysctl_sched_hikari_ewma_shift);
	unsigned int boost  = READ_ONCE(sysctl_sched_hikari_topapp_boost);
	u64 cur_wait_sum, last_wait_sum, delta, ewma, shift_amt, floor, target;
	int cpu;

	if (!hshift)
		return;					/* runtime-disabled */

	cpu = task_cpu(p);

	/*
	 * Hook H1-2(a): screen-off bypass.  When zenith reports the
	 * panel is blanked there is no UX consumer for a vruntime nudge;
	 * skip the EWMA update and the placement work entirely.  The
	 * task's last_wait_sum is left in place so the next on-screen
	 * wake resumes with the existing snapshot rather than re-priming.
	 *
	 * Stubs to false (i.e. always proceed) when zenith is not built
	 * or not the active governor on this CPU.
	 */
	if (zenith_cpu_screen_off(cpu))
		return;

	/*
	 * Defensive clamp: the sysctl handler enforces 1..8 on writes, but
	 * a 0 or out-of-range value here would either divide by zero
	 * conceptually (shift by 0 saturates the multiplier) or shift by
	 * >= 64 which is UB.  Fall back to the default.
	 */
	if (!eshift || eshift > 8)
		eshift = 3;

	cur_wait_sum  = se->statistics.wait_sum;
	last_wait_sum = p->hikari.last_wait_sum;

	/*
	 * Two priming paths, both detected here:
	 *
	 *   1) last_wait_sum == 0:  first observation after fork (or after
	 *      a previous reset).  Just snapshot the current wait_sum so
	 *      the next call sees a real delta.
	 *
	 *   2) cur_wait_sum < last_wait_sum:  wait_sum got reset under us.
	 *      Writing kernel.sched_schedstats=1 zeroes se->statistics
	 *      across every task.  Without this guard the u64 subtraction
	 *      would wrap to a huge value and poison the EWMA for many
	 *      wakes.  Re-snapshot and drop the stale average.
	 *
	 * Either way we must not feed a bogus delta into the EWMA, and we
	 * intentionally skip applying a shift on this priming wake.
	 */
	if (!last_wait_sum || cur_wait_sum < last_wait_sum) {
		p->hikari.last_wait_sum = cur_wait_sum;
		p->hikari.wait_ewma     = 0;
		p->hikari.last_shift    = 0;
		return;
	}

	delta = cur_wait_sum - last_wait_sum;
	p->hikari.last_wait_sum = cur_wait_sum;

	ewma = p->hikari.wait_ewma;
	ewma = ((ewma * ((1ULL << eshift) - 1)) + delta) >> eshift;
	p->hikari.wait_ewma = ewma;

	shift_amt = min_t(u64, ewma >> hshift, sysctl_sched_latency);

	/*
	 * Hook H1-2(b): tighten the cap during audio playback.  Audio
	 * pipelines are paced by the codec, not by CFS; large vruntime
	 * reorderings can produce audible jitter even without missing a
	 * deadline.  Half the scheduler latency horizon is enough to
	 * stay useful as a tiebreaker without disturbing playback.
	 */
	if (zenith_cpu_audio_active(cpu)) {
		u64 audio_cap = sysctl_sched_latency >> 1;

		if (audio_cap && shift_amt > audio_cap)
			shift_amt = audio_cap;
	}

	/*
	 * Hook H1-2(c): optional top-app shift multiplier.  Gated on the
	 * sysctl being non-zero so the cpuset cgroup walk is skipped
	 * entirely in the default configuration.  Result is clamped back
	 * to sysctl_sched_latency.
	 */
	if (boost && shift_amt && zenith_task_is_top_app(p)) {
		u64 boosted = shift_amt << boost;

		shift_amt = min_t(u64, boosted, sysctl_sched_latency);
	}

	p->hikari.last_shift = shift_amt;

	if (!shift_amt)
		return;

	/*
	 * Hook H1-1: when the shift saturates at sysctl_sched_latency the
	 * task has been waiting at least as long as the scheduler's own
	 * latency horizon -- a leading-edge "this CPU is starving a
	 * runnable task" signal.  Forward it to zenith so its cluster-
	 * wake-pulse soft floor can ramp frequency immediately instead
	 * of waiting one or two util-sample windows.  No-op when zenith
	 * is not the active governor or is not built.
	 */
	if (shift_amt == sysctl_sched_latency)
		zenith_signal_wake_demand(cpu);

	target = se->vruntime - shift_amt;
	floor  = cfs_rq->min_vruntime - sysctl_sched_latency;

	/*
	 * Inline of fair.c::max_vruntime() to keep cyclic-vruntime
	 * semantics safe at early boot when min_vruntime may still be
	 * smaller than sysctl_sched_latency.  Pick whichever of (target,
	 * floor) is "later" in signed vruntime time.
	 */
	if ((s64)(target - floor) > 0)
		se->vruntime = target;
	else
		se->vruntime = floor;
}
