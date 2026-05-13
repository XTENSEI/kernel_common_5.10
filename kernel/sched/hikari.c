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

void hikari_apply_wake_shift(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct task_struct *p = container_of(se, struct task_struct, se);
	unsigned int hshift = READ_ONCE(sysctl_sched_hikari_shift);
	unsigned int eshift = READ_ONCE(sysctl_sched_hikari_ewma_shift);
	u64 cur_wait_sum, last_wait_sum, delta, ewma, shift_amt, floor, target;

	if (!hshift)
		return;					/* runtime-disabled */

	/*
	 * Defensive clamp: proc_dou8vec_minmax already enforces 1..8 on the
	 * sysctl, but a 0 or out-of-range value here would either divide by
	 * zero conceptually (shift by 0 saturates the multiplier) or shift
	 * by >= 64 which is UB.  Fall back to the default.
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
		return;
	}

	delta = cur_wait_sum - last_wait_sum;
	p->hikari.last_wait_sum = cur_wait_sum;

	ewma = p->hikari.wait_ewma;
	ewma = ((ewma * ((1ULL << eshift) - 1)) + delta) >> eshift;
	p->hikari.wait_ewma = ewma;

	shift_amt = min_t(u64, ewma >> hshift, sysctl_sched_latency);
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
		zenith_signal_wake_demand(task_cpu(p));

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
