============================================
Hikari -- wake-latency-driven vruntime shift
============================================

Hikari is a small, self-correcting add-on to CFS that nudges the
vruntime of waking tasks backwards by a fraction of their own recent
average wake-to-run wait time.  Tasks the scheduler has been failing to
serve get a small boost on their next wake; tasks that have been served
well get nothing.

Closed-loop where BORE was open-loop.  There is no burst score, no
priority reweight and no preempt-decision change -- just a bounded
shift of ``se->vruntime`` inside ``enqueue_entity()``.

Hook site
=========

The single hook lives in ``kernel/sched/fair.c`` inside
``enqueue_entity()``, right after the existing ``place_entity()`` call::

	if (flags & ENQUEUE_WAKEUP) {
		place_entity(cfs_rq, se, 0);
		if (entity_is_task(se))
			hikari_apply_wake_shift(cfs_rq, se);
	}

The ``entity_is_task(se)`` gate ensures the shift only ever applies to
the task-level entity, never to a group entity in the
``for_each_sched_entity`` walk.

Formula
=======

For every wake the hook computes

::

	delta     = se->statistics.wait_sum - p->hikari.last_wait_sum;
	ewma      = ((ewma_prev * ((1 << eshift) - 1)) + delta) >> eshift;
	shift_amt = min(ewma >> hshift, sysctl_sched_latency);

	target = se->vruntime - shift_amt;
	floor  = cfs_rq->min_vruntime - sysctl_sched_latency;
	se->vruntime = (target later than floor in cyclic order) ?
	               target : floor;

The cap at ``sysctl_sched_latency`` and the floor at
``min_vruntime - sysctl_sched_latency`` mirror the bounds that
``place_entity()`` itself enforces, so ``entity_before()`` and the
rb-tree ordering invariants are preserved.

Per-task state
==============

Three ``u64`` fields are packed into the existing
``ANDROID_KABI_RESERVE(2)``, ``ANDROID_KABI_RESERVE(3)`` and
``ANDROID_KABI_RESERVE(4)`` slots of ``struct task_struct`` via
``_ANDROID_KABI_REPLACE``:

* ``hikari.wait_ewma``     -- smoothed recent wake-to-run wait time
* ``hikari.last_wait_sum`` -- snapshot of ``se.statistics.wait_sum`` at
  the previous wake, used to compute the delta
* ``hikari.last_shift``    -- the shift actually applied on the last
  wake (post audio cap, post top-app boost), zero on the priming
  paths.  Exposed read-only via ``/proc/<pid>/sched`` for tuning
  visibility.

Struct size and the offsets of every following field are preserved, so
this is KMI-safe by construction.

All three fields are zeroed in ``__sched_fork()`` so the first wake
after fork takes the priming path (see below) rather than treating
the parent's accumulated wait_sum as the child's own.

Priming and reset handling
==========================

The hook detects two situations and skips the shift on that wake:

1. **First sample.**  When ``last_wait_sum == 0`` the task has never
   been observed before.  The current ``wait_sum`` is snapshotted and
   we return without seeding the EWMA from what would otherwise be a
   spuriously large initial delta (the parent's accumulated wait_sum
   carried in via ``dup_task_struct``, or the task's own pre-first-sleep
   accumulation).

2. **wait_sum reset.**  ``echo 1 > /proc/sys/kernel/sched_schedstats``
   zeroes ``se->statistics`` across every task on the system.  If the
   next observed ``wait_sum`` is *less* than ``last_wait_sum`` the u64
   subtraction would wrap to a near-maximal delta and poison the EWMA
   for many subsequent wakes.  Treat this as a reset: re-snapshot
   ``last_wait_sum``, drop ``wait_ewma`` and ``last_shift`` to zero,
   return.

Tunables
========

Three ``u8`` sysctls live under ``/proc/sys/kernel/``:

``sched_hikari_shift`` (default ``4``, range ``0..10``)
	Right-shift applied to the EWMA before subtraction.

	* ``0`` disables Hikari at runtime; the hook becomes a no-op and
	  the scheduler is byte-for-byte equivalent to vanilla CFS.
	* Higher values produce milder shifts.  At the default of ``4``,
	  a steady 5 ms wait yields a ~0.3 ms shift, which is well below
	  ``sysctl_sched_latency`` and acts as a tiebreaker against equally
	  starved tasks rather than as a large preferential boost.
	* Values above ``8`` are accepted as tuning headroom but the
	  resulting per-wake shift quickly drops below the scheduler's
	  own minimum granularity and stops mattering in practice.

``sched_hikari_ewma_shift`` (default ``3``, range ``1..8``)
	EWMA smoothing constant.
	``new = (old * ((1 << s) - 1) + delta) >> s``.

	* ``1`` -- nearly instantaneous.
	* ``8`` -- ~256-sample window.
	* Default ``3`` is ``((old * 7) + delta) >> 3``, mild smoothing
	  resistant to one-off latency outliers.

``sched_hikari_topapp_boost`` (default ``0``, range ``0..2``)
	Optional left-shift multiplier applied to ``shift_amt`` for tasks
	running in the ``top-app`` cpuset cgroup.  See Patch H1-2(c)
	below.

	* ``0`` disables the boost.  This is the default and makes the
	  helper a no-op -- the cpuset cgroup walk is skipped entirely.
	* ``1`` doubles the shift, ``2`` quadruples it (still clamped to
	  ``sysctl_sched_latency``).
	* Stacks with zenith's existing ``top_app_floor_pct`` frequency
	  floor; tune both knobs together if you enable this.

Dependency on runtime schedstats
================================

Hikari reads ``se->statistics.wait_sum``, which is updated only when
``schedstat_enabled()`` is true at runtime, regardless of whether
``CONFIG_SCHEDSTATS`` is built in.  If userspace has set
``kernel.sched_schedstats=0`` then ``wait_sum`` stays zero for every
task, every observed delta is zero, the EWMA never grows, and Hikari is
a silent no-op.

This is intentional -- forcibly toggling ``sched_schedstats`` from
inside the wake path would be a much bigger policy change than the
mechanism warrants -- but it is a footgun for anyone benchmarking
Hikari.  Make sure schedstats is enabled at runtime
(``echo 1 > /proc/sys/kernel/sched_schedstats``) before drawing
conclusions about whether Hikari is doing anything.

Default-y rationale
===================

``CONFIG_SCHED_HIKARI`` defaults to ``y`` and the runtime knob defaults
to ``4`` (i.e. active).  This is an intentional deviation from the
"a freshly-booted kernel must be byte-for-byte equivalent to one built
without the feature" rule that other Android-common scheduler additions
have followed.

To restore stock CFS behaviour on a built-in kernel set::

	echo 0 > /proc/sys/kernel/sched_hikari_shift

Zenith coupling (Patch H1-1)
============================

When the shift saturates at ``sysctl_sched_latency`` -- that is, the
task has been waiting at least one full scheduler latency horizon --
Hikari forwards a one-shot wake-demand signal to the zenith cpufreq
governor via ``zenith_signal_wake_demand(cpu)``.

Zenith reuses its existing cluster-wake-pulse soft-floor tier: floor
magnitude comes from ``cluster_wake_pulse_floor_pct`` of
``policy->max``, duration from ``cluster_wake_pulse_ms``.  No new
tunable, no new tier.

Semantics:

* The hook is a single ``WRITE_ONCE`` u64 store on the zenith side,
  callable from arbitrary scheduler context with ``rq->lock`` held.
* Stubbed out to a no-op when ``CONFIG_CPU_FREQ_GOV_ZENITH=n``.
* Stubbed out at runtime when the CPU is not currently governed by
  zenith, when ``policy->governor_data`` is NULL (zenith torn down),
  or when ``cluster_wake_pulse_ms`` is set to zero on the policy.
* Setting ``kernel.sched_hikari_shift=0`` disables Hikari entirely
  and therefore also disables this signal -- there is no separate
  knob to toggle the integration.

Rationale: ``shift_amt`` saturation is a leading-edge indicator of
runnable-task starvation, available roughly one scheduling event
before zenith's own util-based ramp would see the demand.  Reusing
the existing cluster-wake-pulse tier means the two tiers cannot
disagree on how high a freshly-rescued cluster should run, and
disabling either ``cluster_wake_pulse_ms`` or
``cluster_wake_pulse_floor_pct`` disables both arming pathways at
once.

This integration is an intentional deviation from the "Hikari Mini
is a single logical change in one subsystem" framing of the rest of
the patch and is called out in the commit message.

Class scope and inheritance
===========================

* Class scope: ``fair_sched_class`` only.  RT, deadline, idle and stop
  classes are untouched because the hook lives in
  ``kernel/sched/fair.c``.

* Fork inheritance: none.  Children start with
  ``hikari = { 0, 0 }`` from ``__sched_fork()``, and the first observed
  wake takes the priming path.

* No new ``EXPORT_SYMBOL``.  ``hikari_apply_wake_shift`` is internal to
  ``kernel/sched/`` and reached only through the inline declaration in
  ``kernel/sched/hikari.h``.  ``zenith_signal_wake_demand`` is declared
  in ``include/linux/cpufreq_zenith.h`` and resolved at link time
  without an exported symbol.

Context modulation (Patch H1-2)
===============================

Three additional zenith reads inside the wake-shift hook, all
lockless and all stub-to-false when zenith is not built or not the
active governor on the queried CPU.

(a) **Screen-off bypass.**  If ``zenith_cpu_screen_off(cpu)`` returns
    true the hook returns immediately, before any EWMA update.  No
    UX consumer is watching the placement nudge while the panel is
    blanked; ``wait_ewma`` and ``last_wait_sum`` are left in place so
    the next on-screen wake resumes with the existing snapshot
    rather than re-priming.

(b) **Audio cap.**  If ``zenith_cpu_audio_active(cpu)`` returns true,
    ``shift_amt`` is capped at ``sysctl_sched_latency / 2``.  Audio
    pipelines are paced by the codec, not by CFS; large vruntime
    reorderings can produce audible jitter even without missing a
    buffer deadline.  Half the latency horizon stays useful as a
    tiebreaker without disturbing playback.

(c) **Top-app boost.**  When ``kernel.sched_hikari_topapp_boost`` is
    non-zero and ``zenith_task_is_top_app(p)`` returns true,
    ``shift_amt`` is left-shifted by the sysctl value (still clamped
    to ``sysctl_sched_latency``).  The cpuset cgroup walk is gated
    behind the sysctl, so the default (boost=0) costs nothing.

All three reads land inside the existing
``hikari_apply_wake_shift`` body; no new hook sites in ``fair.c``,
``core.c`` or the zenith eval path.

Debug visibility
================

When ``CONFIG_SCHED_HIKARI`` is enabled, ``/proc/<pid>/sched`` (and
the per-task section of ``/proc/sched_debug``) prints two extra
fields::

	hikari.wait_ewma                            :          xxxxx.xxxxxx
	hikari.last_shift                           :          xxxxx.xxxxxx

Both are in nanoseconds and use the standard ``SPLIT_NS`` formatting
shared with the other ``PN()`` task-debug fields.

``wait_ewma`` is the smoothed wake-to-run wait; ``last_shift`` is the
value most recently subtracted from the task's vruntime, after the
audio cap and the optional top-app boost.  Both stay at zero on the
priming paths so a task that has never been pushed by Hikari shows
zeroes rather than stale state.

Future integration points
=========================

Two remaining Hikari/Zenith hooks discussed during design but
deferred from this patch:

* **Per-rq EWMA aggregate as a zenith input.**  Sum or max of Hikari
  per-task EWMAs on a runqueue, fed to zenith alongside util.
  Requires new ``cfs_rq`` accounting; H1+ material.

* **Zenith-supplied effective latency target.**  Replace Hikari's
  hard ``sysctl_sched_latency`` cap with ``zenith_effective_latency
  (cpu)`` so the shift cap tracks zenith's hispeed / brutal / peak-
  headroom pacing.  Architecturally the cleanest hook, but the
  tightest coupling.  Defer until Patch H1-1 and H1-2 have
  measurable wins.
