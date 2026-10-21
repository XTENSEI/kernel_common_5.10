.. SPDX-License-Identifier: GPL-2.0

===========================
The Zenith CPUFreq Governor
===========================

:Author: ShadowBytePrjkt
:Maintainer: XTENSEI (V2/V3 round-U, exclusively for android12-5.10 kernels)

``zenith`` is a custom hybrid CPUFreq governor that combines techniques
from ``schedutil`` (EAS proportional scaling), ``ondemand`` (hard-
threshold snap to maximum), ``schedhorizon`` (efficient-frequency delay
ladder) and the legacy interactive governor (hispeed bias / climb-mode),
and adds an explicit input-event boost path on top.  It is built for
interactive-first Android UI on modern ARM flagship SoCs.

It lives in ``kernel/sched/cpufreq_zenith.c`` and is selected with
``CONFIG_CPU_FREQ_GOV_ZENITH=y``.  To make it the default at boot, set
``CONFIG_CPU_FREQ_DEFAULT_GOV_ZENITH=y`` in your defconfig.

A companion userspace integration script lives in
``zenith-docs/init.zenith.rc``; the broader phone-class kernel stack
(BORE, KSM, ADIOS, CAKE) is documented in
``zenith-docs/zenith-history.rst``.

Decision chain
==============

On every ``cpufreq_governor_update_util()`` callback, ``zenith`` walks
the following chain in ``zenith_get_next_freq()``::

    0. Input Boost          → pin to policy->max for input_boost_ms
                              after any EV_KEY/EV_ABS/EV_REL event
                              (with optional cubic-eased decay)
    1. Ondemand Brutality   → pin to policy->max once load crosses
                              up_threshold (or up_threshold_hispeed when
                              already above hispeed_freq), stay pinned
                              until load drops below down_threshold
                              (hysteresis, with optional entry streak)
    2. Hispeed Ladder       → if hispeed_freq is set and load crosses
                              hispeed_load, climb toward hispeed_freq
                              using climb_mode (snap or step, with
                              entry streak + exit hysteresis)
    3. EAS Proportional     → map_util_freq(util, max_freq, max_cap)
                              with optional capacity headroom; util
                              source picked by util_math_v2 and
                              optionally blended with the kcpustat
                              hispeed signal
    4. Render / Audio /     → per-tier floors and caps when the
       Camera / Game /        corresponding awareness flag is set,
       PSI / Frame Pace /     plus the Stage 4 soft-floor tiers
       Stage 4 floors         (peer_ramp, migration_floor,
                              psi_cpu_floor, frame_overrun) which
                              fire on a deadline rather than a
                              live signal
    5. Powersave Bias       → subtract powersave_bias / 1000 from freq,
                              but only when load < bias_load_threshold
    6. uclamp_min Floor     → ADPF / cgroup uclamp_min raised to floor
    7. Cache Resolve        → cpufreq_driver_resolve_freq()
    8. Efficient Cap        → hold at efficient_freq until the request
                              has been sustained for up_delay_us;
                              up to ZENITH_EFF_BINS_MAX bins with
                              individual delays and exit hysteresis
    9. Light-Load Cap       → hard-clamp to light_load_freq when load
                              is below light_load_threshold
   10. EM / Thermal Cap     → drop the top turbo bin when thermal_state
                              is set, derated by thermal_util_derate
   11. Sampling Down        → extend down_rate_limit by
                              sampling_down_factor while at max
                              (with optional boost_exit_extend)

The input-boost and brutality paths ``goto resolve``, skipping steps
2-6; all paths pass through the remaining steps.

Tracepoints
===========

``zenith`` ships its own trace event header at
``include/trace/events/cpufreq_zenith.h``.  All events are gated on the
runtime tracing-enabled state of their corresponding tracepoint, so the
fast-path cost is one ``static_branch`` test per event::

    zenith_decision         decision-tier breadcrumb (path, freq, load)
    zenith_auto_tune        per-window classifier sample
    zenith_auto_tune_v2     v2 state-machine transitions
    zenith_auto_tune_scenario  scenario classification (game/audio/...)
    zenith_thermal_derate   thermal-pressure-derived util scaling
    zenith_predict          two-tap smoothed predicted util
    zenith_render_floor     render-tier floor application
    zenith_frame_pace       frame-pace floor application
    zenith_audio_band       audio-tier floor / cap application
    zenith_camera_floor     camera-tier floor application
    zenith_game_mode        game-mode tier transitions
    zenith_input_boost      input-boost path entry / decay

The Stage 4 soft-floor tiers (peer_ramp, migration_floor,
psi_cpu_floor, frame_overrun) ride the existing ``zenith_decision``
event via its ``path`` field; bucket counters are exposed through
``zenith_stats`` and the per-policy ``last_decision_path`` sysfs
knob.

Tunables
========

All tunables are exposed under
``/sys/devices/system/cpu/cpufreq/zenith/``.  Writes are range-checked
and return ``-EINVAL`` on out-of-range input.  Defaults below are the
kernel-side defaults; the ``profile`` selector overrides several of
them at once when written.

Rate limiting
-------------

``up_rate_limit_us``
    Minimum time (µs) between two consecutive upward frequency changes.
    Default 500.  Lower = more responsive, more DVFS churn.

``down_rate_limit_us``
    Minimum time (µs) between two consecutive downward frequency
    changes.  Default 2000.  Higher = sticks at elevated frequencies
    longer, lower battery.

``sampling_down_factor``
    While ``target_freq`` is at ``policy->max``, the effective
    ``down_rate_limit_us`` is multiplied by this factor.  Stops
    ping-pong between max and the bin just below max under sustained
    heavy load.  Default 1 (disabled), range 1..10.

``rate_limit_cluster_scale``
    Boolean (0/1, default 0).  When 1, the little cluster gets tighter
    rate limits than the big cluster (rate divided by an internal
    cluster-scale factor) so it tracks short bursts more aggressively
    without affecting the big cluster's DVFS budget.

``down_rate_adaptive``
    Boolean (0/1, default 0).  When 1, recent load variance widens the
    effective ``down_rate_limit_us`` by up to 2× on bursty workloads,
    preserving freq between bursts so the next burst doesn't pay a
    DVFS-ramp penalty.

``freq_stability_margin_pct``
    Holds the current frequency when the new target falls within this
    percent of ``policy->max`` *below* current.  Prevents tiny single-
    bin oscillations at OPP boundaries.  Default 0 (disabled), range
    0..10.

Load thresholds
---------------

``up_threshold``
    Load (in % of CPU capacity) at or above which ``zenith`` jumps
    straight to ``policy->max`` (ondemand-style).  Default 80, range
    1..100.

``up_threshold_adaptive``
    Variance-adaptive shaping: lowers ``dynamic_up_thresh`` by up to
    this percent of its value when the recent load signal is bursty.
    Default 0 (disabled), range 0..ZENITH_UP_THRESHOLD_ADAPTIVE_MAX.

``up_threshold_hispeed``
    Optional second up-threshold that applies only after the policy is
    already above ``hispeed_freq``.  Used to make the climb above
    hispeed require a stronger sustained load than the climb to
    hispeed itself.  0 = disabled (default).  Range 0..100.

``down_threshold``
    Load (%) at or below which the brutality lock is released, letting
    EAS proportional scaling take over again.  Creates a hysteresis
    band between ``down_threshold`` and ``up_threshold``.  Default 60,
    range 0..100.  Set equal to ``up_threshold`` to disable hysteresis.

``down_threshold_adaptive``
    Variance-adaptive shaping for the brutality exit threshold.
    Default 0 (disabled), range 0..ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX.

Hispeed ladder
--------------

``hispeed_freq``
    Target frequency (kHz) the governor should aim for when load
    crosses ``hispeed_load``.  0 = disabled (default), in which case
    ``hispeed_freq_pct`` is consulted next.

``hispeed_freq_pct``
    Per-cluster auto-default for ``hispeed_freq`` when the absolute
    knob is 0: effective hispeed = ``policy->max * hispeed_freq_pct /
    100``.  Default 55.  Set both ``hispeed_freq`` and
    ``hispeed_freq_pct`` to 0 to disable the tier outright.

``hispeed_load``
    Load (%) at or above which the hispeed ladder activates.
    Default 65, range 1..100.

``hispeed_hyst_pct``
    Exit-side hysteresis margin (% of max_cap) on ``hispeed_load``.
    Entering still triggers at ``load_pct >= hispeed_load``; leaving
    requires ``load_pct < (hispeed_load - hispeed_hyst_pct)``.
    Default 10.

``hispeed_entry_streak``
    Symmetric entry-side hysteresis: require ``hispeed_load`` to hold
    for N+1 consecutive samples before activating.  Default 0 (off,
    legacy single-sample entry), range 0..ZENITH_HISPEED_ENTRY_STREAK_MAX.

``brutal_decay_ms``
    Tail-glide window in milliseconds (0..500, default 0).  When 0,
    the brutal-hold cliff exit is the legacy hard cliff: the moment
    ``load_pct`` drops below the (possibly adaptive-shaped) effective
    down threshold, ``brutal_active`` is cleared and the next sample's
    ``freq`` is whatever the EAS proportional math returns.  When
    non-zero, the cliff exit instead arms a linear glide: ``policy->max``
    at arm time, decaying toward the EAS-computed ``freq`` over
    ``brutal_decay_ms``.  Eliminates the audible / visible drop that
    the legacy cliff produces on bursty workloads.  The glide self-
    disarms once the deadline passes; while disarmed (the common case)
    the post-EAS check is a single zero-test branch.

``brutal_entry_streak``
    Same idea for the brutality tier (snap-to-max).  Only gates
    ``climb_mode = SNAP``; STEP mode is unaffected.  Default 0 (off).

``climb_mode``
    How ``zenith`` reaches the brutality target.  0 = SNAP (jump
    directly to ``policy->max`` on the next tick).  1 = STEP (climb
    one ``freq_step_pct``-sized step per tick, bypasses the brutality
    hysteresis).  Default 0.

``freq_step_pct``
    Size of each STEP-mode climb step expressed as a percent of
    ``policy->max``.  Default 5, range 1..100.

``freq_step_adaptive``
    Boolean (0/1, default 0).  When 1, the STEP step scales 1.0×..2.0×
    with the overshoot of load above ``up_threshold``.

Profile selector
----------------

``profile``
    Convenience knob that writes a coherent set of tunables in one go.
    Read returns the active profile name; write accepts one of:
    ``performance``, ``balanced``, ``battery``, ``legacy``, ``custom``.
    Writing any non-``custom`` profile sets the active profile and
    overwrites the dependent tunables; writing ``custom`` does *not*
    revert anything, it just marks the active profile as custom.
    Default is ``custom`` so individual sysfs writes are remembered.

``profile_values`` (read-only)
    Dump of the table-driven profile defaults.  Shows what each named
    profile would write, so userspace can preview a switch without
    committing it.

Auto-tune classifier (V1)
-------------------------

``zenith`` can passively classify the recent load profile (idle / light
/ busy / saturated) by sampling once every 10 s and counting how many
of the last N samples crossed thresholds.  When ``auto_tune = 1``, the
governor periodically rebalances ``up_threshold`` / ``down_threshold``
between two preset bands depending on whether recent input event rate
indicates the screen is being driven (UI work) or not (background).

``auto_tune``
    Enable / disable the V1 classifier worker.  0 (default) = off,
    1 = on.  Toggling on resets the sample counters.

``auto_tune_sat_load_pct``
    A 10 s sample is counted as "saturated" if average load reached
    this percentage.  Default 70, range 0..100.

``auto_tune_hi_sat_pct``
    Saturation rate (samples_saturated / samples_total, in percent) at
    or above which the classifier picks the *hi* band.  Default 60.

``auto_tune_lo_sat_pct``
    Saturation rate at or below which the classifier picks the *lo*
    band.  Default 10.  Between the two it stays in the previous band.

``auto_tune_hi_events_x2``
    Input event rate (events per 2 s, since the worker runs every
    10 s) that the classifier considers "screen actively driven".
    Default 4 (≈ 2 events/s).

``auto_tune_lo_events_x2``
    Input event rate at or below which the classifier considers the
    screen idle.  Default 1.

``auto_tune_scenario``
    Boolean (0/1, default 1).  When 1, the classifier's profile
    selection is biased by scenario flags (game / audio / camera /
    render) lifted from the awareness tiers.

    Default flipped from 0 to 1 in the wave-1 auto-defaults round so
    V1-only builds (``auto_tune_v2=0``) also benefit from
    scenario-aware profile selection out of the box.  Floor / cap
    behaviour stays off because ``audio_floor_pct``,
    ``render_floor_pct`` and ``camera_floor_pct`` still default to 0
    -- only the profile-bias path is enabled.

Auto-tune state machine (V2)
----------------------------

V2 is a state machine layered on top of V1 that classifies into five
states — ``efficiency``, ``balanced``, ``latency``, ``sustained_perf``,
``thermal_recovery`` — and adjusts up to five knobs within per-profile
guardrails.

``auto_tune_v2``
    Boolean (0/1, default 1).  Enables the V2 state machine.  V1 must
    also be enabled (``auto_tune=1``, default 1) for V2 to run.

    Default flipped from 0 to 1 in the wave-1 auto-defaults round.
    All V2 protections remain in place: state changes require
    ``auto_tune_hysteresis_windows`` consecutive 10 s windows to
    agree before committing, ``auto_tune_cooldown_windows`` enforces
    a quiet period after each change, and ``override_mask`` keeps
    user-pinned knobs out of V2's reach.  Set the knob back to 0 in
    ``init.zenith.rc`` to lock the legacy classifier path.

``auto_tune_hysteresis_windows``
    Require this many consecutive 10 s windows to agree before
    committing a state change.  Default 2, range 0..N.

``auto_tune_cooldown_windows``
    Quiet period (in windows) after a state change before another
    change is allowed.  ``thermal_recovery`` and ``sustained_perf``
    bypass this.  Default 1.

``auto_tune_v3``
    Self-calibrating layer on top of V2.  Three accepted values:

      * ``0`` -- off.  No telemetry, no calibration.
      * ``1`` -- observe-only.  Once per ``auto_tune_v3_interval_ms``,
        zenith walks the per-policy ``at_log`` ring, counts V2 state
        transitions, and exposes the result via ``auto_tune_v3_state``.
        No knobs are adjusted.
      * ``2`` -- apply (**default since wave 7**).  Same telemetry as
        observe-only, plus a bounded signed nudge to
        ``auto_tune_hysteresis_windows`` and
        ``auto_tune_cooldown_windows``: when V2 was observed
        thrashing (transitions exceed an internal high-water mark),
        the offsets bump up by one (more hysteresis, slower
        reaction); when V2 was observed sticky (transitions below
        an internal low-water mark), the offsets bump down by one
        (less hysteresis, faster reaction).  Offsets are clamped to
        ``[-1, +4]`` and the resulting effective values are clamped
        to the existing window-count caps and to a ``>=1`` floor.

    The user-set ``auto_tune_hysteresis_windows`` /
    ``auto_tune_cooldown_windows`` scalars are unchanged; V3 nudges
    only the *effective* window count read by the V2 worker.  Switch
    back to ``auto_tune_v3 = 0`` to clear the offsets and revert to
    the user values verbatim.

    Gated by a static branch so the calibration tail is a single
    never-taken jump per V1 window when the feature is off.

``auto_tune_v3_interval_ms``
    Calibration period (ms).  Default 60000 (1 minute).  Clamped on
    store to ``[10000, 600000]``.

``auto_tune_v3_state``
    Read-only.  One line per online policy showing the most recent
    calibration window's transition count and the live signed
    offsets:

    .. code-block::

        policy0: transitions=4 hyst_offset=+1 cool_offset=+1
        policy4: transitions=2 hyst_offset=0 cool_offset=0
        policy7: transitions=0 hyst_offset=-1 cool_offset=-1

    Useful as a dry-run inspection knob in observe-only mode
    (``auto_tune_v3 = 1``) before opting in to apply mode.

``auto_tune_cluster_aware``
    Boolean (0/1, default 0).  When 1, V2 actions are anchored on the
    first CPU of each policy and applied per cluster, so big and
    little can be in different states simultaneously.

``auto_tune_v2_signals``
    Bitmask of signals V2 may consult.  See ``ZENITH_AT_FLAG_*``
    in the source.  Default ``0`` (legacy: classifier only).

``auto_tune_v2_glides``
    Boolean (0/1, default **1**).  Master gate for V2-driven
    population of the round-U-z10 glide / coordination knobs:

      * ``brutal_decay_ms``
      * ``wakeup_boost_ms``
      * ``boot_boost_decay_ms``
      * ``screen_off_glide_ms``
      * ``thermal_pressure_continuous``
      * ``frame_budget_us_auto``

    All seven default to 0 (off / legacy hard cliff).  With this
    knob at 1 (the default), the V2 worker maps the current state
    + signal flags to per-policy effective values for these knobs
    so consumers do not need to hand-tune any of them via sysfs.

    Per-knob writes from userspace continue to win outright: any
    non-zero value written to one of the seven knobs takes
    precedence over the V2-derived value.  The V2 fall-through
    only applies when the user value is 0 (the default).

    Set ``auto_tune_v2_glides`` to 0 to lock all seven knobs back
    to byte-identical legacy behaviour.

    State -> value map (see ``zenith_at_apply_glides`` in the
    source):

      * ``latency``: ``brutal_decay_ms = 150``,
        ``wakeup_boost_ms = 50``
      * ``thermal_recovery`` *or* thermal pressure >= 25%:
        ``thermal_pressure_continuous = 1``
      * any state with ``ZENITH_AT_FLAG_FRAME`` or game_mode and a
        non-zero ``zenith_drm_vblank_us`` cache:
        ``frame_budget_us_auto = 1``,
        ``wakeup_boost_ms = 50``
      * always (state-independent, arm-time / one-shot knobs):
        ``boot_boost_decay_ms = 5000``,
        ``screen_off_glide_ms = 300``

    Ignored unless ``auto_tune_v2`` is also 1; on V1-only or
    auto-tune-off systems the knob is read but no effective values
    are populated.

``auto_tune_v2_tiers``
    Boolean (0/1, default **1**).  Master gate for V2-driven arming
    of the Stage-4 K1 / K2 / K3 floor tiers:

      * ``migration_jump_pct`` / ``migration_floor_window_ms`` /
        ``migration_floor_pct`` (K1, migration-arrival floor)
      * ``psi_cpu_floor_thresh`` (K2, sustained PSI-CPU floor)
      * ``frame_overrun_slack_us`` /
        ``frame_overrun_window_ms`` /
        ``frame_overrun_floor_pct`` (K3, frame-overrun rescue)

    Different shape from ``auto_tune_v2_glides``: where glides
    populates effective values for legacy soft-glide knobs, this
    one *arms* or *disarms* whole floor tiers.  When V2 disarms a
    tier, the per-knob accessor ``zenith_tier_value()`` returns 0
    for that tier's tunables regardless of the profile-set value
    -- the floor effectively stops firing for that V2 window.

    State -> arm map (see ``zenith_at_apply_tiers`` in the source):

      * ``latency``: arm K1 + K2
      * ``ZENITH_AT_FLAG_FRAME`` or ``game_mode``: arm K1 + K3
      * ``efficiency`` / ``balanced``: disarm all three
      * ``thermal_recovery``: disarm all three (V2 actions already
        pull freq down via ``thermal_pressure_continuous``;
        layering K1/K2/K3 floors on top would fight that)
      * ``sustained_perf``: disarm all three (V2 actions already
        pin freq high; layering floors on top is redundant)

    Per-knob writes from userspace continue to win outright: any
    sysfs write to one of the seven K1/K2/K3 knobs sets the
    matching ``ZENITH_AT_OVERRIDE_*`` bit and locks V2 out of that
    knob until the next ``profile`` write (which clears the entire
    override mask).

    Set ``auto_tune_v2_tiers`` to 0 to lock all three tiers back to
    pure profile-driven behaviour without disabling V2 for the
    seven legacy glide knobs above.  When this knob is 0, the
    K1/K2/K3 read sites return the tunable verbatim,
    byte-identical to the pre-Patch-L code path.

    The detector / producer paths (the per-CPU migration-jump
    detector and the exported ``zenith_drm_vblank_event()`` hook)
    are *not* gated by this knob -- only the consumer side at the
    floor read.  This means cross-cluster state disagreements (one
    cluster in ``efficiency``, another in ``frame``) resolve
    correctly: each cluster sees its own tier mask.

    Ignored unless ``auto_tune_v2`` is also 1.

``auto_tune_thermal_slope``
    Boolean (0/1, default 0).  Enables the thermal-slope detector;
    when thermal pressure is rising fast, V2 jumps directly to
    ``thermal_recovery`` without waiting for hysteresis.

``auto_tune_thermal_pressure_pct``
    Threshold (%) of ``arch_scale_thermal_pressure()`` at which the
    thermal-slope detector arms.  Default 30.

``auto_tune_thermal_slope_pct``
    Rate-of-change (% per window) of thermal pressure that triggers
    early ``thermal_recovery`` entry.  Default 15.

``auto_tune_frame_pacing``
    Boolean (0/1, default 0).  When 1 and ``frame_budget_us`` is set,
    sustained frame-budget undershoot biases V2 toward ``latency``.

``auto_tune_sustained_gaming``
    Boolean (0/1, default 0).  When 1 and ``game_mode`` is non-zero
    for an extended window, V2 prefers the ``sustained_perf`` state.

``auto_tune_status`` (read-only)
    Diagnostic dump.  Shows current V2 state, pending state,
    confidence-window count, cooldown remaining, last reason, sample
    counters, signal flags, override mask, and active profile name.

``auto_tune_reset_overrides`` (write-only, write any non-zero)
    Clears the per-tunable user-override bitmask, allowing V2 to
    rewrite knobs the operator previously touched.

Efficient-frequency ladder
--------------------------

``efficient_freq``
    Upper soft cap (kHz).  Requests above this frequency are held at
    ``efficient_freq`` until the request has been sustained for
    ``up_delay_us``, only then allowed through.  0 = disabled
    (default).  Up to ``ZENITH_EFF_BINS_MAX`` (8) bins are supported
    via internal arrays ``eff_freq[]`` / ``eff_delay_us[]``; the
    sysfs knob writes the first bin only.

``up_delay_us``
    Time (µs) a ``>efficient_freq`` request must be sustained before
    the cap lifts.  Default 4000.  Range 0..1000000.

``eff_bin_hyst_pct``
    Exit-side hysteresis margin between adjacent efficient bins, as a
    percent of the higher bin's frequency.  Default 0 (no hysteresis,
    every tick re-evaluates from scratch).

Light-load cap
--------------

``light_load_freq``
    Hard upper cap (kHz) when current util is below
    ``light_load_threshold``.  Saves power on idle-ish workloads.
    0 = disabled (default).

``light_load_threshold``
    Load (%) below which the light-load cap applies.  Default 20,
    range 0..100.

Power-save bias
---------------

``powersave_bias``
    Fraction (in units of 1/1000) to subtract from the proportional
    scaling target after step 3.  Default 0.  Range 0..1000.

``bias_load_threshold``
    Load (%) below which ``powersave_bias`` is applied.  Above this
    threshold the bias is skipped so heavy work is not penalised.
    Default 50, range 0..100.  Set to 100 to always apply (legacy
    behaviour), or to 0 to never apply without touching
    ``powersave_bias``.

Input boost
-----------

``input_boost_ms``
    Duration (ms) to pin ``policy->max`` after any EV_KEY / EV_ABS /
    EV_REL event from the input subsystem.  Gated on ``screen_state``
    so screen-off never boosts.  Default 80, range 0..1000.

``input_boost_decay_ms``
    Length (ms) of the post-pin decay window during which the freq
    floor ramps from the boost ceiling back down to the natural eval.
    0 disables the decay (cliff exit).

``input_boost_decay_curve``
    Boolean (0/1, default 0).  When 1, the decay floor follows a
    cubic ease-in curve instead of linear.  Smoother visually,
    slightly more sustained boost.

``input_boost_big_only``
    Boolean (0/1, default 0).  When 1, only big / prime clusters
    receive the input boost; the little cluster eval path is
    untouched.

``input_boost_cap_pct``
    Cap on the boost ceiling expressed as percent of ``policy->max``.
    Default 100 (no cap).  Set lower to convert the input boost into
    a softer hispeed pin.

``boot_boost_decay_ms``
    Trailing decay window for ``boot_boost``, in milliseconds
    (0..30000, default 0).  When 0, the boot-boost ends as a hard
    cliff at ``boot_boost_ms``.  When non-zero, after
    ``boot_boost_ms`` expires the governor applies a linear floor
    that ramps from ``policy->max`` down to ``policy->min`` over
    ``boot_boost_decay_ms``, mirroring ``input_boost_decay_ms``'s
    tail behaviour for the boot pin.  Useful when boot animations
    or surface-flinger initial paints land just past
    ``boot_boost_ms`` and would otherwise see the sudden drop.

``boot_boost_ms``
    One-shot pin to ``policy->max`` for this many ms after boot,
    decays past the deadline.  Gated by ``screen_state`` and
    ``input_boost_big_only``.  0 = disabled, recommended 30000
    (30 s).

``boot_complete``
    Boolean (0/1).  Boot-completion latch shared globally across all
    policies (any policy's node is equivalent).  Defaults to 0 at
    boot; raised to 1 by either:

    - userspace writing ``1`` to the knob.  Typical use is from
      ``on property:sys.boot_completed=1`` in ``init.zenith.rc``.
    - the in-kernel auto-tune worker observing
      ``ZENITH_BOOT_COMPLETE_CALM_WINDOWS`` consecutive committed
      ``efficiency`` windows past a 5 s grace period (gated on
      ``boot_complete_auto``).

    When raised, the boost path in ``zenith_get_next_freq()`` snaps
    the deadline forward to the latch timestamp -- the cluster
    therefore transitions from Phase 1 (pin to max) to Phase 2
    (decay via ``boot_boost_decay_ms``) immediately, instead of
    cliff-cutting to load-derived frequency the way
    ``write boot_boost_ms 0`` would.

    Write ``0`` to lower the latch (useful for testing the boost
    path on a running system without a reboot).  Values ``>1`` are
    rejected with ``EINVAL``.

``boot_complete_auto``
    Boolean (0/1, default 1).  Gates the in-kernel calm-detect arm
    of the ``boot_complete`` latch.  When 1, the auto-tune worker
    increments a per-policy consecutive-``efficiency`` counter and
    raises the global latch on the first policy that hits
    ``ZENITH_BOOT_COMPLETE_CALM_WINDOWS`` past the grace period.
    When 0, only userspace writes to ``boot_complete`` can raise
    the latch.

    The calm streak counter is per-policy; if any policy
    classifies non-``efficiency`` for a window, that policy's
    counter resets but the others keep counting.  The grace period
    (currently 5 s) prevents the latch from being raised during
    very early boot when the first auto-tune window can land on
    ``efficiency`` for trivial reasons (sample-noise classifier
    bounce before the first heavy workload kicks in).

I/O wait handling
-----------------

``io_is_busy``
    Boolean (0/1, default 1).  When 1, ``SCHED_CPUFREQ_IOWAIT`` flags
    from the scheduler boost the frequency target.  When 0, IO-bound
    workloads get no boost.

``iowait_boost_min``
    Minimum util (in permille of max capacity, range 0..1000) the
    iowait boost may force.  Default 125 (12.5 %).  0 disables the
    floor and lets iowait boost ramp from the natural util signal.

``iowait_stack_pct``
    Percent (0..100) of the iowait-derived boost that is stacked on
    top of util when both are present.  Default 50.  0 reproduces
    the legacy ``max(util, boost)`` semantics.

``iowait_backoff_after_ms``
    When a single iowait episode has been live for this many ms, the
    doubling climb in ``zenith_iowait_boost()`` flips to a halving
    decay.  0 disables the backoff (legacy unbounded climb).

``ignore_nice_load``
    Boolean (0/1, default 0).  When 1, time spent on tasks with nice
    > 0 is excluded from the load calculation.  Useful when
    background tasks are intentionally low-priority and should not
    influence DVFS.

Environment hooks
-----------------

``screen_state``
    Userspace / notifier-driven flag indicating display state.  1 = on
    (default), 0 = off.  When 0, ``up_threshold`` is internally raised
    to 95 and ``powersave_bias`` is internally forced to 500 (50 %),
    so the governor becomes a very sleepy ondemand.  Also disables
    input boost.

``screen_off_glide_ms``
    Soft-glide window for the 1 -> 0 transition on ``screen_state``,
    in milliseconds (0..2000, default 0).  When 0, the screen-off
    cliff is the legacy hard step: ``dynamic_up_thresh`` snaps to 95
    and ``dynamic_bias`` snaps to 500 (the 50%% powersave penalty)
    on the very next sample after ``screen_state`` flips to 0.  When
    non-zero, both quantities ramp linearly from their natural values
    (``up_threshold`` / ``powersave_bias``) to the cliff targets
    across the configured window.  Eliminates the cliff that
    otherwise lands the moment AOD / panel-blank handlers stamp
    ``screen_state=0`` while userspace work is still winding down.

``screen_auto``
    Boolean (0/1, default 1).  When 1, ``zenith`` listens to the fb /
    drm-panel notifier chain and automatically updates
    ``screen_state`` on display blank/unblank.  When 0,
    ``screen_state`` is purely userspace-driven.

    The notifier source is selected at boot in this order:

    1. ``CONFIG_DRM_PANEL_NOTIFY=y`` (vendor builds with a
       ``drm_panel_notifier_register()`` API): preferred whenever
       registration succeeds.  ``dmesg`` shows
       ``screen_auto wired through drm panel notifier``.
    2. ``CONFIG_FB_NOTIFY=y``: fallback when drm is unavailable or
       fails registration.  Common on legacy / 5.10-stable trees.
    3. Neither: ``screen_state`` is bookkeeping-only and only
       userspace can flip it.  ``dmesg`` says so.

    Stock GKI 5.10 does not ship a drm panel notifier; vendor builds
    that backport one (Qualcomm / MediaTek / Samsung have their own)
    must define ``CONFIG_DRM_PANEL_NOTIFY=y`` and provide the
    matching ``<drm/drm_panel_notifier.h>`` header to opt in.

``thermal_pressure_continuous``
    Boolean (0/1, default 1).  When 0, ``zenith`` uses the legacy
    cliff: as soon as ``zenith_thermal_active()`` becomes true,
    ``dynamic_up_thresh`` snaps to 90 % regardless of the actual
    pressure level.  When 1, ``dynamic_up_thresh`` ramps linearly
    from the policy's ``up_threshold`` (at 0 % pressure) to 90 % (at
    100 % pressure), using the same
    ``arch_scale_thermal_pressure()``-derived percentage that the
    auto-tune V2 classifier consumes.  Smooths long-session thermal
    throttling: removes the audible / visible step that would
    otherwise occur the moment ``thermal_state`` flips on after a
    long burst.  The ramp end-points are intentionally hard-coded to
    match the cliff target so the new continuous mode never asks
    for a higher frequency than the legacy cliff would have allowed
    at peak pressure.

    Default flipped from 0 to 1 in the wave-2 auto-defaults round.
    The runtime path is gated on ``thermal_auto`` being on (default
    1) and a non-zero ``arch_scale_thermal_pressure()`` reading, so
    on cool builds the new default is a no-op.  Set to 0 in
    ``init.zenith.rc`` to lock the legacy hard-cliff path.

``prefer_silver_aware``
    **REMOVED** in current builds: ``CONFIG_SCHED_PREFER_SILVER``
    Kconfig was dropped and the sysfs knob removed; the struct
    field remains as dead storage only.  Kept for history.

``prefer_silver_hot_threshold_pct``
    REMOVED (see ``prefer_silver_aware``).

``prefer_silver_hot_bump_pct``
    REMOVED (see ``prefer_silver_aware``).

``thermal_state``
    Userspace / notifier-driven flag indicating thermal pressure.
    1 = hot, 0 = cool (default).  When 1, ``up_threshold`` is
    internally raised to 90 and the energy-model cap drops the top
    turbo bin.

``thermal_auto``
    Boolean (0/1, default 0).  When 1, ``zenith`` listens to thermal
    framework notifications and toggles ``thermal_state``
    automatically when the policy enters / exits a thermal-pressure
    band.

``thermal_util_derate``
    Util-side derating applied when ``thermal_state`` is set, in %
    (0..100).  Default 0 (no extra derating).  Independent of the
    EM-cap step.

``thermal_derate_rate_pct``
    Rate-of-change (% per tick) on the thermal-derate ramp; soft
    derivative term that smooths the binary thermal_state cliff.
    Default 0 (off).

``wakeup_boost``
    Boolean (0/1, default 0).  Brief bypass of ``up_rate_limit_us``
    on idle-to-busy transitions, so the first tick after a long idle
    can climb without the rate-limit clamp.

``wakeup_boost_ms``
    Wall-clock duration of the wakeup-boost bypass, in milliseconds
    (0..200, default 0).  When 0, the legacy tick-based bypass is the
    only mechanism: the next ``ZENITH_WAKEUP_BOOST_TICKS`` (currently
    2) upward transitions skip ``up_rate_limit`` after the detector
    fires.  When non-zero, the detection sites additionally arm a
    per-CPU ``ktime_get_ns()``-based deadline; the up-rate bypass
    stays active until both the tick counter has expired AND the
    deadline has lapsed.  Useful when ``up_rate_limit`` is small
    enough that two ticks elapse in microseconds, leaving the bypass
    too short to actually escape the cold-start sample.

kcpustat hispeed blend
----------------------

These three knobs control an optional blend of kernel-cpu-stat-derived
load (``/proc/stat`` style aggregation across user / sys / iowait /
softirq) into the governor's main util signal.  Useful when the
scheduler's PELT signal is artificially low because of CFS bandwidth
control, but kernel-side activity (network softirq, IRQ load) is high.

``kcpustat_hispeed_enable``
    Boolean (0/1, default 0).  When 1, the kcpustat blend is added on
    top of the EAS proportional util in step 3 of the decision chain.

``kcpustat_window_us``
    Sliding window length (µs) used to compute the kcpustat-derived
    load fraction.  Default 4000, range 1000..100000.  Smaller window
    = more reactive, more variance.

``kcpustat_filter_shift``
    Right-shift count applied to smooth the kcpustat-derived load
    over successive windows (``ema = ema - (ema >> shift) + new``).
    Default 1, range 0..8.  0 = no smoothing, higher = slower to
    react.

Util-math version
-----------------

``util_math_v2``
    Boolean (0/1, default 0).  Selects the util signal source for
    ``zenith_get_util()``.  0 = legacy ``cpu_util_cfs()``-based path
    (5.10 native).  1 = 6.x-style runnable-aware util that includes
    ``cpu_util_irq()`` and ``cpu_util_rt()`` contributions.  The v2
    signal is closer to mainline ``schedutil`` post-6.0.

uclamp / ADPF
-------------

``uclamp_min_respect``
    Boolean (0/1, default 1).  When 1, any task on this policy with a
    cgroup or ADPF-set ``uclamp_min`` raises a frequency floor
    proportional to that minimum.

``uclamp_max_respect``
    Boolean (0/1, default 0).  When 1, any task on this policy with a
    cgroup or ADPF-set ``uclamp_max`` caps the frequency target
    proportional to that maximum.

Predicted util
--------------

``predict_util_pct``
    Percentage (0..100, default 10) of a two-tap util predictor
    blended into the EAS proportional target.  Mitigates ramp-up
    latency on bursty workloads at the cost of a small overshoot
    risk.

    Default flipped from 0 to 10 in the wave-2 auto-defaults round.
    10 is intentionally mild -- a tenth-of-one-step lookahead -- so
    interactive bursts get a small predictive nudge without amplifying
    sample-to-sample noise.  Set to 0 in ``init.zenith.rc`` to disable
    the predictor entirely.

``predict_util_smooth``
    Boolean (0/1, default 1).  When 1, the predictor's two taps are
    additionally low-pass filtered before blending, trading peak
    accuracy for less noise.

    Default flipped from 0 to 1 in the wave-2 auto-defaults round.
    The smooth path is gated on ``predict_util_pct > 0``, so this
    default is a no-op until the predictor itself is enabled.  Pair
    with the wave-2 ``predict_util_pct`` default of 10 for a mild
    two-tap-averaged predictor.

Awareness tiers
---------------

Each awareness tier maintains an RCU-protected list of ``comm`` prefix
strings (mutable from sysfs).  When at least one task whose ``comm``
matches a prefix is runnable on the policy, the tier's floor / cap is
applied between the EAS proportional step and the post-resolve clips.

Render tier::

    render_aware       0/1, default 1 (wave-2 flip).
    render_comms       \n-separated list of comm prefixes; default
                       includes "RenderEngine", "RenderThread",
                       "surfaceflinger", "GraphicsExecutor", etc.
    render_floor_pct   Floor as percent of policy->max when matched.
                       Default 70 (unchanged).  The floor only fires
                       when one of the render comms is the cpu_curr
                       at the moment of a cpufreq decision (cached
                       for ZENITH_RENDER_CACHE_TTL_NS), so idle
                       screens see no floor; only active rendering
                       windows do.

Audio tier::

    audio_aware        0/1, default 1 (wave-2 flip).
    audio_comms        Default includes "AudioFlinger", "AudioOut_",
                       "AudioRecord", "audio.hal", etc.
    audio_floor_pct    Floor (% of max) when matched.  Default 0
                       (unchanged) -- comm walk runs but no floor is
                       applied unless the operator opts in.
    audio_cap_pct      Cap (% of max) when matched, e.g. for
                       low-latency audio threads that should not be
                       allowed to climb to max during low-load
                       intervals.  Default 0 (no cap, unchanged).

Camera tier::

    camera_aware       0/1, default 0.
    camera_active      Userspace flag mirroring an active camera
                       session.  Read-write; userspace toggles based
                       on camera HAL state.
    camera_comms       Default includes "Camera HAL", "CameraServer",
                       "ImageProcessor", etc.
    camera_floor_pct   Floor (% of max) when matched.  Default 0.

Game mode::

    game_mode          0 = off, 1 = level-1 (input-boost emphasis,
                       2 = level-2 (sustained big-cluster bias).
                       Default 0.
    game_auto          Boolean (0/1, default **1** since wave 7).
                       Master gate for the in-kernel game detector.
                       When 1, every cpufreq decision walks the
                       policy's online cpus and
                       matches cpu_curr->comm against the
                       ``game_auto_comms`` table; a 32-decision
                       streak of matches latches a global "game
                       active" state for 5 s.  While the state is
                       active, the effective ``game_mode`` is forced
                       to at least 1, so the existing level-1
                       overlays (hispeed boost + input-boost decay
                       stretch) apply automatically.  Higher
                       user-set or V2-resolved values are preserved
                       verbatim.
    game_auto_state    Read-only.  Returns 1 while the global latch
                       is in the future, 0 otherwise.  Useful for
                       tooling that wants to confirm the detector
                       fired distinct from a manual ``game_mode``
                       write.
    game_auto_comms    CSV of comm prefixes matched against
                       ``cpu_curr->comm``.  Defaults seeded with
                       ``UnityMain``, ``UnityGfxDeviceW``, ``il2cpp``,
                       ``GameThread``.  Same RCU-swap semantics as
                       ``render_comms`` / ``audio_comms`` /
                       ``camera_comms``.  Write empty string to
                       reset to the seed list.

Frame pacing
------------

``frame_budget_us``
    Frame budget (µs).  When set and ``predict_util_pct > 0``, sustained
    util below the frame-pace floor for longer than this budget
    raises a brief floor on ``policy->max`` to recover.  60 Hz =
    16667, 90 Hz = 11111, 120 Hz = 8333.  Default 0 (off).

``frame_budget_us_auto``
    Boolean (0/1, default 0).  When 1, the adaptive frame-budget
    floor (and the auto-tune V2 classifier's ``frame_active`` flag)
    use the drm-side cached vblank period instead of the
    userspace-set ``frame_budget_us`` whenever the cache is non-zero.

    The cache is populated automatically by the drm core: every call
    to ``drm_calc_timestamping_constants()`` (i.e. every panel mode
    set, including refresh-rate switches) feeds the just-computed
    ``framedur_ns`` into the exported kernel API
    ``zenith_set_drm_vblank_us(unsigned int us)``.  Any drm-based
    display path -- drm-bridge, mipi-dsi panel, vendor display HALs
    upstreaming via drm, msm_drm, mtk_drm, etc. -- gets the auto
    refresh-rate handoff for free; no per-driver hooks required.

    Drivers may also call ``zenith_set_drm_vblank_us()`` directly
    (e.g. before the timestamping constants are recomputed, for
    cases where the panel period is known earlier).  Manual writes
    win: each call simply overwrites the cache.

    Falls back to ``frame_budget_us`` silently when the cache is
    empty so existing userspace-driven tunings keep working.  The
    drm hook does not publish a 0 cache on mode disable (dotclock
    == 0) so a CRTC teardown does not clobber another CRTC's
    still-active value.

``drm_vblank_us``
    Read-only.  Reports the most recent vblank period (in
    microseconds) published by drm via
    ``zenith_set_drm_vblank_us()``.  0 means no driver has reported
    yet; useful as a sanity check when ``frame_budget_us_auto`` is
    enabled.

``frame_budget_us_per_policy``
    CSV override of ``frame_budget_us`` per policy (e.g.
    ``policy0=16667,policy4=8333``).  Empty = use the global
    ``frame_budget_us``.

``frame_pace_floor_pct``
    Floor (% of policy->max) raised when the frame-pace timer fires.
    Default 0 (off).

PSI awareness
-------------

``psi_aware``
    Boolean (0/1, default 0).  Enables the PSI-aware tier.

``psi_mem_thresh``
    PSI memory-some pressure (%) at or above which a memory-stall
    floor is applied.  Default 0 (off).

``psi_cpu_thresh``
    PSI cpu-some pressure (%) trigger.  Default 0 (off).

``psi_io_thresh``
    PSI io-some pressure (%) trigger.  Default 0 (off).

Stage 4 floor tiers
-------------------

A family of soft-floor tiers added in the Stage 4 round, layered
above the EAS proportional step but below the input-boost / brutality
fast-path.  All of these default to 0 / off on the BATTERY and LEGACY
profiles; PERFORMANCE and BALANCED arm them via per-profile defaults.
``auto_tune_v2_tiers`` (above) gates V2-driven per-state arming on
top of the per-profile baseline.

The tier read order at each K1/K2/K3 site is:

  1. user sysfs override  ->  return user-set tunable verbatim
     (matching ``ZENITH_AT_OVERRIDE_*`` bit set in
     ``tunables->auto_tune_override_mask``).
  2. ``auto_tune_v2_tiers == 0`` *or* ``auto_tune_v2 == 0``  ->
     return tunable verbatim (V2 wiring off).
  3. V2 worker not run yet (``at_local_tiers_active == false``)
     ->  return tunable verbatim.
  4. V2 disarmed this tier  ->  return 0 (off for this V2 window).
  5. V2 armed this tier  ->  return tunable (profile-set value
     comes through).

The detector / producer paths fire regardless of V2 gating; only
the consumer side at the floor read is gated.

Multi-cluster pre-arm coordination (Patch D)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Class-level pre-arm protocol between the BIG and PRIME clusters.
When one cluster lifts to peak via ``predict_up`` / ``peak_prearm``
/ ``peak_rescue``, it stamps a deadline on the peer's slot; while
the deadline holds, the peer applies a soft floor at
``peer_ramp_floor_pct`` of ``policy->max`` so cross-cluster IPC
chains don't spend their first samples stalled at idle.  LITTLE
neither writes nor reads.  Two file-scope ``atomic64_t`` slots, no
per-policy state.

``peer_ramp_window_ms``
    Length of the soft-floor window (ms).  Range 0..100.
    Default 25.  0 disables both the write side and the read side;
    a cluster never stamps a deadline and never observes one.

``peer_ramp_floor_pct``
    Soft floor (% of ``policy->max``) applied to the peer cluster
    while the deadline holds.  Range 0..100.  Default 60.

Per-profile defaults:

.. code-block:: text

                          PERF / BAL / BAT / LEG
   peer_ramp_window_ms      40    25     0     0
   peer_ramp_floor_pct      70    60     0     0

Migration-arrival soft floor (Patch K1)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Compensates for PELT migration lag.  A high-util task moving
between CPUs takes ~32 ms (one PELT half-life) to fully reflect on
the destination CPU's util signal.  This tier closes that gap.

A per-CPU detector runs on every ``update_util`` tick after
``zenith_iowait_apply()``: if the upward util jump exceeds
``migration_jump_pct`` of ``max_capacity``, stamp a deadline on the
policy's ``migration_in_until_ns``.  While the deadline holds, the
floor tier in ``zenith_get_next_freq()`` applies a soft floor at
``migration_floor_pct`` of ``policy->max``.

``migration_jump_pct``
    Detector threshold (% of ``max_capacity``).  Range 0..100.
    Default 20.

``migration_floor_window_ms``
    Soft-floor window length (ms).  Range 0..100.  Default 30.
    0 disables stamping (the detector still updates
    ``migration_prev_util`` but does not arm a deadline).

``migration_floor_pct``
    Soft floor (% of ``policy->max``) applied while the deadline
    holds.  Range 0..100.  Default 60.  0 leaves stamping in place
    but suppresses the lift; useful for stats / trace correlation.

Per-profile defaults:

.. code-block:: text

                          PERF / BAL / BAT / LEG
   migration_jump_pct       15    20     0     0
   migration_floor_window_ms 35    30    0     0
   migration_floor_pct      70    60     0     0

PSI-CPU sustained-pressure floor (Patch K2)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Mirror of the existing ``psi_cpu_thresh`` cap, pointing the other
direction.  When ``zenith_psi_cpu_some_pct()`` (the 10s EWMA of
system-wide CPU stall %) is at or above ``psi_cpu_floor_thresh``,
the eval lifts to ``zenith_eff_hispeed_freq()``.  Reuses the
existing PSI helper; gated on ``psi_aware``.

``psi_cpu_floor_thresh``
    Threshold (%) of PSI CPU-some 10s EWMA at or above which the
    floor lifts to hispeed.  Range 0..100.  Default 0 (off).

Per-profile defaults:

.. code-block:: text

                          PERF / BAL / BAT / LEG
   psi_cpu_floor_thresh     40     0     0     0

Frame-overrun rescue (Patch K3)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Companion to the existing ``frame_pace_floor`` tier.  ``frame_pace``
arms a floor sized to fit one frame in the budget; this tier
corrects after a frame already missed.

Adds the exported function:

.. code-block:: c

    void zenith_drm_vblank_event(void);

declared in ``include/linux/cpufreq_zenith.h`` (with a
``static inline`` no-op stub when ``CONFIG_CPU_FREQ_GOV_ZENITH=n``).
Display drivers / drm-panel bridges call it from the per-vblank
IRQ handler.  On every call after the first, the wall-clock delta
since the previous vblank is compared against the cached vblank
period plus ``frame_overrun_slack_us``.  If the gap is wider, stamp
a deadline ``frame_overrun_window_ms`` in the future on the
governor-wide overrun slot.  While the deadline holds, every
cluster's eval lifts to a soft floor at
``frame_overrun_floor_pct`` of ``policy->max``.

Stale guard: the per-frame timestamp is cleared on every
screen-state edge so a multi-second AOD / screen-off stretch does
not make the first event after resume look like a giant overrun.

``frame_overrun_slack_us``
    Tolerance (microseconds) beyond the cached vblank period before
    a gap is treated as an overrun.  Range 0..16667.  Default 0
    (off).  0 disables stamping (the producer still updates the
    cached timestamp).  Mirrored into a governor-wide cache on
    every sysfs store and ``zenith_apply_profile()`` pass because
    the producer has no ``struct zenith_policy`` in scope.

``frame_overrun_window_ms``
    Soft-floor window length (ms).  Range 0..200.  Default 50.

``frame_overrun_floor_pct``
    Soft floor (% of ``policy->max``) applied while the deadline
    holds.  Range 0..100.  Default 80.  0 keeps stamping but
    suppresses the lift.

Per-profile defaults:

.. code-block:: text

                          PERF / BAL / BAT / LEG
   frame_overrun_slack_us   3000  4000   0     0  (us)
   frame_overrun_window_ms    60    50   0     0  (ms)
   frame_overrun_floor_pct    90    80   0     0  (%)

**Wiring (in-tree, V4):** ``zenith_drm_vblank_event()`` is wired
from the generic drm core in ``drivers/gpu/drm/drm_vblank.c``,
inside ``drm_handle_vblank()`` (the function both legacy and
KMS vblank IRQs funnel through).  Every in-tree drm driver
inherits the producer hook for free; no per-driver wiring is
required.  Out-of-tree vendor display stacks that call into a
custom vblank-IRQ path (rather than ``drm_handle_vblank()``)
should call ``zenith_drm_vblank_event()`` from there to enable
K3.  The header stub is a ``static inline`` no-op when
``CONFIG_CPU_FREQ_GOV_ZENITH=n``, so callers can stay
unconditional.

**Wiring (vblank period publish, V3):** The companion API
``zenith_set_drm_vblank_us(unsigned int us)`` is wired from
``drm_calc_timestamping_constants()`` (every panel mode-set, every
refresh-rate switch) so the K3 deadline math has an accurate
period without explicit per-driver code.  Out-of-tree drivers
that call ``drm_calc_timestamping_constants()`` get this for
free; out-of-tree drivers that don't may publish the period
manually:

.. code-block:: c

    /* Inside the vendor mode-set / vblank-period change handler.  */
    zenith_set_drm_vblank_us(framedur_ns / NSEC_PER_USEC);

When neither hook is wired and ``frame_budget_us_auto=1``, K3 falls
back to the userspace-set ``frame_budget_us``; if THAT is also
zero, K3 stays disarmed cleanly (same fail-safe as the rest of
the auto-tune scenario overlay).  Diagnose with the read-only
``drm_vblank_us`` sysfs: a non-zero value confirms the publish
path is live; ``zenith_stats`` shows a non-zero
``frame_overrun_count`` when the producer is actually firing.

Read-only diagnostics
---------------------

``zenith_stats``
    Per-policy decision-tier counters: total ticks, brutality entries,
    hispeed entries, EAS, input-boost, light-load cap hits, EM cap
    hits.  Read once from userspace after a benchmark to attribute
    the freq-time histogram to specific tiers.

``zenith_stats_reset`` (write-only)
    Write any non-zero value to clear ``zenith_stats`` and the
    ``at_log`` ring on every policy that shares this tunables set.
    Writing ``0`` is intentionally a no-op (avoids a stray
    ``echo > zenith_stats_reset`` erasing the data the operator was
    about to read).

``at_log``
    Per-policy ring buffer of the most recent auto-tune classifier
    samples.  Each sample is one ``ZENITH_AUTO_TUNE_PERIOD_MS`` window
    (10 s by default), the depth of the ring is
    ``ZENITH_AT_LOG_NR`` (16) so each policy keeps about 160 s of
    post-mortem.  Output is one header line per policy followed by
    one line per sample (oldest first), with ``key=value`` pairs::

        ts_ns=  ktime_get_ns() at push time
        reason= V1 / V2 reason name
        from=   V2 state at the start of the window
        to=     V2 state after this window's actions
        target= V1 picked profile name
        sat=    saturation rate (% of samples > saturation_load_pct)
        evx2=   input event rate (events / 2 s)
        thp=    arch_scale_thermal_pressure() at push (0..1024)
        slope=  thermal pressure delta vs the previous window
        var=    load variance EWMA × 256
        flags=  ZENITH_AT_FLAG_* signal mask
        emerg=  1 if a V2 emergency state was committed

Typical recipes
===============

Phone-class daily driver
------------------------

Active screen, mixed workload, expecting some bursty UI.  This is the
zenith-default sweet spot::

    echo balanced > /sys/.../zenith/profile
    echo 1        > /sys/.../zenith/auto_tune
    echo 1        > /sys/.../zenith/auto_tune_v2
    echo 1        > /sys/.../zenith/screen_auto
    echo 1        > /sys/.../zenith/thermal_auto
    echo 80       > /sys/.../zenith/input_boost_ms

Flagship SoC, gaming ROM
------------------------

Preserves peak performance, adds modest light-load savings::

    echo performance > /sys/.../zenith/profile
    echo 80          > /sys/.../zenith/up_threshold
    echo 60          > /sys/.../zenith/down_threshold
    echo 4           > /sys/.../zenith/sampling_down_factor
    echo 80          > /sys/.../zenith/input_boost_ms
    echo 0           > /sys/.../zenith/auto_tune
    # no light-load cap, no efficient_freq, no bias

Mid-range SoC, battery ROM
--------------------------

Caps idle-ish load aggressively, slower ramp-up::

    echo battery > /sys/.../zenith/profile
    echo 80      > /sys/.../zenith/up_threshold
    echo 50      > /sys/.../zenith/down_threshold
    echo 1       > /sys/.../zenith/sampling_down_factor
    echo 80      > /sys/.../zenith/input_boost_ms
    echo 691200  > /sys/.../zenith/light_load_freq
    echo 30      > /sys/.../zenith/light_load_threshold
    echo 100     > /sys/.../zenith/powersave_bias
    echo 40      > /sys/.../zenith/bias_load_threshold

Ondemand-style legacy
---------------------

Reproduces the pre-hysteresis behaviour for comparison::

    echo legacy > /sys/.../zenith/profile
    echo 80     > /sys/.../zenith/up_threshold
    echo 80     > /sys/.../zenith/down_threshold      # hysteresis off
    echo 1      > /sys/.../zenith/sampling_down_factor
    echo 0      > /sys/.../zenith/input_boost_ms      # boost off
    echo 100    > /sys/.../zenith/bias_load_threshold # always bias

Hispeed-ladder example
----------------------

Force the policy to climb to a known efficient frequency before
allowing brutality, useful when the silicon has a clear voltage cliff
at one bin above ``hispeed_freq``::

    echo 1804800 > /sys/.../zenith/hispeed_freq
    echo 70      > /sys/.../zenith/hispeed_load
    echo 1       > /sys/.../zenith/climb_mode      # step instead of snap
    echo 10      > /sys/.../zenith/freq_step_pct   # 10 % per update

kcpustat blend example
----------------------

Useful when softirq / IRQ load is the bottleneck (heavy networking,
USB bulk traffic) and the scheduler's PELT util alone underestimates
real CPU pressure::

    echo 1     > /sys/.../zenith/kcpustat_hispeed_enable
    echo 4000  > /sys/.../zenith/kcpustat_window_us
    echo 1     > /sys/.../zenith/kcpustat_filter_shift

Relationship to other governors
===============================

``schedutil``
    ``zenith`` inherits the PELT-based ``schedutil_cpu_util`` signal
    and the iowait-boost mechanism.  It differs in three ways: split
    up/down rate limits, an ondemand-style hard threshold for
    snap-to-max, and an explicit input-event boost.  With
    ``util_math_v2 = 1``, the util signal is essentially 6.x
    ``schedutil``'s.

``ondemand``
    ``zenith`` reuses ``sampling_down_factor`` and the
    ``up_threshold`` snap-to-max concept.  It does **not** use
    ondemand's per-policy kworker polling; like ``schedutil`` it runs
    off the scheduler's ``update_util`` hook.

``schedhorizon``
    ``zenith`` borrows the efficient-frequency delay concept and
    extends it to a ladder of up to 8 bins with optional exit
    hysteresis.

``interactive``
    ``zenith`` reuses the hispeed-frequency / hispeed-load idea and
    the climb-mode (snap vs step) toggle, but does not borrow
    interactive's target-load array — that's replaced by the simpler
    ``hispeed_load`` + ``up_threshold_hispeed`` pair plus
    symmetric entry-streak hysteresis.
