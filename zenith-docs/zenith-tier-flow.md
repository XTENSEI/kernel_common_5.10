# zenith — per-tier flow diagram

How a single cpufreq tick flows through `zenith_get_next_freq()`.
Captures the canonical order of every tier, the direction each
applies (floor / cap / either), what gates it, and how arbitration
resolves when multiple tiers fire at once.

Generated against the post-Wave-B tree (commit `0e6752902bb1`).
~36 tiers; new tiers can be added but ordering is otherwise
load-bearing.

## Overview

```
                  cpufreq tick (every cpufreq update interval)
                                      |
                                      v
                       +-----------------------------+
                       |  zenith_update_single() /   |
                       |  zenith_update_shared()     |
                       +--------------+--------------+
                                      |
                                      v
                       +-----------------------------+
                       |  policy / cpu state pull    |
                       |  (util_avg, kcpustat, PSI,  |
                       |   iowait, dl util, cgroup,  |
                       |   render-thread cache,      |
                       |   PMU IPC, EM knee, ...)    |
                       +--------------+--------------+
                                      |
                                      v
                       +-----------------------------+
                       |  zenith_get_next_freq()     |
                       |                             |
                       |  Stage 0: PEAK HYSTERESIS   |
                       |   (early-exit if applicable)|
                       |                             |
                       |  Stage 1: BASE FREQ         |
                       |   (util -> freq map; tp =   |
                       |    "eas" by default)        |
                       |                             |
                       |  Stage 2: FLOOR PHASE       |
                       |   (28 tiers; never lower)   |
                       |                             |
                       |  Stage 3: CAP PHASE         |
                       |   (8 tiers; never raise)    |
                       |                             |
                       |  Stage 4: FINAL CLAMP       |
                       |   (policy->min/max)         |
                       +--------------+--------------+
                                      |
                                      v
                       +-----------------------------+
                       |  cpufreq_driver_target()    |
                       |  / fast_switch              |
                       +-----------------------------+
```

## Arbitration model

- **Floors race higher**: each floor tier compares the running
  `freq` against its own bound and writes back `max(freq, bound)`.
  The order of floor tiers therefore does not matter for the
  *value* of the final freq, only for which `tp_path` label is
  recorded (the *last* floor tier to win sets `tp_path`).
- **Caps race lower**: each cap tier writes back `min(freq, bound)`.
  Same `tp_path` semantics: last cap to fire owns the label.
- **Floor / cap interaction**: floor phase runs first, cap phase
  runs after. A cap can therefore lower a freq that a floor just
  raised. There is no second floor phase after the cap phase.
- **Final clamp**: `clamp(freq, policy->min, policy->max)` runs
  last. A cap can lower below `policy->min`; the final clamp
  re-raises it.

## Tier list (canonical order)

### Stage 0 — peak hysteresis (early exit)

| # | tier | tp_path | direction | gate | note |
|---|------|---------|-----------|------|------|
| 0 | peak hysteresis | `peak_hyst` | early-exit | `peak_hysteresis_streak`, peak_step_down_pct | When the policy has been at peak for a streak, snap freq down by step_down_pct in one shot. Bypasses the rest of the pipeline. |

### Stage 1 — base freq

| # | tier | tp_path | direction | gate | note |
|---|------|---------|-----------|------|------|
| 1 | EAS / schedutil-style util→freq map | `eas` | initial | always | Default starting freq. Derived from PELT util via `map_util_freq()`. Every later tier mutates this. |

### Stage 2 — FLOOR phase (28 tiers)

| # | tier | tp_path | gate | description |
|---|------|---------|------|-------------|
| 2 | input boost | `input_boost` | input event recent + `input_boost_*` | Raise on touch / wakeup. |
| 3 | climb step | `climb_step` | upward util-edge | Step-up multiplier when util is climbing. |
| 4 | snap to max | `snap_max` | extreme-up condition | Single-tick snap to `policy->max`. |
| 5 | brutal hold | `brutal_hold` | brutal entry streak | Hold at high freq for N ticks after a brutal entry. |
| 6 | predict up | `predict_up` | `predict_up_*` | Util-prediction window says we're about to need more freq. |
| 7 | PELT rising edge | `pelt_edge` | `pelt_rising_edge_*` | Coarse PELT delta exceeds threshold. |
| 8 | hispeed | `hispeed` | `hispeed_*` | Classic hispeed-floor tier (load > hispeed_load). |
| 9 | peak headroom prearm | `peak_prearm` | `peak_headroom_prearm` | Pre-arm peak-headroom one tick before rescue fires. |
| 10 | peak headroom rescue | `peak_rescue` | starve streak | Rescue starving load with a freq jump. |
| 11 | uclamp.min floor | `uclamp_min_floor` | uclamp.min on running task | Per-task uclamp.min raises floor. |
| 12 | input boost decay | `input_boost_decay` | input boost still active | Tail of the input_boost window. |
| 13 | boot boost | `boot_boost` | boot-boost active | Initial boot freq pin. |
| 14 | boot boost decay | `boot_boost_decay` | boot-boost decay window | Glide down from boot_boost. |
| 15 | frame pace | `frame_pace` | drm vblank | Pace freq to vblank cadence. |
| 16 | audio floor | `audio_floor` | `audio_aware`, audio thread on cpu | Stable freq for foreground audio. |
| 17 | charger floor | `charger_floor` | `charger_aware`, AC-attached | **Wave A**. Lower floor on AC. |
| 18 | render floor | `render_floor` | `render_aware`, RenderThread on cpu | Floor when any render thread is running. |
| 19 | render-thread util floor | `render_thread_util_floor` | `render_thread_util_aware` + matched task's util_avg ≥ thresh | **Wave A**. More selective than render_floor. |
| 20 | PMU IPC floor | `pmu_ipc_floor` | `pmu_aware` + max IPC ≥ thresh | **Wave B**. Floor when measured IPC indicates compute-bound work. |
| 21 | EM energy-knee floor | `em_floor` | `em_aware` + EM registered | **Wave B**. Floor at energy-knee OPP. |
| 22 | render RT floor | `render_rt_floor` | render thread is RT-pri | Higher floor for RT-priority render threads. |
| 23 | camera floor | `camera_floor` | camera_active | Floor while v4l2 camera is open. |
| 24 | top-app floor | `top_app_floor` | `top_app_aware` + cpuset.cgroups == top-app | **Wave A**. Floor when current is in the top-app cpuset cgroup. |
| 25 | game perf burst floor | `game_perf_burst_floor` | game_perf_burst FSM | Floor while game burst FSM is armed. |
| 26 | DL task floor | `dl_floor` | `dl_task_floor_pct` + DL task on cpu | Floor for SCHED_DEADLINE task. |
| 27 | iowait floor | `io_floor` | iowait stack + iowait_boost path | Floor for iowait-stacked CPU. |
| 28 | peer ramp | `peer_ramp` | peer-ramp window | Floor when sibling cluster recently ramped. |
| 29 | cluster wake pulse | `cluster_wake_pulse` | cluster-wake pulse | Brief pulse when cluster wakes from idle. |
| 30 | fg transition pulse | `fg_transition_pulse` | fg-transition pulse window | Brief pulse on app foreground swap. |
| 31 | migration floor | `migration_floor` | `migration_floor_*` + recent migration | Floor for N ms after task migrated to this cluster. |
| 32 | PSI CPU floor | `psi_cpu_floor` | PSI cpu pressure ≥ thresh | Floor when PSI says the cpu is pressured. |
| 33 | frame overrun floor | `frame_overrun` | frame_overrun_* + drm vblank overrun | Floor after frames overran the vblank deadline. |

### Stage 3 — CAP phase (8 tiers)

| # | tier | tp_path | gate | description |
|---|------|---------|------|-------------|
| 34 | audio cap | `audio_cap` | audio_cap_pct + audio thread | Cap to audio_cap_pct of policy->max for stable audio freq. |
| 35 | uclamp.max cap | `uclamp_max_cap` | uclamp.max on running task | Per-task uclamp.max lowers cap. |
| 36 | PSI mem cap (light) | `psi_mem_cap_light` | PSI mem ≥ thresh | Cap freq when memory pressure is high (no point burning freq when stalled on memory). |
| 37 | quiet hours cap | `quiet_hours_cap` | quiet_hours window + screen-off | Lower cap during quiet-hours window. |
| 38 | sleeper tail cap | `sleeper_tail` | sleeper-tail window | Tail-cap after a long sleep. |
| 39 | light cap | `light_cap` | classifier-driven | Light-load classifier cap. |
| 40 | EM cap | `em_cap` | em_cap_aware (existing pre-Wave-B) | Pre-existing energy-model cap (separate from Wave B em_floor). |
| 41 | auto thermal cap | `auto_thermal_cap` | thermal zone temp ≥ guardrail | Final thermal-aware cap. |

### Stage 4 — final clamp

| # | tier | tp_path | direction | description |
|---|------|---------|-----------|-------------|
| 42 | policy clamp | (kept) | clamp | `clamp(freq, policy->min, policy->max)`. Last word. |

## Decision flow diagram

```
                       Stage 0: peak hysteresis
                          if (peak_hyst_armed)
                              freq = peak_step_down(freq);
                              tp_path = "peak_hyst";
                              GOTO commit;
                                      |
                                      v
                       Stage 1: base
                          freq = map_util_freq(util,
                                  policy->max, scale_cpu);
                          tp_path = "eas";
                                      |
                                      v
                       Stage 2: FLOOR phase (each tier:
                                if (gate && bound > freq) {
                                    freq = bound;
                                    tp_path = "<tier>";
                                }
                                in canonical order #2..#33)
                                      |
                                      v
                       Stage 3: CAP phase (each tier:
                                if (gate && bound < freq) {
                                    freq = bound;
                                    tp_path = "<tier>";
                                }
                                in canonical order #34..#41)
                                      |
                                      v
                       Stage 4: final clamp
                          freq = clamp(freq, policy->min,
                                       policy->max);
                                      |
                                      v
                       trace_cpufreq_zenith_pick(freq, tp_path)
                                      |
                                      v
                       return freq;
```

## Per-tier exit reasons

A tier can:

1. **fire** — its gate says yes and it changed `freq`. `tp_path`
   is updated.
2. **fire-but-no-op** — its gate says yes but its bound did not
   improve `freq` (in the right direction). `freq` and `tp_path`
   unchanged.
3. **gate-off** — its gate says no, the body is skipped entirely.

Tracepoint `cpufreq_zenith_pick` records `tp_path` as the *last*
tier to enter case (1).

## Hot-path ordering rules

Order within each phase is documented (above) but enforced at
review time, not at compile time. Two rules of thumb:

1. **Cheaper gates first**: a tier whose gate is a single
   `READ_ONCE` (e.g. `audio_aware`) goes before one whose gate
   walks a comm list (e.g. `render_floor`). The `tp_path` label
   of the *winning* tier is the same regardless, but skipping
   the comm walk when an earlier tier already saturated is a
   minor savings.
2. **Specificity-ordered**: more selective tiers (higher
   gates, narrower preconditions) go after broader ones. e.g.
   `render_thread_util_floor` (Wave A) intentionally goes
   *after* `render_floor`: if `render_floor` already set the
   floor at e.g. 800 MHz, and util-tier wants 1200 MHz, util-
   tier wins. If util-tier wants only 600 MHz, no-op (can't
   lower).

## Gating dependency graph

```
                     +-----------------------------+
                     | render_aware (master gate)  |
                     +------+--------+-------------+
                            |        |
                +-----------+        +---------+
                v                              v
     +-------------------------+   +-------------------------+
     | render_floor (#18)      |   | render_thread_util_floor|
     | gate: render_aware && comm|  | (#19, Wave A)           |
     | walk match              |   | gate: render_aware &&    |
     +-------------------------+   | render_thread_util_aware |
                                   | && matched util ≥ thresh|
                                   +-------------------------+

                     +-----------------------------+
                     | top_app_aware (master gate) |
                     +-------------+---------------+
                                   |
                                   v
                     +-----------------------------+
                     | top_app_floor (#24, Wave A) |
                     | gate: top_app_aware && task |
                     | in cpuset top-app cgroup    |
                     +-----------------------------+

                     +-----------------------------+
                     | charger_aware (master gate) |
                     +-------------+---------------+
                                   |
                                   v
                     +-----------------------------+
                     | charger_floor (#17, Wave A) |
                     | gate: charger_aware && AC   |
                     | attached                    |
                     +-----------------------------+

                     +-----------------------------+
                     | pmu_aware (master gate)     |
                     +-------------+---------------+
                                   |
                                   v
                     +-----------------------------+
                     | pmu_ipc_floor (#20, Wave B) |
                     | gate: pmu_aware && IPC ≥    |
                     | pmu_ipc_thresh              |
                     +-----------------------------+

                     +-----------------------------+
                     | em_aware (master gate)      |
                     +-------------+---------------+
                                   |
                                   v
                     +-----------------------------+
                     | em_floor (#21, Wave B)      |
                     | gate: em_aware && EM        |
                     | registered                  |
                     +-----------------------------+
```

`render_thread_util_floor` is the only tier with a *two-level*
gate: it requires both `render_aware` (because the comm walk has
to run for util_avg to be observed) and `render_thread_util_aware`
(its own gate). Other Wave-A/B tiers have flat gates.

## Tracepoint legend

`cpufreq_zenith_pick` records `tp_path` as one of the following:

```
peak_hyst, eas, input_boost, climb_step, snap_max, brutal_hold,
predict_up, pelt_edge, hispeed, peak_prearm, peak_rescue,
uclamp_min_floor, input_boost_decay, boot_boost,
boot_boost_decay, frame_pace, audio_floor, charger_floor,
render_floor, render_thread_util_floor, pmu_ipc_floor, em_floor,
render_rt_floor, camera_floor, top_app_floor,
game_perf_burst_floor, dl_floor, io_floor, peer_ramp,
cluster_wake_pulse, fg_transition_pulse, migration_floor,
psi_cpu_floor, frame_overrun, audio_cap, uclamp_max_cap,
psi_mem_cap_light, quiet_hours_cap, sleeper_tail, light_cap,
em_cap, auto_thermal_cap
```

42 distinct labels. Use:

```bash
adb shell 'cd /sys/kernel/debug/tracing && \
    echo 1 > events/cpufreq_zenith/cpufreq_zenith_pick/enable && \
    cat trace_pipe | head -100'
```

to watch tier resolution live.

## Where to look in source

- Stage 0 (peak_hyst): around line 8731 in `cpufreq_zenith.c`.
- Stage 1 (base "eas"): around line 8997.
- Stage 2 (floor phase): lines ~9559..11100.
- Stage 3 (cap phase): lines ~11127..11547.
- Stage 4 (final clamp): a few lines after stage 3.

(Line numbers are post-Wave-B `0e6752902bb1`; will drift as more
tiers land.)
