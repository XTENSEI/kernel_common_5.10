# zenith — tunable cookbook

Inverse index. Recipes for shaping zenith's behaviour by **goal**,
not by **knob**. For each common shape, this lists the tunables to
flip, suggested values for a Cortex-A78-class big cluster, and
known side effects.

For per-tier semantics, see `zenith-tier-flow.md`.

All paths are relative to:

```
/sys/devices/system/cpu/cpufreq/policy<N>/zenith/
```

Apply at boot via `init.zenith.rc` (V1 or later) or interactively
via `echo <value> > <path>`. Values stick for the life of the
governor; on governor restart they reset to their `defaults_*`
counterparts.

---

## Recipe 1 — "max battery on idle screens"

Pin the policy hard at the bottom when the screen is off and the
device is on battery.

```
quiet_hours_screen_off_only      = 1
quiet_hours_cap_pct              = 30        # cap freq at 30% of max
quiet_hours_start_min            = 0         # 24h coverage when off
quiet_hours_end_min              = 1440      # 24h coverage when off

screen_off_glide_ms              = 800       # tail to bottom over 800 ms
batt_hold_scale_pct              = 70        # screen-off bias

charger_aware                    = 1         # opt out when charging
charger_floor_pct                = 0         # but no extra floor on AC either
```

Effect: screen-off pulls freq down to ~30% of policy max within
~1 second. On AC, the cap lifts (gated by `charger_aware`).

Side effects:
- Music / audio playback in screen-off may dip below the audio
  floor. Set `audio_aware=1 audio_floor_pct=40` to override.
- Background sync / notifications that need brief CPU bursts
  will dip-then-recover; usually fine, but slow over LTE.

---

## Recipe 2 — "max latency for gaming"

Hold every active cluster at or near peak while a game is in the
foreground.

```
game_auto                        = 1
game_perf_burst                  = 1
game_perf_burst_floor_pct        = 90        # 90% of policy max
game_auto_comms                  = "Unity\nUnreal\nUE4Game\nUE4Threadpool\nRHIThread\ngametask\neglThread\nMainThr\nEmuThread"

peak_headroom_rescue             = 1
peak_headroom_starve_streak      = 2         # rescue after 2 starve windows
peak_headroom_freq_floor_pct     = 95        # rescue floor

peer_ramp_window_ms              = 16        # 1 vblank
peer_ramp_floor_pct              = 75

migration_floor_window_ms        = 32        # 2 vblanks
migration_floor_pct              = 70

frame_overrun_window_ms          = 16
frame_overrun_floor_pct          = 80
frame_overrun_deep_streak        = 3
frame_overrun_deep_floor_pct     = 90

input_boost_*                    = (raise to taste)

# top-app cgroup also helps:
top_app_aware                    = 1
top_app_floor_pct                = 60
```

Effect: cpufreq pinned high during gameplay. UI thread, render
thread, and any game worker stay above their respective floors;
peak_headroom and frame_overrun catch the rare regression.

Side effects:
- Battery drain ~+15-25% under load.
- Skin temperature warmer; thermal throttle (`auto_thermal_cap`)
  may engage on long sessions.
- For non-game foreground apps with `MainThr` named threads
  (some Java apps), `game_auto` may falsely arm. Trim
  `game_auto_comms` to be safe.

---

## Recipe 3 — "stable freq for foreground audio"

Avoid jitter in audio rendering threads (Bluetooth audio, native
playback, recording).

```
audio_aware                      = 1
audio_floor_pct                  = 50        # 50% of policy max
audio_hyst_ms                    = 250       # 250 ms tail
audio_cap_pct                    = 75        # cap to avoid runaway

# Don't let DL_TASK_FLOOR hammer up:
dl_task_floor_pct                = 35

# Gentle quiet hours; audio overrides cap:
quiet_hours_cap_pct              = 50
```

Effect: when audio render thread is observed, freq sits in the
[50%, 75%] band. Hysteresis keeps it stable for 250 ms after the
audio thread is no longer observed.

Side effects:
- If audio thread misses a comm-walk match, the floor doesn't
  fire. Audio servers with custom comms (qcom-audio, hexagon)
  may need a manual `audio_aware_comms` extension if available.
- Setting `audio_floor_pct` too high prevents idle drop and
  burns battery.

---

## Recipe 4 — "smooth scrolling / UI rendering"

Make scroll, animation, and frame rendering subjectively snappier
without going full-perf.

```
render_aware                     = 1
render_floor_pct                 = 50

# Wave A: more selective, fires only on actual rendering:
render_thread_util_aware         = 1
render_thread_util_thresh        = 256       # 25% of CPU capacity
render_thread_util_floor_pct     = 65

# Foreground responsiveness:
top_app_aware                    = 1
top_app_floor_pct                = 50

# Pulse on app foreground swap:
fg_transition_pulse_ms           = 80
fg_transition_pulse_pct          = 70

# Vblank-paced freq:
frame_pace                       = 1

# Migration floor for cluster moves:
migration_floor_window_ms        = 32
migration_floor_pct              = 60
```

Effect: while RenderThread is doing actual work (util_avg above
25% of CPU capacity), freq sits at 65%. Foreground transitions
get a brief pulse. Top-app foreground gets a 50% baseline floor.

Side effects:
- Static UI (paused video, splash screen) may briefly raise
  freq because comm-walk matches RenderThread (it's still
  alive). Wave A's `render_thread_util_aware` filter mitigates
  this; keep it on.
- `fg_transition_pulse` can falsely arm on every notification
  swipe. Tune the pulse_pct down if it's too aggressive.

---

## Recipe 5 — "thermal-leaning sustained workload"

Long encode / ML / build / sustained streaming. Don't let a
brief peak overrun the thermal envelope.

```
auto_thermal_cap_aware           = 1
auto_thermal_cap_temp_c          = 42        # cap at 42 °C skin temp

# Pre-emptive cap at PSI mem pressure (helps swap-thrashing):
psi_mem_cap_thresh               = 60        # PSI mem some=60
psi_mem_cap_pct                  = 75
psi_mem_cap_window_ms            = 1000

# light_cap takes over once classifier sees light:
sleeper_tail_thresh_us           = 4000
sleeper_tail_pct                 = 60

# Keep peer_ramp / migration off; they raise:
peer_ramp_floor_pct              = 0
migration_floor_pct              = 0

# Cluster wake pulse off (prevents brief peaks):
cluster_wake_pulse_floor_pct     = 0
```

Effect: caps engage proactively at moderate temperature and
memory pressure; floors that would push freq up are disabled.

Side effects:
- Less peak performance available even when not thermally
  limited. Don't combine with Recipe 2 (gaming).
- Long encoders / builds finish slower but cooler.

---

## Recipe 6 — "energy-efficient sustained workload"

Sit at the energy-knee freq for long-running compute. Wave B
specific.

```
em_aware                         = 1
em_floor_pct                     = 100       # right at the knee

pmu_aware                        = 1
pmu_ipc_thresh                   = 80        # 0.8 IPC threshold
pmu_ipc_floor_pct                = 75        # IPC-aware floor

# Peer / migration floors disabled; em_floor handles it:
peer_ramp_floor_pct              = 0
migration_floor_pct              = 0
hispeed_freq_pct                 = 0

batt_hold_scale_pct              = 90        # mild bias toward bottom
```

Effect: freq sits at the OPP that minimises joules per
instruction. PMU IPC counter raises floor by another 25% when
the workload is actually compute-bound (≥ 0.8 IPC).

Side effects:
- Foreground latency slightly worse than Recipe 4. Mix-and-
  match if you need both: Recipe 6 baseline + Recipe 4 top_app.
- Requires `CONFIG_ENERGY_MODEL=y` and `CONFIG_PERF_EVENTS=y`
  on the device.

---

## Recipe 7 — "quick wakeup responsiveness"

Crisp wake-from-idle, minimal lag on touch.

```
input_boost_*                    = (raise input boost duration)
input_boost_freq_pct             = 80
input_boost_window_ms            = 200
input_boost_decay_window_ms      = 150

cluster_wake_pulse_ms            = 12        # 1 vblank @ 60Hz
cluster_wake_pulse_idle_ms       = 8         # 0.5 vblank
cluster_wake_pulse_floor_pct     = 70

fg_transition_pulse_ms           = 64
fg_transition_pulse_pct          = 75

# Boot boost helps post-cold-boot too:
boot_boost_window_ms             = 30000     # 30s
boot_boost_freq_pct              = 75
```

Effect: every input event lifts freq for 200 ms, with a 150 ms
decay tail. Cluster-wake pulse gives an extra 12 ms of high freq
when a CPU first comes out of idle.

Side effects:
- Slight battery cost; input_boost is by far the largest
  contributor to "wakeup CPU spike" battery on most devices.
- If the device has a janky touch driver firing spurious
  events at idle, input_boost may falsely arm. Check
  `/sys/class/input/input*/inhibited` before tuning.

---

## Recipe 8 — "minimum freq while charging"

Don't pin to peak just because the charger is plugged in.

```
charger_aware                    = 1
charger_floor_pct                = 0         # no extra floor on AC

# But still respect the per-app uclamp.min:
ZENITH_FEATURE_ENABLED(uclamp_min_observer)  = 1
```

Effect: AC attached lifts the implicit "battery low conservation"
gate but does not impose a hard floor.

(Pre-Wave-A behaviour was: charger plugged → many internal floors
silently lift. Wave A's `charger_aware`/`charger_floor_pct` makes
this explicit.)

Side effects:
- Apps that depend on background sync running at high freq
  during charge (e.g., backup utilities) may take longer.

---

## Recipe 9 — "background apps stay low"

System-background and background tasks shouldn't pin clusters.

```
bg_util_scale_pct                = 60        # scale background util by 60%

# top-app prioritised:
top_app_aware                    = 1
top_app_floor_pct                = 55

# Conservative floors for background:
audio_floor_pct                  = 35
render_floor_pct                 = 0         # off; only top-app render counts

# Fast back-off:
down_threshold                   = 5
peak_step_down_pct               = 35
peak_hysteresis_streak           = 2
```

Effect: PELT util from background tasks is scaled down 40%
before the freq decision; only top-app gets a real floor.

Side effects:
- Long-running background work (backups, indexing) takes
  longer. Acceptable on most phones; may annoy on tablets.

---

## Recipe 10 — "compute-bound workloads (encoders, ML)"

Wave B specific. IPC-aware floor.

```
pmu_aware                        = 1
pmu_ipc_thresh                   = 120       # 1.2 IPC ≈ "actually computing"
pmu_ipc_floor_pct                = 85

# Skip memory-pressure cap; encoders are compute, not memory:
psi_mem_cap_pct                  = 100       # disabled

# Hold thresholds:
hispeed_load                     = 70
brutal_entry_streak              = 2
```

Effect: encoder threads sustaining 1.2+ IPC trigger an 85% freq
floor. Memory-bound stalls (which would push IPC down) don't
trigger.

Side effects:
- Need `CONFIG_PERF_EVENTS=y`.
- The first auto_tune cycle (10s) is needed for the IPC
  measurement to converge. Not for very brief encodes.

---

## Recipe 11 — "memory-bound workloads (browser, large datasets)"

Don't burn freq when stalled on memory bandwidth.

```
psi_mem_cap_thresh               = 50        # PSI mem some=50
psi_mem_cap_pct                  = 60
psi_mem_cap_window_ms            = 500

# IPC inverse: fall back to base when IPC is low:
pmu_aware                        = 1
pmu_ipc_thresh                   = 80
pmu_ipc_floor_pct                = 0         # don't raise on low IPC

# Tail for prefetcher hits:
sleeper_tail_thresh_us           = 2000
sleeper_tail_pct                 = 70
```

Effect: when memory pressure is high, freq is capped at 60%; the
IPC floor is gated on but disabled (no raise for memory-bound
work).

Side effects:
- Some browser workloads have 0.4-0.6 IPC (memory-bound) and
  0.9-1.4 IPC (JIT-hot). The 80% threshold catches the latter
  only.

---

## Recipe 12 — "render at 90+ Hz screens"

If your panel is 90 Hz or 120 Hz, default frame_overrun thresholds
may be too coarse.

```
frame_pace                       = 1
frame_overrun_slack_us           = 1000      # 1 ms slack at 90 Hz
frame_overrun_window_ms          = 11        # 1 vblank @ 90 Hz
frame_overrun_floor_pct          = 75
frame_overrun_deep_streak        = 4
frame_overrun_deep_floor_pct     = 90

# DRM vblank notifier should be enabled:
drm_vblank_us                    = 11000     # 11 ms = 90 Hz
```

For 120 Hz: `drm_vblank_us=8333`,
`frame_overrun_window_ms=8`, `frame_overrun_slack_us=750`.

Side effects:
- Vblank-paced freq adds ~1-2 mW idle when nothing is rendering.
- Inaccurate `drm_vblank_us` causes false overrun arms.

---

## Recipe 13 — "light boot pin"

Avoid the 60-second post-boot "everything pinned" period that some
governors apply.

```
boot_boost_window_ms             = 5000      # 5s only
boot_boost_freq_pct              = 65        # gentle
boot_boost_decay_window_ms       = 3000

# After boot boost, fall to schedutil-level quickly:
auto_eval_ms                     = 5000      # classify every 5s
auto_hysteresis_ms               = 2500
```

Effect: 5s of mild pin (65%), 3s of glide-down, then standard
classifier behaviour.

Side effects:
- App pre-warming (Android's pre-launch / RAM resident apps)
  takes a few seconds longer to settle.

---

## Recipe 14 — "v4l2 camera latency"

If you have a Camera app open and want frame-capture latency to
be predictable.

```
camera_floor_pct                 = 65
camera_aware                     = 1

# Avoid quiet_hours interfering during camera open:
quiet_hours_screen_off_only      = 1

# Audio floor too (camera = video):
audio_aware                      = 1
audio_floor_pct                  = 45
```

Effect: while a v4l2 device is open, freq stays above 65%.

Side effects:
- Battery cost while camera app is open in background.
  Acceptable; user expectation is "camera open = device active".

---

## Recipe 15 — "totally vanilla pre-Wave-A behaviour"

Disable every Wave-A and Wave-B opt-in tier. Useful for A/B
comparisons.

```
charger_aware                    = 0
charger_floor_pct                = 0

top_app_aware                    = 0
top_app_floor_pct                = 0

render_thread_util_aware         = 0
render_thread_util_thresh        = 0
render_thread_util_floor_pct     = 0

pmu_aware                        = 0
pmu_ipc_thresh                   = 100      # default; irrelevant when off
pmu_ipc_floor_pct                = 0

em_aware                         = 0
em_floor_pct                     = 0
```

Effect: bit-identical to a pre-Wave-A boot. Use for diff'ing
behaviour against a reference.

---

## Recipe combinations

Recipes are largely orthogonal. Common combinations:

- **"Default phone"** (Recipes 1 + 4 + 7 + 9): battery on idle,
  smooth UI, snappy wake, low background.
- **"Gaming phone"** (Recipes 2 + 7 + 8): max latency, fast wake,
  no charger throttle.
- **"Audio production"** (Recipes 3 + 5 + 8): stable foreground
  audio + thermal-aware sustained.
- **"Encoder rig"** (Recipes 6 + 10 + 5): energy-knee sustained,
  IPC-aware floor, thermal cap.
- **"Browser-only laptop"** (Recipes 4 + 9 + 11): smooth scroll
  + low background + memory-aware cap.

Mix-and-match by stacking the recipe blocks. Conflicting writes
to the same knob: last write wins. Recipes use distinct knobs
where possible to avoid this.

---

## Profile-based shortcuts

If you don't want to copy-paste recipe blocks, the `profile`
sysfs node bundles common combinations:

```bash
echo balanced     > /sys/.../zenith/profile   # ≈ Recipe 4 + 9
echo performance  > /sys/.../zenith/profile   # ≈ Recipe 2 + 7
echo powersave    > /sys/.../zenith/profile   # ≈ Recipe 1 + 9
echo latency      > /sys/.../zenith/profile   # ≈ Recipe 4 + 7
echo efficiency   > /sys/.../zenith/profile   # ≈ Recipe 6 + 9
```

Profiles are bulk-update operations; they overwrite all canonical
tunables in one shot. After the bulk write, you can layer
individual tweaks on top with no loss of generality.

---

## Discovery / debugging

To verify a recipe is working:

1. Apply.
2. Drive the workload (Recipe 4 = scroll, Recipe 2 = game, etc.).
3. Watch the tier resolution:

```bash
adb shell 'cd /sys/kernel/debug/tracing && \
    echo 1 > events/cpufreq_zenith/cpufreq_zenith_pick/enable && \
    cat trace_pipe | head -200'
```

The `tp_path` field tells you which tier set the final freq. If
your recipe has e.g. `top_app_floor_pct=60` but `tp_path` always
shows `eas` or `hispeed`, the top_app gate is not firing. Check:

```bash
adb shell cat /sys/.../zenith/top_app_aware    # is gate on?
adb shell cat /proc/<pid>/cpuset               # is task in top-app?
adb shell cat /sys/.../zenith/top_app_floor_pct  # is value non-zero?
```

For Wave B / EM:

```bash
adb shell cat /sys/.../zenith/em_aware
adb shell cat /sys/devices/system/cpu/cpu0/cpu_capacity
adb shell ls /sys/devices/.../em_perf_domain*  # cpufreq driver should expose
```

If `em_cpu_get()` returns NULL for the policy, `em_floor` cannot
fire even with the gate on; the cpufreq driver must call
`em_dev_register_perf_domain()` for the EM to be available.

---

## Hard rules (paraphrasing soul.md)

- All values are unsigned. Negative writes are -EINVAL.
- All percentages are 0-100 unless documented otherwise (e.g.
  `em_floor_pct` is 0-200).
- All time values are in the units in their name (`*_ms`, `*_us`,
  `*_ns`).
- Setting a `*_aware` to 0 disables the entire tier; underlying
  values stay but are inert.
- Setting a `*_floor_pct` or `*_cap_pct` to 0 disables the tier
  even if the master gate is on.
- Tunables never get renamed or removed. Only new tunables are
  added.

---

## Cross-reference

For per-tier semantics, see `zenith-tier-flow.md` (this tarball).
For boot-time defaults, see `init.zenith.rc` V5+ (Wave A drop).
For build-time / debug-kernel testing, see Wave C `runbook.md`.
