# Custom Driver Audit — Full Code-Level Review (2026-08-15)

Scope: every custom component in the GrayRavens stack on
`android12-5.10-experimental` (HEAD `fe2424237ac0`).  This is a full
code-level audit — not just the reboot-symptom mapping from the earlier
pass — covering bugs, dead code, bootloop risk, KMI safety, and
concurrency.

Legend: 🔴 proven bug / blocking · 🟡 probable / needs device data ·
🟢 clean · ⚪ dead code (compiled but not wired / not active)

---

## Summary — the two big stories

### 1. Half the "game-mode stack" is compiled but NOT WIRED — dead code

The clean-base re-import commit `2139a6d29f46` ("zenith: GrayRavens
custom drivers on clean ACK LTS base") brought the custom .c files back
into the tree **without re-applying the core-kernel wiring commits**.
Result: several components compile, register sysfs, and log "loaded",
but their hooks are never called:

| Component | Claimed hook | Reality on this branch | Status |
|---|---|---|---|
| `kasumi` | `kasumi_dampen()` called from `thermal_zone_get_temp()` (thermal_helpers.c) | `thermal_helpers.c` is pristine upstream — **zero** kasumi references. `kasumi_dampen` has **no callers anywhere**. | ⚪ dead |
| `iyashi` | `iyashi_clamp_target()` called from `thermal_cdev_update()` (thermal_helpers.c) | Same — **zero** iyashi references in the thermal core. | ⚪ dead |
| `hikari` | `hikari_on_enqueue/dequeue/wake_up/select_cpu` called from fair.c / core.c | fair.c + core.c pristine — **zero** hikari references. `hikari_on_*` have **no callers**. | ⚪ dead |
| `thermal_charger_guard` | suppresses kasumi/iyashi while charging | Calls `kasumi_set_charger_suppressed()` / `iyashi_set_charger_suppressed()`, which flip a flag — but nothing reads that flag (no dampen/clamp path). | ⚪ dead |
| `zenith_gpu_switch` | listens to zenith game-mode notifier | **Wired and live** — zenith's atomic notifier fires it. Works. | 🟢 |

Proof (git):
- `git log --all -S kasumi_dampen -- drivers/thermal/thermal_helpers.c`
  → commits `61dae849f63f`…`a94dd9c62f09` (the wiring) exist on other
  branches but are **not ancestors of HEAD**.  The re-import
  `2139a6d29f46` re-added kasumi.c/iyashi.c and Makefile/Kconfig lines
  but left `thermal_helpers.c` untouched.
- `git merge-base --is-ancestor ed3b288b0844 HEAD` → **NO**.  Commit
  `ed3b288b0844` ("hikari: wire scheduler hooks in fair.c and core.c")
  is not in this branch's ancestry.
- `grep -rln hikari_on_enqueue --include='*.c'` → only hikari.c itself.

**Impact**: thermal dampening (kasumi), the thermal performance floor
(iyashi), and the entire wake-time engine (hikari) silently do
**nothing** on this branch.  Sysfs nodes exist, dmesg says "loaded",
profiles get applied to tunables that are never read.  This is not a
crash risk — it's a **feature-completeness regression** introduced by
the rebase.

**Bootloop risk: NONE** — dead code can't loop.  (Ironically the
symptom-mapping doc's kasumi-thermal-hiding theory is moot: kasumi
isn't even active.)

Also verified: the same wiring is missing in `android13-5.15` (port
target) — `thermal_helpers.c` and `fair.c` there have zero kasumi /
iyashi / hikari refs too.  Fix once, fix both trees.

### 2. Only one provable kernel-side bug was ever found — and it's fixed

The zios sleep-in-atomic (`zios_dispatch_request` calling
`thermal_zone_get_temp()` / `power_supply_*` under the blk-mq hctx
spinlock) was the only real panic-class bug.  It was fixed by
`fe2424237ac0` ("zios: defer thermal/battery checks out of dispatch
path"), then **the whole scheduler was dropped** on 2026-08-16
(`84db02b8bbb8`, "block: drop zios I/O scheduler") — stock ROM tuning
scripts flip the live elevator at runtime (`elv_iosched_store`) and
land on `mq-zios`, where the custom dispatch/read-ahead behavior kept
producing crash and reboot reports on GKI 2.0 devices.  This matches
the tester's `BUG: scheduling while atomic: kworker/u16:4` +
`workqueue leaked lock` log exactly (V14 shipped with the unfixed bug).

---

## Per-component findings

### `block/zios-iosched.c` — 🗑️ REMOVED (2026-08-16, `84db02b8bbb8`)

- Dispatch-path sleep bug fixed at `fe2424237ac0`, then the driver was
  removed outright for universal-boot stability: nothing selected it as
  the default, runtime elevator flips from userland tuning land on
  `mq-zios`, and its workload heuristics mutate global read-ahead
  (`ra_pages`) at runtime.  GKI keeps mq-deadline / kyber / bfq.
- Removal touched `block/Kconfig.iosched`, `block/Makefile`,
  `gki_defconfig`, and deleted `block/zios-iosched.c` (1.6k lines,
  1,592 deletions total).  No cross-driver references existed.

### `block/ssg-iosched.c` + `block/ssg-cgroup.c` — ⚪ not enabled

- Dispatch is spinlock-only, no sleep-capable calls; cgroup callbacks
  are standard Samsung code.  **But `CONFIG_MQ_IOSCHED_SSG` is not set
  in gki_defconfig** — ssg isn't built into the shipped kernel at all.
  No risk; just dead config on GKI.

### `kernel/sched/selene.c` — 🟢 live, with minor notes

- Blocking notifier chain + scan workqueue: all process context, no
  preempt imbalance.  Sysfs handlers take `selene_lock` only.
- ⚠ Minor: `selene_load_custom_list()` reads `/data/misc/selene/
  custom_games.txt` at module init — `/data` usually isn't mounted at
  init time, so the file silently never loads (returns early).  It
  works when the file exists, which it rarely does at boot.
- ⚠ Minor: the scan's "Phase 1" only inspects the **first** top-app
  task per tick (breaks on first match); if the game's process isn't
  the first top-app task it can be missed.  Functional limitation, not
  a crash.
- `selene_read_cmdline()` uses `filp_open("/proc/%d/cmdline")` from the
  scan workqueue — process context, safe.  Slightly unusual (kernel
  reading its own procfs) but not a bug.

### `kernel/sched/koakuma.c` — 🟢 live

- Preload runs in `schedule_work` (process context) with
  `get_task_struct()` ref held across the mmap walk — no UAF.
- ⚠ Minor: `koakuma_preload_work_fn()` returns early when
  `get_task_mm()` returns NULL (game exited) **without**
  `put_task_struct()` — leaks the task ref on that path.  One ref per
  missed preload, bounded by game-start events; not a crash, worth a
  one-line fix.
- GAME_START handler: ref taken under RCU, stored under mutex — the
  `put_task_struct` on re-entry is balanced.  Clean.

### `kernel/sched/lucid.c` — 🟢 live, with one race worth fixing

- ⚠ **Race (low severity)**: `lucid_read_oom_adj()` / `lucid_write_oom_adj()`
  dereference `task->sighand` / `task->signal` after a NULL check but
  without `lock_task_sighand()`.  If the task exits between the check
  and the lock, `sighand` can be freed → use-after-free.  The kernel's
  canonical pattern is `lock_task_sighand(task, &flags)` which handles
  exactly this race.  In practice the window is tiny and tasks checked
  are usually alive (game start), but `lucid_write_oom_adj`'s direct
  `spin_lock_irqsave(&task->sighand->siglock)` is the one place in the
  whole stack with a real (if rare) UAF possibility.  Recommend
  switching to `lock_task_sighand()`.
- PID-cache restore on GAME_STOP uses `find_get_pid` — PID-reuse
  between start and stop would restore the new process's OOM score, but
  scores are per-session and the window is tiny.  Minor.
- Kill-list matching is substring-based (`strstr`) on comm/cmdline —
  a short pattern like "a" matches nearly everything; operators should
  use full package names.  Config risk, not a code bug.

### `kernel/sched/hikari.c` — ⚪ DEAD on this branch (see summary)

- The whole wake-time engine (EWMA measurement, uclamp boost, freq-hint
  publish, placement) is never invoked — fair.c/core.c don't call it.
- The code *itself* is well-written (spinlock side table, static-key
  gating, self-disable with kill flag).  `BUG()` under
  `CONFIG_HIKARI_DEBUG` is debug-only.
- What DOES work: sysfs/sysctl plumbing and `hikari_apply_profile()`
  writes — but since the notifier chain is only fired from
  `hikari_on_dequeue()` (never called), even the cross-links to
  zenith/iyashi are inert.
- **Fix**: cherry-pick the wiring (`ed3b288b0844` from the old branch)
  or re-apply the fair.c/core.c hook calls, then re-verify KMI (the
  side-table design means no task_struct layout change — good).

### `kernel/sched/cpufreq_zenith.c` (24k lines) — 🟢 live, spot-audited

- Registers as a cpufreq governor + registers tracepoint vendor-hooks
  (`register_trace_android_vh_*`) — the rollback path on governor-reg
  failure unregisters everything (no leak).  Solid.
- **Zero** `BUG()/BUG_ON/panic()` in the whole file.
- `zenith_is_game_mode_active()` is a static-key + READ_ONCE — safe on
  the zios dispatch path.
- Mutexes (`zenith_psi_cgroup_lock`, `zenith_comm_table_lock`,
  `work_lock`, `global_tunables_lock`) all in sysfs/workqueue/process
  contexts — no sleep-in-atomic found.
- The game-mode notifier chain is `ATOMIC_NOTIFIER_HEAD` — correct for
  the scheduler tick context it's fired from; gpu_switch's callback
  (mod_delayed_work) is atomic-safe.
- ⚠ One design risk (not a bug): `CPU_FREQ_DEFAULT_GOV_SCHEDUTIL` is
  still `=y` in gki_defconfig — zenith must be selected at runtime
  (`echo zenith > scaling_governor`), which the docs' init.zenith.rc
  expects.  Nothing switches it automatically in the kernel.  If the
  init script isn't installed on a device, **the whole governor is
  inactive** and the device runs stock schedutil.  Verify the target
  device actually installs init.zenith.rc (or flip the defconfig
  default).

### `kernel/sched/cpufreq_schedhorizon.c` — 🟢 clean

- Schedutil derivative; same locking as upstream schedutil; not
  default; no issues found.

### `drivers/thermal/iyashi.c` — ⚪ DEAD (no clamp hook wired)

- `iyashi_clamp_target()` never called (thermal_helpers.c pristine).
- Its `enforce_min` FREQ_QOS path is the only live code — but it's
  default-off and only meaningful via sysfs.
- Code quality: self-test at init, static-branch gating, qos lifecycle
  handled.  Just unwired.
- **Fix**: add the `iyashi_clamp_target(cdev, target)` call in
  `thermal_cdev_update()` (per original `f9785120bbb7`).

### `drivers/thermal/kasumi.c` — ⚪ DEAD (no dampen hook wired)

- `kasumi_dampen()` never called; `kasumi_zone_allowed()` never called.
- Safety self-test at init passes against its own math but proves
  nothing about the (missing) wiring.
- **Fix**: add the `kasumi_zone_allowed()` + `kasumi_dampen()` calls in
  `thermal_zone_get_temp()` (per original `a94dd9c62f09`).

### `drivers/thermal/thermal_charger_guard.c` — ⚪ DEAD (suppresses nothing)

- Its whole purpose is suppressing kasumi/iyashi while charging; both
  targets are unwired so the suppression flags are no-ops.
- Re-audit after the wiring fix; the module itself (debounce, batt-temp
  safety gate, tier logic) is clean.

### `drivers/devfreq/zenith_gpu_switch.c` — 🟢 live

- Game-mode notifier → `mod_delayed_work` (atomic-safe, correct for
  zenith's ATOMIC chain).  12-attempt GPU probe guard; `gpu_exiting`
  flag prevents re-queue after teardown; governor-missing fallback
  (5s flap fix `c10d621c19ab`) intact.
- ⚠ Minor: the VM tunable writes (dirty ratios, min_free_kbytes) are
  direct global assignments, not via the sysctl handlers — they take
  effect immediately but skip any sysctl-side side effects.  Acceptable
  for this driver (mirrors what userspace tuning scripts do), but the
  save/restore is on game transitions only — if the module is unloaded
  mid-game the tunables stay at game values.  Minor.

### `kernel/kaguya.c` — 🟢 live (default-off)

- Default `enabled=0` (commit `18bf09eeb4e5`) — work loop doesn't even
  schedule when off.  `call_usermodehelper` only when enabled +
  props exist.  Buffer-bounded cmd build (4K) with truncation guard.
- ⚠ Minor: `props_add` allows any name/value up to 128 chars but there
  is no validation against shell metacharacters — a value containing a
  single quote would break the `sh -c` quoting (injection).  This is
  root-only sysfs so it's not a privilege boundary, but a
  `!strchr(value, '\'')` reject would be safer.
- `KAGUYA_CMDLINE_SPOOF` is `n` in defconfig — `/proc/cmdline` /
  `/proc/bootconfig` hooks (`fs/proc/cmdline.c` etc.) compile out.
  Safe.

### `net/ipv4/tcp_bbr3.c` + `net/ipv4/tcp_plb.c` — 🟢 clean

- KABI-safe (state in `icsk_ca_priv`, NULL-guarded callbacks, `tp`
  declarations fixed).  WARN_ONCEs match upstream BBR patterns.  Default
  congestion control is BBR3 (`CONFIG_DEFAULT_BBR3=y`) — this one IS
  live and correct.

---

## Bootloop / reboot risk assessment

| Scenario | Verdict |
|---|---|
| Kernel panic in custom code | **None found.** The only panic-class bug was zios's sleep-in-atomic; zios is now **removed** (`84db02b8bbb8`). |
| Bootloop from dead subsystems | **Impossible** — kasumi/iyashi/hikari don't run. |
| Reboot from workqueue wedge | Gone — zios removed entirely. No custom scheduler remains in the tree. |
| lucid sighand UAF | Rare (process exiting exactly during game-start OOM scan). Low probability, but **worth fixing** with `lock_task_sighand()`. |
| Device "boots on A, not B" | Not a kernel-code issue — classically dtbo/vendor-blob mismatch (see symptom mapping below). |
| Flicker | Display-side (refresh-rate/panel-config), not this tree's drivers. |

## Fix status (2026-08-15) — all applied to the working tree

1. ✅ **kasumi + iyashi re-wired** into the thermal core
   (`drivers/thermal/thermal_helpers.c`): `kasumi_dampen()` hook in
   `thermal_zone_get_temp()` (after emulation block, under tz->lock),
   `iyashi_clamp_target()` hook in `thermal_cdev_update()` (under
   cdev->lock, before `set_cur_state`).  Both include their headers;
   CONFIG=n builds get the passthrough stubs.
2. ✅ **hikari re-wired** into the scheduler:
   - `kernel/sched/fair.c` — `hikari_on_enqueue()` in
     `enqueue_task_fair()`, `hikari_select_cpu()` in
     `select_task_rq_fair()` (after the vendor RVH, before
     `record_wakee()`), `hikari_on_dequeue()` in `pick_next_task_fair()`
     `done:` path (after `update_misfit_status`).
   - `kernel/sched/core.c` — `hikari_on_wake_up()` in
     `try_to_wake_up()` (before `ttwu_queue`), plus the uclamp boost in
     `uclamp_eff_value()` via `uclamp_apply_hikari_boost()`.
   - All hook signatures match `<linux/hikari.h>`; the atomic freq-hint
     notifier consumer (zenith) is READ_ONCE-only, so it's safe on the
     scheduler hot path.
3. ✅ **lucid**: OOM helpers now use `lock_task_sighand()` — closes the
   sighand UAF race on exiting tasks.
4. ✅ **koakuma**: `put_task_struct(task)` on the `get_task_mm()` NULL
   path — no more ref leak.
5. ✅ **zios**: `periodic_work` moved to a dedicated `zios` workqueue
   (WQ_UNBOUND) — superseded on 2026-08-16 by **removing zios entirely**
   (`84db02b8bbb8`); no custom I/O scheduler remains.
6. ✅ **kaguya**: `props_add` rejects single quotes in name/value — no
   more shell-injection into the `sh -c` resetprop batch.

Remaining (deliberately untouched):

- **Default governor**: `gki_defconfig` keeps `CPU_FREQ_DEFAULT_GOV_SCHEDUTIL`.
  This is the *safe universal-boot* choice: the kernel boots stock-sane
  on every device and zenith activates via init.zenith.rc at runtime.
  Flipping the default to zenith would make every 5.10 device run the
  custom governor by default — the opposite of "boots everywhere with
  no issues".
- **thermal_charger_guard** becomes meaningful again now that
  kasumi/iyashi are live — its own logic was already audited clean.
- **5.15 port** must carry the same wiring fixes (currently missing
there too).

Verification: `scripts/checkpatch.pl` clean (0 errors, 0 warnings),
no exported-symbol signature changes, no struct layout changes
(hikari stays in its side table) → KMI CRCs unaffected.  Full build
+ KMI checker still needed on a toolchain-equipped machine.

## Usefulness audit (2026-08-15): selene / lucid / koakuma / kaguya

Do the game-chain drivers earn their keep, and can any of them cause the
reboot / bootloop / flicker reports?  Verdicts are based on the live code
at HEAD, not the marketing description.

| Driver | Default | What it ACTUALLY does | Cost | Risk | Verdict |
|---|---|---|---|---|---|
| **selene** | ON (scans every 500 ms) | Walks all tasks, matches top-app comm/cmdline vs the 531-game table, broadcasts game start/stop events. **Every** game-mode feature (gpu_switch, koakuma, lucid, zenith, iyashi, kasumi) consumes these events. | ~2 full task-walks/sec, always | Low (RCU walk, audited clean) | **Keep — the hub.** Without it the whole stack is dead. Optional: raise scan interval to 1000 ms when idle. |
| **lucid** | ON | **Does NOT kill anything** (no `send_sig`/`force_sig`/`do_group_exit` anywhere). Only rewrites `oom_score_adj`: game -> -900 (OOM-invincible), matched bg procs -> +500 (OOM-first), and **restores both on game stop**. "killed_count" is really a deprioritized counter. | One pass on game start/stop | Low (was: UAF race on sighand — fixed). adj-only changes cannot crash or reboot a device. | **Keep — low risk, moderate value.** Real benefit: game survives OOM pressure. lmkd does similar, but -900 on the game is a genuine add. |
| **koakuma** | ON | One-time preload of game exec + read-only (non-COW, >=64 KB) mappings into page cache, capped at 64 MB default. Skips writable/COW/tiny VMAs. | One burst of up to 64 MB readahead per game start | Low (was: task-ref leak — fixed) | **Keep — modest value.** 64 MB vs GB-sized games is a small head start, but launch-time readahead is real and it costs nothing continuously. |
| **kaguya** | **OFF** (`kaguya_enabled=false`, cmdline spoof off) | When enabled: every 10 s runs resetprop to enforce `ro.boot.*` spoof props; optional `/proc/cmdline` rewrite. | Zero when off; 10 s subprocess ticks when on | None when off. When ON: subprocess + 30 s boot delay; cmdline spoof is the only one of the four with a device-boot failure mode (strict vendor init checks) | **Keep, but leave default-OFF** — pure integrity-spoofing feature, zero stability value. For a "boots everywhere" kernel it should stay off (it already is). |

Bottom line: none of the four can cause the reported reboots/bootloops in
their current form — lucid never signals, koakuma is a bounded one-shot,
selene is a read-only scanner, kaguya is off.  The reboot-class bugs were
zios (removed at HEAD, `84db02b8bbb8`) and lucid's UAF (fixed at HEAD).
Flicker was the GPU high-load promotion (removed at HEAD,
`1d0e7383da50`).  Bootloop/no-boot on specific devices is a
vendor_boot/KMI mismatch (device-side), not these drivers.
