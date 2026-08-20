# GrayRavens → android13-5.15 (ACK) Port Audit

Full-scale audit, generated 2026-08-08 from the live trees.

- **Source**: `XTENSEI/kernel_common_5.10` @ `android12-5.10-experimental` (HEAD `7ebcfcf0a674`, base Linux **5.10.264**)
- **Target**: `XTENSEI/kernel_common_5.15` @ `android13-5.15-experimental` (HEAD `b885b84af7a2`, base Linux **5.15.211**)
- **Target branches ready**: `android13-5.15-aosp` (upstream, read-only), `android13-5.15-experimental` (port staging), `android13-5.15-rebased` (stable), `android13-5.15-lts`
- **Target tree status**: completely clean — zero GrayRavens code present.

---

## 1. MGLRU status in 5.15 — ✅ already in

| Check | 5.15 (android13-5.15) | 5.10 (android12-5.10-MGLRU) |
|---|---|---|
| `CONFIG_LRU_GEN` | ✅ `=y` in gki_defconfig (line 129) | ✅ `=y` (line 750) |
| `CONFIG_LRU_GEN_ENABLED` | ❌ **not set** | ✅ `=y` (line 751) |
| mm/vmscan.c lru_gen code | ✅ 159 refs | ✅ present |

**Verdict**: MGLRU (multi-gen LRU) is fully compiled into 5.15. It is simply not
**enabled by default** — 5.15 ships it off-by-default (runtime switch
`/sys/kernel/mm/lru_gen/enabled`, bit 0 = anon, bit 1 = file, 0x3 = all).

**Port action**: add to `gki_defconfig`:
```
CONFIG_LRU_GEN=y
CONFIG_LRU_GEN_ENABLED=y        # enable anon+file by default (matches 5.10 MGLRU branch)
CONFIG_LRU_GEN_STATS=y          # optional, full stats for debugging
```
Note 5.15 Kconfig constraint: `depends on !MAXSMP && (64BIT || !SPARSEMEM || SPARSEMEM_VMEMMAP)` — arm64 GKI is 64BIT, fine.

---

## 2. Port manifest — 94 files (from author commits, merges excluded)

Source: `git log --no-merges --author='アンドレイ' --name-only origin/android12-5.10-experimental`

### 2a. NEW custom drivers / files (must be created in 5.15)

| File | What | 5.15 status |
|---|---|---|
| `block/adios.c` | ADIOS I/O scheduler (Samsung) | new |
| `block/ssg-iosched.c` | SSG I/O scheduler | new |
| `block/ssg-cgroup.c` / `block/ssg-cgroup.h` | SSG cgroup support | new |
| `block/zios-iosched.c` | **ZIOS** game-aware I/O scheduler | new |
| `drivers/devfreq/zenith_gpu_switch.c` | **zenith_gpu_switch** devfreq auto-switcher | new |
| `drivers/thermal/iyashi.c` / `iyashi.h` | **Iyashi** thermal floor | new |
| `drivers/thermal/kasumi.c` / `kasumi.h` | **Kasumi** thermal dampening | new |
| `drivers/thermal/thermal_charger_guard.c` | **Charger guard** thermal cdev | new |
| `kernel/kaguya.c` | **Kaguya** safeprop enforcer | new |
| `kernel/sched/cpufreq_schedhorizon.c` | **Schedhorizon** governor | new |
| `kernel/sched/cpufreq_zenith.c` | **Zenith** governor | new |
| `kernel/sched/hikari.c` | **Hikari** wake-time policy | new |
| `kernel/sched/koakuma.c` | **Koakuma** game preloader | new |
| `kernel/sched/lucid.c` | **Lucid** game-mode OOM/task killer | new |
| `kernel/sched/selene.c` + `selene.h` + `selene_gamelist.c/.h` | **Selene** game detection (531-game table) | new |
| `kernel/zenith-allowlist.c` | **Zenith allowlist** (off by default) | new |
| `net/ipv4/tcp_bbr3.c` | **BBRv3** congestion control | new |
| `net/ipv4/tcp_plb.c` | BBRv3 PLB (per-link balance) | ❌ **absent in 5.15** — must port |

### 2b. Modified in-tree files (hooks/wiring — cherry-pick + adapt)

**Block**
- `block/Kconfig.iosched` — add `MQ_IOSCHED_ADIOS`, `MQ_IOSCHED_ZIOS`, `MQ_IOSCHED_SSG[_CGROUP]`
- `block/Makefile` — `adios.o`, `zios-iosched.o`, `ssg-iosched.o`+`ssg-cgroup.o` (note the `ssg.o` wrapper trick, lines 27-35)

**cpufreq**
- `drivers/cpufreq/Kconfig` — `CPU_FREQ_GOV_ZENITH`, `CPU_FREQ_DEFAULT_GOV_ZENITH`, `CPU_FREQ_GOV_SCHEDHORIZON`, `ZENITH_DEBUG_MSG`

**devfreq**
- `drivers/devfreq/Kconfig` — `DEVFREQ_ZENITH_GPU_SWITCH`
- `drivers/devfreq/Makefile` — `zenith_gpu_switch.o`
- `drivers/devfreq/devfreq.c` + `include/linux/devfreq.h` — GPU governor hooks / attrs

**thermal**
- `drivers/thermal/Kconfig` — `IYASHI`, `KASUMI`, `THERMAL_CHARGER_GUARD`
- `drivers/thermal/Makefile` — `iyashi.o`, `kasumi.o`, `thermal_charger_guard.o`
- `drivers/thermal/cpufreq_cooling.c` + `include/linux/cpu_cooling.h` — cooling floor + kcompactd helpers

**fs / kaguya**
- `fs/proc/cmdline.c`, `fs/proc/bootconfig.c` — `kaguya_spoof_boot_args()` hooks (`CONFIG_KAGUYA_CMDLINE_SPOOF`)

**scheduler (the big one)**
- `kernel/sched/Makefile` — schedhorizon/zenith/hikari/selene/lucid/koakuma objects
- `kernel/sched/core.c`, `fair.c`, `rt.c`, `idle.c`, `autogroup.c`, `debug.c`, `features.h`, `pelt.c`, `psi.c`, `sched.h`, `sched-pelt.h`, `cpufreq_schedutil.c` — hooks for zenith/hikari/selene integration
- `include/linux/sched.h` — hikari `hikari_flags` bitfield + related

**headers**
- `include/linux/cpufreq_zenith.h`, `hikari.h`, `kaguya.h`, `selene.h`, `zenith_profiles.h`
- `include/trace/events/cpufreq_zenith.h`, `hikari.h`, `iyashi.h`, `kasumi.h`
- `include/linux/compaction.h` + `mm/compaction.c` — kcompactd helpers

**TCP / BBRv3**
- `net/ipv4/Kconfig`, `net/ipv4/Makefile` — `TCP_CONG_BBR3`, `tcp_plb.o`
- `net/ipv4/tcp.c`, `tcp_cong.c`, `tcp_input.c`, `tcp_ipv4.c`, `tcp_minisocks.c`, `tcp_output.c`, `tcp_rate.c`, `sysctl_net_ipv4.c`, `net/ipv4/tcp_bbr.c` (upstream BBRv1 modified)
- `include/linux/tcp.h`, `include/net/tcp.h`

**net/ipv6 (MLD / mcast)**
- `net/ipv6/mcast.c`, `addrconf.c`, `addrconf_core.c`, `af_inet6.c`, `icmp.c`
- `include/net/mld.h`, `addrconf.h`, `if_inet6.h`, `ndisc.h`
- ⚠️ NOTE: the "Revert mld: convert ipv6 multicast to RCU" series was applied on 5.10 to keep KMI stable — verify current delta before porting; likely skip entirely.

**defconfig / build**
- `arch/arm64/configs/gki_defconfig` — the GrayRavens block (lines ~769-777) + LRU_GEN_ENABLED
- `Makefile`, `scripts/Makefile.clang` — build tweaks
- `KMI_function_symbols_test.py` — CI infra
- `arch/arm64/Kconfig`, `arch/arm64/include/asm/assembler.h`, `arch/arm64/include/asm/thread_info.h` — stackprotector canaries (per-task)

**Docs**
- `zenith-docs/*` (init.zenith.rc, cookbook, rst, tier-flow)

### 2c. Likely NOT to port (upstream-merge fallout / KMI reverts)
- `drivers/gpu/drm/amd/amdgpu/amdgpu_device.c`, `drivers/s390/net/qeth_l3_main.c`, `net/batman-adv/multicast.c`, `net/tipc/udp_media.c`, `drivers/hid/hid-core.c`, `drivers/pci/rebar.c`, `include/linux/pci.h`, `include/linux/netdevice.h`, `include/uapi/linux/{inet_diag,rtnetlink}.h`
- These were touched by the 5.10.26x stable merges / KMI restores. Check each: if the fix already exists in 5.15, skip.

---

## 3. API delta verification (5.15 vs 5.10) — verified against live 5.15 source

| API | 5.15 status | Port difficulty |
|---|---|---|
| `struct cpufreq_governor` | **identical** (init/exit/start/stop/limits/show_setspeed/store_setspeed/flags) | 🟢 trivial |
| `map_util_freq()` | present (`include/linux/sched/cpufreq.h` + `trace_android_vh_map_util_freq` hooks) | 🟢 trivial |
| `struct sugov_policy` / `sugov_cpu` | present, same shape | 🟢 trivial |
| `update_util_data` / `sugov_update_single/shared` | present | 🟢 trivial |
| `struct thermal_cooling_device_ops` | identical + `ANDROID_KABI_RESERVE(1)` | 🟢 trivial |
| `struct thermal_zone_device_ops` | identical | 🟢 trivial |
| `struct devfreq_governor` + `DEVFREQ_GOV_*` events | identical (name/attrs/flags/get_target_freq/event_handler) | 🟢 trivial |
| `struct elevator_mq_ops` (incl. `limit_depth`) | identical + KABI reserve | 🟢 trivial |
| `enqueue_task`/`dequeue_task` + `trace_android_rvh_*` hooks | present | 🟡 easy |
| `saved_command_line` / `fs/proc/bootconfig.c` | present | 🟢 trivial (kaguya) |
| PSI (`psi_system`, `psi_memstall_*`) | present | 🟢 trivial |
| `cpufreq_cooling_register` / `of_cpufreq_cooling_register` | present | 🟢 trivial |
| `kcompactd` (mm/compaction.c) | present | 🟡 easy |
| `tcp_plb.c` | ❌ absent — part of BBRv3 series | 🟠 medium |
| `task_struct` (hikari flags) | **5.15 layout differs from 5.10** — verify bitfield placement | 🟠 medium |
| `sched_pelt` / PELT internals | 5.15 changed some util math | 🟠 medium |

**Bottom line**: the governor/thermal/devfreq/block APIs are essentially
unchanged between 5.10 and 5.15 — the port is mostly cherry-pick + mechanical
adaptation. The real work is the scheduler stack (hikari/zenith hooks) and the
BBRv3 TCP series.

---

## 4. Port order (risk-ascending)

1. **MGLRU default-enable** (defconfig only) — instant win
2. **BBRv3 + tcp_plb** — self-contained in net/, moderate API drift
3. **zenith_gpu_switch** (devfreq) + **iyashi/kasumi/charger_guard** (thermal) — APIs identical
4. **zios / ssg / adios** (block) — elevator_mq_ops identical
5. **kaguya** (props + cmdline spoof) — small, self-contained
6. **selene → koakuma → lucid** — notifier chain, depends on selene first
7. **hikari** (sched) — task_struct bitfield + hooks
8. **zenith** (cpufreq) — biggest file, ~24k lines; needs schedutil/psi/EM integration
9. **schedhorizon**, **zenith-allowlist**, **prefer-silver/assembler/stackprotector** bits
10. **zenith-docs** copy + Kconfig/Makefile/defconfig wiring throughout

---

## 5. Build / CI for 5.15

- GrayRavens-GKI-Builder workflow takes `kernel_ref` — point it at
  `android13-5.15-experimental` and rename output to `GrayRavens-Zenithed-V15-…` (or keep V14-Yukari branding if desired).
- SUSFS patch branch: use `gki-android13-5.15` (workflow currently hardcodes
  `gki-android12-5.10` — must be parameterized).
- KernelSU: `pershoot/KernelSU-Next dev-susfs` supports 5.15; KWS (KOWX) also fine.
- KMI: android13-5.15 has its own `abi_gki_aarch64.xml` — rerun KMI checker
  against the 5.15 tree, not the 5.10 one.

---

## 6. Open questions for you

1. **Branding**: keep `GrayRavens-Zenithed-V14-Yukari` name or bump to V15?
2. **MGLRU**: enable anon+file by default (`LRU_GEN_ENABLED=y`), or leave the runtime switch?
3. **Scheduler scope**: port the full zenith/hikari/selene stack as-is, or trim anything?
4. **KMI**: should the 5.15 port keep the same "revert-to-preserve-KMI" discipline (mld RCU etc.)?
