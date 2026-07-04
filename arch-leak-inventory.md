# Architecture Header Leak Inventory (Zephyr, nucleo_g474re)

Evidence-backed inventory of *architecture header leaks*: places where Zephyr
core/arch headers transitively drag SoC / CMSIS / vendor-HAL headers into kernel
translation units (TUs). Target: `hello_world` on `nucleo_g474re`
(STM32G474, Cortex-M4F, `CONFIG_ARM_MPU=y`, `CONFIG_CPU_HAS_ARM_MPU=y`).

**Branch:** `tskr/safety/claude/task-arch-leak-inventory`
**Analyzed sources:** worktree of `main` (this file's tree). The include-edge CSV
was extracted from the `collab-safety` checkout at `/wrk/z/ws-safety/zephyr`; all
headers cited below were spot-checked byte-for-byte against `main` (see
§7 for the one discrepancy found).

---

## 1. Executive summary

- Restricting attention to what is **reachable from the 31 `zephyr/kernel/*.c`
  TUs**, the *entire* vendor payload — **69 STM32Cube HAL headers, 6 vendor CMSIS
  headers, 1 `zephyr/soc` header, and the 2 `cmsis_6` Zephyr shim headers** — is
  pulled in through **exactly two `core → vendor` include edges**, and both land
  on the same chokepoint node `cmsis_core.h`:

  | # | Including header (core) | Target (vendor) |
  |---|---|---|
  | E1 | `include/zephyr/arch/arm/mpu/arm_mpu_v7m.h:13` | `modules/cmsis_6/cmsis_core.h` |
  | E2 | `include/zephyr/arch/arm/asm_inline_gcc.h:24`   | `modules/cmsis_6/cmsis_core.h` |

- **The two edges are redundant paths to the same subgraph.** Cutting either one
  *alone* removes **0** headers. Cutting **both together** removes **79** nodes
  from kernel-TU reach (325 → 246). This is the single most important finding:
  any remediation that fixes only ARM-MPU (the "known dominant chain") buys
  nothing until `asm_inline_gcc.h` is fixed too.

- **arm_mpu hypothesis: CONFIRMED (with a critical caveat).** `arm_mpu_v7m.h`
  uses CMSIS *only* for compile-time register-field constants
  (`MPU_RASR_*_Pos/_Msk`, `ARM_MPU_REGION_SIZE_*`) — no register structs, no
  intrinsics. A constants-only split is sound. **But** because it is not the only
  path (E2 exists), the constants split alone yields a reach-delta of 0; the
  claimed "~110 headers" only materialise when **both** E1 and E2 are cut, and
  the actual number here is **79** (of which ~75 are the HAL+vendor-CMSIS mass the
  known-facts note cited).

- Realistic net win after redirecting both edges to a device-independent CMSIS
  intrinsics/constants header: **~75 headers gone from every kernel TU** (the 69
  HAL + 6 vendor CMSIS; `arm_acle.h` and 1-2 compiler-intrinsic headers return,
  the `soc.h`/shim nodes stay gone under this config).

---

## 2. Methodology & reproduction

Input: direct-include edge list (`from,to`, absolute paths) for the whole build:
`scratchpad/includes.csv` (1038 edges, 534 distinct nodes).

**Territory definitions used** (matching the task):
- *core*: `zephyr/include/**`, `zephyr/kernel/**`, `zephyr/arch/**`
- *vendor/SoC*: `zephyr/soc/**`, `zephyr/modules/**` (the `cmsis_6` shim),
  `zephyr/drivers/**`, `/wrk/z/ws-safety/modules/hal/**`

**Reachability & reach-delta** — helper scripts in `scratchpad/`
(`analyze.py`, `analyze2.py`): build the directed graph from the CSV, BFS from the
31 kernel TUs to get the reachable set `R`; a *crossing edge* is any `(a,b)` with
`a ∈ R`, `a` in core territory, `b` in vendor territory. Reach-delta of cutting a
set of edges = `|R| − |R'|` where `R'` is the BFS reach with those edges removed.

```
# reproduce the headline numbers
python3 scratchpad/analyze.py    # crossing edges + per-edge delta
python3 scratchpad/analyze2.py   # combined-cut scenarios + importer lists
```

Symbol-level "why" was established by reading each including header on `main` and
grepping the CMSIS/HAL sources under
`/wrk/z/ws-safety/modules/hal/cmsis_6/CMSIS/Core/Include/` for every symbol used.

---

## 3. Crossing-edge table (core → vendor, reachable from kernel TUs)

Only two edges exist. Both are analysed in full.

| Edge | file:line | Target | Classification | Symbols actually used | CONFIG gate | Δreach alone | Δreach in {E1,E2} |
|------|-----------|--------|----------------|------------------------|-------------|:---:|:---:|
| **E1** | `arch/arm/mpu/arm_mpu_v7m.h:13` `#include <cmsis_core.h>` | `cmsis_core.h` | **constants-only** (splittable) | `MPU_RASR_AP_Pos/_Msk`, `MPU_RASR_XN_Msk`, `MPU_RASR_S/B/C/TEX/SRD/SIZE_*_Pos/_Msk`, `ARM_MPU_REGION_SIZE_*` — all `#define` constants | reached via `arch.h:281` under `CONFIG_ARM_MPU && CONFIG_CPU_HAS_ARM_MPU` | 0 | **79** |
| **E2** | `arch/arm/asm_inline_gcc.h:24` `#include <cmsis_core.h>` | `cmsis_core.h` | **register-access / intrinsics** (satisfiable from device-independent CMSIS) | `__get_PRIMASK`, `__set_PRIMASK`, `__disable_irq`, `__enable_irq`, `__get_BASEPRI`, `__set_BASEPRI`, `__set_BASEPRI_MAX`, `__ISB` | effectively unconditional (pulled by `asm_inline.h` ← `arch/cpu.h` ← `kernel_includes.h`) | 0 | **79** |

**Importer chains** (from `analyze2.py`, all nodes ∈ `R`):
```
cmsis_core.h  <-  arm_mpu_v7m.h  <-  arm_mpu.h  <-  arch.h (arm)
cmsis_core.h  <-  asm_inline_gcc.h  <-  asm_inline.h
```

### The amplifier edge (inside vendor territory, but the actual funnel)

Not a `core→vendor` edge by the territory rule (source is the `cmsis_6` shim), but
it is *why* one `cmsis_core.h` include explodes into 75+ headers:

- `modules/cmsis_6/cmsis_core.h:10` → `cmsis_core_m.h` (gated `CONFIG_CPU_CORTEX_M`)
- **`modules/cmsis_6/cmsis_core_m.h:24` `#include <soc.h>`** → `soc/st/stm32/stm32g4x/soc.h`
- `soc/st/stm32/stm32g4x/soc.h` → `modules/hal/.../soc/stm32g4xx.h` (HAL device umbrella)
- `stm32g4xx.h` → **`stm32g4xx_hal.h`** (the *entire* HAL, ~69 headers) **and**
  `stm32g474xx.h` → `core_cm4.h` (the CMSIS core that actually defines the symbols
  E1/E2 want).

Classification of `cmsis_core_m.h:24`: **config-check / convention include.** `soc.h`
is pulled so the CMSIS device-parameter macros (`__NVIC_PRIO_BITS`, `__MPU_PRESENT`,
`__FPU_PRESENT`, …) are defined before the consistency `#error` checks at
`cmsis_core_m.h:26-69` and before `core_cm4.h` is (transitively) configured. The
device header is genuinely required by CMSIS's model to *use registers/intrinsics*,
but **not** to obtain the *constants* E1 needs.

---

## 4. Per-leak remediation & quantified payoff

### R1 — Redirect `asm_inline_gcc.h` to device-independent CMSIS intrinsics (fixes E2)

`asm_inline_gcc.h:44-147` needs only Cortex-M intrinsics
(`__get_PRIMASK`/`__set_BASEPRI`/`__ISB`/…). These are defined in
`m-profile/cmsis_gcc_m.h`, reached through the **device-independent**
`cmsis_compiler.h` (`cmsis_compiler.h:26` needs only `<stdint.h>`;
`cmsis_compiler.h:57` → `cmsis_gcc.h:30,1019` → `arm_acle.h` + `cmsis_gcc_m.h`).
No `soc.h`, no device header, no HAL.

**Fix:** replace `#include <cmsis_core.h>` at `asm_inline_gcc.h:24` with an include
of a Zephyr-owned intrinsics wrapper (or `<cmsis_compiler.h>` directly). Push the
full-`cmsis_core.h` need (if any) down; there is none in this file.
**Risk:** low-medium — must confirm every ARM variant path (`CONFIG_ARMV6_M...`,
`CONFIG_ARMV7_R`, `_A`) still resolves its intrinsics from the compiler header.

### R2 — Constants-only split of `arm_mpu_v7m.h` (fixes E1)

`arm_mpu_v7m.h` uses CMSIS purely for constants (see §5). Two sub-actions:

- **R2a:** introduce a small Zephyr-owned header (e.g.
  `arch/arm/mpu/arm_mpu_v7m_regs.h`) defining the ~12 `MPU_RASR_*_Pos/_Msk`
  architectural constants and the `ARM_MPU_REGION_SIZE_*` values (these are fixed
  ARMv7-M/PMSAv7 architecture constants, identical across `core_cm3/cm4/cm7/…`,
  cf. `core_cm4.h:1266,1269,1286` and `m-profile/armv7m_mpu.h:32`). Replace
  `arm_mpu_v7m.h:13` `#include <cmsis_core.h>` with it.
- **R2b:** the MPU *register users* — `arch/arm/core/mpu/arm_mpu.c:297,299,316`
  (`MPU->CTRL…`), `arm_mpu_v7_internal.h:47,49`, `cortex_m/arm_mpu_internal.h:13,22,27`
  (`MPU->RBAR/RASR/RNR/TYPE`) — currently obtain `cmsis_core.h` *transitively via*
  `arm_mpu.h → arm_mpu_v7m.h`. After R2a they must **`#include <cmsis_core.h>`
  directly** in those impl files (they legitimately need `MPU_Type`/`MPU_BASE`,
  `core_cm4.h:1223,1569-1570`). This is a "push include down into the driver"
  change; these files are *not* kernel TUs, so it does not re-introduce the leak.

**Risk:** low — mechanical; constants are stable. The only trap is forgetting R2b
(would break the MPU driver build).

### R3 (structural, optional) — narrow `cmsis_core_m.h`'s `soc.h` include

If R1+R2 land, no kernel-reachable core header pulls `cmsis_core.h` for
*constants/intrinsics* anymore, so the `cmsis_core_m.h:24 → soc.h` funnel only
fires for TUs that genuinely need full device/HAL access (SoC init, drivers).
No change strictly required, but documenting the "`soc.h` convention" (a CMSIS
device-parameterisation requirement, not a Zephyr accident) prevents regressions.

### Payoff table

| Action | Edges cut | Δreach (kernel TUs) | Notes |
|--------|-----------|:---:|-------|
| R2 only | E1 | **0** | redundant with E2 — no win alone |
| R1 only | E2 | **0** | redundant with E1 — no win alone |
| **R1 + R2** | E1 + E2 | **79** (325→246) | 69 HAL + 6 vendor CMSIS + 2 shim + 1 soc + 1 `arm_acle.h` |
| R1 + R2, net after re-adding device-indep CMSIS | E1+E2 | **~75** | `arm_acle.h`/compiler-intrinsic headers return; HAL+vendor-CMSIS stay gone |

---

## 5. arm_mpu hypothesis — verdict & evidence

**Verdict: CONFIRMED that the include is constants-only; REFUTED that fixing it
alone pays off.**

Every CMSIS symbol referenced by `arm_mpu_v7m.h` is a compile-time constant:

- `MPU_RASR_AP_Pos/_Msk`, `MPU_RASR_XN_Msk`, `MPU_RASR_S/B/C_Msk`,
  `MPU_RASR_TEX_Pos`, `MPU_RASR_SRD_Pos/_Msk`, `MPU_RASR_SIZE_Pos/_Msk` — used at
  `arm_mpu_v7m.h:22-77,123-128,225-228`; **all** are `#define` bit constants, e.g.
  `core_cm4.h:1266` `MPU_RASR_XN_Msk (1UL << MPU_RASR_XN_Pos)`,
  `core_cm4.h:1269` `MPU_RASR_AP_Msk (0x7UL << MPU_RASR_AP_Pos)`,
  `core_cm4.h:1286` `MPU_RASR_SIZE_Pos 1U`.
- `ARM_MPU_REGION_SIZE_##size` — used at `arm_mpu_v7m.h:77` (`REGION_SIZE`); defined
  as constants in `m-profile/armv7m_mpu.h:32` (`ARM_MPU_REGION_SIZE_32B ((uint8_t)0x04U)`).

No `MPU_Type`, no `MPU->…`, no `MPU_BASE`, no intrinsic function appears in
`arm_mpu_v7m.h`. The register-struct/`MPU->` accesses live exclusively in the MPU
*driver* under `arch/arm/core/mpu/` (`arm_mpu.c:297`, `arm_mpu_v7_internal.h:47`,
`cortex_m/arm_mpu_internal.h:13`) — i.e. in `.c`-side impl, not the kernel-facing
public header. This is exactly the split the hypothesis predicted.

**Kernel-facing consumers of `arm_mpu.h`** need only: the alignment/guard macros
`CONFIG_ARM_MPU_REGION_MIN_ALIGN_AND_SIZE` (a Kconfig, `arch.h:149,205,263`), the
types `arm_mpu_region_attr_t` / `k_mem_partition_attr_t` / `struct arm_mpu_config`
/ `struct z_mpu_context_retained` (`arm_mpu_v7m.h:145,148`, `arm_mpu.h:53,85,98`),
and the `K_MEM_PARTITION_*` macros. The *only* kernel-tree `.c` that instantiates
these is `kernel/userspace/mem_domain.c` — which is **not** in the `hello_world`
kernel-TU set (`CONFIG_USERSPACE`-gated). So for this config the whole
`arm_mpu.h → …→ cmsis_core.h` chain is pulled purely to make types/macros
*available*, none are actually exercised by a compiled kernel TU.

**Caveat (the refutation):** the hypothesis' "~110 headers cut" assumed ARM-MPU was
the sole path. It is not — `asm_inline_gcc.h` (E2) independently reaches the same
`cmsis_core.h`. Measured reach-delta of R2 alone = **0**; only R1+R2 gives 79 (~75
net). The constants split is *necessary but not sufficient*.

---

## 6. Structural observations affecting feasibility

- **`kernel.h ↔ tracing ↔ tracking` include cycle (confirmed):**
  `include/zephyr/kernel.h → tracing/tracing.h`, `tracing/tracing.h → tracing/tracking.h`,
  and `tracing/tracking.h → include/zephyr/kernel.h` form a 3-node cycle (CSV
  edges). Guard-protected but it means `kernel.h` is not a clean DAG root; any
  reasoning about "what `kernel.h` pulls" must treat this SCC as a unit.
- **STM32 HAL internal cycle (confirmed):** `stm32g4xx.h → stm32g4xx_hal.h → … →
  stm32g4xx_hal_def.h → stm32g4xx.h` (5 HAL nodes include `stm32g4xx.h` back).
  Vendor code, guard-protected, not editable in Zephyr — reinforces that the fix
  must be to *not enter* this subgraph, not to prune inside it.
- **`cmsis_core_m.h`'s `soc.h` convention** (`cmsis_core_m.h:24`): the shim pulls
  the SoC device header so CMSIS device-parameter macros exist for the
  consistency `#error` checks (`cmsis_core_m.h:26-69`) and to parameterise
  `core_cm4.h`. This is intrinsic to the CMSIS model — the amplification from "1
  include" to "75 headers" happens *here*, so the leverage is upstream, at E1/E2,
  keeping core headers from ever needing `cmsis_core.h`.
- **Single chokepoint:** because both leaks converge on `cmsis_core.h`, the graph
  is unusually forgiving — two small, local header edits (R1, R2) sever the whole
  vendor mass. No kernel `.c` edits are required.

---

## 7. collab-safety vs main discrepancies

Diffed the six headers central to this analysis (`arm_mpu_v7m.h`, `arm_mpu.h`,
`asm_inline_gcc.h`, `cmsis_core.h`, `cmsis_core_m.h`, `arch.h`) between the CSV
source tree (`/wrk/z/ws-safety/zephyr`, collab-safety) and this `main` worktree:

- **`arm_mpu_v7m.h`: DIFFERS** — collab-safety's copy is **missing the
  `ZEPHYR_INCLUDE_ARCH_ARM_MPU_ARM_MPU_V7M_H_` include guard** that `main` has
  (guard added at `main` lines 8-10 and 300-301). The `#include <cmsis_core.h>`
  line and every CMSIS symbol use are identical, so the leak analysis is
  unaffected; the missing guard on collab-safety is a latent double-include hazard
  worth flagging separately.
- All other five headers: **byte-identical** on both trees.

Reachability note: the CSV yields 325 kernel-TU-reachable nodes / 69 STM32 HAL,
vs. the "~236 headers / ~75 HAL" figure in the task's known-facts. The ~75 HAL
figure matches once the vendor-CMSIS headers are folded in; the total differs
because the CSV is a whole-build edge list (per-TU counts vary). The **relative**
result — one chokepoint, two edges, ~75-header payoff — is robust to that.
