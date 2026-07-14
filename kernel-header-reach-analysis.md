# Kernel TU header reach: symbol-level analysis and cut proposals

Status: analysis only — no header restructuring is part of this work package.
Companion to `arch-leak-inventory.md` (the CMSIS edge cut). All numbers are
reproducible; every section lists its command.

**Substrates.** Two builds of `samples/hello_world` @ `nucleo_g474re` from this
branch are used deliberately:

- *Enforcement substrate* (gcc, `west build -b nucleo_g474re`): the production
  toolchain. All gate numbers (reach budget 210, `check_include_layering.py`)
  are defined here.
- *Analysis substrate* (SDK LLVM, `west build ... -- -DTOOLCHAIN_VARIANT_COMPILER=llvm`,
  post-processed with `extra/dep-analyzer/clang_compile_db.py`): what the
  clang-based symbol tooling (`include_attribution.py`, clang-include-cleaner,
  libclang) parses. Its counts differ slightly (clang builtin headers,
  `toolchain/llvm.h` tail) but the zephyr-owned include *structure* is
  identical.

All symbol-level results are config-relative (this board, this Kconfig):
"unused" may mean "unused when `CONFIG_USERSPACE=n`". Treat per-config evidence
as a lower bound on need, not global truth.

## 1. Where reach stands (gcc substrate)

Per-TU transitive header count across the 31 kernel TUs: max **204**
(`kernel/init.c`), then thread.c 187, sched.c 186; `sem.c` 168; floor of the
distribution ~160. Locked by the `kernel-reach-budget` rule
(`scripts/include_layers.yml`, budget 210, one violation per over-budget TU).

```sh
west build -p always -b nucleo_g474re -d build-gcc zephyr/samples/hello_world
python3 zephyr/scripts/check_include_layering.py build-gcc          # gates
```

## 2. TU-level include hygiene is NOT the lever

`include_attribution.py` (libclang walk + clang-include-cleaner) over all 31
kernel TUs: applying every per-TU verdict — drop unused direct includes, add
the missing direct ones — moves mean reach only **177 → 167** (clang
substrate). Reason: every TU's minimal set legitimately keeps
`<zephyr/kernel.h>` (they all use `k_*` APIs), and kernel.h's own closure IS
the universe. Cleanups worth doing opportunistically (e.g. `sem.c`: 5 of its
10 direct includes provide no used symbol in this config), but the yield is
~6%.

```sh
west build -p always -b nucleo_g474re -d build-llvm zephyr/samples/hello_world \
    -- -DTOOLCHAIN_VARIANT_COMPILER=llvm
python3 extra/dep-analyzer/clang_compile_db.py build-llvm -o clangdb \
    --only "$PWD/zephyr/kernel/" --verify
/usr/bin/python3 extra/dep-analyzer/include_attribution.py clangdb \
    --only "$PWD/zephyr/kernel/" --json attribution.json
```

## 3. The aggregate is where the mass is — member-level verdicts

`include_attribution.py` classifies every direct member of `kernel.h` +
`kernel_includes.h` by whether its transitive closure delivers symbols the
TUs actually use, and whether it is the *only* member delivering them:

| verdict | members | meaning |
|---|---|---|
| CYCLIC | `tracing/tracing.h` | closure re-enters kernel.h — see §4 |
| exclusive (load-bearing) | `sys_clock.h` (20 TUs), `spinlock.h` (19), `tracing/tracing_macros.h` (18), `sys/kobject.h` (10), generated `syscalls/kernel.h` (8), `kernel/thread_stack.h` (3), `irq.h` (2), `sys/ring_buffer.h` (1), `sys/sflist.h` (1) | ≥1 used symbol arrives ONLY via this member |
| used, never exclusive | 21 members incl. **`arch/cpu.h`**, `kernel_structs.h`, `sys/atomic.h`, `fatal.h`, `app_memory/mem_domain.h`, `kernel/thread.h`, `sys/util.h`, … | every symbol they deliver also arrives via another member (redundant paths) |
| never used | `kernel_version.h`, generated `heap_constants.h`, `stdbool.h`, `stddef.h` (as members — obviously used directly elsewhere) | provide no used symbol to any of the 31 TUs |

The headline: **`arch/cpu.h` — the 64-header subtree carrying all 22
devicetree headers — is exclusive for zero TUs.** Kernel code needs *symbols
that happen to live behind it* (arch_irq_lock, per-CPU structs), but every one
of those also arrives through other members, because several members
themselves funnel into the arch subtree.

## 4. Preconditions: the cycles

The kernel header graph is not a DAG (5 cyclic SCCs; command below):

1. **`kernel.h → tracing/tracing.h → tracing/tracking.h → kernel.h`** — the
   big one. It makes tracing.h's closure identical to kernel.h's and defeats
   any member-level slicing arithmetic until broken. The kernel TUs' real
   tracing need is `tracing_macros.h` (exclusive for 18 TUs); `tracking.h`'s
   back-include of kernel.h is a layering bug on its own.
2. `arch/arch_interface.h ↔ arch/cpu.h`
3. `kernel/include/kernel_internal.h ↔ kernel/include/kthread.h` (private)
4. `sys/time_units.h ↔ sys/util.h`
5. `linker/section_tags.h ↔ linker/sections.h`

```sh
python3 extra/dep-analyzer/include_graph.py --cycles clangdb
```

## 5. The arch funnel (cut proposal A — highest leverage)

Direct fan-in from non-arch core headers into the public arch tree (true
direct-include edges; the `-H`-based graph under-reports these — use
`direct_include_edges.py`):

```
kernel_includes.h        -> arch/cpu.h        (the "official" edge)
fatal.h                  -> arch/cpu.h  + arch/exception.h
irq.h                    -> arch/cpu.h
spinlock.h               -> arch/cpu.h
sys/cbprintf_internal.h  -> arch/cpu.h
kernel_structs.h         -> arch/structs.h    (narrow — fine)
syscall.h                -> arch/syscall.h    (narrow — fine)
sys/barrier.h            -> arch/arm/barrier.h (narrow — fine)
timing/timing.h          -> arch/arch_interface.h
```

Same lesson as the CMSIS cut: cutting one edge removes nothing; the five
`arch/cpu.h` edges must go together. What `spinlock.h`/`irq.h`/`fatal.h`
actually consume is the *arch interface* (`arch_irq_lock()`, exception types),
not the whole per-arch `arch.h` (which is what includes `devicetree.h` at
`arch/arm/arch.h:20` and drags `device.h` via `sw_isr_table.h:18`). The shape
of the fix is upstream issue #19666's direction: split
`arch/arch_interface.h` into a declarations-only interface header that does
not pull the per-arch `arch.h`/devicetree world, and point the five funnel
edges at it. Expected yield (gcc substrate): the arch subtree minus the narrow
imports ≈ **25–30 headers off every kernel TU**, including all 22 devicetree
headers. Risks: `arch_interface.h ↔ arch/cpu.h` cycle (§4.2) must be broken
first; inline arch functions (spinlock fast path) must stay inlinable —
the declarations/definitions split has to keep the definitions reachable for
TUs that need codegen, e.g. via the existing `arch_inlines.h` mechanism.

## 6. kernel.h slicing (cut proposal B — after A)

With the tracing cycle broken and the arch funnel narrowed, the member table
in §3 is the slicing map:

- The load-bearing core every kernel TU needs is small: `sys_clock.h`,
  `spinlock.h`, `tracing_macros.h`, `sys/kobject.h`, the generated syscall
  stub, plus the struct definitions (`kernel_structs.h`).
- API-family slices under the existing `include/zephyr/kernel/` namespace
  (precedent: `kernel/thread.h`, `kernel/obj_core.h` already exist and are
  included by `kernel_includes.h`): `kernel/sem.h`, `kernel/mutex.h`,
  `kernel/queue.h`, `kernel/work.h`, … each carrying its `__syscall` decls and
  its own trailing `#include <zephyr/syscalls/<basename>.h>` stub.
  Constraint: stubs are keyed by basename (`parse_syscalls.py`), and
  `include/zephyr/sys/sem.h` already exists — slice basenames must not
  collide (e.g. use `kernel/k_sem.h`-style names or verify uniqueness).
- `kernel.h` remains a 100%-compatible umbrella including all slices
  (community direction, RFC #49578); kernel *TUs* migrate to slices, and the
  `kernel_includes.h:16` `#error` guard is relaxed for the slice namespace.
- Immediate deletions from the aggregate for kernel-TU purposes: the
  never-used members (§3), e.g. `kernel_version.h` (version.c includes it
  directly — the only consumer).

Yield estimate: for a leaf TU like `sem.c`, the §2 floor (~167) minus the
arch subtree (§5, ~25–30) minus never-used/redundant members it doesn't need
lands the theoretical floor near **120–130**; TUs like `init.c`/`sched.c`
stay higher (they genuinely consume more). Refine against the ratchet as cuts
land — drop the budget from 210 in steps.

## 7. Long tail (cut proposal C — opportunistic)

- `sys/atomic.h` fans out to 15 headers (`atomic_builtin.h` etc.); used by 30
  TUs but never exclusively — its payload arrives redundantly. A
  declarations-only atomic slice is possible but low priority.
- Toolchain/libc tail (25 headers, gcc substrate) is irreducible without libc
  surgery — out of scope.
- Private `kernel/include/*` headers (8) are already narrow; the
  `kernel_internal.h ↔ kthread.h` cycle (§4.3) is worth breaking for hygiene.

## 8. Verification discipline for future cuts

Every cut lands against the gates: `check_include_layering.py` green
(kernel-no-hal at 0, reach budget ratcheted down), `.text`-identical binary
comparison before committing (as in the CMSIS cut), userspace/MPU + cross-SoC
smoke builds. The attribution tooling reruns in minutes; regenerate
`attribution.json` after each cut and update §3's table.
