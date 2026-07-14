# Why "sem.c only needs a dozen includes" is too naive

Companion note to `kernel-header-reach-analysis.md`, distilled from the
symbol-level attribution of `kernel/sem.c` (see §2/§3 there; data:
`include_attribution.py` over the LLVM-variant build of
`hello_world` @ `nucleo_g474re`).

## The seductive observation

Per-symbol attribution shows that sem.c's entire external symbol diet comes
from **14 headers**, and its consumption of the `<zephyr/kernel.h>` aggregate
is exactly **one type and one macro**: `struct k_sem` (three field accesses)
and `K_NO_WAIT`. Everything else arrives from narrow headers: `spinlock.h`
(25 uses), `ksched.h`/`wait_q.h` (18), `sys/clock.h` (7),
`tracing/tracing_macros.h` (8), `sys/internal/kobject_internal.h`, picolibc
`errno.h`, assertion macros. So: give sem.c a sliced `kernel/k_sem.h` and a
dozen direct includes, and it's done — right?

## The fallacy

sem.c *already* has only 10 direct includes — and still transitively reaches
168 headers. The naive count conflates two different levels: the **symbol
diet** (which headers declare what sem.c uses) and the **transitive reach**
(what those headers need in order to exist). The 168 was never caused by
sem.c including too much; it is caused by what its dozen doorways carry.
Five compounding reasons:

### 1. Providers must be self-contained, and their needs dwarf their surface
`spinlock.h` hands sem.c four symbols — and has a transitive closure of 132
headers, because `k_spin_lock()` is an inline function whose body needs
`arch_irq_lock()`, which today means the per-arch `arch.h` world (the
five-edge `arch/cpu.h` funnel of the main analysis). Measured consequence:
applying every include-cleaner verdict across all 31 kernel TUs moves mean
reach only 177 → 167. Minimal *direct* sets are nearly free; the mass sits
inside the doorways.

### 2. Type completeness is viral
sem.c needs `struct k_sem` complete (sizeof, member access) — no forward
declaration suffices. `struct k_sem` embeds `_wait_q_t` → `kernel_structs.h`
→ thread struct → `struct _callee_saved`, a genuinely **arch-defined register
layout** (`arch/structs.h`). Even a perfect `k_sem.h` slice irreducibly needs
a slice of the arch layer. The realistic target is never "zero arch headers";
it is "the 3-header `arch/structs.h` import instead of the 64-header
`arch/cpu.h` funnel".

### 3. Inlining needs definitions, not declarations
A declarations-only diet would minimize reach and destroy the spinlock fast
path: everything the compiler must codegen into `sem.o` — inline function
bodies, macros, constants — must be textually present in the TU. Any
interface/implementation split of the arch layer has to keep definitions
reachable for TUs that need codegen (the existing `arch_inlines.h` mechanism
is the precedent). This puts a hard floor well above 14 headers.

### 4. One config's truth is only a lower bound
The attribution says `internal/syscall_handler.h` provides nothing to sem.c —
because this build has `CONFIG_USERSPACE=n`. With userspace, the syscall
verification blocks light up. With `CONFIG_TRACING`, the
`SYS_PORT_TRACING_*` macros stop expanding to nothing and need real
declarations. SMP adds IPI paths; assertions pull printk machinery. The
honest minimal include set is the **union over supported configurations**;
per-config measurements find waste but cannot serve as the spec.

### 5. Zephyr headers are historically not self-contained
Several headers compile only because something else was included first
(upstream issue #41543's core complaint; enforced today only by the
`kernel_includes.h` "use the aggregate" `#error` guard). Until slices are
made genuinely standalone — each header compiling on its own — "just include
the twelve you need" can fail in include-order-dependent ways.

## The realistic end-state

sem.c's floor is the union of its providers' *properly slimmed* closures:
spinlock-interface + `sys/clock.h` + kobject + private sched headers + a thin
`arch/structs.h`/`arch_inlines` slice + the libc tail — roughly **120–130
headers** (main analysis §6), not 14. And it is reachable only in sequence:
break the `kernel.h ↔ tracing` cycle, narrow the five-edge arch funnel, then
slice — each step shrinking the closures that the same dozen doorways carry.
The `kernel-reach-budget` checker rule ratchets each step down from the
current 210.

## Upstream references

Context for any of this work going upstream; found during the tooling survey
(2026-07-14):

- **RFC [#49578](https://github.com/zephyrproject-rtos/zephyr/issues/49578)**
  — deprecated `<zephyr/zephyr.h>` in favor of `<zephyr/kernel.h>`: the
  community direction is a *consolidated* app-facing kernel umbrella. Any
  slicing must keep `kernel.h` 100%-compatible; slices are for the kernel's
  own TUs (and willing subsystems), not a breaking API change.
- **Issue [#41543](https://github.com/zephyrproject-rtos/zephyr/issues/41543)**
  — "Improve header structure and usage": the open umbrella issue for
  non-self-contained headers and transitive-include reliance; reason 5 above.
  Earlier incarnations:
  [#16426](https://github.com/zephyrproject-rtos/zephyr/issues/16426),
  [#16539](https://github.com/zephyrproject-rtos/zephyr/issues/16539),
  [#12129](https://github.com/zephyrproject-rtos/zephyr/issues/12129).
- **Issue [#19666](https://github.com/zephyrproject-rtos/zephyr/issues/19666)**
  — private/public header boundary; splitting `arch_interface.h` into a
  kernel-to-arch vs public arch interface. This is the shape of cut proposal
  A (the arch/cpu.h funnel) and of reasons 1-3's resolution.
- **RFC [#73114](https://github.com/zephyrproject-rtos/zephyr/issues/73114)**
  — namespacing generated headers under `zephyr/`; interacts with any header
  moves because syscall stubs are generated per-basename into
  `zephyr/syscalls/` (basename-collision constraint for new slices, e.g.
  `include/zephyr/sys/sem.h` already exists).
- **PRs [#88500](https://github.com/zephyrproject-rtos/zephyr/pull/88500),
  [#88490](https://github.com/zephyrproject-rtos/zephyr/pull/88490)** —
  example of how include hygiene actually lands upstream: incremental,
  per-subsystem IWYU-style cleanups, not big-bang restructuring. Zephyr also
  ships opt-in IWYU plumbing (`cmake/sca/iwyu/sca.cmake`,
  `doc/develop/sca/iwyu.rst`) but no CI include gate
  (`scripts/ci/check_compliance.py` has none).
