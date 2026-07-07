/**
 * Copyright (c) 2023 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_ARCH_ARM_BARRIER_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_BARRIER_H_

#ifndef ZEPHYR_INCLUDE_SYS_BARRIER_H_
#error Please include <zephyr/sys/barrier.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The barrier instructions below are emitted as native inline assembly
 * instead of the CMSIS-Core __DMB()/__DSB()/__ISB() intrinsics, so that this
 * header (which is in the kernel's architecture-portable include graph) does
 * not pull the vendor CMSIS/HAL headers in via <cmsis_core.h>.
 *
 * Each replacement reproduces the corresponding CMSIS-Core GCC intrinsic
 * exactly (same instruction and full-system "0xF"/SY option, "volatile"
 * qualifier and "memory" clobber), so both the semantics and the generated
 * code are unchanged. The DMB/DSB/ISB instructions and their SY (0b1111)
 * option are architectural: ARMv7-M ARM (DDI 0403) A7.7.33-34/A7.7.47,
 * ARMv7-A/R ARM (DDI 0406) A8.8.43-44/A8.8.53.
 */

static ALWAYS_INLINE void z_barrier_sync_synchronize(void)
{
	__sync_synchronize();
}

/* Data Memory Barrier, full system: equivalent of CMSIS __DMB() */
static ALWAYS_INLINE void z_barrier_dmem_fence_full(void)
{
	__asm__ volatile("dmb 0xF" ::: "memory");
}

/* Data Synchronization Barrier, full system: equivalent of CMSIS __DSB() */
static ALWAYS_INLINE void z_barrier_dsync_fence_full(void)
{
	__asm__ volatile("dsb 0xF" ::: "memory");
}

/* Instruction Synchronization Barrier: equivalent of CMSIS __ISB() */
static ALWAYS_INLINE void z_barrier_isync_fence_full(void)
{
	__asm__ volatile("isb 0xF" ::: "memory");
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_BARRIER_H_ */
