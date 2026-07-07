/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Stack helpers for Cortex-M CPUs
 *
 * Stack helper functions.
 */

#ifndef ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_CORTEX_M_STACK_H_
#define ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_CORTEX_M_STACK_H_

#ifdef _ASMLANGUAGE

/* nothing */

#else

/*
 * This helper is always-inlined into arch_kernel_init() and thus compiled
 * into kernel translation units: use the Zephyr-owned Cortex-M register
 * definitions instead of <cmsis_core.h> to keep the kernel's
 * architecture-portable include graph free of vendor CMSIS/HAL headers.
 */
#include <cortex_m/scs.h>

#ifdef __cplusplus
extern "C" {
#endif

K_KERNEL_STACK_ARRAY_DECLARE(z_interrupt_stacks, CONFIG_MP_MAX_NUM_CPUS, CONFIG_ISR_STACK_SIZE);

/**
 *
 * @brief Setup interrupt stack
 *
 * On Cortex-M, the interrupt stack is registered in the MSP (main stack
 * pointer) register, and switched to automatically when taking an exception.
 *
 */
static ALWAYS_INLINE void z_arm_interrupt_stack_setup(void)
{
	uint32_t msp = (uint32_t)(K_KERNEL_STACK_BUFFER(z_interrupt_stacks[0])) +
		       K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[0]);

	z_arm_set_msp(msp);
#if defined(CONFIG_BUILTIN_STACK_GUARD)
#if defined(CONFIG_CPU_CORTEX_M_HAS_SPLIM)
	z_arm_set_msplim((uint32_t)z_interrupt_stacks[0]);
#else
#error "Built-in MSP limit checks not supported by HW"
#endif
#endif /* CONFIG_BUILTIN_STACK_GUARD */

#if defined(CONFIG_STACK_ALIGN_DOUBLE_WORD)
	/* Enforce double-word stack alignment on exception entry
	 * for Cortex-M3 and Cortex-M4 (ARMv7-M) MCUs. For the rest
	 * of ARM Cortex-M processors this setting is enforced by
	 * default and it is not configurable.
	 */
#if defined(CONFIG_CPU_CORTEX_M3) || defined(CONFIG_CPU_CORTEX_M4)
	Z_ARM_SCB->ccr |= Z_ARM_SCB_CCR_STKALIGN_Msk;
#endif
#endif /* CONFIG_STACK_ALIGN_DOUBLE_WORD */
}

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_ARM_INCLUDE_AARCH32_CORTEX_M_STACK_H_ */
