/* ARM AArch32 GCC specific public inline assembler functions and macros */

/*
 * Copyright (c) 2015, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Either public functions or macros or invoked by public functions */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_GCC_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_GCC_H_

/*
 * The file must not be included directly
 * Include arch/cpu.h instead
 */

#ifndef _ASMLANGUAGE

#include <zephyr/toolchain.h>
#include <zephyr/types.h>
#include <zephyr/arch/exception.h>

#if defined(CONFIG_CPU_AARCH32_CORTEX_R) || defined(CONFIG_CPU_AARCH32_CORTEX_A)
#include <zephyr/arch/arm/cortex_a_r/cpu.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Native GCC inline-assembly replacements for the small set of CMSIS-Core
 * intrinsics this header used to rely on. Providing them locally lets the ARM
 * arch layer drop its <cmsis_core.h> dependency, keeping the kernel's
 * architecture-portable include graph free of vendor CMSIS/HAL headers.
 *
 * Each helper reproduces, byte-for-byte, the instruction, operand constraints,
 * "volatile" qualifier and clobber list of the corresponding CMSIS-Core
 * intrinsic (CMSIS-Core cmsis_gcc.h and m-profile/cmsis_gcc_m.h), so the code
 * generated at each call site is identical to the previous CMSIS-based one.
 * In particular the PRIMASK/BASEPRI *readers* deliberately have NO "memory"
 * clobber, matching CMSIS __get_PRIMASK()/__get_BASEPRI(). The names are
 * deliberately NOT the CMSIS ones (no __get_PRIMASK() etc.) to avoid macro/
 * symbol redefinition clashes in translation units that also include CMSIS.
 */

/* Equivalent of CMSIS __get_PRIMASK(): reads PRIMASK, no memory clobber. */
static ALWAYS_INLINE uint32_t z_arm_get_primask(void)
{
	uint32_t result;

	__asm__ volatile("MRS %0, primask" : "=r"(result));
	return result;
}

/* Equivalent of CMSIS __set_PRIMASK(): writes PRIMASK, memory clobber. */
static ALWAYS_INLINE void z_arm_set_primask(uint32_t primask)
{
	__asm__ volatile("MSR primask, %0" : : "r"(primask) : "memory");
}

/* Equivalent of CMSIS __enable_irq(): clears PRIMASK, memory clobber. */
static ALWAYS_INLINE void z_arm_enable_irq(void)
{
	__asm__ volatile("cpsie i" : : : "memory");
}

/* Equivalent of CMSIS __disable_irq(): sets PRIMASK, memory clobber. */
static ALWAYS_INLINE void z_arm_disable_irq(void)
{
	__asm__ volatile("cpsid i" : : : "memory");
}

/* Equivalent of CMSIS __ISB(): Instruction Synchronization Barrier. */
static ALWAYS_INLINE void z_arm_isb(void)
{
	__asm__ volatile("isb 0xF" : : : "memory");
}

#if defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
/* BASEPRI only exists on ARMv7-M / ARMv8-M Mainline. */

/* Equivalent of CMSIS __get_BASEPRI(): reads BASEPRI, no memory clobber. */
static ALWAYS_INLINE uint32_t z_arm_get_basepri(void)
{
	uint32_t result;

	__asm__ volatile("MRS %0, basepri" : "=r"(result));
	return result;
}

/* Equivalent of CMSIS __set_BASEPRI(): writes BASEPRI, memory clobber. */
static ALWAYS_INLINE void z_arm_set_basepri(uint32_t basepri)
{
	__asm__ volatile("MSR basepri, %0" : : "r"(basepri) : "memory");
}

/* Equivalent of CMSIS __set_BASEPRI_MAX(): writes BASEPRI_MAX, memory clobber. */
static ALWAYS_INLINE void z_arm_set_basepri_max(uint32_t basepri)
{
	__asm__ volatile("MSR basepri_max, %0" : : "r"(basepri) : "memory");
}
#endif /* CONFIG_ARMV7_M_ARMV8_M_MAINLINE */

/* On ARMv7-M and ARMv8-M Mainline CPUs, this function prevents regular
 * exceptions (i.e. with interrupt priority lower than or equal to
 * _EXC_IRQ_DEFAULT_PRIO) from interrupting the CPU. NMI, Faults, SVC,
 * and Zero Latency IRQs (if supported) may still interrupt the CPU.
 *
 * On ARMv6-M and ARMv8-M Baseline CPUs, this function reads the value of
 * PRIMASK which shows if interrupts are enabled, then disables all interrupts
 * except NMI.
 */

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	unsigned int key;

#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
#if CONFIG_MP_MAX_NUM_CPUS == 1 || defined(CONFIG_ARMV8_M_BASELINE)
	key = z_arm_get_primask();
	z_arm_disable_irq();
#else
#error "Cortex-M0 and Cortex-M0+ require SoC specific support for cross core synchronisation."
#endif
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	key = z_arm_get_basepri();
	z_arm_set_basepri_max(_EXC_IRQ_DEFAULT_PRIO);
	z_arm_isb();
#elif defined(CONFIG_ARMV7_R) || defined(CONFIG_AARCH32_ARMV8_R) \
	|| defined(CONFIG_ARMV7_A) || defined(CONFIG_AARCH32_ARMV8_A)
	__asm__ volatile(
		"mrs %0, cpsr;"
		"and %0, #" STRINGIFY(I_BIT) ";"
		"cpsid i;"
		: "=r" (key)
		:
		: "memory", "cc");
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */

	return key;
}


/* On Cortex-M0/M0+, this enables all interrupts if they were not
 * previously disabled.
 */

static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	if (key != 0U) {
		return;
	}
	z_arm_enable_irq();
	z_arm_isb();
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	z_arm_set_basepri(key);
	z_arm_isb();
#elif defined(CONFIG_ARMV7_R) || defined(CONFIG_AARCH32_ARMV8_R) \
	|| defined(CONFIG_ARMV7_A) || defined(CONFIG_AARCH32_ARMV8_A)
	if (key != 0U) {
		return;
	}
	z_arm_enable_irq();
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	/* This convention works for both PRIMASK and BASEPRI */
	return key == 0U;
}

/** Implementation of @ref arch_cpu_irqs_are_enabled. */
static ALWAYS_INLINE bool arch_cpu_irqs_are_enabled(void)
{
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	return z_arm_get_primask() == 0U;
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	return z_arm_get_basepri() == 0U;
#elif defined(CONFIG_ARMV7_R) || defined(CONFIG_AARCH32_ARMV8_R) \
	|| defined(CONFIG_ARMV7_A) || defined(CONFIG_AARCH32_ARMV8_A)
	unsigned int cpsr;

	__asm__ volatile("mrs %0, cpsr" : "=r" (cpsr));
	return (cpsr & I_BIT) == 0U;
#else
#error Unknown ARM architecture
#endif
}

#ifdef CONFIG_ZERO_LATENCY_IRQS

static ALWAYS_INLINE unsigned int arch_zli_lock(void)
{
	unsigned int key;

	key = z_arm_get_primask();

	/*
	 * The cpsid instruction is self synchronizing within the instruction stream, no need for
	 * an explicit ISB.
	 */
	z_arm_disable_irq();

	return key;
}

static ALWAYS_INLINE void arch_zli_unlock(unsigned int key)
{
	z_arm_set_primask(key);
	z_arm_isb();
}

#endif /* CONFIG_ZERO_LATENCY_IRQS */

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_GCC_H_ */
