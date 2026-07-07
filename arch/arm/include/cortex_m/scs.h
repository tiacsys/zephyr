/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Zephyr-owned Cortex-M System Control Space register definitions
 *
 * Definitions for the small subset of the ARMv6-M/ARMv7-M/ARMv8-M System
 * Control Space (SCS) registers, and of the CPU special registers, that the
 * (arch-portable) kernel include graph needs: the always-inline helpers in
 * <cortex_m/stack.h>, <cortex_m/exception.h> and <cortex_m/kernel_arch_func.h>
 * are compiled into kernel translation units, so they must not rely on the
 * vendor CMSIS/HAL headers (<cmsis_core.h>).
 *
 * All register layouts and field encodings below are architectural, taken
 * from the "ARMv7-M Architecture Reference Manual" (ARM DDI 0403, section
 * B3.2 "System Control Space") and the "ARMv8-M Architecture Reference
 * Manual" (ARM DDI 0553, section D1.2), and are identical to the
 * corresponding CMSIS-Core definitions. The names are deliberately NOT the
 * CMSIS ones (no SCB, SCB_*_Msk, __set_MSP(), ...) so that translation units
 * which legitimately include both this header and <cmsis_core.h> (arch/SoC
 * code) never see conflicting macro redefinitions.
 *
 * The inline-assembly helpers reproduce, byte-for-byte, the instruction,
 * operand constraints, "volatile" qualifier and clobber list of the
 * corresponding CMSIS-Core GCC intrinsics, so the code generated at each
 * call site is identical to the previous CMSIS-based one.
 */

#ifndef ZEPHYR_ARCH_ARM_INCLUDE_CORTEX_M_SCS_H_
#define ZEPHYR_ARCH_ARM_INCLUDE_CORTEX_M_SCS_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>
#include <zephyr/toolchain.h>
#include <zephyr/arch/arm/cortex_m/nvic.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * System Control Block (SCB) register block, at 0xE000ED00 in the SCS
 * (ARM DDI 0403 B3.2.2, table B3-4; ARM DDI 0553 D1.2).
 *
 * Only the registers up to and including BFAR are described; the kernel
 * include graph does not need anything beyond them. On ARMv6-M and ARMv8-M
 * Baseline the System Handler Priority Registers support word access only
 * and SHPR1 is not implemented (ARM DDI 0419 B3.2.10-B3.2.12), whereas on
 * ARMv7-M/ARMv8-M Mainline they are byte-accessible (SHPR1-SHPR3,
 * ARM DDI 0403 B3.2.10-B3.2.12).
 */
struct z_arm_scb {
	volatile uint32_t cpuid; /* 0xE000ED00: CPUID Base Register (RO) */
	volatile uint32_t icsr;  /* 0xE000ED04: Interrupt Control and State Register */
	volatile uint32_t vtor;  /* 0xE000ED08: Vector Table Offset Register */
	volatile uint32_t aircr; /* 0xE000ED0C: Application Interrupt and Reset Control Register */
	volatile uint32_t scr;   /* 0xE000ED10: System Control Register */
	volatile uint32_t ccr;   /* 0xE000ED14: Configuration and Control Register */
#if defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	volatile uint8_t shpr[12U]; /* 0xE000ED18: System Handler Priority Registers 1-3 */
#else
	uint32_t reserved0;        /* 0xE000ED18: SHPR1 is not implemented */
	volatile uint32_t shpr[2U]; /* 0xE000ED1C: System Handler Priority Registers 2-3 */
#endif
	volatile uint32_t shcsr; /* 0xE000ED24: System Handler Control and State Register */
	volatile uint32_t cfsr;  /* 0xE000ED28: Configurable Fault Status Register */
	volatile uint32_t hfsr;  /* 0xE000ED2C: HardFault Status Register */
	volatile uint32_t dfsr;  /* 0xE000ED30: Debug Fault Status Register */
	volatile uint32_t mmfar; /* 0xE000ED34: MemManage Fault Address Register */
	volatile uint32_t bfar;  /* 0xE000ED38: BusFault Address Register */
};

#define Z_ARM_SCB ((struct z_arm_scb *)0xE000ED00UL)

/* ICSR: PENDSVSET, ARM DDI 0403 B3.2.4 */
#define Z_ARM_SCB_ICSR_PENDSVSET_Pos 28U
#define Z_ARM_SCB_ICSR_PENDSVSET_Msk (1UL << Z_ARM_SCB_ICSR_PENDSVSET_Pos)

/* AIRCR: VECTKEY and BFHFNMINS, ARM DDI 0403 B3.2.6 / ARM DDI 0553 D1.2.3 */
#define Z_ARM_SCB_AIRCR_VECTKEY_Pos   16U
#define Z_ARM_SCB_AIRCR_VECTKEY_Msk   (0xFFFFUL << Z_ARM_SCB_AIRCR_VECTKEY_Pos)
#define Z_ARM_SCB_AIRCR_BFHFNMINS_Pos 13U
#define Z_ARM_SCB_AIRCR_BFHFNMINS_Msk (1UL << Z_ARM_SCB_AIRCR_BFHFNMINS_Pos)

/* CCR: STKALIGN, ARM DDI 0403 B3.2.8 */
#define Z_ARM_SCB_CCR_STKALIGN_Pos 9U
#define Z_ARM_SCB_CCR_STKALIGN_Msk (1UL << Z_ARM_SCB_CCR_STKALIGN_Pos)

/* SHCSR: fault-exception enable bits, ARM DDI 0403 B3.2.13 / ARM DDI 0553 D1.2.234 */
#define Z_ARM_SCB_SHCSR_SECUREFAULTENA_Pos 19U
#define Z_ARM_SCB_SHCSR_SECUREFAULTENA_Msk (1UL << Z_ARM_SCB_SHCSR_SECUREFAULTENA_Pos)
#define Z_ARM_SCB_SHCSR_USGFAULTENA_Pos    18U
#define Z_ARM_SCB_SHCSR_USGFAULTENA_Msk    (1UL << Z_ARM_SCB_SHCSR_USGFAULTENA_Pos)
#define Z_ARM_SCB_SHCSR_BUSFAULTENA_Pos    17U
#define Z_ARM_SCB_SHCSR_BUSFAULTENA_Msk    (1UL << Z_ARM_SCB_SHCSR_BUSFAULTENA_Pos)
#define Z_ARM_SCB_SHCSR_MEMFAULTENA_Pos    16U
#define Z_ARM_SCB_SHCSR_MEMFAULTENA_Msk    (1UL << Z_ARM_SCB_SHCSR_MEMFAULTENA_Pos)

/* CFSR: per-fault status sub-register masks, ARM DDI 0403 B3.2.15 */
#define Z_ARM_SCB_CFSR_USGFAULTSR_Pos 16U
#define Z_ARM_SCB_CFSR_USGFAULTSR_Msk (0xFFFFUL << Z_ARM_SCB_CFSR_USGFAULTSR_Pos)
#define Z_ARM_SCB_CFSR_BUSFAULTSR_Pos 8U
#define Z_ARM_SCB_CFSR_BUSFAULTSR_Msk (0xFFUL << Z_ARM_SCB_CFSR_BUSFAULTSR_Pos)
#define Z_ARM_SCB_CFSR_MEMFAULTSR_Pos 0U
#define Z_ARM_SCB_CFSR_MEMFAULTSR_Msk (0xFFUL /*<< Z_ARM_SCB_CFSR_MEMFAULTSR_Pos*/)

/* IPSR: exception-number field, ARM DDI 0403 B1.4.2 */
#define Z_ARM_IPSR_ISR_Pos 0U
#define Z_ARM_IPSR_ISR_Msk (0x1FFUL /*<< Z_ARM_IPSR_ISR_Pos*/)

/*
 * System exception numbers, expressed in the CMSIS IRQn_Type convention
 * (architectural exception number minus 16, ARM DDI 0403 B1.5.2), as consumed
 * by z_arm_exc_set_priority() below.
 */
#define Z_ARM_IRQN_MEMMANAGE    (-12)
#define Z_ARM_IRQN_BUSFAULT     (-11)
#define Z_ARM_IRQN_USAGEFAULT   (-10)
#define Z_ARM_IRQN_SECUREFAULT  (-9)
#define Z_ARM_IRQN_SVCALL       (-5)
#define Z_ARM_IRQN_DEBUGMONITOR (-4)
#define Z_ARM_IRQN_PENDSV       (-2)
#define Z_ARM_IRQN_SYSTICK      (-1)

/**
 * @brief Set the priority of a Cortex-M system exception
 *
 * Equivalent of CMSIS NVIC_SetPriority() restricted to system exceptions
 * (negative IRQn values in the CMSIS convention); the priority computation
 * and SHPR access width mirror the CMSIS-Core __NVIC_SetPriority()
 * implementations exactly. NUM_IRQ_PRIO_BITS is devicetree-provided and is
 * build-time asserted equal to the CMSIS __NVIC_PRIO_BITS (see
 * modules/cmsis_6/cmsis_core_m.h).
 *
 * @param exc system exception number in CMSIS IRQn convention (< 0)
 * @param priority priority (0 .. (2^NUM_IRQ_PRIO_BITS)-1)
 */
static ALWAYS_INLINE void z_arm_exc_set_priority(int32_t exc, uint32_t priority)
{
#if defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	Z_ARM_SCB->shpr[(((uint32_t)exc) & 0xFUL) - 4UL] =
		(uint8_t)((priority << (8U - NUM_IRQ_PRIO_BITS)) & (uint32_t)0xFFUL);
#else
	uint32_t shift = (((uint32_t)exc) & 0x03UL) * 8UL;
	uint32_t idx = ((((uint32_t)exc) & 0x0FUL) - 8UL) >> 2UL;

	Z_ARM_SCB->shpr[idx] =
		((uint32_t)(Z_ARM_SCB->shpr[idx] & ~(0xFFUL << shift)) |
		 (((priority << (8U - NUM_IRQ_PRIO_BITS)) & (uint32_t)0xFFUL) << shift));
#endif
}

/* Equivalent of CMSIS __get_IPSR(): reads IPSR, no memory clobber. */
static ALWAYS_INLINE uint32_t z_arm_get_ipsr(void)
{
	uint32_t result;

	__asm__ volatile("MRS %0, ipsr" : "=r"(result));
	return result;
}

/* Equivalent of CMSIS __set_MSP(): writes the Main Stack Pointer, no clobbers. */
static ALWAYS_INLINE void z_arm_set_msp(uint32_t top_of_main_stack)
{
	__asm__ volatile("MSR msp, %0" : : "r"(top_of_main_stack) :);
}

#if defined(CONFIG_CPU_CORTEX_M_HAS_SPLIM)
/* Equivalent of CMSIS __set_MSPLIM(): writes the Main Stack Pointer Limit. */
static ALWAYS_INLINE void z_arm_set_msplim(uint32_t main_stack_ptr_limit)
{
#if (!(defined(__ARM_ARCH_8M_MAIN__) && (__ARM_ARCH_8M_MAIN__ == 1)) &&                            \
     !(defined(__ARM_ARCH_8_1M_MAIN__) && (__ARM_ARCH_8_1M_MAIN__ == 1)) &&                        \
     (!defined(__ARM_FEATURE_CMSE) || (__ARM_FEATURE_CMSE < 3)))
	/* without the Main Extension, the non-secure MSPLIM is RAZ/WI */
	(void)main_stack_ptr_limit;
#else
	__asm__ volatile("MSR msplim, %0" : : "r"(main_stack_ptr_limit));
#endif
}
#endif /* CONFIG_CPU_CORTEX_M_HAS_SPLIM */

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_ARM_INCLUDE_CORTEX_M_SCS_H_ */
