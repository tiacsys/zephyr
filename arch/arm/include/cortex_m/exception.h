/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Exception/interrupt context helpers for Cortex-M CPUs
 *
 * Exception/interrupt context helpers.
 */

#ifndef ZEPHYR_ARCH_ARM_INCLUDE_CORTEX_M_EXCEPTION_H_
#define ZEPHYR_ARCH_ARM_INCLUDE_CORTEX_M_EXCEPTION_H_

#include <zephyr/arch/cpu.h>

#ifdef _ASMLANGUAGE

/* nothing */

#else

/*
 * The always-inline helpers below (arch_is_in_isr() in the kernel's hot path,
 * z_arm_exc_setup()/z_arm_clear_faults() inlined into arch_kernel_init(), ...)
 * are compiled into kernel translation units: use the Zephyr-owned Cortex-M
 * register definitions instead of <cmsis_core.h> to keep the kernel's
 * architecture-portable include graph free of vendor CMSIS/HAL headers.
 */
#include <cortex_m/scs.h>
#include <zephyr/arch/arm/exception.h>
#include <zephyr/irq_offload.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_IRQ_OFFLOAD
extern volatile irq_offload_routine_t offload_routine;
#endif

/* Writes to the AIRCR must be accompanied by a write of the value 0x05FA
 * to the Vector Key field, otherwise the writes are ignored.
 */
#define AIRCR_VECT_KEY_PERMIT_WRITE            0x05FAUL

/* Exception Return (EXC_RETURN) is provided in LR upon exception entry.
 * It is used to perform an exception return and to detect possible state
 * transition upon exception.
 */

/* Prefix. Indicates that this is an EXC_RETURN value.
 * This field reads as 0b11111111.
 */
#define EXC_RETURN_INDICATOR_PREFIX            (0xFFU << 24)
/* bit[0]: Exception Secure. The security domain the exception was taken to. */
#define EXC_RETURN_EXCEPTION_SECURE_Pos        0
#define EXC_RETURN_EXCEPTION_SECURE_Msk        BIT(EXC_RETURN_EXCEPTION_SECURE_Pos)
#define EXC_RETURN_EXCEPTION_SECURE_Non_Secure 0
#define EXC_RETURN_EXCEPTION_SECURE_Secure     EXC_RETURN_EXCEPTION_SECURE_Msk
/* bit[2]: Stack Pointer selection. */
#define EXC_RETURN_SPSEL_Pos                   2
#define EXC_RETURN_SPSEL_Msk                   BIT(EXC_RETURN_SPSEL_Pos)
#define EXC_RETURN_SPSEL_MAIN                  0
#define EXC_RETURN_SPSEL_PROCESS               EXC_RETURN_SPSEL_Msk
/* bit[3]: Mode. Indicates the Mode that was stacked from. */
#define EXC_RETURN_MODE_Pos                    3
#define EXC_RETURN_MODE_Msk                    BIT(EXC_RETURN_MODE_Pos)
#define EXC_RETURN_MODE_HANDLER                0
#define EXC_RETURN_MODE_THREAD                 EXC_RETURN_MODE_Msk
/* bit[4]: Stack frame type. Indicates whether the stack frame is a standard
 * integer only stack frame or an extended floating-point stack frame.
 */
#define EXC_RETURN_STACK_FRAME_TYPE_Pos        4
#define EXC_RETURN_STACK_FRAME_TYPE_Msk        BIT(EXC_RETURN_STACK_FRAME_TYPE_Pos)
#define EXC_RETURN_STACK_FRAME_TYPE_EXTENDED   0
#define EXC_RETURN_STACK_FRAME_TYPE_STANDARD   EXC_RETURN_STACK_FRAME_TYPE_Msk
/* bit[5]: Default callee register stacking. Indicates whether the default
 * stacking rules apply, or whether the callee registers are already on the
 * stack.
 */
#define EXC_RETURN_CALLEE_STACK_Pos            5
#define EXC_RETURN_CALLEE_STACK_Msk            BIT(EXC_RETURN_CALLEE_STACK_Pos)
#define EXC_RETURN_CALLEE_STACK_SKIPPED        0
#define EXC_RETURN_CALLEE_STACK_DEFAULT        EXC_RETURN_CALLEE_STACK_Msk
/* bit[6]: Secure or Non-secure stack. Indicates whether a Secure or
 * Non-secure stack is used to restore stack frame on exception return.
 */
#define EXC_RETURN_RETURN_STACK_Pos            6
#define EXC_RETURN_RETURN_STACK_Msk            BIT(EXC_RETURN_RETURN_STACK_Pos)
#define EXC_RETURN_RETURN_STACK_Non_Secure     0
#define EXC_RETURN_RETURN_STACK_Secure         EXC_RETURN_RETURN_STACK_Msk

/*
 * The current executing vector is found in the IPSR register. All
 * IRQs and system exceptions are considered as interrupt context.
 */
static ALWAYS_INLINE bool arch_is_in_isr(void)
{
	return (z_arm_get_ipsr()) ? (true) : (false);
}

/**
 * @brief Find out if we were in ISR context
 *        before the current exception occurred.
 *
 * A function that determines, based on inspecting the current
 * ESF, whether the processor was in handler mode before entering
 * the current exception state (i.e. nested exception) or not.
 *
 * Notes:
 * - The function shall only be called from ISR context.
 * - We do not use ARM processor state flags to determine
 *   whether we are in a nested exception; we rely on the
 *   RETPSR value stacked on the ESF. Hence, the function
 *   assumes that the ESF stack frame has a valid RETPSR
 *   value.
 *
 * @param esf the exception stack frame (cannot be NULL)
 * @return true if execution state was in handler mode, before
 *              the current exception occurred, otherwise false.
 */
static ALWAYS_INLINE bool arch_is_in_nested_exception(const struct arch_esf *esf)
{
	return (esf->basic.xpsr & Z_ARM_IPSR_ISR_Msk) ? (true) : (false);
}

#if defined(CONFIG_USERSPACE)
/**
 * @brief Is the thread in unprivileged mode
 *
 * @param esf the exception stack frame (unused)
 * @return true if the current thread was in unprivileged mode
 */
static ALWAYS_INLINE bool z_arm_preempted_thread_in_user_mode(const struct arch_esf *esf)
{
	return z_arm_thread_is_in_user_mode();
}
#endif

/**
 * @brief Setup system exceptions
 *
 * Set exception priorities to conform with the BASEPRI locking mechanism.
 * Set PendSV priority to lowest possible.
 *
 * Enable fault exceptions.
 */
static ALWAYS_INLINE void z_arm_exc_setup(void)
{
	/* PendSV is set to lowest priority, regardless of it being used.
	 * This is done as the IRQ is always enabled.
	 */
	z_arm_exc_set_priority(Z_ARM_IRQN_PENDSV, _EXC_PENDSV_PRIO);

#ifdef CONFIG_CPU_CORTEX_M_HAS_BASEPRI
	/* Note: SVCall IRQ priority level is left to default (0)
	 * for Cortex-M variants without BASEPRI (e.g. ARMv6-M).
	 */
	z_arm_exc_set_priority(Z_ARM_IRQN_SVCALL, _EXC_SVC_PRIO);
#endif

#ifdef CONFIG_CPU_CORTEX_M_HAS_PROGRAMMABLE_FAULT_PRIOS
	z_arm_exc_set_priority(Z_ARM_IRQN_MEMMANAGE, _EXC_FAULT_PRIO);
	z_arm_exc_set_priority(Z_ARM_IRQN_BUSFAULT, _EXC_FAULT_PRIO);
	z_arm_exc_set_priority(Z_ARM_IRQN_USAGEFAULT, _EXC_FAULT_PRIO);
#if defined(CONFIG_CORTEX_M_DEBUG_MONITOR_HOOK)
	z_arm_exc_set_priority(Z_ARM_IRQN_DEBUGMONITOR, IRQ_PRIO_LOWEST);
#elif defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	z_arm_exc_set_priority(Z_ARM_IRQN_DEBUGMONITOR, _EXC_FAULT_PRIO);
#endif
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
	z_arm_exc_set_priority(Z_ARM_IRQN_SECUREFAULT, _EXC_FAULT_PRIO);
#endif /* CONFIG_ARM_SECURE_FIRMWARE */

	/* Enable Usage, Mem, & Bus Faults */
	Z_ARM_SCB->shcsr |= Z_ARM_SCB_SHCSR_USGFAULTENA_Msk | Z_ARM_SCB_SHCSR_MEMFAULTENA_Msk |
			    Z_ARM_SCB_SHCSR_BUSFAULTENA_Msk;
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
	/* Enable Secure Fault */
	Z_ARM_SCB->shcsr |= Z_ARM_SCB_SHCSR_SECUREFAULTENA_Msk;
	/* Clear BFAR before setting BusFaults to target Non-Secure state. */
	Z_ARM_SCB->bfar = 0;
#endif /* CONFIG_ARM_SECURE_FIRMWARE */
#endif /* CONFIG_CPU_CORTEX_M_HAS_PROGRAMMABLE_FAULT_PRIOS */

#if defined(CONFIG_ARM_SECURE_FIRMWARE) && !defined(CONFIG_ARM_SECURE_BUSFAULT_HARDFAULT_NMI)
	/* Set NMI, Hard, and Bus Faults as Non-Secure.
	 * NMI and Bus Faults targeting the Secure state will
	 * escalate to a SecureFault or SecureHardFault.
	 */
	Z_ARM_SCB->aircr = (Z_ARM_SCB->aircr & (~(Z_ARM_SCB_AIRCR_VECTKEY_Msk))) |
			   Z_ARM_SCB_AIRCR_BFHFNMINS_Msk |
			   ((AIRCR_VECT_KEY_PERMIT_WRITE << Z_ARM_SCB_AIRCR_VECTKEY_Pos) &
			    Z_ARM_SCB_AIRCR_VECTKEY_Msk);
	/* Note: Fault conditions that would generate a SecureFault
	 * in a PE with the Main Extension instead generate a
	 * SecureHardFault in a PE without the Main Extension.
	 */
#endif /* ARM_SECURE_FIRMWARE && !ARM_SECURE_BUSFAULT_HARDFAULT_NMI */

#if defined(CONFIG_CPU_CORTEX_M_HAS_SYSTICK) && !defined(CONFIG_CORTEX_M_SYSTICK)
	/* SoC implements SysTick, but the system does not use it
	 * as driver for system timing. However, the SysTick IRQ is
	 * always enabled, so we must ensure the interrupt priority
	 * is set to a level lower than the kernel interrupts (for
	 * the assert mechanism to work properly) in case the SysTick
	 * interrupt is accidentally raised.
	 */
	NVIC_SetPriority(SysTick_IRQn, _EXC_IRQ_DEFAULT_PRIO);
#endif /* CPU_CORTEX_M_HAS_SYSTICK && ! CORTEX_M_SYSTICK */
}

/**
 * @brief Clear Fault exceptions
 *
 * Clear out exceptions for Mem, Bus, Usage and Hard Faults
 */
static ALWAYS_INLINE void z_arm_clear_faults(void)
{
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	/* Reset all faults */
	Z_ARM_SCB->cfsr = Z_ARM_SCB_CFSR_USGFAULTSR_Msk | Z_ARM_SCB_CFSR_MEMFAULTSR_Msk |
			  Z_ARM_SCB_CFSR_BUSFAULTSR_Msk;

	/* Clear all Hard Faults - HFSR is write-one-to-clear */
	Z_ARM_SCB->hfsr = 0xffffffff;
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
}

/**
 * @brief Set z_arm_coredump_fault_sp to stack pointer value expected by GDB
 *
 * @param esf exception frame
 * @param exc_return EXC_RETURN value present in LR after exception entry.
 */
static ALWAYS_INLINE void z_arm_set_fault_sp(const struct arch_esf *esf, uint32_t exc_return)
{
#ifdef CONFIG_DEBUG_COREDUMP
	z_arm_coredump_fault_sp = POINTER_TO_UINT(esf);
#if defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE) || defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	/* Gdb expects a stack pointer that does not include the exception stack frame in order to
	 * unwind. So adjust the stack pointer accordingly.
	 */
	z_arm_coredump_fault_sp += sizeof(esf->basic);

#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
	/* Assess whether thread had been using the FP registers and add size of additional
	 * registers if necessary
	 */
	if ((exc_return & EXC_RETURN_STACK_FRAME_TYPE_STANDARD) ==
	    EXC_RETURN_STACK_FRAME_TYPE_EXTENDED) {
		z_arm_coredump_fault_sp += sizeof(esf->fpu);
	}
#endif /* CONFIG_FPU && CONFIG_FPU_SHARING */

#if !(defined(CONFIG_ARMV8_M_MAINLINE) || defined(CONFIG_ARMV8_M_BASELINE))
	if ((esf->basic.xpsr & Z_ARM_SCB_CCR_STKALIGN_Msk) == Z_ARM_SCB_CCR_STKALIGN_Msk) {
		/* Adjust stack alignment after PSR bit[9] detected */
		z_arm_coredump_fault_sp |= 0x4;
	}
#endif /* !CONFIG_ARMV8_M_MAINLINE */

#endif /* CONFIG_ARMV7_M_ARMV8_M_MAINLINE || CONFIG_ARMV6_M_ARMV8_M_BASELINE */
#endif /* CONFIG_DEBUG_COREDUMP */
}

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_ARM_INCLUDE_CORTEX_M_EXCEPTION_H_ */
