/*
 * Copyright (c) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/irq_offload.h>

#include <zephyr/kernel/thread_stack.h>

#define IV_CTRL_PROTECTION_EXCEPTION 21

#define CTRL_PROTECTION_ERRORCODE_NEAR_RET 1
#define CTRL_PROTECTION_ERRORCODE_ENDBRANCH 3

#define STACKSIZE 1024
#define THREAD_PRIORITY 5

K_SEM_DEFINE(error_handler_sem, 0, 1);

volatile bool expect_fault;
volatile int expect_code;
volatile int expect_reason;

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *pEsf)
{
	if (expect_fault) {
#ifdef CONFIG_X86_64
		zassert_equal(pEsf->vector, expect_reason, "unexpected exception");
		zassert_equal(pEsf->code, expect_code, "unexpected error code");
#else
		zassert_equal(z_x86_exception_vector, expect_reason, "unexpected exception");
		zassert_equal(pEsf->errorCode, expect_code, "unexpected error code");
#endif
		printk("fatal error expected as part of test case\n");
		expect_fault = false;

		k_sem_give(&error_handler_sem);
	} else {
		printk("fatal error was unexpected, aborting\n");
		TC_END_REPORT(TC_FAIL);
		k_fatal_halt(reason);
	}
}

#ifdef CONFIG_HW_SHADOW_STACK
void thread_a_entry(void *p1, void *p2, void *p3);
K_SEM_DEFINE(thread_a_sem, 0, 1);
K_THREAD_DEFINE(thread_a, STACKSIZE, thread_a_entry, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, -1);

void thread_b_entry(void *p1, void *p2, void *p3);
K_SEM_DEFINE(thread_b_sem, 0, 1);
K_SEM_DEFINE(thread_b_irq_sem, 0, 1);
K_THREAD_DEFINE(thread_b, STACKSIZE, thread_b_entry, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, -1);

static bool is_shstk_enabled(void)
{
	long cur;

	cur = z_x86_msr_read(X86_S_CET_MSR);
	return (cur & X86_S_CET_MSR_SHSTK_EN) == X86_S_CET_MSR_SHSTK_EN;
}

void thread_c_entry(void *p1, void *p2, void *p3)
{
	zassert_true(is_shstk_enabled(), "shadow stack not enabled on static thread");
}

K_THREAD_DEFINE(thread_c, STACKSIZE, thread_c_entry, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, 0);

void __attribute__((optimize("O0"))) foo(void)
{
	printk("foo called\n");
}

void __attribute__((optimize("O0"))) fail(void)
{
	long a[] = {0};

	printk("should fail after this\n");

	*(a + 2) = (long)&foo;
}

struct k_work work;

void work_handler(struct k_work *wrk)
{
	printk("work handler\n");

	zassert_true(is_shstk_enabled(), "shadow stack not enabled");
}

/**
 * @brief Verify that the hardware shadow stack is enabled when a work item runs
 * on the system work queue.
 *
 * @details
 * The build contains this test only if CONFIG_HW_SHADOW_STACK is enabled. The
 * work handler reads X86_S_CET_MSR and asserts that the shadow stack enable bit
 * is set.
 *
 * Test steps:
 * - Initialize a work item with work_handler().
 * - Submit the work item with k_work_submit().
 *
 * Expected result:
 * - In work_handler(), the X86_S_CET_MSR_SHSTK_EN bit of X86_S_CET_MSR is set.
 *
 * @testid{TSPEC-ARCHX86-008}
 * @draft
 */
ZTEST(cet, test_shstk_work_q)
{
	k_work_init(&work, work_handler);
	k_work_submit(&work);
}

void intr_handler(const void *p)
{
	printk("interrupt handler\n");

	if (p != NULL) {
		/* Test one nested level. It should just work. */
		printk("trying interrupt handler\n");
		irq_offload(intr_handler, NULL);

		k_sem_give((struct k_sem *)p);
	} else {
		printk("interrupt handler nested\n");
	}
}

void thread_b_entry(void *p1, void *p2, void *p3)
{
	k_sem_take(&thread_b_sem, K_FOREVER);

	irq_offload(intr_handler, &thread_b_irq_sem);

	k_sem_take(&thread_b_irq_sem, K_FOREVER);
}

/**
 * @brief Verify that an interrupt handler and a nested interrupt handler run
 * with the hardware shadow stack enabled.
 *
 * @details
 * thread_b calls irq_offload() with intr_handler(). intr_handler() calls
 * irq_offload() one more time, which gives one nested interrupt level. An
 * unexpected fatal error stops the system and fails the test.
 *
 * Test steps:
 * - Start thread_b.
 * - Give thread_b_sem to let thread_b offload the interrupt handler.
 * - Wait until thread_b ends, with k_thread_join().
 *
 * Expected result:
 * - The interrupt handler and the nested handler run.
 * - The interrupt handler gives thread_b_irq_sem, and thread_b ends.
 * - No fatal error occurs.
 *
 * @testid{TSPEC-ARCHX86-009}
 * @draft
 */
ZTEST(cet, test_shstk_irq)
{
	k_thread_start(thread_b);

	k_sem_give(&thread_b_sem);

	k_thread_join(thread_b, K_FOREVER);
}

void thread_a_entry(void *p1, void *p2, void *p3)
{
	k_sem_take(&thread_a_sem, K_FOREVER);

	fail();

	zassert_unreachable("should not reach here");
}

/**
 * @brief Verify that the hardware shadow stack detects a changed return address
 * and causes a control protection exception.
 *
 * @details
 * fail() writes the address of foo() past the end of a local array. This write
 * changes the return address on the normal stack, but not on the shadow stack.
 * The return from fail() then causes a control protection exception.
 *
 * Test steps:
 * - Start thread_a.
 * - Set the expected exception to IV_CTRL_PROTECTION_EXCEPTION and
 *   CTRL_PROTECTION_ERRORCODE_NEAR_RET.
 * - Give thread_a_sem to let thread_a call fail().
 * - Wait for error_handler_sem. Then abort thread_a.
 *
 * Expected result:
 * - A fatal error occurs with the vector IV_CTRL_PROTECTION_EXCEPTION and the
 *   error code CTRL_PROTECTION_ERRORCODE_NEAR_RET.
 * - The fatal error handler gives error_handler_sem.
 * - thread_a does not run the code after fail().
 *
 * @testid{TSPEC-ARCHX86-010}
 * @draft
 */
ZTEST(cet, test_shstk)
{
	k_thread_start(thread_a);

	expect_fault = true;
	expect_code = CTRL_PROTECTION_ERRORCODE_NEAR_RET;
	expect_reason = IV_CTRL_PROTECTION_EXCEPTION;
	k_sem_give(&thread_a_sem);

	k_sem_take(&error_handler_sem, K_FOREVER);
	k_thread_abort(thread_a);
}
#endif /* CONFIG_HW_SHADOW_STACK */

#ifdef CONFIG_X86_CET_IBT
extern int should_work(int a);
extern int should_not_work(int a);

/* Round trip to trick optimisations and ensure the calls are indirect */
int do_call(int (*func)(int), int a)
{
	return func(a);
}

/**
 * @brief Verify that indirect branch tracking faults on an indirect call to a
 * function without an end-branch instruction.
 *
 * @details
 * should_work() starts with an endbr32 or endbr64 instruction, and
 * should_not_work() does not. do_call() calls each function through a function
 * pointer. The build contains this test only if CONFIG_X86_CET_IBT is enabled.
 *
 * Test steps:
 * - Call should_work() through do_call() with the argument 1.
 * - Set the expected exception to IV_CTRL_PROTECTION_EXCEPTION and
 *   CTRL_PROTECTION_ERRORCODE_ENDBRANCH.
 * - Call should_not_work() through do_call() with the argument 1.
 *
 * Expected result:
 * - should_work() returns 2.
 * - The call to should_not_work() causes a fatal error with the vector
 *   IV_CTRL_PROTECTION_EXCEPTION and the error code
 *   CTRL_PROTECTION_ERRORCODE_ENDBRANCH.
 * - The code after the call does not run.
 *
 * @testid{TSPEC-ARCHX86-011}
 * @draft
 */
ZTEST(cet, test_ibt)
{
	zassert_equal(do_call(should_work, 1), 2, "should_work failed");

	expect_fault = true;
	expect_code = CTRL_PROTECTION_ERRORCODE_ENDBRANCH;
	expect_reason = IV_CTRL_PROTECTION_EXCEPTION;
	do_call(should_not_work, 1);
	zassert_unreachable("should_not_work did not fault");
}
#endif /* CONFIG_X86_CET_IBT */

ZTEST_SUITE(cet, NULL, NULL, NULL, NULL, NULL);
