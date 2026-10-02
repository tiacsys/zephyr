/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/clock.h>
#include <stdbool.h>
#include <zephyr/tc_util.h>
#include <zephyr/ztest.h>

#define THREAD_STACK		(384 + CONFIG_TEST_EXTRA_STACK_SIZE)

#define TEST_TICKS_TO_SLEEP	(CONFIG_SYS_CLOCK_TICKS_PER_SEC / 2)

/* Helper thread data */
static K_THREAD_STACK_DEFINE(helper_tstack, THREAD_STACK);
static struct k_thread helper_tdata;
static k_tid_t  helper_ttid;

/* time that the thread was actually sleeping */
static int actual_sleep_ticks;
static int actual_post_kernel_sleep_ticks;
static int actual_app_sleep_ticks;

static bool test_failure = true;

static void helper_thread(void *p1, void *p2, void *p3)
{
	test_failure = false;
}

static int ticks_to_sleep(int ticks)
{
	uint32_t start_time;
	uint32_t stop_time;

	start_time = k_cycle_get_32();
	k_sleep(K_MSEC(k_ticks_to_ms_floor64(ticks)));
	stop_time = k_cycle_get_32();

	return (stop_time - start_time) / k_ticks_to_cyc_floor32(1);
}


static int test_early_sleep_post_kernel(void)
{
	actual_post_kernel_sleep_ticks = ticks_to_sleep(TEST_TICKS_TO_SLEEP);
	return 0;
}

SYS_INIT(test_early_sleep_post_kernel,
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

static int test_early_sleep_app(void)
{
	actual_app_sleep_ticks = ticks_to_sleep(TEST_TICKS_TO_SLEEP);
	return 0;
}

SYS_INIT(test_early_sleep_app, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

/**
 * @brief Verify that k_sleep() works during system initialization and after the
 * system starts.
 *
 * @details
 * Two SYS_INIT() functions call k_sleep() at the POST_KERNEL and the
 * APPLICATION levels and record the slept ticks. The test cannot call k_sleep()
 * at the PRE_KERNEL_1 or PRE_KERNEL_2 level. At these levels, the kernel
 * initializes its core objects and the devices.
 *
 * The test also verifies that a thread of lower priority runs while the test
 * thread sleeps.
 *
 * Test steps:
 * - Set the priority of the test thread to 0.
 * - Create a helper thread with a priority one level lower than the test
 *   thread.
 * - Read the slept ticks of the POST_KERNEL and the APPLICATION levels.
 * - Call k_sleep() for TEST_TICKS_TO_SLEEP ticks. Measure the slept ticks with
 *   k_cycle_get_32().
 * - Read the flag that the helper thread clears.
 *
 * Expected result:
 * - At each level and after the system starts, the slept ticks plus 1 are more
 *   than TEST_TICKS_TO_SLEEP.
 * - The helper thread ran.
 *
 * @testid{TSPEC-EARLYSLEEP-001}
 * @draft
 * @see k_sleep()
 *
 * @ingroup kernel_sleep_tests
 */
ZTEST(earlysleep, test_early_sleep)
{
	TC_PRINT("Testing early sleeping\n");

	/*
	 * Main thread(test_main) priority is 0 but ztest thread runs at
	 * priority -1. To run the test smoothly make both main and ztest
	 * threads run at same priority level.
	 */
	k_thread_priority_set(k_current_get(), 0);

	TC_PRINT("msec per tick: %" PRId64 ".%03" PRId64 ", ticks to sleep: %d\n",
			k_ticks_to_ms_floor64(1000) / 1000U,
			k_ticks_to_ms_floor64(1000) % 1000,
			TEST_TICKS_TO_SLEEP);

	/* Create a lower priority thread */
	helper_ttid = k_thread_create(&helper_tdata,
				   helper_tstack, THREAD_STACK,
				   helper_thread, NULL, NULL, NULL,
				   k_thread_priority_get(k_current_get()) + 1,
				   K_INHERIT_PERMS, K_NO_WAIT);

	TC_PRINT("k_sleep() ticks at POST_KERNEL level: %d\n",
					actual_post_kernel_sleep_ticks);
	zassert_true((actual_post_kernel_sleep_ticks + 1) >
					TEST_TICKS_TO_SLEEP, NULL);

	TC_PRINT("k_sleep() ticks at APPLICATION level: %d\n",
					actual_app_sleep_ticks);
	zassert_true((actual_app_sleep_ticks + 1) >
					TEST_TICKS_TO_SLEEP, NULL);

	actual_sleep_ticks = ticks_to_sleep(TEST_TICKS_TO_SLEEP);
	TC_PRINT("k_sleep() ticks on running system: %d\n",
					actual_sleep_ticks);
	zassert_true((actual_sleep_ticks + 1) >
					TEST_TICKS_TO_SLEEP, NULL);

	zassert_false(test_failure, "Lower priority thread not ran!!");
}

ZTEST_SUITE(earlysleep, NULL, NULL,
		ztest_simple_1cpu_before, ztest_simple_1cpu_after, NULL);
