/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arch_interface.h>

#if defined(CONFIG_SCHED_CPU_MASK) && (CONFIG_MP_MAX_NUM_CPUS > 1)
#define SMP_TEST
#endif

#ifdef SMP_TEST
#define MAX_NUM_THREADS CONFIG_MP_MAX_NUM_CPUS
#define STACK_SIZE  1024
#define PRIORITY    7

static struct k_thread threads[MAX_NUM_THREADS];
static K_THREAD_STACK_ARRAY_DEFINE(tstack, MAX_NUM_THREADS, STACK_SIZE);
#endif

#define WAIT_US 1000
#define WAIT_NS (WAIT_US * 1000)
#define TOLERANCE 0.1

static void perform_tests(void)
{
	timing_t start, middle, end;
	uint64_t diff1, diff2, diff_all, freq_hz;
	uint64_t diff1_ns, diff2_ns, diff_all_ns, diff_avg_ns;
	uint32_t freq_mhz;

	arch_timing_start();

	start = arch_timing_counter_get();
	k_busy_wait(WAIT_US);
	middle = arch_timing_counter_get();
	k_busy_wait(WAIT_US);
	end = arch_timing_counter_get();

	/* Time shouldn't stop or go backwards */
	diff1 = arch_timing_cycles_get(&start, &middle);
	diff2 = arch_timing_cycles_get(&middle, &end);
	diff_all = arch_timing_cycles_get(&start, &end);
	zassert_true(diff1 > 0, NULL);
	zassert_true(diff2 > 0, NULL);
	zassert_true(diff_all > 0, NULL);

	/* Differences shouldn't be so different, as both are spaced by
	 * k_busy_wait(WAIT_US).
	 */
	zassert_within(diff1, diff2, diff1 * TOLERANCE, NULL);
	zassert_within(diff_all, diff1 + diff2, (diff1 + diff2) * TOLERANCE,
		       NULL);

	freq_hz = arch_timing_freq_get();
	freq_mhz = arch_timing_freq_get_mhz();
	zassert_equal(freq_mhz, (uint32_t)(freq_hz / 1000000ULL), NULL);

	diff1_ns = arch_timing_cycles_to_ns(diff1);
	diff2_ns = arch_timing_cycles_to_ns(diff2);
	diff_all_ns = arch_timing_cycles_to_ns(diff_all);

	/* Ensure the differences are close to 100us */
	zassert_within(diff1_ns, WAIT_NS, WAIT_NS * TOLERANCE, NULL);
	zassert_within(diff2_ns, WAIT_NS, WAIT_NS * TOLERANCE, NULL);
	zassert_within(diff_all_ns, 2 * WAIT_NS, 2 * WAIT_NS * TOLERANCE, NULL);

	diff_avg_ns = arch_timing_cycles_to_ns_avg(diff1 + diff2, 2);
	zassert_within(diff_avg_ns, WAIT_NS, WAIT_NS * TOLERANCE, NULL);

	arch_timing_stop();
}

void *timing_setup(void)
{
	arch_timing_init();

	return NULL;
}

/**
 * @brief Verify that the arch timing functions measure two busy waits of 1 ms
 * with a tolerance of 10 percent.
 *
 * @details
 * The suite setup function calls arch_timing_init(). The test then calls
 * perform_tests() two times. Thus it also verifies that the timing functions
 * work again after arch_timing_stop().
 *
 * Test steps:
 * - Call arch_timing_start().
 * - Call arch_timing_counter_get() before, between and after two k_busy_wait()
 *   calls of 1000 us.
 * - Get the cycle counts of the two intervals and of the total with
 *   arch_timing_cycles_get().
 * - Get the frequency with arch_timing_freq_get() and
 *   arch_timing_freq_get_mhz().
 * - Convert the cycle counts to nanoseconds with arch_timing_cycles_to_ns().
 * - Get the average of the two intervals with arch_timing_cycles_to_ns_avg().
 * - Call arch_timing_stop().
 * - Do all these steps a second time.
 *
 * Expected result:
 * - All cycle counts are more than 0.
 * - The two intervals are equal within 10 percent. The total is equal to their
 *   sum within 10 percent.
 * - arch_timing_freq_get_mhz() returns the value of arch_timing_freq_get()
 *   divided by 1000000.
 * - Each interval and the average are 1000000 ns within 10 percent.
 * - The total is 2000000 ns within 10 percent.
 *
 * @testid{TSPEC-ARCHCOMMON-060}
 * @draft
 * @see arch_timing_init(), arch_timing_start(), arch_timing_counter_get(),
 * arch_timing_cycles_get(), arch_timing_freq_get(), arch_timing_freq_get_mhz(),
 * arch_timing_cycles_to_ns(), arch_timing_cycles_to_ns_avg(),
 * arch_timing_stop()
 */
ZTEST(arch_timing, test_arch_timing)
{
	perform_tests();
	/* Run tests again to ensure nothing breaks after arch_timing_stop */
	perform_tests();
}

#ifdef SMP_TEST
static void thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	perform_tests();
	/* Run tests again to ensure nothing breaks after arch_timing_stop */
	perform_tests();
}

/**
 * @brief Verify that the arch timing functions measure busy waits correctly on
 * each CPU.
 *
 * @details
 * The test creates one thread for each CPU and uses a CPU mask to keep each
 * thread on its CPU. Each thread calls perform_tests() two times, as
 * test_arch_timing does. The build contains this test only if
 * CONFIG_SCHED_CPU_MASK is enabled and CONFIG_MP_MAX_NUM_CPUS is more than 1.
 *
 * Test steps:
 * - Create one thread for each CPU that arch_num_cpus() gives, with the delay
 *   K_FOREVER.
 * - Enable the CPU mask of each thread for its CPU only. Then start the thread.
 * - Join all the threads.
 *
 * Expected result:
 * - On each CPU, the measurements meet the same limits as in test_arch_timing.
 * - All threads end.
 *
 * @testid{TSPEC-ARCHCOMMON-061}
 * @draft
 * @see arch_timing_init(), arch_timing_start(), arch_timing_counter_get(),
 * arch_timing_cycles_get(), arch_timing_freq_get(), arch_timing_freq_get_mhz(),
 * arch_timing_cycles_to_ns(), arch_timing_cycles_to_ns_avg(),
 * arch_timing_stop(), k_thread_cpu_mask_enable()
 */
ZTEST(arch_timing, test_arch_timing_smp)
{
	int i;
	unsigned int num_threads = arch_num_cpus();

	for (i = 0; i < num_threads; i++) {
		k_thread_create(&threads[i], tstack[i], STACK_SIZE,
				thread_entry, NULL, NULL, NULL,
				PRIORITY, 0, K_FOREVER);
		k_thread_cpu_mask_enable(&threads[i], i);
		k_thread_start(&threads[i]);
	}

	for (i = 0; i < num_threads; i++) {
		k_thread_join(&threads[i], K_FOREVER);
	}
}
#else
ZTEST(arch_timing, test_arch_timing_smp)
{
	ztest_test_skip();
}
#endif

ZTEST_SUITE(arch_timing, NULL, timing_setup, NULL, NULL, NULL);
