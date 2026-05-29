/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include "test_queue.h"

#ifdef CONFIG_64BIT
#define MAX_SZ 128
#else
#define MAX_SZ 96
#endif
K_HEAP_DEFINE(test_pool, MAX_SZ * 4 + 128);

/*test case main entry*/
void *queue_test_setup(void)
{
	k_thread_heap_assign(k_current_get(), &test_pool);

	return NULL;
}

/**
 * @defgroup kernel_queue_module Kernel Queue Test Module
 * @ingroup kernel_queue_tests
 * @brief Test module at tests/kernel/queue — contains all queue test suites.
 */

/**
 * @defgroup queue_api Queue API ZTest suite
 * @ingroup kernel_queue_module
 * @brief k_queue API tests covering ISR/thread contexts, memory allocation,
 *        multi-queue independence, user-space permissions, and unique-append.
 *
 * @details
 * Contains tests that do not require single-CPU pinning.  Covers ISR-to-thread
 * and thread-to-ISR data passing via all five insertion APIs, implicit memory
 * allocation with heap-exhaustion and success verification, simultaneous
 * operation of multiple independent queues, user-mode permission enforcement
 * (expected kernel oops on unpermissioned k_queue access), and rejection of
 * duplicate items by k_queue_unique_append().
 */
ZTEST_SUITE(queue_api, NULL, queue_test_setup, NULL, NULL, NULL);

/**
 * @defgroup queue_api_1cpu Queue API 1CPU ZTest suite
 * @ingroup kernel_queue_module
 * @brief k_queue API tests requiring single-CPU pinning -- thread-to-thread
 *        passing, concurrent consumer dispatch, and priority-ordered delivery.
 *
 * @details
 * Contains tests pinned to a single CPU via ztest_simple_1cpu_before/after.
 * Covers thread-to-thread data passing for both runtime-initialised and
 * compile-time-defined queues, correct item delivery to each of two concurrent
 * waiting threads, absence of the historical CONFIG_POLL race condition, and
 * priority-then-FIFO dispatch ordering when three threads at two priority levels
 * compete for items.
 */
ZTEST_SUITE(queue_api_1cpu, NULL, queue_test_setup, ztest_simple_1cpu_before,
	    ztest_simple_1cpu_after, NULL);
