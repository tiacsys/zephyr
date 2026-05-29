/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file main.c FIFO API test module entry point -- suite and group definitions
 */

/**
 * @defgroup kernel_fifo_api_module Kernel FIFO API Test Module
 * @ingroup kernel_fifo_tests
 * @brief Test module at tests/kernel/fifo/fifo_api -- contains all fifo_api test suites.
 */

/**
 * @defgroup fifo_api FIFO API ZTest suite
 * @ingroup kernel_fifo_api_module
 * @brief k_fifo API tests across ISR and thread contexts -- cross-context data
 *        passing, k_fifo_is_empty() correctness, and failure-mode behaviour.
 *
 * @details
 * Contains tests that do not require single-CPU pinning.  Covers ISR-to-thread
 * and thread-to-ISR data passing using all three insertion APIs (k_fifo_put(),
 * k_fifo_put_list(), k_fifo_put_slist()), k_fifo_is_empty() correctness
 * verified from both thread and ISR context after put and get operations, and
 * k_fifo_get() returning NULL on an empty fifo with K_NO_WAIT and a finite
 * timeout.
 */

/**
 * @defgroup fifo_api_1cpu FIFO API 1CPU ZTest suite
 * @ingroup kernel_fifo_api_module
 * @brief k_fifo API tests requiring single-CPU pinning -- thread-to-thread
 *        passing, cancel-wait semantics, and a multi-context stress loop.
 *
 * @details
 * Contains tests pinned to a single CPU via ztest_simple_1cpu_before/after.
 * Covers thread-to-thread data passing for both runtime-initialised and
 * compile-time-defined fifos, k_fifo_cancel_wait() causing a blocked
 * k_fifo_get() to return NULL within an expected timeframe, and LOOPS (32)
 * repeated multi-context put/get cycles exercised across the main thread,
 * an ISR, and a consumer thread.
 */

#include <zephyr/ztest.h>

ZTEST_SUITE(fifo_api, NULL, NULL, NULL, NULL, NULL);

ZTEST_SUITE(fifo_api_1cpu, NULL, NULL,
		ztest_simple_1cpu_before, ztest_simple_1cpu_after, NULL);
