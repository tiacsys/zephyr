/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_fifo_fail.c Failure-mode tests for the FIFO API
 */

#include "test_fifo.h"

#define TIMEOUT K_MSEC(100)

/**
 * @addtogroup tests_kernel_fifo
 * @{
 */

/**
 * @brief Verify k_fifo_get() on an empty FIFO returns NULL.
 *
 * @details
 * When no data is queued, k_fifo_get() must return NULL rather than block
 * indefinitely or return stale data. The contract is checked both with
 * K_NO_WAIT (immediate) and with a finite timeout that is allowed to expire.
 *
 * Test steps:
 * - Initialize an empty FIFO.
 * - Call k_fifo_get() with K_NO_WAIT and verify it returns NULL.
 * - Call k_fifo_get() with a finite timeout and verify it returns NULL.
 *
 * Expected result:
 * - Both k_fifo_get() calls return NULL.
 *
 * @see k_fifo_get()
 * @verifies ZEP-SRS-24-7
 */
ZTEST(fifo_api, test_fifo_get_fail)
{
	static struct k_fifo fifo;

	/** @par Arrange
	 * -# Initialise an empty @p fifo via k_fifo_init().
	 */
	k_fifo_init(&fifo);

	/** @par Act
	 * -# Call k_fifo_get() with K_NO_WAIT on the empty fifo.
	 * -# Call k_fifo_get() with TIMEOUT (100 ms) on the still-empty fifo.
	 */

	/** @par Assert
	 * -# Both calls return NULL, confirming that k_fifo_get() does not
	 *    block past its timeout and correctly signals an empty fifo.
	 */
	/**TESTPOINT: fifo get returns NULL*/
	zassert_is_null(k_fifo_get(&fifo, K_NO_WAIT), NULL);
	zassert_is_null(k_fifo_get(&fifo, TIMEOUT), NULL);
}
