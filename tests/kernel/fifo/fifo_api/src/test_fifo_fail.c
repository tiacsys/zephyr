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
 * @brief k_fifo_get() returns NULL on an empty fifo with both K_NO_WAIT and a
 * finite timeout.
 *
 * @details
 * Verifies that k_fifo_get() does not block indefinitely and returns NULL when
 * no item is available -- both when called with K_NO_WAIT (immediate return)
 * and with a finite timeout (TIMEOUT = 100 ms, after which it expires).
 *
 * @reqref{zep-srs-24-7}
 *
 * @see k_fifo_init(), k_fifo_get()
 * @testid{TSPEC-FIFO-API-005}
 * @draft
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
