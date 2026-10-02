/*
 * Copyright (c) 2023 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

/**
 * @brief Verify that the timepoint functions give the correct expiry and
 * timeout for K_NO_WAIT, K_FOREVER and a finite timeout.
 *
 * @details
 * sys_timepoint_calc() calculates an absolute timepoint from a timeout. For the
 * finite timeout, the test sleeps past the timepoint and verifies that the
 * timepoint expires.
 *
 * Test steps:
 * - Calculate a timepoint for K_NO_WAIT.
 * - Get its expiry with sys_timepoint_expired() and its timeout with
 *   sys_timepoint_timeout().
 * - Do the same for K_FOREVER and for K_SECONDS(1).
 * - Sleep 1100 ms. Then get the expiry and the timeout of the K_SECONDS(1)
 *   timepoint again.
 *
 * Expected result:
 * - The K_NO_WAIT timepoint is expired, and its timeout is K_NO_WAIT.
 * - The K_FOREVER timepoint is not expired, and its timeout is K_FOREVER.
 * - Before the sleep, the K_SECONDS(1) timepoint is not expired. Its timeout is
 *   more than 0 ticks and not more than K_SECONDS(1).
 * - After the sleep, the K_SECONDS(1) timepoint is expired, and its timeout is
 *   K_NO_WAIT.
 *
 * @see sys_timepoint_calc(), sys_timepoint_expired(), sys_timepoint_timeout()
 */
ZTEST(timepoints, test_timepoint_api)
{
	k_timepoint_t timepoint;
	k_timeout_t timeout, remaining;

	timeout = K_NO_WAIT;
	timepoint = sys_timepoint_calc(timeout);
	zassert_true(sys_timepoint_expired(timepoint));
	remaining = sys_timepoint_timeout(timepoint);
	zassert_true(K_TIMEOUT_EQ(remaining, K_NO_WAIT));

	timeout = K_FOREVER;
	timepoint = sys_timepoint_calc(timeout);
	zassert_false(sys_timepoint_expired(timepoint));
	remaining = sys_timepoint_timeout(timepoint);
	zassert_true(K_TIMEOUT_EQ(remaining, K_FOREVER));

	timeout = K_SECONDS(1);
	timepoint = sys_timepoint_calc(timeout);
	zassert_false(sys_timepoint_expired(timepoint));
	remaining = sys_timepoint_timeout(timepoint);
	zassert_true(remaining.ticks <= timeout.ticks && remaining.ticks != 0);
	k_sleep(K_MSEC(1100));
	zassert_true(sys_timepoint_expired(timepoint));
	remaining = sys_timepoint_timeout(timepoint);
	zassert_true(K_TIMEOUT_EQ(remaining, K_NO_WAIT));
}

/**
 * @brief Verify that sys_timepoint_cmp() gives the correct order of two
 * timepoints, also for K_NO_WAIT and K_FOREVER.
 *
 * @details
 * sys_timepoint_cmp() returns 0 for equal timepoints. It returns a negative
 * value if the first timepoint is earlier, and a positive value if it is later.
 *
 * Test steps:
 * - Compare two copies of one timepoint, for K_NO_WAIT, K_FOREVER and
 *   K_MSEC(1).
 * - Compare the timepoints of K_NO_WAIT and K_MSEC(1).
 * - Compare the timepoints of K_MSEC(1) and K_FOREVER.
 * - Compare the timepoints of K_MSEC(100) and K_MSEC(200).
 * - Compare the timepoints of K_NO_WAIT and K_FOREVER.
 * - Do each comparison in both orders.
 *
 * Expected result:
 * - Two copies of one timepoint compare as equal.
 * - The timepoint of the shorter timeout compares as earlier, and the other one
 *   compares as later.
 *
 * @see sys_timepoint_calc(), sys_timepoint_cmp()
 */
ZTEST(timepoints, test_comparison)
{
	k_timepoint_t a, b;

	a = sys_timepoint_calc(K_NO_WAIT);
	b = a;
	zassert_true(sys_timepoint_cmp(a, b) == 0);
	zassert_true(sys_timepoint_cmp(b, a) == 0);

	a = sys_timepoint_calc(K_FOREVER);
	b = a;
	zassert_true(sys_timepoint_cmp(a, b) == 0);
	zassert_true(sys_timepoint_cmp(b, a) == 0);

	a = sys_timepoint_calc(K_NO_WAIT);
	b = sys_timepoint_calc(K_MSEC(1));
	zassert_true(sys_timepoint_cmp(a, b) < 0);
	zassert_true(sys_timepoint_cmp(b, a) > 0);

	a = sys_timepoint_calc(K_MSEC(1));
	b = sys_timepoint_calc(K_FOREVER);
	zassert_true(sys_timepoint_cmp(a, b) < 0);
	zassert_true(sys_timepoint_cmp(b, a) > 0);

	a = sys_timepoint_calc(K_MSEC(1));
	b = a;
	zassert_true(sys_timepoint_cmp(a, b) == 0);
	zassert_true(sys_timepoint_cmp(b, a) == 0);

	a = sys_timepoint_calc(K_MSEC(100));
	b = sys_timepoint_calc(K_MSEC(200));
	zassert_true(sys_timepoint_cmp(a, b) < 0);
	zassert_true(sys_timepoint_cmp(b, a) > 0);

	a = sys_timepoint_calc(K_NO_WAIT);
	b = sys_timepoint_calc(K_FOREVER);
	zassert_true(sys_timepoint_cmp(a, b) < 0);
	zassert_true(sys_timepoint_cmp(b, a) > 0);
}

ZTEST_SUITE(timepoints, NULL, NULL, NULL, NULL, NULL);
