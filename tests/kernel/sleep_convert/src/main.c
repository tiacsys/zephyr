/*
 * Copyright (c) 2026 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Exercise the tick to millisecond and tick to microsecond converters that
 * k_sleep() and k_usleep() use on their return value.
 *
 * Those converters pick one of several forms at compile time from
 * CONFIG_SYS_CLOCK_TICKS_PER_SEC, and the narrow ones are only safe because
 * of a clamp applied beforehand in the tick domain.  So the interesting
 * coverage is a whole build per tick rate, which tests.yaml provides:
 *
 *   100      ms: whole milliseconds per tick   us: whole microseconds per tick
 *   1000     ms: 1:1                           us: whole microseconds per tick
 *   10000    ms: whole ticks per millisecond   us: whole microseconds per tick
 *   32768    ms: split                         us: two stage split
 *   12345    ms: split                         us: two stage split
 *   1000000  ms: whole ticks per millisecond   us: 1:1
 *
 * The reference is the generic 64 bit converter plus an explicit saturation,
 * which is the contract these helpers must meet.  On a 64 bit target the
 * helpers are defined to be exactly that, so there the checks below only
 * really police the saturation, which is the point they still earn.
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

#define TICK_HZ ((uint32_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC)

/* Highest tick count k_sleep_ticks() can report, being the result of a
 * 32 bit subtraction narrowed to a positive int32_t.
 */
#define MAX_SLEEP_TICKS ((uint32_t)INT32_MAX)

static int32_t ref_ms(uint32_t ticks)
{
	uint64_t ms = k_ticks_to_ms_ceil64((uint64_t)ticks);

	return ms > (uint64_t)INT32_MAX ? INT32_MAX : (int32_t)ms;
}

static int32_t ref_us(uint32_t ticks)
{
	uint64_t us = k_ticks_to_us_ceil64((uint64_t)ticks);

	return us > (uint64_t)INT32_MAX ? INT32_MAX : (int32_t)us;
}

static void check(uint32_t ticks)
{
	zassert_equal(z_sleep_ticks_to_int32_ms(ticks), ref_ms(ticks),
		      "ms conversion of %u ticks: got %d, expected %d", ticks,
		      z_sleep_ticks_to_int32_ms(ticks), ref_ms(ticks));

	zassert_equal(z_sleep_ticks_to_int32_us(ticks), ref_us(ticks),
		      "us conversion of %u ticks: got %d, expected %d", ticks,
		      z_sleep_ticks_to_int32_us(ticks), ref_us(ticks));
}

/**
 * @brief Verify that the sleep tick conversions are correct for each tick count
 * from 0 to 10000.
 *
 * @details
 * In this range, the quotient and the remainder of the split conversion forms
 * both change at each step. check() compares z_sleep_ticks_to_int32_ms() and
 * z_sleep_ticks_to_int32_us() with a reference. The reference is the 64-bit
 * ceiling conversion with saturation at INT32_MAX.
 *
 * Test steps:
 * - For each tick count from 0 to 10000, call check().
 *
 * Expected result:
 * - For each tick count, both conversions are equal to the reference.
 *
 * @see z_sleep_ticks_to_int32_ms(), z_sleep_ticks_to_int32_us()
 */
ZTEST(sleep_convert, test_small_values)
{
	for (uint32_t t = 0; t <= 10000U; t++) {
		check(t);
	}
}

/**
 * @brief Verify that the sleep tick conversions are correct at sample points
 * through the full range of tick counts.
 *
 * @details
 * The stride 1048573 is coprime with the usual tick rates. Thus the samples
 * give many different remainders up to the top of the range. The samples are
 * sparse on purpose, because this test covers the full range.
 *
 * test_small_values and test_boundaries cover the cases that distinguish the
 * conversion forms. check() compares z_sleep_ticks_to_int32_ms() and
 * z_sleep_ticks_to_int32_us() with a reference. The reference is the 64-bit
 * ceiling conversion with saturation at INT32_MAX.
 *
 * Test steps:
 * - For each tick count from 0 to MAX_SLEEP_TICKS in steps of 1048573, call
 *   check().
 *
 * Expected result:
 * - For each tick count, both conversions are equal to the reference.
 *
 * @see z_sleep_ticks_to_int32_ms(), z_sleep_ticks_to_int32_us()
 */
ZTEST(sleep_convert, test_whole_range)
{
	for (uint64_t t = 0; t <= (uint64_t)MAX_SLEEP_TICKS; t += 1048573U) {
		check((uint32_t)t);
	}
}

/**
 * @brief Verify that the sleep tick conversions are correct at the tick counts
 * where the converters change behavior.
 *
 * @details
 * The converters change behavior near the tick rate, where the quotient and the
 * remainder split. They also change near the clamp bounds. check() compares
 * z_sleep_ticks_to_int32_ms() and z_sleep_ticks_to_int32_us() with a reference.
 * The reference is the 64-bit ceiling conversion with saturation at INT32_MAX.
 *
 * Test steps:
 * - Calculate the clamp bounds for milliseconds and microseconds from INT32_MAX
 *   and the tick rate.
 * - Call check() for 0, 1 and 2.
 * - Call check() near the tick rate, near two times the tick rate and near each
 *   clamp bound.
 * - Call check() for MAX_SLEEP_TICKS - 1 and MAX_SLEEP_TICKS.
 * - Skip each probe that is more than MAX_SLEEP_TICKS.
 *
 * Expected result:
 * - For each tick count, both conversions are equal to the reference.
 *
 * @see z_sleep_ticks_to_int32_ms(), z_sleep_ticks_to_int32_us()
 */
ZTEST(sleep_convert, test_boundaries)
{
	uint64_t clamp_ms = (uint64_t)INT32_MAX * TICK_HZ / MSEC_PER_SEC;
	uint64_t clamp_us = (uint64_t)INT32_MAX * TICK_HZ / USEC_PER_SEC;
	uint64_t probes[] = {
		0, 1, 2,
		TICK_HZ - 1, TICK_HZ, TICK_HZ + 1,
		(uint64_t)TICK_HZ * 2, (uint64_t)TICK_HZ * 2 + 1,
		clamp_ms - 1, clamp_ms, clamp_ms + 1,
		clamp_us - 1, clamp_us, clamp_us + 1,
		MAX_SLEEP_TICKS - 1, MAX_SLEEP_TICKS,
	};

	for (unsigned int i = 0; i < ARRAY_SIZE(probes); i++) {
		if (probes[i] > (uint64_t)MAX_SLEEP_TICKS) {
			continue;
		}
		check((uint32_t)probes[i]);
	}
}

/**
 * @brief Verify that the sleep tick conversions saturate at INT32_MAX past the
 * clamp bound and do not wrap.
 *
 * @details
 * The saturation lets the conversion itself use narrow arithmetic. If all tick
 * counts up to MAX_SLEEP_TICKS fit in the result, no saturation is possible.
 *
 * Test steps:
 * - Calculate the clamp bounds for milliseconds and microseconds.
 * - If a clamp bound is less than MAX_SLEEP_TICKS, convert MAX_SLEEP_TICKS and
 *   the clamp bound plus 1.
 * - If a clamp bound is not less than MAX_SLEEP_TICKS, convert MAX_SLEEP_TICKS
 *   only.
 *
 * Expected result:
 * - Past a clamp bound, the conversion returns INT32_MAX.
 * - Without a reachable clamp bound, the conversion of MAX_SLEEP_TICKS is not
 *   more than INT32_MAX.
 *
 * @see z_sleep_ticks_to_int32_ms(), z_sleep_ticks_to_int32_us()
 */
ZTEST(sleep_convert, test_saturation)
{
	uint64_t clamp_ms = (uint64_t)INT32_MAX * TICK_HZ / MSEC_PER_SEC;
	uint64_t clamp_us = (uint64_t)INT32_MAX * TICK_HZ / USEC_PER_SEC;

	if (clamp_ms < (uint64_t)MAX_SLEEP_TICKS) {
		zassert_equal(z_sleep_ticks_to_int32_ms(MAX_SLEEP_TICKS), INT32_MAX,
			      "ms conversion failed to saturate");
		zassert_equal(z_sleep_ticks_to_int32_ms((uint32_t)clamp_ms + 1), INT32_MAX,
			      "ms conversion failed to saturate just past the bound");
	} else {
		/* every reachable tick count is representable in milliseconds */
		zassert_true(z_sleep_ticks_to_int32_ms(MAX_SLEEP_TICKS) <= INT32_MAX);
	}

	if (clamp_us < (uint64_t)MAX_SLEEP_TICKS) {
		zassert_equal(z_sleep_ticks_to_int32_us(MAX_SLEEP_TICKS), INT32_MAX,
			      "us conversion failed to saturate");
		zassert_equal(z_sleep_ticks_to_int32_us((uint32_t)clamp_us + 1), INT32_MAX,
			      "us conversion failed to saturate just past the bound");
	} else {
		zassert_true(z_sleep_ticks_to_int32_us(MAX_SLEEP_TICKS) <= INT32_MAX);
	}
}

/**
 * @brief Verify that the sleep tick conversions never round down.
 *
 * @details
 * k_sleep() and k_usleep() return the time that is left. A sleep for this time
 * must cover the ticks that were left. Thus the result, converted back to
 * ticks, must not be less than the tick count.
 *
 * Test steps:
 * - For each tick count from 1 to 2000, convert the count to milliseconds and
 *   to microseconds.
 * - Convert each result back to ticks with k_ms_to_ticks_floor64() and
 *   k_us_to_ticks_floor64().
 *
 * Expected result:
 * - Each result in ticks is equal to or more than the tick count, or the result
 *   is INT32_MAX.
 *
 * @see z_sleep_ticks_to_int32_ms(), z_sleep_ticks_to_int32_us()
 */
ZTEST(sleep_convert, test_rounds_up)
{
	for (uint32_t t = 1; t <= 2000U; t++) {
		int32_t ms = z_sleep_ticks_to_int32_ms(t);
		int32_t us = z_sleep_ticks_to_int32_us(t);

		zassert_true((uint64_t)k_ms_to_ticks_floor64(ms) >= t || ms == INT32_MAX,
			     "%d ms is short of %u ticks", ms, t);
		zassert_true((uint64_t)k_us_to_ticks_floor64(us) >= t || us == INT32_MAX,
			     "%d us is short of %u ticks", us, t);
	}
}

/* Defined in cpp_build.cpp: exists so the header is compiled as C++ too. */
extern void sleep_convert_cpp_build(void);

/**
 * @brief Verify that the inline sleep API compiles as C++ with warnings as
 * errors.
 *
 * @details
 * The sleep API is a set of inline functions in a header. C++ rejects a
 * narrowing conversion in a braced initializer, such as Z_TIMEOUT_TICKS_INIT(),
 * that C accepts. The test has no assertions.
 *
 * cpp_build.cpp calls the sleep API, and the build compiles it as C++ with
 * CONFIG_COMPILER_WARNINGS_AS_ERRORS. The function calls k_sleep(), k_msleep(),
 * k_usleep(), k_sleep_ticks() and the two conversion helpers with zero values.
 *
 * Test steps:
 * - Call sleep_convert_cpp_build().
 *
 * Expected result:
 * - The C++ file compiles without warnings.
 * - sleep_convert_cpp_build() returns.
 *
 * @see k_sleep(), k_msleep(), k_usleep(), k_sleep_ticks()
 */
ZTEST(sleep_convert, test_cpp_build)
{
	sleep_convert_cpp_build();
}

ZTEST_SUITE(sleep_convert, NULL, NULL, NULL, NULL, NULL);
