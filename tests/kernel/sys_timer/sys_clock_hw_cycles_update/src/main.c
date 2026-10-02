/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/drivers/timer/system_timer.h>

/**
 * @brief Verify that an update to the current hardware cycle frequency does not
 * change the frequency.
 *
 * @details
 * z_sys_clock_hw_cycles_per_sec_update() sets the hardware cycle frequency of
 * the system timer at run time. An update to the current value must have no
 * effect.
 *
 * Test steps:
 * - Read the frequency with sys_clock_hw_cycles_per_sec().
 * - Call z_sys_clock_hw_cycles_per_sec_update() with this frequency.
 * - Read the frequency again.
 *
 * Expected result:
 * - The frequency is the same as before the update.
 *
 * @testid{TSPEC-SYSTIMER-004}
 * @draft
 * @see z_sys_clock_hw_cycles_per_sec_update(), sys_clock_hw_cycles_per_sec()
 */
ZTEST(sys_clock_hw_cycles_update, test_update_no_change_is_noop)
{
	uint32_t old_hz = sys_clock_hw_cycles_per_sec();

	z_sys_clock_hw_cycles_per_sec_update(old_hz);
	zassert_equal(sys_clock_hw_cycles_per_sec(), old_hz, "frequency changed unexpectedly");
}

/**
 * @brief Verify that z_sys_clock_hw_cycles_per_sec_update() ignores a frequency
 * of 0.
 *
 * @details
 * A hardware cycle frequency of 0 is not valid. The update function must keep
 * the current frequency.
 *
 * Test steps:
 * - Read the frequency with sys_clock_hw_cycles_per_sec().
 * - Call z_sys_clock_hw_cycles_per_sec_update() with 0.
 * - Read the frequency again.
 *
 * Expected result:
 * - The frequency is the same as before the update.
 *
 * @testid{TSPEC-SYSTIMER-005}
 * @draft
 * @see z_sys_clock_hw_cycles_per_sec_update(), sys_clock_hw_cycles_per_sec()
 */
ZTEST(sys_clock_hw_cycles_update, test_update_zero_is_ignored)
{
	uint32_t old_hz = sys_clock_hw_cycles_per_sec();

	z_sys_clock_hw_cycles_per_sec_update(0U);
	zassert_equal(sys_clock_hw_cycles_per_sec(), old_hz, "frequency changed unexpectedly");
}

/**
 * @brief Verify that sys_clock_hw_cycles_per_sec() returns the new frequency
 * after an update.
 *
 * @details
 * The test selects a new frequency that is different from the current
 * frequency.
 *
 * Test steps:
 * - Read the frequency with sys_clock_hw_cycles_per_sec().
 * - Select 1000000 Hz, or 1000001 Hz if the current frequency is 1000000 Hz.
 * - Call z_sys_clock_hw_cycles_per_sec_update() with the new frequency.
 * - Read the frequency again.
 *
 * Expected result:
 * - sys_clock_hw_cycles_per_sec() returns the new frequency.
 *
 * @testid{TSPEC-SYSTIMER-006}
 * @draft
 * @see z_sys_clock_hw_cycles_per_sec_update(), sys_clock_hw_cycles_per_sec()
 */
ZTEST(sys_clock_hw_cycles_update, test_update_changes_value_is_visible_via_getter)
{
	uint32_t old_hz = sys_clock_hw_cycles_per_sec();
	uint32_t new_hz = (old_hz == 1000000U) ? 1000001U : 1000000U;

	z_sys_clock_hw_cycles_per_sec_update(new_hz);
	zassert_equal(sys_clock_hw_cycles_per_sec(), new_hz, "frequency not updated");
}

ZTEST_SUITE(sys_clock_hw_cycles_update, NULL, NULL, NULL, NULL, NULL);
