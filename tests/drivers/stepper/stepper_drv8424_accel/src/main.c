/*
 * Copyright 2024 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/drivers/gpio.h"
#include "zephyr/kernel.h"
#include "zephyr/ztest_assert.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/stepper.h>

#define ACCELERATION 50

struct drv8424_accel_fixture {
	const struct device *dev;
	stepper_event_callback_t callback;
};

struct k_poll_signal stepper_signal;
struct k_poll_event stepper_event;

static void drv8424_accel_print_event_callback(const struct device *dev, enum stepper_event event,
					       void *dummy)
{
	switch (event) {
	case STEPPER_EVENT_STEPS_COMPLETED:
		k_poll_signal_raise(&stepper_signal, STEPPER_EVENT_STEPS_COMPLETED);
		break;
	case STEPPER_EVENT_LEFT_END_STOP_DETECTED:
		k_poll_signal_raise(&stepper_signal, STEPPER_EVENT_LEFT_END_STOP_DETECTED);
		break;
	case STEPPER_EVENT_RIGHT_END_STOP_DETECTED:
		k_poll_signal_raise(&stepper_signal, STEPPER_EVENT_RIGHT_END_STOP_DETECTED);
		break;
	case STEPPER_EVENT_STALL_DETECTED:
		k_poll_signal_raise(&stepper_signal, STEPPER_EVENT_STALL_DETECTED);
		break;
	default:
		break;
	}
}

static void *drv8424_accel_setup(void)
{
	static struct drv8424_accel_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(drv8424)),
		.callback = drv8424_accel_print_event_callback,
	};

	k_poll_signal_init(&stepper_signal);
	k_poll_event_init(&stepper_event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &stepper_signal);

	zassert_not_null(fixture.dev);
	return &fixture;
}

static void drv8424_accel_before(void *f)
{
	struct drv8424_accel_fixture *fixture = f;
	(void)stepper_enable(fixture->dev, false);
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_reference_position(fixture->dev, 0);
	(void)stepper_set_micro_step_res(fixture->dev, 1);
	k_poll_signal_reset(&stepper_signal);
}

static void drv8424_accel_after(void *f)
{
	struct drv8424_accel_fixture *fixture = f;
	(void)stepper_enable(fixture->dev, false);
	(void)stepper_enable(fixture->dev, true);
}

static void test_move_by_different_speeds(struct drv8424_accel_fixture *fixture,
					  int32_t speed_start, int32_t speed_test, int32_t steps,
					  int32_t pos_target)
{
	int direction = STEPPER_DIRECTION_POSITIVE;
	if (steps < 0) {
		direction = STEPPER_DIRECTION_NEGATIVE;
	}

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, speed_test);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, direction, speed_start);
	k_busy_wait(1000000 * ceil(speed_start * 1.0 / ACCELERATION) + 100000);
	(void)stepper_move_by(fixture->dev, steps);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;
	int32_t pos = 0;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);

	zassert_true(pos_target - 1 <= pos && pos <= pos_target + 1,
		     "Current position should be between %d and %d but is %d", pos_target - 1,
		     pos_target + 1, pos);
}

static void test_move_to_different_speeds(struct drv8424_accel_fixture *fixture,
					  int32_t speed_start, int32_t speed_test,
					  int32_t pos_target)
{
	int direction = STEPPER_DIRECTION_POSITIVE;
	if (pos_target < 0) {
		direction = STEPPER_DIRECTION_NEGATIVE;
	}

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, speed_test);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, direction, speed_start);
	k_busy_wait(1000000 * ceil(speed_start * 1.0 / ACCELERATION) + 100000);
	(void)stepper_move_to(fixture->dev, pos_target);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;
	int32_t pos = 0;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, pos_target, "Target position should be %d but is %d", pos_target, pos);
}

static void test_run_different_speeds(struct drv8424_accel_fixture *fixture, int32_t speed_start,
				      int32_t speed_test, int32_t pos_target, uint32_t t_start,
				      uint32_t t_test, int direction)
{
	int32_t pos = 0;
	int start_pos;
	stepper_get_actual_position(fixture->dev, &start_pos);

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_run(fixture->dev, direction, speed_start);
	k_busy_wait(t_start);
	(void)stepper_run(fixture->dev, direction, speed_test);
	k_busy_wait(t_test);

	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_true(IN_RANGE(pos, pos_target - 5, pos_target + 5),
		     "Current position should be between %d and %d but is %d", pos_target - 5,
		     pos_target + 5, pos);
}

ZTEST_SUITE(drv8424_accel, NULL, drv8424_accel_setup, drv8424_accel_before, drv8424_accel_after,
	    NULL);

ZTEST_F(drv8424_accel, test_run_positive_direction_correct_position_from_zero_speed)
{
	test_run_different_speeds(fixture, 0, 50, 30, 0, 1100000, STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(drv8424_accel, test_run_negative_direction_correct_position_from_zero_speed)
{
	test_run_different_speeds(fixture, 0, 50, -30, 0, 1100000, STEPPER_DIRECTION_NEGATIVE);
}

ZTEST_F(drv8424_accel, test_run_positive_direction_correct_position_to_zero_speed)
{
	test_run_different_speeds(fixture, 50, 0, 50, 1000000, 1100000, STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(drv8424_accel, test_run_negative_direction_correct_position_to_zero_speed)
{
	test_run_different_speeds(fixture, 50, 0, -50, 1000000, 1100000,
				  STEPPER_DIRECTION_NEGATIVE);
}

ZTEST_F(drv8424_accel, test_run_positive_direction_to_zero_speed_signals)
{
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE, 50);
	k_busy_wait(1100000);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE, 0);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
}

ZTEST_F(drv8424_accel, test_run_negative_direction_to_zero_speed_signals)
{
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE, 50);
	k_busy_wait(1100000);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE, 0);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
}

ZTEST_F(drv8424_accel, test_run_positive_direction_correct_position_from_lower_speed)
{
	test_run_different_speeds(fixture, 50, 100, 110, 1000000, 1100000,
				  STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(drv8424_accel, test_run_negative_direction_correct_position_from_lower_speed)
{
	test_run_different_speeds(fixture, 50, 100, -110, 1000000, 1100000,
				  STEPPER_DIRECTION_NEGATIVE);
}
ZTEST_F(drv8424_accel, test_run_positive_direction_correct_position_from_higher_speed)
{
	test_run_different_speeds(fixture, 100, 50, 180, 2000000, 1100000,
				  STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(drv8424_accel, test_run_negative_direction_correct_position_from_higher_speed)
{
	test_run_different_speeds(fixture, 100, 50, -180, 2000000, 1100000,
				  STEPPER_DIRECTION_NEGATIVE);
}

ZTEST_F(drv8424_accel, test_run_positive_direction_correct_position_from_same_speed)
{
	test_run_different_speeds(fixture, 50, 50, 75, 1000000, 1000000,
				  STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(drv8424_accel, test_run_negative_direction_correct_position_from_same_speed)
{
	test_run_different_speeds(fixture, 50, 50, -75, 1000000, 1000000,
				  STEPPER_DIRECTION_NEGATIVE);
}

ZTEST_F(drv8424_accel, test_move_to_positive_direction_movement_from_zero_speed)
{
	test_move_to_different_speeds(fixture, 0, 50, 50);
}

ZTEST_F(drv8424_accel, test_move_to_negative_direction_movement_from_zero_speed)
{
	test_move_to_different_speeds(fixture, 0, 50, -50);
}

ZTEST_F(drv8424_accel, test_move_to_positive_direction_movement_from_same_speed)
{
	test_move_to_different_speeds(fixture, 50, 50, 80);
}

ZTEST_F(drv8424_accel, test_move_to_negative_direction_movement_from_same_speed)
{
	test_move_to_different_speeds(fixture, 50, 50, -80);
}

ZTEST_F(drv8424_accel, test_move_to_positive_direction_movement_from_lower_speed)
{
	test_move_to_different_speeds(fixture, 50, 100, 230);
}

ZTEST_F(drv8424_accel, test_move_to_negative_direction_movement_from_lower_speed)
{
	test_move_to_different_speeds(fixture, 50, 100, -230);
}

ZTEST_F(drv8424_accel, test_move_to_positive_direction_movement_from_higher_speed)
{
	test_move_to_different_speeds(fixture, 100, 50, 230);
}

ZTEST_F(drv8424_accel, test_move_to_negative_direction_movement_from_higher_speed)
{
	test_move_to_different_speeds(fixture, 100, 50, -230);
}

ZTEST_F(drv8424_accel, test_move_by_positive_direction_movement_from_zero_speed)
{
	test_move_by_different_speeds(fixture, 0, 50, 50, 50);
	uint32_t pos = 0;

	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 50, "Target position should be %d but is %d", 50, pos);
}

ZTEST_F(drv8424_accel, test_move_by_negative_direction_movement_from_zero_speed)
{
	test_move_by_different_speeds(fixture, 0, 50, -50, -50);
	uint32_t pos = 0;

	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, -50, "Target position should be %d but is %d", -50, pos);
}

ZTEST_F(drv8424_accel, test_move_by_positive_direction_movement_from_same_speed)
{
	test_move_by_different_speeds(fixture, 50, 50, 50, 80);
}

ZTEST_F(drv8424_accel, test_move_by_negative_direction_movement_from_same_speed)
{
	test_move_by_different_speeds(fixture, 50, 50, -50, -80);
}

ZTEST_F(drv8424_accel, test_move_by_positive_direction_movement_from_lower_speed)
{
	test_move_by_different_speeds(fixture, 50, 100, 200, 230);
}

ZTEST_F(drv8424_accel, test_move_by_negative_direction_movement_from_lower_speed)
{
	test_move_by_different_speeds(fixture, 50, 100, -200, -230);
}

ZTEST_F(drv8424_accel, test_move_by_positive_direction_movement_from_higher_speed)
{
	test_move_by_different_speeds(fixture, 100, 50, 100, 210);
}

ZTEST_F(drv8424_accel, test_move_by_negative_direction_movement_from_higher_speed)
{
	test_move_by_different_speeds(fixture, 100, 50, -100, -210);
}

ZTEST_F(drv8424_accel, test_run_negative_to_posive_direction_change)
{
	int ret = 0;
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE, 50);
	k_busy_wait(100000);
	ret = stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE, 100);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(drv8424_accel, test_run_positive_to_negative_direction_change)
{
	int ret = 0;
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE, 50);
	k_busy_wait(100000);
	ret = stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE, 100);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(drv8424_accel, test_move_to_negative_to_posive_direction_change)
{
	int ret = 0;
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	(void)stepper_move_to(fixture->dev, -50);
	k_busy_wait(100000);
	ret = stepper_move_to(fixture->dev, 50);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(drv8424_accel, test_move_to_positive_to_negative_direction_change)
{
	int ret = 0;
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	(void)stepper_move_to(fixture->dev, 50);
	k_busy_wait(100000);
	ret = stepper_move_to(fixture->dev, -50);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(drv8424_accel, test_move_by_negative_to_posive_direction_change)
{
	int ret = 0;
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	(void)stepper_move_by(fixture->dev, -50);
	k_busy_wait(100000);
	ret = stepper_move_by(fixture->dev, 50);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(drv8424_accel, test_move_by_positive_to_negative_direction_change)
{
	int ret = 0;
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	(void)stepper_move_by(fixture->dev, 50);
	k_busy_wait(100000);
	ret = stepper_move_by(fixture->dev, -50);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(drv8424_accel, test_move_to_positive_direction_to_low_position_difference)
{
	int ret = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE, 50);
	k_busy_wait(1100000);
	(void)stepper_set_max_velocity(fixture->dev, 100);
	ret = stepper_move_to(fixture->dev, 40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from lower to "
		      "higher velocity",
		      -EINVAL, ret);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	ret = stepper_move_to(fixture->dev, 40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from existing "
		      "velocity with same velocity",
		      -EINVAL, ret);
	(void)stepper_set_max_velocity(fixture->dev, 25);
	ret = stepper_move_to(fixture->dev, 40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from higher to "
		      "lower velocity",
		      -EINVAL, ret);
}

ZTEST_F(drv8424_accel, test_move_to_negative_direction_to_low_position_difference)
{
	int ret = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE, 50);
	k_busy_wait(1100000);
	(void)stepper_set_max_velocity(fixture->dev, 100);
	ret = stepper_move_to(fixture->dev, -40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from lower to "
		      "higher velocity",
		      -EINVAL, ret);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	ret = stepper_move_to(fixture->dev, -40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from existing "
		      "velocity with same velocity",
		      -EINVAL, ret);
	(void)stepper_set_max_velocity(fixture->dev, 25);
	ret = stepper_move_to(fixture->dev, -40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from higher to "
		      "lower velocity",
		      -EINVAL, ret);
}

ZTEST_F(drv8424_accel, test_move_by_positive_direction_to_low_position_difference)
{
	int ret = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE, 50);
	k_busy_wait(1100000);
	(void)stepper_set_max_velocity(fixture->dev, 100);
	ret = stepper_move_by(fixture->dev, 10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from lower to "
		      "higher velocity",
		      -EINVAL, ret);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	ret = stepper_move_by(fixture->dev, 10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from existing "
		      "velocity with same velocity",
		      -EINVAL, ret);
	(void)stepper_set_max_velocity(fixture->dev, 25);
	ret = stepper_move_by(fixture->dev, 10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from higher to "
		      "lower velocity",
		      -EINVAL, ret);
}

ZTEST_F(drv8424_accel, test_move_by_negative_direction_to_low_position_difference)
{
	int ret = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE, 50);
	k_busy_wait(1100000);
	(void)stepper_set_max_velocity(fixture->dev, 100);
	ret = stepper_move_by(fixture->dev, -10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from lower to "
		      "higher velocity",
		      -EINVAL, ret);
	(void)stepper_set_max_velocity(fixture->dev, 50);
	ret = stepper_move_by(fixture->dev, -10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from existing "
		      "velocity with same velocity",
		      -EINVAL, ret);
	(void)stepper_set_max_velocity(fixture->dev, 25);
	ret = stepper_move_by(fixture->dev, -10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from higher to "
		      "lower velocity",
		      -EINVAL, ret);
}
