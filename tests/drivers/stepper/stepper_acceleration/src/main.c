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
#define LOW_SPD_I    40000000
#define MED_SPD_I    20000000
#define HIGH_SPD_I   10000000

struct stepper_acceleration_fixture {
	const struct device *dev;
	stepper_event_callback_t callback;
};

struct k_poll_signal stepper_signal;
struct k_poll_event stepper_event;

static void stepper_acceleration_print_event_callback(const struct device *dev,
						      enum stepper_event event, void *dummy)
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

static void *stepper_acceleration_setup(void)
{
	static struct stepper_acceleration_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(stepper_motor)),
		.callback = stepper_acceleration_print_event_callback,
	};

	k_poll_signal_init(&stepper_signal);
	k_poll_event_init(&stepper_event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &stepper_signal);

	zassert_not_null(fixture.dev);
	return &fixture;
}

static void stepper_acceleration_before(void *f)
{
	struct stepper_acceleration_fixture *fixture = f;
	(void)stepper_enable(fixture->dev, false);
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_reference_position(fixture->dev, 0);
	k_poll_signal_reset(&stepper_signal);
}

static void stepper_acceleration_after(void *f)
{
	struct stepper_acceleration_fixture *fixture = f;
	(void)stepper_enable(fixture->dev, false);
	(void)stepper_enable(fixture->dev, true);
}

static void test_move_by_different_speeds(struct stepper_acceleration_fixture *fixture,
					  int32_t velocity_start, int32_t velocity_test,
					  int32_t steps, int32_t pos_target, uint32_t test_time_us)
{
	int direction = STEPPER_DIRECTION_POSITIVE;
	unsigned int signaled;
	int result;
	int32_t pos;
	uint32_t test_time;
	uint64_t interval_start = 0;
	uint64_t interval_test = 0;

	if (steps < 0) {
		direction = STEPPER_DIRECTION_NEGATIVE;
	}

	if (velocity_start != 0) {
		interval_start = NSEC_PER_SEC / velocity_start;
	}

	if (velocity_test != 0) {
		interval_test = NSEC_PER_SEC / velocity_test;
	}

	/* Add one interval time of both speeds to test_time to catch delays and algorithm
	 * inaccuracies
	 */
	test_time =
		test_time_us + (interval_start / NSEC_PER_USEC) + (interval_test / NSEC_PER_USEC);

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, interval_start);
	(void)stepper_run(fixture->dev, direction);
	/* Acceleration time + 1/10 second + first step delay*/
	k_usleep(1000000 * (uint32_t)ceil(velocity_start * 1.0 / ACCELERATION) + 100000 +
		 interval_start / NSEC_PER_USEC);
	(void)stepper_set_microstep_interval(fixture->dev, interval_test);
	(void)stepper_move_by(fixture->dev, steps);
	(void)k_poll(&stepper_event, 1, K_USEC(test_time));

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);

	zassert_true(IN_RANGE(pos, pos_target - 1, pos_target + 1),
		     "Current position should be between %d and %d but is %d", pos_target - 1,
		     pos_target + 1, pos);
}

static void test_move_to_different_speeds(struct stepper_acceleration_fixture *fixture,
					  uint32_t velocity_start, uint32_t velocity_test,
					  int32_t pos_target, uint32_t test_time_us)
{
	int direction = STEPPER_DIRECTION_POSITIVE;
	unsigned int signaled;
	int result;
	int32_t pos;
	uint32_t test_time;
	uint64_t interval_start = 0;
	uint64_t interval_test = 0;

	if (pos_target < 0) {
		direction = STEPPER_DIRECTION_NEGATIVE;
	}

	if (velocity_start != 0) {
		interval_start = NSEC_PER_SEC / velocity_start;
	}

	if (velocity_test != 0) {
		interval_test = NSEC_PER_SEC / velocity_test;
	}

	/* Add one interval time of both speeds to test_time to catch delays and algorithm
	 * inaccuracies
	 */
	test_time =
		test_time_us + (interval_start / NSEC_PER_USEC) + (interval_test / NSEC_PER_USEC);

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, interval_start);
	(void)stepper_run(fixture->dev, direction);
	/* Acceleration time + 1/10 second + first step delay*/
	k_usleep(1000000 * (uint32_t)ceil(velocity_start * 1.0 / ACCELERATION) + 100000 +
		 interval_start / NSEC_PER_USEC);
	(void)stepper_set_microstep_interval(fixture->dev, interval_test);
	(void)stepper_move_to(fixture->dev, pos_target);
	(void)k_poll(&stepper_event, 1, K_USEC(test_time));

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, pos_target, "Target position should be %d but is %d", pos_target, pos);
}

static void test_run_different_speeds(struct stepper_acceleration_fixture *fixture,
				      uint32_t velocity_start, uint32_t velocity_test,
				      int32_t pos_target, uint32_t t_start, uint32_t t_test,
				      int direction)
{
	int32_t pos;
	uint64_t interval_start = 0;
	uint64_t interval_test = 0;

	if (velocity_start != 0) {
		interval_start = NSEC_PER_SEC / velocity_start;
	}

	if (velocity_test != 0) {
		interval_test = NSEC_PER_SEC / velocity_test;
	}

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, interval_start);
	(void)stepper_run(fixture->dev, direction);
	k_usleep(t_start);
	(void)stepper_set_microstep_interval(fixture->dev, interval_test);
	(void)stepper_run(fixture->dev, direction);
	k_usleep(t_test);

	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_true(IN_RANGE(pos, pos_target - 2, pos_target + 2),
		     "Current position should be between %d and %d but is %d", pos_target - 2,
		     pos_target + 2, pos);
}

ZTEST_SUITE(stepper_acceleration, NULL, stepper_acceleration_setup, stepper_acceleration_before,
	    stepper_acceleration_after, NULL);

ZTEST_F(stepper_acceleration, test_run_positive_direction_correct_position_from_zero_speed)
{
	test_run_different_speeds(fixture, 0, 50, 30, 0, 1100000, STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(stepper_acceleration, test_run_negative_direction_correct_position_from_zero_speed)
{
	test_run_different_speeds(fixture, 0, 50, -30, 0, 1100000, STEPPER_DIRECTION_NEGATIVE);
}

ZTEST_F(stepper_acceleration, test_run_positive_direction_correct_position_to_zero_speed)
{
	test_run_different_speeds(fixture, 50, 0, 50, 1000000, 1100000, STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(stepper_acceleration, test_run_negative_direction_correct_position_to_zero_speed)
{
	test_run_different_speeds(fixture, 50, 0, -50, 1000000, 1100000,
				  STEPPER_DIRECTION_NEGATIVE);
}

ZTEST_F(stepper_acceleration, test_run_positive_direction_to_zero_speed_signals)
{
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	k_usleep(1100000);
	(void)stepper_set_microstep_interval(fixture->dev, 0);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	(void)k_poll(&stepper_event, 1, K_MSEC(1040));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
}

ZTEST_F(stepper_acceleration, test_run_negative_direction_to_zero_speed_signals)
{
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	k_usleep(1100000);
	(void)stepper_set_microstep_interval(fixture->dev, 0);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	(void)k_poll(&stepper_event, 1, K_MSEC(1040));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
}

ZTEST_F(stepper_acceleration, test_run_positive_direction_correct_position_from_lower_speed)
{
	test_run_different_speeds(fixture, 50, 100, 110, 1000000, 1100000,
				  STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(stepper_acceleration, test_run_negative_direction_correct_position_from_lower_speed)
{
	test_run_different_speeds(fixture, 50, 100, -110, 1000000, 1100000,
				  STEPPER_DIRECTION_NEGATIVE);
}
ZTEST_F(stepper_acceleration, test_run_positive_direction_correct_position_from_higher_speed)
{
	test_run_different_speeds(fixture, 100, 50, 190, 2100000, 1100000,
				  STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(stepper_acceleration, test_run_negative_direction_correct_position_from_higher_speed)
{
	test_run_different_speeds(fixture, 100, 50, -190, 2100000, 1100000,
				  STEPPER_DIRECTION_NEGATIVE);
}

ZTEST_F(stepper_acceleration, test_run_positive_direction_correct_position_from_same_speed)
{
	test_run_different_speeds(fixture, 50, 50, 75, 1000000, 1000000,
				  STEPPER_DIRECTION_POSITIVE);
}

ZTEST_F(stepper_acceleration, test_run_negative_direction_correct_position_from_same_speed)
{
	test_run_different_speeds(fixture, 50, 50, -75, 1000000, 1000000,
				  STEPPER_DIRECTION_NEGATIVE);
}

ZTEST_F(stepper_acceleration, test_move_to_positive_direction_movement_from_zero_speed)
{
	test_move_to_different_speeds(fixture, 0, 50, 50, 2000000);
}

ZTEST_F(stepper_acceleration, test_move_to_negative_direction_movement_from_zero_speed)
{
	test_move_to_different_speeds(fixture, 0, 50, -50, 2000000);
}

ZTEST_F(stepper_acceleration, test_move_to_positive_direction_movement_from_same_speed)
{
	test_move_to_different_speeds(fixture, 50, 50, 80, 1500000);
}

ZTEST_F(stepper_acceleration, test_move_to_negative_direction_movement_from_same_speed)
{
	test_move_to_different_speeds(fixture, 50, 50, -80, 1500000);
}

ZTEST_F(stepper_acceleration, test_move_to_positive_direction_movement_from_lower_speed)
{
	test_move_to_different_speeds(fixture, 50, 100, 230, 3250000);
}

ZTEST_F(stepper_acceleration, test_move_to_negative_direction_movement_from_lower_speed)
{
	test_move_to_different_speeds(fixture, 50, 100, -230, 3250000);
}

ZTEST_F(stepper_acceleration, test_move_to_positive_direction_movement_from_higher_speed)
{
	test_move_to_different_speeds(fixture, 100, 50, 230, 2400000);
}

ZTEST_F(stepper_acceleration, test_move_to_negative_direction_movement_from_higher_speed)
{
	test_move_to_different_speeds(fixture, 100, 50, -230, 2400000);
}

ZTEST_F(stepper_acceleration, test_move_by_positive_direction_movement_from_zero_speed)
{
	test_move_by_different_speeds(fixture, 0, 50, 50, 50, 2000000);
	uint32_t pos;

	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 50, "Target position should be %d but is %d", 50, pos);
}

ZTEST_F(stepper_acceleration, test_move_by_negative_direction_movement_from_zero_speed)
{
	test_move_by_different_speeds(fixture, 0, 50, -50, -50, 2000000);
	uint32_t pos;

	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, -50, "Target position should be %d but is %d", -50, pos);
}

ZTEST_F(stepper_acceleration, test_move_by_positive_direction_movement_from_same_speed)
{
	test_move_by_different_speeds(fixture, 50, 50, 50, 80, 1500000);
}

ZTEST_F(stepper_acceleration, test_move_by_negative_direction_movement_from_same_speed)
{
	test_move_by_different_speeds(fixture, 50, 50, -50, -80, 1500000);
}

ZTEST_F(stepper_acceleration, test_move_by_positive_direction_movement_from_lower_speed)
{
	test_move_by_different_speeds(fixture, 50, 100, 200, 230, 3250000);
}

ZTEST_F(stepper_acceleration, test_move_by_negative_direction_movement_from_lower_speed)
{
	test_move_by_different_speeds(fixture, 50, 100, -200, -230, 3250000);
}

ZTEST_F(stepper_acceleration, test_move_by_positive_direction_movement_from_higher_speed)
{
	test_move_by_different_speeds(fixture, 100, 50, 100, 210, 2000000);
}

ZTEST_F(stepper_acceleration, test_move_by_negative_direction_movement_from_higher_speed)
{
	test_move_by_different_speeds(fixture, 100, 50, -100, -210, 2000000);
}

ZTEST_F(stepper_acceleration, test_run_negative_to_posive_direction_change)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	k_usleep(100000);
	(void)stepper_set_microstep_interval(fixture->dev, HIGH_SPD_I);
	ret = stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(stepper_acceleration, test_run_positive_to_negative_direction_change)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	k_usleep(100000);
	(void)stepper_set_microstep_interval(fixture->dev, HIGH_SPD_I);
	ret = stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(stepper_acceleration, test_move_to_negative_to_posive_direction_change)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_to(fixture->dev, -50);
	k_usleep(100000);
	ret = stepper_move_to(fixture->dev, 50);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(stepper_acceleration, test_move_to_positive_to_negative_direction_change)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_to(fixture->dev, 50);
	k_usleep(100000);
	ret = stepper_move_to(fixture->dev, -50);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(stepper_acceleration, test_move_by_negative_to_posive_direction_change)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_by(fixture->dev, -50);
	k_usleep(100000);
	ret = stepper_move_by(fixture->dev, 50);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(stepper_acceleration, test_move_by_positive_to_negative_direction_change)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_by(fixture->dev, 50);
	k_usleep(100000);
	ret = stepper_move_by(fixture->dev, -50);
	zassert_equal(ret, -ENOTSUP, "Should return error code %d but returned %d", -ENOTSUP, ret);
}

ZTEST_F(stepper_acceleration, test_move_to_positive_direction_to_low_position_difference)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	k_usleep(1100000);
	(void)stepper_set_microstep_interval(fixture->dev, HIGH_SPD_I);
	ret = stepper_move_to(fixture->dev, 40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from lower to "
		      "higher velocity",
		      -EINVAL, ret);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	ret = stepper_move_to(fixture->dev, 40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from existing "
		      "velocity with same velocity",
		      -EINVAL, ret);
	(void)stepper_set_microstep_interval(fixture->dev, LOW_SPD_I);
	ret = stepper_move_to(fixture->dev, 40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from higher to "
		      "lower velocity",
		      -EINVAL, ret);
}

ZTEST_F(stepper_acceleration, test_move_to_negative_direction_to_low_position_difference)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	k_usleep(1100000);
	(void)stepper_set_microstep_interval(fixture->dev, HIGH_SPD_I);
	ret = stepper_move_to(fixture->dev, -40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from lower to "
		      "higher velocity",
		      -EINVAL, ret);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	ret = stepper_move_to(fixture->dev, -40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from existing "
		      "velocity with same velocity",
		      -EINVAL, ret);
	(void)stepper_set_microstep_interval(fixture->dev, LOW_SPD_I);
	ret = stepper_move_to(fixture->dev, -40);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from higher to "
		      "lower velocity",
		      -EINVAL, ret);
}

ZTEST_F(stepper_acceleration, test_move_by_positive_direction_to_low_position_difference)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	k_usleep(1100000);
	(void)stepper_set_microstep_interval(fixture->dev, HIGH_SPD_I);
	ret = stepper_move_by(fixture->dev, 10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from lower to "
		      "higher velocity",
		      -EINVAL, ret);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	ret = stepper_move_by(fixture->dev, 10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from existing "
		      "velocity with same velocity",
		      -EINVAL, ret);
	(void)stepper_set_microstep_interval(fixture->dev, LOW_SPD_I);
	ret = stepper_move_by(fixture->dev, 10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from higher to "
		      "lower velocity",
		      -EINVAL, ret);
}

ZTEST_F(stepper_acceleration, test_move_by_negative_direction_to_low_position_difference)
{
	int ret;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	k_usleep(1100000);
	(void)stepper_set_microstep_interval(fixture->dev, HIGH_SPD_I);
	ret = stepper_move_by(fixture->dev, -10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from lower to "
		      "higher velocity",
		      -EINVAL, ret);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	ret = stepper_move_by(fixture->dev, -10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from existing "
		      "velocity with same velocity",
		      -EINVAL, ret);
	(void)stepper_set_microstep_interval(fixture->dev, LOW_SPD_I);
	ret = stepper_move_by(fixture->dev, -10);
	zassert_equal(ret, -EINVAL,
		      "Should return error code %d but returned %d for move_to from higher to "
		      "lower velocity",
		      -EINVAL, ret);
}

ZTEST_F(stepper_acceleration, test_move_by_negative_to_positive_direction_change_when_stopped)
{

	unsigned int signaled;
	int result;
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_by(fixture->dev, -50);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));
	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");

	k_poll_signal_reset(&stepper_signal);
	(void)stepper_move_by(fixture->dev, 50);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);

	zassert_equal(0, pos, "Position should be 0 but is %d", pos);
}

ZTEST_F(stepper_acceleration, test_move_by_positive_to_negative_direction_change_when_stopped)
{

	unsigned int signaled;
	int result;
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_by(fixture->dev, 50);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));
	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");

	k_poll_signal_reset(&stepper_signal);
	(void)stepper_move_by(fixture->dev, -50);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);

	zassert_equal(0, pos, "Position should be 0 but is %d", pos);
}

ZTEST_F(stepper_acceleration, test_move_to_negative_to_positive_direction_change_when_stopped)
{

	unsigned int signaled;
	int result;
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_to(fixture->dev, -50);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));
	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");

	k_poll_signal_reset(&stepper_signal);
	(void)stepper_move_to(fixture->dev, 0);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);

	zassert_equal(0, pos, "Position should be 0 but is %d", pos);
}

ZTEST_F(stepper_acceleration, test_move_to_positive_to_negative_direction_change_when_stopped)
{

	unsigned int signaled;
	int result;
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_to(fixture->dev, 50);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));
	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");

	k_poll_signal_reset(&stepper_signal);
	(void)stepper_move_to(fixture->dev, 0);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);

	zassert_equal(0, pos, "Position should be 0 but is %d", pos);
}

ZTEST_F(stepper_acceleration, test_run_negative_to_positive_direction_change_when_stopped)
{

	unsigned int signaled;
	int result;
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	k_msleep(1000);
	(void)stepper_set_microstep_interval(fixture->dev, 0);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));
	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");

	k_poll_signal_reset(&stepper_signal);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	k_msleep(1000);
	(void)stepper_set_microstep_interval(fixture->dev, 0);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);

	zassert_true(IN_RANGE(pos, -2, 2), "Current position should be between %d and %d but is %d",
		     -2, 2, pos);
}

ZTEST_F(stepper_acceleration, test_run_positive_to_negative_direction_change_when_stopped)
{

	unsigned int signaled;
	int result;
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	k_msleep(1000);
	(void)stepper_set_microstep_interval(fixture->dev, 0);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));
	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");

	k_poll_signal_reset(&stepper_signal);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	k_msleep(1000);
	(void)stepper_set_microstep_interval(fixture->dev, 0);
	(void)stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	(void)k_poll(&stepper_event, 1, K_SECONDS(3));

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "No event detected");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED,
		      "Event was not STEPPER_EVENT_STEPS_COMPLETED event");
	(void)stepper_get_actual_position(fixture->dev, &pos);

	zassert_true(IN_RANGE(pos, -2, 2), "Current position should be between %d and %d but is %d",
		     -2, 2, pos);
}

ZTEST_F(stepper_acceleration, test_move_by_positive_direction_correct_deceleration_time)
{
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_by(fixture->dev, 60);
	(void)k_msleep(1700);
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 54, "Current position should be 54, but is %d", pos);
}

ZTEST_F(stepper_acceleration, test_move_by_negative_direction_correct_deceleration_time)
{
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_by(fixture->dev, -60);
	(void)k_msleep(1700);
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, -54, "Current position should be -54, but is %d", pos);
}

ZTEST_F(stepper_acceleration, test_move_to_positive_direction_correct_deceleration_time)
{
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_to(fixture->dev, 60);
	(void)k_msleep(1700);
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 54, "Current position should be 54, but is %d", pos);
}

ZTEST_F(stepper_acceleration, test_move_to_negative_direction_correct_deceleration_time)
{
	int pos;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_event_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_microstep_interval(fixture->dev, MED_SPD_I);
	(void)stepper_move_to(fixture->dev, -60);
	(void)k_msleep(1700);
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, -54, "Current position should be -54, but is %d", pos);
}
