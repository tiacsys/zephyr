/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Navimatix GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/stepper.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cz_accel_controller_test, CONFIG_STEPPER_LOG_LEVEL);

struct cz_accel_controller_fixture {
	const struct device *dev;
	stepper_event_callback_t callback;
};

struct k_poll_signal stepper_signal;
struct k_poll_event stepper_event;
void *user_data_received;

#define STEPPER_LOW_SPEED_INTERVAL          66666
#define STEPPER_HIGH_SPEED_INTERVAL         44444
#define STEPPER_TIMEOUT_0_TO_LOW_MS         1000
#define STEPPER_TIMEOUT_0_TO_LOW_HALF_MS    500
#define STEPPER_TIMEOUT_LOW_TO_HIGH_MS      500
#define STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS 250
#define STEPPER_TIMEOUT_TOLERANCE_MS        400
#define TOLERANCE_LOW                       (1.0 - ((double)CONFIG_STEPPER_TEST_STEPPING_TOLERANCE) / 100.0)
#define TOLERANCE_HIGH                      (1.0 + ((double)CONFIG_STEPPER_TEST_STEPPING_TOLERANCE) / 100.0)

#define STEPPER_STEPS_0_TO_LOW               7500
#define STEPPER_STEPS_0_TO_LOW_LOWER_HALF    1875
#define STEPPER_STEPS_0_TO_LOW_UPPER_HALF    5625
#define STEPPER_STEPS_LOW_TO_HIGH            9375
#define STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF 4219
#define STEPPER_STEPS_LOW_TO_HIGH_UPPER_HALF 5156
#define STEPPER_STEPS_LOW_1_SEC              15000
#define STEPPER_STEPS_HIGH_1_SEC             22500

#define STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position)                                \
	{                                                                                          \
		int position = 0;                                                                  \
		zassert_ok(stepper_get_actual_position(fixture->dev, &position));                  \
		zassert_between_inclusive(                                                         \
			position, (int)(target_position * TOLERANCE_LOW),                          \
			(int)(target_position * TOLERANCE_HIGH),                                   \
			"Actual position is %d, but should be between %d and %d", position,        \
			(int)(target_position * TOLERANCE_LOW),                                    \
			(int)(target_position * TOLERANCE_HIGH));                                  \
	}

#define STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)                                \
	{                                                                                          \
		int position = 0;                                                                  \
		zassert_ok(stepper_get_actual_position(fixture->dev, &position));                  \
		zassert_between_inclusive(                                                         \
			position, (int)(target_position * TOLERANCE_HIGH),                         \
			(int)(target_position * TOLERANCE_LOW),                                    \
			"Actual position is %d, but should be between %d and %d", position,        \
			(int)(target_position * TOLERANCE_HIGH),                                   \
			(int)(target_position * TOLERANCE_LOW));                                   \
	}

#define POLL_AND_CHECK_SIGNAL(signal, event, expected_event, timeout)                              \
	({                                                                                         \
		do {                                                                               \
			(void)k_poll(&(event), 1, timeout);                                        \
			unsigned int signaled;                                                     \
			int result;                                                                \
			k_poll_signal_check(&(signal), &signaled, &result);                        \
			zassert_equal(signaled, 1, "Signal not set");                              \
			zassert_equal(result, (expected_event), "Wrong Signal Set");               \
		} while (0);                                                                       \
	})

static void cz_accel_controller_print_event_callback(const struct device *dev,
						     enum stepper_event event, void *user_data)
{
	const struct device *dev_callback = user_data;
	user_data_received = user_data;

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
	case STEPPER_EVENT_STOPPED:
		k_poll_signal_raise(&stepper_signal, STEPPER_EVENT_STOPPED);
		break;
	default:
		break;
	}

	LOG_DBG("Event %d, %s called for %s, expected for %s\n", event, __func__,
		dev_callback->name, dev->name);
}

static void *cz_accel_controller_setup(void)
{
	static struct cz_accel_controller_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_ALIAS(stepper)),
		.callback = cz_accel_controller_print_event_callback,
	};

	k_poll_signal_init(&stepper_signal);
	k_poll_event_init(&stepper_event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &stepper_signal);
	zassert_ok(stepper_set_microstep_interval(fixture.dev, STEPPER_LOW_SPEED_INTERVAL));

	zassert_not_null(fixture.dev);
	zassert_equal(
		stepper_set_event_callback(fixture.dev, fixture.callback, (void *)fixture.dev), 0,
		"Failed to set event callback");
	return &fixture;
}

static void cz_accel_controller_before(void *f)
{
	struct cz_accel_controller_fixture *fixture = f;
	zassert_ok(stepper_move_by(fixture->dev, 0));
	(void)stepper_set_reference_position(fixture->dev, 0);

	k_msleep(1); // Sleep to account for event handling in another thread

	k_poll_signal_reset(&stepper_signal);

	user_data_received = NULL;
}

static void cz_accel_controller_after(void *f)
{
	struct cz_accel_controller_fixture *fixture = f;
	zassert_ok(stepper_move_by(fixture->dev, 0));

	k_msleep(1); // Sleep to account for event handling in another thread
}

ZTEST_SUITE(cz_accel_controller, NULL, cz_accel_controller_setup, cz_accel_controller_before,
	    cz_accel_controller_after, NULL);

ZTEST_F(cz_accel_controller, test_set_reference_position)
{
	int position = 1;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, 0, "Actual position should be 0 but is %d", position);
	zassert_ok(stepper_set_reference_position(fixture->dev, 12));
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, 12, "Actual position should be 12 but is %d", position);
}

ZTEST_F(cz_accel_controller, test_set_reference_position_fails_while_moving)
{
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(100);
	int ret = stepper_set_reference_position(fixture->dev, 12);
	zassert_equal(
		ret, -EBUSY,
		"Attempting to set referenc position while moving should return %d but returned %d",
		-EBUSY, ret);
}

ZTEST_F(cz_accel_controller, test_is_moving)
{
	bool moving = true;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));

	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(20);
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_true(moving, "Stepper should be moving after run function, but is not");
	zassert_ok(stepper_move_by(fixture->dev, 0));
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	zassert_ok(stepper_move_by(fixture->dev, 80000));
	k_msleep(20);
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_true(moving, "Stepper should be moving after move_by function, but is not");
	zassert_ok(stepper_move_by(fixture->dev, 0));
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	zassert_ok(stepper_move_to(fixture->dev, 80000));
	k_msleep(20);
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_true(moving, "Stepper should be moving after move_to function, but is not");
}

ZTEST_F(cz_accel_controller, test_run_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000);
	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_run_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000);
	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_run_positive_direction_from_higher_speed)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
			      STEPPER_STEPS_HIGH_1_SEC + STEPPER_STEPS_LOW_TO_HIGH +
			      STEPPER_STEPS_LOW_1_SEC;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_run_negative_direction_from_higher_speed)
{
	int target_position =
		-(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH + STEPPER_STEPS_HIGH_1_SEC +
		  STEPPER_STEPS_LOW_TO_HIGH + STEPPER_STEPS_LOW_1_SEC);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_run_positive_direction_from_lower_speed)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC +
			      STEPPER_STEPS_LOW_TO_HIGH + STEPPER_STEPS_HIGH_1_SEC;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_run_negative_direction_from_lower_speed)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC +
				STEPPER_STEPS_LOW_TO_HIGH + STEPPER_STEPS_HIGH_1_SEC);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_run_direction_change_while_moving_positive_to_negative_fails)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	zassert_equal(ret, -EINVAL,
		      "Direction change should fall with return code %d but has return code %d",
		      -EINVAL, ret);

	k_msleep(500);
	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_run_direction_change_while_moving_negative_to_positive_fails)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	zassert_equal(ret, -EINVAL,
		      "Direction change should fall with return code %d but has return code %d",
		      -EINVAL, ret);

	k_msleep(500);
	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_stop_positive_direction)
{
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 100);
	zassert_ok(stepper_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STOPPED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));
	bool moving = true;
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	int position1;
	int position2;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position1));
	k_msleep(1000);
	zassert_ok(stepper_get_actual_position(fixture->dev, &position2));
	zassert_equal(position1, position2,
		      "Position should not have changed after stopping, but it did");
}

ZTEST_F(cz_accel_controller, test_stop_negative_direction)
{
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 100);
	zassert_ok(stepper_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STOPPED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));
	bool moving = true;
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	int position1;
	int position2;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position1));
	k_msleep(1000);
	zassert_ok(stepper_get_actual_position(fixture->dev, &position2));
	zassert_equal(position1, position2,
		      "Position should not have changed after stopping, but it did");
}

ZTEST_F(cz_accel_controller, test_move_by_positive_direction)
{
	int target_position = 20000;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, 20000));

	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(2333 + STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_by_negative_direction)
{
	int target_position = -20000;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, -20000));

	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(2333 + STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_by_positive_direction_insufficient_steps)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, target_position));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_0_TO_LOW_MS +
				     STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_by_negative_direction_insufficient_steps)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_0_TO_LOW);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, target_position));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_0_TO_LOW_MS +
				     STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_by_positive_direction_insufficient_steps_while_moving_fails)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC / 2;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_move_by(fixture->dev, STEPPER_STEPS_0_TO_LOW / 2);
	zassert_equal(
		ret, -EINVAL,
		"move_by with insufficient steps should fail with return code %d but returned %d",
		-EINVAL, ret);
	k_msleep(500);
	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller,
	test_move_by_positive_direction_insufficient_steps_while_moving_higher_speed)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF +
			      STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF +
							 STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF +
							 STEPPER_STEPS_0_TO_LOW));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));
	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_by_negative_direction_insufficient_steps_while_moving_fails)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC / 2);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_move_by(fixture->dev, -(STEPPER_STEPS_0_TO_LOW / 2));
	zassert_equal(
		ret, -EINVAL,
		"move_by with insufficient steps should fail with return code %d but returned %d",
		-EINVAL, ret);
	k_msleep(500);
	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller,
	test_move_by_negative_direction_insufficient_steps_while_moving_higher_speed)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF +
				STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF + STEPPER_STEPS_0_TO_LOW);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, -(STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF +
						   STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF +
						   STEPPER_STEPS_0_TO_LOW)));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));
	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

// ZTEST_F(cz_accel_controller, test_move_by_positive_direction_lower_speed_while_moving)
// {
// 	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
// 			      STEPPER_STEPS_HIGH_1_SEC + 25000;
// 	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
// 	zassert_ok(stepper_move_by(fixture->dev, target_position));
// 	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);
// 	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));

// 	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
// 			      K_MSEC(2040 + STEPPER_TIMEOUT_TOLERANCE_MS));

// 	int position = 0;
// 	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
// 	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
// 		      position);
// }

// ZTEST_F(cz_accel_controller, test_move_by_negative_direction_lower_speed_while_moving)
// {
// 	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
// 				STEPPER_STEPS_HIGH_1_SEC + 25000);
// 	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
// 	zassert_ok(stepper_move_by(fixture->dev, target_position));
// 	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);
// 	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));

// 	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
// 			      K_MSEC(2040 + STEPPER_TIMEOUT_TOLERANCE_MS));

// 	int position = 0;
// 	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
// 	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
// 		      position);
// }

// ZTEST_F(cz_accel_controller, test_move_by_positive_direction_higher_speed_while_moving)
// {
// 	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC + 35000;
// 	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
// 	zassert_ok(stepper_move_by(fixture->dev, target_position));
// 	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000);
// 	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));

// 	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
// 			      K_MSEC(2400 + STEPPER_TIMEOUT_TOLERANCE_MS));

// 	int position = 0;
// 	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
// 	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
// 		      position);
// }

// ZTEST_F(cz_accel_controller, test_move_by_negative_direction_higher_speed_while_moving)
// {
// 	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC + 35000);
// 	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
// 	zassert_ok(stepper_move_by(fixture->dev, target_position));
// 	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000);
// 	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));

// 	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
// 			      K_MSEC(2400 + STEPPER_TIMEOUT_TOLERANCE_MS));

// 	int position = 0;
// 	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
// 	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
// 		      position);
// }

ZTEST_F(cz_accel_controller, test_move_by_changing_microstep_interval_fails)
{
	int target_position =
		STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, target_position));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL);
	zassert_equal(ret, -EBUSY,
		      "set_microstep_interval should fail with %d while moving but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(
		stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
		K_MSEC(1000 + STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_by_direction_change_while_moving_positive_to_negative_fails)
{
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_move_by(fixture->dev, -30000);
	zassert_equal(ret, -EINVAL,
		      "Direction change should fall with return code %d but has return code %d",
		      -EINVAL, ret);

	int position1 = 0;
	int position2 = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position1));
	k_msleep(500);
	zassert_ok(stepper_get_actual_position(fixture->dev, &position2));
	zassert(position1 < position2, "Stepper should have continued to run but did not.");
}

ZTEST_F(cz_accel_controller, test_move_by_direction_change_while_moving_negative_to_positive_fails)
{
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_move_by(fixture->dev, 30000);
	zassert_equal(ret, -EINVAL,
		      "Direction change should fall with return code %d but has return code %d",
		      -EINVAL, ret);

	int position1 = 0;
	int position2 = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position1));
	k_msleep(500);
	zassert_ok(stepper_get_actual_position(fixture->dev, &position2));
	zassert(position1 > position2, "Stepper should have continued to run but did not.");
}

ZTEST_F(cz_accel_controller, test_move_by_0_steps_immediate_stop_positive_direction)
{
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_move_by(fixture->dev, 0));

	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_TOLERANCE_MS));
	bool moving = true;
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving after stopping, but it does.");
	int position1;
	int position2;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position1));
	k_msleep(1000);
	zassert_ok(stepper_get_actual_position(fixture->dev, &position2));
	zassert_equal(position1, position2, "Position should not have changed, but it did.");
}

ZTEST_F(cz_accel_controller, test_move_by_0_steps_immediate_stop_negative_direction)
{
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_move_by(fixture->dev, 0));

	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_TOLERANCE_MS));
	bool moving = true;
	zassert_ok(stepper_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving after stopping, but it does.");
	int position1;
	int position2;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position1));
	k_msleep(1000);
	zassert_ok(stepper_get_actual_position(fixture->dev, &position2));
	zassert_equal(position1, position2, "Position should not have changed, but it did.");
}

ZTEST_F(cz_accel_controller, test_move_by_fails_while_moving_positioning_mode)
{
	int target_position =
		STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC / 2 + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, target_position));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_move_by(fixture->dev, STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC +
							STEPPER_STEPS_0_TO_LOW);
	zassert_equal(ret, -EBUSY, "move_by should fail with %d while moving but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(
		stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
		K_MSEC(500 + STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_by_to_run_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC * 2;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, STEPPER_STEPS_0_TO_LOW +
							 STEPPER_STEPS_LOW_1_SEC / 2 +
							 STEPPER_STEPS_0_TO_LOW));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	k_msleep(2000);

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_by_to_run_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC * 2);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(
		fixture->dev,
		-(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC / 2 + STEPPER_STEPS_0_TO_LOW)));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	k_msleep(2000);

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_to_positive_direction)
{
	int target_position = 20000;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_to(fixture->dev, 20000));

	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(2333 + STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_to_negative_direction)
{
	int target_position = -20000;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_to(fixture->dev, -20000));

	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(2333 + STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_to_positive_direction_insufficient_steps)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_to(fixture->dev, target_position));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_0_TO_LOW_MS +
				     STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_move_to_negative_direction_insufficient_steps)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_0_TO_LOW);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_to(fixture->dev, target_position));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_0_TO_LOW_MS +
				     STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

//  ZTEST_F(cz_accel_controller, test_move_to_positive_direction_lower_speed_while_moving)
//  {
//  	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
//  			      STEPPER_STEPS_HIGH_1_SEC + 25000;
//  	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
//  	zassert_ok(stepper_move_to(fixture->dev, target_position));
//  	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);
//  	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));

// 	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
// 			      K_MSEC(2040 + STEPPER_TIMEOUT_TOLERANCE_MS));

// 	int position = 0;
// 	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
// 	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
// 		      position);
// }

ZTEST_F(cz_accel_controller, test_move_to_changing_microstep_interval_fails)
{
	int target_position =
		STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_to(fixture->dev, target_position));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL);
	zassert_equal(ret, -EBUSY,
		      "set_microstep_interval should fail with %d while moving but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(
		stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
		K_MSEC(1000 + STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

//  ZTEST_F(cz_accel_controller, test_move_to_negative_direction_lower_speed_while_moving)
//  {
//  	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
//  				STEPPER_STEPS_HIGH_1_SEC + 25000);
//  	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
//  	zassert_ok(stepper_move_to(fixture->dev, target_position));
//  	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);
//  	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));

// 	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
// 			      K_MSEC(2040 + STEPPER_TIMEOUT_TOLERANCE_MS));

// 	int position = 0;
// 	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
// 	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
// 		      position);
// }

//  ZTEST_F(cz_accel_controller, test_move_to_positive_direction_higher_speed_while_moving)
//  {
//  	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC + 35000;
//  	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
//  	zassert_ok(stepper_move_to(fixture->dev, target_position));
//  	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000);
//  	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));

// 	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
// 			      K_MSEC(2400 + STEPPER_TIMEOUT_TOLERANCE_MS));

// 	int position = 0;
// 	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
// 	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
// 		      position);
// }

//  ZTEST_F(cz_accel_controller, test_move_to_negative_direction_higher_speed_while_moving)
//  {
//  	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC + 35000);
//  	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
//  	zassert_ok(stepper_move_to(fixture->dev, target_position));
//  	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000);
//  	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));

// 	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
// 			      K_MSEC(2400 + STEPPER_TIMEOUT_TOLERANCE_MS));

// 	int position = 0;
// 	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
// 	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
// 		      position);
// }

ZTEST_F(cz_accel_controller, test_move_to_fails_while_moving_velocity_mode)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC * 1.5;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret =
		stepper_move_to(fixture->dev, STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC / 2 +
						      STEPPER_STEPS_0_TO_LOW);
	zassert_equal(ret, -EBUSY, "move_to should fail with %d while moving but returned %d",
		      -EBUSY, ret);
	k_msleep(500 + STEPPER_TIMEOUT_0_TO_LOW_MS);
	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_to_fails_while_moving_positioning_mode)
{
	int target_position =
		STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC / 2 + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, target_position));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_move_to(fixture->dev, STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_1_SEC +
							STEPPER_STEPS_0_TO_LOW);
	zassert_equal(ret, -EBUSY, "move_to should fail with %d while moving but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(
		stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
		K_MSEC(500 + STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(cz_accel_controller, test_run_acceleration_to_acceleration_positive_direction)
{
	int target_position =
		STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH + STEPPER_STEPS_HIGH_1_SEC;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS / 2);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS / 2 + STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_run_acceleration_to_acceleration_negative_direction)
{
	int target_position =
		-(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH + STEPPER_STEPS_HIGH_1_SEC);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS / 2);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS / 2 + STEPPER_TIMEOUT_LOW_TO_HIGH_MS + 1000);

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_run_acceleration_to_deceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF +
			      STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF + STEPPER_STEPS_LOW_1_SEC;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS + 1000);

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_run_acceleration_to_deceleration_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF +
				STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF + STEPPER_STEPS_LOW_1_SEC);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS + 1000);

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_run_deceleration_to_acceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
			      STEPPER_STEPS_LOW_TO_HIGH_UPPER_HALF +
			      STEPPER_STEPS_LOW_TO_HIGH_UPPER_HALF + STEPPER_STEPS_HIGH_1_SEC;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS + 1000);

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_run_deceleration_to_acceleration_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
				STEPPER_STEPS_LOW_TO_HIGH_UPPER_HALF +
				STEPPER_STEPS_LOW_TO_HIGH_UPPER_HALF + STEPPER_STEPS_HIGH_1_SEC);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS + 1000);

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_run_deceleration_to_deceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
			      STEPPER_STEPS_LOW_TO_HIGH + STEPPER_STEPS_0_TO_LOW_UPPER_HALF +
			      STEPPER_STEPS_LOW_1_SEC / 2;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL * 2));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_HALF_MS + 1000);

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_run_deceleration_to_deceleration_negative_direction)
{
	int target_position =
		-(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH + STEPPER_STEPS_LOW_TO_HIGH +
		  STEPPER_STEPS_0_TO_LOW_UPPER_HALF + STEPPER_STEPS_LOW_1_SEC / 2);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL * 2));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_HALF_MS + 1000);

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_by_acceleration_to_acceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + 35000;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, 35000));
	// Wait time is calculaed based on the original acceleration parameters and might need to be
	// adapted for other ones
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_LOW_TO_HIGH_MS +
				     STEPPER_TIMEOUT_LOW_TO_HIGH_MS + STEPPER_TIMEOUT_0_TO_LOW_MS +
				     390 + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_by_acceleration_to_acceleration_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + 35000);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, -35000));
	// Wait time is calculaed based on the original acceleration parameters and might need to be
	// adapted for other ones
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_LOW_TO_HIGH_MS +
				     STEPPER_TIMEOUT_LOW_TO_HIGH_MS + STEPPER_TIMEOUT_0_TO_LOW_MS +
				     390 + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_move_by_acceleration_to_deceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF + 25000;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, 25000));
	// Wait time is calculaed based on the original acceleration parameters and might need to be
	// adapted for other ones
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_TIMEOUT_0_TO_LOW_MS + 890 +
				     STEPPER_TIMEOUT_TOLERANCE_MS));
	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_by_acceleration_to_deceleration_negative_direction)
{
	int target_position =
		-(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH_LOWER_HALF + 25000);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, -25000));
	// Wait time is calculaed based on the original acceleration parameters and might need to be
	// adapted for other ones
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_TIMEOUT_0_TO_LOW_MS + 890 +
				     STEPPER_TIMEOUT_TOLERANCE_MS));
	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_move_by_deceleration_to_acceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
			      STEPPER_STEPS_LOW_TO_HIGH_UPPER_HALF + 35000;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, 35000));
	// Wait time is calculaed based on the original acceleration parameters and might need to be
	// adapted for other ones
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS +
				     580 + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_by_deceleration_to_acceleration_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
				STEPPER_STEPS_LOW_TO_HIGH_UPPER_HALF + 35000);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_move_by(fixture->dev, -35000));
	// Wait time is calculaed based on the original acceleration parameters and might need to be
	// adapted for other ones
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS +
				     580 + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_move_by_deceleration_to_deceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
			      STEPPER_STEPS_LOW_TO_HIGH + 15000;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL * 2));
	zassert_ok(stepper_move_by(fixture->dev, 15000));
	// Wait time is calculaed based on the original acceleration parameters and might need to be
	// adapted for other ones
	POLL_AND_CHECK_SIGNAL(
		stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
		K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000 + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_move_by_deceleration_to_deceleration_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
				STEPPER_STEPS_LOW_TO_HIGH + 15000);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL * 2));
	zassert_ok(stepper_move_by(fixture->dev, -15000));
	// Wait time is calculaed based on the original acceleration parameters and might need to be
	// adapted for other ones
	POLL_AND_CHECK_SIGNAL(
		stepper_signal, stepper_event, STEPPER_EVENT_STEPS_COMPLETED,
		K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + 1000 + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_stop_from_acceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STOPPED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_stop_from_acceleration_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_0_TO_LOW);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STOPPED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_stop_from_deceleration_positive_direction)
{
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
			      STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STOPPED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_stop_from_deceleration_negative_direction)
{
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH +
				STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_LOW_TO_HIGH);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_HIGH_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	k_msleep(STEPPER_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STOPPED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(cz_accel_controller, test_stop_movement_commands_fail_positive_direction)
{
	int ret = 0;
	int target_position = STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_0_TO_LOW;
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_stop(fixture->dev));

	ret = stepper_move_by(fixture->dev, 25000);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call move_by while stopping should return %d but returned %d",
		      -EBUSY, ret);

	ret = stepper_move_to(fixture->dev, 30000);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call move_to while stopping should return %d but returned %d",
		      -EBUSY, ret);

	ret = stepper_run(fixture->dev, STEPPER_DIRECTION_POSITIVE);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call run while stopping should return %d but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STOPPED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(cz_accel_controller, test_stop_movement_commands_fail_negative_direction)
{
	int ret = 0;
	int target_position = -(STEPPER_STEPS_0_TO_LOW + STEPPER_STEPS_0_TO_LOW);
	zassert_ok(stepper_set_microstep_interval(fixture->dev, STEPPER_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_stop(fixture->dev));

	ret = stepper_move_by(fixture->dev, -25000);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call move_by while stopping should return %d but returned %d",
		      -EBUSY, ret);

	ret = stepper_move_to(fixture->dev, -30000);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call move_to while stopping should return %d but returned %d",
		      -EBUSY, ret);

	ret = stepper_run(fixture->dev, STEPPER_DIRECTION_NEGATIVE);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call run while stopping should return %d but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(stepper_signal, stepper_event, STEPPER_EVENT_STOPPED,
			      K_MSEC(STEPPER_TIMEOUT_0_TO_LOW_MS + STEPPER_TIMEOUT_TOLERANCE_MS));

	STEPPER_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}
