/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Navimatix GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(controller_test, CONFIG_STEPPER_LOG_LEVEL);

struct accel_controller_fixture {
	const struct device *dev;
	stepper_ctrl_event_callback_t callback;
};

struct k_poll_signal stepper_ctrl_signal;
struct k_poll_event stepper_ctrl_event;
void *user_data_received;

#define STEPPER_CTRL_LOW_SPEED_INTERVAL  666666
#define STEPPER_CTRL_HIGH_SPEED_INTERVAL 444444
#define STEPPER_CTRL_LOW_SPEED           1500
#define STEPPER_CTRL_HIGH_SPEED          2250
#define STEPPER_CTRL_ACCELERATION        1500
#define STEPPER_CTRL_DECELERATION        3000
// TODO: Test for different ramp parameters

#define STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS         1000
#define STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_HALF_MS    500
#define STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS      500
#define STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS 250

#define STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS         500
#define STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_HALF_MS    250
#define STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS      250
#define STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS 125

#define STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW               750
#define STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW_LOWER_HALF    187
#define STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW_UPPER_HALF    563
#define STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH            938
#define STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF 422
#define STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_UPPER_HALF 513

#define STEPPER_CTRL_DECEL_STEPS_LOW_TO_0               375
#define STEPPER_CTRL_DECEL_STEPS_LOW_TO_0_LOWER_HALF    94
#define STEPPER_CTRL_DECEL_STEPS_LOW_TO_0_UPPER_HALF    281
#define STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW            469
#define STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_LOWER_HALF 212
#define STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_UPPER_HALF 257

#define STEPPER_CTRL_STEPS_LOW_1_SEC  1500
#define STEPPER_CTRL_STEPS_HIGH_1_SEC 2250

#define TOLERANCE_LOW  (1.0 - ((double)CONFIG_STEPPER_CTRL_TEST_STEPPING_TOLERANCE) / 100.0)
#define TOLERANCE_HIGH (1.0 + ((double)CONFIG_STEPPER_CTRL_TEST_STEPPING_TOLERANCE) / 100.0)

#define STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position)                           \
	{                                                                                          \
		int position = 0;                                                                  \
		zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));             \
		zassert_between_inclusive(                                                         \
			position, (int)(target_position * TOLERANCE_LOW),                          \
			(int)(target_position * TOLERANCE_HIGH),                                   \
			"Actual position is %d, but should be between %d and %d", position,        \
			(int)(target_position * TOLERANCE_LOW),                                    \
			(int)(target_position * TOLERANCE_HIGH));                                  \
	}

#define STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)                           \
	{                                                                                          \
		int position = 0;                                                                  \
		zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));             \
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

const struct stepper_ctrl_ramp low_speed_ramp = {
	.acceleration_max = STEPPER_CTRL_ACCELERATION,
	.deceleration_max = STEPPER_CTRL_DECELERATION,
	.speed_max = STEPPER_CTRL_LOW_SPEED,
};

const struct stepper_ctrl_ramp high_speed_ramp = {
	.acceleration_max = STEPPER_CTRL_ACCELERATION,
	.deceleration_max = STEPPER_CTRL_DECELERATION,
	.speed_max = STEPPER_CTRL_HIGH_SPEED,
};

static void accel_controller_print_event_callback(const struct device *dev,
						  enum stepper_ctrl_event event, void *user_data)
{
	const struct device *dev_callback = user_data;
	user_data_received = user_data;

	switch (event) {
	case STEPPER_CTRL_EVENT_STEPS_COMPLETED:
		k_poll_signal_raise(&stepper_ctrl_signal, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
		break;
	case STEPPER_CTRL_EVENT_LEFT_END_STOP_DETECTED:
		k_poll_signal_raise(&stepper_ctrl_signal,
				    STEPPER_CTRL_EVENT_LEFT_END_STOP_DETECTED);
		break;
	case STEPPER_CTRL_EVENT_RIGHT_END_STOP_DETECTED:
		k_poll_signal_raise(&stepper_ctrl_signal,
				    STEPPER_CTRL_EVENT_RIGHT_END_STOP_DETECTED);
		break;
	case STEPPER_CTRL_EVENT_STOPPED:
		k_poll_signal_raise(&stepper_ctrl_signal, STEPPER_CTRL_EVENT_STOPPED);
		break;
	default:
		break;
	}

	LOG_DBG("Event %d, %s called for %s, expected for %s\n", event, __func__,
		dev_callback->name, dev->name);
}

static void *accel_controller_setup(void)
{
	static struct accel_controller_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_ALIAS(stepper)),
		.callback = accel_controller_print_event_callback,
	};

	k_poll_signal_init(&stepper_ctrl_signal);
	k_poll_event_init(&stepper_ctrl_event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &stepper_ctrl_signal);
	// zassert_ok(
	// 	stepper_ctrl_set_microstep_interval(fixture.dev, STEPPER_CTRL_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_ctrl_configure_ramp(fixture.dev, &low_speed_ramp));

	zassert_not_null(fixture.dev);
	zassert_equal(stepper_ctrl_set_event_cb(fixture.dev, fixture.callback, (void *)fixture.dev),
		      0, "Failed to set event callback");
	return &fixture;
}

static void accel_controller_before(void *f)
{
	struct accel_controller_fixture *fixture = f;

	zassert_ok(stepper_ctrl_move_by(fixture->dev, 0));
	/* Sleep to account for event handling in another thread. */
	k_msleep(20);
	(void)stepper_ctrl_set_reference_position(fixture->dev, 0);

	k_poll_signal_reset(&stepper_ctrl_signal);

	user_data_received = NULL;
}

ZTEST_SUITE(accel_controller, NULL, accel_controller_setup, accel_controller_before, NULL, NULL);

ZTEST_F(accel_controller, test_set_reference_position)
{
	int position = 1;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, 0, "Actual position should be 0 but is %d", position);
	zassert_ok(stepper_ctrl_set_reference_position(fixture->dev, 12));
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, 12, "Actual position should be 12 but is %d", position);
}

ZTEST_F(accel_controller, test_set_reference_position_fails_while_moving)
{
	// zassert_ok(
	// 	stepper_ctrl_set_microstep_interval(fixture->dev, STEPPER_CTRL_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(10);
	int ret = stepper_ctrl_set_reference_position(fixture->dev, 12);
	zassert_equal(
		ret, -EBUSY,
		"Attempting to set referenc position while moving should return %d but returned %d",
		-EBUSY, ret);
}

ZTEST_F(accel_controller, test_is_moving)
{
	bool moving = true;
	// zassert_ok(
	// 	stepper_ctrl_set_microstep_interval(fixture->dev, STEPPER_CTRL_LOW_SPEED_INTERVAL));
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));

	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(20);
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_true(moving, "Stepper should be moving after run function, but is not");
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 0));
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 80000));
	k_msleep(20);
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_true(moving, "Stepper should be moving after move_by function, but is not");
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 0));
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	zassert_ok(stepper_ctrl_move_to(fixture->dev, 80000));
	k_msleep(20);
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_true(moving, "Stepper should be moving after move_to function, but is not");
}

ZTEST_F(accel_controller, test_run_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 10;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS + 100);
	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_run_negative_direction)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 10);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS + 100);
	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_run_positive_direction_from_higher_speed)
{
	int target_position =
		STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		STEPPER_CTRL_STEPS_HIGH_1_SEC / 10 + STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW +
		STEPPER_CTRL_STEPS_LOW_1_SEC / 10;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS + 100);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_run_negative_direction_from_higher_speed)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		  STEPPER_CTRL_STEPS_HIGH_1_SEC / 10 + STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW +
		  STEPPER_CTRL_STEPS_LOW_1_SEC / 10);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS + 100);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_run_positive_direction_from_lower_speed)
{
	int target_position =
		STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 10 +
		STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH + STEPPER_CTRL_STEPS_HIGH_1_SEC / 10;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS + 100);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_run_negative_direction_from_lower_speed)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 10 +
		  STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH + STEPPER_CTRL_STEPS_HIGH_1_SEC / 10);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS + 100);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_run_direction_change_while_moving_positive_to_negative_fails)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW_LOWER_HALF;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_HALF_MS / 2);
	int ret = stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE);
	zassert_equal(ret, -EINVAL,
		      "Direction change should fall with return code %d but has return code %d",
		      -EINVAL, ret);

	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_HALF_MS / 2);
	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_run_direction_change_while_moving_negative_to_positive_fails)
{
	int target_position = -(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW_LOWER_HALF);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_HALF_MS / 2);
	int ret = stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE);
	zassert_equal(ret, -EINVAL,
		      "Direction change should fall with return code %d but has return code %d",
		      -EINVAL, ret);

	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_HALF_MS / 2);
	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_stop_positive_direction)
{
	int32_t target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
				  STEPPER_CTRL_STEPS_LOW_1_SEC / 10 +
				  STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS + 100);
	zassert_ok(stepper_ctrl_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STOPPED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));
	bool moving = true;
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	int position1;
	int position2;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position1));
	k_msleep(100);
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position2));
	zassert_equal(position1, position2,
		      "Position should not have changed after stopping, but it did");
	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_stop_negative_direction)
{
	int32_t target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 10 +
		  STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS + 100);
	zassert_ok(stepper_ctrl_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STOPPED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));
	bool moving = true;
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving, but is");
	int position1;
	int position2;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position1));
	k_msleep(100);
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position2));
	zassert_equal(position1, position2,
		      "Position should not have changed after stopping, but it did");
	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_positive_direction)
{
	int target_position = 2000;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 2000));

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(2083 + CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_by_negative_direction)
{
	int target_position = -2000;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, -2000));

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(2083 + CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_by_positive_direction_insufficient_steps_for_full_speed)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, target_position));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_by_negative_direction_insufficient_steps_for_full_speed)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, target_position));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_by_positive_direction_insufficient_steps_while_moving_fails)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 2;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_ctrl_move_by(fixture->dev, STEPPER_CTRL_DECEL_STEPS_LOW_TO_0 / 2);
	zassert_equal(ret, -EINVAL,
		      "move_by with insufficient steps to stop should fail with return code %d but "
		      "returned %d",
		      -EINVAL, ret);
	k_msleep(500);
	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller,
	test_move_by_positive_direction_insufficient_steps_for_full_speed_while_moving_higher_speed)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
			      STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF +
			      STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_LOWER_HALF +
			      STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev,
					STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF +
						STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_LOWER_HALF +
						STEPPER_CTRL_DECEL_STEPS_LOW_TO_0));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));
	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_negative_direction_insufficient_steps_while_moving_fails)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 2);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_ctrl_move_by(fixture->dev, -(STEPPER_CTRL_DECEL_STEPS_LOW_TO_0 / 2));
	zassert_equal(ret, -EINVAL,
		      "move_by with insufficient steps to stop should fail with return code %d but "
		      "returned %d",
		      -EINVAL, ret);
	k_msleep(500);
	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(accel_controller,
	test_move_by_negative_direction_insufficient_steps_for_full_speed_while_moving_higher_speed)
{
	int target_position = -(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
				STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF +
				STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_LOWER_HALF +
				STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev,
					-(STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF +
					  STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_LOWER_HALF +
					  STEPPER_CTRL_DECEL_STEPS_LOW_TO_0)));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));
	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_changing_ramp_configuration_fails_while_moving)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC +
			      STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, target_position));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp);
	zassert_equal(ret, -EBUSY,
		      "configure_ramp should fail with %d while moving but returned %d", -EBUSY,
		      ret);

	/* Wait for the estimated end time of case of the microstep interval change going through.
	 * Only then start polling for the steps completed event.
	 */
	k_msleep(1300);
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(200 + CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_by_direction_change_while_moving_positive_to_negative_fails)
{
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(100);
	int ret = stepper_ctrl_move_by(fixture->dev, -30000);
	zassert_equal(ret, -EINVAL,
		      "Direction change should fall with return code %d but has return code %d",
		      -EINVAL, ret);

	int position1 = 0;
	int position2 = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position1));
	k_msleep(100);
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position2));
	zassert(position1 < position2, "Stepper should have continued to run but did not.");
}

ZTEST_F(accel_controller, test_move_by_direction_change_while_moving_negative_to_positive_fails)
{
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(100);
	int ret = stepper_ctrl_move_by(fixture->dev, 30000);
	zassert_equal(ret, -EINVAL,
		      "Direction change should fall with return code %d but has return code %d",
		      -EINVAL, ret);

	int position1 = 0;
	int position2 = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position1));
	k_msleep(100);
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position2));
	zassert(position1 > position2, "Stepper should have continued to run but did not.");
}

ZTEST_F(accel_controller, test_move_by_0_steps_immediate_stop_positive_direction)
{
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(200);
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 0));

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED, K_MSEC(20));
	bool moving = true;
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving after stopping, but it does.");
	int position1;
	int position2;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position1));
	k_msleep(100);
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position2));
	zassert_equal(position1, position2, "Position should not have changed, but it did.");
}

ZTEST_F(accel_controller, test_move_by_0_steps_immediate_stop_negative_direction)
{
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(200);
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 0));

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED, K_MSEC(20));
	bool moving = true;
	zassert_ok(stepper_ctrl_is_moving(fixture->dev, &moving));
	zassert_false(moving, "Stepper should not be moving after stopping, but it does.");
	int position1;
	int position2;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position1));
	k_msleep(100);
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position2));
	zassert_equal(position1, position2, "Position should not have changed, but it did.");
}

ZTEST_F(accel_controller, test_move_by_fails_while_moving_positioning_mode)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 2 +
			      STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, target_position));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_ctrl_move_by(fixture->dev, STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
							     STEPPER_CTRL_STEPS_LOW_1_SEC +
							     STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_equal(ret, -EBUSY, "move_by should fail with %d while moving but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(
		stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STEPS_COMPLETED,
		K_MSEC(STEPPER_CTRL_STEPS_LOW_1_SEC / 2 + STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
		       CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_by_to_run_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
							      STEPPER_CTRL_STEPS_LOW_1_SEC / 10 +
							      STEPPER_CTRL_DECEL_STEPS_LOW_TO_0));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE);
	k_msleep(1000);

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_to_run_negative_direction)
{
	int target_position = -(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, -(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
							STEPPER_CTRL_STEPS_LOW_1_SEC / 10 +
							STEPPER_CTRL_DECEL_STEPS_LOW_TO_0)));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE);
	k_msleep(1000);

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_to_positive_direction)
{
	int target_position = 2000;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_to(fixture->dev, 2000));

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(2083 + CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_to_negative_direction)
{
	int target_position = -2000;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_to(fixture->dev, -2000));

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(2083 + CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_to_positive_direction_insufficient_steps_for_full_speed)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_to(fixture->dev, target_position));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_to_negative_direction_insufficient_steps_for_full_speed)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_to(fixture->dev, target_position));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_to_changing_ramp_configuration_fails_while_moving)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC +
			      STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_to(fixture->dev, target_position));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp);
	zassert_equal(ret, -EBUSY,
		      "configure_ramp should fail with %d while moving but returned %d", -EBUSY,
		      ret);

	/* Wait for the estimated end time of case of the microstep interval change going through.
	 * Only then start polling for the steps completed event.
	 */
	k_msleep(1300);
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(200 + CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_move_to_fails_while_moving_velocity_mode)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_ctrl_move_to(fixture->dev, STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
							     STEPPER_CTRL_STEPS_LOW_1_SEC / 10 +
							     STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_equal(ret, -EBUSY, "move_to should fail with %d while moving but returned %d",
		      -EBUSY, ret);
	k_msleep(500 + STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS);
	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_to_fails_while_moving_positioning_mode)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_STEPS_LOW_1_SEC / 2 +
			      STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, target_position));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	int ret = stepper_ctrl_move_to(fixture->dev, STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
							     STEPPER_CTRL_STEPS_LOW_1_SEC +
							     STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_equal(ret, -EBUSY, "move_to should fail with %d while moving but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(500 + STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	int position = 0;
	zassert_ok(stepper_ctrl_get_actual_position(fixture->dev, &position));
	zassert_equal(position, target_position, "Position should be %d but is %d", target_position,
		      position);
}

ZTEST_F(accel_controller, test_run_acceleration_to_acceleration_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
			      STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
			      STEPPER_CTRL_STEPS_HIGH_1_SEC / 10;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS / 2);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS / 2 +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_run_acceleration_to_acceleration_negative_direction)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		  STEPPER_CTRL_STEPS_HIGH_1_SEC / 10);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS / 2);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS / 2 +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_run_acceleration_to_deceleration_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
			      STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF +
			      STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_LOWER_HALF +
			      STEPPER_CTRL_STEPS_LOW_1_SEC / 10;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_run_acceleration_to_deceleration_negative_direction)
{
	int target_position = -(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
				STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF +
				STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_LOWER_HALF +
				STEPPER_CTRL_STEPS_LOW_1_SEC / 10);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_run_deceleration_to_acceleration_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
			      STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
			      STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_UPPER_HALF +
			      STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_UPPER_HALF +
			      STEPPER_CTRL_STEPS_HIGH_1_SEC / 10;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_run_deceleration_to_acceleration_negative_direction)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		  STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_UPPER_HALF +
		  STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_UPPER_HALF +
		  STEPPER_CTRL_STEPS_HIGH_1_SEC / 10);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS + 100);

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_run_deceleration_to_deceleration_positive_direction)
{
	// TODO: Something is iffy with these two tests
	int target_position =
		STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW +
		STEPPER_CTRL_DECEL_STEPS_LOW_TO_0_UPPER_HALF + STEPPER_CTRL_STEPS_LOW_1_SEC / 10;
	const struct stepper_ctrl_ramp very_low_speed_ramp = {
		.acceleration_max = STEPPER_CTRL_ACCELERATION,
		.deceleration_max = STEPPER_CTRL_DECELERATION,
		.speed_max = STEPPER_CTRL_LOW_SPEED / 2,
	};
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &very_low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_HALF_MS + 200);

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_run_deceleration_to_deceleration_negative_direction)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		  STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW +
		  STEPPER_CTRL_DECEL_STEPS_LOW_TO_0_UPPER_HALF + STEPPER_CTRL_STEPS_LOW_1_SEC / 10);
	const struct stepper_ctrl_ramp very_low_speed_ramp = {
		.acceleration_max = STEPPER_CTRL_ACCELERATION,
		.deceleration_max = STEPPER_CTRL_DECELERATION,
		.speed_max = STEPPER_CTRL_LOW_SPEED / 2,
	};
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &very_low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_HALF_MS + 200);

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_acceleration_to_acceleration_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + 2000;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 2000));
	/* Wait time is calculaed based on the original acceleration parameters and might need to be
	 * adapted for other ones
	 */
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS + 97 +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_acceleration_to_acceleration_negative_direction)
{
	int target_position = -(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + 2000);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	int pos;
	stepper_ctrl_get_actual_position(fixture->dev, &pos);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, -2000));
	/* Wait time is calculaed based on the original acceleration parameters and might need to be
	 * adapted for other ones
	 */
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS + 97 +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_move_by_acceleration_to_deceleration_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
			      STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF + 1000;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 1000));
	/* Wait time is calculaed based on the original acceleration parameters and might need to be
	 * adapted for other ones
	 */
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS + 275 +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));
	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_acceleration_to_deceleration_negative_direction)
{
	int target_position = -(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
				STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH_LOWER_HALF + 1000);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, -1000));
	/* Wait time is calculaed based on the original acceleration parameters and might need to be
	 * adapted for other ones
	 */
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS + 275 +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));
	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_move_by_deceleration_to_acceleration_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
			      STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
			      STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_UPPER_HALF + 1500;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 1500));
	/* Wait time is calculaed based on the original acceleration parameters and might need to be
	 * adapted for other ones
	 */
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS + 178 +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_deceleration_to_acceleration_negative_direction)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		  STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_UPPER_HALF + 1500);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_move_by(fixture->dev, -1500));
	/* Wait time is calculaed based on the original acceleration parameters and might need to be
	 * adapted for other ones
	 */
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_MS + 178 +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_move_by_deceleration_to_deceleration_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW +
			      STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
			      STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_UPPER_HALF + 1000;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS);
	zassert_ok(stepper_ctrl_move_by(fixture->dev, 825));
	/* Wait time is calculaed based on the original acceleration parameters and might need to be
	 * adapted for other ones
	 */
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS + 275 +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_move_by_deceleration_to_deceleration_negative_direction)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		  STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW_UPPER_HALF + 1000);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS);
	zassert_ok(stepper_ctrl_move_by(fixture->dev, -1000));
	/* Wait time is calculaed based on the original acceleration parameters and might need to be
	 * adapted for other ones
	 */
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event,
			      STEPPER_CTRL_EVENT_STEPS_COMPLETED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS + 275 +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_stop_from_acceleration_positive_direction)
{
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_ctrl_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STOPPED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_stop_from_acceleration_negative_direction)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_ctrl_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STOPPED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_stop_from_deceleration_positive_direction)
{
	int target_position =
		STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS);
	zassert_ok(stepper_ctrl_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STOPPED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_stop_from_deceleration_negative_direction)
{
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_ACCEL_STEPS_LOW_TO_HIGH +
		  STEPPER_CTRL_DECEL_STEPS_HIGH_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &high_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS +
		 STEPPER_CTRL_ACCEL_TIMEOUT_LOW_TO_HIGH_MS);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	k_msleep(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS);
	zassert_ok(stepper_ctrl_stop(fixture->dev));
	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STOPPED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_HIGH_TO_LOW_HALF_MS +
				     STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position)
}

ZTEST_F(accel_controller, test_stop_movement_commands_fail_positive_direction)
{
	int ret = 0;
	int target_position = STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0;
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_ctrl_stop(fixture->dev));

	ret = stepper_ctrl_move_by(fixture->dev, 25000);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call move_by while stopping should return %d but returned %d",
		      -EBUSY, ret);

	ret = stepper_ctrl_move_to(fixture->dev, 30000);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call move_to while stopping should return %d but returned %d",
		      -EBUSY, ret);

	ret = stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_POSITIVE);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call run while stopping should return %d but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STOPPED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_POSITIVE(fixture, target_position);
}

ZTEST_F(accel_controller, test_stop_movement_commands_fail_negative_direction)
{
	int ret = 0;
	int target_position =
		-(STEPPER_CTRL_ACCEL_STEPS_0_TO_LOW + STEPPER_CTRL_DECEL_STEPS_LOW_TO_0);
	zassert_ok(stepper_ctrl_configure_ramp(fixture->dev, &low_speed_ramp));
	zassert_ok(stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE));
	k_msleep(STEPPER_CTRL_ACCEL_TIMEOUT_0_TO_LOW_MS);
	zassert_ok(stepper_ctrl_stop(fixture->dev));

	ret = stepper_ctrl_move_by(fixture->dev, -25000);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call move_by while stopping should return %d but returned %d",
		      -EBUSY, ret);

	ret = stepper_ctrl_move_to(fixture->dev, -30000);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call move_to while stopping should return %d but returned %d",
		      -EBUSY, ret);

	ret = stepper_ctrl_run(fixture->dev, STEPPER_CTRL_DIRECTION_NEGATIVE);
	zassert_equal(ret, -EBUSY,
		      "Attempting to call run while stopping should return %d but returned %d",
		      -EBUSY, ret);

	POLL_AND_CHECK_SIGNAL(stepper_ctrl_signal, stepper_ctrl_event, STEPPER_CTRL_EVENT_STOPPED,
			      K_MSEC(STEPPER_CTRL_DECEL_TIMEOUT_LOW_TO_0_MS +
				     CONFIG_STEPPER_CTRL_TEST_TIMEOUT_TOLERANCE_MS));

	STEPPER_CTRL_POSITION_COMPARE_NEGATIVE(fixture, target_position);
}
