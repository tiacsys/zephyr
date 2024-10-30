/*
 * Copyright 2024 TiaC Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/drivers/gpio.h"
#include "zephyr/kernel.h"
#include "zephyr/ztest_assert.h"
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/stepper.h>

struct stepper_fixture {
	const struct device *dev;
	stepper_event_callback_t callback;
};

struct k_poll_signal stepper_signal;
struct k_poll_event stepper_event;

struct gpio_dt_spec en_pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(drv8424), en_gpios, {0});
struct gpio_dt_spec slp_pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(drv8424), sleep_gpios, {0});

static void stepper_print_event_callback(const struct device *dev, enum stepper_event event,
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

static void *stepper_setup(void)
{
	static struct stepper_fixture fixture = {
		.dev = DEVICE_DT_GET(DT_NODELABEL(drv8424)),
		.callback = stepper_print_event_callback,
	};

	k_poll_signal_init(&stepper_signal);
	k_poll_event_init(&stepper_event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &stepper_signal);

	zassert_not_null(fixture.dev);
	return &fixture;
}

static void stepper_before(void *f)
{
	struct stepper_fixture *fixture = f;
	(void)stepper_set_actual_position(fixture->dev, 0);
	(void)stepper_set_micro_step_res(fixture->dev, 1);
	k_poll_signal_reset(&stepper_signal);
}

static void stepper_after(void *f)
{
	struct stepper_fixture *fixture = f;
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE, 0);
}

ZTEST_SUITE(stepper, NULL, stepper_setup, stepper_before, stepper_after, NULL);

ZTEST_F(stepper, test_micro_step_res_set)
{
	(void)stepper_set_micro_step_res(fixture->dev, 4);
	enum stepper_micro_step_resolution res;
	(void)stepper_get_micro_step_res(fixture->dev, &res);
	zassert_equal(res, 4, "Micro step resolution not set correctly, should be %d but is %d", 4,
		      res);
}

ZTEST_F(stepper, test_micro_step_res_set_incorrect_value_detection)
{
	int ret = 0;
	ret = stepper_set_micro_step_res(fixture->dev, 3);
	zassert_equal(ret, -EINVAL, "Command should fail with error %d but returned %d", -EINVAL,
		      ret);
}

ZTEST_F(stepper, test_actual_position_set)
{
	int32_t pos = 100u;
	(void)stepper_set_actual_position(fixture->dev, pos);
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 100u, "Actual position not set correctly");
}

ZTEST_F(stepper, test_enable_on_gpio_pins)
{
	int value = 0;
	(void)stepper_enable(fixture->dev, true);
	/* As sleep and enable pins are optional, check if they exist*/
	if (en_pin.port != NULL) {
		value = gpio_pin_get_dt(&en_pin);
		zassert_equal(value, 1, "Enable pin should be set");
	}
	if (slp_pin.port != NULL) {
		value = gpio_pin_get_dt(&slp_pin);
		zassert_equal(value, 0, "Sleep pin should not be set");
	}
}

ZTEST_F(stepper, test_enable_off_gpio_pins)
{
	int value = 0;
	/* Enable first to ensure that disable works correctly and the check is not against values
	 * from initialisation or from previous tests*/
	(void)stepper_enable(fixture->dev, true);
	(void)stepper_enable(fixture->dev, false);
	/* As sleep and enable pins are optional, check if they exist*/
	if (en_pin.port != NULL) {
		value = gpio_pin_get_dt(&en_pin);
		zassert_equal(value, 0, "Enable pin should not be set");
	}
	if (slp_pin.port != NULL) {
		value = gpio_pin_get_dt(&slp_pin);
		zassert_equal(value, 1, "Sleep pin should be set");
	}
}

ZTEST_F(stepper, test_is_not_moving_when_disabled)
{
	int32_t steps = 100;
	bool moving = true;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_move(fixture->dev, steps);
	(void)stepper_enable(fixture->dev, false);
	(void)stepper_is_moving(fixture->dev, &moving);
	zassert_false(moving, "Driver should not be in state is_moving after being disabled");
}

ZTEST_F(stepper, test_position_not_updating_when_disabled)
{
	int32_t steps = 1000;
	int32_t position_1 = 0;
	int32_t position_2 = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_move(fixture->dev, steps);
	(void)stepper_enable(fixture->dev, false);
	(void)stepper_get_actual_position(fixture->dev, &position_1);
	k_msleep(100);
	(void)stepper_get_actual_position(fixture->dev, &position_2);
	zassert_equal(position_2, position_1,
		      "Actual position should not have changed from %d but is %d", position_1,
		      position_2);
}

ZTEST_F(stepper, test_is_not_moving_when_reenabled_after_movement)
{
	int32_t steps = 1000;
	bool moving = true;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_move(fixture->dev, steps);
	(void)stepper_enable(fixture->dev, false);
	(void)k_msleep(100);
	(void)stepper_enable(fixture->dev, true);
	(void)k_msleep(100);
	(void)stepper_is_moving(fixture->dev, &moving);
	zassert_false(moving, "Driver should not be in state is_moving after being reenabled");
}
ZTEST_F(stepper, test_position_not_updating_when_reenabled_after_movement)
{
	int32_t steps = 1000;
	int32_t position_1 = 0;
	int32_t position_2 = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_move(fixture->dev, steps);
	(void)stepper_enable(fixture->dev, false);
	(void)stepper_get_actual_position(fixture->dev, &position_1);
	(void)k_msleep(100);
	(void)stepper_enable(fixture->dev, true);
	(void)k_msleep(100);
	(void)stepper_get_actual_position(fixture->dev, &position_2);
	zassert_equal(position_2, position_1,
		      "Actual position should not have changed from %d but is %d", position_1,
		      position_2);
}

ZTEST_F(stepper, test_set_target_position_positive_direction_movement)
{
	int32_t pos = 100;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_target_position(fixture->dev, pos);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 100u, "Target position should be %d but is %d", 100u, pos);
}

ZTEST_F(stepper, test_set_target_position_negative_direction_movement)
{
	int32_t pos = -100;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_target_position(fixture->dev, pos);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, -100, "Target position should be %d but is %d", -100, pos);
}

ZTEST_F(stepper, test_set_target_position_identical_current_and_target_position)
{
	int32_t pos = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_target_position(fixture->dev, pos);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 0, "Target position should not have changed from %d but is %d", 0, pos);
}

ZTEST_F(stepper, test_set_target_position_zero_velocity)
{
	int32_t pos = 100;
	int32_t ret = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 0u);
	ret = stepper_set_target_position(fixture->dev, pos);

	zassert_not_equal(ret, 0, "Command should fail with an error code, but returned 0");
	k_msleep(100);
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 0, "Target position should not have changed from %d but is %d", 0, pos);
}

ZTEST_F(stepper, test_set_target_position_is_moving_true_while_moving)
{
	int32_t pos = 100;
	bool moving = false;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_target_position(fixture->dev, pos);
	(void)stepper_is_moving(fixture->dev, &moving);
	zassert_true(moving, "Driver should be in state is_moving while moving");
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
}

ZTEST_F(stepper, test_set_target_position_is_moving_false_when_completed)
{
	int32_t pos = 100;
	bool moving = false;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_set_target_position(fixture->dev, pos);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
	(void)stepper_is_moving(fixture->dev, &moving);
	zassert_false(moving, "Driver should not be in state is_moving after finishing");
}

ZTEST_F(stepper, test_set_target_position_no_movement_when_disabled)
{
	int32_t pos = 100;
	int32_t curr_pos = 100;
	int32_t ret = 0;

	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_enable(fixture->dev, false);

	ret = stepper_set_target_position(fixture->dev, pos);
	zassert_not_equal(
		ret, 0,
		"Movement command should not run when not enabled and should return error code");
	(void)stepper_get_actual_position(fixture->dev, &curr_pos);
	zassert_equal(curr_pos, 0, "Current position should not have changed from %d but is %d", 0,
		      curr_pos);
}

ZTEST_F(stepper, test_move_positive_step_count)
{
	int32_t steps = 100;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_move(fixture->dev, steps);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
	(void)stepper_get_actual_position(fixture->dev, &steps);
	zassert_equal(steps, 100u, "Target position should be %d but is %d", 100u, steps);
}

ZTEST_F(stepper, test_move_negative_step_count)
{
	int32_t steps = -100;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_move(fixture->dev, steps);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
	(void)stepper_get_actual_position(fixture->dev, &steps);
	zassert_equal(steps, -100, "Target position should be %d but is %d", -100, steps);
}

ZTEST_F(stepper, test_move_zero_steps_no_movement)
{
	int32_t steps = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_move(fixture->dev, steps);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
	(void)stepper_get_actual_position(fixture->dev, &steps);
	zassert_equal(steps, 0, "Target position should be %d but is %d", 0, steps);
}

ZTEST_F(stepper, test_move_zero_velocity)
{
	int32_t steps = 100;
	int32_t ret = 0;
	int32_t pos = 100;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 0u);
	ret = stepper_move(fixture->dev, steps);

	zassert_not_equal(ret, 0, "Command should fail with an error code, but returned 0");
	k_msleep(100);
	(void)stepper_get_actual_position(fixture->dev, &pos);
	zassert_equal(pos, 0, "Target position should not have changed from %d but is %d", 0, pos);
}

ZTEST_F(stepper, test_move_is_moving_true_while_moving)
{
	int32_t steps = 100;
	bool moving = false;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_move(fixture->dev, steps);
	(void)stepper_is_moving(fixture->dev, &moving);
	zassert_true(moving, "Driver should be in state is_moving");
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
}

ZTEST_F(stepper, test_move_is_moving_false_when_completed)
{
	int32_t steps = 100;
	bool moving = true;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_set_callback(fixture->dev, fixture->callback, NULL);
	(void)stepper_move(fixture->dev, steps);
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Signal not set");
	zassert_equal(result, STEPPER_EVENT_STEPS_COMPLETED, "Signal not set");
	(void)stepper_is_moving(fixture->dev, &moving);
	zassert_false(moving, "Driver should not be in state is_moving after completion");
}

ZTEST_F(stepper, test_move_no_movement_when_disabled)
{
	int32_t steps = 100;
	int32_t curr_pos = 100;
	int32_t ret = 0;

	(void)stepper_set_max_velocity(fixture->dev, 100u);
	(void)stepper_enable(fixture->dev, false);

	ret = stepper_move(fixture->dev, steps);
	zassert_not_equal(ret, 0, "Movement command should not run when not enabled");
	(void)stepper_get_actual_position(fixture->dev, &curr_pos);
	zassert_equal(curr_pos, 0, "Current position should not have changed from %d but is %d", 0,
		      curr_pos);
}

ZTEST_F(stepper, test_constant_velocity_mode_positive_direction_correct_position)
{
	int32_t velocity = 100;
	int32_t steps = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE,
						    velocity);
	k_msleep(100);

	(void)stepper_get_actual_position(fixture->dev, &steps);
	zassert_true(steps > 0, "Current position should be positive but is %d", steps);
}

ZTEST_F(stepper, test_constant_velocity_mode_negative_direction_correct_position)
{
	int32_t velocity = 100;
	int32_t steps = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_NEGATIVE,
						    velocity);
	k_msleep(100);

	(void)stepper_get_actual_position(fixture->dev, &steps);
	zassert_true(steps < 0, "Current position should be negative but is %d", steps);
}

ZTEST_F(stepper, test_constant_velocity_mode_zero_velocity_correct_position)
{
	int32_t velocity = 0;
	int32_t steps = 0;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE,
						    velocity);
	k_msleep(100);

	zassert_equal(steps, 0, "Current position should not have changed from %d but is %d", 0,
		      steps);
}

ZTEST_F(stepper, test_constant_velocity_mode_is_moving_true_when_velocity_greater_zero)
{
	int32_t velocity = 100;
	bool moving = false;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE,
						    velocity);
	(void)stepper_is_moving(fixture->dev, &moving);
	zassert_true(moving, "Driver should be in state is_moving");
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE, 0);
}

ZTEST_F(stepper, test_constant_velocity_mode_is_moving_false_when_velocity_zero)
{
	int32_t velocity = 100;
	bool moving = true;

	(void)stepper_enable(fixture->dev, true);
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE,
						    velocity);
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE, 0);
	(void)stepper_is_moving(fixture->dev, &moving);
	zassert_false(moving, "Driver should not be in state is_moving when velocity is 0");
}

ZTEST_F(stepper, test_constant_velocity_mode_no_movement_when_disabled)
{
	int32_t velocity = 100;
	int32_t steps = 100;
	int32_t ret = 0;

	(void)stepper_enable(fixture->dev, false);

	ret = stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE,
						    velocity);
	zassert_not_equal(ret, 0, "Movement command should not run when not enabled");
	(void)stepper_get_actual_position(fixture->dev, &steps);
	zassert_equal(steps, 0, "Current position should not have changed from %d but is %d", 0,
		      steps);
}
