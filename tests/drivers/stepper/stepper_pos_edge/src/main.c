/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Jilay Sandeep Pandya
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/stepper.h>

struct stepper_fixture {
	const struct device *dev;
	stepper_event_callback_t callback;
};

struct k_poll_signal stepper_signal;
struct k_poll_event stepper_event;
void *user_data_received;

static void stepper_print_event_callback(const struct device *dev, enum stepper_event event,
					 void *user_data)
{
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
		.dev = DEVICE_DT_GET(DT_NODELABEL(motor_2)),
		.callback = stepper_print_event_callback,
	};

	k_poll_signal_init(&stepper_signal);
	k_poll_event_init(&stepper_event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &stepper_signal);
	user_data_received = NULL;
	zassert_not_null(fixture.dev);
	return &fixture;
}

static void stepper_before(void *f)
{
	struct stepper_fixture *fixture = f;
	(void)stepper_set_actual_position(fixture->dev, 0);
	k_poll_signal_reset(&stepper_signal);
	(void)stepper_enable(fixture->dev, true);
}

static void stepper_teardown(void *f){
	struct stepper_fixture *fixture = f;

	(void)stepper_enable(fixture->dev, false);

}

ZTEST_SUITE(stepper, NULL, stepper_setup, stepper_before, NULL, stepper_teardown);

/* ZTEST_F(stepper, test_micro_step_res) */
/* { */
/* 	(void)stepper_set_micro_step_res(fixture->dev, 2); */
/* 	enum stepper_micro_step_resolution res; */
/* 	(void)stepper_get_micro_step_res(fixture->dev, &res); */
/* 	zassert_equal(res, 2, "Micro step resolution not set correctly"); */
/* } */

/* ZTEST_F(stepper, test_actual_position) */
/* { */
/* 	int32_t pos = 100u; */
/* 	(void)stepper_set_actual_position(fixture->dev, pos); */
/* 	(void)stepper_get_actual_position(fixture->dev, &pos); */
/* 	zassert_equal(pos, 100u, "Actual position not set correctly"); */
/* } */

ZTEST_F(stepper, test_max_position_event)
{
	int32_t pos = INT32_MAX - 10;

	/* Pass the function name as user data */
	(void)stepper_set_callback(fixture->dev, fixture->callback, &fixture);

	(void)stepper_set_actual_position(fixture->dev, pos);
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_POSITIVE, 100u);

	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;
	int32_t value = 0;
	(void)stepper_get_actual_position(fixture->dev, &value);

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	printk("Current Position: %d\n", value);
	zassert_equal(signaled, 1, "Maximum Position event not detected");
	zassert_equal(result, STEPPER_EVENT_RIGHT_END_STOP_DETECTED, "Maximum Position event not detected");
}

ZTEST_F(stepper, test_min_position_event)
{
	int32_t pos = INT32_MIN + 10;

	/* Pass the function name as user data */
	(void)stepper_set_callback(fixture->dev, fixture->callback, &fixture);

	(void)stepper_set_actual_position(fixture->dev, pos);
	(void)stepper_enable_constant_velocity_mode(fixture->dev, STEPPER_DIRECTION_NEGATIVE, 100u);

	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	unsigned int signaled;
	int result;

	k_poll_signal_check(&stepper_signal, &signaled, &result);
	zassert_equal(signaled, 1, "Minimum Position event not detected");
	zassert_equal(result, STEPPER_EVENT_LEFT_END_STOP_DETECTED, "Minimum Position event not detected");
}
