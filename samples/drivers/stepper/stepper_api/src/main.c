/*
 * Copyright (c) 2024 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include "zephyr/sys/printk.h"
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper.h>

/* Time to run the stepper driver in constant velocity mode, 2000 msec = 2 sec */
#define SLEEP_TIME_MS 2000

/* The devicetree node identifier for the "stepper0" alias. */
#define STEPPER_NODE DT_ALIAS(stepper0)

/* Devicepointer for the stepper driver*/
static const struct device *stepper = DEVICE_DT_GET(STEPPER_NODE);

int MICROSTEPS[] = {STEPPER_MICRO_STEP_1,  STEPPER_MICRO_STEP_2,   STEPPER_MICRO_STEP_4,
		    STEPPER_MICRO_STEP_8,  STEPPER_MICRO_STEP_16,  STEPPER_MICRO_STEP_32,
		    STEPPER_MICRO_STEP_64, STEPPER_MICRO_STEP_128, STEPPER_MICRO_STEP_256};

struct k_poll_signal stepper_signal;
struct k_poll_event stepper_event;

static void stepper_print_event_callback(const struct device *dev, enum stepper_event event,
					 void *user_data)
{
	/* Signal the rest of the sample that movement is finished */
	k_poll_signal_raise(&stepper_signal, STEPPER_EVENT_STEPS_COMPLETED);
}

int main(void)
{
	int ret;
	/* The sample targets a speed of 200 full steps per second. This value will later be
	 * converted into the correct number of microsteps per second. */
	int speed = 200;
	int base_speed = 200;
	/* The sample targets a movement distance of 200 full steps for the movecommand or the
	 * set_target_position command if the previous 2 movement commands were not supported. This
	 * value is also converted to the correct number of microsteps later on. */
	int step_count = 200;
	int base_step_count = 200;

	/* Enable stepper driver */
	ret = stepper_enable(stepper, true);
	if (ret != 0) {
		printk("Could not enable stepper driver (Error Code: %d)\n", ret);
		return ret;
	}

	/* Initialize polling, it is later used to determine when a stepper movement is finished. */
	k_poll_signal_init(&stepper_signal);
	k_poll_event_init(&stepper_event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &stepper_signal);

	/* Set the microstep resolution to the highest possibe value supported by the driver. Error
	 * messages during this process are to be expected. */
	int resolution = 0;
	printk("Attempting to set microstep resolution to highest possible value.\nError messages "
	       "from the driver concerning unsopported microstep resolutions are to be "
	       "expected.\n");
	for (int i = 0; i < 9; i++) {
		ret = stepper_set_micro_step_res(stepper, MICROSTEPS[i]);
		if (ret == 0) {
			speed = base_speed * MICROSTEPS[i];
			step_count = base_step_count * MICROSTEPS[i];
			resolution = MICROSTEPS[i];
		}
	}
	if (resolution != 0) {
		printk("Set microstep resolution to %d\n\n", resolution);
	} else {
		printk("Could not set any valid microstep resolution\n");
		return -EINVAL;
	}

	/* Set stepper callback that is triggered whenever an event with the stepper motor occurs.
	 * In this sample the completion event is always assumed to have triggered. */
	ret = stepper_set_callback(stepper, stepper_print_event_callback, NULL);

	/* Set the maximum speed that is used by the move mode to 200 steps per second, converted to
	 * microsteps per second */
	ret = stepper_set_max_velocity(stepper, speed);
	if (ret != 0) {
		printk("Could not set max velocity (Error Code: %d)\n", ret);
	}

	/* Consecutively attempt to execute three differend commands to move the stepper motor:
	 * - enable_constant_velocity_mode, which simply runs the motor at the given speed
	 * - move, which moves the motor for x (micro) steps
	 * - set_target_position, which moves the motor to the given position in (micro) steps
	 * As stepper drivers are not required to support all three commands, the sample will skip
	 * unavailable ones.*/

	/* Move stepper in constant velocity mode with a speed of 200 fullsteps per second,
	 * converted into microsteps per second, for 2 seconds. The movement direction is positive.
	 */
	printk("Constant velocity mode positive direction for %d milliseconds\n", SLEEP_TIME_MS);
	ret = stepper_enable_constant_velocity_mode(stepper, STEPPER_DIRECTION_POSITIVE, speed);
	if (ret != 0) {
		printk("Skipped constant velocity mode as it is not supported by the driver.\n");
	} else {
		k_msleep(SLEEP_TIME_MS);
		stepper_enable_constant_velocity_mode(stepper, STEPPER_DIRECTION_POSITIVE, 0);
	}
	k_msleep(200);

	/* Move stepper for 200 full steps, converted to microsteps, at a speed of 200
	 * fullsteps per second in opposite (negative) direction to the previous command.
	 */
	printk("Move in negative direction for %d (micro) steps\n", step_count);
	ret = stepper_move(stepper, -step_count);
	if (ret != 0) {
		printk("Skipped move as it is not supported by the driver.\n");
		k_poll_signal_raise(&stepper_signal, STEPPER_EVENT_STEPS_COMPLETED);
	}
	/* Wait for movement to finish */
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	k_poll_signal_reset(&stepper_signal);

	k_msleep(200);

	/* Move to target position 0 i.e. starting position. If the the motor is allready at this
	 * position, because the two previous commands were not supported, move instead to the
	 * position 200, converted from fullsteps to microsteps. */
	int target = 0;
	int position = 0;
	stepper_get_actual_position(stepper, &position);
	if (position == 0) {
		target = step_count;
	}
	printk("Move to position (in (micro) steps) %d\n", target);
	ret = stepper_set_target_position(stepper, target);
	if (ret != 0) {
		printk("Skipped set target position as it is not supported by the driver.\n");
		k_poll_signal_raise(&stepper_signal, STEPPER_EVENT_STEPS_COMPLETED);
	}
	/* Wait for movement to finish */
	(void)k_poll(&stepper_event, 1, K_SECONDS(5));
	k_poll_signal_reset(&stepper_signal);

	k_msleep(2000);

	return 0;
}
