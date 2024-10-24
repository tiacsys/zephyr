/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 TiaC Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "step_dir_stepper.h"
#include <zephyr/irq.h>
#include "zephyr/sys_clock.h"
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/counter.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(step_dir_counter, CONFIG_STEPPER_LOG_LEVEL);

struct step_dir_counter_data {
	const struct device *counter;
	bool constant_velocity;
	enum stepper_direction constant_velocity_direction;
	bool step_signal_high;
	struct counter_top_cfg counter_top_cfg;
};

static int step_dir_counter_set_counter_frequency(struct step_dir_stepper_context *ctx,
						  uint32_t freq_hz)
{
	struct step_dir_counter_data *data = (struct step_dir_counter_data *)ctx->impl_data;
	int ret = 0;

	if (freq_hz == 0) {
		return -EINVAL;
	}

	data->counter_top_cfg.ticks = counter_us_to_ticks(data->counter, USEC_PER_SEC / freq_hz);

	/* Lock interrupts while modifying counter settings */
	int irq_key = irq_lock();

	ret = counter_set_top_value(data->counter, &data->counter_top_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to set counter top value (error: %d)", ret);
		goto end;
	}

end:
	irq_unlock(irq_key);
	return ret;
}

static void step_dir_counter_top_cb(const struct device *dev, void *data)
{
	struct step_dir_stepper_context *ctx = (struct step_dir_stepper_context *)data;
	struct step_dir_counter_data *counter_data = (struct step_dir_counter_data *)ctx->impl_data;

	if (!counter_data->constant_velocity) {
		/* Check if target position is reached */
		if (ctx->actual_position == ctx->target_position) {
			counter_stop(counter_data->counter);
			ctx->is_moving = false;
			if (ctx->event_callback != NULL) {
				step_dir_schedule_user_callback(ctx, STEPPER_EVENT_STEPS_COMPLETED);
			}

			return;
		}
	}

	/* Determine direction we're going in for counting purposes */
	enum stepper_direction direction = 0;
	if (counter_data->constant_velocity) {
		direction = counter_data->constant_velocity_direction;
	} else {
		direction = (ctx->target_position >= ctx->actual_position)
				    ? STEPPER_DIRECTION_POSITIVE
				    : STEPPER_DIRECTION_NEGATIVE;
	}

	/* Switch step pin on or off depending position in step period */
	if (counter_data->step_signal_high) {
		/* Generate a falling edge and count a completed step */
		gpio_pin_set_dt(ctx->step_pin, 0);
		if (direction == STEPPER_DIRECTION_POSITIVE) {
			ctx->actual_position++;
		} else {
			ctx->actual_position--;
		}
	} else {
		/* Generate a rising edge */
		gpio_pin_set_dt(ctx->step_pin, 1);
	}

	counter_data->step_signal_high = !counter_data->step_signal_high;
}

static int step_dir_counter_begin_positioning(struct step_dir_stepper_context *ctx)
{
	struct step_dir_counter_data *data = (struct step_dir_counter_data *)ctx->impl_data;
	int ret = 0;

	/* Lock interrupts while modifying counter settings */
	int irq_key = irq_lock();

	/* Set direction pin */
	enum stepper_direction direction = (ctx->target_position >= ctx->actual_position)
						   ? STEPPER_DIRECTION_POSITIVE
						   : STEPPER_DIRECTION_NEGATIVE;
	ret = step_dir_stepper_set_dir(ctx, direction);
	if (ret != 0) {
		return ret;
	}

	/* Disable constant velocity mode if active */
	data->constant_velocity = false;

	/* Set counter to correct frequency */
	ret = step_dir_counter_set_counter_frequency(ctx, ctx->max_velocity);
	if (ret != 0) {
		goto end;
	}

	/* Start counter */
	ret = counter_start(data->counter);
	if (ret != 0) {
		goto end;
	}
	ctx->is_moving = true;

end:
	irq_unlock(irq_key);
	return ret;
}

int step_dir_counter_init(struct step_dir_stepper_context *ctx, struct step_dir_counter_data *data,
			  const struct device *dev, const struct gpio_dt_spec *step_pin,
			  const struct gpio_dt_spec *dir_pin, const struct device *counter)
{
	int ret = 0;

	ret = step_dir_init(ctx, dev, step_pin, dir_pin);
	if (ret != 0) {
		return ret;
	}

	*data = (struct step_dir_counter_data){
		.counter = counter,
		.step_signal_high = false,
		.counter_top_cfg =
			{
				.callback = step_dir_counter_top_cb,
				.flags = 0,
				.user_data = ctx,
				.ticks = counter_us_to_ticks(counter, USEC_PER_SEC),
			},
	};

	ctx->impl_data = data;

	return 0;
}

int step_dir_counter_set_target_position(struct step_dir_stepper_context *ctx,
					 int32_t target_position)
{
	ctx->target_position = target_position;
	return step_dir_counter_begin_positioning(ctx);
}

int step_dir_counter_move(struct step_dir_stepper_context *ctx, int32_t microsteps)
{
	int32_t target_position = ctx->actual_position + microsteps;
	return step_dir_counter_set_target_position(ctx, target_position);
}

int step_dir_counter_enable_constant_velocity_mode(struct step_dir_stepper_context *ctx,
						   enum stepper_direction direction,
						   uint32_t velocity)
{
	struct step_dir_counter_data *data = (struct step_dir_counter_data *)ctx->impl_data;
	int ret = 0;

	if (velocity == 0) {
		counter_stop(data->counter);
		ctx->is_moving = false;
		return 0;
	}

	/* Set direction pin */
	ret = step_dir_stepper_set_dir(ctx, direction);
	if (ret != 0) {
		return ret;
	}

	/* Lock interrupts while modifying counter settings */
	int irq_key = irq_lock();

	/* Set flags for constant velocity mode */
	data->constant_velocity = true;
	data->constant_velocity_direction = direction;

	/* Set counter frequency */
	ret = step_dir_counter_set_counter_frequency(ctx, velocity);
	if (ret != 0) {
		goto end;
	}

	/* Start counter */
	ret = counter_start(data->counter);
	if (ret != 0) {
		goto end;
	}
	ctx->is_moving = true;

end:
	irq_unlock(irq_key);
	return ret;
}
