/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 TiaC Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/gpio.h>

#include "step_dir_stepper.h"

struct step_dir_stepper_callback_work {
	/** The work item itself. */
	struct k_work work;
	/** Type of event to handle. */
	enum stepper_event event;
	/** Stepper context. */
	struct step_dir_stepper_context *ctx;
};

int step_dir_init(struct step_dir_stepper_context *ctx, const struct device *dev,
		  const struct gpio_dt_spec *step_pin, const struct gpio_dt_spec *dir_pin)
{
	int ret = 0;

	if (step_pin == NULL || dir_pin == NULL) {
		return -EINVAL;
	}

	*ctx = (struct step_dir_stepper_context){
		.dev = dev,
		.step_pin = step_pin,
		.dir_pin = dir_pin,
		.max_velocity = 0,
		.is_moving = false,
		.actual_position = 0,
		.target_position = 0,
	};

	ret = gpio_pin_configure_dt(ctx->step_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(ctx->dir_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

int step_dir_stepper_set_dir(struct step_dir_stepper_context *ctx, enum stepper_direction direction)
{
	int dir_value = 0;
	switch (direction) {
	case STEPPER_DIRECTION_POSITIVE:
		dir_value = 1;
		break;
	case STEPPER_DIRECTION_NEGATIVE:
		dir_value = 0;
		break;
	default:
		return -EINVAL;
	}

	return gpio_pin_set_dt(ctx->dir_pin, dir_value);
}

int step_dir_set_max_velocity(struct step_dir_stepper_context *ctx, uint32_t max_velocity)
{
	ctx->max_velocity = max_velocity;
	return 0;
}

int step_dir_set_actual_position(struct step_dir_stepper_context *ctx, int32_t position)
{
	ctx->actual_position = position;
	return 0;
}

int step_dir_get_actual_position(struct step_dir_stepper_context *ctx, int32_t *position)
{
	*position = ctx->actual_position;
	return 0;
}

int step_dir_is_moving(struct step_dir_stepper_context *ctx, bool *is_moving)
{
	*is_moving = ctx->is_moving;
	return 0;
}

int step_dir_set_event_callback(struct step_dir_stepper_context *ctx,
				stepper_event_callback_t callback, void *user_data)
{
	ctx->event_callback = callback;
	ctx->event_callback_user_data = user_data;
	return 0;
}

static void step_dir_user_callback_work_fn(struct k_work *work)
{
	/* Get required data from outer struct */
	struct step_dir_stepper_callback_work *callback_work =
		CONTAINER_OF(work, struct step_dir_stepper_callback_work, work);
	struct step_dir_stepper_context *ctx = callback_work->ctx;
	enum stepper_event event = callback_work->event;

	/* Run the callback */
	if (ctx->event_callback != NULL) {
		ctx->event_callback(ctx->dev, event, ctx->event_callback_user_data);
	}

	/* Free the work item */
	k_free(callback_work);
}

int step_dir_schedule_user_callback(struct step_dir_stepper_context *ctx, enum stepper_event event)
{
	/* Create and schedule work item to run user callback */
	struct step_dir_stepper_callback_work *callback_work =
		k_malloc(sizeof(struct step_dir_stepper_callback_work));
	if (callback_work == NULL) {
		return -ENOMEM;
	}

	/* Fill out data for work item */
	*callback_work = (struct step_dir_stepper_callback_work){
		.ctx = ctx,
		.event = STEPPER_EVENT_STEPS_COMPLETED,
	};
	k_work_init(&callback_work->work, step_dir_user_callback_work_fn);

	/* Try to submit the item */
	int ret = k_work_submit(&callback_work->work);
	/* ret == 0 shouldn't happen (it would mean we've submitted this same item before), but
	 * there's no reason to force a segfault in case it somehow does happen. */
	if (ret != 1 && ret != 0) {
		/* We failed to submit the item, so we need to free it here */
		k_free(callback_work);
		return -ENODEV; // FIXME: What's the correct error code here?
	}

	return 0;
}
