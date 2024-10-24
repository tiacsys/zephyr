/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 TiaC Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ZEPHYR_STEP_DIR_STEPPER_H_
#define _ZEPHYR_STEP_DIR_STEPPER_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper.h>

/**
 * @brief Common context data for STEP/DIR interface stepper motor controllers.
 */
struct step_dir_stepper_context {
	/** Stepper controller device. */
	const struct device *dev;
	/** STEP pin. */
	const struct gpio_dt_spec *step_pin;
	/** DIR pin. */
	const struct gpio_dt_spec *dir_pin;
	/** Maximum velocity during positioning (steps/s). */
	uint32_t max_velocity;
	/** Actual current position. */
	int32_t actual_position;
	/** Target position. */;
	int32_t target_position;
	/** Whether the motor is currently moving. */
	bool is_moving;
	/** Event handler registered by user. */
	stepper_event_callback_t event_callback;
	/** User data to pass to event handler. */
	void *event_callback_user_data;
	/** Implementation specific data. */
	void *impl_data;
};

int step_dir_init(struct step_dir_stepper_context *ctx, const struct device *dev,
		  const struct gpio_dt_spec *step_pin, const struct gpio_dt_spec *dir_pin);

int step_dir_stepper_set_dir(struct step_dir_stepper_context *ctx,
			     enum stepper_direction direction);

int step_dir_set_max_velocity(struct step_dir_stepper_context *ctx, uint32_t max_velocity);

int step_dir_set_actual_position(struct step_dir_stepper_context *ctx, int32_t position);

int step_dir_get_actual_position(struct step_dir_stepper_context *ctx, int32_t *position);

int step_dir_is_moving(struct step_dir_stepper_context *ctx, bool *is_moving);

int step_dir_set_event_callback(struct step_dir_stepper_context *ctx,
				stepper_event_callback_t callback, void *user_data);

int step_dir_schedule_user_callback(struct step_dir_stepper_context *ctx, enum stepper_event event);

#endif /* _ZEPHYR_STEP_DIR_STEPPER_H_ */
