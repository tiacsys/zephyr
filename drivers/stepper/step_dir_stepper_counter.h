/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 TiaC Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ZEPHYR_STEP_DIR_STEPPER_COUNTER_H_
#define _ZEPHYR_STEP_DIR_STEPPER_COUNTER_H_

#include <zephyr/device.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/counter.h>
#include "step_dir_stepper.h"

struct step_dir_counter_data {
	const struct device *counter;
	bool constant_velocity;
	enum stepper_direction constant_velocity_direction;
	bool step_signal_high;
	struct counter_top_cfg counter_top_cfg;
};

int step_dir_counter_init(struct step_dir_stepper_context *ctx, struct step_dir_counter_data *data,
			  const struct device *dev, const struct gpio_dt_spec *step_pin,
			  const struct gpio_dt_spec *dir_pin, const struct device *counter);

int step_dir_counter_set_target_position(struct step_dir_stepper_context *ctx,
					 int32_t target_position);

int step_dir_counter_move(struct step_dir_stepper_context *ctx, int32_t microsteps);

int step_dir_counter_enable_constant_velocity_mode(struct step_dir_stepper_context *ctx,
						   enum stepper_direction direction,
						   uint32_t velocity);

#endif /* _ZEPHYR_STEP_DIR_STEPPER_COUNTER_H_ */
