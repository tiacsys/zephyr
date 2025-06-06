/*
 * Copyright 2025 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "step_dir_stepper.h"

int step_dir_stepper_move_by_wrapper(const struct device *dev, const int32_t micro_steps)
{
	printk("Test\n");
        return 0;
}

int step_dir_stepper_set_microstep_interval_wrapper(const struct device *dev,
						    const uint64_t microstep_interval_ns)
{
	printk("Test\n");
        return 0;
}

int step_dir_stepper_set_reference_position_wrapper(const struct device *dev, const int32_t value)
{
	printk("Test\n");
        return 0;
}

int step_dir_stepper_get_actual_position_wrapper(const struct device *dev, int32_t *value)
{
	printk("Test\n");
        return 0;
}

int step_dir_stepper_move_to_wrapper(const struct device *dev, const int32_t value)
{
	printk("Test\n");
        return 0;
}

int step_dir_stepper_is_moving_wrapper(const struct device *dev, bool *is_moving)
{
        printk("Test\n");
        return 0;
}

int step_dir_stepper_run_wrapper(const struct device *dev, const enum stepper_direction direction)
{
        printk("Test\n");
        return 0;
}

int step_dir_stepper_stop_wrapper(const struct device *dev)
{
        printk("Test\n");
        return 0;
}

int step_dir_stepper_set_event_callback_wrapper(const struct device *dev,
						stepper_event_callback_t callback, void *user_data)
{
	printk("Test\n");
        return 0;
}
