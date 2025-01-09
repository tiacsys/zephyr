/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT vnd_timing_source_counter

#include <zephyr/drivers/counter.h>
#include "step_dir_stepper_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(step_dir_stepper);

struct step_counter_config {
	const struct step_dir_stepper_common_config *step_dir_conf;
	const struct device *step_dir;
	const struct device *counter;
};

struct step_counter_data {
	struct step_dir_stepper_common_data *step_dir_data;
	struct counter_top_cfg counter_top_cfg;
	bool counter_running;
};

static void step_counter_top_interrupt(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	struct step_dir_stepper_common_data *data = user_data;
	// LOG_INF("Data-Dev-Name: %s", data->dev->name);
	// LOG_INF("Interrupt");
	

	stepper_handle_timing_signal(data->dev);
}

int step_counter_timing_source_update(const struct device *dev, const uint32_t velocity)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	int ret;
	// LOG_INF("Entry");
	// k_msleep(10);

	if (velocity == 0) {
		return -EINVAL;
	}

	// LOG_INF("Velocity: %u", velocity);
	// LOG_INF("Counter Stuff: %u", counter_us_to_ticks(config->counter, USEC_PER_SEC)/velocity);


	// LOG_INF("Top Value Before: %u", counter_get_top_value(config->counter));
	data->counter_top_cfg.ticks =
		DIV_ROUND_UP(counter_us_to_ticks(config->counter, USEC_PER_SEC), velocity);
	// LOG_INF("Top Value After: %u", counter_get_top_value(config->counter));

	// LOG_INF("Second");
	// k_msleep(10);

	/* Lock interrupts while modifying counter settings */
	// if (data->counter_top_cfg.callback == step_counter_top_interrupt) {
	// 	LOG_INF("Top Interrupt Remains");
	// }
	data->counter_top_cfg.callback = step_counter_top_interrupt;
	data->counter_top_cfg.user_data = data;
	data->counter_top_cfg.flags = 0;
	int key = irq_lock();

	ret = counter_set_top_value(config->counter, &data->counter_top_cfg);
	// LOG_INF("Third");
	// k_msleep(10);

	irq_unlock(key);

	// LOG_INF("Fourth");
	// k_msleep(10);

	if (ret != 0) {
		LOG_ERR("%s: Failed to set counter top value (error: %d)", dev->name, ret);
		return ret;
	}
	LOG_INF("Updated");

	return 0;
}

int step_counter_timing_source_start(const struct device *dev)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	int ret;

	ret = counter_start(config->counter);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to start counter: %d", ret);
		return ret;
	}

	data->counter_running = true;
	// LOG_INF("Started");
	// LOG_INF("Top Value: %u", counter_get_top_value(config->counter));
	// LOG_INF("Frequency: %u", counter_get_frequency(config->counter));
	

	return 0;
}

int step_counter_timing_source_stop(const struct device *dev)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	int ret;

	ret = counter_stop(config->counter);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to stop counter: %d", ret);
		return ret;
	}

	data->counter_running = false;
	// LOG_INF("Stoped");

	return 0;
}

bool step_counter_timing_source_needs_reschedule(const struct device *dev)
{
	ARG_UNUSED(dev);
	return false;
}

bool step_counter_timing_source_is_running(const struct device *dev)
{
	struct step_dir_stepper_common_data *data = dev->data;

	return data->counter_running;
}

int step_counter_timing_source_init(const struct device *dev)
{
	const struct step_counter_config *config = dev->config;
	struct step_counter_data *data = dev->data;
	// LOG_INF("Counter Timing Init");
	// LOG_INF("Device Name: %s", dev->name);
	// LOG_INF("Counter Name: %s", config->counter->name);
	// k_msleep(10);

	if (!device_is_ready(config->counter)) {
		LOG_ERR("Counter device is not ready");
		return -ENODEV;
	}

	data->step_dir_data = config->step_dir->data;

	data->counter_top_cfg.callback = step_counter_top_interrupt;
	data->counter_top_cfg.user_data = data;
	data->counter_top_cfg.flags = 0;
	data->counter_top_cfg.ticks = counter_us_to_ticks(config->counter, 1000000);
	counter_set_top_value(config->counter, &data->counter_top_cfg);

	// LOG_INF("Ticks: %u", data->counter_top_cfg.ticks);

	// LOG_INF("Counter Timing Init Finished");
	// k_msleep(10);

	return 0;
}

const struct stepper_timing_source_api step_counter_timing_source_api = {
	.init = step_counter_timing_source_init,
	.update = step_counter_timing_source_update,
	.start = step_counter_timing_source_start,
	.needs_reschedule = step_counter_timing_source_needs_reschedule,
	.stop = step_counter_timing_source_stop,
	.is_running = step_counter_timing_source_is_running,
};

#define STEP_COUNTER_DEVICE(inst)                                                                  \
                                                                                                   \
	static const struct step_counter_config step_counter_config_##inst = {                     \
		.counter = DEVICE_DT_GET(DT_INST_PHANDLE(inst, counter)),                          \
	};                                                                                         \
                                                                                                   \
	static struct step_counter_data step_counter_data_##inst;                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &step_counter_timing_source_init, NULL,                        \
			      &step_counter_data_##inst, &step_counter_config_##inst, POST_KERNEL, \
			      CONFIG_STEPPER_INIT_PRIORITY, &step_counter_timing_source_api);

DT_INST_FOREACH_STATUS_OKAY(STEP_COUNTER_DEVICE)
