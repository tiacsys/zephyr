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
	const struct device *step_dir;
	const struct device *counter;
};

struct step_counter_data {
	struct counter_top_cfg counter_top_cfg;
	bool counter_running;
};

static void step_counter_top_interrupt(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	const struct step_counter_config *config = user_data;

	stepper_handle_timing_signal(config->step_dir);
}

int step_counter_timing_source_update(const struct device *dev, const uint32_t velocity)
{
	const struct step_counter_config *config = dev->config;
	struct step_counter_data *data = dev->data;
	int ret;

	if (velocity == 0) {
		return -EINVAL;
	}

	data->counter_top_cfg.ticks =
		DIV_ROUND_UP(counter_us_to_ticks(config->counter, USEC_PER_SEC), velocity);

	int key = irq_lock();

	ret = counter_set_top_value(config->counter, &data->counter_top_cfg);

	irq_unlock(key);

	if (ret != 0) {
		LOG_ERR("%s: Failed to set counter top value (error: %d)", dev->name, ret);
		return ret;
	}

	return 0;
}

int step_counter_timing_source_start(const struct device *dev)
{
	const struct step_counter_config *config = dev->config;
	struct step_counter_data *data = dev->data;
	int ret;

	ret = counter_start(config->counter);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to start counter: %d", ret);
		return ret;
	}

	data->counter_running = true;

	return 0;
}

int step_counter_timing_source_stop(const struct device *dev)
{
	const struct step_counter_config *config = dev->config;
	struct step_counter_data *data = dev->data;
	int ret;

	ret = counter_stop(config->counter);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to stop counter: %d", ret);
		return ret;
	}

	data->counter_running = false;

	return 0;
}

bool step_counter_timing_source_needs_reschedule(const struct device *dev)
{
	ARG_UNUSED(dev);
	return false;
}

bool step_counter_timing_source_is_running(const struct device *dev)
{
	struct step_counter_data *data = dev->data;

	return data->counter_running;
}

int step_counter_timing_source_init(const struct device *dev)
{
	const struct step_counter_config *config = dev->config;
	struct step_counter_data *data = dev->data;

	if (!device_is_ready(config->counter)) {
		LOG_ERR("Counter device is not ready");
		return -ENODEV;
	}

	data->counter_top_cfg.callback = step_counter_top_interrupt;
	data->counter_top_cfg.user_data = (struct step_counter_config *)config;
	data->counter_top_cfg.flags = 0;
	data->counter_top_cfg.ticks = counter_us_to_ticks(config->counter, 1000000);
	counter_set_top_value(config->counter, &data->counter_top_cfg);

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
		.step_dir = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                   \
	};                                                                                         \
                                                                                                   \
	static struct step_counter_data step_counter_data_##inst;                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &step_counter_timing_source_init, NULL,                        \
			      &step_counter_data_##inst, &step_counter_config_##inst, POST_KERNEL, \
			      CONFIG_STEPPER_INIT_PRIORITY, &step_counter_timing_source_api);

DT_INST_FOREACH_STATUS_OKAY(STEP_COUNTER_DEVICE)
