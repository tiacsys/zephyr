/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT vnd_timing_source_work

#include "step_dir_stepper_timing_source.h"
#include "step_dir_stepper_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(step_dir_stepper);

struct step_work_data {
	struct k_work_delayable stepper_dwork;
	uint32_t velocity;
	const struct device *step_dir;
};

static k_timeout_t stepper_movement_delay(const struct device *dev)
{
	const struct step_work_data *data = dev->data;

	// LOG_INF("Entered Delay Calculation with velocity: %u", data->velocity);

	if (data->velocity == 0) {
		return K_FOREVER;
	}
	// LOG_INF("Calculated Delay in ms: %lld", K_USEC(USEC_PER_SEC / data->velocity).ticks/ K_MSEC(1).ticks);

	return K_USEC(USEC_PER_SEC / data->velocity);
}

static void stepper_work_step_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct step_work_data *data = CONTAINER_OF(dwork, struct step_work_data, stepper_dwork);

	stepper_handle_timing_signal(data->step_dir);
}

int step_work_timing_source_init(const struct device *dev)
{
	struct step_work_data *data = dev->data;

	data->velocity = 100;

	k_work_init_delayable(&data->stepper_dwork, stepper_work_step_handler);

	return 0;
}

int step_work_timing_source_update(const struct device *dev, const uint32_t velocity)
{
	struct step_work_data *data = dev->data;

	data->velocity = velocity;

	return 0;
}

int step_work_timing_source_start(const struct device *dev)
{
	struct step_work_data *data = dev->data;

	return k_work_reschedule(&data->stepper_dwork, stepper_movement_delay(dev));
}

int step_work_timing_source_stop(const struct device *dev)
{
	struct step_work_data *data = dev->data;

	return k_work_cancel_delayable(&data->stepper_dwork);
}

bool step_work_timing_source_needs_reschedule(const struct device *dev)
{
	ARG_UNUSED(dev);
	return true;
}

bool step_work_timing_source_is_running(const struct device *dev)
{
	struct step_work_data *data = dev->data;

	return k_work_delayable_is_pending(&data->stepper_dwork);
}

const struct stepper_timing_source_api step_work_timing_source_api = {
	.init = step_work_timing_source_init,
	.update = step_work_timing_source_update,
	.start = step_work_timing_source_start,
	.needs_reschedule = step_work_timing_source_needs_reschedule,
	.stop = step_work_timing_source_stop,
	.is_running = step_work_timing_source_is_running,
};

#define STEP_WORK_DEVICE(inst)                                                                     \
                                                                                                   \
	static struct step_work_data step_work_data_##inst = {                                     \
		.step_dir = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &step_work_timing_source_init, NULL, &step_work_data_##inst,   \
			      NULL, POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,                     \
			      &step_work_timing_source_api);

DT_INST_FOREACH_STATUS_OKAY(STEP_WORK_DEVICE)
