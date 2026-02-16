/*
 * Copyright 2024 Fabian Blatz <fabianblatz@gmail.com>
 * Copyright 2026 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "include/stepper_controller_event_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stepper_controller_event_common, CONFIG_STEPPER_LOG_LEVEL);

void stepper_controller_event_common_trigger_callback(const struct device *dev, enum stepper_event event)
{
	struct stepper_controller_event_common_data *data = dev->data;

	if (!data->callback) {
		LOG_WRN_ONCE("No callback set");
		return;
	}

	if (!k_is_in_isr()) {
		data->callback(dev, event, data->event_cb_user_data);
		return;
	}

#ifdef CONFIG_STEPPER_CONTROLLER_ISR_SAFE_EVENTS
	/* Dispatch to msgq instead of raising directly */
	int ret = k_msgq_put(&data->event_msgq, &event, K_NO_WAIT);

	if (ret != 0) {
		LOG_WRN("Failed to put event in msgq: %d", ret);
	}

	ret = k_work_submit(&data->event_callback_work);
	if (ret < 0) {
		LOG_ERR("Failed to submit work item: %d", ret);
	}
#else
	data->callback(dev, event, data->event_cb_user_data);
#endif /* CONFIG_STEPPER_CONTROLLER_ISR_SAFE_EVENTS */
}

#ifdef CONFIG_STEPPER_CONTROLLER_ISR_SAFE_EVENTS
static void stepper_controller_event_common_event_handler(struct k_work *work)
{
	struct stepper_controller_event_common_data *data =
		CONTAINER_OF(work, struct stepper_controller_event_common_data, event_callback_work);
	enum stepper_event event;
	int ret;

	ret = k_msgq_get(&data->event_msgq, &event, K_NO_WAIT);
	if (ret != 0) {
		return;
	}

	/* Run the callback */
	if (data->callback != NULL) {
		data->callback(data->dev, event, data->event_cb_user_data);
	}

	/* If there are more pending events, resubmit this work item to handle them */
	if (k_msgq_num_used_get(&data->event_msgq) > 0) {
		k_work_submit(work);
	}
}
#endif /* CONFIG_STEPPER_CONTROLLER_ISR_SAFE_EVENTS */

int stepper_controller_event_common_init(const struct device *dev)
{
#ifdef CONFIG_STEPPER_CONTROLLER_ISR_SAFE_EVENTS
	struct stepper_controller_event_common_data *data = dev->data;

	k_msgq_init(&data->event_msgq, data->event_msgq_buffer, sizeof(enum stepper_event),
		    CONFIG_STEPPER_CONTROLLER_EVENT_QUEUE_LEN);
	k_work_init(&data->event_callback_work, stepper_controller_event_common_event_handler);
#endif /* CONFIG_STEPPER_CONTROLLER_ISR_SAFE_EVENTS */
	return 0;
}
