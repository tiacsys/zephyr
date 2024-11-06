/*
 * Copyright (c) 2023 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <hardware/timer.h>
#include <hardware/pwm.h>
#include <hardware/structs/pwm.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/slist.h>
#include "counter_rpi_pico_pit_controller.h"
#include <cmsis_core.h>

#define LOG_LEVEL CONFIG_COUNTER_LOG_LEVEL
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(counter_rpi_pico_pit_channel, LOG_LEVEL);

#define DT_DRV_COMPAT raspberrypi_pico_pit_controller

struct counter_rpi_pico_pit_controller_data {
	sys_slist_t cb;
	const struct device *dev;
};

struct counter_rpi_pico_pit_controller_config {
	struct counter_config_info info;
	int32_t slice;
	void (*irq_config_func)(const struct device *dev);
};

static void counter_rpi_pico_pit_controller_isr(const struct device *dev)
{
	const struct counter_rpi_pico_pit_controller_config *config = dev->config;
	struct counter_rpi_pico_pit_controller_data *data = dev->data;
	struct rpi_pico_pit_callback *cb, *tmp;

	uint32_t status_mask = pwm_get_irq_status_mask();
	if (sys_slist_is_empty(&data->cb)) {
		LOG_INF("Callback ListEmpty");
	}

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->cb, cb, tmp, node) {
		// LOG_INF("Status Mask: %u Slice Mask: %u", status_mask, cb->slice_mask);
		if (cb->slice_mask & status_mask) {
			__ASSERT(cb->handler->callback, "No callback handler!");
			// LOG_INF("Calling callback");
			cb->handler->callback(dev, cb->handler->user_data);
			pwm_clear_irq(find_lsb_set(cb->slice_mask) - 1);
		}
	}
}

uint32_t counter_rpi_pico_pit_controller_get_pending_int(const struct device *dev, uint32_t mask)
{
	uint32_t inter_mask = pwm_get_irq_status_mask();
	if (inter_mask & mask) {
		return 1;
	} else {
		return 0;
	}
}

int counter_rpi_pico_pit_controller_manage_callback(const struct device *dev,
						    struct rpi_pico_pit_callback *callback,
						    bool set)
{
	LOG_INF("Managing Callback");
	struct counter_rpi_pico_pit_controller_data *data = dev->data;
	__ASSERT(callback, "No callback!");
	__ASSERT(callback->handler, "No callback handler!");

	if (!sys_slist_is_empty(&data->cb)) {
		if (!sys_slist_find_and_remove(&data->cb, &callback->node)) {
			if (!set) {
				return -EINVAL;
			}
		}
	} else if (!set) {
		return -EINVAL;
	}

	if (set) {
		LOG_INF("Adding Callback");
		sys_slist_prepend(&data->cb, &callback->node);
		if (sys_slist_is_empty(&data->cb)) {
			LOG_INF("Syslist Empty");
		}
	}

	return 0;
}

static int counter_rpi_pico_pit_controller_init(const struct device *dev)
{
	LOG_INF("Driver Init");
	const struct counter_rpi_pico_pit_controller_config *config = dev->config;
	struct counter_rpi_pico_pit_controller_data *data = dev->data;

	LOG_INF("Device Name: %s", data->dev->name);
	config->irq_config_func(dev);

	return 0;
}

#define COUNTER_RPI_PICO_PIT(inst)                                                                 \
	static void counter_rpi_pico_pit_##inst##_irq_config(const struct device *dev)             \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_INST_PROP(inst, pwm)),                                      \
			    DT_IRQ(DT_INST_PROP(inst, pwm), priority),                             \
			    counter_rpi_pico_pit_controller_isr, DEVICE_DT_INST_GET(inst), 0);     \
		irq_enable(DT_IRQN(DT_INST_PROP(inst, pwm)));                                      \
	}                                                                                          \
	static const struct counter_rpi_pico_pit_controller_config counter_##inst##_config = {     \
		.info =                                                                            \
			{                                                                          \
				.max_top_value = UINT16_MAX,                                       \
				.freq = 488400,                                                    \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = 0,                                                     \
			},                                                                         \
		.slice = 0,                                                                        \
		.irq_config_func = counter_rpi_pico_pit_##inst##_irq_config,                       \
	};                                                                                         \
	static struct counter_rpi_pico_pit_controller_data counter_##inst##_data = {               \
		.cb = NULL,                                                                        \
		.dev = DEVICE_DT_INST_GET(inst),                                                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, counter_rpi_pico_pit_controller_init, NULL,                    \
			      &counter_##inst##_data, &counter_##inst##_config, POST_KERNEL,       \
			      CONFIG_COUNTER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_RPI_PICO_PIT)
