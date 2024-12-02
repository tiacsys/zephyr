/*
 * Copyright (c) 2021 Kent Hall.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include <stdbool.h>
#include <stdint.h>
#define DT_DRV_COMPAT zephyr_counter_emul

#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_emul, CONFIG_COUNTER_LOG_LEVEL);

struct counter_emul_callback_data {
	void *callback_user_data;
	const struct device *counter;
};

struct counter_emul_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t guard_period;
	uint32_t freq;
	struct k_timer base_timer;
	struct k_timer alarm1;
	struct k_timer alarm2;
	struct k_timer alarm3;
	struct k_timer alarm4;
	uint32_t top_value;
	uint32_t value_backup;
	struct counter_emul_callback_data user_data;
};

struct counter_emul_config {
	struct counter_config_info info;
	bool counting_down;
};

static int counter_emul_start(const struct device *dev)
{
	struct counter_emul_data *data = dev->data;
	k_timer_start(&data->base_timer, K_TICKS(data->top_value - data->value_backup),
		      K_TICKS(data->top_value));
	return 0;
}

static int counter_emul_stop(const struct device *dev)
{
	struct counter_emul_data *data = dev->data;
	k_timer_stop(&data->base_timer);
	counter_get_value(dev, &data->value_backup);
	return 0;
}

static uint32_t counter_emul_get_top_value(const struct device *dev)
{
	struct counter_emul_data *data = dev->data;
	return data->top_value;
}

static int counter_stm32_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_emul_config *config = dev->config;
	struct counter_emul_data *data = dev->data;

	if (config->counting_down) {
		*ticks = k_timer_remaining_ticks(&data->base_timer);
	} else {
		*ticks = data->top_value - (uint32_t)k_timer_remaining_ticks(&data->base_timer);
	}
	return 0;
}

static int counter_emul_set_alarm(const struct device *dev, uint8_t chan,
				  const struct counter_alarm_cfg *alarm_cfg)
{
	return -ENOTSUP;
}

static int counter_emul_cancel_alarm(const struct device *dev, uint8_t chan)
{
	return -ENOTSUP;
}

static int counter_emul_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	struct counter_emul_data *data = dev->data;
	if (cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		return -ENOTSUP;
	}
	counter_stop(dev);
	data->top_value = cfg->ticks;
	data->top_cb = cfg->callback;
	data->user_data.callback_user_data = cfg->user_data;
	k_timer_user_data_set(&data->base_timer, &data->user_data);
	counter_emul_start(dev);
	return 0;
}

static uint32_t counter_emul_get_pending_int(const struct device *dev)
{
	return -ENOTSUP;
}

static uint32_t counter_emul_get_guard_period(const struct device *dev, uint32_t flags)
{
	return -ENOTSUP;
}

static int counter_emul_set_guard_period(const struct device *dev, uint32_t guard, uint32_t flags)
{
	return -ENOTSUP;
}

static uint32_t counter_emul_get_freq(const struct device *dev)
{
	return (uint32_t)K_SECONDS(1).ticks;
}

static void counter_emul_top_irq_handle(struct k_timer *timer)
{
	struct counter_emul_callback_data *cb_data = k_timer_user_data_get(timer);
	struct counter_emul_data *data = cb_data->counter->data;
	if (data->top_cb != NULL) {
		data->top_cb(cb_data->counter, cb_data->callback_user_data);
	}
}

static void counter_stm32_alarm_irq_handle(const struct device *dev, uint32_t id)
{
}

static int counter_emul_init(const struct device *dev)
{
	struct counter_emul_data *data = dev->data;
	k_timer_init(&data->base_timer, counter_emul_top_irq_handle, NULL);
	k_timer_start(&data->base_timer, K_TICKS(UINT32_MAX), K_TICKS(UINT32_MAX));
	data->value_backup = 0;
	data->top_value = UINT32_MAX;
	data->user_data.counter = dev;
	data->user_data.callback_user_data = NULL;
	k_timer_user_data_set(&data->base_timer, &data->user_data);

	return 0;
}

static const struct counter_driver_api counter_emul_driver_api = {
	.start = counter_emul_start,
	.stop = counter_emul_stop,
	.get_value = counter_stm32_get_value,
	.set_alarm = counter_emul_set_alarm,
	.cancel_alarm = counter_emul_cancel_alarm,
	.set_top_value = counter_emul_set_top_value,
	.get_pending_int = counter_emul_get_pending_int,
	.get_top_value = counter_emul_get_top_value,
	.get_guard_period = counter_emul_get_guard_period,
	.set_guard_period = counter_emul_set_guard_period,
	.get_freq = counter_emul_get_freq,
};

#define COUNTER_DEVICE_INIT(idx)                                                                   \
	static struct counter_emul_data counter##idx##_data;                                       \
                                                                                                   \
	static const struct counter_emul_config counter##idx##_config = {                          \
		.info =                                                                            \
			{                                                                          \
				.max_top_value = UINT32_MAX,                                       \
				.flags = DT_INST_PROP(idx, counting_down)                          \
						 ? 0                                               \
						 : COUNTER_CONFIG_INFO_COUNT_UP,                   \
				.channels = 4,                                                     \
			},                                                                         \
		.counting_down = DT_INST_PROP(idx, counting_down),                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, counter_emul_init, NULL, &counter##idx##_data,                  \
			      &counter##idx##_config, PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,  \
			      &counter_emul_driver_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_DEVICE_INIT)
