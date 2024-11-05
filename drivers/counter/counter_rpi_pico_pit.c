/*
 * Copyright (c) 2023 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
// #include <cstdint>
#include <hardware/timer.h>
#include <hardware/pwm.h>
#include <hardware/structs/pwm.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/irq.h>
#include <cmsis_core.h>

#define LOG_LEVEL CONFIG_COUNTER_LOG_LEVEL
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(counter_rpi_pico_pit, LOG_LEVEL);

#define DT_DRV_COMPAT raspberrypi_pico_pit

struct counter_rpi_pico_pit_data {
	pwm_config config_pwm;
	uint16_t top_value;
};

struct counter_rpi_pico_pit_config {
	struct counter_config_info info;
	int32_t slice;
};

static int counter_rpi_pico_pit_start(const struct device *dev)
{
	const struct counter_rpi_pico_pit_config *config = dev->config;

	pwm_set_counter(config->slice, 0);
	pwm_set_enabled(config->slice, true);

	return 0;
}

static int counter_rpi_pico_pit_stop(const struct device *dev)
{
	const struct counter_rpi_pico_pit_config *config = dev->config;

	pwm_set_enabled(config->slice, false);

	return 0;
}

static uint32_t counter_rpi_pico_pit_get_top_value(const struct device *dev)
{
	struct counter_rpi_pico_pit_data *data = dev->data;

	return data->top_value;
}

static int counter_rpi_pico_pit_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_rpi_pico_pit_config *config = dev->config;
	*ticks = pwm_get_counter(config->slice);
	return 0;
}

static int counter_rpi_pico_pit_set_alarm(const struct device *dev, uint8_t id,
					  const struct counter_alarm_cfg *alarm_cfg)
{
	return -ENOTSUP;
}

static int counter_rpi_pico_pit_cancel_alarm(const struct device *dev, uint8_t id)
{
	return -ENOTSUP;
}

static int counter_rpi_pico_pit_set_top_value(const struct device *dev,
					      const struct counter_top_cfg *cfg)
{
	const struct counter_rpi_pico_pit_config *config = dev->config;
	struct counter_rpi_pico_pit_data *data = dev->data;

	if (cfg->ticks == 0 || cfg->ticks > UINT16_MAX) {
		return -EINVAL;
	}
	if (cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		return -ENOTSUP;
	}

	pwm_set_enabled(config->slice, false);
	pwm_set_chan_level(config->slice, 1, 0);
	pwm_set_chan_level(config->slice, 0, 0);
	pwm_set_counter(config->slice, 0);

	// uint16_t cycles = (config->info.freq / cfg->ticks);

	data->config_pwm = pwm_get_default_config();
	pwm_config_set_wrap(&data->config_pwm, cfg->ticks);
	pwm_config_set_clkdiv_int_frac(&data->config_pwm, 255, 15);
	data->top_value = cfg->ticks;
	pwm_init(config->slice, &data->config_pwm, true);

	// TODO: Interrupt handling
	// Temp

	return 0;
}

static uint32_t counter_rpi_pico_pit_get_pending_int(const struct device *dev)
{
	uint32_t inter_mask = pwm_get_irq_status_mask();
	if (inter_mask == 0) {
		return 0;
	} else {
		return 1;
	}
}

static uint32_t counter_rpi_pico_pit_get_guard_period(const struct device *dev, uint32_t flags)
{
	return -ENOTSUP;
}

static int counter_rpi_pico_pit_set_guard_period(const struct device *dev, uint32_t guard,
						 uint32_t flags)
{
	return -ENOTSUP;
}

static int counter_rpi_pico_pit_init(const struct device *dev)
{
	const struct counter_rpi_pico_pit_config *config = dev->config;
	int ret;

	// config->irq_config();

	return 0;
}

static const struct counter_driver_api counter_rpi_pico_driver_api = {
	.start = counter_rpi_pico_pit_start,
	.stop = counter_rpi_pico_pit_stop,
	.get_value = counter_rpi_pico_pit_get_value,
	.set_alarm = counter_rpi_pico_pit_set_alarm,
	.cancel_alarm = counter_rpi_pico_pit_cancel_alarm,
	.set_top_value = counter_rpi_pico_pit_set_top_value,
	.get_pending_int = counter_rpi_pico_pit_get_pending_int,
	.get_top_value = counter_rpi_pico_pit_get_top_value,
	.get_guard_period = counter_rpi_pico_pit_get_guard_period,
	.set_guard_period = counter_rpi_pico_pit_set_guard_period,
};

#define COUNTER_RPI_PICO_PIT(inst)                                                                 \
	static const struct counter_rpi_pico_pit_config counter_##inst##_config = {                \
		.info =                                                                            \
			{                                                                          \
				.max_top_value = UINT16_MAX,                                       \
				.freq = 488400,                                                    \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = 0,                                                     \
			},                                                                         \
		.slice = DT_INST_PROP(inst, pwm_slice),                                            \
	};                                                                                         \
	static struct counter_rpi_pico_pit_data counter_##inst##_data = {                          \
		.config_pwm = NULL,                                                                \
		.top_value = UINT16_MAX,                                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, counter_rpi_pico_pit_init, NULL, &counter_##inst##_data,       \
			      &counter_##inst##_config, POST_KERNEL, CONFIG_COUNTER_INIT_PRIORITY, \
			      &counter_rpi_pico_driver_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_RPI_PICO_PIT)
