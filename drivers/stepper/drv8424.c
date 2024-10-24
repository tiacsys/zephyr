/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 TiaC Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util.h"
#include "zephyr/sys_clock.h"
#include "stepper/step_dir_stepper.h"
#include "stepper/step_dir_stepper_counter.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv8424, CONFIG_STEPPER_LOG_LEVEL);

#define DT_DRV_COMPAT ti_drv8424

/**
 * @brief DRV8424 stepper driver configuration data.
 *
 * This structure contains all of the devicetree specifications for the pins
 * needed by a given DRV8424 stepper driver.
 */
struct drv8424_config {
	/** Direction pin. */
	struct gpio_dt_spec dir_pin;
	/** Step pin. */
	struct gpio_dt_spec step_pin;
	/** Sleep pin. */
	struct gpio_dt_spec sleep_pin;
	/** Enable pin. */
	struct gpio_dt_spec en_pin;
	/** Microstep configuration pin 0. */
	struct gpio_dt_spec m0_pin;
	/** Microstep configuration pin 1. */
	struct gpio_dt_spec m1_pin;
	/** Counter used as timing source. */
	const struct device *counter;
};

/* Struct for storing the states of output pins. */
struct drv8424_pin_states {
	uint8_t sleep: 1;
	uint8_t en: 1;
	uint8_t m0: 2;
	uint8_t m1: 2;
};

/**
 * @brief DRV8424 stepper driver data.
 *
 * This structure contains mutable data used by a DRV8424 stepper driver.
 */
struct drv8424_data {
	/** Back pointer to the device. */
	const struct device *dev;
	/** Whether the device is enabled */
	bool enabled;
	/** Struct containing the states of different pins. */
	struct drv8424_pin_states pin_states;
	/** Current microstep resolution. */
	enum stepper_micro_step_resolution ms_res;
	/** General Step/Dir context */
	struct step_dir_stepper_context step_dir_ctx;
	/** Counter-Step/Dir specific data */
	struct step_dir_counter_data step_dir_counter_data;
};

/*
 * If microstep setter fails, attempt to recover into previous state.
 */
static int drv8424_microstep_recovery(const struct device *dev)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	int ret = 0;

	int m0_value = data->pin_states.m0;
	int m1_value = data->pin_states.m1;

	/* Reset m0 pin as it may have been disconnected */
	ret = gpio_pin_configure_dt(&config->m0_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to restore microstep configuration (error: %d)", dev->name,
			ret);
		return ret;
	}

	/* Reset m1 pin as it may have been disconnected. */
	ret = gpio_pin_configure_dt(&config->m1_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to restore microstep configuration (error: %d)", dev->name,
			ret);
		return ret;
	}

	switch (m0_value) {
	case 0:
		ret = gpio_pin_set_dt(&config->m0_pin, 0);
		break;
	case 1:
		ret = gpio_pin_set_dt(&config->m0_pin, 1);
		break;
	case 2:
		/* Hi-Z is set by configuring pin as disconnected, not all gpio controllers support
		 * this. */
		ret = gpio_pin_configure_dt(&config->m0_pin, GPIO_DISCONNECTED);
		break;
	default:
		break;
	}

	if (ret != 0) {
		LOG_ERR("%s: Failed to restore microstep configuration (error: %d)", dev->name,
			ret);
		return ret;
	}

	switch (m1_value) {
	case 0:
		ret = gpio_pin_set_dt(&config->m1_pin, 0);
		break;
	case 1:
		ret = gpio_pin_set_dt(&config->m1_pin, 1);
		break;
	case 2:
		/* Hi-Z is set by configuring pin as disconnected, not all gpio controllers support
		 * this. */
		ret = gpio_pin_configure_dt(&config->m1_pin, GPIO_DISCONNECTED);
		break;
	default:
		break;
	}

	if (ret != 0) {
		LOG_ERR("%s: Failed to restore microstep configuration (error: %d)", dev->name,
			ret);
		return ret;
	}

	return 0;
}

static int drv8424_enable(const struct device *dev, bool enable)
{
	int ret;
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	bool has_enable_pin = config->en_pin.port != NULL;
	bool has_sleep_pin = config->sleep_pin.port != NULL;

	/* Check availability of sleep and and enable pins, as these might be hardwired. */
	if (!has_sleep_pin && !has_enable_pin) {
		LOG_ERR("%s: Failed to enable/disable device, neither sleep pin nor enable pin are "
			"available. The device is always on.",
			dev->name);
		return -ENOTSUP;
	}

	if (has_sleep_pin) {
		ret = gpio_pin_set_dt(&config->sleep_pin, !enable);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set sleep_pin (error: %d)", dev->name, ret);
			return ret;
		}
		data->pin_states.sleep = enable ? 0U : 1U;
	}

	if (has_enable_pin) {
		ret = gpio_pin_set_dt(&config->en_pin, enable);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set en_pin (error: %d)", dev->name, ret);
			return ret;
		}
		data->pin_states.en = enable ? 1U : 0U;
	}

	data->enabled = enable;
	if (!enable) {
		data->step_dir_ctx.is_moving = false; // FIXME: abstraction leaks here
	}

	return 0;
}

static int drv8424_move(const struct device *dev, int32_t micro_steps)
{
	struct drv8424_data *data = dev->data;

	if (data->step_dir_ctx.max_velocity == 0) {
		LOG_ERR("%s: Invalid max. velocity %d configured", dev->name,
			data->step_dir_ctx.max_velocity);
		return -EINVAL;
	}

	if (!data->enabled) {
		return -ENODEV;
	}

	return step_dir_counter_move(&data->step_dir_ctx, micro_steps);
}

static int drv8424_is_moving(const struct device *dev, bool *is_moving)
{
	struct drv8424_data *data = dev->data;
	return step_dir_is_moving(&data->step_dir_ctx, is_moving);
}

static int drv8424_set_actual_position(const struct device *dev, int32_t position)
{
	struct drv8424_data *data = dev->data;
	return step_dir_set_actual_position(&data->step_dir_ctx, position);
}

static int drv8424_get_actual_position(const struct device *dev, int32_t *position)
{
	struct drv8424_data *data = dev->data;
	return step_dir_get_actual_position(&data->step_dir_ctx, position);
}

static int drv8424_set_target_position(const struct device *dev, int32_t position)
{
	struct drv8424_data *data = dev->data;

	if (!data->enabled) {
		return -ENODEV;
	}

	return step_dir_counter_set_target_position(&data->step_dir_ctx, position);
}

static int drv8424_set_max_velocity(const struct device *dev, uint32_t velocity)
{
	struct drv8424_data *data = dev->data;
	return step_dir_set_max_velocity(&data->step_dir_ctx, velocity);
}

static int drv8424_enable_constant_velocity_mode(const struct device *dev,
						 const enum stepper_direction direction,
						 const uint32_t velocity)
{
	struct drv8424_data *data = dev->data;

	if (!data->enabled) {
		return -ENODEV;
	}

	return step_dir_counter_enable_constant_velocity_mode(&data->step_dir_ctx, direction,
							      velocity);
}

static int drv8424_set_micro_step_res(const struct device *dev,
				      enum stepper_micro_step_resolution micro_step_res)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	int ret = 0;

	int m0_value = 0;
	int m1_value = 0;

	/* 0: low
	 * 1: high
	 * 2: Hi-Z
	 * 3: 330kΩ
	 */
	switch (micro_step_res) {
	case STEPPER_MICRO_STEP_1:
		m0_value = 0;
		m1_value = 0;
		break;
	case STEPPER_MICRO_STEP_2:
		m0_value = 2;
		m1_value = 0;
		break;
	case STEPPER_MICRO_STEP_4:
		m0_value = 0;
		m1_value = 1;
		break;
	case STEPPER_MICRO_STEP_8:
		m0_value = 1;
		m1_value = 1;
		break;
	case STEPPER_MICRO_STEP_16:
		m0_value = 2;
		m1_value = 1;
		break;
	case STEPPER_MICRO_STEP_32:
		m0_value = 0;
		m1_value = 2;
		break;
	case STEPPER_MICRO_STEP_64:
		m0_value = 2;
		m1_value = 3;
		break;
	case STEPPER_MICRO_STEP_128:
		m0_value = 2;
		m1_value = 2;
		break;
	case STEPPER_MICRO_STEP_256:
		m0_value = 1;
		m1_value = 2;
		break;
	default:
		return -EINVAL;
	};

	/* Reset m0 pin as it may have been disconnected. */
	ret = gpio_pin_configure_dt(&config->m0_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to reset m0_pin (error: %d)", dev->name, ret);
		drv8424_microstep_recovery(dev);
		return ret;
	}

	/* Reset m1 pin as it may have been disconnected. */
	ret = gpio_pin_configure_dt(&config->m1_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to reset m1_pin (error: %d)", dev->name, ret);
		drv8424_microstep_recovery(dev);
		return ret;
	}

	/* Set m0 pin */
	switch (m0_value) {
	case 0:
		ret = gpio_pin_set_dt(&config->m0_pin, 0);
		break;
	case 1:
		ret = gpio_pin_set_dt(&config->m0_pin, 1);
		break;
	case 2:
		/* Hi-Z is set by configuring pin as disconnected, not
		 * all gpio controllers support this. */
		ret = gpio_pin_configure_dt(&config->m0_pin, GPIO_DISCONNECTED);
		break;
	default:
		break;
	}

	if (ret != 0) {
		LOG_ERR("%s: Failed to set m0_pin (error: %d)", dev->name, ret);
		drv8424_microstep_recovery(dev);
		return ret;
	}

	switch (m1_value) {
	case 0:
		ret = gpio_pin_set_dt(&config->m1_pin, 0);
		break;
	case 1:
		ret = gpio_pin_set_dt(&config->m1_pin, 1);
		break;
	case 2:
		/* Hi-Z is set by configuring pin as disconnected, not
		 * all gpio controllers support this. */
		ret = gpio_pin_configure_dt(&config->m1_pin, GPIO_DISCONNECTED);
		break;
	default:
		break;
	}

	if (ret != 0) {
		LOG_ERR("%s: Failed to set m1_pin (error: %d)", dev->name, ret);
		drv8424_microstep_recovery(dev);
		return ret;
	}

	data->ms_res = micro_step_res;
	data->pin_states.m0 = m0_value;
	data->pin_states.m1 = m1_value;

	return 0;
}

static int drv8424_get_micro_step_res(const struct device *dev,
				      enum stepper_micro_step_resolution *micro_step_res)
{
	struct drv8424_data *data = dev->data;
	*micro_step_res = data->ms_res;
	return 0;
}

static int drv8424_set_event_callback(const struct device *dev, stepper_event_callback_t callback,
				      void *user_data)
{
	struct drv8424_data *data = dev->data;
	return step_dir_set_event_callback(&data->step_dir_ctx, callback, user_data);
}

static int drv8424_init(const struct device *dev)
{
	const struct drv8424_config *const config = dev->config;
	struct drv8424_data *const data = dev->data;
	int ret = 0;

	ret = step_dir_counter_init(&data->step_dir_ctx, &data->step_dir_counter_data, dev,
				    &config->step_pin, &config->dir_pin, config->counter);
	if (ret != 0) {
		LOG_ERR("%s: Failed to initialize step/dir stepper (error %d)", dev->name, ret);
		return ret;
	}

	/* Configure sleep pin if it is available */
	if (config->sleep_pin.port != NULL) {
		ret = gpio_pin_configure_dt(&config->sleep_pin, GPIO_OUTPUT_ACTIVE);
		if (ret != 0) {
			LOG_ERR("%s: Failed to configure sleep_pin (error: %d)", dev->name, ret);
			return ret;
		}
		data->pin_states.sleep = 1U;
	}

	/* Configure enable pin if it is available */
	if (config->en_pin.port != NULL) {
		ret = gpio_pin_configure_dt(&config->en_pin, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("%s: Failed to configure en_pin (error: %d)", dev->name, ret);
			return ret;
		}
		data->pin_states.en = 0U;
	}

	/* Configure microstep pin 0 */
	ret = gpio_pin_configure_dt(&config->m0_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to configure m0_pin (error: %d)", dev->name, ret);
		return ret;
	}
	data->pin_states.m0 = 0U;

	/* Configure microstep pin 1 */
	ret = gpio_pin_configure_dt(&config->m1_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to configure m1_pin (error: %d)", dev->name, ret);
		return ret;
	}
	data->pin_states.m1 = 0U;

	return 0;
}

const struct stepper_driver_api drv8424_stepper_api = {
	.enable = drv8424_enable,
	.move = drv8424_move,
	.is_moving = drv8424_is_moving,
	.set_actual_position = drv8424_set_actual_position,
	.get_actual_position = drv8424_get_actual_position,
	.set_target_position = drv8424_set_target_position,
	.set_max_velocity = drv8424_set_max_velocity,
	.enable_constant_velocity_mode = drv8424_enable_constant_velocity_mode,
	.set_micro_step_res = drv8424_set_micro_step_res,
	.get_micro_step_res = drv8424_get_micro_step_res,
	.set_event_callback = drv8424_set_event_callback,
};

#define DRV8424_DEVICE(inst)                                                                       \
                                                                                                   \
	static const struct drv8424_config drv8424_config_##inst = {                               \
		.dir_pin = GPIO_DT_SPEC_INST_GET(inst, dir_gpios),                                 \
		.step_pin = GPIO_DT_SPEC_INST_GET(inst, step_gpios),                               \
		.sleep_pin = GPIO_DT_SPEC_INST_GET_OR(inst, sleep_gpios, {0}),                     \
		.en_pin = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                           \
		.m0_pin = GPIO_DT_SPEC_INST_GET_OR(inst, m0_gpios, {0}),                           \
		.m1_pin = GPIO_DT_SPEC_INST_GET_OR(inst, m1_gpios, {0}),                           \
		.counter = DEVICE_DT_GET(DT_INST_PHANDLE(inst, counter)),                          \
	};                                                                                         \
                                                                                                   \
	static struct drv8424_data drv8424_data_##inst = {                                         \
		.ms_res = STEPPER_MICRO_STEP_1,                                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &drv8424_init,          /* Init */                             \
			      NULL,                         /* PM */                               \
			      &drv8424_data_##inst,         /* Data */                             \
			      &drv8424_config_##inst,       /* Config */                           \
			      POST_KERNEL,                  /* Init stage */                       \
			      CONFIG_STEPPER_INIT_PRIORITY, /* Init priority */                    \
			      &drv8424_stepper_api);        /* API */

DT_INST_FOREACH_STATUS_OKAY(DRV8424_DEVICE)
