/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Navimatix GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_drv8424

#include <zephyr/kernel.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper_drv8424.h>
#include "../step_dir/step_dir_stepper_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv8424, CONFIG_STEPPER_LOG_LEVEL);

/* Enable and wake up times of the drv84xx stepper controller family. Only after they have elapsed
 * are controller output signals guaranteed to be valid.
 */
#define DRV84XX_ENABLE_TIME  K_USEC(5)
#define DRV84XX_WAKE_UP_TIME K_USEC(1200)

/**
 * @brief DRV8424 stepper driver configuration data.
 *
 * This structure contains all of the devicetree specifications for the pins
 * needed by a given DRV8424 stepper driver.
 */
struct drv8424_config {
	struct step_dir_stepper_common_config common;
	struct gpio_dt_spec sleep_pin;
	struct gpio_dt_spec en_pin;
	struct gpio_dt_spec m0_pin;
	struct gpio_dt_spec m1_pin;
	struct gpio_dt_spec fault_pin;
};

/* Struct for storing the states of output pins. */
struct drv8424_pin_states {
	uint8_t sleep: 1;
	uint8_t en: 1;
	uint8_t m0: 2;
	uint8_t m1: 2;
};

/**
 * @brief DRV84XX stepper driver data.
 */
struct drv8424_data {
	const struct step_dir_stepper_common_data common;
	const struct device *dev;
	bool enabled;
	struct drv8424_pin_states pin_states;
	enum stepper_micro_step_resolution ustep_res;
	struct gpio_callback fault_cb_data;
};

STEP_DIR_STEPPER_STRUCT_CHECK(struct drv8424_config, struct drv8424_data);

static int drv8424_set_microstep_pin(const struct device *dev, const struct gpio_dt_spec *pin,
				     int value)
{
	int ret;

	/* Reset microstep pin as it may have been disconnected. */
	ret = gpio_pin_configure_dt(pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to reset micro-step pin (error: %d)", dev->name, ret);
		return ret;
	}

	/* Set microstep pin */
	switch (value) {
	case 0:
		ret = gpio_pin_set_dt(pin, 0);
		break;
	case 1:
		ret = gpio_pin_set_dt(pin, 1);
		break;
	case 2:
		/* Hi-Z is set by configuring pin as disconnected, not
		 * all gpio controllers support this.
		 */
		ret = gpio_pin_configure_dt(pin, GPIO_DISCONNECTED);
		break;
	default:
		break;
	}

	if (ret != 0) {
		LOG_ERR("%s: Failed to set micro-step pin (error: %d)", dev->name, ret);
		return ret;
	}
	return 0;
}

/*
 * If microstep setter fails, attempt to recover into previous state.
 */
int drv8424_microstep_recovery(const struct device *dev)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	int ret;

	uint8_t m0_value = data->pin_states.m0;
	uint8_t m1_value = data->pin_states.m1;

	ret = drv8424_set_microstep_pin(dev, &config->m0_pin, m0_value);
	if (ret != 0) {
		LOG_ERR("%s: Failed to restore microstep configuration (error: %d)", dev->name,
			ret);
		return ret;
	}

	ret = drv8424_set_microstep_pin(dev, &config->m1_pin, m1_value);
	if (ret != 0) {
		LOG_ERR("%s: Failed to restore microstep configuration (error: %d)", dev->name,
			ret);
		return ret;
	}

	return 0;
}

static int drv8424_check_en_sleep_pin(const struct drv8424_config *config)
{
	bool has_sleep_pin = config->sleep_pin.port != NULL;
	bool has_enable_pin = config->en_pin.port != NULL;

	if (!has_sleep_pin && !has_enable_pin) {
		LOG_ERR("Failed to enable/disable device, neither sleep pin nor enable pin are "
			"available. The device is always on.");
		return -ENOTSUP;
	}

	return 0;
}

static int drv8424_set_en_pin_state(const struct device *dev, bool enable)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	bool has_enable_pin = config->en_pin.port != NULL;
	int ret;

	if (has_enable_pin) {
		ret = gpio_pin_set_dt(&config->en_pin, enable);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set en_pin (error: %d)", dev->name, ret);
			return ret;
		}
		data->pin_states.en = enable ? 1U : 0U;
	}

	return 0;
}

static int drv8424_set_sleep_pin_state(const struct device *dev, bool enable)
{
	int ret;
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	bool has_sleep_pin = config->sleep_pin.port != NULL;

	if (has_sleep_pin) {
		ret = gpio_pin_set_dt(&config->sleep_pin, !enable);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set sleep_pin (error: %d)", dev->name, ret);
			return ret;
		}
		data->pin_states.sleep = enable ? 0U : 1U;
	}

	return 0;
}

static int drv8424_enable(const struct device *dev)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	bool has_enable_pin = config->en_pin.port != NULL;
	bool has_sleep_pin = config->sleep_pin.port != NULL;
	bool has_fault_pin = config->fault_pin.port != NULL;
	k_timeout_t enable_timeout;
	int ret;

	ret = drv8424_check_en_sleep_pin(config);
	if (ret != 0) {
		return ret;
	}

	ret = drv8424_set_sleep_pin_state(dev, true);
	if (ret != 0) {
		return ret;
	}

	ret = drv8424_set_en_pin_state(dev, true);
	if (ret != 0) {
		return ret;
	}

	if (has_enable_pin) {
		enable_timeout = DRV84XX_ENABLE_TIME;
	}

	if (has_sleep_pin) {
		enable_timeout = DRV84XX_WAKE_UP_TIME;
	}

	if (has_fault_pin) {
		/* Wait after enable/wakeup until the fault pin is guaranteed to be in the
		 * proper state.
		 */
		k_sleep(enable_timeout);
		ret = gpio_add_callback_dt(&config->fault_pin, &data->fault_cb_data);
		if (ret != 0) {
			LOG_ERR("%s: Failed to add fault callback (error: %d)", dev->name, ret);
			return ret;
		}
	}

	data->enabled = true;

	return ret;
}

static int drv8424_disable(const struct device *dev)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	bool has_fault_pin = config->fault_pin.port != NULL;
	int ret;

	ret = drv8424_check_en_sleep_pin(config);
	if (ret != 0) {
		return ret;
	}

	ret = drv8424_set_sleep_pin_state(dev, false);
	if (ret != 0) {
		return ret;
	}

	ret = drv8424_set_en_pin_state(dev, false);
	if (ret != 0) {
		return ret;
	}

	if (has_fault_pin) {
		ret = gpio_remove_callback_dt(&config->fault_pin, &data->fault_cb_data);
		if (ret != 0) {
			LOG_ERR("%s: Failed to remove fault callback (error: %d)", dev->name, ret);
			return ret;
		}
	}

	config->common.timing_source->stop(dev);

	data->enabled = false;

	return ret;
}

static int drv8424_set_micro_step_res(const struct device *dev,
				      enum stepper_micro_step_resolution micro_step_res)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	int ret;

	uint8_t m0_value = 0;
	uint8_t m1_value = 0;

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
		return -ENOTSUP;
	};

	ret = drv8424_set_microstep_pin(dev, &config->m0_pin, m0_value);
	if (ret != 0) {
		return ret;
	}

	ret = drv8424_set_microstep_pin(dev, &config->m1_pin, m1_value);
	if (ret != 0) {
		return ret;
	}

	data->ustep_res = micro_step_res;
	data->pin_states.m0 = m0_value;
	data->pin_states.m1 = m1_value;

	return 0;
}

static int drv8424_get_micro_step_res(const struct device *dev,
				      enum stepper_micro_step_resolution *micro_step_res)
{
	struct drv8424_data *data = dev->data;
	*micro_step_res = data->ustep_res;
	return 0;
}

static int drv8424_move_to(const struct device *dev, int32_t target)
{
	struct drv8424_data *data = dev->data;

	if (!data->enabled) {
		LOG_ERR("Failed to move to target position, device is not enabled");
		return -ECANCELED;
	}

	return step_dir_stepper_common_move_to(dev, target);
}

static int drv8424_move_by(const struct device *dev, int32_t steps)
{
	struct drv8424_data *data = dev->data;

	if (!data->enabled) {
		LOG_ERR("Failed to move by delta, device is not enabled");
		return -ECANCELED;
	}

	return step_dir_stepper_common_move_by(dev, steps);
}

static int drv8424_run(const struct device *dev, enum stepper_direction direction)
{
	struct drv8424_data *data = dev->data;

	if (!data->enabled) {
		LOG_ERR("Failed to run stepper, device is not enabled");
		return -ECANCELED;
	}

	return step_dir_stepper_common_run(dev, direction);
}

void fault_event(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct drv8424_data *data = CONTAINER_OF(cb, struct drv8424_data, fault_cb_data);

	stepper_trigger_callback(data->dev, STEPPER_EVENT_FAULT_DETECTED);
}

static int drv8424_init(const struct device *dev)
{
	const struct drv8424_config *const config = dev->config;
	struct drv8424_data *const data = dev->data;
	int ret;

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

	ret = drv8424_set_micro_step_res(dev, data->ustep_res);
	if (ret != 0) {
		return ret;
	}

	ret = step_dir_stepper_common_init(dev);
	if (ret != 0) {
		LOG_ERR("Failed to initialize common step direction stepper (error: %d)", ret);
		return ret;
	}

	/* Configure fault pin if it is available */
	if (config->fault_pin.port != NULL) {
		ret = gpio_pin_configure_dt(&config->fault_pin, GPIO_INPUT);
		if (ret != 0) {
			LOG_ERR("%s: Failed to configure fault_pin (error: %d)", dev->name, ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&config->fault_pin,
						      GPIO_INT_EDGE_TO_INACTIVE);
		if (ret != 0) {
			LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret,
				config->fault_pin.port->name, config->fault_pin.pin);
			return ret;
		}

		gpio_init_callback(&data->fault_cb_data, fault_event, BIT(config->fault_pin.pin));
	}

	return 0;
}

static DEVICE_API(stepper, drv8424_stepper_api) = {
	.enable = drv8424_enable,
	.disable = drv8424_disable,
	.move_by = drv8424_move_by,
	.move_to = drv8424_move_to,
	.is_moving = step_dir_stepper_common_is_moving,
	.set_reference_position = step_dir_stepper_common_set_reference_position,
	.get_actual_position = step_dir_stepper_common_get_actual_position,
	.set_microstep_interval = step_dir_stepper_common_set_microstep_interval,
	.run = drv8424_run,
	.stop = step_dir_stepper_common_stop,
	.set_micro_step_res = drv8424_set_micro_step_res,
	.get_micro_step_res = drv8424_get_micro_step_res,
	.set_event_callback = step_dir_stepper_common_set_event_callback,
};

#define DRV8424_DEVICE(inst)                                                                       \
                                                                                                   \
	static const struct drv8424_config drv8424_config_##inst = {                               \
		.common = STEP_DIR_STEPPER_DT_INST_COMMON_CONFIG_INIT(inst),                       \
		.sleep_pin = GPIO_DT_SPEC_INST_GET_OR(inst, sleep_gpios, {0}),                     \
		.en_pin = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                           \
		.m0_pin = GPIO_DT_SPEC_INST_GET(inst, m0_gpios),                                   \
		.m1_pin = GPIO_DT_SPEC_INST_GET(inst, m1_gpios),                                   \
		.fault_pin = GPIO_DT_SPEC_INST_GET_OR(inst, fault_gpios, {0}),                     \
	};                                                                                         \
                                                                                                   \
	static struct drv8424_data drv8424_data_##inst = {                                         \
		.common = STEP_DIR_STEPPER_DT_INST_COMMON_DATA_INIT(inst),                         \
		.ustep_res = DT_INST_PROP(inst, micro_step_res),                                   \
		.dev = DEVICE_DT_GET(DT_INST(inst, DT_DRV_COMPAT)),                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &drv8424_init, NULL, &drv8424_data_##inst,                     \
			      &drv8424_config_##inst, POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,   \
			      &drv8424_stepper_api);

DT_INST_FOREACH_STATUS_OKAY(DRV8424_DEVICE)
