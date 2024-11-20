/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 TiaC Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util.h"
#include "zephyr/sys_clock.h"
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
	/** Back pointer to the device, e.g. for access to config */
	const struct device *dev;
	/** Whether the device is enabled */
	bool enabled;
	/** Struct containing the states of different pins. */
	struct drv8424_pin_states pin_states;
	/** Current microstep resolution. */
	enum stepper_micro_step_resolution ms_res;
	/** Maximum velocity (steps/s). */
	uint32_t max_velocity;
	/** Actual current position */
	int32_t actual_position;
	/** Target position */;
	int32_t target_position;
	/** Whether the motor is currently moving */
	bool is_moving;
	/** Whether we're in constant velocity mode */
	bool constant_velocity;
	/** Which direction we're going if in constat velocity mode */
	enum stepper_direction constant_velocity_direction;
	/** Event handler registered by user */
	stepper_event_callback_t event_callback;
	/** Event handler user data */
	void *event_callback_user_data;
	/** Whether the step signal is currently high or low. */
	bool step_signal_high;
	/** Struct containing counter top configuration. */
	struct counter_top_cfg counter_top_cfg;
};

struct drv8424_callback_work {
	/** The work item itself. */
	struct k_work work;
	/** Type of event to handle. */
	enum stepper_event event;
	/** Driver instance data. */
	struct drv8424_data *driver_data;
};

static void drv8424_user_callback_work_fn(struct k_work *work)
{
	/* Get required data from outer struct */
	struct drv8424_callback_work *callback_work =
		CONTAINER_OF(work, struct drv8424_callback_work, work);
	struct drv8424_data *driver_data = callback_work->driver_data;
	enum stepper_event event = callback_work->event;

	/* Run the callback */
	if (driver_data->event_callback != NULL) {
		driver_data->event_callback(driver_data->dev, event,
					    driver_data->event_callback_user_data);
	}

	/* Free the work item */
	k_free(callback_work);
}

static int drv8424_schedule_user_callback(struct drv8424_data *data, enum stepper_event event)
{
	/* Create and schedule work item to run user callback */
	struct drv8424_callback_work *callback_work =
		k_malloc(sizeof(struct drv8424_callback_work));
	if (callback_work == NULL) {
		return -ENOMEM;
	}

	/* Fill out data for work item */
	*callback_work = (struct drv8424_callback_work){
		.driver_data = data,
		.event = STEPPER_EVENT_STEPS_COMPLETED,
	};
	k_work_init(&callback_work->work, drv8424_user_callback_work_fn);

	/* Try to submit the item */
	int ret = k_work_submit(&callback_work->work);
	/* ret == 0 shouldn't happen (it would mean we've submitted this same item before), but
	 * there's no reason to force a segfault in case it somehow does happen. */
	if (ret != 1 && ret != 0) {
		/* We failed to submit the item, so we need to free it here */
		k_free(callback_work);
		return -ENODEV; // FIXME: What's the correct error code here?
	}

	return 0;
}

static void drv8424_positioning_top_interrupt(const struct device *dev, void *user_data)
{
	struct drv8424_data *data = (struct drv8424_data *)user_data;
	const struct drv8424_config *config = (struct drv8424_config *)data->dev->config;

	if (!data->constant_velocity) {
		/* Check if target position is reached */
		if (data->actual_position == data->target_position) {
			counter_stop(config->counter);
			if (data->event_callback != NULL) {
				/* Ignore return value since we can't do anything about it anyway */
				drv8424_schedule_user_callback(data, STEPPER_EVENT_STEPS_COMPLETED);
			}
			data->is_moving = false;
			return;
		}
	}

	/* Determine direction we're going in for counting purposes */
	enum stepper_direction direction = 0;
	if (data->constant_velocity) {
		direction = data->constant_velocity_direction;
	} else {
		direction = (data->target_position >= data->actual_position)
				    ? STEPPER_DIRECTION_POSITIVE
				    : STEPPER_DIRECTION_NEGATIVE;
	}

	/* Switch step pin on or off depending position in step period */
	if (data->step_signal_high) {
		/* Generate a falling edge and count a completed step */
		gpio_pin_set_dt(&config->step_pin, 0);
		if (direction == STEPPER_DIRECTION_POSITIVE) {
			data->actual_position++;
		} else {
			data->actual_position--;
		}
	} else {
		/* Generate a rising edge */
		gpio_pin_set_dt(&config->step_pin, 1);
	}

	data->step_signal_high = !data->step_signal_high;
}

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
		counter_stop(config->counter);
		data->is_moving = false;
	}

	return 0;
}

static int drv8424_set_counter_frequency(const struct device *dev, uint32_t freq_hz)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	int ret = 0;

	if (freq_hz == 0) {
		return -EINVAL;
	}

	data->counter_top_cfg.ticks = counter_us_to_ticks(config->counter, USEC_PER_SEC / freq_hz);

	ret = counter_set_top_value(config->counter, &data->counter_top_cfg);
	if (ret != 0) {
		LOG_ERR("%s: Failed to set counter top value (error: %d)", dev->name, ret);
		return ret;
	}

	return 0;
}

static int drv8424_start_positioning(const struct device *dev)
{

	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	int ret = 0;

	/* Unset constant velocity flag if present */
	data->constant_velocity = false;

	/* Set direction pin */
	int dir_value = (data->target_position >= data->actual_position) ? 1 : 0;
	ret = gpio_pin_set_dt(&config->dir_pin, dir_value);
	if (ret != 0) {
		LOG_ERR("%s: Failed to set direction pin (error %d)", dev->name, ret);
		return ret;
	}

	/* Lock interrupts while modifying counter settings */
	int key = irq_lock();

	/* Set counter to correct frequency */
	ret = drv8424_set_counter_frequency(dev, data->max_velocity * 2);
	if (ret != 0) {
		LOG_ERR("%s: Failed to set counter frequency (error %d)", dev->name, ret);
		goto end;
	}

	data->is_moving = true;

	/* Start counter */
	ret = counter_start(config->counter);
	if (ret != 0) {
		LOG_ERR("%s: Failed to start counter (error: %d)", dev->name, ret);
		data->is_moving = false;
		goto end;
	}

end:
	irq_unlock(key);
	return ret;
}

static int drv8424_move(const struct device *dev, int32_t micro_steps)
{
	struct drv8424_data *data = dev->data;
	int ret = 0;

	if (data->max_velocity == 0) {
		LOG_ERR("%s: Invalid max. velocity %d configured", dev->name, data->max_velocity);
		return -EINVAL;
	}

	if (!data->enabled) {
		return -ENODEV;
	}

	/* Compute target position */
	data->target_position = data->actual_position + micro_steps;

	ret = drv8424_start_positioning(dev);
	if (ret != 0) {
		LOG_ERR("%s: Failed to begin positioning (error %d)", dev->name, ret);
		return ret;
	};

	return 0;
}

static int drv8424_is_moving(const struct device *dev, bool *is_moving)
{
	struct drv8424_data *data = dev->data;

	*is_moving = data->is_moving;

	return 0;
}

static int drv8424_set_actual_position(const struct device *dev, int32_t position)
{
	struct drv8424_data *data = dev->data;

	data->actual_position = position;

	return 0;
}

static int drv8424_get_actual_position(const struct device *dev, int32_t *position)
{
	struct drv8424_data *data = dev->data;

	*position = data->actual_position;

	return 0;
}

static int drv8424_set_target_position(const struct device *dev, int32_t position)
{
	struct drv8424_data *data = dev->data;
	int ret = 0;

	if (!data->enabled) {
		return -ENODEV;
	}

	data->target_position = position;

	ret = drv8424_start_positioning(dev);
	if (ret != 0) {
		LOG_ERR("%s: Failed to begin positioning (error %d)", dev->name, ret);
		return ret;
	};

	return 0;
}

static int drv8424_set_max_velocity(const struct device *dev, uint32_t velocity)
{
	struct drv8424_data *data = dev->data;
	data->max_velocity = velocity;
	return 0;
}

static int drv8424_enable_constant_velocity_mode(const struct device *dev,
						 const enum stepper_direction direction,
						 const uint32_t velocity)
{
	const struct drv8424_config *config = dev->config;
	struct drv8424_data *data = dev->data;
	int ret = 0;

	if (!data->enabled) {
		return -ENODEV;
	}

	/* Set direction pin */
	int dir_value = (direction == STEPPER_DIRECTION_POSITIVE) ? 1 : 0;
	ret = gpio_pin_set_dt(&config->dir_pin, dir_value);
	if (ret != 0) {
		LOG_ERR("%s: Failed to set direction pin (error %d)", dev->name, ret);
		return ret;
	}

	/* Lock interrupts while modifying settings used by ISR */
	int key = irq_lock();

	/* Set data used in counter interrupt */
	data->constant_velocity = true;
	data->constant_velocity_direction = direction;
	data->is_moving = true;

	/* Treat velocity 0 by not stepping at all */
	if (velocity == 0) {
		ret = counter_stop(config->counter);
		if (ret != 0) {
			LOG_ERR("%s: Failed to stop counter (error %d)", dev->name, ret);
			data->is_moving = false;
			goto end;
		}
		data->is_moving = false;
	} else {
		ret = drv8424_set_counter_frequency(dev, velocity * 2);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set counter frequency (error %d)", dev->name, ret);
			data->is_moving = false;
			goto end;
		}

		/* Start counter */
		ret = counter_start(config->counter);
		if (ret != 0) {
			LOG_ERR("%s: Failed to start counter (error %d)", dev->name, ret);
			data->is_moving = false;
			goto end;
		}
	}

end:
	irq_unlock(key);
	return ret;
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

	data->event_callback = callback;
	data->event_callback_user_data = user_data;

	return 0;
}

static int drv8424_init(const struct device *dev)
{
	const struct drv8424_config *const config = dev->config;
	struct drv8424_data *const data = dev->data;
	int ret = 0;

	/* Set device back pointer */
	data->dev = dev;

	/* Configure direction pin */
	ret = gpio_pin_configure_dt(&config->dir_pin, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to configure dir_pin (error: %d)", dev->name, ret);
		return ret;
	}

	/* Configure step pin */
	ret = gpio_pin_configure_dt(&config->step_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to configure step_pin (error: %d)", dev->name, ret);
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

	/* Set initial counter configuration */
	data->step_signal_high = false;
	data->counter_top_cfg.callback = drv8424_positioning_top_interrupt;
	data->counter_top_cfg.user_data = data;
	data->counter_top_cfg.flags = 0;
	data->counter_top_cfg.ticks = counter_us_to_ticks(config->counter, 1000000);

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
		.actual_position = 0,                                                              \
		.target_position = 0,                                                              \
		.is_moving = false,                                                                \
		.event_callback = NULL,                                                            \
		.event_callback_user_data = NULL,                                                  \
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
