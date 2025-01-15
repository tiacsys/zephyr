/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Navimatix GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_drv8424_accel

#include <zephyr/sys/util.h>
#include <math.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/stepper/stepper_drv8424_accel.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv8424_accel, CONFIG_STEPPER_LOG_LEVEL);

/* Number of steps for which the accurate ramp calculation algorithm is used, once the error is low
 * enough, an approximation algorithm is used
 */
#define ACCURATE_STEPS 15

/** number of picoseconds per micorsecond */
#define PSEC_PER_USEC (1000 * NSEC_PER_USEC)

/** number of picoseconds per second */
#define PSEC_PER_SEC ((uint64_t)PSEC_PER_USEC * (uint64_t)USEC_PER_SEC)

/** divisor that is used several times for tick calculation */
#define TICK_DIVISOR (2 * PSEC_PER_USEC)

/**
 * @brief DRV8424 stepper driver configuration data.
 *
 * This structure contains all of the devicetree specifications for the pins
 * needed by a given DRV8424 stepper driver.
 */
struct drv8424_accel_config {
	struct gpio_dt_spec dir_pin;
	struct gpio_dt_spec step_pin;
	struct gpio_dt_spec sleep_pin;
	struct gpio_dt_spec en_pin;
	struct gpio_dt_spec m0_pin;
	struct gpio_dt_spec m1_pin;
	const struct device *counter;
	uint32_t acceleration; /* In micro-steps/s^2*/
};

/* Struct for storing the states of output pins. */
struct drv8424_accel_pin_states {
	uint8_t sleep: 1;
	uint8_t en: 1;
	uint8_t m0: 2;
	uint8_t m1: 2;
};

/* Struct for storing ramp information */
struct drv8424_accel_ramp_data {
	/* Iterator for the current step in the current phase. */
	uint32_t step_index;
	uint64_t current_time_int; /* in pikoseconds ps */
	uint64_t base_time_int;    /* in ps */
	/* Number of steps to take during constant speed phase. */
	uint32_t const_steps;
	/* Number of steps to take during deceleration phase. */
	uint32_t decel_steps;
	/* Number of steps to take during initial phase (accel or decel). */
	uint32_t accel_steps;
	const struct gpio_dt_spec *step_pin;
	uint32_t ticks_us;
	/** Velocity during const velocity phase (steps/s). */
	uint32_t const_velocity;
	const struct device *counter;
};

/**
 * @brief DRV8424 accel stepper driver data.
 *
 * This structure contains mutable data used by a DRV8424 stepper driver.
 */
struct drv8424_accel_data {
	/** Back pointer to the device, e.g. for access to config */
	const struct device *dev;
	bool enabled;
	struct drv8424_accel_pin_states pin_states;
	enum stepper_micro_step_resolution ustep_res;
	uint32_t max_velocity;
	uint32_t current_velocity;
	int32_t reference_position;
	bool is_moving;
	bool constant_velocity;
	enum stepper_direction direction;
	stepper_event_callback_t event_callback;
	void *event_callback_user_data;
	bool step_signal_high;
	struct counter_top_cfg counter_top_cfg;
	struct drv8424_accel_ramp_data ramp_data;
};

struct drv8424_accel_callback_work {
	struct k_work work;
	enum stepper_event event;
	struct drv8424_accel_data *driver_data;
};

static int drv8424_accel_set_microstep_pin(const struct device *dev, const struct gpio_dt_spec *pin,
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

static void drv8424_accel_user_callback_work_fn(struct k_work *work)
{
	/* Get required data from outer struct */
	struct drv8424_accel_callback_work *callback_work =
		CONTAINER_OF(work, struct drv8424_accel_callback_work, work);
	struct drv8424_accel_data *driver_data = callback_work->driver_data;
	enum stepper_event event = callback_work->event;

	/* Run the callback */
	if (driver_data->event_callback != NULL) {
		driver_data->event_callback(driver_data->dev, event,
					    driver_data->event_callback_user_data);
	}

	/* Free the work item */
	k_free(callback_work);
}

static int drv8424_accel_schedule_user_callback(struct drv8424_accel_data *data,
						enum stepper_event event)
{
	/* Create and schedule work item to run user callback */
	struct drv8424_accel_callback_work *callback_work =
		k_malloc(sizeof(struct drv8424_accel_callback_work));
	if (callback_work == NULL) {
		return -ENOMEM;
	}

	/* Fill out data for work item */
	*callback_work = (struct drv8424_accel_callback_work){
		.driver_data = data,
		.event = STEPPER_EVENT_STEPS_COMPLETED,
	};
	k_work_init(&callback_work->work, drv8424_accel_user_callback_work_fn);

	/* Try to submit the item */
	int ret = k_work_submit(&callback_work->work);
	/* ret == 0 shouldn't happen (it would mean we've submitted this same item before), but
	 * there's no reason to force a segfault in case it somehow does happen.
	 */
	if (ret != 1 && ret != 0) {
		/* We failed to submit the item, so we need to free it here */
		k_free(callback_work);
		return -ENODEV; // FIXME: What's the correct error code here?
	}

	return 0;
}

static void drv8424_accel_positioning_smooth_deceleration(const struct device *dev, void *user_data)
{
	struct drv8424_accel_data *data = user_data;
	uint64_t new_time_int;

	/* Switch step pin on or off depending position in step period and calculate length of the
	 * new step period
	 */
	if (data->step_signal_high) {
		gpio_pin_set_dt(data->ramp_data.step_pin, 0);
		data->ramp_data.step_index--;
		if (data->direction == STEPPER_DIRECTION_POSITIVE) {
			data->reference_position++;
		} else {
			data->reference_position--;
		}
	} else {
		/* Use Approximation as long as error is small enough */
		if (data->ramp_data.step_index > ACCURATE_STEPS) {
			/*
			 * Iterative algorithm using Taylor expansion, see 'Generate stepper-motor
			 * speed profiles in real time' (2005) by David Austin
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			uint64_t t_n = data->ramp_data.current_time_int;
			uint64_t adjust = 2 * t_n / (4 * data->ramp_data.step_index + 1);
			new_time_int = t_n + adjust;
		} else {
			/* Use accurate (but expensive) calculation when error would be large
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			new_time_int = data->ramp_data.base_time_int *
				       (sqrtf(data->ramp_data.step_index + 0.0f) -
					sqrtf(data->ramp_data.step_index - 1.0f));
		}
		data->counter_top_cfg.ticks =
			data->ramp_data.ticks_us * DIV_ROUND_UP(new_time_int, TICK_DIVISOR);
		data->ramp_data.current_time_int = new_time_int;
		data->current_velocity = PSEC_PER_SEC / new_time_int;
		gpio_pin_set_dt(data->ramp_data.step_pin, 1);
		counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
	}

	data->step_signal_high = !data->step_signal_high;

	/* Stop Counter if positioning_smooth move is finished */
	if (data->ramp_data.step_index == 0) {
		counter_stop(data->ramp_data.counter);
		if (data->event_callback != NULL) {
			/* Ignore return value since we can't do anything about it anyway */
			drv8424_accel_schedule_user_callback(data, STEPPER_EVENT_STEPS_COMPLETED);
		}
		data->is_moving = false;
		gpio_pin_set_dt(data->ramp_data.step_pin, 0);
		data->step_signal_high = false;
		data->current_velocity = 0;
	}
}

static void drv8424_accel_positioning_smooth_constant(const struct device *dev, void *user_data)
{
	struct drv8424_accel_data *data = user_data;

	/* Switch step pin on or off depending position in step period */
	if (data->step_signal_high) {
		if (!data->constant_velocity) {
			data->ramp_data.step_index++;
		}
		gpio_pin_set_dt(data->ramp_data.step_pin, 0);
		if (data->direction == STEPPER_DIRECTION_POSITIVE) {
			data->reference_position++;
		} else {
			data->reference_position--;
		}
	} else {
		gpio_pin_set_dt(data->ramp_data.step_pin, 1);
		if (data->ramp_data.step_index % 5 == 0) {
		}
	}

	data->step_signal_high = !data->step_signal_high;

	/* If constant speed section finished, switch to deceleration */
	if (data->ramp_data.step_index == data->ramp_data.const_steps) {
		data->ramp_data.step_index = data->ramp_data.decel_steps;
		data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_deceleration;
		counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
	}
}

static void drv8424_accel_positioning_smooth_acceleration(const struct device *dev, void *user_data)
{
	struct drv8424_accel_data *data = user_data;
	uint64_t new_time_int;

	/* Switch step pin on or off depending position in step period and calculate length of the
	 * new step period
	 */
	if (data->step_signal_high) {
		data->ramp_data.step_index++;

		gpio_pin_set_dt(data->ramp_data.step_pin, 0);
		if (data->direction == STEPPER_DIRECTION_POSITIVE) {
			data->reference_position++;
		} else {
			data->reference_position--;
		}
	} else {
		/* Use Approximation once error is small enough*/
		if (data->ramp_data.step_index > ACCURATE_STEPS) {
			/*
			 * Iterative algorithm using Taylor expansion, see 'Generate stepper-motor
			 * speed profiles in real time' (2005) by David Austin
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			uint64_t t_n_1 = data->ramp_data.current_time_int;
			uint64_t adjust = 2 * t_n_1 / (4 * data->ramp_data.step_index + 3);
			new_time_int = t_n_1 - adjust;
		} else {
			/* Use accurate (but expensive) calculation when error would be large
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			new_time_int = data->ramp_data.base_time_int *
				       (sqrtf(data->ramp_data.step_index + 1.0f) -
					sqrtf(data->ramp_data.step_index + 0.0f));
		}
		data->counter_top_cfg.ticks =
			data->ramp_data.ticks_us * DIV_ROUND_UP(new_time_int, TICK_DIVISOR);
		data->ramp_data.current_time_int = new_time_int;
		data->current_velocity = PSEC_PER_SEC / (new_time_int);
		gpio_pin_set_dt(data->ramp_data.step_pin, 1);
		counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
	}

	data->step_signal_high = !data->step_signal_high;

	if (data->ramp_data.step_index ==
	    data->ramp_data.accel_steps) { /* Acceleration section is finished */
		if (data->ramp_data.const_steps == 0 && data->ramp_data.decel_steps != 0) {
			/* If no constant speed section switch to deceleration if decelerations
			 * steps exist
			 */
			data->ramp_data.step_index = data->ramp_data.decel_steps;
			data->counter_top_cfg.callback =
				drv8424_accel_positioning_smooth_deceleration;
			counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
		} else if (data->ramp_data.decel_steps == 0) {
			/* If no deceleration steps: movement finished*/
			counter_stop(data->ramp_data.counter);
			if (data->event_callback != NULL) {
				/* Ignore return value since we can't do anything about it anyway */
				drv8424_accel_schedule_user_callback(data,
								     STEPPER_EVENT_STEPS_COMPLETED);
			}
			data->is_moving = false;
			gpio_pin_set_dt(data->ramp_data.step_pin, 0);
			data->step_signal_high = false;
			data->current_velocity = 0;
		} else {
			/* Otherwise switch to constant speed */
			data->ramp_data.step_index = 0;
			data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_constant;
			data->counter_top_cfg.ticks =
				data->ramp_data.ticks_us *
				DIV_ROUND_UP(USEC_PER_SEC, 2 * data->ramp_data.const_velocity);
			data->current_velocity = data->ramp_data.const_velocity;
			counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
		}
	}
}

static void drv8424_accel_positioning_smooth_deceleration_start(const struct device *dev,
								void *user_data)
{
	struct drv8424_accel_data *data = user_data;
	uint64_t new_time_int;

	/* Switch step pin on or off depending position in step period and calculate length of the
	 * new step period
	 */
	if (data->step_signal_high) {
		gpio_pin_set_dt(data->ramp_data.step_pin, 0);
		data->ramp_data.step_index--;
		if (data->direction == STEPPER_DIRECTION_POSITIVE) {
			data->reference_position++;
		} else {
			data->reference_position--;
		}
	} else {
		/* Use Approximation as long as error is small enough */
		if (data->ramp_data.step_index > ACCURATE_STEPS) {
			/*
			 * Iterative algorithm using Taylor expansion, see 'Generate stepper-motor
			 * speed profiles in real time' (2005) by David Austin
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			uint64_t t_n = data->ramp_data.current_time_int;
			uint64_t adjust = 2 * t_n / (4 * data->ramp_data.step_index - 3);
			new_time_int = t_n + adjust;
		} else {
			/* Use accurate (but expensive) calculation when error would be large
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			new_time_int = data->ramp_data.base_time_int *
				       (sqrtf(data->ramp_data.step_index + 0.0f) -
					sqrtf(data->ramp_data.step_index - 1.0f));
		}
		data->counter_top_cfg.ticks =
			data->ramp_data.ticks_us * DIV_ROUND_UP(new_time_int, TICK_DIVISOR);
		data->ramp_data.current_time_int = new_time_int;
		data->current_velocity = PSEC_PER_SEC / (new_time_int);
		gpio_pin_set_dt(data->ramp_data.step_pin, 1);
		counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
	}

	data->step_signal_high = !data->step_signal_high;

	/* Stop Counter if positioning_smooth move is finished */
	if (data->ramp_data.step_index ==
	    data->ramp_data.accel_steps) { /* Deceleration start section is finished */
		if (data->ramp_data.const_steps == 0 && data->ramp_data.decel_steps != 0) {
			/* If no constant speed section switch to deceleration if decelerations
			 * steps exist
			 */
			data->ramp_data.step_index = data->ramp_data.decel_steps;
			data->counter_top_cfg.callback =
				drv8424_accel_positioning_smooth_deceleration;
			counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
		} else if (data->ramp_data.decel_steps == 0) {
			/* If no deceleration steps: movement finished*/
			counter_stop(data->ramp_data.counter);
			if (data->event_callback != NULL) {
				/* Ignore return value since we can't do anything about it anyway */
				drv8424_accel_schedule_user_callback(data,
								     STEPPER_EVENT_STEPS_COMPLETED);
			}
			data->is_moving = false;
			gpio_pin_set_dt(data->ramp_data.step_pin, 0);
			data->step_signal_high = false;
			data->current_velocity = 0;
		} else {
			/* Otherwise switch to constant speed */
			data->ramp_data.step_index = 0;
			data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_constant;
			data->counter_top_cfg.ticks =
				data->ramp_data.ticks_us *
				DIV_ROUND_UP(USEC_PER_SEC, 2 * data->ramp_data.const_velocity);
			data->current_velocity = data->ramp_data.const_velocity;
			counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
		}
	}
}

/*
 * If microstep setter fails, attempt to recover into previous state.
 */
int drv8424_accel_microstep_recovery(const struct device *dev)
{
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;
	int ret;

	uint8_t m0_value = data->pin_states.m0;
	uint8_t m1_value = data->pin_states.m1;

	ret = drv8424_accel_set_microstep_pin(dev, &config->m0_pin, m0_value);
	if (ret != 0) {
		LOG_ERR("%s: Failed to restore microstep configuration (error: %d)", dev->name,
			ret);
		return ret;
	}

	ret = drv8424_accel_set_microstep_pin(dev, &config->m1_pin, m1_value);
	if (ret != 0) {
		LOG_ERR("%s: Failed to restore microstep configuration (error: %d)", dev->name,
			ret);
		return ret;
	}

	return 0;
}

static int drv8424_accel_enable(const struct device *dev, bool enable)
{
	int ret;
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;
	bool has_enable_pin = config->en_pin.port != NULL;
	bool has_sleep_pin = config->sleep_pin.port != NULL;

	/* Check availability of sleep and enable pins, as these might be hardwired. */
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
		gpio_pin_set_dt(&config->step_pin, 0);
		data->step_signal_high = false;
		data->current_velocity = 0;
	}

	return 0;
}

static int drv8424_phase_steps_adjusting(const struct device *dev, uint32_t *accel_steps,
					 uint32_t *const_steps, uint32_t *decel_steps,
					 uint32_t total_steps, uint32_t start_velocity,
					 uint32_t const_velocity, uint32_t index)
{
	/* Catch edge case*/
	if (total_steps == 1) {
		*accel_steps = 1;
		*const_steps = 0;
		*decel_steps = 0;
		return 0;
	}

	/* Decelerate at start */
	if (const_velocity < start_velocity) {
		/* If stop cant be reached from start velocity with straight deceleration, throw
		 * error */
		if (*decel_steps + (index - *accel_steps) > total_steps) {
			LOG_ERR("%s: Total step count is to low, it is %u, but needs to be at "
				"least %u.",
				dev->name, total_steps, *decel_steps + (index - *accel_steps));
			return -EINVAL;
		}
		/* Else simply set last value */
		else {
			*const_steps = total_steps - *decel_steps - (index - *accel_steps);
		}
	}
	/* Accalerate at start*/
	else {
		/* If enough steps available, simply set last value */
		if (*decel_steps + (*accel_steps - index) <= total_steps) {
			*const_steps = total_steps - *decel_steps - (*accel_steps - index);
		} else {
			/* If stop cant be reached from start velocity with straight deceleration,
			 * throw error */
			if (index > total_steps) {
				return -EINVAL;
			} else {
				uint32_t remain_steps = total_steps - index;
				*accel_steps = index + (remain_steps / 2);
				*decel_steps = *accel_steps;
				if (remain_steps % 2 == 1) {
					*const_steps = 1;
				}
			}
		}
	}

	return 0;
}

static int drv8424_accel_calculate_acceleration(const struct device *dev, uint32_t steps,
						uint32_t end_velocity, bool run)
{
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;

	/* Algorithm uses sec for acceleration time, adjusts acceleration for current microstep
	 * resolution
	 */
	float accel_time = end_velocity * 1.0f / config->acceleration; /* s */

	/* Split total steps into steps for the three phases */
	uint32_t accel_steps = ceilf((end_velocity * accel_time) / 2); /* steps */
	uint32_t decel_steps = accel_steps;                            /* steps */
	uint32_t const_steps = 0;                                      /* steps */
	uint32_t step_index = 0;

	float acceleration_adjusted =
		(float)((end_velocity * end_velocity * 1.0) / (2.0 * accel_steps));
	float base_time =
		sqrtf(2.0f / acceleration_adjusted) * USEC_PER_SEC; /* µs, 2.0 contains *1 step */

	step_index =
		(data->current_velocity * data->current_velocity) / (2 * acceleration_adjusted);

	if (!run) {
		int ret = drv8424_phase_steps_adjusting(dev, &accel_steps, &const_steps,
							&decel_steps, steps, data->current_velocity,
							end_velocity, step_index);
		if (ret != 0) {
			return ret;
		}
	} else {
		const_steps = 10; /* Dummy value for correct behaviour */
	}

	/* Configure interrupt data */
	data->ramp_data.accel_steps = accel_steps;
	data->ramp_data.const_steps = const_steps;
	data->ramp_data.decel_steps = decel_steps;
	data->ramp_data.step_index = step_index;
	data->ramp_data.ticks_us = counter_us_to_ticks(config->counter, 1);
	/* Multiplications to get additional accuracy*/
	data->ramp_data.current_time_int = base_time * PSEC_PER_USEC; /* pikoseconds ps */
	data->ramp_data.base_time_int = base_time * PSEC_PER_USEC;    /* in ps */

	if (data->current_velocity < end_velocity) {
		data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_acceleration;
	} else if (data->current_velocity > end_velocity) {
		data->counter_top_cfg.callback =
			drv8424_accel_positioning_smooth_deceleration_start;
	} else {
		data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_constant;
		data->ramp_data.step_index = 0;
	}

	data->ramp_data.current_time_int =
		data->ramp_data.base_time_int * (sqrtf(data->ramp_data.step_index + 1.0f) -
						 sqrtf(data->ramp_data.step_index + 0.0f));

	data->ramp_data.const_velocity = end_velocity;

	return 0;
}

static int drv8424_accel_move_positioning_smooth(const struct device *dev, uint32_t step_count)
{
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;
	int ret;

	if (data->max_velocity == 0) {
		return -EINVAL;
	}

	int dir_value = (data->direction == STEPPER_DIRECTION_POSITIVE) ? 1 : 0;
	ret = gpio_pin_set_dt(&config->dir_pin, dir_value);
	if (ret != 0) {
		LOG_ERR("%s: Failed to set direction pin (error %d)", dev->name, ret);
		return ret;
	}

	int key = irq_lock();
	ret = drv8424_accel_calculate_acceleration(dev, step_count, data->max_velocity, false);
	if (ret != 0) {
		goto end;
	}

	if (data->current_velocity == 0) {
		data->counter_top_cfg.ticks =
			DIV_ROUND_UP(((uint64_t)counter_get_frequency(config->counter) *
				      data->ramp_data.base_time_int),
				     ((uint64_t)USEC_PER_SEC * TICK_DIVISOR));
	}

	ret = counter_set_top_value(config->counter, &data->counter_top_cfg);
	if (ret != 0) {
		LOG_ERR("%s: Failed to set counter top value (error: %d)", dev->name, ret);
		goto end;
	}
	data->is_moving = true;

	ret = counter_start(config->counter);
	if (ret != 0) {
		LOG_ERR("%s: Failed to start counter (error: %d)", dev->name, ret);
		goto end;
	}

end:
	if (ret == 0) {
		data->constant_velocity = false;
	}
	data->constant_velocity = false;
	irq_unlock(key);

	return ret;
}

static int drv8424_accel_move_by(const struct device *dev, int32_t micro_steps)
{
	struct drv8424_accel_data *data = dev->data;
	int ret;

	if (data->max_velocity == 0) {
		LOG_ERR("%s: Invalid max. velocity %d configured", dev->name, data->max_velocity);
		return -EINVAL;
	}

	if (!data->enabled) {
		return -ECANCELED;
	}

	if (micro_steps < 0) {
		if (data->direction == STEPPER_DIRECTION_POSITIVE && data->is_moving) {
			LOG_ERR("%s: Can't change direction while moving (error %d)", dev->name,
				-ENOTSUP);
			return -ENOTSUP;
		}
		data->direction = STEPPER_DIRECTION_NEGATIVE;
	} else {
		if (data->direction == STEPPER_DIRECTION_NEGATIVE && data->is_moving) {
			LOG_ERR("%s: Can't change direction while moving (error %d)", dev->name,
				-ENOTSUP);
			return -ENOTSUP;
		}
		data->direction = STEPPER_DIRECTION_POSITIVE;
	}

	ret = drv8424_accel_move_positioning_smooth(dev, labs(micro_steps));
	if (ret != 0) {
		LOG_ERR("%s: Failed to begin positioning (error %d)", dev->name, ret);
		return ret;
	};

	return 0;
}

static int drv8424_accel_is_moving(const struct device *dev, bool *is_moving)
{
	struct drv8424_accel_data *data = dev->data;

	*is_moving = data->is_moving;

	return 0;
}

static int drv8424_accel_set_reference_position(const struct device *dev, int32_t position)
{
	struct drv8424_accel_data *data = dev->data;

	data->reference_position = position;

	return 0;
}

static int drv8424_accel_get_actual_position(const struct device *dev, int32_t *position)
{
	struct drv8424_accel_data *data = dev->data;

	*position = data->reference_position;

	return 0;
}

static int drv8424_accel_move_to(const struct device *dev, int32_t position)
{
	struct drv8424_accel_data *data = dev->data;
	int ret;
	int64_t steps = position - data->reference_position;

	if (!data->enabled) {
		return -ECANCELED;
	}

	if (steps < 0) {
		if (data->direction == STEPPER_DIRECTION_POSITIVE && data->is_moving) {
			LOG_ERR("%s: Can't change direction while moving (error %d)", dev->name,
				-ENOTSUP);
			return -ENOTSUP;
		}
		data->direction = STEPPER_DIRECTION_NEGATIVE;
	} else {
		if (data->direction == STEPPER_DIRECTION_NEGATIVE && data->is_moving) {
			LOG_ERR("%s: Can't change direction while moving (error %d)", dev->name,
				-ENOTSUP);
			return -ENOTSUP;
		}
		data->direction = STEPPER_DIRECTION_POSITIVE;
	}

	ret = drv8424_accel_move_positioning_smooth(dev, llabs(steps));
	if (ret != 0) {
		LOG_ERR("%s: Failed to begin positioning (error %d)", dev->name, ret);
		return ret;
	};

	return 0;
}

static int drv8424_accel_set_max_velocity(const struct device *dev, uint32_t velocity)
{
	struct drv8424_accel_data *data = dev->data;
	data->max_velocity = velocity;
	return 0;
}

static int drv8424_accel_run(const struct device *dev, const enum stepper_direction direction,
			     const uint32_t velocity)
{
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;
	int ret;

	if (!data->enabled) {
		return -ECANCELED;
	}
	if (data->direction != direction && data->is_moving && velocity != 0) {
		LOG_ERR("%s: Can't change direction while moving (error %d)", dev->name, -ENOTSUP);
		return -ENOTSUP;
	}

	int dir_value = (direction == STEPPER_DIRECTION_POSITIVE) ? 1 : 0;
	ret = gpio_pin_set_dt(&config->dir_pin, dir_value);
	if (ret != 0) {
		LOG_ERR("%s: Failed to set direction pin (error %d)", dev->name, ret);
		return ret;
	}

	/* Lock interrupts while modifying settings used by ISR */
	int key = irq_lock();

	if (velocity == 0) {
		if (data->current_velocity != 0) {
			data->constant_velocity = true;
			data->direction = direction;
			data->is_moving = true;
			uint32_t steps = data->current_velocity * data->current_velocity * 1.0f /
					 config->acceleration;
			ret = drv8424_accel_calculate_acceleration(dev, steps,
								   data->current_velocity, true);
			data->ramp_data.step_index = data->ramp_data.decel_steps;
			data->counter_top_cfg.callback =
				drv8424_accel_positioning_smooth_deceleration;
			data->ramp_data.current_time_int =
				DIV_ROUND_UP(PSEC_PER_SEC, data->current_velocity);
			data->counter_top_cfg.user_data = data;
			ret = counter_set_top_value(config->counter, &data->counter_top_cfg);
			if (ret != 0) {
				LOG_ERR("%s: Failed to set counter top value (error: %d)",
					dev->name, ret);
				goto end;
			}

			ret = counter_start(config->counter);
			if (ret != 0) {
				LOG_ERR("%s: Failed to start counter (error %d)", dev->name, ret);
				data->is_moving = false;
				goto end;
			}
		}

	} else {
		data->constant_velocity = true;
		data->direction = direction;
		data->is_moving = true;
		ret = drv8424_accel_calculate_acceleration(dev, 0, velocity, true);
		if (data->current_velocity == 0) {
			data->counter_top_cfg.ticks =
				counter_us_to_ticks(config->counter, USEC_PER_MSEC);
		}
		if (data->current_velocity > data->ramp_data.const_velocity) {
			data->counter_top_cfg.callback =
				drv8424_accel_positioning_smooth_deceleration_start;
		}
		if (data->current_velocity == data->ramp_data.const_velocity) {
			data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_constant;
			data->counter_top_cfg.ticks = counter_us_to_ticks(
				config->counter,
				(uint32_t)DIV_ROUND_UP(USEC_PER_SEC, (2 * data->current_velocity)));
		}
		data->counter_top_cfg.user_data = data;

		ret = counter_set_top_value(config->counter, &data->counter_top_cfg);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set counter top value (error: %d)", dev->name, ret);
			goto end;
		}

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

static int drv8424_accel_set_micro_step_res(const struct device *dev,
					    enum stepper_micro_step_resolution micro_step_res)
{
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;
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
		return -EINVAL;
	};

	ret = drv8424_accel_set_microstep_pin(dev, &config->m0_pin, m0_value);
	if (ret != 0) {
		return ret;
	}

	ret = drv8424_accel_set_microstep_pin(dev, &config->m1_pin, m1_value);
	if (ret != 0) {
		return ret;
	}

	data->ustep_res = micro_step_res;
	data->pin_states.m0 = m0_value;
	data->pin_states.m1 = m1_value;

	return 0;
}

static int drv8424_accel_get_micro_step_res(const struct device *dev,
					    enum stepper_micro_step_resolution *micro_step_res)
{
	struct drv8424_accel_data *data = dev->data;
	*micro_step_res = data->ustep_res;
	return 0;
}

static int drv8424_accel_set_event_callback(const struct device *dev,
					    stepper_event_callback_t callback, void *user_data)
{
	struct drv8424_accel_data *data = dev->data;

	data->event_callback = callback;
	data->event_callback_user_data = user_data;

	return 0;
}

static int drv8424_accel_init(const struct device *dev)
{
	const struct drv8424_accel_config *const config = dev->config;
	struct drv8424_accel_data *const data = dev->data;
	int ret;

	data->dev = dev;

	ret = gpio_pin_configure_dt(&config->dir_pin, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to configure dir_pin (error: %d)", dev->name, ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&config->step_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to configure step_pin (error: %d)", dev->name, ret);
		return ret;
	}
	data->ramp_data.step_pin = &config->step_pin;

	if (config->sleep_pin.port != NULL) {
		ret = gpio_pin_configure_dt(&config->sleep_pin, GPIO_OUTPUT_ACTIVE);
		if (ret != 0) {
			LOG_ERR("%s: Failed to configure sleep_pin (error: %d)", dev->name, ret);
			return ret;
		}
		data->pin_states.sleep = 1U;
	}

	if (config->en_pin.port != NULL) {
		ret = gpio_pin_configure_dt(&config->en_pin, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("%s: Failed to configure en_pin (error: %d)", dev->name, ret);
			return ret;
		}
		data->pin_states.en = 0U;
	}

	ret = gpio_pin_configure_dt(&config->m0_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to configure m0_pin (error: %d)", dev->name, ret);
		return ret;
	}
	data->pin_states.m0 = 0U;

	ret = gpio_pin_configure_dt(&config->m1_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to configure m1_pin (error: %d)", dev->name, ret);
		return ret;
	}
	data->pin_states.m1 = 0U;

	data->step_signal_high = false;
	data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_acceleration;
	data->counter_top_cfg.user_data = data;
	data->counter_top_cfg.flags = 0;
	data->counter_top_cfg.ticks = counter_us_to_ticks(config->counter, 1000000);

	data->ramp_data.counter = config->counter;
	data->current_velocity = 0;

	return 0;
}

static DEVICE_API(stepper, drv8424_accel_stepper_api) = {
	.enable = drv8424_accel_enable,
	.move_by = drv8424_accel_move_by,
	.move_to = drv8424_accel_move_to,
	.is_moving = drv8424_accel_is_moving,
	.set_reference_position = drv8424_accel_set_reference_position,
	.get_actual_position = drv8424_accel_get_actual_position,
	.set_max_velocity = drv8424_accel_set_max_velocity,
	.run = drv8424_accel_run,
	.set_micro_step_res = drv8424_accel_set_micro_step_res,
	.get_micro_step_res = drv8424_accel_get_micro_step_res,
	.set_event_callback = drv8424_accel_set_event_callback,
};

#define DRV8424_ACCEL_DEVICE(inst)                                                                 \
                                                                                                   \
	static const struct drv8424_accel_config drv8424_accel_config_##inst = {                   \
		.dir_pin = GPIO_DT_SPEC_INST_GET(inst, dir_gpios),                                 \
		.step_pin = GPIO_DT_SPEC_INST_GET(inst, step_gpios),                               \
		.sleep_pin = GPIO_DT_SPEC_INST_GET_OR(inst, sleep_gpios, {0}),                     \
		.en_pin = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                           \
		.m0_pin = GPIO_DT_SPEC_INST_GET(inst, m0_gpios),                                   \
		.m1_pin = GPIO_DT_SPEC_INST_GET(inst, m1_gpios),                                   \
		.counter = DEVICE_DT_GET(DT_INST_PHANDLE(inst, counter)),                          \
		.acceleration = DT_INST_PROP(inst, acceleration),                                  \
	};                                                                                         \
                                                                                                   \
	static struct drv8424_accel_data drv8424_accel_data_##inst = {                             \
		.ustep_res = DT_INST_PROP(inst, micro_step_res),                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &drv8424_accel_init, NULL, &drv8424_accel_data_##inst,         \
			      &drv8424_accel_config_##inst, POST_KERNEL,                           \
			      CONFIG_STEPPER_INIT_PRIORITY, &drv8424_accel_stepper_api);

DT_INST_FOREACH_STATUS_OKAY(DRV8424_ACCEL_DEVICE)
