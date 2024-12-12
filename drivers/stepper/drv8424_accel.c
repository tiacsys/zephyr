/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Navimatix GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util.h"
#include "zephyr/sys_clock.h"
#include <math.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv8424_accel, CONFIG_STEPPER_LOG_LEVEL);

#define DT_DRV_COMPAT ti_drv8424_accel

/* Number of steps for which the accurate ramp calculation algorithm is used, once the error is low
 * enough, an approximation algorithm is used
 */
#define ACCURATE_STEPS 15

/**
 * @brief DRV8424 stepper driver configuration data.
 *
 * This structure contains all of the devicetree specifications for the pins
 * needed by a given DRV8424 stepper driver.
 */
struct drv8424_accel_config {
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
	/** Acceleration in steps/s^2. */
	uint32_t acceleration;
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
	/* Current step period time in us. */
	uint64_t current_time_int;
	/* Step period of the first step of acceleration. */
	uint64_t base_time_int;
	/* Number of steps to take during constant speed phase. */
	uint32_t const_steps;
	/* Number of steps to take during deceleration phase. */
	uint32_t decel_steps;
	/* Number of steps to take during acceleration phase. */
	uint32_t accel_steps;
	/* Pointer to step_pin dt_spec. */
	const struct gpio_dt_spec *step_pin;
	/* Counter ticks per us, saved here as value is constant and function call expensive*/
	uint32_t ticks_us;
	/** Velocity during const velocity phase (steps/s). */
	uint32_t const_velocity;
	/* Counter reference, because *dev in counter interrupt doesn't allways references the
	 * counter
	 */
	const struct device *counter;
	/* Position at the time the move command was given, used to account for position
	 * inaccuracies because of delays
	 */
	uint32_t start_position;
	/* If it is the first step of a move command. Used for the same calculation as
	 * start_position
	 */
	bool is_first_step;
};

/**
 * @brief DRV8424 accel stepper driver data.
 *
 * This structure contains mutable data used by a DRV8424 stepper driver.
 */
struct drv8424_accel_data {
	/** Back pointer to the device, e.g. for access to config */
	const struct device *dev;
	/** Whether the device is enabled */
	bool enabled;
	/** Struct containing the states of different pins. */
	struct drv8424_accel_pin_states pin_states;
	/** Current microstep resolution. */
	enum stepper_micro_step_resolution ms_res;
	/** Maximum velocity (steps/s). */
	uint32_t max_velocity;
	/** Current velocity (steps/s). */
	uint32_t current_velocity;
	/** Current reference position. */
	int32_t reference_position;
	/** Target position. */;
	// int32_t target_position;
	/** Whether the motor is currently moving. */
	bool is_moving;
	/** Whether we're in constant velocity mode. */
	bool constant_velocity;
	/** Which direction we're going. */
	enum stepper_direction direction;
	/** Event handler registered by user. */
	stepper_event_callback_t event_callback;
	/** Event handler user data. */
	void *event_callback_user_data;
	/** Whether the step signal is currently high or low. */
	bool step_signal_high;
	/** Struct containing counter top configuration. */
	struct counter_top_cfg counter_top_cfg;
	/** Struct containing ramp information. */
	struct drv8424_accel_ramp_data ramp_data;
};

struct drv8424_accel_callback_work {
	/** The work item itself. */
	struct k_work work;
	/** Type of event to handle. */
	enum stepper_event event;
	/** Driver instance data. */
	struct drv8424_accel_data *driver_data;
};

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

static void drv8424_accel_positioning_top_interrupt(const struct device *dev, void *user_data)
{
	struct drv8424_accel_data *data = (struct drv8424_accel_data *)user_data;
	const struct drv8424_accel_config *config =
		(struct drv8424_accel_config *)data->dev->config;

	/* Switch step pin on or off depending position in step period */
	if (data->step_signal_high) {
		/* Generate a falling edge and count a completed step */
		gpio_pin_set_dt(&config->step_pin, 0);
		if (data->direction == STEPPER_DIRECTION_POSITIVE) {
			data->reference_position++;
		} else {
			data->reference_position--;
		}
	} else {
		/* Generate a rising edge */
		gpio_pin_set_dt(&config->step_pin, 1);
	}

	data->step_signal_high = !data->step_signal_high;
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
		// LOG_INF("Index: %u", data->ramp_data.step_index);

	} else {
		/* data->n is 1 larger than actual value to prevent overflow */
		uint32_t n_adjusted = data->ramp_data.step_index - 1;
		/* Use Approximation as long as error is small enough */
		if (data->ramp_data.step_index > ACCURATE_STEPS) {
			/*
			 * Iterative algorithm using Taylor expansion, see 'Generate stepper-motor
			 * speed profiles in real time' (2005) by David Austin
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			uint64_t t_n = data->ramp_data.current_time_int;
			uint64_t adjust = 2 * t_n / (4 * n_adjusted + 1);
			new_time_int = t_n + adjust;
		} else {
			// LOG_INF("Yeah");
			/* Use accurate (but expensive) calculation when error would be large
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			new_time_int = data->ramp_data.base_time_int *
				       (sqrtf(n_adjusted + 1.5f) - sqrtf(n_adjusted + 0.5f));
			// LOG_INF("Time: %llu", data->ramp_data.base_time_int);
		}
		data->counter_top_cfg.ticks =
			data->ramp_data.ticks_us * (uint32_t)(new_time_int / 1000000) / 2;
		data->ramp_data.current_time_int = new_time_int;
		data->current_velocity = 1000000000000 / (new_time_int);
		// LOG_INF("Current Velocity: %u", data->current_velocity);
		gpio_pin_set_dt(data->ramp_data.step_pin, 1);
		counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
		// LOG_INF("Current Time Decel: %llu", data->ramp_data.current_time_int / 1000000);
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
	// uint32_t start;
	// counter_get_value(data->ramp_data.counter, &start);
	/* Adjust for additional steps that might have been taken between calculating the total
	 * number of steps to take and entering the interrupt of the first step to take
	 */
	// if (data->ramp_data.is_first_step) {
	// 	data->ramp_data.is_first_step = false;
	// 	int diff = data->reference_position - data->ramp_data.start_position;
	// 	if (diff != 0) {
	// 		diff = abs(diff);
	// 		LOG_INF("T1");
	// 		/* If difference can be subtracted from const velocity phase, do so*/
	// 		if (data->ramp_data.const_steps >= diff) {
	// 			LOG_INF("T2");
	// 			data->ramp_data.const_steps -= diff;
	// 			diff = 0;
	// 		} else {
	// 			LOG_INF("T3");
	// 			diff -= data->ramp_data.const_steps;
	// 			data->ramp_data.const_steps = 0;
	// 			/* Special case for 1 total move step*/
	// 			if (data->ramp_data.accel_steps == 1 &&
	// 			    data->ramp_data.decel_steps == 0) {
	// 				LOG_INF("T4");
	// 				if (data->event_callback != NULL) {
	// 					/* Ignore return value since we can't do anything
	// 					 * about it anyway */
	// 					drv8424_accel_schedule_user_callback(
	// 						data, STEPPER_EVENT_STEPS_COMPLETED);
	// 				}
	// 				counter_stop(data->ramp_data.counter);
	// 				data->is_moving = false;
	// 				gpio_pin_set_dt(data->ramp_data.step_pin, 0);
	// 				data->step_signal_high = false;
	// 				data->current_velocity = 0;
	// 				return;
	// 			}
	// 			if (diff % 2 == 1) {
	// 				LOG_INF("T5");
	// 				diff--;
	// 				data->ramp_data.decel_steps--;
	// 			}
	// 			data->ramp_data.decel_steps -= diff / 2;
	// 			data->ramp_data.accel_steps -= diff / 2;
	// 			if (data->ramp_data.step_index == data->ramp_data.accel_steps) {
	// 				LOG_INF("T6");
	// 				goto end;
	// 			} else if (data->ramp_data.step_index >
	// 				   data->ramp_data.accel_steps) {
	// 				LOG_INF("T7");
	// 				if (data->event_callback != NULL) {
	// 					/* Ignore return value since we can't do anything
	// 					 * about it anyway */
	// 					drv8424_accel_schedule_user_callback(
	// 						data, STEPPER_EVENT_STEPS_COMPLETED);
	// 				}
	// 				counter_stop(data->ramp_data.counter);
	// 				data->is_moving = false;
	// 				gpio_pin_set_dt(data->ramp_data.step_pin, 0);
	// 				data->step_signal_high = false;
	// 				data->current_velocity = 0;
	// 				return;
	// 			}
	// 		}
	// 	}
	// }

	uint64_t new_time_int;
	// LOG_INF("Yeah %u %u", data->ramp_data.step_index, data->ramp_data.accel_steps);

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
				       (sqrtf(data->ramp_data.step_index + 1.5f) -
					sqrtf(data->ramp_data.step_index + 0.5f));
		}
		data->counter_top_cfg.ticks =
			data->ramp_data.ticks_us * (uint32_t)(new_time_int / 1000000) / 2;
		data->ramp_data.current_time_int = new_time_int;
		// uint32_t delay;
		// counter_get_value(dev, &delay);
		// LOG_INF("Delay in us: %llu, in ticks: %u", counter_ticks_to_us(dev, delay -
		// start), 	delay - start);
		// LOG_INF("Current Time: %llu", data->ramp_data.current_time_int / 1000000);
		data->current_velocity = 1000000000000 / (new_time_int);
		// LOG_INF("Current Velocity: %u", data->current_velocity);
		gpio_pin_set_dt(data->ramp_data.step_pin, 1);
		counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
	}

	data->step_signal_high = !data->step_signal_high;

	if (data->ramp_data.step_index ==
	    data->ramp_data.accel_steps) { /* Acceleration section is finished */
// end:
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
			data->counter_top_cfg.ticks = data->ramp_data.ticks_us *
						      (1000000 / data->ramp_data.const_velocity) /
						      2;
			data->current_velocity = data->ramp_data.const_velocity;
			// LOG_INF("Current Velocity: %u", data->current_velocity);
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
		// LOG_INF("Index: %u", data->ramp_data.step_index);

	} else {
		/* data->n is 1 larger than actual value to prevent overflow */
		uint32_t n_adjusted = data->ramp_data.step_index - 1;
		/* Use Approximation as long as error is small enough */
		if (data->ramp_data.step_index > ACCURATE_STEPS) {
			/*
			 * Iterative algorithm using Taylor expansion, see 'Generate stepper-motor
			 * speed profiles in real time' (2005) by David Austin
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			uint64_t t_n = data->ramp_data.current_time_int;
			uint64_t adjust = 2 * t_n / (4 * n_adjusted + 1);
			new_time_int = t_n + adjust;
		} else {
			// LOG_INF("Yeah");
			/* Use accurate (but expensive) calculation when error would be large
			 * Added 0.5 to n equivalent for better acceleration behaviour
			 */
			new_time_int = data->ramp_data.base_time_int *
				       (sqrtf(n_adjusted + 1.5f) - sqrtf(n_adjusted + 0.5f));
			// LOG_INF("Time: %llu", data->ramp_data.base_time_int);
		}
		data->counter_top_cfg.ticks =
			data->ramp_data.ticks_us * (uint32_t)(new_time_int / 1000000) / 2;
		data->ramp_data.current_time_int = new_time_int;
		data->current_velocity = 1000000000000 / (new_time_int);
		// LOG_INF("Current Velocity: %u", data->current_velocity);
		gpio_pin_set_dt(data->ramp_data.step_pin, 1);
		counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
		// LOG_INF("Current Time Decel: %llu", data->ramp_data.current_time_int / 1000000);
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
			data->counter_top_cfg.ticks = data->ramp_data.ticks_us *
						      (1000000 / data->ramp_data.const_velocity) /
						      2;
			data->current_velocity = data->ramp_data.const_velocity;
			// LOG_INF("Current Velocity: %u", data->current_velocity);
			counter_set_top_value(data->ramp_data.counter, &data->counter_top_cfg);
		}
	}
}

/*
 * If microstep setter fails, attempt to recover into previous state.
 */
static int drv8424_accel_microstep_recovery(const struct device *dev)
{
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;
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
		 * this.
		 */
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
		 * this.
		 */
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
	LOG_INF("Started Adjusting");
	/* Catch edge case*/
	if (total_steps == 1) {
		*accel_steps = 1;
		*const_steps = 0;
		*decel_steps = 0;
		return 0;
	}
	LOG_INF("Start V: %u, Const V: %u", start_velocity, const_velocity);

	/* Decelerate at start */
	if (const_velocity < start_velocity) {
		LOG_INF("Deceleration-Start");
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
		LOG_INF("Test1");
		/* If enough steps available, simply set last value */
		if (*decel_steps + (*accel_steps - index) <= total_steps) {
			LOG_INF("Test2");
			LOG_INF("A: %u, C: %u, D: %u, Total: %u, Index: %u", *accel_steps,
				*const_steps, *decel_steps, total_steps, index);
			*const_steps = total_steps - *decel_steps - (*accel_steps - index);
		} else {
			/* If stop cant be reached from start velocity with straight deceleration,
			 * throw error */
			if (index > total_steps) {
				LOG_INF("Test3");
				LOG_ERR("%s: Total step count is to low, it is %u, but needs to be "
					"at least %u.",
					dev->name, total_steps, index);
				return -EINVAL;
			} else {
				LOG_INF("Test4");
				uint32_t remain_steps = total_steps - index;
				*accel_steps = index + (remain_steps / 2);
				*decel_steps = *accel_steps;
				if (remain_steps % 2 == 1) {
					LOG_INF("Test5");
					*const_steps = 1;
				}
				LOG_INF("A: %u, C: %u, D: %u, Total: %u, Index: %u", *accel_steps,
					*const_steps, *decel_steps, total_steps, index);
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

	// uint32_t start;
	// counter_get_value(data->ramp_data.counter, &start);

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
	float base_time = sqrtf(2.0f / acceleration_adjusted) *
			  1000000U; /* µs (via 1000000), 2.0 contains *1 step */

	step_index =
		(data->current_velocity * data->current_velocity) / (2 * acceleration_adjusted);
	LOG_INF("Index: %u", step_index);

	if (!run) {
		int ret = drv8424_phase_steps_adjusting(dev, &accel_steps, &const_steps,
							&decel_steps, steps, data->current_velocity,
							end_velocity, step_index);
		// LOG_INF("Index: %u Accel: %u", step_index, accel_steps);
		if (ret != 0) {
			return ret;
		}
	} else {
		const_steps = 10; /* Dummy value for correct behaviour */
	}
	LOG_INF("Accel: %u, Const: %u, Decel: %u", accel_steps, const_steps, decel_steps);

	uint32_t start;
	counter_get_value(data->ramp_data.counter, &start);

	/* Configure interrupt data */
	data->ramp_data.accel_steps = accel_steps;
	data->ramp_data.const_steps = const_steps;
	data->ramp_data.decel_steps = decel_steps;
	data->ramp_data.step_index = step_index;
	data->ramp_data.ticks_us = counter_us_to_ticks(config->counter, 1);
	/* Multiplications to get additional accuracy*/
	data->ramp_data.current_time_int = base_time * 1000000;
	data->ramp_data.base_time_int = base_time * 1000000;

	if (data->current_velocity <= end_velocity) {
		data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_acceleration;
	} else {
		// LOG_INF("Yeah");
		data->counter_top_cfg.callback =
			drv8424_accel_positioning_smooth_deceleration_start;
	}

	data->ramp_data.current_time_int =
		data->ramp_data.base_time_int * (sqrtf(data->ramp_data.step_index + 1.5f) -
						 sqrtf(data->ramp_data.step_index + 0.5f));

	data->ramp_data.const_velocity = end_velocity;

	uint32_t delay;
	counter_get_value(data->ramp_data.counter, &delay);
	LOG_INF("Delay in us: %llu, in ticks: %u", counter_ticks_to_us(data->ramp_data.counter, delay -
	start), 	delay - start);

	return 0;
}

static int drv8424_accel_move_positioning_smooth(const struct device *dev, uint32_t step_count)
{
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;
	int ret = 0;

	

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

	// data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_acceleration;
	data->counter_top_cfg.ticks = counter_us_to_ticks(
		config->counter, (uint32_t)data->ramp_data.base_time_int / 2000000);
	data->counter_top_cfg.user_data = data;
	data->counter_top_cfg.flags = 0;

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
	int ret = 0;

	if (data->max_velocity == 0) {
		LOG_ERR("%s: Invalid max. velocity %d configured", dev->name, data->max_velocity);
		return -EINVAL;
	}

	if (!data->enabled) {
		return -ENODEV;
	}

	if (micro_steps < 0) {
		data->direction = STEPPER_DIRECTION_NEGATIVE;
	} else {
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
	// const struct drv8424_accel_config *config = dev->config;
	int ret = 0;

	if (!data->enabled) {
		return -ENODEV;
	}

	// uint32_t start;
	// counter_get_value(config->counter, &start);
	LOG_INF("Ref Pos Start: %u", data->reference_position);
	// data->target_position = position;
	int64_t steps = position - data->reference_position;
	data->ramp_data.is_first_step = true;
	data->ramp_data.start_position = data->reference_position;
	// data->reference_position++;
	// data->reference_position++;
	// data->reference_position++;

	if (steps < 0) {
		data->direction = STEPPER_DIRECTION_NEGATIVE;
	} else {
		data->direction = STEPPER_DIRECTION_POSITIVE;
	}

	ret = drv8424_accel_move_positioning_smooth(dev, llabs(steps));
	if (ret != 0) {
		LOG_ERR("%s: Failed to begin positioning (error %d)", dev->name, ret);
		return ret;
	};
	LOG_INF("Ref End Start: %u", data->reference_position);
	// uint32_t delay;
	// counter_get_value(config->counter, &delay);
	// LOG_INF("Delay in us: %llu, in ticks: %u",
	// 	counter_ticks_to_us(config->counter, delay - start), delay - start);

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

	ret = drv8424_accel_calculate_acceleration(dev, 0, velocity, true);

	/* Set data used in counter interrupt */
	data->constant_velocity = true;
	data->direction = direction;
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
		gpio_pin_set_dt(&config->step_pin, 0);
		data->step_signal_high = false;
		data->current_velocity = 0;
	} else {
		// data->counter_top_cfg.callback = drv8424_accel_positioning_smooth_acceleration;
		data->counter_top_cfg.user_data = data;
		data->counter_top_cfg.ticks = counter_us_to_ticks(
			config->counter, (uint32_t)data->ramp_data.base_time_int / 2000000);

		ret = counter_set_top_value(config->counter, &data->counter_top_cfg);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set counter top value (error: %d)", dev->name, ret);
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

static int drv8424_accel_set_micro_step_res(const struct device *dev,
					    enum stepper_micro_step_resolution micro_step_res)
{
	const struct drv8424_accel_config *config = dev->config;
	struct drv8424_accel_data *data = dev->data;
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
		drv8424_accel_microstep_recovery(dev);
		return ret;
	}

	/* Reset m1 pin as it may have been disconnected. */
	ret = gpio_pin_configure_dt(&config->m1_pin, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("%s: Failed to reset m1_pin (error: %d)", dev->name, ret);
		drv8424_accel_microstep_recovery(dev);
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
		 * all gpio controllers support this.
		 */
		ret = gpio_pin_configure_dt(&config->m0_pin, GPIO_DISCONNECTED);
		break;
	default:
		break;
	}

	if (ret != 0) {
		LOG_ERR("%s: Failed to set m0_pin (error: %d)", dev->name, ret);
		drv8424_accel_microstep_recovery(dev);
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
		 * all gpio controllers support this.
		 */
		ret = gpio_pin_configure_dt(&config->m1_pin, GPIO_DISCONNECTED);
		break;
	default:
		break;
	}

	if (ret != 0) {
		LOG_ERR("%s: Failed to set m1_pin (error: %d)", dev->name, ret);
		drv8424_accel_microstep_recovery(dev);
		return ret;
	}

	data->ms_res = micro_step_res;
	data->pin_states.m0 = m0_value;
	data->pin_states.m1 = m1_value;

	return 0;
}

static int drv8424_accel_get_micro_step_res(const struct device *dev,
					    enum stepper_micro_step_resolution *micro_step_res)
{
	struct drv8424_accel_data *data = dev->data;
	*micro_step_res = data->ms_res;
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
	data->ramp_data.step_pin = &config->step_pin;

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
	data->counter_top_cfg.callback = drv8424_accel_positioning_top_interrupt;
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
		.m0_pin = GPIO_DT_SPEC_INST_GET_OR(inst, m0_gpios, {0}),                           \
		.m1_pin = GPIO_DT_SPEC_INST_GET_OR(inst, m1_gpios, {0}),                           \
		.counter = DEVICE_DT_GET(DT_INST_PHANDLE(inst, counter)),                          \
		.acceleration = DT_INST_PROP(inst, acceleration),                                  \
	};                                                                                         \
                                                                                                   \
	static struct drv8424_accel_data drv8424_accel_data_##inst = {                             \
		.ms_res = STEPPER_MICRO_STEP_1,                                                    \
		.reference_position = 0, /*.target_position = 0,*/                                 \
		.is_moving = false,                                                                \
		.event_callback = NULL,                                                            \
		.event_callback_user_data = NULL,                                                  \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &drv8424_accel_init,    /* Init */                             \
			      NULL,                         /* PM */                               \
			      &drv8424_accel_data_##inst,   /* Data */                             \
			      &drv8424_accel_config_##inst, /* Config */                           \
			      POST_KERNEL,                  /* Init stage */                       \
			      CONFIG_STEPPER_INIT_PRIORITY, /* Init priority */                    \
			      &drv8424_accel_stepper_api);  /* API */

DT_INST_FOREACH_STATUS_OKAY(DRV8424_ACCEL_DEVICE)
