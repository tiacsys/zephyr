/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "step_dir_stepper_stm_timer.h"
#include "zephyr/drivers/pinctrl.h"
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(step_dir_stepper_stm_timer, CONFIG_STEPPER_LOG_LEVEL);

static void stepper_trigger_callback_stm_timer(const struct device *dev, enum stepper_event event)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;
	LOG_INF("HIHIHI");
	LOG_INF("Saved-Callback: %u++++++++++++++++++++", (uint32_t)data->callback);
	LOG_INF("Device Name: %s", dev->name);

	if (!data->callback) {
		LOG_INF("No Callback");
		LOG_WRN_ONCE("No callback set");
		return;
	}

	// if (!k_is_in_isr()) {
		LOG_INF("Executing Callback");
		data->callback(dev, event, data->event_cb_user_data);
		return;
	// }

#ifdef CONFIG_STEPPER_STEP_DIR_GENERATE_ISR_SAFE_EVENTS
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
	LOG_WRN_ONCE("Event callback called from ISR context without ISR safe events enabled");
#endif /* CONFIG_STEPPER_STEP_DIR_GENERATE_ISR_SAFE_EVENTS */
}

#ifdef CONFIG_STEPPER_STEP_DIR_GENERATE_ISR_SAFE_EVENTS
static void stepper_work_event_handler_stm_timer(struct k_work *work)
{
	struct step_dir_stepper_stm_timer_data *data =
		CONTAINER_OF(work, struct step_dir_stepper_stm_timer_data, event_callback_work);
	enum stepper_event event;
	int ret;

	ret = k_msgq_get(&data->event_msgq, &event, K_NO_WAIT);
	if (ret != 0) {
		return;
	}
	LOG_INF("Running Callback");

	/* Run the callback */
	if (data->callback != NULL) {
		data->callback(data->dev, event, data->event_cb_user_data);
	}

	/* If there are more pending events, resubmit this work item to handle them */
	if (k_msgq_num_used_get(&data->event_msgq) > 0) {
		k_work_submit(work);
	}
}
#endif /* CONFIG_STEPPER_STEP_DIR_GENERATE_ISR_SAFE_EVENTS */

static void step_dir_stepper_stm_timer_count_reached(const struct device *dev, void *user_data)
{
	const struct device *stepper = user_data;
	const struct step_dir_stepper_stm_timer_config *config = stepper->config;
	struct step_dir_stepper_stm_timer_data *data = stepper->data;

	LOG_INF("Finished");

	if (data->run_mode == STEPPER_RUN_MODE_POSITION) {
		LOG_INF("Ding Dong Bong");
		data->counter_running = false;
		counter_stop(config->step_generator);
		LOG_INF("Calling Callback handler");
		stepper_trigger_callback_stm_timer(stepper, STEPPER_EVENT_STEPS_COMPLETED);
		
	}

	if (data->direction == STEPPER_DIRECTION_POSITIVE) {
		LOG_INF("Added Reached %u", data->cfg_count.ticks);
		data->actual_position += data->cfg_count.ticks;
	} else {
		LOG_INF("Subtracted Reached %u", data->cfg_count.ticks);
		data->actual_position -= data->cfg_count.ticks;
	}
	LL_TIM_SetCounter(config->tim_count, 0);
}

// static void update_direction_from_step_count_stm_timer(const struct device *dev)
// {
// 	struct step_dir_stepper_stm_timer_data *data = dev->data;

// 	if (data->step_count > 0) {
// 		data->direction = STEPPER_DIRECTION_POSITIVE;
// 	} else if (data->step_count < 0) {
// 		data->direction = STEPPER_DIRECTION_NEGATIVE;
// 	} else {
// 		LOG_ERR("Step count is zero");
// 	}
// }

int step_dir_stepper_stm_timer_init(const struct device *dev)
{
	const struct step_dir_stepper_stm_timer_config *config = dev->config;
	struct step_dir_stepper_stm_timer_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&config->dir_pin)) {
		LOG_ERR("dir pin is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->dir_pin, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure dir pin: %d", ret);
		return ret;
	}

	// ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	// if (ret < 0) {
	// 	LOG_ERR("Step-Dir pinctrl setup failed (%d)", ret);
	// 	return ret;
	// }

	LL_TIM_EnableAllOutputs(config->tim_gen);

	LL_TIM_SetTriggerOutput(config->tim_gen, LL_TIM_TRGO_UPDATE); // CR2-MMS - Update

	LL_TIM_EnableMasterSlaveMode(config->tim_count);                        // SMCR-MSM 1
	LL_TIM_SetTriggerInput(config->tim_count, config->trigger_input);       // SMCR-TS 000
	LL_TIM_SetClockSource(config->tim_count, LL_TIM_CLOCKSOURCE_EXT_MODE1); // SMCR-SMS 0111

	data->cfg_count.flags = 0;
	data->cfg_count.ticks = 100;
	data->cfg_count.callback = step_dir_stepper_stm_timer_count_reached;
	data->cfg_count.user_data = (void *)dev;

	counter_set_top_value(config->step_counter, &data->cfg_count);
	counter_start(config->step_counter);

#ifdef CONFIG_STEPPER_STEP_DIR_GENERATE_ISR_SAFE_EVENTS
	k_msgq_init(&data->event_msgq, data->event_msgq_buffer, sizeof(enum stepper_event),
		    CONFIG_STEPPER_STEP_DIR_EVENT_QUEUE_LEN);
	k_work_init(&data->event_callback_work, stepper_work_event_handler_stm_timer);
#endif /* CONFIG_STEPPER_STEP_DIR_GENERATE_ISR_SAFE_EVENTS */

	return 0;
}

int step_dir_stepper_stm_timer_move_by(const struct device *dev, const int32_t micro_steps)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;
	const struct step_dir_stepper_stm_timer_config *config = dev->config;
	// TODO: Error Handling

	if (data->microstep_interval_ns == 0) {
		LOG_ERR("Step interval not set or invalid step interval set");
		return -EINVAL;
	}

	/* Stop step signal, no error handling, as counter driver always returns 0 */
	counter_stop(config->step_generator);

	/* Update position */
	uint32_t pos_delta;
	counter_get_value(config->step_counter, &pos_delta);
	if (data->direction == STEPPER_DIRECTION_POSITIVE) {
		LOG_INF("Added Move By %u", pos_delta);
		data->actual_position += pos_delta;
	} else {
		LOG_INF("Subtracted Move By %u", pos_delta);
		data->actual_position -= pos_delta;
	}
	LL_TIM_SetCounter(config->tim_count, 0);

	/* If no steps need to be taken, we are finished */
	if (micro_steps == 0) {
		data->counter_running = false;
		stepper_trigger_callback_stm_timer(dev, STEPPER_EVENT_STEPS_COMPLETED);
		//TODO: Maybe update position?
		return 0;
	}

	/* Update Direction */
	if (micro_steps > 0) {
		gpio_pin_set_dt(&config->dir_pin, 1);
		data->direction = STEPPER_DIRECTION_POSITIVE;
	} else {
		gpio_pin_set_dt(&config->dir_pin, 0);
		data->direction = STEPPER_DIRECTION_NEGATIVE;
	}

	/* Update Step Count, note that counter value gets reset automatically. */
	data->cfg_count.ticks = abs(micro_steps);
	counter_set_top_value(config->step_counter, &data->cfg_count);

	data->run_mode = STEPPER_RUN_MODE_POSITION;

	/* Start step signal*/
	counter_start(config->step_generator);
	data->counter_running = true;

	return 0;
}

int step_dir_stepper_stm_timer_set_microstep_interval(const struct device *dev,
						      const uint64_t microstep_interval_ns)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;
	const struct step_dir_stepper_stm_timer_config *config = dev->config;

	if (microstep_interval_ns == 0) {
		LOG_ERR("Step interval cannot be zero");
		return -EINVAL;
	}

	data->microstep_interval_ns = microstep_interval_ns;

	// uint64_t freq = counter_get_frequency(config->step_generator);

	// data->cfg_gen.ticks = (microstep_interval_ns * freq) / NSEC_PER_SEC;
	data->cfg_gen.ticks = counter_us_to_ticks(config->step_generator, microstep_interval_ns/NSEC_PER_USEC);
	// data->cfg_gen.ticks = counter_us_to_ticks(config->step_generator, 20000);
	counter_set_top_value(config->step_generator, &data->cfg_gen);

	return 0;
}

int step_dir_stepper_stm_timer_set_reference_position(const struct device *dev, const int32_t value)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;

	data->actual_position = value;

	return 0;
}

int step_dir_stepper_stm_timer_get_actual_position(const struct device *dev, int32_t *value)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;
	const struct step_dir_stepper_stm_timer_config *config = dev->config;

	*value = data->actual_position;
	if (data->counter_running) {
		LOG_INF("Ping Pos");
		uint32_t pos_delta;
		counter_get_value(config->step_counter, &pos_delta);
		LOG_INF("Pos Delta: %u", pos_delta);
		LOG_INF("Actual Pos: %i", data->actual_position);
		if (data->direction == STEPPER_DIRECTION_POSITIVE) {
			*value += pos_delta;
		} else {
			*value -= pos_delta;
		}
	}

	return 0;
}

int step_dir_stepper_stm_timer_move_to(const struct device *dev, const int32_t value)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;
	// const struct step_dir_stepper_stm_timer_config *config = dev->config;

	// if (data->microstep_interval_ns == 0) {
	// 	LOG_ERR("Step interval not set or invalid step interval set");
	// 	return -EINVAL;
	// }

	// step_dir_stepper_stm_timer_move_by(dev, value - data->actual_position);

	return step_dir_stepper_stm_timer_move_by(dev, value - data->actual_position);
}

int step_dir_stepper_stm_timer_is_moving(const struct device *dev, bool *is_moving)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;

	*is_moving = data->counter_running;

	return 0;
}

int step_dir_stepper_stm_timer_run(const struct device *dev, const enum stepper_direction direction)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;
	const struct step_dir_stepper_stm_timer_config *config = dev->config;
	// TODO: Error handling

	/* Stop step signal, no error handling, as counter driver always returns 0 */
	counter_stop(config->step_generator);

	/* Update position */
	uint32_t pos_delta;
	counter_get_value(config->step_counter, &pos_delta);
	if (data->direction == STEPPER_DIRECTION_POSITIVE) {
		LOG_INF("Added Run %u", pos_delta);
		data->actual_position += pos_delta;
	} else {
		LOG_INF("Subtracted Run %u", pos_delta);
		data->actual_position -= pos_delta;
	}

	/* Update Direction */
	data->direction = direction;

	/* Set step count to max. The driver will only update position at that point, not stop. Note
	 * that reaching that point causes integer over/underflow, but that is an api limitation.
	 */
	data->cfg_count.ticks = UINT32_MAX;
	counter_set_top_value(config->step_counter, &data->cfg_count);

	data->run_mode = STEPPER_RUN_MODE_VELOCITY;

	/* Start step signal*/
	counter_start(config->step_generator);
	data->counter_running = true;
	LOG_INF("Starting run");

	return 0;
}

int step_dir_stepper_stm_timer_set_event_callback(const struct device *dev,
						  stepper_event_callback_t callback,
						  void *user_data)
{
	struct step_dir_stepper_stm_timer_data *data = dev->data;

	data->callback = callback;
	data->event_cb_user_data = user_data;
	LOG_INF("Callback: %u---------------------------", (uint32_t)callback);
	LOG_INF("Saved-Callback: %u---------------------", (uint32_t)data->callback);
	return 0;
}

int step_dir_stepper_stm_timer_stop(const struct device *dev)
{
	const struct step_dir_stepper_stm_timer_config *config = dev->config;
	struct step_dir_stepper_stm_timer_data *data = dev->data;

	counter_stop(config->step_generator);
	if (data->counter_running) {
		uint32_t pos_delta;
		counter_get_value(config->step_counter, &pos_delta);
		if (data->direction == STEPPER_DIRECTION_POSITIVE) {
			LOG_INF("Added Stop %u", pos_delta);
			data->actual_position += pos_delta;
		} else {
			LOG_INF("Subtracted Stop %u", pos_delta);
			data->actual_position -= pos_delta;
		}
	}
	LL_TIM_SetCounter(config->tim_count, 0);

	data->counter_running = false;
	return 0;
}
