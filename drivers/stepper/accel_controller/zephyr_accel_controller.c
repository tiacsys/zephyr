/*
 * Copyright 2025 Copyright (c) 2026 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#define DT_DRV_COMPAT zephyr_accel_controller

#include <stdlib.h>
#include <zephyr/drivers/stepper.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(czag_accel_controller, CONFIG_STEPPER_LOG_LEVEL);

#define STEPPER_RUN_MODE_STOP (STEPPER_RUN_MODE_VELOCITY + 25)

K_THREAD_STACK_DEFINE(czag_accel_controller_work_queue_stack,
		      CONFIG_ZEPHYR_ACCEL_CONTROLLER_WORK_QUEUE_STACK_SIZE);
static struct k_work_q czag_accel_controller_wq;
bool wq_initialized = false;

struct czag_accel_controller_config {
	const struct device *stepper_ctrl;
	uint64_t accel_decel_time_microstep_ns;
	uint32_t update_interval_ns;
	float accel_decel_mult;
};

struct czag_accel_controller_data {
	struct k_sem semaphore;
	uint32_t step_width_ns;
	struct k_work_delayable accel_work;
	struct k_work_delayable decel_work_start;
	struct k_work_delayable decel_work;
	uint32_t start_segments;
	uint32_t stop_segments;
	uint32_t segment_index_start;
	uint32_t segment_index_stop;
	uint64_t shadow_microstep_interval_ns;
	uint64_t microstep_interval_ns;
	uint32_t shadow_steps_s;
	uint32_t steps_s;
	uint32_t steps_s_start;
	uint32_t steps_s_stop;
	uint32_t steps_s_current;
	uint64_t const_time_ns;
	enum stepper_direction direction;
	bool decelerate;
	bool stopping;
	const struct device *dev;
	stepper_event_callback_t callback;
	void *callback_data;
	bool callback_initialized;
	bool accel_cancel;
	bool decel_start_cancel;
	bool decel_cancel;
	enum stepper_run_mode mode;
	int target_pos;
};

void czag_callback_handler(const struct device *dev, const enum stepper_event event,
			   void *user_data)
{
	struct czag_accel_controller_data *data = user_data;

	if (event == STEPPER_EVENT_STOPPED || event == STEPPER_EVENT_STEPS_COMPLETED) {
		data->mode = STEPPER_RUN_MODE_HOLD;
	}
	if (event == STEPPER_EVENT_STEPS_COMPLETED && data->stopping) {
		data->stopping = false;
		data->callback(data->dev, STEPPER_EVENT_STOPPED, data->callback_data);
	} else {
		data->callback(data->dev, event, data->callback_data);
	}
}

static void czag_event_callback_init(const struct device *dev)
{
	const struct czag_accel_controller_config *config = dev->config;
	struct czag_accel_controller_data *data = dev->data;

	if (!data->callback_initialized) {
		stepper_set_event_callback(config->stepper_ctrl, czag_callback_handler, data);
		data->callback_initialized = true;
	}
}
static int czag_accel_controller_adjust_speed(const struct device *dev, const int32_t micro_steps,
					      uint32_t steps_s_start, uint32_t *accel_steps,
					      uint32_t *decel_steps, uint32_t *new_steps_s)
{
	const struct czag_accel_controller_config *config = dev->config;
	// struct czag_accel_controller_data *data = dev->data;

	uint32_t decel_steps_base = ((uint64_t)steps_s_start * (uint64_t)steps_s_start *
				     config->accel_decel_time_microstep_ns / NSEC_PER_SEC / 2) *
				    config->accel_decel_mult;

	if (decel_steps_base > abs(micro_steps)) {
		LOG_ERR("Insufficient steps to decelerate, is %d but needs at least %u",
			abs(micro_steps), decel_steps_base);
		return -EINVAL;
	}
	if (decel_steps_base == abs(micro_steps)) {
		*accel_steps = 0;
		*decel_steps = decel_steps_base;
		*new_steps_s = steps_s_start;
		return 0;
	}
	// Only decel_steps_base < abs(micro_steps) remains
	uint32_t diff = *accel_steps + *decel_steps - abs(micro_steps);
	*accel_steps -= diff / 2;
	*decel_steps -= diff / 2;

	*new_steps_s = sqrtl(
		steps_s_start +
		2.0 * (*accel_steps) /
			((long double)config->accel_decel_mult *
			 ((long double)config->accel_decel_time_microstep_ns / NSEC_PER_SEC)));
	return 0;
}

static int czag_accel_controller_move_by(const struct device *dev, const int32_t micro_steps)
{
	const struct czag_accel_controller_config *config = dev->config;
	struct czag_accel_controller_data *data = dev->data;

	bool running = false;
	int ret = 0;
	uint32_t steps_s_start;
	uint64_t base_steps_s;

	if ((data->mode == STEPPER_RUN_MODE_POSITION || data->mode == STEPPER_RUN_MODE_STOP) &&
	    micro_steps != 0) {
		LOG_ERR("move_by while run_mode is POSITION or STOP is not supported");
		return -EBUSY;
	}

	czag_event_callback_init(dev);

	k_sem_take(&data->semaphore, K_FOREVER);

	// TODO: Consolidate code shared with run

	stepper_is_moving(config->stepper_ctrl, &running);
	if (!running) {
		steps_s_start = 0;
		base_steps_s = 0;
	} else {
		steps_s_start = data->steps_s_current;
		base_steps_s = min(steps_s_start, data->shadow_steps_s);
	}

	if (micro_steps > 0) {
		if (data->direction == STEPPER_DIRECTION_NEGATIVE && running) {
			LOG_ERR("Change of direction of moving system not supported");
			ret = -EINVAL;
			goto end;
		}
		data->direction = STEPPER_DIRECTION_POSITIVE;
	} else if (micro_steps < 0) {
		if (data->direction == STEPPER_DIRECTION_POSITIVE && running) {
			LOG_ERR("Change of direction of moving system not supported");
			ret = -EINVAL;
			goto end;
		}
		data->direction = STEPPER_DIRECTION_NEGATIVE;
	} else {
		data->steps_s_current = 0;
		ret = 0;
		goto end;
	}

	uint32_t accel_steps =
		(abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
		 abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
		 config->accel_decel_time_microstep_ns / NSEC_PER_SEC / 2) *
			config->accel_decel_mult +
		(base_steps_s * abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
		 config->accel_decel_time_microstep_ns / NSEC_PER_SEC) *
			config->accel_decel_mult;
	uint32_t decel_steps = ((uint64_t)data->shadow_steps_s * (uint64_t)data->shadow_steps_s *
				config->accel_decel_time_microstep_ns / NSEC_PER_SEC / 2) *
			       config->accel_decel_mult;

	// TODO: Implement solution for this to work instead
	if (accel_steps + decel_steps > abs(micro_steps)) {
		// LOG_ERR("Step number to smal to perform acceleration and deceleration, is %d but
		// " 	"needs to be at least %u", 	abs(micro_steps), accel_steps + decel_steps); ret =
		// -EINVAL; goto end;
		uint32_t new_steps_s = 0;
		ret = czag_accel_controller_adjust_speed(dev, micro_steps, steps_s_start,
							 &accel_steps, &decel_steps, &new_steps_s);
		if (ret != 0) {
			goto end;
		}
		data->steps_s = new_steps_s;
		data->microstep_interval_ns = NSEC_PER_MSEC / new_steps_s;
	} else {
		data->microstep_interval_ns = data->shadow_microstep_interval_ns;
		data->steps_s = data->shadow_steps_s;
	}

	data->mode = STEPPER_RUN_MODE_POSITION;

	data->steps_s_start = steps_s_start;
	data->start_segments =
		(uint64_t)abs((int32_t)data->steps_s - (int32_t)data->steps_s_start) *
		config->accel_decel_time_microstep_ns / ((uint64_t)config->update_interval_ns);
	data->stop_segments = (data->steps_s) * config->accel_decel_time_microstep_ns /
			      ((uint64_t)config->update_interval_ns);

	data->segment_index_stop = data->stop_segments;
	data->steps_s_stop = data->steps_s;

	uint64_t const_steps = (abs(micro_steps) - accel_steps - decel_steps);
	data->const_time_ns = const_steps * NSEC_PER_SEC / data->steps_s;

	data->decelerate = true;
	data->stopping = false;

	int32_t position;

	stepper_get_actual_position(config->stepper_ctrl, &position);
	data->target_pos = position + micro_steps;

	LOG_DBG("Started movement, Accel Steps: %u, Const Steps: %llu, Decel Steps: %u",
		accel_steps, const_steps, decel_steps);
	LOG_DBG("Accel Segments: %u, Decel Segments: %u",
		data->start_segments, data->stop_segments);
	if (data->steps_s_current < data->steps_s) {
		data->segment_index_start = 1;
		if (!running) {
			uint32_t target_steps_s =
				((uint64_t)data->segment_index_start * 1000 + 500) *
				data->steps_s / (data->start_segments * 1000);
			uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
			// LOG_INF("T Step Int: %llu", target_step_interval);
			stepper_set_microstep_interval(config->stepper_ctrl, target_step_interval);
		}
		data->accel_cancel = false;
		data->decel_start_cancel = true;
		data->decel_cancel = true;
		k_work_reschedule(&data->accel_work, K_NSEC(config->update_interval_ns));
	} else if (data->steps_s_current > data->steps_s) {
		data->segment_index_start = data->start_segments;
		data->accel_cancel = true;
		data->decel_start_cancel = false;
		data->decel_cancel = true;
		k_work_reschedule(&data->decel_work_start, K_NSEC(config->update_interval_ns));
	} else {

		data->accel_cancel = true;
		data->decel_start_cancel = true;
		data->decel_cancel = false;
		k_work_reschedule(&data->decel_work, K_NSEC(data->const_time_ns));
	}

end:
	k_sem_give(&data->semaphore);

	if (ret == 0) {
		return stepper_move_by(config->stepper_ctrl, micro_steps);
	} else {
		return ret;
	}
}

static int czag_accel_controller_set_microstep_interval(const struct device *dev,
							const uint64_t microstep_interval_ns)
{
	struct czag_accel_controller_data *data = dev->data;
	const struct czag_accel_controller_config *config = data->dev->config;

	if (data->mode == STEPPER_RUN_MODE_POSITION || data->mode == STEPPER_RUN_MODE_STOP) {
		LOG_ERR("set_microstep_interval while run_mode is POSITION or STOP is not "
			"supported");
		return -EBUSY;
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	data->shadow_microstep_interval_ns = microstep_interval_ns;
	data->shadow_steps_s = NSEC_PER_SEC / microstep_interval_ns;

	k_sem_give(&data->semaphore);

	//TODO: Error handling
	if (data->mode == STEPPER_RUN_MODE_VELOCITY) {
		stepper_run(dev, data->direction);
	}

	if (data->mode == STEPPER_RUN_MODE_HOLD) {
		stepper_set_microstep_interval(config->stepper_ctrl, microstep_interval_ns);
	}

	return 0;
}

void czag_accel_controller_wq_accel_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct czag_accel_controller_data *data =
		CONTAINER_OF(dwork, struct czag_accel_controller_data, accel_work);
	const struct czag_accel_controller_config *config = data->dev->config;

	k_sem_take(&data->semaphore, K_FOREVER);
	if (data->accel_cancel) {
		data->accel_cancel = false;
		goto end;
	}

	if (data->segment_index_start == 0) {
		LOG_ERR("Error during acceleration calculation");
		stepper_stop(config->stepper_ctrl);
		goto end;
	}
	uint32_t target_steps_s =
		((uint64_t)data->segment_index_start * 1000 + 500) *
			((uint64_t)data->steps_s - (uint64_t)data->steps_s_start) /
			((uint64_t)data->start_segments * 1000) +
		(uint64_t)data->steps_s_start;
	uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
	// Clamp new microstep interval
	if (data->microstep_interval_ns > target_step_interval) {
		target_step_interval = data->microstep_interval_ns;
		target_steps_s = data->steps_s;
	}
	int ret = stepper_set_microstep_interval(config->stepper_ctrl, target_step_interval);
	data->segment_index_start++;
	if (ret != 0) {
		goto end;
	}
	data->steps_s_current = target_steps_s;

	if (data->segment_index_start <= data->start_segments) {
		k_work_reschedule(dwork, K_NSEC(config->update_interval_ns));
	} else {
		LOG_DBG("Finished Acceleration");
		if (data->decelerate) {
			data->decel_cancel = false;
			k_work_reschedule(&data->decel_work, K_NSEC(data->const_time_ns));
		}
		int32_t position;
		stepper_get_actual_position(config->stepper_ctrl, &position);
		LOG_DBG("Position: %d", position);
	}
end:
	k_sem_give(&data->semaphore);
}

void czag_accel_controller_wq_decel_start_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct czag_accel_controller_data *data =
		CONTAINER_OF(dwork, struct czag_accel_controller_data, decel_work_start);
	const struct czag_accel_controller_config *config = data->dev->config;

	k_sem_take(&data->semaphore, K_FOREVER);

	if (data->decel_start_cancel) {
		data->decel_start_cancel = false;
		goto end;
	}

	uint32_t target_steps_s =
		((uint64_t)data->segment_index_start * 1000 + 500) *
			((uint64_t)data->steps_s_start - (uint64_t)data->steps_s) /
			((uint64_t)data->start_segments * 1000) +
		(uint64_t)data->steps_s;
	uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
	// Clamp new microstep interval
	if (data->microstep_interval_ns < target_step_interval) {
		target_step_interval = data->microstep_interval_ns;
		target_steps_s = data->steps_s;
	}
	int ret = stepper_set_microstep_interval(config->stepper_ctrl, target_step_interval);
	data->segment_index_start--;
	if (ret != 0) {
		goto end;
	}
	data->steps_s_current = target_steps_s;

	if (data->segment_index_start > 0) {
		k_work_reschedule(dwork, K_NSEC(config->update_interval_ns));
	} else {
		LOG_DBG("Finished Deceleration at start");
		if (data->decelerate) {
			data->decel_cancel = false;
			k_work_reschedule(&data->decel_work, K_NSEC(data->const_time_ns));
		}
		int32_t position;
		stepper_get_actual_position(config->stepper_ctrl, &position);
		LOG_DBG("Position: %d", position);
	}
end:
	k_sem_give(&data->semaphore);
}

void czag_accel_controller_wq_decel_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct czag_accel_controller_data *data =
		CONTAINER_OF(dwork, struct czag_accel_controller_data, decel_work);
	const struct czag_accel_controller_config *config = data->dev->config;
	if (data->segment_index_stop == data->stop_segments) {
		int32_t position;
		stepper_get_actual_position(config->stepper_ctrl, &position);
		LOG_DBG("Started Deceleration");
		LOG_DBG("Position: %d", position);
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	if (data->decel_cancel) {
		data->decel_cancel = false;
		goto end;
	}

	uint32_t target_steps_s = ((uint64_t)data->segment_index_stop * 1000 + 500) *
				  (uint64_t)data->steps_s_stop /
				  ((uint64_t)data->stop_segments * 1000);
	if (target_steps_s == 0) {
		target_steps_s = 1;
	}
	uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
	// Clamp new microstep interval
	if (data->microstep_interval_ns > target_step_interval) {
		target_step_interval = data->microstep_interval_ns;
	}
	int ret = stepper_set_microstep_interval(config->stepper_ctrl, target_step_interval);
	data->segment_index_stop--;
	if (ret != 0) {
		goto end;
	}
	data->steps_s_current = target_steps_s;

	if (data->segment_index_stop > 0) {
		k_work_reschedule(dwork, K_NSEC(config->update_interval_ns));
	} else {
		data->decelerate = false;
		data->steps_s_current = 0;
		LOG_DBG("Finished Deceleration");
		int32_t position;
		stepper_get_actual_position(config->stepper_ctrl, &position);
		LOG_DBG("Position: %d", position);
	}
end:
	k_sem_give(&data->semaphore);
}

int czag_accel_controller_set_reference_position(const struct device *dev, const int32_t value)
{
	const struct czag_accel_controller_config *config = dev->config;

	bool moving;
	stepper_is_moving(config->stepper_ctrl, &moving);
	if (moving) {
		LOG_ERR("set_reference_position while moving is not supported");
		return -EBUSY;
	}

	return stepper_set_reference_position(config->stepper_ctrl, value);
}

int czag_accel_controller_get_actual_position(const struct device *dev, int32_t *value)
{
	const struct czag_accel_controller_config *config = dev->config;
	return stepper_get_actual_position(config->stepper_ctrl, value);
}

int czag_accel_controller_move_to(const struct device *dev, const int32_t value)
{
	const struct czag_accel_controller_config *config = dev->config;

	bool moving;
	stepper_is_moving(config->stepper_ctrl, &moving);
	if (moving) {
		LOG_ERR("move_to while moving is not supported");
		return -EBUSY;
	}

	int32_t position;

	stepper_get_actual_position(config->stepper_ctrl, &position);

	return czag_accel_controller_move_by(dev, value - position);
}

int czag_accel_controller_is_moving(const struct device *dev, bool *is_moving)
{
	const struct czag_accel_controller_config *config = dev->config;
	return stepper_is_moving(config->stepper_ctrl, is_moving);
}

int czag_accel_controller_run(const struct device *dev, const enum stepper_direction direction)
{
	const struct czag_accel_controller_config *config = dev->config;
	struct czag_accel_controller_data *data = dev->data;

	czag_event_callback_init(dev);

	bool running = false;
	int ret = 0;

	if (data->mode == STEPPER_RUN_MODE_STOP) {
		LOG_ERR("run while run_mode is STOP is not supported");
		return -EBUSY;
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	stepper_is_moving(config->stepper_ctrl, &running);
	if (running && direction != data->direction) {
		LOG_ERR("Change of direction of moving system not supported");
		ret = -EINVAL;
		goto end;
	}

	data->microstep_interval_ns = data->shadow_microstep_interval_ns;
	data->steps_s = data->shadow_steps_s;

	data->mode = STEPPER_RUN_MODE_VELOCITY;

	data->stopping = false;

	if (!running) {
		data->steps_s_start = 0;
	} else {
		data->steps_s_start = data->steps_s_current;
	}

	data->start_segments =
		(uint64_t)abs((int32_t)data->steps_s - (int32_t)data->steps_s_start) *
		config->accel_decel_time_microstep_ns / ((uint64_t)config->update_interval_ns);
	data->direction = direction;

	data->decelerate = false;

	LOG_DBG("Started Acceleration, Segments: %u", data->start_segments);
	if (data->steps_s_current < data->steps_s) {
		data->segment_index_start = 1;
		if (!running) {
			uint32_t target_steps_s =
				((uint64_t)data->segment_index_start * 1000 + 500) *
				(uint64_t)data->steps_s / (data->start_segments * 1000);
			uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
			stepper_set_microstep_interval(config->stepper_ctrl, target_step_interval);
		}
		data->accel_cancel = false;
		data->decel_start_cancel = true;
		data->decel_cancel = true;
		k_work_reschedule(&data->accel_work, K_NSEC(config->update_interval_ns));
	} else if (data->steps_s_current > data->steps_s) {
		data->segment_index_start = data->start_segments;
		data->accel_cancel = true;
		data->decel_start_cancel = false;
		data->decel_cancel = true;
		k_work_reschedule(&data->decel_work_start, K_NSEC(config->update_interval_ns));
	} else {
		data->accel_cancel = true;
		data->decel_start_cancel = true;
		data->decel_cancel = true;
	}
end:
	k_sem_give(&data->semaphore);
	if (ret == 0) {
		return stepper_run(config->stepper_ctrl, direction);
	} else {
		return ret;
	}
}

int czag_accel_controller_set_event_callback(const struct device *dev,
					     stepper_event_callback_t callback, void *user_data)
{
	struct czag_accel_controller_data *data = dev->data;

	data->callback = callback;
	data->callback_data = user_data;
	return 0;
}

int czag_accel_controller_stop(const struct device *dev)
{
	const struct czag_accel_controller_config *config = dev->config;
	struct czag_accel_controller_data *data = dev->data;

	czag_event_callback_init(dev);

	if (data->mode == STEPPER_RUN_MODE_STOP) {
		return 0;
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	// No update of shadow values, as we just want to stop

	data->stopping = true;
	data->mode = STEPPER_RUN_MODE_STOP;

	int32_t decel_steps = ((uint64_t)data->steps_s_current * (uint64_t)data->steps_s_current *
			       config->accel_decel_time_microstep_ns / NSEC_PER_SEC / 2) *
			      config->accel_decel_mult;

	data->stop_segments = ((uint64_t)data->steps_s_current) *
			      config->accel_decel_time_microstep_ns /
			      ((uint64_t)config->update_interval_ns);
	data->segment_index_stop = data->stop_segments;
	data->steps_s_stop = data->steps_s_current;
	if (data->direction == STEPPER_DIRECTION_NEGATIVE) {
		decel_steps = -decel_steps;
	}
	stepper_move_by(config->stepper_ctrl, decel_steps);
	LOG_DBG("Started Decelleration: %d", decel_steps);
	data->accel_cancel = true;
	data->decel_start_cancel = true;
	data->decel_cancel = false;
	k_work_reschedule(&data->decel_work, K_NSEC(config->update_interval_ns));

	k_sem_give(&data->semaphore);

	return 0;
}

static int czag_accel_controller_init(const struct device *dev)
{
	struct czag_accel_controller_data *data = dev->data;

	if (!wq_initialized) {
		k_work_queue_init(&czag_accel_controller_wq);

		k_work_queue_start(
			&czag_accel_controller_wq, czag_accel_controller_work_queue_stack,
			K_THREAD_STACK_SIZEOF(czag_accel_controller_work_queue_stack), -5, NULL);
		wq_initialized = true;
	}

	data->decelerate = false;
	data->accel_cancel = false;
	data->decel_start_cancel = false;
	data->decel_cancel = false;
	data->mode = STEPPER_RUN_MODE_HOLD;
	k_sem_init(&data->semaphore, 1, 1);

	k_work_init_delayable(&data->accel_work, czag_accel_controller_wq_accel_handler);
	k_work_init_delayable(&data->decel_work_start,
			      czag_accel_controller_wq_decel_start_handler);
	k_work_init_delayable(&data->decel_work, czag_accel_controller_wq_decel_handler);

	return 0;
}

static DEVICE_API(stepper, czag_accel_controller_api) = {
	.move_by = czag_accel_controller_move_by,
	.move_to = czag_accel_controller_move_to,
	.is_moving = czag_accel_controller_is_moving,
	.set_reference_position = czag_accel_controller_set_reference_position,
	.get_actual_position = czag_accel_controller_get_actual_position,
	.set_event_callback = czag_accel_controller_set_event_callback,
	.set_microstep_interval = czag_accel_controller_set_microstep_interval,
	.run = czag_accel_controller_run,
	.stop = czag_accel_controller_stop,
};

#define CZ_ACCEL_CONTROLLER_DEFINE(inst)                                                           \
	static const struct czag_accel_controller_config czag_accel_controller_config_##inst = {   \
		.stepper_ctrl = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), stepper_ctrl)),        \
		.accel_decel_time_microstep_ns = DT_INST_PROP(inst, accel_time_microstep),         \
		.update_interval_ns = DT_INST_PROP(inst, update_interval),                         \
		.accel_decel_mult = 0.98f + (float)DT_INST_PROP(inst, delay) / 1000.0f,             \
	};                                                                                         \
	static struct czag_accel_controller_data czag_accel_controller_data_##inst = {             \
		.dev = DEVICE_DT_INST_GET(inst),                                                   \
		.callback_initialized = false,                                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, czag_accel_controller_init, NULL,                              \
			      &czag_accel_controller_data_##inst,                                  \
			      &czag_accel_controller_config_##inst, POST_KERNEL,                   \
			      CONFIG_STEPPER_INIT_PRIORITY, &czag_accel_controller_api);

DT_INST_FOREACH_STATUS_OKAY(CZ_ACCEL_CONTROLLER_DEFINE)
