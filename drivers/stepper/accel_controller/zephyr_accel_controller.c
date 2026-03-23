/*
 * Copyright 2025 Copyright (c) 2026 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_accel_controller

#include <math.h>
#include <stdlib.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zephyr_accel_controller, CONFIG_STEPPER_LOG_LEVEL);

/* Custom stepper run mode extension for state tracking. */
// TODO: extend api
#define STEPPER_CTRL_RUN_MODE_STOP (STEPPER_CTRL_RUN_MODE_VELOCITY + 25)

K_THREAD_STACK_DEFINE(zephyr_accel_controller_work_queue_stack,
		      CONFIG_ZEPHYR_ACCEL_CONTROLLER_WORK_QUEUE_STACK_SIZE);
static struct k_work_q zephyr_accel_controller_wq;

struct zephyr_accel_controller_config {
	const struct device *stepper_ctrl;
	uint32_t update_interval_ns;
	float accel_decel_mult;
};

struct zephyr_accel_controller_data {
	struct k_sem semaphore;
	struct k_work_delayable accel_work;
	struct k_work_delayable decel_work_start;
	struct k_work_delayable decel_work;
	uint64_t accel_time_microstep_ns;
	uint64_t decel_time_microstep_ns;
	uint32_t start_segments;
	uint32_t stop_segments;
	uint32_t segment_index;
	uint64_t shadow_microstep_interval_ns;
	uint64_t microstep_interval_ns;
	uint32_t shadow_steps_s;
	uint32_t steps_s;
	uint32_t steps_s_start;
	uint32_t steps_s_stop_start;
	uint32_t steps_s_current;
	uint64_t const_time_ns;
	enum stepper_ctrl_direction direction;
	const struct device *dev;
	stepper_ctrl_event_callback_t callback;
	void *callback_data;
	bool callback_initialized;
	bool accel_cancel;
	bool decel_start_cancel;
	bool decel_cancel;
	enum stepper_ctrl_run_mode mode;
};

// TODO: extend api: speed reached event

void accel_controller_callback_handler(const struct device *dev,
				       const enum stepper_ctrl_event event, void *user_data)
{
	struct zephyr_accel_controller_data *data = user_data;
	enum stepper_ctrl_event modified_event = event;

	if (event == STEPPER_CTRL_EVENT_STEPS_COMPLETED &&
	    data->mode == STEPPER_CTRL_RUN_MODE_STOP) {
		modified_event = STEPPER_CTRL_EVENT_STOPPED;
	}
	data->mode = STEPPER_CTRL_RUN_MODE_HOLD;
	data->steps_s_current = 0;
	data->callback(data->dev, modified_event, data->callback_data);
}

/* As the controlled motion controller might be initialized after this motion controller,
 * register the callback on the first motion function called.
 */
static inline int accel_controller_event_callback_init(const struct device *dev)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	struct zephyr_accel_controller_data *data = dev->data;
	int ret = 0;

	if (!data->callback_initialized) {
		ret = stepper_ctrl_set_event_cb(config->stepper_ctrl,
						accel_controller_callback_handler, data);
		if (ret != 0) {
			LOG_ERR("Could not setup event handling");
			return ret;
		}
		data->callback_initialized = true;
	}

	return ret;
}

static inline uint32_t zephyr_accel_controller_decel_steps(const struct device *dev,
							   uint64_t start_steps_s)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	struct zephyr_accel_controller_data *data = dev->data;

	return (start_steps_s * start_steps_s * data->decel_time_microstep_ns / NSEC_PER_SEC / 2) *
	       config->accel_decel_mult;
}

static inline int zephyr_accel_controller_adjust_speed(const struct device *dev,
						       const int32_t micro_steps,
						       uint32_t steps_s_start,
						       uint32_t *accel_steps, uint32_t *decel_steps,
						       uint32_t *new_steps_s)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	struct zephyr_accel_controller_data *data = dev->data;

	uint32_t decel_steps_base = zephyr_accel_controller_decel_steps(dev, steps_s_start);

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
	/* Only decel_steps_base < abs(micro_steps) remains */
	uint32_t diff = *accel_steps + *decel_steps - abs(micro_steps);
	*accel_steps -= diff / 2;
	*decel_steps -= diff / 2;

	// TODO:Adjust
	*new_steps_s = sqrtl(steps_s_start +
			     2.0 * (*accel_steps) /
				     ((long double)config->accel_decel_mult *
				      ((long double)data->accel_time_microstep_ns / NSEC_PER_SEC)));

	return 0;
}

static inline uint32_t zephyr_accel_controller_start_segments(const struct device *dev,
							      uint32_t start_steps_s,
							      uint32_t stop_steps_s)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	struct zephyr_accel_controller_data *data = dev->data;

	// TODO: Adjust for deceleratig start
	if (start_steps_s <= stop_steps_s) {
		return (uint64_t)(stop_steps_s - start_steps_s) * data->accel_time_microstep_ns /
		       ((uint64_t)config->update_interval_ns);
	} else {
		return (uint64_t)abs((int32_t)start_steps_s - (int32_t)stop_steps_s) *
		       data->decel_time_microstep_ns / ((uint64_t)config->update_interval_ns);
	}
}

static int zephyr_accel_controller_move_by(const struct device *dev, const int32_t micro_steps)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	struct zephyr_accel_controller_data *data = dev->data;

	bool running = false;
	int ret = 0;
	uint32_t steps_s_start;
	uint64_t base_steps_s;

	ret = accel_controller_event_callback_init(dev);
	if (ret != 0) {
		return ret;
	}

	if ((data->mode == STEPPER_CTRL_RUN_MODE_POSITION ||
	     data->mode == STEPPER_CTRL_RUN_MODE_STOP) &&
	    micro_steps != 0) {
		LOG_ERR("move_by while run_mode is POSITION or STOP is not supported");
		return -EBUSY;
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	stepper_ctrl_is_moving(config->stepper_ctrl, &running);
	if (!running) {
		steps_s_start = 0;
		base_steps_s = 0;
	} else {
		steps_s_start = data->steps_s_current;
		base_steps_s = min(steps_s_start, data->shadow_steps_s);
	}

	if (micro_steps > 0) {
		if (data->direction == STEPPER_CTRL_DIRECTION_NEGATIVE && running) {
			LOG_ERR("Change of direction of moving system not supported");
			ret = -EINVAL;
			goto end;
		}
		data->direction = STEPPER_CTRL_DIRECTION_POSITIVE;
	} else if (micro_steps < 0) {
		if (data->direction == STEPPER_CTRL_DIRECTION_POSITIVE && running) {
			LOG_ERR("Change of direction of moving system not supported");
			ret = -EINVAL;
			goto end;
		}
		data->direction = STEPPER_CTRL_DIRECTION_NEGATIVE;
	} else {
		data->steps_s_current = 0;
		data->accel_cancel = true;
		data->decel_start_cancel = true;
		data->decel_cancel = true;
		ret = 0;
		goto end;
	}

	// TODO: Adjust for decel start
	uint32_t accel_steps = 0;
	if (data->shadow_steps_s >= steps_s_start) {
		accel_steps = (abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
			       abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
			       data->accel_time_microstep_ns / NSEC_PER_SEC / 2) *
				      config->accel_decel_mult +
			      (base_steps_s *
			       abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
			       data->accel_time_microstep_ns / NSEC_PER_SEC) *
				      config->accel_decel_mult;
	} else {
		accel_steps = (abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
			       abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
			       data->decel_time_microstep_ns / NSEC_PER_SEC / 2) *
				      config->accel_decel_mult +
			      (base_steps_s *
			       abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
			       data->decel_time_microstep_ns / NSEC_PER_SEC) *
				      config->accel_decel_mult;
	}
	// uint32_t accel_steps =
	// 	(abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
	// 	 abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
	// 	 data->accel_time_microstep_ns / NSEC_PER_SEC / 2) *
	// 		config->accel_decel_mult +
	// 	(base_steps_s * abs((int32_t)data->shadow_steps_s - (int32_t)steps_s_start) *
	// 	 data->accel_time_microstep_ns / NSEC_PER_SEC) *
	// 		config->accel_decel_mult;
	uint32_t decel_steps = zephyr_accel_controller_decel_steps(dev, data->shadow_steps_s);

	if (accel_steps + decel_steps > abs(micro_steps)) {
		uint32_t new_steps_s = 0;
		ret = zephyr_accel_controller_adjust_speed(
			dev, micro_steps, steps_s_start, &accel_steps, &decel_steps, &new_steps_s);
		if (ret != 0) {
			goto end;
		}
		data->steps_s = new_steps_s;
		data->microstep_interval_ns = NSEC_PER_SEC / new_steps_s;
	} else {
		data->microstep_interval_ns = data->shadow_microstep_interval_ns;
		data->steps_s = data->shadow_steps_s;
	}

	data->mode = STEPPER_CTRL_RUN_MODE_POSITION;

	data->steps_s_start = steps_s_start;
	data->start_segments =
		zephyr_accel_controller_start_segments(dev, steps_s_start, data->steps_s);
	data->stop_segments = (data->steps_s) * data->decel_time_microstep_ns /
			      ((uint64_t)config->update_interval_ns);

	data->steps_s_stop_start = data->steps_s;

	uint64_t const_steps = (abs(micro_steps) - accel_steps - decel_steps);
	data->const_time_ns = const_steps * NSEC_PER_SEC / data->steps_s;

	LOG_DBG("Started movement, Accel Steps: %u, Const Steps: %llu, Decel Steps: %u",
		accel_steps, const_steps, decel_steps);
	LOG_DBG("Accel Segments: %u, Decel Segments: %u", data->start_segments,
		data->stop_segments);
	if (data->steps_s_current < data->steps_s) {
		data->segment_index = 1;
		if (!running) {
			uint32_t target_steps_s = ((uint64_t)data->segment_index * 1000 + 500) *
						  data->steps_s / (data->start_segments * 1000);
			uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
			stepper_ctrl_set_microstep_interval(config->stepper_ctrl,
							    target_step_interval);
		}
		data->accel_cancel = false;
		data->decel_start_cancel = true;
		data->decel_cancel = true;
		k_work_reschedule(&data->accel_work, K_NSEC(config->update_interval_ns));
	} else if (data->steps_s_current > data->steps_s) {
		data->segment_index = data->start_segments;
		data->accel_cancel = true;
		data->decel_start_cancel = false;
		data->decel_cancel = true;
		k_work_reschedule(&data->decel_work_start, K_NSEC(config->update_interval_ns));
	} else {

		data->accel_cancel = true;
		data->decel_start_cancel = true;
		data->decel_cancel = false;
		data->segment_index = data->stop_segments;
		k_work_reschedule(&data->decel_work, K_NSEC(data->const_time_ns));
	}

end:
	k_sem_give(&data->semaphore);

	if (ret == 0) {
		return stepper_ctrl_move_by(config->stepper_ctrl, micro_steps);
	} else {
		return ret;
	}
}

static int zephyr_accel_controller_set_microstep_interval(const struct device *dev,
							  const uint64_t microstep_interval_ns)
{
	struct zephyr_accel_controller_data *data = dev->data;
	const struct zephyr_accel_controller_config *config = dev->config;

	return -ENOSYS;

	/* This is, because delays during calculations would result in additional steps being taken.
	 */
	if (data->mode == STEPPER_CTRL_RUN_MODE_POSITION) {
		LOG_ERR("set_microstep_interval while run_mode is POSITION or STOP is not "
			"supported");
		return -EBUSY;
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	data->shadow_microstep_interval_ns = microstep_interval_ns;
	data->shadow_steps_s = NSEC_PER_SEC / microstep_interval_ns;

	k_sem_give(&data->semaphore);

	if (data->mode == STEPPER_CTRL_RUN_MODE_VELOCITY) {
		return stepper_ctrl_run(dev, data->direction);
	}

	if (data->mode == STEPPER_CTRL_RUN_MODE_HOLD) {
		return stepper_ctrl_set_microstep_interval(config->stepper_ctrl,
							   microstep_interval_ns);
	}

	return 0;
}

static int zephyr_accel_controller_configure_ramp(const struct device *dev,
						  const struct stepper_ctrl_ramp *ramp)
{
	struct zephyr_accel_controller_data *data = dev->data;
	const struct zephyr_accel_controller_config *config = dev->config;

	/* This is, because delays during calculations would result in additional steps being taken.
	 */
	if (data->mode == STEPPER_CTRL_RUN_MODE_POSITION) {
		LOG_ERR("configure_ramp while run_mode is POSITION or STOP is not "
			"supported");
		return -EBUSY;
	}

	if (ramp->acceleration_max == 0 || ramp->deceleration_max == 0 || ramp->speed_max == 0) {
		LOG_ERR("At least one ramp parameter is 0");
		return -EINVAL;
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	uint64_t microstep_interval_ns = NSEC_PER_SEC / ramp->speed_max;

	data->shadow_microstep_interval_ns = microstep_interval_ns;
	data->shadow_steps_s = NSEC_PER_SEC / microstep_interval_ns;

	data->accel_time_microstep_ns = NSEC_PER_SEC / ramp->acceleration_max;
	data->decel_time_microstep_ns = NSEC_PER_SEC / ramp->deceleration_max;

	k_sem_give(&data->semaphore);

	if (data->mode == STEPPER_CTRL_RUN_MODE_VELOCITY) {
		return stepper_ctrl_run(dev, data->direction);
	}

	if (data->mode == STEPPER_CTRL_RUN_MODE_HOLD) {
		return stepper_ctrl_set_microstep_interval(config->stepper_ctrl,
							   microstep_interval_ns);
	}

	return 0;
}

void zephyr_accel_controller_wq_accel_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct zephyr_accel_controller_data *data =
		CONTAINER_OF(dwork, struct zephyr_accel_controller_data, accel_work);
	const struct zephyr_accel_controller_config *config = data->dev->config;

	k_sem_take(&data->semaphore, K_FOREVER);
	if (data->accel_cancel) {
		data->accel_cancel = false;
		goto end;
	}

	if (data->segment_index == 0) {
		LOG_ERR("Error during acceleration calculation");
		stepper_ctrl_stop(config->stepper_ctrl);
		goto end;
	}
	uint32_t target_steps_s =
		((uint64_t)data->segment_index * 1000 + 500) *
			((uint64_t)data->steps_s - (uint64_t)data->steps_s_start) /
			((uint64_t)data->start_segments * 1000) +
		(uint64_t)data->steps_s_start;
	uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
	/* Clamp new microstep interval */
	if (data->steps_s < target_steps_s) {
		target_step_interval = data->microstep_interval_ns;
		target_steps_s = data->steps_s;
	}
	int ret = stepper_ctrl_set_microstep_interval(config->stepper_ctrl, target_step_interval);
	data->segment_index++;
	if (ret != 0) {
		goto end;
	}
	data->steps_s_current = target_steps_s;

	if (data->segment_index <= data->start_segments) {
		k_work_reschedule(dwork, K_NSEC(config->update_interval_ns));
	} else {
		LOG_DBG("Finished Acceleration");
		if (data->mode == STEPPER_CTRL_RUN_MODE_POSITION) {
			data->decel_cancel = false;
			data->segment_index = data->stop_segments;
			k_work_reschedule(&data->decel_work, K_NSEC(data->const_time_ns));
		}
		int32_t position;
		stepper_ctrl_get_actual_position(config->stepper_ctrl, &position);
		LOG_DBG("Position: %d", position);
	}
end:
	k_sem_give(&data->semaphore);
}

void zephyr_accel_controller_wq_decel_start_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct zephyr_accel_controller_data *data =
		CONTAINER_OF(dwork, struct zephyr_accel_controller_data, decel_work_start);
	const struct zephyr_accel_controller_config *config = data->dev->config;

	k_sem_take(&data->semaphore, K_FOREVER);

	if (data->decel_start_cancel) {
		data->decel_start_cancel = false;
		goto end;
	}

	uint32_t target_steps_s =
		((uint64_t)data->segment_index * 1000 + 500) *
			((uint64_t)data->steps_s_start - (uint64_t)data->steps_s) /
			((uint64_t)data->start_segments * 1000) +
		(uint64_t)data->steps_s;
	uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
	/* Clamp new microstep interval */
	if (data->steps_s > target_steps_s) {
		target_step_interval = data->microstep_interval_ns;
		target_steps_s = data->steps_s;
	}
	int ret = stepper_ctrl_set_microstep_interval(config->stepper_ctrl, target_step_interval);
	data->segment_index--;
	if (ret != 0) {
		goto end;
	}
	data->steps_s_current = target_steps_s;

	if (data->segment_index > 0) {
		k_work_reschedule(dwork, K_NSEC(config->update_interval_ns));
	} else {
		LOG_DBG("Finished Deceleration at start");
		if (data->mode == STEPPER_CTRL_RUN_MODE_POSITION) {
			data->decel_cancel = false;
			data->segment_index = data->stop_segments;
			k_work_reschedule(&data->decel_work, K_NSEC(data->const_time_ns));
		}
		int32_t position;
		stepper_ctrl_get_actual_position(config->stepper_ctrl, &position);
		LOG_DBG("Position: %d", position);
	}
end:
	k_sem_give(&data->semaphore);
}

void zephyr_accel_controller_wq_decel_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct zephyr_accel_controller_data *data =
		CONTAINER_OF(dwork, struct zephyr_accel_controller_data, decel_work);
	const struct zephyr_accel_controller_config *config = data->dev->config;
	if (data->segment_index == data->stop_segments) {
		int32_t position;
		stepper_ctrl_get_actual_position(config->stepper_ctrl, &position);
		LOG_DBG("Started Deceleration");
		LOG_DBG("Position: %d", position);
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	if (data->decel_cancel) {
		data->decel_cancel = false;
		goto end;
	}

	uint32_t target_steps_s = ((uint64_t)data->segment_index * 1000 + 500) *
				  (uint64_t)data->steps_s_stop_start /
				  ((uint64_t)data->stop_segments * 1000);
	if (target_steps_s == 0) {
		target_steps_s = 1;
	}
	uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
	/* Clamp new microstep interval */
	if (data->steps_s < target_steps_s) {
		target_step_interval = data->microstep_interval_ns;
	}
	int ret = stepper_ctrl_set_microstep_interval(config->stepper_ctrl, target_step_interval);
	data->segment_index--;
	if (ret != 0) {
		goto end;
	}
	data->steps_s_current = target_steps_s;

	if (data->segment_index > 0) {
		k_work_reschedule(dwork, K_NSEC(config->update_interval_ns));
	} else {
		data->steps_s_current = 0;
		LOG_DBG("Finished Deceleration");
		int32_t position;
		stepper_ctrl_get_actual_position(config->stepper_ctrl, &position);
		LOG_DBG("Position: %d", position);
	}
end:
	k_sem_give(&data->semaphore);
}

int zephyr_accel_controller_set_reference_position(const struct device *dev, const int32_t value)
{
	const struct zephyr_accel_controller_config *config = dev->config;

	bool moving;
	stepper_ctrl_is_moving(config->stepper_ctrl, &moving);
	if (moving) {
		LOG_ERR("set_reference_position while moving is not supported");
		return -EBUSY;
	}

	return stepper_ctrl_set_reference_position(config->stepper_ctrl, value);
}

int zephyr_accel_controller_get_actual_position(const struct device *dev, int32_t *value)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	return stepper_ctrl_get_actual_position(config->stepper_ctrl, value);
}

int zephyr_accel_controller_move_to(const struct device *dev, const int32_t value)
{
	const struct zephyr_accel_controller_config *config = dev->config;

	/* Limitations of the implementation mean that calling move_to while the stepper motor is
	 * moving is not possible. While the position itself is tracked correctly, the stepper
	 * motion controller would stop at a slightly different position.
	 */
	bool moving;
	stepper_ctrl_is_moving(config->stepper_ctrl, &moving);
	if (moving) {
		LOG_ERR("move_to while moving is not supported");
		return -EBUSY;
	}

	int32_t position;

	stepper_ctrl_get_actual_position(config->stepper_ctrl, &position);

	return zephyr_accel_controller_move_by(dev, value - position);
}

int zephyr_accel_controller_is_moving(const struct device *dev, bool *is_moving)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	return stepper_ctrl_is_moving(config->stepper_ctrl, is_moving);
}

int zephyr_accel_controller_run(const struct device *dev,
				const enum stepper_ctrl_direction direction)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	struct zephyr_accel_controller_data *data = dev->data;
	bool running = false;
	int ret = 0;

	ret = accel_controller_event_callback_init(dev);
	if (ret != 0) {
		return ret;
	}

	if (data->mode == STEPPER_CTRL_RUN_MODE_STOP) {
		LOG_ERR("run while run_mode is STOP is not supported");
		return -EBUSY;
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	ret = stepper_ctrl_is_moving(config->stepper_ctrl, &running);
	if (ret != 0) {
		LOG_ERR("Could check if stepper motion controller is moving");
		return ret;
	}
	if (running && direction != data->direction) {
		LOG_ERR("Change of direction of moving system not supported");
		ret = -EINVAL;
		goto end;
	}

	data->microstep_interval_ns = data->shadow_microstep_interval_ns;
	data->steps_s = data->shadow_steps_s;

	data->mode = STEPPER_CTRL_RUN_MODE_VELOCITY;

	if (!running) {
		data->steps_s_start = 0;
	} else {
		data->steps_s_start = data->steps_s_current;
	}

	data->start_segments =
		zephyr_accel_controller_start_segments(dev, data->steps_s_start, data->steps_s);
	data->direction = direction;

	LOG_DBG("Started Acceleration, Segments: %u", data->start_segments);
	// TODO: Check this segment
	if (data->steps_s_current < data->steps_s) {
		data->segment_index = 1;
		if (!running) {
			uint32_t target_steps_s = ((uint64_t)data->segment_index * 1000 + 500) *
						  (uint64_t)data->steps_s /
						  (data->start_segments * 1000);
			uint64_t target_step_interval = NSEC_PER_SEC / target_steps_s;
			stepper_ctrl_set_microstep_interval(config->stepper_ctrl,
							    target_step_interval);
		}
		data->accel_cancel = false;
		data->decel_start_cancel = true;
		data->decel_cancel = true;
		k_work_reschedule(&data->accel_work, K_NSEC(config->update_interval_ns));
	} else if (data->steps_s_current > data->steps_s) {
		data->segment_index = data->start_segments;
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
		return stepper_ctrl_run(config->stepper_ctrl, direction);
	} else {
		return ret;
	}
}

int zephyr_accel_controller_set_event_callback(const struct device *dev,
					       stepper_ctrl_event_callback_t callback,
					       void *user_data)
{
	struct zephyr_accel_controller_data *data = dev->data;

	data->callback = callback;
	data->callback_data = user_data;
	return 0;
}

int zephyr_accel_controller_stop(const struct device *dev)
{
	const struct zephyr_accel_controller_config *config = dev->config;
	struct zephyr_accel_controller_data *data = dev->data;
	int ret = 0;

	ret = accel_controller_event_callback_init(dev);
	if (ret != 0) {
		return ret;
	}

	if (data->mode == STEPPER_CTRL_RUN_MODE_STOP) {
		return 0;
	}

	k_sem_take(&data->semaphore, K_FOREVER);

	/* No update of shadow values, as we just want to stop */

	data->mode = STEPPER_CTRL_RUN_MODE_STOP;

	int32_t decel_steps = zephyr_accel_controller_decel_steps(dev, data->steps_s_current);

	data->stop_segments = ((uint64_t)data->steps_s_current) * data->decel_time_microstep_ns /
			      ((uint64_t)config->update_interval_ns);
	data->segment_index = data->stop_segments;
	data->steps_s_stop_start = data->steps_s_current;
	if (data->direction == STEPPER_CTRL_DIRECTION_NEGATIVE) {
		decel_steps = -decel_steps;
	}
	stepper_ctrl_move_by(config->stepper_ctrl, decel_steps);
	LOG_DBG("Started Decelleration: %d", decel_steps);
	data->accel_cancel = true;
	data->decel_start_cancel = true;
	data->decel_cancel = false;
	k_work_reschedule(&data->decel_work, K_NSEC(config->update_interval_ns));

	k_sem_give(&data->semaphore);

	return 0;
}

static int zephyr_accel_controller_init(const struct device *dev)
{
	struct zephyr_accel_controller_data *data = dev->data;

	data->mode = STEPPER_CTRL_RUN_MODE_HOLD;
	k_sem_init(&data->semaphore, 1, 1);

	k_work_init_delayable(&data->accel_work, zephyr_accel_controller_wq_accel_handler);
	k_work_init_delayable(&data->decel_work_start,
			      zephyr_accel_controller_wq_decel_start_handler);
	k_work_init_delayable(&data->decel_work, zephyr_accel_controller_wq_decel_handler);

	return 0;
}

static int zephyr_accel_controller_wq_init(void)
{
	k_work_queue_init(&zephyr_accel_controller_wq);

	k_work_queue_start(&zephyr_accel_controller_wq, zephyr_accel_controller_work_queue_stack,
			   K_THREAD_STACK_SIZEOF(zephyr_accel_controller_work_queue_stack),
			   CONFIG_ZEPHYR_ACCEL_CONTROLLER_WORK_QUEUE_PRIORITY, NULL);

	return 0;
}

SYS_INIT(zephyr_accel_controller_wq_init, POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY);

static DEVICE_API(stepper_ctrl, zephyr_accel_controller_api) = {
	.move_by = zephyr_accel_controller_move_by,
	.move_to = zephyr_accel_controller_move_to,
	.is_moving = zephyr_accel_controller_is_moving,
	.set_reference_position = zephyr_accel_controller_set_reference_position,
	.get_actual_position = zephyr_accel_controller_get_actual_position,
	.set_event_cb = zephyr_accel_controller_set_event_callback,
	.set_microstep_interval = zephyr_accel_controller_set_microstep_interval,
	.configure_ramp = zephyr_accel_controller_configure_ramp,
	.run = zephyr_accel_controller_run,
	.stop = zephyr_accel_controller_stop,
};

#define ZEPHYR_ACCEL_CONTROLLER_DEFINE(inst)                                                       \
	static const struct zephyr_accel_controller_config zephyr_accel_controller_config_##inst = \
		{                                                                                  \
			.stepper_ctrl =                                                            \
				DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), stepper_ctrl)),        \
			.update_interval_ns = DT_INST_PROP(inst, update_interval),                 \
			.accel_decel_mult = (float)DT_INST_PROP(inst, step_adjustment) / 1000.0f,  \
	};                                                                                         \
	static struct zephyr_accel_controller_data zephyr_accel_controller_data_##inst = {         \
		.dev = DEVICE_DT_INST_GET(inst),                                                   \
		.callback_initialized = false,                                                     \
		.accel_time_microstep_ns = DT_INST_PROP(inst, accel_time_microstep),               \
		.decel_time_microstep_ns = DT_INST_PROP(inst, decel_time_microstep),               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, zephyr_accel_controller_init, NULL,                            \
			      &zephyr_accel_controller_data_##inst,                                \
			      &zephyr_accel_controller_config_##inst, POST_KERNEL,                 \
			      CONFIG_STEPPER_INIT_PRIORITY, &zephyr_accel_controller_api);

DT_INST_FOREACH_STATUS_OKAY(ZEPHYR_ACCEL_CONTROLLER_DEFINE)
