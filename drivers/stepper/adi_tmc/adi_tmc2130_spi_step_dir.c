/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Navimatix GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adi_tmc_reg.h"
#define DT_DRV_COMPAT adi_tmc2130_spi_step_dir

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include "zephyr/devicetree.h"
#include "zephyr/device.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/stepper.h>
#include "../step_dir/step_dir_stepper_common_accel.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmc2130, CONFIG_STEPPER_LOG_LEVEL);

struct tmc2130_spi_sd_config {
	struct step_dir_stepper_common_accel_config common;
	struct gpio_dt_spec en_pin;
	struct spi_dt_spec spi;
};

struct tmc2130_spi_sd_data {
	const struct step_dir_stepper_common_accel_data common;
	bool enabled;
	enum stepper_micro_step_resolution ustep_res;
	uint8_t irun;
	uint8_t ihold;
	uint8_t iholddelay;
	uint8_t tpowerdown;
};

static uint8_t tmc2130_spi_sd_ms_res_translator(int32_t ms_res)
{
	uint8_t reg_value;

	switch (ms_res) {
	case STEPPER_MICRO_STEP_1:
		reg_value = 0x08;
		break;
	case STEPPER_MICRO_STEP_2:
		reg_value = 0x07;
		break;
	case STEPPER_MICRO_STEP_4:
		reg_value = 0x06;
		break;
	case STEPPER_MICRO_STEP_8:
		reg_value = 0x05;
		break;
	case STEPPER_MICRO_STEP_16:
		reg_value = 0x04;
		break;
	case STEPPER_MICRO_STEP_32:
		reg_value = 0x03;
		break;
	case STEPPER_MICRO_STEP_64:
		reg_value = 0x02;
		break;
	case STEPPER_MICRO_STEP_128:
		reg_value = 0x01;
		break;
	case STEPPER_MICRO_STEP_256:
		reg_value = 0x00;
		break;
	default:
		reg_value = 0xFF;
	};

	return reg_value;
}

static int tmc2130_spi_sd_stepper_enable(const struct device *dev, bool enable)
{
	const struct tmc2130_spi_sd_config *config = dev->config;
	struct tmc2130_spi_sd_data *data = dev->data;
	int ret;

	/* Check availability of sleep and enable pins, as these might be hardwired. */
	if (config->en_pin.port == NULL) {
		LOG_ERR("%s: Failed to enable/disable device, enable pin is not "
			"available. The device is always on.",
			dev->name);
		return -ENOTSUP;
	}

	if (enable) {
		ret = gpio_pin_set_dt(&config->en_pin, 1);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set en_pin (error: %d)", dev->name, ret);
			return ret;
		}
	} else {
		ret = gpio_pin_set_dt(&config->en_pin, 0);
		if (ret != 0) {
			LOG_ERR("%s: Failed to set en_pin (error: %d)", dev->name, ret);
			return ret;
		}
	}
	data->enabled = enable;

	if (!enable) {
		step_dir_stepper_common_accel_stop(dev);
		gpio_pin_set_dt(&config->common.step_pin, 0);
	}

	return 0;
}

static int
tmc2130_spi_sd_stepper_set_micro_step_res(const struct device *dev,
					  enum stepper_micro_step_resolution micro_step_res)
{
	const struct tmc2130_spi_sd_config *config = dev->config;
	struct tmc2130_spi_sd_data *data = dev->data;
	int ret;

	uint8_t ustep_res_reg_value = tmc2130_spi_sd_ms_res_translator(micro_step_res);

	if (ustep_res_reg_value == 0xFF) {
		return -EINVAL;
	}
	uint8_t tx_buffer1[5] = {TMC2130_CHOPCONF, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx_buffer[5];

	struct spi_buf spi_buffer_tx = {
		.buf = &tx_buffer1,
		.len = sizeof(tx_buffer1),
	};
	struct spi_buf_set spi_buffer_array_tx = {
		.buffers = &spi_buffer_tx,
		.count = 1U,
	};

	struct spi_buf spi_buffer_rx = {
		.buf = &rx_buffer,
		.len = sizeof(rx_buffer),
	};
	struct spi_buf_set spi_buffer_array_rx = {
		.buffers = &spi_buffer_rx,
		.count = 1U,
	};

	/* Double read, as read value of first command is transmitted on second command. */
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to read register 0x%x (error code: %d)", dev->name,
			tx_buffer1[0], ret);
		return ret;
	}
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to read register 0x%x (error code: %d)", dev->name,
			tx_buffer1[0], ret);
		return ret;
	}

	/* Copy current configuration. */
	tx_buffer1[2] = rx_buffer[2];
	tx_buffer1[3] = rx_buffer[3];
	tx_buffer1[4] = rx_buffer[4];

	/* Set new ustep resolution and enable write. */
	tx_buffer1[1] = ustep_res_reg_value;
	tx_buffer1[0] += TMC2130_WRITE_BIT;

	/* Write new ustep resolution. */
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
			tx_buffer1[0], ret);
		return ret;
	}

	data->ustep_res = micro_step_res;

	return 0;
}

static int
tmc2130_spi_sd_stepper_get_micro_step_res(const struct device *dev,
					  enum stepper_micro_step_resolution *micro_step_res)
{
	struct tmc2130_spi_sd_data *data = dev->data;
	*micro_step_res = data->ustep_res;
	// TODO: maybe read from register?
	return 0;
}

static int tmc2130_spi_sd_stepper_move_to(const struct device *dev, int32_t target)
{
	struct tmc2130_spi_sd_data *data = dev->data;

	if (!data->enabled) {
		LOG_ERR("Failed to move to target position, device is not enabled");
		return -ECANCELED;
	}

	return step_dir_stepper_common_accel_move_to(dev, target);
}

static int tmc2130_spi_sd_stepper_move_by(const struct device *dev, int32_t steps)
{
	struct tmc2130_spi_sd_data *data = dev->data;

	if (!data->enabled) {
		LOG_ERR("Failed to move by delta, device is not enabled");
		return -ECANCELED;
	}

	return step_dir_stepper_common_accel_move_by(dev, steps);
}

static int tmc2130_spi_sd_stepper_run(const struct device *dev, enum stepper_direction direction)
{
	struct tmc2130_spi_sd_data *data = dev->data;

	if (!data->enabled) {
		LOG_ERR("Failed to run stepper, device is not enabled");
		return -ECANCELED;
	}

	return step_dir_stepper_common_accel_run(dev, direction);
}

static int tmc2130_spi_sd_stepper_init(const struct device *dev)
{
	const struct tmc2130_spi_sd_config *config = dev->config;
	struct tmc2130_spi_sd_data *data = dev->data;

	uint8_t ustep_res_reg_value = tmc2130_spi_sd_ms_res_translator(data->ustep_res);

	// TODO: Replace some values with DT entries, and maybe move some parameters to config.

	uint8_t tx_buffer1[5] = {
		TMC2130_WRITE_BIT + TMC2130_CHOPCONF, ustep_res_reg_value, 0x01, 0x00,
		0xC3}; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle), MRES=8
	uint8_t tx_buffer2[5] = {
		TMC2130_WRITE_BIT + TMC2130_IHOLD_IRUN, 0x00, data->iholddelay, data->irun,
		data->ihold}; // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
	uint8_t tx_buffer3[5] = {
		TMC2130_WRITE_BIT + TMC2130_TPOWERDOWN, 0x00, 0x00, 0x00,
		data->tpowerdown}; // TPOWERDOWN=10: Delay before power down in stand still
	uint8_t tx_buffer4[5] = {TMC2130_WRITE_BIT + TMC2130_GCONF, 0x00, 0x00, 0x00,
				 0x04}; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
	uint8_t tx_buffer5[5] = {
		TMC2130_WRITE_BIT + TMC2130_TPWMTHRS, 0x00, 0x00, 0x01,
		0xF4}; // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
	uint8_t tx_buffer6[5] = {
		TMC2130_WRITE_BIT + TMC2130_PWMCONF, 0x00, 0x04, 0x01,
		0xC8}; // PWMCONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1

	uint8_t rx_buffer[5];

	struct spi_buf spi_buffer_tx = {
		.buf = &tx_buffer1,
		.len = sizeof(tx_buffer1),
	};
	struct spi_buf_set spi_buffer_array_tx = {
		.buffers = &spi_buffer_tx,
		.count = 1U,
	};

	struct spi_buf spi_buffer_rx = {
		.buf = &rx_buffer,
		.len = sizeof(rx_buffer),
	};
	struct spi_buf_set spi_buffer_array_rx = {
		.buffers = &spi_buffer_rx,
		.count = 1U,
	};

	int ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
			tx_buffer1[0], ret);
		return ret;
	}
	spi_buffer_tx.buf = &tx_buffer2;
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
			tx_buffer2[0], ret);
		return ret;
	}
	spi_buffer_tx.buf = &tx_buffer3;
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
			tx_buffer3[0], ret);
		return ret;
	}
	spi_buffer_tx.buf = &tx_buffer4;
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
			tx_buffer4[0], ret);
		return ret;
	}
	spi_buffer_tx.buf = &tx_buffer5;
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
			tx_buffer5[0], ret);
		return ret;
	}
	spi_buffer_tx.buf = &tx_buffer6;
	ret = spi_transceive_dt(&config->spi, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (ret != 0) {
		LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
			tx_buffer6[0], ret);
		return ret;
	}

	/* Configure enable pin if it is available */
	if (config->en_pin.port != NULL) {
		ret = gpio_pin_configure_dt(&config->en_pin, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("%s: Failed to configure en_pin (error: %d)", dev->name, ret);
			return ret;
		}
		// data->pin_states.en = 0U;
	} else {
		data->enabled = true;
	}

	ret = step_dir_stepper_common_accel_init(dev);
	if (ret != 0) {
		LOG_ERR("%s: Failed to initialize common step direction stepper (error: %d)",
			dev->name, ret);
		return ret;
	}

	return 0;
}

static DEVICE_API(stepper, tmc2130_spi_sd_stepper_api) = {
	.enable = tmc2130_spi_sd_stepper_enable,
	.move_by = tmc2130_spi_sd_stepper_move_by,
	.is_moving = step_dir_stepper_common_accel_is_moving,
	.set_reference_position = step_dir_stepper_common_accel_set_reference_position,
	.get_actual_position = step_dir_stepper_common_accel_get_actual_position,
	.move_to = tmc2130_spi_sd_stepper_move_to,
	.set_microstep_interval = step_dir_stepper_common_accel_set_microstep_interval,
	.run = tmc2130_spi_sd_stepper_run,
	.set_event_callback = step_dir_stepper_common_accel_set_event_callback,
	.set_micro_step_res = tmc2130_spi_sd_stepper_set_micro_step_res,
	.get_micro_step_res = tmc2130_spi_sd_stepper_get_micro_step_res,
};

#define TMC2130_SPI_SD_STEPPER_DEVICE(inst)                                                        \
	static const struct tmc2130_spi_sd_config tmc2130_spi_sd_config_##inst = {                 \
		.common = STEP_DIR_STEPPER_DT_INST_COMMON_ACCEL_CONFIG_INIT(inst),                 \
		.en_pin = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                           \
		.spi = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |               \
					     SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8)),     \
					    0),                                                    \
	};                                                                                         \
	static struct tmc2130_spi_sd_data tmc2130_spi_sd_data_##inst = {                           \
		.common = STEP_DIR_STEPPER_DT_INST_COMMON_ACCEL_DATA_INIT(inst),                   \
		.ustep_res = DT_INST_PROP(inst, micro_step_res),                                   \
		.irun = DT_INST_PROP(inst, irun),                                                  \
		.ihold = DT_INST_PROP(inst, ihold),                                                \
		.iholddelay = DT_INST_PROP(inst, iholddelay),                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tmc2130_spi_sd_stepper_init, NULL,                             \
			      &tmc2130_spi_sd_data_##inst, &tmc2130_spi_sd_config_##inst,          \
			      POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,                           \
			      &tmc2130_spi_sd_stepper_api);

DT_INST_FOREACH_STATUS_OKAY(TMC2130_SPI_SD_STEPPER_DEVICE)