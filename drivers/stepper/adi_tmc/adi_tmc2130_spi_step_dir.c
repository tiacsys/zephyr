/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Navimatix GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc2130_spi_step_dir

#include "adi_tmc_reg.h"
#include "adi_tmc_spi.h"
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

#define REG_INIT_NUMBER 6

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

	if (!enable) {
		ret = step_dir_stepper_common_accel_stop(dev);
		if (ret != 0) {
			LOG_ERR("%s: Failed to stop stepper (error: %d)", dev->name, ret);
			return ret;
		}
		if (!config->common.dual_edge) {
			ret = gpio_pin_set_dt(&config->common.step_pin, 0);
			if (ret != 0) {
				LOG_ERR("%s: Failed to set step_pin (error: %d)", dev->name, ret);
				return ret;
			}
		}
	}

	data->enabled = enable;

	return 0;
}

static int
tmc2130_spi_sd_stepper_set_micro_step_res(const struct device *dev,
					  enum stepper_micro_step_resolution micro_step_res)
{
	const struct tmc2130_spi_sd_config *config = dev->config;
	struct tmc2130_spi_sd_data *data = dev->data;
	int ret;
	uint32_t read_data;
	uint32_t write_data;

	uint32_t ustep_res_reg_value = tmc2130_spi_sd_ms_res_translator(micro_step_res);

	if (ustep_res_reg_value == 0xFF) {
		return -EINVAL;
	}
	/* Add dual edge config if needed */
	if (config->common.dual_edge) {
		ustep_res_reg_value += TMC2130_DOUBLE_EDGE_OFFSET;
	}

	ret = tmc_spi_read_register(&config->spi, 0, TMC2130_CHOPCONF, &read_data);
	if (ret != 0) {
		LOG_ERR("%s: Failed to read register 0x%x (error code: %d)", dev->name,
			TMC2130_CHOPCONF, ret);
		return ret;
	}
	write_data = read_data + (ustep_res_reg_value << 24);

	ret = tmc_spi_write_register(&config->spi, TMC2130_WRITE_BIT, TMC2130_CHOPCONF, write_data);
	if (ret != 0) {
		LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
			TMC2130_CHOPCONF, ret);
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
	int ret;

	uint8_t ustep_res_reg_value = tmc2130_spi_sd_ms_res_translator(data->ustep_res);
	/* Dual Edge is configured in the same byte as microstep resolution */
	if (config->common.dual_edge) {
		ustep_res_reg_value += TMC2130_DOUBLE_EDGE_OFFSET;
	}

	// TODO: Replace some values with DT entries, and maybe move some parameters to config.

	uint32_t data_1 = (ustep_res_reg_value << 24) + (0x01 << 16) + (0x00 << 8) + 0xC3;
	uint32_t data_2 = (0x00 << 24) + (data->iholddelay << 16) + (data->irun << 8) + data->ihold;
	uint32_t data_3 = (0x00 << 24) + (0x00 << 16) + (0x00 << 8) + data->tpowerdown;
	uint32_t data_4 = (0x00 << 24) + (0x00 << 16) + (0x00 << 8) + 0x04;
	uint32_t data_5 = (0x00 << 24) + (0x00 << 16) + (0x01 << 8) + 0xF4;
	uint32_t data_6 = (0x00 << 24) + (0x04 << 16) + (0x01 << 8) + 0xC8;

	uint8_t reg_addr[REG_INIT_NUMBER] = {TMC2130_CHOPCONF,   TMC2130_IHOLD_IRUN,
					     TMC2130_TPOWERDOWN, TMC2130_GCONF,
					     TMC2130_TPWMTHRS,   TMC2130_PWMCONF};
	uint32_t reg_data[REG_INIT_NUMBER] = {data_1, data_2, data_3, data_4, data_5, data_6};

	for (int i = 0; i < REG_INIT_NUMBER; i++) {
		ret = tmc_spi_write_register(&config->spi, TMC2130_WRITE_BIT, reg_addr[i],
					     reg_data[i]);
		if (ret != 0) {
			LOG_ERR("%s: Failed to write register 0x%x (error code: %d)", dev->name,
				reg_addr[i], ret);
			return ret;
		}
	}

	/* Configure enable pin if it is available */
	if (config->en_pin.port != NULL) {
		ret = gpio_pin_configure_dt(&config->en_pin, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("%s: Failed to configure en_pin (error: %d)", dev->name, ret);
			return ret;
		}
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