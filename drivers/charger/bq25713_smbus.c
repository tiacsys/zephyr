/*
 * Copyright 2025 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "bq25713.h"
#include <zephyr/logging/log.h>

#if BQ25713_BUS_SMBUS

LOG_MODULE_REGISTER(ti_bq25710_smbus, CONFIG_CHARGER_LOG_LEVEL);

static int bq25713_write_smbus(const struct device *dev, uint8_t reg, uint16_t value)
{
	const struct bq25713_config *const config = dev->config;

	return smbus_word_data_write(config->bus.smbus.bus, config->bus.smbus.addr, reg, value);
}

static int bq25713_read_smbus(const struct device *dev, uint8_t reg, uint16_t *value)
{
	const struct bq25713_config *config = dev->config;

	return smbus_word_data_read(config->bus.smbus.bus, config->bus.smbus.addr, reg, value);
}

static int bq25713_update_smbus(const struct device *dev, uint8_t reg, uint16_t mask,
				uint16_t value)
{
	const struct bq25713_config *const config = dev->config;
	uint16_t old_data;
	uint16_t new_data;
	int ret;

	ret = smbus_word_data_read(config->bus.smbus.bus, config->bus.smbus.addr, reg, &old_data);
	if (ret < 0) {
		LOG_ERR("Unable to read register 0x%02x", reg);
		return ret;
	}
	new_data = (old_data & mask) | (value & mask);

	return smbus_word_data_write(config->bus.smbus.bus, config->bus.smbus.addr, reg, new_data);
}

const struct bq25713_bus_io bq25713_bus_io_smbus = {
	.update = bq25713_update_smbus,
	.write = bq25713_write_smbus,
	.read = bq25713_read_smbus,
};

uint8_t reg_lookup_smbus[9] = {BQ25710_REG_CO0,
			       BQ25710_REG_CC,
			       BQ25710_REG_CS,
			       BQ25710_REG_CV,
			       BQ25710_REG_IIN_HOST,
			       BQ25710_REG_IIN_DPM,
			       BQ25710_REG_MIN_SYS_VOLTAGE,
			       BQ25710_REG_VIN,
			       BQ25710_REG_ID};

#endif /* BQ25713_BUS_SMBUS */
