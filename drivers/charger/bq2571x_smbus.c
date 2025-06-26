/*
 * Copyright 2025 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "bq2571x.h"
#include <zephyr/logging/log.h>

#if BQ2571X_BUS_SMBUS

LOG_MODULE_REGISTER(ti_bq2571x_smbus, CONFIG_CHARGER_LOG_LEVEL);

static int bq2571x_write_smbus(const struct device *dev, uint8_t reg, uint16_t value)
{
	const struct bq2571x_config *const config = dev->config;

	return smbus_word_data_write(config->bus.smbus.bus, config->bus.smbus.addr, reg, value);
}

static int bq2571x_read_smbus(const struct device *dev, uint8_t reg, uint16_t *value)
{
	const struct bq2571x_config *config = dev->config;

	return smbus_word_data_read(config->bus.smbus.bus, config->bus.smbus.addr, reg, value);
}

static int bq2571x_update_smbus(const struct device *dev, uint8_t reg, uint16_t mask,
				uint16_t value)
{
	const struct bq2571x_config *const config = dev->config;
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

const struct bq2571x_bus_io bq2571x_bus_io_smbus = {
	.update = bq2571x_update_smbus,
	.write = bq2571x_write_smbus,
	.read = bq2571x_read_smbus,
};

/* The smbus communication is used by the BQ25710. */
uint8_t bq2571x_reg_lookup_smbus[9] = {BQ25710_REG_CO0,
			       BQ25710_REG_CC,
			       BQ25710_REG_CS,
			       BQ25710_REG_CV,
			       BQ25710_REG_IIN_HOST,
			       BQ25710_REG_IIN_DPM,
			       BQ25710_REG_MIN_SYS_VOLTAGE,
			       BQ25710_REG_VIN,
			       BQ25710_REG_ID};

#endif /* BQ2571X_BUS_SMBUS */
