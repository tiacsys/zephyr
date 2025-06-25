/*
 * Copyright 2025 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "bq25713.h"
#include "zephyr/sys/byteorder.h"
#include <zephyr/logging/log.h>

#if BQ25713_BUS_I2C

LOG_MODULE_REGISTER(ti_bq25713_i2c, CONFIG_CHARGER_LOG_LEVEL);

static int bq25713_write16_i2c(const struct device *dev, uint8_t reg, uint16_t value)
{
	const struct bq25713_config *const config = dev->config;
	uint8_t buf[3];

	buf[0] = reg;
	sys_put_le16(value, &buf[1]);

	return i2c_write_dt(&config->bus.i2c, buf, sizeof(buf));
}

static int bq25713_read16_i2c(const struct device *dev, uint8_t reg, uint16_t *value)
{
	const struct bq25713_config *config = dev->config;
	uint8_t i2c_data[2];
	int ret;

	ret = i2c_burst_read_dt(&config->bus.i2c, reg, i2c_data, sizeof(i2c_data));
	if (ret < 0) {
		LOG_ERR("Unable to read register");
		return ret;
	}

	*value = sys_get_be16(i2c_data);

	return 0;
}

static int bq25713_update16_i2c(const struct device *dev, uint8_t reg, uint16_t mask,
				uint16_t value)
{
	const struct bq25713_config *const config = dev->config;
	uint8_t i2c_data[2];
	uint8_t buf[3];
	uint16_t old_data;
	uint16_t new_data;
	int ret;

	// TODO: Maybe just call read16 and write16 and modify value in between

	ret = i2c_burst_read_dt(&config->bus.i2c, reg, i2c_data, sizeof(i2c_data));
	if (ret < 0) {
		LOG_ERR("Unable to update register");
		return ret;
	}
	old_data = sys_get_be16(i2c_data);
	new_data = (old_data & mask) | (value & mask);

	buf[0] = reg;
	sys_put_le16(value, &buf[1]);

	return i2c_write_dt(&config->bus.i2c, buf, sizeof(buf));
}

const struct bq25713_bus_io bq25713_bus_io_i2c = {
	.update = bq25713_update16_i2c,
	.write = bq25713_write16_i2c,
	.read = bq25713_read16_i2c,
};

#endif /* BQ25713_BUS_I2C */
