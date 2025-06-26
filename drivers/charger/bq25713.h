/*
 * Copyright 2025 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CHARGER_BQ25713_H_
#define ZEPHYR_DRIVERS_CHARGER_BQ25713_H_

#include "zephyr/drivers/smbus.h"
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define DT_DRV_COMPAT ti_bq25713

#define BQ25713_REG_CO0             0
#define BQ25713_REG_CC              1
#define BQ25713_REG_CS              2
#define BQ25713_REG_CV              3
#define BQ25713_REG_IIN_HOST        4
#define BQ25713_REG_IIN_DPM         5
#define BQ25713_REG_MIN_SYS_VOLTAGE 6
#define BQ25713_REG_VIN             7
#define BQ25713_REG_ID              8

#define BQ25713_REG_UPPER_MASK  GENMASK(15, 8)
#define BQ25713_REG_UPPER_SHIFT 8

enum bq25713_reg_area {
	/* Access complete register */
	BQ25713_REG_COMPLETE,
	/* Access lower byte of register */
	BQ25713_REG_LOW,
	/* Access upper byte of register */
	BQ25713_REG_HIGH
};

/* Charge Option 0 */
#define BQ25713_REG_CO0_LOW          0x00
#define BQ25713_REG_CO0_INHIBIT      0x01
#define BQ25713_REG_CO0_INHIBIT_MASK BIT(0)

/* Charge Current */
#define BQ25713_REG_CC_LOW                    0x02
#define BQ25713_REG_CC_CHARGE_CURRENT_MASK    GENMASK(12, 6)
#define BQ25713_REG_CC_CHARGE_CURRENT_STEP_UA 64000
#define BQ25713_REG_CC_CHARGE_CURRENT_MIN_UA  0
#define BQ25713_REG_CC_CHARGE_CURRENT_MAX_UA  8128000

/* Charger Status */
#define BQ25713_REG_CS_LOW             0x20
#define BQ25713_REG_CS_AC_STAT         0x01
#define BQ25713_REG_CS_AC_STAT_MASK    BIT(15)
#define BQ25713_REG_CS_PRE_FAST_CHARGE GENMASK(10, 9)
#define BQ25713_REG_CS_PRECHARGE       BIT(1)
#define BQ25713_REG_CS_FASTCHARGE      BIT(2)

/* Max Charge Voltage */
#define BQ25713_REG_CV_LOW                    0x04
#define BQ25713_REG_CV_CHARGE_VOLTAGE_MASK    GENMASK(14, 3)
#define BQ25713_REG_CV_CHARGE_VOLTAGE_STEP_UV 8000
#define BQ25713_REG_CV_CHARGE_VOLTAGE_MIN_UV  1024000
#define BQ25713_REG_CV_CHARGE_VOLTAGE_MAX_UV  19200000

/* Input current set by host IDPM */ //TODO: Check
#define BQ25713_REG_IIN_HOST_LOW    0x0E
#define BQ25713_REG_IIN_HOST_MASK    GENMASK(7, 0)
#define BQ25713_REG_IIN_HOST_STEP_UA 50000
#define BQ25713_REG_IIN_HOST_MIN_UV  BQ25713_REG_IIN_HOST_STEP_UA
#define BQ25713_REG_IIN_HOST_MAX_UV  6400000

/*  Input voltage IDPM */
#define BQ25713_REG_IIN_DPM_LOW     0x24
#define BQ25713_REG_IIN_DPM_MASK    GENMASK(15, 8)
#define BQ25713_REG_IIN_DPM_STEP_UA BQ25713_REG_IIN_HOST_STEP_UA

/* Mininum system voltage */
#define BQ25713_REG_MIN_SYS_VOLTAGE_LOW      0x0C
#define BQ25713_REG_MIN_SYS_VOLTAGE_MASK    GENMASK(5, 0)
#define BQ25713_REG_MIN_SYS_VOLTAGE_STEP_UV 256000
#define BQ25713_REG_MIN_SYS_VOLTAGE_MIN_UV  1024000
#define BQ25713_REG_MIN_SYS_VOLTAGE_MAX_UV  16128000

/*  Input voltage VDPM */
#define BQ25713_REG_VIN_LOW                0x0A
#define BQ25713_REG_VIN_DPM_MASK           GENMASK(13, 6)
#define BQ25713_REG_VIN_DPM_STEP_UV        64000
#define BQ25713_REG_VIN_DPM_OFFSET_UV      3200000
#define BQ25713_REG_VIN_DPM_VOLTAGE_MIN_UV BQ25713_REG_VIN_DPM_OFFSET_UV
#define BQ25713_REG_VIN_DPM_VOLTAGE_MAX_UV 195200000

/* Manufacture ID */
#define BQ25713_REG_ID_LOW       0x2E
#define BQ25713_REG_ID_PN_25713  0x4088
#define BQ25713_REG_ID_PN_25713B 0x408A

#define BQ25713_FACTOR_U_TO_M 1000

#define BQ25713_BUS_I2C   DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#define BQ25713_BUS_SMBUS DT_ANY_INST_ON_BUS_STATUS_OKAY(smbus)

typedef int (*bq25713_write8_fn)(const struct device *dev, uint8_t reg, uint8_t value);
typedef int (*bq25713_read8_fn)(const struct device *dev, uint8_t reg, uint8_t *value);
typedef int (*bq25713_update_fn)(const struct device *dev, uint8_t reg, uint16_t mask,
				 uint16_t value);
typedef int (*bq25713_write_fn)(const struct device *dev, uint8_t reg, uint16_t value);
typedef int (*bq25713_read_fn)(const struct device *dev, uint8_t reg, uint16_t *value);

struct bq25713_bus_io {
	bq25713_update_fn update;
	bq25713_write_fn write;
	bq25713_read_fn read;
};

union bq25713_bus {
#if BQ25713_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
#if BQ25713_BUS_SMBUS
	struct smbus_dt_spec smbus;
#endif
};

struct bq25713_config {
	union bq25713_bus bus;
	const struct bq25713_bus_io *bus_io;
	uint8_t (*reg_lookup)[9];
	uint32_t vsys_min_uv;
	uint32_t ichg_ua;
	uint32_t vreg_uv;
};

#if BQ25713_BUS_I2C
extern const struct bq25713_bus_io bq25713_bus_io_i2c;
extern uint8_t reg_lookup_i2c[9];
#endif

#endif /* ZEPHYR_DRIVERS_CHARGER_BQ25713_H_ */
