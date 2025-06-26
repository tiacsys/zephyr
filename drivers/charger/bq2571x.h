/*
 * Copyright 2025 Palta Tech, S.A
 * Copyright 2025 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CHARGER_BQ2571X_H_
#define ZEPHYR_DRIVERS_CHARGER_BQ2571X_H_

#include "zephyr/drivers/smbus.h"
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define DT_DRV_COMPAT ti_bq2571x

/* Indices for the register address lookup table. */
#define BQ2571X_REG_CO0             0
#define BQ2571X_REG_CC              1
#define BQ2571X_REG_CS              2
#define BQ2571X_REG_CV              3
#define BQ2571X_REG_IIN_HOST        4
#define BQ2571X_REG_IIN_DPM         5
#define BQ2571X_REG_MIN_SYS_VOLTAGE 6
#define BQ2571X_REG_VIN             7
#define BQ2571X_REG_ID              8

/* BQ25713/BQ25713B Register addresses */
#define BQ25713_REG_CO0_LOW             0x00
#define BQ25713_REG_CC_LOW              0x02
#define BQ25713_REG_CS_LOW              0x20
#define BQ25713_REG_CV_LOW              0x04
#define BQ25713_REG_IIN_HOST_LOW        0x0E
#define BQ25713_REG_IIN_DPM_LOW         0x24
#define BQ25713_REG_MIN_SYS_VOLTAGE_LOW 0x0C
#define BQ25713_REG_VIN_LOW             0x0A
#define BQ25713_REG_ID_LOW              0x2E

/* BQ25710 Register addresses */
#define BQ25710_REG_CO0             0x12
#define BQ25710_REG_CC              0x14
#define BQ25710_REG_CS              0x20
#define BQ25710_REG_CV              0x15
#define BQ25710_REG_IIN_HOST        0x3F
#define BQ25710_REG_IIN_DPM         0x22
#define BQ25710_REG_MIN_SYS_VOLTAGE 0x3E
#define BQ25710_REG_VIN             0x3D
#define BQ25710_REG_ID              0xFF

/* Helper macros for reading/writing only upper bytes of registers. */
#define BQ2571X_REG_UPPER_MASK  GENMASK(15, 8)
#define BQ2571X_REG_UPPER_SHIFT 8

/* Charge Option 0 */
#define BQ2571X_REG_CO0_INHIBIT      0x01
#define BQ2571X_REG_CO0_INHIBIT_MASK BIT(0)

/* Charge Current */
#define BQ2571X_REG_CC_CHARGE_CURRENT_MASK    GENMASK(12, 6)
#define BQ2571X_REG_CC_CHARGE_CURRENT_STEP_UA 64000
#define BQ2571X_REG_CC_CHARGE_CURRENT_MIN_UA  0
#define BQ2571X_REG_CC_CHARGE_CURRENT_MAX_UA  8128000

/* Charger Status */
#define BQ2571X_REG_CS_AC_STAT         0x01
#define BQ2571X_REG_CS_AC_STAT_MASK    BIT(15)
#define BQ2571X_REG_CS_PRE_FAST_CHARGE GENMASK(10, 9)
#define BQ2571X_REG_CS_PRECHARGE       BIT(1)
#define BQ2571X_REG_CS_FASTCHARGE      BIT(2)

/* Max Charge Voltage */
#define BQ2571X_REG_CV_CHARGE_VOLTAGE_MASK    GENMASK(14, 3)
#define BQ2571X_REG_CV_CHARGE_VOLTAGE_STEP_UV 8000
#define BQ2571X_REG_CV_CHARGE_VOLTAGE_MIN_UV  1024000
#define BQ2571X_REG_CV_CHARGE_VOLTAGE_MAX_UV  19200000

/* Input current set by host IDPM */
#define BQ2571X_REG_IIN_HOST_MASK    GENMASK(7, 0)
#define BQ2571X_REG_IIN_HOST_STEP_UA 50000
#define BQ2571X_REG_IIN_HOST_MIN_UV  BQ2571X_REG_IIN_HOST_STEP_UA
#define BQ2571X_REG_IIN_HOST_MAX_UV  6400000

/*  Input current IDPM */
#define BQ2571X_REG_IIN_DPM_MASK    GENMASK(15, 8)
#define BQ2571X_REG_IIN_DPM_STEP_UA BQ2571X_REG_IIN_HOST_STEP_UA

/* Mininum system voltage */
#define BQ2571X_REG_MIN_SYS_VOLTAGE_MASK    GENMASK(5, 0)
#define BQ2571X_REG_MIN_SYS_VOLTAGE_STEP_UV 256000
#define BQ2571X_REG_MIN_SYS_VOLTAGE_MIN_UV  1024000
#define BQ2571X_REG_MIN_SYS_VOLTAGE_MAX_UV  16128000

/*  Input voltage VDPM */
#define BQ2571X_REG_VIN_DPM_MASK           GENMASK(13, 6)
#define BQ2571X_REG_VIN_DPM_STEP_UV        64000
#define BQ2571X_REG_VIN_DPM_OFFSET_UV      3200000
#define BQ2571X_REG_VIN_DPM_VOLTAGE_MIN_UV BQ2571X_REG_VIN_DPM_OFFSET_UV
#define BQ2571X_REG_VIN_DPM_VOLTAGE_MAX_UV 195200000

/* Device ID */
#define BQ2571X_REG_ID_PN_25713  0x4088
#define BQ2571X_REG_ID_PN_25713B 0x408A
// #define BQ25713_REG_ID_PN_25710  0x4089
#define BQ2571X_REG_ID_PN_25710  0x0089

#define BQ2571X_FACTOR_U_TO_M 1000

#define BQ2571X_BUS_I2C   DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#define BQ2571X_BUS_SMBUS DT_ANY_INST_ON_BUS_STATUS_OKAY(smbus)

typedef int (*bq2571x_update_fn)(const struct device *dev, uint8_t reg, uint16_t mask,
				 uint16_t value);
typedef int (*bq2571x_write_fn)(const struct device *dev, uint8_t reg, uint16_t value);
typedef int (*bq2571x_read_fn)(const struct device *dev, uint8_t reg, uint16_t *value);

struct bq2571x_bus_io {
	bq2571x_update_fn update;
	bq2571x_write_fn write;
	bq2571x_read_fn read;
};

union bq2571x_bus {
#if BQ2571X_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
#if BQ2571X_BUS_SMBUS
	struct smbus_dt_spec smbus;
#endif
};

struct bq2571x_config {
	union bq2571x_bus bus;
	const struct bq2571x_bus_io *bus_io;
	uint8_t (*reg_lookup)[9];
	uint32_t vsys_min_uv;
	uint32_t ichg_ua;
	uint32_t vreg_uv;
};

#if BQ2571X_BUS_I2C
extern const struct bq2571x_bus_io bq2571x_bus_io_i2c;
extern uint8_t bq2571x_reg_lookup_i2c[9];
#endif
#if BQ2571X_BUS_SMBUS
extern const struct bq2571x_bus_io bq2571x_bus_io_smbus;
extern uint8_t bq2571x_reg_lookup_smbus[9];
#endif

#endif /* ZEPHYR_DRIVERS_CHARGER_BQ2571X_H_ */
