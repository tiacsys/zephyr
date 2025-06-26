/*
 * Copyright 2025 Palta Tech, S.A
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BQ25713 Datasheet: https://www.ti.com/lit/ds/symlink/bq25713.pdf
 */

#define DT_DRV_COMPAT ti_bq25713

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/sys/byteorder.h>
#include "bq25713.h"
#include <zephyr/drivers/smbus.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ti_bq25713, CONFIG_CHARGER_LOG_LEVEL);

static int bq25713_set_minimum_system_voltage(const struct device *dev, uint32_t voltage_uv)
{
	const struct bq25713_config *const config = dev->config;

	if (!IN_RANGE(voltage_uv, BQ25713_REG_MIN_SYS_VOLTAGE_MIN_UV,
		      BQ25713_REG_MIN_SYS_VOLTAGE_MAX_UV)) {
		LOG_WRN("minimum system voltage out of range: %umV, "
			"clamping to the nearest limit",
			voltage_uv / BQ25713_FACTOR_U_TO_M);
	}

	uint32_t v;

	voltage_uv = CLAMP(voltage_uv, BQ25713_REG_MIN_SYS_VOLTAGE_MIN_UV,
			   BQ25713_REG_MIN_SYS_VOLTAGE_MAX_UV);
	v = voltage_uv / BQ25713_REG_MIN_SYS_VOLTAGE_STEP_UV;
	v = FIELD_PREP(BQ25713_REG_MIN_SYS_VOLTAGE_MASK, v);
	v = v << BQ25713_REG_UPPER_SHIFT;
	return config->bus_io->update(dev, (*config->reg_lookup)[BQ25713_REG_MIN_SYS_VOLTAGE],
				      BQ25713_REG_UPPER_MASK, v);
}

static int bq25713_set_constant_charge_current(const struct device *dev, uint32_t current_ua)
{
	const struct bq25713_config *const config = dev->config;

	if (current_ua > BQ25713_REG_CC_CHARGE_CURRENT_MAX_UA) {
		LOG_WRN("charging current out of range: %umA, "
			"clamping to the nearest limit",
			current_ua / BQ25713_FACTOR_U_TO_M);
	}
	current_ua = CLAMP(current_ua, BQ25713_REG_CC_CHARGE_CURRENT_MIN_UA,
			   BQ25713_REG_CC_CHARGE_CURRENT_MAX_UA);
	uint32_t v;

	v = current_ua / BQ25713_REG_CC_CHARGE_CURRENT_STEP_UA;
	v = FIELD_PREP(BQ25713_REG_CC_CHARGE_CURRENT_MASK, v);

	return config->bus_io->write(dev, (*config->reg_lookup)[BQ25713_REG_CC], v);
}

static int bq25713_set_constant_charge_voltage(const struct device *dev, uint32_t voltage_uv)
{
	const struct bq25713_config *const config = dev->config;

	if (!IN_RANGE(voltage_uv, BQ25713_REG_CV_CHARGE_VOLTAGE_MIN_UV,
		      BQ25713_REG_CV_CHARGE_VOLTAGE_MAX_UV)) {
		LOG_WRN("charging voltage out of range: %umV, "
			"clamping to the nearest limit",
			voltage_uv / BQ25713_FACTOR_U_TO_M);
	}

	uint32_t v;

	voltage_uv = CLAMP(voltage_uv, BQ25713_REG_CV_CHARGE_VOLTAGE_MIN_UV,
			   BQ25713_REG_CV_CHARGE_VOLTAGE_MAX_UV);
	v = voltage_uv / BQ25713_REG_CV_CHARGE_VOLTAGE_STEP_UV;
	v = FIELD_PREP(BQ25713_REG_CV_CHARGE_VOLTAGE_MASK, v);
	return config->bus_io->write(dev, (*config->reg_lookup)[BQ25713_REG_CV], v);
}

static int bq25713_set_iindpm(const struct device *dev, uint32_t current_ua)
{
	const struct bq25713_config *const config = dev->config;

	if (!IN_RANGE(current_ua, BQ25713_REG_IIN_HOST_MIN_UV, BQ25713_REG_IIN_HOST_MAX_UV)) {
		LOG_WRN("input current regulation out of range: %umA, "
			"clamping to the nearest limit",
			current_ua / BQ25713_FACTOR_U_TO_M);
	}

	uint32_t v;

	current_ua = CLAMP(current_ua, BQ25713_REG_IIN_HOST_MIN_UV, BQ25713_REG_IIN_HOST_MAX_UV);
	v = current_ua / BQ25713_REG_IIN_HOST_STEP_UA;
	v = FIELD_PREP(BQ25713_REG_IIN_HOST_MASK, v);
	v = v << BQ25713_REG_UPPER_SHIFT;
	return config->bus_io->update(dev, (*config->reg_lookup)[BQ25713_REG_IIN_HOST],
				      BQ25713_REG_UPPER_MASK, v);
}

static int bq25713_set_vindpm(const struct device *dev, uint32_t voltage_ua)
{
	const struct bq25713_config *const config = dev->config;

	if (!IN_RANGE(voltage_ua, BQ25713_REG_VIN_DPM_VOLTAGE_MIN_UV,
		      BQ25713_REG_VIN_DPM_VOLTAGE_MAX_UV)) {
		LOG_WRN("input voltage regulation of range: %umV, "
			"clamping to the nearest limit",
			voltage_ua / BQ25713_FACTOR_U_TO_M);
	}

	uint32_t v;

	voltage_ua = CLAMP(voltage_ua, BQ25713_REG_VIN_DPM_VOLTAGE_MIN_UV,
			   BQ25713_REG_VIN_DPM_VOLTAGE_MAX_UV);
	v = (voltage_ua - BQ25713_REG_VIN_DPM_OFFSET_UV) / BQ25713_REG_VIN_DPM_STEP_UV;
	v = FIELD_PREP(BQ25713_REG_VIN_DPM_MASK, v);
	return config->bus_io->write(dev, (*config->reg_lookup)[BQ25713_REG_VIN], v);
}

static int bq25713_get_constant_charge_current(const struct device *dev, uint32_t *current_ua)
{
	const struct bq25713_config *const config = dev->config;
	uint16_t v;
	int ret;

	ret = config->bus_io->read(dev, (*config->reg_lookup)[BQ25713_REG_CC], &v);
	if (ret < 0) {
		return ret;
	}

	v = FIELD_GET(BQ25713_REG_CC_CHARGE_CURRENT_MASK, v);

	*current_ua = v * BQ25713_REG_CC_CHARGE_CURRENT_STEP_UA;

	return 0;
}

static int bq25713_get_constant_charge_voltage(const struct device *dev, uint32_t *voltage_uv)
{
	const struct bq25713_config *const config = dev->config;
	uint16_t value;
	int ret;

	ret = config->bus_io->read(dev, (*config->reg_lookup)[BQ25713_REG_CV], &value);
	if (ret < 0) {
		return ret;
	}
	value = FIELD_GET(BQ25713_REG_CV_CHARGE_VOLTAGE_MASK, value);

	*voltage_uv = value * BQ25713_REG_CV_CHARGE_VOLTAGE_STEP_UV;

	return 0;
}

static int bq25713_get_iindpm(const struct device *dev, uint32_t *current_ua)
{
	const struct bq25713_config *const config = dev->config;
	uint16_t value;
	int ret;

	ret = config->bus_io->read(dev, (*config->reg_lookup)[BQ25713_REG_IIN_DPM], &value);
	if (ret < 0) {
		return ret;
	}
	value = FIELD_GET(BQ25713_REG_IIN_DPM_MASK, value);

	*current_ua = value * BQ25713_REG_IIN_DPM_STEP_UA;

	return 0;
}

static int bq25713_get_vindpm(const struct device *dev, uint32_t *voltage_uv)
{
	const struct bq25713_config *const config = dev->config;
	uint16_t value;
	int ret;

	ret = config->bus_io->read(dev, (*config->reg_lookup)[BQ25713_REG_VIN], &value);
	if (ret < 0) {
		return ret;
	}
	value = FIELD_GET(BQ25713_REG_VIN_DPM_MASK, value);

	*voltage_uv = (value * BQ25713_REG_VIN_DPM_STEP_UV) + BQ25713_REG_VIN_DPM_OFFSET_UV;

	return 0;
}

static int bq25713_get_status(const struct device *dev, enum charger_status *status)
{
	const struct bq25713_config *const config = dev->config;
	uint16_t charge_status;
	int ret;

	ret = config->bus_io->read(dev, (*config->reg_lookup)[BQ25713_REG_CS], &charge_status);
	if (ret < 0) {
		return ret;
	}

	switch (FIELD_GET(BQ25713_REG_CS_PRE_FAST_CHARGE, charge_status)) {
	case BQ25713_REG_CS_FASTCHARGE:
		__fallthrough;
	case BQ25713_REG_CS_PRECHARGE:
		*status = CHARGER_STATUS_CHARGING;
		break;
	default:
		*status = CHARGER_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static int bq25713_get_online(const struct device *dev, enum charger_online *online)
{
	const struct bq25713_config *const config = dev->config;
	uint16_t status;
	int ret;

	ret = config->bus_io->read(dev, (*config->reg_lookup)[BQ25713_REG_CS], &status);
	if (ret < 0) {
		return ret;
	}

	status = FIELD_GET(BQ25713_REG_CS_AC_STAT_MASK, status);
	if (status == BQ25713_REG_CS_AC_STAT) {
		*online = CHARGER_ONLINE_FIXED;
	} else {
		*online = CHARGER_ONLINE_OFFLINE;
	}

	return 0;
}

static int bq25713_charger_get_charge_type(const struct device *dev,
					   enum charger_charge_type *charge_type)
{
	*charge_type = CHARGER_CHARGE_TYPE_UNKNOWN;
	return 0;
}

static int bq25713_get_prop(const struct device *dev, charger_prop_t prop,
			    union charger_propval *value)
{
	switch (prop) {
	case CHARGER_PROP_ONLINE:
		return bq25713_get_online(dev, &value->online);
	case CHARGER_PROP_CHARGE_TYPE:
		return bq25713_charger_get_charge_type(dev, &value->charge_type);
	case CHARGER_PROP_STATUS:
		return bq25713_get_status(dev, &value->status);
	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
		return bq25713_get_constant_charge_current(dev, &value->const_charge_current_ua);
	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
		return bq25713_get_constant_charge_voltage(dev, &value->const_charge_voltage_uv);
	case CHARGER_PROP_INPUT_REGULATION_CURRENT_UA:
		return bq25713_get_iindpm(dev, &value->input_current_regulation_current_ua);
	case CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV:
		return bq25713_get_vindpm(dev, &value->input_voltage_regulation_voltage_uv);
	default:
		return -ENOTSUP;
	}
}

static int bq25713_set_prop(const struct device *dev, charger_prop_t prop,
			    const union charger_propval *value)
{
	switch (prop) {
	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
		return bq25713_set_constant_charge_current(dev, value->const_charge_current_ua);
	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
		return bq25713_set_constant_charge_voltage(dev, value->const_charge_voltage_uv);
	case CHARGER_PROP_INPUT_REGULATION_CURRENT_UA:
		return bq25713_set_iindpm(dev, value->input_current_regulation_current_ua);
	case CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV:
		return bq25713_set_vindpm(dev, value->input_voltage_regulation_voltage_uv);
	default:
		return -ENOTSUP;
	}
}

static int bq25713_charge_enable(const struct device *dev, const bool enable)
{
	const struct bq25713_config *const config = dev->config;
	uint8_t value = enable ? 0 : BQ25713_REG_CO0_INHIBIT;

	return config->bus_io->update(dev, (*config->reg_lookup)[BQ25713_REG_CO0],
				      BQ25713_REG_CO0_INHIBIT_MASK, value);
}

static int bq25713_set_config(const struct device *dev)
{
	const struct bq25713_config *const config = dev->config;
	union charger_propval value;
	int ret;

	value.const_charge_current_ua = config->ichg_ua;

	ret = bq25713_set_constant_charge_current(dev, value.const_charge_current_ua);
	if (ret < 0) {
		return ret;
	}

	value.const_charge_voltage_uv = config->vreg_uv;

	ret = bq25713_set_constant_charge_voltage(dev, value.const_charge_voltage_uv);
	if (ret < 0) {
		return ret;
	}

	return bq25713_set_minimum_system_voltage(dev, config->vsys_min_uv);
}

static int bq25713_init(const struct device *dev)
{
	const struct bq25713_config *const config = dev->config;
	uint16_t value;
	int ret;

	// TODO: Check if SMBus gan get  manufacturer and device id in one go like this
	ret = config->bus_io->read(dev, (*config->reg_lookup)[BQ25713_REG_ID], &value);
	if (ret < 0) {
		LOG_ERR("Unable to read Device ID Register 0x%02x",
			(*config->reg_lookup)[BQ25713_REG_ID]);
		return ret;
	}

	switch (value) {
	case BQ25713_REG_ID_PN_25710:
		__fallthrough;
	case BQ25713_REG_ID_PN_25713:
		__fallthrough;
	case BQ25713_REG_ID_PN_25713B:
		break;
	default:
		LOG_ERR("Error unknown model: 0x%04x\n", value);
		return -ENODEV;
	}

	return bq25713_set_config(dev);
}

static DEVICE_API(charger, bq7513_driver_api) = {
	.get_property = bq25713_get_prop,
	.set_property = bq25713_set_prop,
	.charge_enable = bq25713_charge_enable,
};

/* Initializes a struct bq25713_config for an instance on a SMBus bus. */
#define BQ25713_CONFIG_SMBUS(inst)                                                                 \
	.bus.smbus = SMBUS_DT_SPEC_INST_GET(inst), .bus_io = &bq25713_bus_io_smbus,                \
	.reg_lookup = &reg_lookup_smbus,

/* Initializes a struct bq25713_config for an instance on an I2C bus. */
#define BQ25713_CONFIG_I2C(inst)                                                                   \
	.bus.i2c = I2C_DT_SPEC_INST_GET(inst), .bus_io = &bq25713_bus_io_i2c,                      \
	.reg_lookup = &reg_lookup_i2c,

#define BQ25713_INIT(inst)                                                                         \
                                                                                                   \
	static const struct bq25713_config bq25713_config_##inst = {                               \
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c),                                             \
			    (BQ25713_CONFIG_I2C(inst)),                                            \
			    (BQ25713_CONFIG_SMBUS(inst))) .ichg_ua =                                  \
				  DT_INST_PROP(inst, constant_charge_current_max_microamp),        \
			 .vreg_uv = DT_INST_PROP(inst, constant_charge_voltage_max_microvolt),     \
			 .vsys_min_uv =                                                            \
				 DT_INST_PROP(inst, system_voltage_min_threshold_microvolt),       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, bq25713_init, NULL, NULL, &bq25713_config_##inst, POST_KERNEL, \
			      CONFIG_CHARGER_INIT_PRIORITY, &bq7513_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ25713_INIT)
