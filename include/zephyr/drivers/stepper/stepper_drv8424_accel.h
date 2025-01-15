/**
 * @file drivers/stepper/stepper_drv8424_accel.h
 *
 * @brief Public API for DRV8424 Stepper Controller with Acceleration Specific Functions
 *
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Navimatix GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/drivers/stepper.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief After microstep setter fails, attempt to recover into previous state.
 *
 * @param dev Pointer to the stepper motor controller instance
 *
 * @retval 0 Success
 * @retval <0 Error code dependent on the gpio controller of the microstep pins
 */
int drv8424_accel_microstep_recovery(const struct device *dev);

#ifdef __cplusplus
}
#endif
