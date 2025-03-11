/*
 * Copyright (c) 2021-2024 TiaC Systems
 * Copyright (c) 2019-2021 Li-Pro.Net
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include "stm32f7xx_ll_tim.h"
#include "zephyr/devicetree.h"
#include "zephyr/device.h"
#include "zephyr/drivers/pwm.h"
#include "zephyr/drivers/stepper.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/shell/shell.h>

#include <stm32_ll_rcc.h>
// #include <stm32f767xx.h>

const struct device *counter2 = DEVICE_DT_GET(DT_NODELABEL(counter2));
const struct device *counter5 = DEVICE_DT_GET(DT_NODELABEL(counter5));
const struct device *pwm2 = DEVICE_DT_GET(DT_NODELABEL(pwm2));
const struct device *stepper = DEVICE_DT_GET(DT_NODELABEL(drv8424));
const struct device *dac = DEVICE_DT_GET(DT_NODELABEL(mcp4726));

void test_callback(const struct device *dev, void *user_data)
{

	pwm_set_cycles(pwm2, 1, 1, 1, 0);
	printk("Ping\n");
}

TIM_TypeDef *TIMx5 = ((TIM_TypeDef *)DT_REG_ADDR(DT_NODELABEL(timers5)));

TIM_TypeDef *TIMx2 = ((TIM_TypeDef *)DT_REG_ADDR(DT_NODELABEL(timers2)));

/* Empty main */
int main(void)
{
	int ret;
	ret = dac_write_value(dac, 0, 2045);
	printk("Dac Write Return Value: %i\n", ret);
	ret = stepper_enable(stepper, true);
	printk("Stepper Enable Return Value: %i\n", ret);
	stepper_set_micro_step_res(stepper, 1);

	struct counter_top_cfg cfg;
	cfg.flags = 0;
	cfg.ticks = 200; // 200 Ticks/s
	cfg.callback = test_callback;
	cfg.user_data = NULL;
	// counter_set_top_value(counter2, &cfg);
	// counter_start(counter2);

	LL_TIM_SetTriggerOutput(TIMx2, LL_TIM_TRGO_UPDATE); // CR2-MMS - Update

	LL_TIM_EnableMasterSlaveMode(TIMx5);			// SMCR-MSM 1

	LL_TIM_SetTriggerInput(TIMx5, LL_TIM_TS_ITR0);              // SMCR-TS 000
	LL_TIM_SetClockSource(TIMx5, LL_TIM_CLOCKSOURCE_EXT_MODE1); // SMCR-SMS 0111
	counter_set_top_value(counter5, &cfg);
	counter_start(counter5);

	pwm_set(pwm2, 1, 5000000, 2500000, 0);

	return 0;
}
