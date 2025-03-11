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
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/shell/shell.h>

#include <stm32_ll_rcc.h>
// #include <stm32f767xx.h>

const struct device *counter2 = DEVICE_DT_GET(DT_NODELABEL(counter2));
const struct device *counter5 = DEVICE_DT_GET(DT_NODELABEL(counter5));
const struct device *pwm2 = DEVICE_DT_GET(DT_NODELABEL(pwm2));

void test_callback(const struct device *dev, void *user_data)
{
	printk("Ping\n");
}

TIM_TypeDef *TIMx5 = ((TIM_TypeDef *)DT_REG_ADDR(DT_NODELABEL(timers5)));

TIM_TypeDef *TIMx2 = ((TIM_TypeDef *)DT_REG_ADDR(DT_NODELABEL(timers2)));


/* Empty main */
int main(void)
{
	
	struct counter_top_cfg cfg;
	cfg.flags = 0;
	cfg.ticks = 100*60;	// 100 Ticks/s
	cfg.callback = test_callback;
	cfg.user_data = NULL;
	// counter_set_top_value(counter2, &cfg);
	// counter_start(counter2);

	
	LL_TIM_SetTriggerOutput(TIMx2, LL_TIM_TRGO_UPDATE);	// CR2-MMS - Update
	// LL_TIM_EnableMasterSlaveMode(TIMx2);

	LL_TIM_EnableMasterSlaveMode(TIMx5);

	/* @param  TriggerInput This parameter can be one of the following values:
	 *         @arg @ref LL_TIM_TS_ITR0
	 *         @arg @ref LL_TIM_TS_ITR1
	 *         @arg @ref LL_TIM_TS_ITR2
	 *         @arg @ref LL_TIM_TS_ITR3
	 *         @arg @ref LL_TIM_TS_TI1F_ED
	 *         @arg @ref LL_TIM_TS_TI1FP1
	 *         @arg @ref LL_TIM_TS_TI2FP2
	 *         @arg @ref LL_TIM_TS_ETRF*/
	// TIM_TypeDef *TIMx;

	LL_TIM_SetTriggerInput(TIMx5, LL_TIM_TS_ITR0);	// SMCR-TS 000
	LL_TIM_SetClockSource(TIMx5, LL_TIM_CLOCKSOURCE_EXT_MODE1);	// SMCR-SMS 0111
	counter_set_top_value(counter5, &cfg);
	counter_start(counter5);

	pwm_set(pwm2, 1, 10000000, 5000000, 0);


		return 0;
}
