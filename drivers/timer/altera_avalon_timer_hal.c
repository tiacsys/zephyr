/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT altr_interval_timer

#include <kernel.h>
#include <arch/cpu.h>
#include <device.h>
#include <drivers/timer/system_timer.h>

#include <altera_common.h>
#include "altera_avalon_timer_regs.h"
#include "altera_avalon_timer.h"

#define TIMER_NODE		DT_NODELABEL(systick)

BUILD_ASSERT(DT_NODE_HAS_COMPAT_STATUS(TIMER_NODE, altr_interval_timer, okay),
		"Configured system timer is not compatible with this driver "
		"or disabled or has no node label systick in the device tree.");

#define TIMER_IRQ		DT_IRQN(TIMER_NODE)
#define TIMER_BASE_ADDR		DT_REG_ADDR(TIMER_NODE)
#define TIMER_CLOCK_FREQUENCY	DT_PROP(TIMER_NODE, clock_frequency)

#define TICKS_PER_SEC		CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CYCLES_PER_SEC		TIMER_CLOCK_FREQUENCY
#define CYCLES_PER_TICK		(CYCLES_PER_SEC / TICKS_PER_SEC)

BUILD_ASSERT(TIMER_CLOCK_FREQUENCY == CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
		"Configured system timer frequency does not match the timer "
		"clock frequency in the device tree.");

BUILD_ASSERT(CYCLES_PER_SEC >= TICKS_PER_SEC,
		"Timer clock frequency must be greater than the system tick "
		"frequency.");

BUILD_ASSERT((CYCLES_PER_SEC % TICKS_PER_SEC) == 0,
		"Timer clock frequency is not divisible by the system tick "
		"frequency.");

/* The old driver "now" API would return a full uptime value.  The new
 * one only requires the driver to track ticks since the last announce
 * call.  Implement the new call in terms of the old one on legacy
 * drivers by keeping (yet another) uptime value locally.
 */
static uint32_t driver_uptime;

static uint32_t accumulated_cycle_count;

static int32_t _sys_idle_elapsed_ticks = 1;

static void wrapped_announce(int32_t ticks)
{
	driver_uptime += ticks;
	sys_clock_announce(ticks);
}

static void timer_irq_handler(const void *unused)
{
	ARG_UNUSED(unused);

	accumulated_cycle_count += k_ticks_to_cyc_floor32(1);

	/* Clear the interrupt */
	alt_handle_irq((void *)TIMER_BASE_ADDR, TIMER_IRQ);

	wrapped_announce(_sys_idle_elapsed_ticks);
}

int sys_clock_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	IOWR_ALTERA_AVALON_TIMER_PERIODL(TIMER_BASE_ADDR,
			k_ticks_to_cyc_floor32(1) & 0xFFFF);
	IOWR_ALTERA_AVALON_TIMER_PERIODH(TIMER_BASE_ADDR,
			(k_ticks_to_cyc_floor32(1) >> 16) & 0xFFFF);

	IRQ_CONNECT(TIMER_IRQ, 0, timer_irq_handler, NULL, 0);
	irq_enable(TIMER_IRQ);

	alt_avalon_timer_sc_init((void *)TIMER_BASE_ADDR, 0,
			TIMER_IRQ, k_ticks_to_cyc_floor32(1));

	return 0;
}

uint32_t sys_clock_cycle_get_32(void)
{
	/* Per the Altera Embedded IP Peripherals guide, you cannot
	 * use a timer instance for both the system clock and timestamps
	 * at the same time.
	 *
	 * Having this function return accumulated_cycle_count + get_snapshot()
	 * does not work reliably. It's possible for the current countdown
	 * to reset to the next interval before the timer interrupt is
	 * delivered (and accumulated cycle count gets updated). The result
	 * is an unlucky call to this function will appear to jump backward
	 * in time.
	 *
	 * To properly obtain timestamps, the CPU must be configured with
	 * a second timer peripheral instance that is configured to
	 * count down from some large initial 64-bit value. This
	 * is currently unimplemented.
	 */
	return accumulated_cycle_count;
}

uint32_t sys_clock_elapsed(void)
{
	return 0;
}
