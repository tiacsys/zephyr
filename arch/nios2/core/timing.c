/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <sys_clock.h>
#include <timing/timing.h>

#include "altera_avalon_timer_regs.h"

#define TIMER_BASE_ADDR		CONFIG_ARCH_TIMING_BASE_ADDRESS

BUILD_ASSERT(TIMER_BASE_ADDR > 0,
		"Configured system timer has incalid base address. "
		"Check node with label systick in the device tree.");

#define NIOS2_SUBTRACT_CLOCK_CYCLES(val)                                      \
	((IORD_ALTERA_AVALON_TIMER_PERIODH(TIMER_BASE_ADDR) << 16 |           \
	  (IORD_ALTERA_AVALON_TIMER_PERIODL(TIMER_BASE_ADDR))) -              \
	 ((uint32_t)val))

#define TIMING_INFO_OS_GET_TIME()                                             \
	(NIOS2_SUBTRACT_CLOCK_CYCLES(                                         \
		((uint32_t)IORD_ALTERA_AVALON_TIMER_SNAPH(TIMER_BASE_ADDR)    \
		 << 16) |                                                     \
		((uint32_t)IORD_ALTERA_AVALON_TIMER_SNAPL(TIMER_BASE_ADDR))))

void arch_timing_init(void)
{
}

void arch_timing_start(void)
{
}

void arch_timing_stop(void)
{
}

timing_t arch_timing_counter_get(void)
{
	IOWR_ALTERA_AVALON_TIMER_SNAPL(TIMER_BASE_ADDR, 10);
	return TIMING_INFO_OS_GET_TIME();
}

uint64_t arch_timing_cycles_get(volatile timing_t *const start,
				volatile timing_t *const end)
{
	return (*end - *start);
}

uint64_t arch_timing_freq_get(void)
{
	return sys_clock_hw_cycles_per_sec();
}

uint64_t arch_timing_cycles_to_ns(uint64_t cycles)
{
	return k_cyc_to_ns_floor64(cycles);
}

uint64_t arch_timing_cycles_to_ns_avg(uint64_t cycles, uint32_t count)
{
	return arch_timing_cycles_to_ns(cycles) / count;
}

uint32_t arch_timing_freq_get_mhz(void)
{
	return (uint32_t)(arch_timing_freq_get() / 1000000U);
}
