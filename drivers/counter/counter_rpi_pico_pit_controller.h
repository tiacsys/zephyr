/*
 * Copyright (c) 2023 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/counter.h>
#include <zephyr/sys/slist.h>

struct rpi_pico_pit_callback {
	sys_snode_t node;

	/** Actual callback function being called when relevant. */
	struct counter_top_cfg *handler;

	/** A mask containing the slice of the channel the callback belongs to
	 */
	uint32_t slice_mask;
};

uint32_t counter_rpi_pico_pit_controller_get_pending_int(const struct device *dev, uint32_t mask);

int counter_rpi_pico_pit_controller_manage_callback(const struct device *dev,
						    struct rpi_pico_pit_callback *callback,
						    bool set);
