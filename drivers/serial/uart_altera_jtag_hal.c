/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT altr_jtag_uart

/**
 * @brief Altera JTAG UART Small Driver
 *
 * This is the driver for the Altera JTAG UART core serial device.
 *
 * Before individual UART port can be used, uart_altera_jtag_init()
 * has to be called to setup the port. This driver supports only
 * the HAL "small driver" representation without interrupts, handshake
 * signal, or concurrent access locking. Therefore this implementation
 * skips the file descriptions as provided by HAL driver and will call
 * the low-layer functions directly (have to declare here explicitly).
 */

#include <device.h>
#include <drivers/uart.h>
#include <sys/sys_io.h>

#include "altera_avalon_jtag_uart.h"
#include "altera_avalon_jtag_uart_regs.h"

#define DEV_CFG(dev) \
	((const struct uart_device_config * const)(dev)->config)

extern int altera_avalon_jtag_uart_read(altera_avalon_jtag_uart_state *sp,
		char *buffer, int space, int flags);
extern int altera_avalon_jtag_uart_write(altera_avalon_jtag_uart_state *sp,
		const char *ptr, int count, int flags);

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void uart_altera_jtag_poll_out(const struct device *dev,
					       unsigned char c)
{
	const struct uart_device_config *config;
	altera_avalon_jtag_uart_state ustate;

	config = DEV_CFG(dev);

	ustate.base = config->regs;
	altera_avalon_jtag_uart_write(&ustate, &c, 1, 0);
}

/**
 * @brief Initialize individual UART port
 *
 * This routine is called to reset the chip in a quiescent state.
 *
 * @param dev UART device struct
 *
 * @return The initialization success status.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, it was failed.
 */
static int uart_altera_jtag_init(const struct device *dev)
{
	const struct uart_device_config *config;

	config = DEV_CFG(dev);

	/*
	 * Work around to clear interrupt enable bits
	 * as it is not being done by HAL driver explicitly.
	 */
	IOWR_ALTERA_AVALON_JTAG_UART_CONTROL(config->regs, 0);

	return 0;
}

static const struct uart_driver_api uart_altera_jtag_driver_api = {
	.poll_in = NULL,
	.poll_out = &uart_altera_jtag_poll_out,
	.err_check = NULL,
};

#define UART_ALTERA_JTAG_DEV_CFG(port) \
static struct uart_device_config uart_altera_jtag_dev_cfg_##port = { \
	.regs = DT_INST_REG_ADDR(port), \
	.sys_clk_freq = 0, /* Unused */ \
}

#define UART_ALTERA_JTAG_INIT(port) \
DEVICE_DT_INST_DEFINE(port, \
	uart_altera_jtag_init, \
	NULL, \
	NULL, \
	&uart_altera_jtag_dev_cfg_##port, \
	PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
	&uart_altera_jtag_driver_api)

#define UART_ALTERA_JTAG_INSTANTIATE(inst)	\
	UART_ALTERA_JTAG_DEV_CFG(inst);		\
	UART_ALTERA_JTAG_INIT(inst);

DT_INST_FOREACH_STATUS_OKAY(UART_ALTERA_JTAG_INSTANTIATE)
