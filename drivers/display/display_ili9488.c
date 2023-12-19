/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2023 TiaC Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_ili9488.h"
#include "display_ili9xxx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_ili9488, CONFIG_DISPLAY_LOG_LEVEL);

int ili9488_regs_init(const struct device *dev)
{
	const struct ili9xxx_config *config = dev->config;
	const struct ili9488_regs *regs = config->regs;

	int r;

	LOG_HEXDUMP_DBG(regs->frmctr1, ILI9488_FRMCTR1_LEN, "FRMCTR1");
	r = config->reg_transmit_fn(dev, ILI9488_FRMCTR1, regs->frmctr1,
					 ILI9488_FRMCTR1_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->disctrl, ILI9488_DISCTRL_LEN, "DISCTRL");
	r = config->reg_transmit_fn(dev, ILI9488_DISCTRL, regs->disctrl,
					 ILI9488_DISCTRL_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pwctrl1, ILI9488_PWCTRL1_LEN, "PWCTRL1");
	r = config->reg_transmit_fn(dev, ILI9488_PWCTRL1, regs->pwctrl1,
					 ILI9488_PWCTRL1_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pwctrl2, ILI9488_PWCTRL2_LEN, "PWCTRL2");
	r = config->reg_transmit_fn(dev, ILI9488_PWCTRL2, regs->pwctrl2,
					 ILI9488_PWCTRL2_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->vmctrl, ILI9488_VMCTRL_LEN, "VMCTRL");
	r = config->reg_transmit_fn(dev, ILI9488_VMCTRL, regs->vmctrl,
					 ILI9488_VMCTRL_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pgamctrl, ILI9488_PGAMCTRL_LEN, "PGAMCTRL");
	r = config->reg_transmit_fn(dev, ILI9488_PGAMCTRL, regs->pgamctrl,
					 ILI9488_PGAMCTRL_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->ngamctrl, ILI9488_NGAMCTRL_LEN, "NGAMCTRL");
	r = config->reg_transmit_fn(dev, ILI9488_NGAMCTRL, regs->ngamctrl,
					 ILI9488_NGAMCTRL_LEN);
	if (r < 0) {
		return r;
	}

	return 0;
}
