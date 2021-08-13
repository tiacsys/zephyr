/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT altr_msgdma

#include <device.h>
#include <errno.h>
#include <init.h>
#include <string.h>
#include <soc.h>
#include <drivers/dma.h>

#include <altera_common.h>
#include "altera_msgdma.h"

#define LOG_LEVEL CONFIG_DMA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(dma_nios2_msgdma);

/* Device constant configuration parameters */
struct dma_nios2_msgdma_dev_config {
	void (*irq_config_func)(const struct device *dev);
	alt_msgdma_dev *msgdma_dev;
};

/* Device run time data */
struct dma_nios2_msgdma_dev_data {
	struct k_sem sem_lock;
	dma_callback_t user_callback;
	void *user_data;
	alt_msgdma_standard_descriptor msgdma_desc;
};

#define DEV_CFG(dev) \
	((const struct dma_nios2_msgdma_dev_config * const)(dev)->config)
#define DEV_DATA(dev) \
	((struct dma_nios2_msgdma_dev_data * const)(dev)->data)

static void dma_nios2_msgdma_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct dma_nios2_msgdma_dev_config *dev_cfg = DEV_CFG(dev);

	/* Call Altera HAL driver ISR */
	alt_handle_irq(dev_cfg->msgdma_dev, dev_cfg->msgdma_dev->irq_ID);
}

static void dma_nios2_msgdma_callback(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct dma_nios2_msgdma_dev_config *dev_cfg = DEV_CFG(dev);
	struct dma_nios2_msgdma_dev_data *dev_data = DEV_DATA(dev);
	int err_code;
	uint32_t status;

	status = IORD_ALTERA_MSGDMA_CSR_STATUS(dev_cfg->msgdma_dev->csr_base);

	if (status & ALTERA_MSGDMA_CSR_STOPPED_ON_ERROR_MASK) {
		err_code = -EIO;
	} else if (status & ALTERA_MSGDMA_CSR_BUSY_MASK) {
		err_code = -EBUSY;
	} else {
		err_code = 0;
	}

	LOG_DBG("Nios-II mSGDMA CSR status Reg: 0x%x", status);

	dev_data->user_callback(dev, dev_data->user_data, 0, err_code);
}

static int dma_nios2_msgdma_config(const struct device *dev,
				uint32_t channel, struct dma_config *cfg)
{
	const struct dma_nios2_msgdma_dev_config *dev_cfg = DEV_CFG(dev);
	struct dma_nios2_msgdma_dev_data *dev_data = DEV_DATA(dev);
	int status;

	/* Nios-II MSGDMA supports only one channel per DMA core */
	if (channel != 0U) {
		LOG_ERR("'channel' value %d is not supported", channel);
		return -EINVAL;
	}

	if (dev_cfg->msgdma_dev->prefetcher_enable) {
		if (cfg->block_count > 1U) {
			LOG_ERR("driver yet add support multiple descriptors");
			return -EINVAL;
		}
	} else {
		if (cfg->block_count != 1U) {
			LOG_ERR("'block_count' value %d is not supported",
					cfg->block_count);
			return -EINVAL;
		}
	}

	if (cfg->head_block == NULL) {
		LOG_ERR("head_block ptr is NULL");
		return -EINVAL;
	}

	if (cfg->head_block->block_size > dev_cfg->msgdma_dev->max_byte) {
		LOG_ERR("'block_size' value %d is too big",
			    cfg->head_block->block_size);
		return -EINVAL;
	}

	/* Lock by semaphore */
	k_sem_take(&dev_data->sem_lock, K_FOREVER);

#define ALTERA_MSGDMA_DESCRIPTOR_CONTROL \
	( ALTERA_MSGDMA_DESCRIPTOR_CONTROL_TRANSFER_COMPLETE_IRQ_MASK \
	| ALTERA_MSGDMA_DESCRIPTOR_CONTROL_EARLY_TERMINATION_IRQ_MASK )

	dev_data->user_callback = cfg->dma_callback;
	dev_data->user_data = cfg->user_data;

	switch (cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		status = alt_msgdma_construct_standard_mm_to_mm_descriptor(
				dev_cfg->msgdma_dev,
				&dev_data->msgdma_desc,
				(alt_u32 *)cfg->head_block->source_address,
				(alt_u32 *)cfg->head_block->dest_address,
				(alt_u32)cfg->head_block->block_size,
				ALTERA_MSGDMA_DESCRIPTOR_CONTROL);
		break;
	case MEMORY_TO_PERIPHERAL:
		status = alt_msgdma_construct_standard_mm_to_st_descriptor(
				dev_cfg->msgdma_dev,
				&dev_data->msgdma_desc,
				(alt_u32 *)cfg->head_block->source_address,
				(alt_u32)cfg->head_block->block_size,
				ALTERA_MSGDMA_DESCRIPTOR_CONTROL);
		break;
	case PERIPHERAL_TO_MEMORY:
		status = alt_msgdma_construct_standard_st_to_mm_descriptor(
				dev_cfg->msgdma_dev,
				&dev_data->msgdma_desc,
				(alt_u32 *)cfg->head_block->dest_address,
				(alt_u32)cfg->head_block->block_size,
				ALTERA_MSGDMA_DESCRIPTOR_CONTROL);
		break;
	default:
		LOG_ERR("'channel_direction' value %d is not supported",
			    cfg->channel_direction);
		status = -EINVAL;
		break;
	}

#undef ALTERA_MSGDMA_DESCRIPTOR_CONTROL

	/* Register Nios-II mSGDMA callback */
	alt_msgdma_register_callback(
			dev_cfg->msgdma_dev,
			dma_nios2_msgdma_callback,
			ALTERA_MSGDMA_CSR_GLOBAL_INTERRUPT_MASK |
			ALTERA_MSGDMA_CSR_STOP_ON_ERROR_MASK |
			ALTERA_MSGDMA_CSR_STOP_ON_EARLY_TERMINATION_MASK,
			(void *)dev);

	/* Clear the IRQ status */
	IOWR_ALTERA_MSGDMA_CSR_STATUS(
			dev_cfg->msgdma_dev->csr_base,
			ALTERA_MSGDMA_CSR_IRQ_SET_MASK);

	/* Unlock by semaphore */
	k_sem_give(&dev_data->sem_lock);

	return status;
}

static int dma_nios2_msgdma_transfer_start(const struct device *dev,
						uint32_t channel)
{
	const struct dma_nios2_msgdma_dev_config *dev_cfg = DEV_CFG(dev);
	struct dma_nios2_msgdma_dev_data *dev_data = DEV_DATA(dev);
	int status;

	/* Nios-II mSGDMA supports only one channel per DMA core */
	if (channel != 0U) {
		LOG_ERR("Invalid channel number");
		return -EINVAL;
	}

	/* Lock by semaphore */
	k_sem_take(&dev_data->sem_lock, K_FOREVER);

	/* Start the Nios-II mSGDMA Dispatcher (non-blocking transfer) */
	status = alt_msgdma_standard_descriptor_async_transfer(
			dev_cfg->msgdma_dev, &dev_data->msgdma_desc);

	/* Unlock by semaphore */
	k_sem_give(&dev_data->sem_lock);

	if (status < 0) {
		LOG_ERR("Nios-II mSGDMA transfer error (%d)", status);
	}

	LOG_DBG("Nios-II mSGDMA dispatcher started");

	return status;
}

static int dma_nios2_msgdma_transfer_stop(const struct device *dev,
						uint32_t channel)
{
	const struct dma_nios2_msgdma_dev_config *dev_cfg = DEV_CFG(dev);
	struct dma_nios2_msgdma_dev_data *dev_data = DEV_DATA(dev);
	int ret = -EIO;
	uint32_t status;

	/* Lock by semaphore */
	k_sem_take(&dev_data->sem_lock, K_FOREVER);

	/* Stop the Nios-II mSGDMA dispatcher */
	IOWR_ALTERA_MSGDMA_CSR_CONTROL(dev_cfg->msgdma_dev->csr_base,
						ALTERA_MSGDMA_CSR_STOP_MASK);
	status = IORD_ALTERA_MSGDMA_CSR_STATUS(dev_cfg->msgdma_dev->csr_base);

	/* Unlock by semaphore */
	k_sem_give(&dev_data->sem_lock);

	if (status & ALTERA_MSGDMA_CSR_STOP_STATE_MASK) {
		LOG_DBG("Nios-II mSGDMA dispatcher stopped");
		ret = 0;
	}

	LOG_DBG("Nios-II mSGDMA CSR status Reg: 0x%x", status);

	return status;
}

static const struct dma_driver_api dma_nios2_msgdma_driver_api = {
	.config = dma_nios2_msgdma_config,
	.start = dma_nios2_msgdma_transfer_start,
	.stop = dma_nios2_msgdma_transfer_stop,
};

static int dma_nios2_msgdma_init(const struct device *dev)
{
	const struct dma_nios2_msgdma_dev_config *dev_cfg = DEV_CFG(dev);
	struct dma_nios2_msgdma_dev_data *dev_data = DEV_DATA(dev);

	/* Initialize semaphore */
	k_sem_init(&dev_data->sem_lock, 1, 1);

	/* Initialize module */
	alt_msgdma_init(dev_cfg->msgdma_dev,
			dev_cfg->msgdma_dev->irq_controller_ID,
			dev_cfg->msgdma_dev->irq_ID);

	/* Configure interrupts */
	dev_cfg->irq_config_func(dev);

	/* Enable module's IRQ */
	irq_enable(dev_cfg->msgdma_dev->irq_ID);

	return 0;
}

#define DMA_NIOS2_MSGDMA_IRQ_CONF_FUNC(inst) \
static void dma_nios2_msgdma_irq_config_##inst(const struct device *dev) \
{ \
	ARG_UNUSED(dev); \
	IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), \
			dma_nios2_msgdma_isr, DEVICE_DT_INST_GET(inst), 0); \
}

/* Simulate ALTERA_MSGDMA_CSR_DESCRIPTOR_SLAVE_INSTANCE() but from DTS. */
#define DMA_NIOS2_ALT_MSGDMA_DEV_INSTANCE(inst) \
static alt_msgdma_dev msgdma_dev_##inst = { \
	.llist = ALT_LLIST_ENTRY, \
	.name = DT_INST_LABEL(inst), \
	.csr_base = (alt_u32 *)DT_INST_REG_ADDR_BY_NAME(inst,csr), \
	.descriptor_base = (alt_u32 *)DT_INST_REG_ADDR_BY_NAME(inst,desc), \
	.irq_controller_ID = (alt_u32)0, /* cpu inst */ \
	.irq_ID = (alt_u32)DT_INST_IRQN(inst), \
	.descriptor_fifo_depth = (alt_u32)DT_INST_PROP(inst, desc_fifo_depth), \
	.burst_enable = (alt_u8)DT_INST_PROP(inst, burst), \
	.burst_wrapping_support = (alt_u8)DT_INST_PROP(inst, burst_wrapping), \
	.data_fifo_depth = (alt_u32)DT_INST_PROP(inst, data_fifo_depth), \
	.data_width = (alt_u32)DT_INST_PROP(inst, data_width), \
	.max_burst_count = (alt_u32)DT_INST_PROP(inst, max_burst_count), \
	.max_byte = (alt_u32)DT_INST_PROP(inst, max_xfer_byte), \
	.max_stride = (alt_u64)DT_INST_PROP(inst, max_stride_words), \
	.programmable_burst_enable \
		= (alt_u8)DT_INST_PROP(inst, burst_programming), \
	.stride_enable = (alt_u8)DT_INST_PROP(inst, stride_addressing), \
	.enhanced_features = (alt_u8)DT_INST_PROP(inst, extended_features), \
	.response_port = (alt_u8)DT_INST_PROP(inst, response_port), \
	.prefetcher_enable = (alt_u8)DT_INST_PROP(inst, pre_fetching), \
};

#define DMA_NIOS2_MSGDMA_DEV_CFG(inst) \
static struct dma_nios2_msgdma_dev_config dma_nios2_msgdma_dev_cfg_##inst = { \
	.irq_config_func = dma_nios2_msgdma_irq_config_##inst, \
	.msgdma_dev = &msgdma_dev_##inst, \
};

#define DMA_NIOS2_MSGDMA_DEV_DATA(inst) \
static struct dma_nios2_msgdma_dev_data dma_nios2_msgdma_dev_data_##inst = { \
};

#define DMA_NIOS2_MSGDMA_INIT(inst) \
DEVICE_DT_INST_DEFINE(inst, \
		&dma_nios2_msgdma_init, \
		NULL, \
		&dma_nios2_msgdma_dev_data_##inst, \
		&dma_nios2_msgdma_dev_cfg_##inst, \
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		&dma_nios2_msgdma_driver_api);

#define DMA_NIOS2_MSGDMA_INSTANTIATE(inst) \
	DMA_NIOS2_MSGDMA_IRQ_CONF_FUNC(inst); \
	DMA_NIOS2_ALT_MSGDMA_DEV_INSTANCE(inst); \
	DMA_NIOS2_MSGDMA_DEV_CFG(inst); \
	DMA_NIOS2_MSGDMA_DEV_DATA(inst); \
	DMA_NIOS2_MSGDMA_INIT(inst);

DT_INST_FOREACH_STATUS_OKAY(DMA_NIOS2_MSGDMA_INSTANTIATE)
