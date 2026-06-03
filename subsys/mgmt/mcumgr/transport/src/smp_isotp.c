/*
 * Copyright (c) 2026, inovex GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief ISO-TP (CAN, ISO 15765-2) transport for the MCUmgr SMP protocol.
 *
 * ISO-TP performs its own segmentation, reassembly and flow control, so a
 * single ISO-TP message carries one complete SMP packet. The MCUmgr reassembly
 * layer is therefore not used and the transport MTU is the net_buf size, capped
 * at the 4095-byte ISO-TP message limit.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/net_buf.h>
#include <zephyr/drivers/can.h>
#include <zephyr/canbus/isotp.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>

#include <mgmt/mcumgr/transport/smp_internal.h>

LOG_MODULE_REGISTER(smp_isotp, CONFIG_MCUMGR_TRANSPORT_ISOTP_LOG_LEVEL);

/* ISO-TP single message can carry at most 4095 bytes (12-bit length field). */
#define SMP_ISOTP_MAX_MSG_SIZE 4095U

/* CAN ID flags shared by RX and TX addresses. */
#if defined(CONFIG_MCUMGR_TRANSPORT_ISOTP_EXTENDED_ID)
#define SMP_ISOTP_ID_FLAGS ISOTP_MSG_IDE
#else
#define SMP_ISOTP_ID_FLAGS 0
#endif

#if defined(CONFIG_MCUMGR_TRANSPORT_ISOTP_CAN_FD)
#define SMP_ISOTP_FD_FLAGS (ISOTP_MSG_FDF | ISOTP_MSG_BRS)
#define SMP_ISOTP_TX_DL    CONFIG_MCUMGR_TRANSPORT_ISOTP_TX_DL
#else
#define SMP_ISOTP_FD_FLAGS 0
#define SMP_ISOTP_TX_DL    0
#endif

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

/* Zephyr's ISO-TP API uses separate receive (isotp_bind) and send (isotp_send)
 * contexts, each of which installs an exact CAN RX filter: the receive context
 * filters on the incoming data identifier, the send context filters on the
 * identifier it expects flow-control on. If flow control shared a data
 * identifier, both contexts would filter the same CAN ID and every frame would
 * be delivered to both state machines, aborting transfers with
 * ISOTP_N_UNEXP_PDU. The flow-control identifiers are therefore configured
 * separately from, and must be distinct from, the data identifiers.
 */
BUILD_ASSERT(CONFIG_MCUMGR_TRANSPORT_ISOTP_RX_ID != CONFIG_MCUMGR_TRANSPORT_ISOTP_RX_FC_ID,
	     "ISO-TP RX data and RX flow-control CAN identifiers must differ");
BUILD_ASSERT(CONFIG_MCUMGR_TRANSPORT_ISOTP_TX_ID != CONFIG_MCUMGR_TRANSPORT_ISOTP_TX_FC_ID,
	     "ISO-TP TX data and TX flow-control CAN identifiers must differ");

static const struct isotp_msg_id smp_isotp_rx_addr = {
	.ext_id = CONFIG_MCUMGR_TRANSPORT_ISOTP_RX_ID,
	.flags = SMP_ISOTP_ID_FLAGS | SMP_ISOTP_FD_FLAGS,
};

static const struct isotp_msg_id smp_isotp_tx_addr = {
	.ext_id = CONFIG_MCUMGR_TRANSPORT_ISOTP_TX_ID,
	.dl = SMP_ISOTP_TX_DL,
	.flags = SMP_ISOTP_ID_FLAGS | SMP_ISOTP_FD_FLAGS,
};

/* Flow-control frame this node SENDS while receiving (paired with rx_addr in
 * isotp_bind as the FC transmit identifier).
 */
static const struct isotp_msg_id smp_isotp_fc_tx_addr = {
	.ext_id = CONFIG_MCUMGR_TRANSPORT_ISOTP_TX_FC_ID,
	.flags = SMP_ISOTP_ID_FLAGS | SMP_ISOTP_FD_FLAGS,
};

/* Flow-control frame this node RECEIVES while sending (paired with tx_addr in
 * isotp_send as the FC receive identifier).
 */
static const struct isotp_msg_id smp_isotp_fc_rx_addr = {
	.ext_id = CONFIG_MCUMGR_TRANSPORT_ISOTP_RX_FC_ID,
	.flags = SMP_ISOTP_ID_FLAGS | SMP_ISOTP_FD_FLAGS,
};

static const struct isotp_fc_opts smp_isotp_fc_opts = {
	.bs = CONFIG_MCUMGR_TRANSPORT_ISOTP_FC_BS,
	.stmin = CONFIG_MCUMGR_TRANSPORT_ISOTP_FC_STMIN,
};

static struct isotp_recv_ctx smp_isotp_recv_ctx;
static struct isotp_send_ctx smp_isotp_send_ctx;

/* Serialises transmissions and carries the completion result from the ISO-TP
 * work queue back to the SMP work queue thread running the output function.
 */
static struct k_sem smp_isotp_tx_sem;
static int smp_isotp_tx_result;

static int smp_isotp_tx_pkt(struct net_buf *nb);
static uint16_t smp_isotp_get_mtu(const struct net_buf *nb);

struct smp_transport smp_isotp_transport = {
	.functions.output = smp_isotp_tx_pkt,
	.functions.get_mtu = smp_isotp_get_mtu,
};

#ifdef CONFIG_SMP_CLIENT
static struct smp_client_transport_entry smp_isotp_client_transport = {
	.smpt = &smp_isotp_transport,
	.smpt_type = SMP_ISOTP_TRANSPORT,
};
#endif

static struct k_thread smp_isotp_thread;
static K_KERNEL_STACK_DEFINE(smp_isotp_stack, CONFIG_MCUMGR_TRANSPORT_ISOTP_THREAD_STACK_SIZE);

static void smp_isotp_tx_complete(int error_nr, void *arg)
{
	ARG_UNUSED(arg);

	smp_isotp_tx_result = error_nr;
	k_sem_give(&smp_isotp_tx_sem);
}

static uint16_t smp_isotp_get_mtu(const struct net_buf *nb)
{
	ARG_UNUSED(nb);

	return (uint16_t)MIN(CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE, SMP_ISOTP_MAX_MSG_SIZE);
}

static int smp_isotp_tx_pkt(struct net_buf *nb)
{
	int rc;

	if (nb == NULL) {
		return MGMT_ERR_EINVAL;
	}

	/* The net_buf data must remain valid until the ISO-TP send completes, so
	 * the completion callback signals a semaphore that this thread waits on.
	 */
	k_sem_reset(&smp_isotp_tx_sem);
	smp_isotp_tx_result = ISOTP_N_OK;

	rc = isotp_send(&smp_isotp_send_ctx, can_dev, nb->data, nb->len,
			&smp_isotp_tx_addr, &smp_isotp_fc_rx_addr,
			smp_isotp_tx_complete, NULL);
	if (rc != ISOTP_N_OK) {
		LOG_ERR("Failed to start ISO-TP send: %d", rc);
		smp_packet_free(nb);
		return MGMT_ERR_EUNKNOWN;
	}

	if (k_sem_take(&smp_isotp_tx_sem,
		       K_MSEC(CONFIG_MCUMGR_TRANSPORT_ISOTP_TX_TIMEOUT_MS)) != 0) {
		LOG_ERR("ISO-TP send timed out");
		smp_packet_free(nb);
		return MGMT_ERR_ETIMEOUT;
	}

	if (smp_isotp_tx_result != ISOTP_N_OK) {
		LOG_ERR("ISO-TP send failed: %d", smp_isotp_tx_result);
		smp_packet_free(nb);
		return MGMT_ERR_EUNKNOWN;
	}

	smp_packet_free(nb);

	return MGMT_ERR_EOK;
}

/* Receive one complete ISO-TP message into a freshly allocated SMP net_buf.
 * Returns the buffer on success (caller owns it), or NULL on error/overflow.
 */
static struct net_buf *smp_isotp_recv_msg(void)
{
	struct net_buf *nb;
	struct net_buf *frag;
	int rem_len;

	nb = smp_packet_alloc();
	if (nb == NULL) {
		LOG_ERR("Failed to allocate SMP packet");
		return NULL;
	}

	do {
		rem_len = isotp_recv_net(&smp_isotp_recv_ctx, &frag, K_FOREVER);
		if (rem_len < 0) {
			LOG_ERR("ISO-TP receive error: %d", rem_len);
			smp_packet_free(nb);
			return NULL;
		}

		while (frag != NULL) {
			if (frag->len > net_buf_tailroom(nb)) {
				LOG_ERR("ISO-TP message exceeds buffer size, dropping");
				net_buf_unref(frag);
				smp_packet_free(nb);
				return NULL;
			}

			net_buf_add_mem(nb, frag->data, frag->len);
			frag = net_buf_frag_del(NULL, frag);
		}
	} while (rem_len > 0);

	return nb;
}

static void smp_isotp_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int rc;
	struct net_buf *nb;

	rc = isotp_bind(&smp_isotp_recv_ctx, can_dev, &smp_isotp_rx_addr, &smp_isotp_fc_tx_addr,
			&smp_isotp_fc_opts, K_FOREVER);
	if (rc != ISOTP_N_OK) {
		LOG_ERR("Failed to bind ISO-TP RX address: %d", rc);
		return;
	}

	while (1) {
		nb = smp_isotp_recv_msg();
		if (nb != NULL) {
			smp_rx_req(&smp_isotp_transport, nb);
		}
	}
}

static void smp_isotp_start(void)
{
	int rc;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device %s not ready", can_dev->name);
		return;
	}

#ifdef CONFIG_MCUMGR_TRANSPORT_ISOTP_AUTO_START
	can_mode_t mode = CAN_MODE_NORMAL;

#ifdef CONFIG_MCUMGR_TRANSPORT_ISOTP_CAN_FD
	mode |= CAN_MODE_FD;
#endif

	rc = can_set_mode(can_dev, mode);
	if (rc != 0) {
		LOG_ERR("Failed to set CAN mode: %d", rc);
		return;
	}

	rc = can_start(can_dev);
	if (rc != 0 && rc != -EALREADY) {
		LOG_ERR("Failed to start CAN device: %d", rc);
		return;
	}
#endif /* CONFIG_MCUMGR_TRANSPORT_ISOTP_AUTO_START */

	k_sem_init(&smp_isotp_tx_sem, 0, 1);

	rc = smp_transport_init(&smp_isotp_transport);
	if (rc != 0) {
		LOG_ERR("Failed to init ISO-TP MCUmgr SMP transport: %d", rc);
		return;
	}

#ifdef CONFIG_SMP_CLIENT
	smp_client_transport_register(&smp_isotp_client_transport);
#endif

	k_thread_create(&smp_isotp_thread, smp_isotp_stack,
			K_KERNEL_STACK_SIZEOF(smp_isotp_stack), smp_isotp_rx_thread,
			NULL, NULL, NULL, CONFIG_MCUMGR_TRANSPORT_ISOTP_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&smp_isotp_thread, "smp_isotp_rx");
}

MCUMGR_HANDLER_DEFINE(smp_isotp, smp_isotp_start);
