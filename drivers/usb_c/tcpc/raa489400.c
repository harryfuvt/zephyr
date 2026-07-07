/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * RAA489400 USB Type-C Port Controller (TCPC) Driver
 * Based on RAA489400 Datasheet R16DS0292EU0100 Rev.1.00
 *
 * All register I/O uses tcpci_priv.h helpers:
 *   tcpci_read_reg8 / tcpci_write_reg8 / tcpci_update_reg8
 *   tcpci_read_reg16 / tcpci_write_reg16
 *   tcpci_tcpm_get_cc
 *   tcpci_tcpm_set_cc
 *   tcpci_tcpm_select_rp_value
 *   tcpci_tcpm_get_rp_value
 *   tcpci_tcpm_set_polarity
 *   tcpci_tcpm_set_vconn
 *   tcpci_tcpm_set_roles
 *   tcpci_tcpm_set_rx_type
 *   tcpci_tcpm_transmit_data
 *   tcpci_tcpm_get_rx_pending_msg
 *   tcpci_tcpm_mask_status_register
 *   tcpci_tcpm_get_status_register
 *   tcpci_tcpm_clear_status_register
 *   tcpci_tcpm_dump_std_reg
 *   tcpci_alert_reg_to_enum
 */

#define DT_DRV_COMPAT renesas_raa489400

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/usb_c/usbc_tcpc.h>
#include <zephyr/drivers/usb_c/tcpci_priv.h>
#include <zephyr/usb_c/tcpci.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>

#include "raa489400.h"

LOG_MODULE_REGISTER(raa489400, CONFIG_USBC_LOG_LEVEL);

/* ----------------------------------------------------------------
 * Driver config — compile-time, from devicetree
 * ---------------------------------------------------------------- */
struct raa489400_cfg {
	const struct i2c_dt_spec bus;
	const struct gpio_dt_spec alert_gpio;
};

/* ----------------------------------------------------------------
 * Driver data — runtime state
 * ---------------------------------------------------------------- */
struct raa489400_data {
	/** GPIO callback for ALERT# interrupt */
	struct gpio_callback alert_cb;
	/** Work item: handle alert outside ISR context */
	struct k_work alert_work;
	/** Alert callback registered by USB-C stack */
	tcpc_alert_handler_cb_t alert_handler;
	/** Opaque data passed back to alert_handler */
	void *alert_handler_data;
	/** USB-C connector device (set by set_alert_handler_cb) */
	const struct device *dev;
	/** Cached CC line states */
	enum tc_cc_voltage_state cc1;
	enum tc_cc_voltage_state cc2;
	/** VCONN enabled state */
	bool vconn_enabled;
	/** One-slot RX FIFO */
	struct pd_msg rx_msg;
	bool msg_pending;
};

/* ================================================================
 * Forward declarations
 * ================================================================ */
static void raa489400_alert_work_handler(struct k_work *work);
static void raa489400_alert_gpio_cb(const struct device *port,
				    struct gpio_callback *cb,
				    gpio_port_pins_t pins);

/* ================================================================
 * Vendor-specific helpers (use tcpci_priv directly)
 * ================================================================ */

/**
 * @brief Read and verify VENDOR_ID to confirm chip identity.
 */
static int raa489400_verify_chip(const struct device *dev)
{
	const struct raa489400_cfg *cfg = dev->config;
	uint16_t id;
	int ret;

	ret = tcpci_read_reg16(&cfg->bus, RAA489400_REG_VENDOR_ID, &id);
	if (ret) {
		return ret;
	}
	if (id != RAA489400_VENDOR_ID) {
		LOG_ERR("Unexpected VENDOR_ID 0x%04x (expected 0x%04x)",
			id, RAA489400_VENDOR_ID);
		return -ENODEV;
	}
	return 0;
}

/**
 * @brief Apply mandatory one-time vendor register settings.
 *
 * Per RAA489400 datasheet:
 *   CONTROL1 bits[2:0] must be 001b
 *   TYPE_C_PARAMETER bits[8:2] must stay at reset value 0b110_0100
 */
static int raa489400_vendor_init(const struct device *dev)
{
	const struct raa489400_cfg *cfg = dev->config;
	int ret;

	/* CONTROL1: oscillator calibration bits must be 001b */
	ret = tcpci_update_reg8(&cfg->bus,
				RAA489400_REG_CONTROL1,
				RAA489400_CONTROL1_OSC_CAL_MASK,
				RAA489400_CONTROL1_OSC_CAL_VALUE);
	if (ret) {
		return ret;
	}

	/* TYPE_C_PARAMETER: preserve reserved bits at reset value,
	 * clear Stop_24MHz_OSC and Force_Enable_24MHz.             */
	return tcpci_write_reg16(&cfg->bus,
				 RAA489400_REG_TYPE_C_PARAMETER,
				 RAA489400_TYPE_C_RESERVED_BITS);
}

/* ================================================================
 * tcpc_driver_api: init
 * ================================================================ */
static int raa489400_init(const struct device *dev)
{
	const struct raa489400_cfg *cfg = dev->config;
	struct raa489400_data *data = dev->data;
	uint8_t pwr_status;
	int ret;

	/* 1. Verify I2C bus is ready */
	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* 2. Wait for TCPC internal init to complete.
	 *    POWER_STATUS.TCPC_Initialization_Status (B6) = 1 while busy.
	 *    Datasheet: clears after ~8 ms (VSYS33-powered startup).    */
	for (int i = 0; i < 50; i++) {
		ret = tcpci_read_reg8(&cfg->bus,
				      RAA489400_REG_POWER_STATUS,
				      &pwr_status);
		if (ret) {
			return ret;
		}
		if (!(pwr_status & RAA489400_PWR_STATUS_INIT)) {
			break;
		}
		k_msleep(10);
	}
	if (pwr_status & RAA489400_PWR_STATUS_INIT) {
		LOG_ERR("RAA489400 init timeout");
		return -ETIMEDOUT;
	}

	/* 3. Verify chip identity */
	ret = raa489400_verify_chip(dev);
	if (ret) {
		return ret;
	}

	/* 4. Apply vendor-specific mandatory register settings */
	ret = raa489400_vendor_init(dev);
	if (ret) {
		return ret;
	}

	/* 5. Enable VBUS detection */
	ret = tcpci_write_reg8(&cfg->bus,
			       RAA489400_REG_COMMAND,
			       RAA489400_CMD_ENABLE_VBUS_DETECT);
	if (ret) {
		return ret;
	}

	/* 6. Clear AllRegistersResetToDefault (always set at POR) */
	ret = tcpci_write_reg8(&cfg->bus,
			       RAA489400_REG_FAULT_STATUS,
			       RAA489400_FAULT_STATUS_ALL_REGS_RESET);
	if (ret) {
		return ret;
	}

	/* 7. Unmask alerts: CC | POWER | FAULT | RX | TX | SNK_DISC | VENDOR */
	ret = tcpci_tcpm_mask_status_register(
		&cfg->bus, TCPC_ALERT_STATUS,
		TCPC_REG_ALERT_CC_STATUS        |
		TCPC_REG_ALERT_POWER_STATUS     |
		TCPC_REG_ALERT_FAULT            |
		TCPC_REG_ALERT_RX_STATUS        |
		TCPC_REG_ALERT_RX_HARD_RST      |
		TCPC_REG_ALERT_TX_COMPLETE      |
		TCPC_REG_ALERT_RX_BUF_OVF      |
		TCPC_REG_ALERT_VENDOR_DEF);
	if (ret) {
		return ret;
	}

	/* 8. Configure ALERT# GPIO interrupt (active-low falling edge) */
	data->dev = dev;
	k_work_init(&data->alert_work, raa489400_alert_work_handler);

	ret = gpio_pin_configure_dt(&cfg->alert_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}
	gpio_init_callback(&data->alert_cb,
			   raa489400_alert_gpio_cb,
			   BIT(cfg->alert_gpio.pin));
	ret = gpio_add_callback(cfg->alert_gpio.port, &data->alert_cb);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->alert_gpio,
					      GPIO_INT_EDGE_FALLING);
	if (ret) {
		return ret;
	}

	LOG_INF("RAA489400 TCPC initialized (addr 0x%02x)", cfg->bus.addr);
	return 0;
}

/* ================================================================
 * tcpc_driver_api: get_cc
 *
 * Delegates entirely to tcpci_tcpm_get_cc() from tcpci_priv.h,
 * which reads CC_STATUS (0x1D) and converts raw bits to Zephyr
 * tc_cc_voltage_state enums, handling both source and sink roles.
 * ================================================================ */
static int raa489400_get_cc(const struct device *dev,
			    enum tc_cc_voltage_state *cc1,
			    enum tc_cc_voltage_state *cc2)
{
	const struct raa489400_cfg *cfg = dev->config;
	struct raa489400_data *data = dev->data;
	int ret;

	ret = tcpci_tcpm_get_cc(&cfg->bus, cc1, cc2);
	if (ret == 0) {
		data->cc1 = *cc1;
		data->cc2 = *cc2;
	}
	return ret;
}

/* ================================================================
 * tcpc_driver_api: set_cc
 *
 * Delegates to tcpci_tcpm_set_cc() which writes ROLE_CONTROL (0x1A)
 * and issues COMMAND.Look4Connection.
 * ================================================================ */
static int raa489400_set_cc(const struct device *dev, enum tc_cc_pull pull)
{
	const struct raa489400_cfg *cfg = dev->config;

	return tcpci_tcpm_set_cc(&cfg->bus, pull);
}

/* ================================================================
 * tcpc_driver_api: select_rp_value
 *
 * Delegates to tcpci_tcpm_select_rp_value() which writes the Rp
 * field of ROLE_CONTROL (0x1A) bits[5:4].
 * ================================================================ */
static int raa489400_select_rp_value(const struct device *dev,
				     enum tc_rp_value rp)
{
	const struct raa489400_cfg *cfg = dev->config;

	return tcpci_tcpm_select_rp_value(&cfg->bus, rp);
}

/* ================================================================
 * tcpc_driver_api: get_rp_value
 * ================================================================ */
static int raa489400_get_rp_value(const struct device *dev,
				  enum tc_rp_value *rp)
{
	const struct raa489400_cfg *cfg = dev->config;

	return tcpci_tcpm_get_rp_value(&cfg->bus, rp);
}

/* ================================================================
 * tcpc_driver_api: set_cc_polarity
 *
 * Delegates to tcpci_tcpm_set_polarity() which writes
 * TCPC_CONTROL (0x19) bit0 (PlugOrientation).
 * ================================================================ */
static int raa489400_set_cc_polarity(const struct device *dev,
				     enum tc_cc_polarity polarity)
{
	const struct raa489400_cfg *cfg = dev->config;

	return tcpci_tcpm_set_cc_polarity(&cfg->bus, polarity);
}

/* ================================================================
 * tcpc_driver_api: set_vconn
 *
 * Delegates to tcpci_tcpm_set_vconn() which writes
 * POWER_CONTROL (0x1C) bit0 (EnableVCONN).
 * RAA489400 uses VCONN_POWER pin (5V) for the internal VCONN MUX.
 * ================================================================ */
static int raa489400_set_vconn(const struct device *dev, bool enable)
{
	const struct raa489400_cfg *cfg = dev->config;
	struct raa489400_data *data = dev->data;
	int ret;

	ret = tcpci_tcpm_set_vconn(&cfg->bus, enable);
	if (ret == 0) {
		data->vconn_enabled = enable;
	}
	return ret;
}

/* ================================================================
 * tcpc_driver_api: set_roles
 *
 * Delegates to tcpci_tcpm_set_roles() which writes
 * MESSAGE_HEADER_INFO (0x2E) with power role and data role bits.
 * ================================================================ */
static int raa489400_set_roles(const struct device *dev,
			       enum tc_power_role power_role,
			       enum tc_data_role data_role)
{
	const struct raa489400_cfg *cfg = dev->config;

	return tcpci_tcpm_set_roles(&cfg->bus, PD_REV30, power_role, data_role);
}

/* ================================================================
 * tcpc_driver_api: set_rx_enable
 *
 * Delegates to tcpci_tcpm_set_rx_type() which writes
 * RECEIVE_DETECT (0x2F).
 * ================================================================ */
static int raa489400_set_rx_enable(const struct device *dev, bool enable)
{
	const struct raa489400_cfg *cfg = dev->config;

	/* Enable SOP + Hard Reset detection when active; 0 to disable.
	 * TCPC_REG_RX_DETECT_SOP_HRST_MASK = SOP | Hard Reset combined. */
	uint8_t rx_type = enable ? TCPC_REG_RX_DETECT_SOP_HRST_MASK : 0;

	return tcpci_tcpm_set_rx_type(&cfg->bus, rx_type);
}

/* ================================================================
 * tcpc_driver_api: get_rx_pending_msg
 *
 * Delegates to tcpci_tcpm_get_rx_pending_msg() which reads the
 * RX buffer (READABLE_BYTE_COUNT + RX_BUF_FRAME_TYPE + payload).
 * ================================================================ */
static int raa489400_get_rx_pending_msg(const struct device *dev,
					struct pd_msg *msg)
{
	const struct raa489400_cfg *cfg = dev->config;
	struct raa489400_data *data = dev->data;
	uint8_t rxbcnt;    /* READABLE_BYTE_COUNT  */
	uint8_t rxftype;   /* RX_BUF_FRAME_TYPE    */
	uint16_t rxhead;   /* first two payload bytes = PD header */
	int rx_data_size;
	int ret;

	/* ── Read the RX buffer at register TCPC_REG_RX_BUFFER (0x30) ──
	 * TCPCi Rev2.0 layout (single I2C burst from 0x30):
	 *   Byte 0: READABLE_BYTE_COUNT  = M + 2
	 *   Byte 1: RX_BUF_FRAME_TYPE
	 *   Bytes 2-3: PD message header (RX_BUF_BYTE_0, RX_BUF_BYTE_1)
	 *   Bytes 4..M+1: PD data objects
	 * ─────────────────────────────────────────────────────────── */

	/* Read byte count */
	ret = tcpci_read_reg8(&cfg->bus, TCPC_REG_RX_BUFFER, &rxbcnt);
	if (ret) {
		return ret;
	}

	/* Read frame type */
	ret = tcpci_read_reg8(&cfg->bus, TCPC_REG_RX_BUFFER + 1, &rxftype);
	if (ret) {
		return ret;
	}

	/* Read PD header (2 bytes) */
	ret = tcpci_read_reg16(&cfg->bus, TCPC_REG_RX_BUFFER + 2, &rxhead);
	if (ret) {
		return ret;
	}

	/* Data objects size = total bytes - header(2) - frame_type(1) - byte_cnt(1) */
	rx_data_size = rxbcnt - 3;
	if (rx_data_size < 0 || rx_data_size > (int)sizeof(msg->data)) {
		LOG_WRN("Invalid RX byte count %d", rxbcnt);
		tcpci_write_reg8(&cfg->bus, TCPC_REG_COMMAND,
				 TCPC_REG_COMMAND_RESET_RECEIVE_BUF);
		return -EMSGSIZE;
	}

	/* Rx frame type */
	msg->type = rxftype;

	/* Rx header */
	msg->header.raw_value = (uint16_t)rxhead;

	/* Rx data size */
	msg->len = rx_data_size;

	/* Rx data objects — burst read starting at offset +4 from TCPC_REG_RX_BUFFER */
	if (rx_data_size > 0) {
		ret = i2c_burst_read_dt(&cfg->bus,
					TCPC_REG_RX_BUFFER + 4,
					msg->data,
					rx_data_size);
		if (ret) {
			LOG_ERR("Failed to read Rx data: %d", ret);
			return ret;
		}
	}

	/* Clear RX_STATUS alert bit (W1C) */
	tcpci_write_reg16(&cfg->bus, TCPC_REG_ALERT, TCPC_REG_ALERT_RX_STATUS);

	data->msg_pending = false;
	return 0;
}

/* ================================================================
 * tcpc_driver_api: transmit_data
 *
 * Delegates to tcpci_tcpm_transmit_data() which writes the TX buffer
 * (I2C_WRITE_BYTE_COUNT + TX_BUF_BYTE_x) then TRANSMIT (0x50).
 * ================================================================ */
static int raa489400_transmit_data(const struct device *dev,
				   struct pd_msg *msg)
{
	const struct raa489400_cfg *cfg = dev->config;

	/* 3 retries per USB PD specification */
	return tcpci_tcpm_transmit_data(&cfg->bus, msg, 3);
}

/* ================================================================
 * tcpc_driver_api: get_status_register
 *
 * For vendor-defined status, reads VENDOR_STATUS (0xA0) directly.
 * All standard status registers delegate to tcpci_tcpm_get_status_register().
 * ================================================================ */
static int raa489400_get_status_register(const struct device *dev,
					 enum tcpc_status_reg reg,
					 uint32_t *status)
{
	const struct raa489400_cfg *cfg = dev->config;

	if (reg == TCPC_VENDOR_DEFINED_STATUS) {
		return tcpci_read_reg16(&cfg->bus,
					RAA489400_REG_VENDOR_STATUS,
					(uint16_t *)status);
	}
	return tcpci_tcpm_get_status_register(&cfg->bus, reg,
					      (uint16_t *)status);
}

/* ================================================================
 * tcpc_driver_api: clear_status_register
 *
 * For vendor-defined status, writes VENDOR_STATUS (0xA0) W1C.
 * All standard status registers delegate to tcpci_tcpm_clear_status_register().
 * ================================================================ */
static int raa489400_clear_status_register(const struct device *dev,
					   enum tcpc_status_reg reg,
					   uint32_t mask)
{
	const struct raa489400_cfg *cfg = dev->config;

	if (reg == TCPC_VENDOR_DEFINED_STATUS) {
		return tcpci_write_reg16(&cfg->bus,
					 RAA489400_REG_VENDOR_STATUS,
					 (uint16_t)mask);
	}
	return tcpci_tcpm_clear_status_register(&cfg->bus, reg,
						(uint16_t)mask);
}

/* ================================================================
 * tcpc_driver_api: mask_status_register
 * ================================================================ */
static int raa489400_mask_status_register(const struct device *dev,
					  enum tcpc_status_reg reg,
					  uint32_t mask)
{
	const struct raa489400_cfg *cfg = dev->config;

	if (reg == TCPC_VENDOR_DEFINED_STATUS) {
		return tcpci_write_reg16(&cfg->bus,
					 RAA489400_REG_VENDOR_STATUS_MASK,
					 (uint16_t)mask);
	}
	return tcpci_tcpm_mask_status_register(&cfg->bus, reg,
					       (uint16_t)mask);
}

/* ================================================================
 * tcpc_driver_api: dump_std_reg
 *
 * Dumps standard TCPCI registers via tcpci_tcpm_dump_std_reg(),
 * then adds RAA489400 vendor-specific registers.
 * ================================================================ */
static int raa489400_dump_std_reg(const struct device *dev)
{
	const struct raa489400_cfg *cfg = dev->config;
	uint16_t val16;
	uint8_t val8;

	/* Standard TCPCI registers via tcpci helper */
	tcpci_tcpm_dump_std_reg(&cfg->bus);

	/* RAA489400 vendor registers */
	tcpci_read_reg8(&cfg->bus,  RAA489400_REG_CHIP_REVISION,  &val8);
	LOG_INF("CHIP_REV       = 0x%02x", val8);
	tcpci_read_reg16(&cfg->bus, RAA489400_REG_VENDOR_STATUS,  &val16);
	LOG_INF("VENDOR_STATUS  = 0x%04x", val16);
	tcpci_read_reg8(&cfg->bus,  RAA489400_REG_VBUS_PATH_CTRL, &val8);
	LOG_INF("VBUS_PATH_CTRL = 0x%02x", val8);
	tcpci_read_reg16(&cfg->bus, RAA489400_REG_VBUS_FAULT_CTRL, &val16);
	LOG_INF("VBUS_FAULT_CTRL= 0x%04x", val16);
	tcpci_read_reg8(&cfg->bus,  RAA489400_REG_VCONN_FAULT_CTRL, &val8);
	LOG_INF("VCONN_FAULT    = 0x%02x", val8);

	return 0;
}



/* ================================================================
 * Alert work handler — runs in system workqueue, not ISR context
 * ================================================================ */
static void raa489400_alert_work_handler(struct k_work *work)
{
	struct raa489400_data *data =
		CONTAINER_OF(work, struct raa489400_data, alert_work);
	const struct device *dev = data->dev;
	const struct raa489400_cfg *cfg = dev->config;
	uint16_t alert_reg;
	int ret;

	/* Read the ALERT register */
	ret = tcpci_read_reg16(&cfg->bus, TCPC_REG_ALERT, &alert_reg);
	if (ret || alert_reg == 0) {
		return;
	}

	LOG_DBG("ALERT = 0x%04x", alert_reg);

	/* Handle vendor-defined alert separately before the loop */
	if (alert_reg & TCPC_REG_ALERT_VENDOR_DEF) {
		uint16_t vs;

		tcpci_read_reg16(&cfg->bus,
				 RAA489400_REG_VENDOR_STATUS, &vs);
		LOG_DBG("VENDOR_STATUS = 0x%04x", vs);
		/* Clear vendor status (W1C) */
		tcpci_write_reg16(&cfg->bus,
				  RAA489400_REG_VENDOR_STATUS, vs);
		/* Clear vendor bit in ALERT */
		tcpci_write_reg16(&cfg->bus,
				  TCPC_REG_ALERT,
				  TCPC_REG_ALERT_VENDOR_DEF);

		if (data->alert_handler) {
			data->alert_handler(dev, data->alert_handler_data,
					    TCPC_ALERT_VENDOR_DEFINED);
		}
		alert_reg &= ~TCPC_REG_ALERT_VENDOR_DEF;
	}

	/* Convert remaining alert bits to tcpc_alert enum one at a time */
	while (alert_reg) {
		enum tcpc_alert alert = tcpci_alert_reg_to_enum(alert_reg);
		uint16_t bit;

		/* Find the bit that corresponds to this alert */
		switch (alert) {
		case TCPC_ALERT_CC_STATUS:
			bit = TCPC_REG_ALERT_CC_STATUS;
			break;
		case TCPC_ALERT_POWER_STATUS:
			bit = TCPC_REG_ALERT_POWER_STATUS;
			break;
		case TCPC_ALERT_MSG_STATUS:
			bit = TCPC_REG_ALERT_RX_STATUS;
			data->msg_pending = true;
			break;
		case TCPC_ALERT_HARD_RESET_RECEIVED:
			bit = TCPC_REG_ALERT_RX_HARD_RST;
			break;
		case TCPC_ALERT_TRANSMIT_MSG_SUCCESS:
		case TCPC_ALERT_TRANSMIT_MSG_FAILED:
		case TCPC_ALERT_TRANSMIT_MSG_DISCARDED:
			bit = TCPC_REG_ALERT_TX_COMPLETE;
			break;
		case TCPC_ALERT_FAULT_STATUS:
			bit = TCPC_REG_ALERT_FAULT;
			break;
		case TCPC_ALERT_VBUS_SNK_DISCONNECT:
			bit = TCPC_REG_ALERT_VBUS_DISCNCT;
			break;
		default:
			/* Clear entire remaining register to avoid loop */
			tcpci_write_reg16(&cfg->bus,
					  TCPC_REG_ALERT, alert_reg);
			alert_reg = 0;
			continue;
		}

		/* Clear just this alert bit (W1C) */
		tcpci_write_reg16(&cfg->bus, TCPC_REG_ALERT, bit);
		alert_reg &= ~bit;

		/* Notify the USB-C stack */
		if (data->alert_handler) {
			data->alert_handler(dev, data->alert_handler_data,
					    alert);
		}
	}
}

/* ================================================================
 * GPIO ISR — fires when ALERT# falls; schedules work item
 * ================================================================ */
static void raa489400_alert_gpio_cb(const struct device *port,
				    struct gpio_callback *cb,
				    gpio_port_pins_t pins)
{
	struct raa489400_data *data =
		CONTAINER_OF(cb, struct raa489400_data, alert_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_work_submit(&data->alert_work);
}

/* ================================================================
 * tcpc_driver_api: set_alert_handler_cb
 * ================================================================ */
static int raa489400_set_alert_handler_cb(const struct device *dev,
					  tcpc_alert_handler_cb_t handler,
					  void *handler_data)
{
	struct raa489400_data *data = dev->data;

	data->alert_handler = handler;
	data->alert_handler_data = handler_data;
	return 0;
}




/* ================================================================
 * tcpc_driver_api: get_snk_ctrl
 *
 * Queries whether the RAA489400 is currently sinking VBUS by reading
 * POWER_STATUS (0x1E) bit0 (SinkingVBUS).
 * Returns true if sinking, false if not, negative on error.
 * ================================================================ */
static int raa489400_get_snk_ctrl(const struct device *dev)
{
	const struct raa489400_cfg *cfg = dev->config;
	uint8_t pwr_status;
	int ret;

	ret = tcpci_read_reg8(&cfg->bus,
			      RAA489400_REG_POWER_STATUS,
			      &pwr_status);
	if (ret) {
		return ret;
	}
	return !!(pwr_status & RAA489400_PWR_STATUS_SINKING_VBUS);
}

/* ================================================================
 * tcpc_driver_api: set_snk_ctrl
 *
 * Enables or disables the VBUS sink path by writing COMMAND (0x23):
 *   0x55 = SinkVbus        RAA489400 enables VSNK_GATE → N-FET on
 *   0x44 = DisableSinkVbus RAA489400 disables VSNK_GATE → N-FET off
 *
 * Per RAA489400 datasheet Table 30 (Section 7.2.21).
 * ================================================================ */
static int raa489400_set_snk_ctrl(const struct device *dev, bool enable)
{
	const struct raa489400_cfg *cfg = dev->config;
	uint8_t cmd = enable ? RAA489400_CMD_SINK_VBUS         /* 0x55 */
			     : RAA489400_CMD_DISABLE_SINK_VBUS; /* 0x44 */

	LOG_DBG("SNK ctrl: %s", enable ? "enable" : "disable");
	return tcpci_write_reg8(&cfg->bus, TCPC_REG_COMMAND, cmd);
}

/* ================================================================
 * tcpc_driver_api: get_src_ctrl
 *
 * Queries whether the RAA489400 is currently sourcing VBUS by reading
 * POWER_STATUS (0x1E) bit4 (SourcingVBUS).
 * Returns true if sourcing, false if not, negative on error.
 * ================================================================ */
static int raa489400_get_src_ctrl(const struct device *dev)
{
	const struct raa489400_cfg *cfg = dev->config;
	uint8_t pwr_status;
	int ret;

	ret = tcpci_read_reg8(&cfg->bus,
			      RAA489400_REG_POWER_STATUS,
			      &pwr_status);
	if (ret) {
		return ret;
	}
	return !!(pwr_status & RAA489400_PWR_STATUS_SOURCING_VBUS);
}

/* ================================================================
 * tcpc_driver_api: set_src_ctrl
 *
 * Enables or disables the VBUS source path by writing COMMAND (0x23):
 *   0x77 = SourceVbusDefaultVoltage  enables VSRC_GATE (5V source)
 *   0x66 = DisableSourceVbus         disables VSRC_GATE
 *
 * Note: VBUS_CP_CTRL.VSRC_CP_EN (0x88 bit1) must be set before
 * enabling source so the VSRC_GATE charge pump is ready. The charge
 * pump takes up to 50ms to stabilize (datasheet Section 3.4).
 * Per RAA489400 datasheet Table 30 (Section 7.2.21).
 * ================================================================ */
static int raa489400_set_src_ctrl(const struct device *dev, bool enable)
{
	const struct raa489400_cfg *cfg = dev->config;
	int ret;

	if (enable) {
		/* Enable VSRC_GATE charge pump before sourcing */
		ret = tcpci_update_reg8(&cfg->bus,
					RAA489400_REG_VBUS_CP_CTRL,
					RAA489400_VBUS_CP_VSRC_CP_EN,
					RAA489400_VBUS_CP_VSRC_CP_EN);
		if (ret) {
			return ret;
		}
	}

	uint8_t cmd = enable ? RAA489400_CMD_SOURCE_VBUS_DEFAULT  /* 0x77 */
			     : RAA489400_CMD_DISABLE_SOURCE_VBUS;  /* 0x66 */

	LOG_DBG("SRC ctrl: %s", enable ? "enable" : "disable");
	ret = tcpci_write_reg8(&cfg->bus, TCPC_REG_COMMAND, cmd);

	if (!enable) {
		/* Disable charge pump after turning source off */
		tcpci_update_reg8(&cfg->bus,
				  RAA489400_REG_VBUS_CP_CTRL,
				  RAA489400_VBUS_CP_VSRC_CP_EN,
				  0);
	}
	return ret;
}

/* ================================================================
 * tcpc_driver_api: get_chip_info
 *
 * Reads VENDOR_ID, PRODUCT_ID and DEVICE_ID from the RAA489400
 * and populates tcpc_chip_info. Firmware version is not stored in
 * a register so fw_version_number is set to CHIP_REVISION (0x80).
 * ================================================================ */
static int raa489400_get_chip_info(const struct device *dev,
				   struct tcpc_chip_info *chip_info)
{
	const struct raa489400_cfg *cfg = dev->config;
	uint16_t val;
	uint8_t rev;
	int ret;

	ret = tcpci_read_reg16(&cfg->bus, RAA489400_REG_VENDOR_ID, &val);
	if (ret) {
		return ret;
	}
	chip_info->vendor_id = val;

	ret = tcpci_read_reg16(&cfg->bus, RAA489400_REG_PRODUCT_ID, &val);
	if (ret) {
		return ret;
	}
	chip_info->product_id = val;

	ret = tcpci_read_reg16(&cfg->bus, RAA489400_REG_DEVICE_ID, &val);
	if (ret) {
		return ret;
	}
	chip_info->device_id = val;

	ret = tcpci_read_reg8(&cfg->bus, RAA489400_REG_CHIP_REVISION, &rev);
	if (ret) {
		return ret;
	}
	chip_info->fw_version_number = rev;

	return 0;
}

/* ================================================================
 * tcpc_driver_api: set_low_power_mode
 *
 * Enables/disables the RAA489400 24MHz oscillator to save power
 * when no USB-C device is connected, via TYPE_C_PARAMETER (0xE2)
 * bit13 (Stop_24MHz_OSC). Per datasheet Section 5.8.
 * ================================================================ */
static int raa489400_set_low_power_mode(const struct device *dev, bool enable)
{
	const struct raa489400_cfg *cfg = dev->config;

	return tcpci_update_reg8(&cfg->bus,
				 RAA489400_REG_TYPE_C_PARAMETER + 1,
				 BIT(5), /* bit13 of 16-bit reg = bit5 of high byte */
				 enable ? BIT(5) : 0);
}

/* ================================================================
 * tcpc_driver_api: sop_prime_enable
 *
 * Enables/disables SOP' and SOP'' message reception by writing
 * RECEIVE_DETECT (0x2F) bits [1] and [2].
 * ================================================================ */
static int raa489400_sop_prime_enable(const struct device *dev, bool enable)
{
	const struct raa489400_cfg *cfg = dev->config;
	uint8_t mask = BIT(1) | BIT(2); /* SOP' and SOP'' bits */

	return tcpci_update_reg8(&cfg->bus,
				 TCPC_REG_RX_DETECT,
				 mask,
				 enable ? mask : 0);
}

/* ================================================================
 * tcpc_driver_api: set_bist_test_mode
 *
 * Enables/disables BIST Test Mode via TCPC_CONTROL (0x19) bit1.
 * Per RAA489400 datasheet Section 7.2.12.
 * Note: TYPE_C_PARAMETER Force_Enable_24MHz must be set to 1b
 * before starting BIST mode per datasheet Section 7.2.63.
 * ================================================================ */
static int raa489400_set_bist_test_mode(const struct device *dev, bool enable)
{
	const struct raa489400_cfg *cfg = dev->config;
	int ret;

	if (enable) {
		/* Force 24MHz clock on during BIST */
		ret = tcpci_update_reg8(&cfg->bus,
					RAA489400_REG_TYPE_C_PARAMETER + 1,
					BIT(6),
					BIT(6));
		if (ret) {
			return ret;
		}
	}

	ret = tcpci_update_reg8(&cfg->bus,
				TCPC_REG_TCPC_CTRL,
				RAA489400_TCPC_CTRL_BIST_TEST_MODE,
				enable ? RAA489400_TCPC_CTRL_BIST_TEST_MODE : 0);

	if (!enable) {
		/* Restore automatic power saving */
		tcpci_update_reg8(&cfg->bus,
				  RAA489400_REG_TYPE_C_PARAMETER + 1,
				  BIT(6), 0);
	}
	return ret;
}

/* ================================================================
 * tcpc_driver_api: vconn_discharge
 *
 * Controls VCONN discharge. The RAA489400 handles VCONN discharge
 * automatically when VCONN is disabled via set_vconn(false).
 * This function provides manual control if needed.
 * ================================================================ */
static int raa489400_vconn_discharge(const struct device *dev, bool enable)
{
	/* RAA489400 discharges VCONN automatically when EnableVCONN=0.
	 * No separate discharge register is needed for normal operation. */
	ARG_UNUSED(dev);
	ARG_UNUSED(enable);
	return 0;
}

/* ================================================================
 * tcpc_driver_api: set_vconn_cb / set_vconn_discharge_cb
 * ================================================================ */
static void raa489400_set_vconn_cb(const struct device *dev,
				   tcpc_vconn_control_cb_t vconn_cb,
				   const struct device *usbc_dev)
{
	/* RAA489400 has an internal VCONN MUX controlled via POWER_CONTROL
	 * register. Signature: tcpc_api_set_vconn_cb_t (usbc_tcpc.h line 219) */
	ARG_UNUSED(dev);
	ARG_UNUSED(vconn_cb);
	ARG_UNUSED(usbc_dev);
}

static void raa489400_set_vconn_discharge_cb(const struct device *dev,
					     tcpc_vconn_discharge_cb_t cb,
					     const struct device *usbc_dev)
{
	/* RAA489400 auto-discharges VCONN when EnableVCONN is cleared.
	 * Signature: tcpc_api_set_vconn_discharge_cb_t (usbc_tcpc.h line 210) */
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(usbc_dev);
}


/* ================================================================
 * tcpc_driver_api: set_debug_accessory
 * Not needed for sink-only operation. Stub returns 0.
 * ================================================================ */
static int raa489400_set_debug_accessory(const struct device *dev, bool enable)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(enable);
	return 0;
}

/* ================================================================
 * tcpc_driver_api: set_debug_detach
 * Not needed for sink-only operation. Stub returns 0.
 * ================================================================ */
static int raa489400_set_debug_detach(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

/* ================================================================
 * tcpc_driver_api: set_drp_toggle
 *
 * Enables/disables DRP (Dual Role Power) toggling by writing
 * ROLE_CONTROL (0x1A) bit6 (DRP) and issuing Look4Connection.
 * ================================================================ */
static int raa489400_set_drp_toggle(const struct device *dev, bool enable)
{
	const struct raa489400_cfg *cfg = dev->config;
	int ret;

	ret = tcpci_update_reg8(&cfg->bus,
				RAA489400_REG_ROLE_CONTROL,
				RAA489400_ROLE_CTRL_DRP,
				enable ? RAA489400_ROLE_CTRL_DRP : 0);
	if (ret) {
		return ret;
	}

	if (enable) {
		return tcpci_write_reg8(&cfg->bus,
					TCPC_REG_COMMAND,
					TCPC_REG_COMMAND_LOOK4CONNECTION);
	}
	return 0;
}

/* ================================================================
 * Driver API table
 * ================================================================ */
static const struct tcpc_driver_api raa489400_driver_api = {
	/* Matches struct tcpc_driver_api in usbc_tcpc.h exactly */
	.init                   = raa489400_init,
	.get_cc                 = raa489400_get_cc,
	.select_rp_value        = raa489400_select_rp_value,
	.get_rp_value           = raa489400_get_rp_value,
	.set_cc                 = raa489400_set_cc,
	.set_vconn_discharge_cb = raa489400_set_vconn_discharge_cb,
	.set_vconn_cb           = raa489400_set_vconn_cb,
	.vconn_discharge        = raa489400_vconn_discharge,
	.set_vconn              = raa489400_set_vconn,
	.set_roles              = raa489400_set_roles,
	.get_rx_pending_msg     = raa489400_get_rx_pending_msg,
	.set_rx_enable          = raa489400_set_rx_enable,
	.set_cc_polarity        = raa489400_set_cc_polarity,
	.transmit_data          = raa489400_transmit_data,
	.dump_std_reg           = raa489400_dump_std_reg,
	.get_status_register    = raa489400_get_status_register,
	.clear_status_register  = raa489400_clear_status_register,
	.mask_status_register   = raa489400_mask_status_register,
	.set_debug_accessory    = raa489400_set_debug_accessory,
	.set_debug_detach       = raa489400_set_debug_detach,
	.set_drp_toggle         = raa489400_set_drp_toggle,
	.get_snk_ctrl           = raa489400_get_snk_ctrl,
	.set_snk_ctrl           = raa489400_set_snk_ctrl,
	.get_src_ctrl           = raa489400_get_src_ctrl,
	.set_src_ctrl           = raa489400_set_src_ctrl,
	.get_chip_info          = raa489400_get_chip_info,
	.set_low_power_mode     = raa489400_set_low_power_mode,
	.sop_prime_enable       = raa489400_sop_prime_enable,
	.set_bist_test_mode     = raa489400_set_bist_test_mode,
	.set_alert_handler_cb   = raa489400_set_alert_handler_cb,
};

/* ================================================================
 * Per-instance registration macros
 * ================================================================ */
#define RAA489400_DRIVER_CFG_INIT(inst)                                \
	{                                                              \
		.bus        = I2C_DT_SPEC_INST_GET(inst),              \
		.alert_gpio = GPIO_DT_SPEC_INST_GET(inst, alert_gpios),\
	}

#define RAA489400_DRIVER_DATA_INIT(inst)                               \
	{                                                              \
		.vconn_enabled = false,                                \
		.msg_pending   = false,                                \
	}

#define RAA489400_DEVICE_INIT(inst)                                    \
	static const struct raa489400_cfg raa489400_cfg_##inst =       \
		RAA489400_DRIVER_CFG_INIT(inst);                       \
	static struct raa489400_data raa489400_data_##inst =           \
		RAA489400_DRIVER_DATA_INIT(inst);                      \
	DEVICE_DT_INST_DEFINE(inst,                                    \
			      raa489400_init,                          \
			      NULL,                                    \
			      &raa489400_data_##inst,                  \
			      &raa489400_cfg_##inst,                   \
			      POST_KERNEL,                             \
			      CONFIG_USBC_TCPC_INIT_PRIORITY,          \
			      &raa489400_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RAA489400_DEVICE_INIT)
