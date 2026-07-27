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
 * Board VBUS-path / GPIO configuration
 *
 * Taken verbatim (bit-for-bit) from the verified bare-metal driver
 * that was validated on the RAA489400 evaluation board. On this
 * board the VBUS SINK power switch is driven through GPIO1, so GPIO1
 * must be a push-pull output-enable, GPIO2 an input, and the VBUS
 * path steered to GPIO1 with the sink switch enabled. Without this
 * the sink FET is never in circuit and no stable power contract can
 * form even though CC detection and I2C work.
 *
 * These bit definitions are not in raa489400.h (read-only), so they
 * are reproduced here with the datasheet register/section numbers.
 * ---------------------------------------------------------------- */
/* 7.2.41 GPIOx_CTRL (82h/83h) */
#define RAA_GPIO_IE        (1 << 4)   /* input enable                 */
#define RAA_GPIO_PP        (1 << 3)   /* output push-pull             */
#define RAA_GPIO_OE        (1 << 2)   /* output enable (pin=GPIO_O)   */
/* 7.2.46 VBUS_GPIO_CTRL (87h) */
#define RAA_VBUS_GPIO_EN_SNK   (1 << 1)   /* loaded to GPIOn_O for sink */

/* Concrete values written during init (see raa489400_board_vbus_init) */
#define RAA_GPIO1_CTRL_VAL     (RAA_GPIO_PP | RAA_GPIO_OE)              /* 0x0C */
#define RAA_GPIO2_CTRL_VAL     (RAA_GPIO_IE)                           /* 0x10 */
#define RAA_VBUS_PATH_CTRL_VAL (RAA489400_VBUS_PATH_SNK_SEL_GPIO1 | \
				RAA489400_VBUS_PATH_SNK_EN)           /* 0x03 */
#define RAA_VBUS_GPIO_CTRL_VAL (RAA_VBUS_GPIO_EN_SNK)                  /* 0x02 */


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
	/** Serializes all I2C access to the chip between the alert work
	 *  handler (system workqueue thread) and the TCPC API calls made
	 *  by the USB-C stack thread. Without this, the two contexts issue
	 *  overlapping transfers on the shared bus and the RA IIC master
	 *  reports "Another transfer was in progress" / "Write failed". */
	struct k_mutex bus_lock;
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
	/** True once init has fully completed and alerts may be serviced.
	 *  Guards the alert work handler against touching I2C while init
	 *  is still mid-sequence (the two would collide on the bus). */
	bool initialized;
};

/* ================================================================
 * Forward declarations
 * ================================================================ */
static void raa489400_alert_work_handler(struct k_work *work);
static void raa489400_alert_gpio_cb(const struct device *port,
				    struct gpio_callback *cb,
				    gpio_port_pins_t pins);


static int raa489400_tcpc_init(const struct device *dev)
{
	struct raa489400_data *data = dev->data;

	if (!data->initialized) {
		// if (data->init_retries > CONFIG_USBC_TCPC_PS8XXX_INIT_RETRIES) {
		// 	LOG_ERR("TCPC was not initialized correctly");
		// 	return -EIO;
		// }
		return -EAGAIN;
	}
	LOG_INF("RAA489400 TCPC already initialized");
	return 0;
}
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
 * Board VBUS-path / GPIO static configuration
 *
 * Mirrors "Phase 1: Static configuration" of the verified bare-metal
 * tcpci_init(). Must run once after the chip is identified and vendor
 * registers are applied, before attach detection begins.
 * ================================================================ */
static int raa489400_board_vbus_init(const struct device *dev)
{
	const struct raa489400_cfg *cfg = dev->config;
	int ret;

	ret = tcpci_write_reg8(&cfg->bus, RAA489400_REG_GPIO1_CTRL,
			       RAA_GPIO1_CTRL_VAL);
	if (ret) {
		return ret;
	}
	ret = tcpci_write_reg8(&cfg->bus, RAA489400_REG_GPIO2_CTRL,
			       RAA_GPIO2_CTRL_VAL);
	if (ret) {
		return ret;
	}
	ret = tcpci_write_reg8(&cfg->bus, RAA489400_REG_VBUS_PATH_CTRL,
			       RAA_VBUS_PATH_CTRL_VAL);
	if (ret) {
		return ret;
	}
	ret = tcpci_write_reg8(&cfg->bus, RAA489400_REG_VBUS_GPIO_CTRL,
			       RAA_VBUS_GPIO_CTRL_VAL);
	if (ret) {
		return ret;
	}

	LOG_DBG("VBUS sink path configured via GPIO1 "
		"(GPIO1=0x%02x PATH=0x%02x GPIO_CTRL=0x%02x)",
		RAA_GPIO1_CTRL_VAL, RAA_VBUS_PATH_CTRL_VAL,
		RAA_VBUS_GPIO_CTRL_VAL);
	return 0;
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

	/* 4a. Board VBUS-path / GPIO static config (verified bare-metal
	 *     Phase 1). On this eval board the sink FET is driven via
	 *     GPIO1; without this the power path is never in circuit. */
	// ret = raa489400_board_vbus_init(dev);
	// if (ret) {
	// 	return ret;
	// }

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

	/* 6a. Enable VBUS voltage monitoring.
	 *
	 * POWER_CONTROL (0x1C) bit6 (VBUS_VOLTAGE Monitor) has a reset
	 * value of 1b (monitoring DISABLED) — POWER_CONTROL resets to
	 * 0x62. While this bit is set, VBUS_VOLTAGE (0x70) reads all
	 * zeroes. The zephyr,usb-c-vbus-tcpci VBUS driver reads VBUS
	 * over I2C from register 0x70, so this bit MUST be cleared or
	 * the USB-C stack never sees VBUS present.
	 *
	 * Clearing bit6 (write 0) enables VBUS voltage monitoring.
	 */
	ret = tcpci_update_reg8(&cfg->bus,
				RAA489400_REG_POWER_CONTROL,
				RAA489400_PWR_CTRL_VBUS_MON_DIS,
				0);
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
	k_mutex_init(&data->bus_lock);
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

	/* Clear any alert bits latched during init so ALERT# starts
	 * deasserted, THEN mark the driver ready. Both must happen
	 * before the interrupt is enabled: if the IRQ fired now, the
	 * work handler would issue I2C while this init sequence is
	 * still running, and the two would collide on the bus
	 * ("Another transfer was in progress"). */
	tcpci_write_reg16(&cfg->bus, TCPC_REG_ALERT, 0xFFFF);
	data->initialized = true;

	LOG_INF("RAA489400 TCPC initialized (addr 0x%02x)", cfg->bus.addr);

	/* NOW enable the interrupt — init I2C is finished. */
	ret = gpio_pin_interrupt_configure_dt(&cfg->alert_gpio,
					      GPIO_INT_EDGE_FALLING);
	if (ret) {
		/* Not fatal for bring-up: the USB-C stack still polls
		 * CC/power periodically, so the port can operate (with
		 * higher latency) even if the ALERT# line can't raise an
		 * interrupt on this board. */
		LOG_WRN("ALERT# IRQ unavailable (%d); running without interrupt",
			ret);
		return 0;
	}

	/* An alert may have latched between the 0xFFFF clear above and
	 * the interrupt being enabled. With an edge-triggered line that
	 * produces no new edge, so drain once explicitly. */
	k_work_submit(&data->alert_work);

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

	k_mutex_lock(&data->bus_lock, K_FOREVER);
	ret = tcpci_tcpm_get_cc(&cfg->bus, cc1, cc2);
	if (ret == 0) {
		data->cc1 = *cc1;
		data->cc2 = *cc2;
	}
	k_mutex_unlock(&data->bus_lock);
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
	struct raa489400_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->bus_lock, K_FOREVER);
	ret = tcpci_tcpm_set_cc(&cfg->bus, pull);
	k_mutex_unlock(&data->bus_lock);
	return ret;
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
	struct raa489400_data *data = dev->data;
	int ret;

	/* Enable SOP + Hard Reset detection when active; 0 to disable.
	 * TCPC_REG_RX_DETECT_SOP_HRST_MASK = SOP | Hard Reset combined. */
	uint8_t rx_type = enable ? TCPC_REG_RX_DETECT_SOP_HRST_MASK : 0;

	k_mutex_lock(&data->bus_lock, K_FOREVER);

	/* The RAA489400 (like other FUSB307B-family TCPCs) can silently
	 * fail to latch RECEIVE_DETECT (0x2F) on the first write, after
	 * which the BMC receiver never arms and every incoming PD message
	 * is dropped — the sink then times out in Wait_For_Capabilities
	 * and hard-resets in a loop. Write, read back, and retry until it
	 * sticks (bounded), so RX is guaranteed enabled. */
	for (int attempt = 0; attempt < 3; attempt++) {
		uint8_t rb = 0xFF;

		ret = tcpci_tcpm_set_rx_type(&cfg->bus, rx_type);
		if (ret) {
			continue;
		}

		ret = tcpci_read_reg8(&cfg->bus, TCPC_REG_RX_DETECT, &rb);
		if (ret == 0 && rb == rx_type) {
			break;   /* confirmed */
		}
		LOG_WRN("RECEIVE_DETECT wrote 0x%02x read 0x%02x (retry %d)",
			rx_type, rb, attempt);
		ret = -EIO;
	}

	k_mutex_unlock(&data->bus_lock);
	return ret;
}

/* ================================================================
 * tcpc_driver_api: get_rx_pending_msg
 *
 * Delegates to tcpci_tcpm_get_rx_pending_msg() which reads the
 * RX buffer (READABLE_BYTE_COUNT + RX_BUF_FRAME_TYPE + payload).
 * ================================================================ */
// static int raa489400_get_rx_pending_msg(const struct device *dev,
// 					struct pd_msg *msg)
// {
// 	const struct raa489400_cfg *cfg = dev->config;
// 	struct raa489400_data *data = dev->data;
// 	uint8_t rxbcnt;    /* READABLE_BYTE_COUNT  */
// 	uint8_t rxftype;   /* RX_BUF_FRAME_TYPE    */
// 	uint16_t rxhead;   /* first two payload bytes = PD header */
// 	int rx_data_size;
// 	int ret;

// 	k_mutex_lock(&data->bus_lock, K_FOREVER);

// 	/* ── Read the RX buffer at register TCPC_REG_RX_BUFFER (0x30) ──
// 	 * TCPCi Rev2.0 layout (single I2C burst from 0x30):
// 	 *   Byte 0: READABLE_BYTE_COUNT  = M + 2
// 	 *   Byte 1: RX_BUF_FRAME_TYPE
// 	 *   Bytes 2-3: PD message header (RX_BUF_BYTE_0, RX_BUF_BYTE_1)
// 	 *   Bytes 4..M+1: PD data objects
// 	 * ─────────────────────────────────────────────────────────── */

// 	/* Read byte count */
// 	ret = tcpci_read_reg8(&cfg->bus, TCPC_REG_RX_BUFFER, &rxbcnt);
// 	if (ret) {
// 		k_mutex_unlock(&data->bus_lock);
// 		return ret;
// 	}

// 	/* Read frame type */
// 	ret = tcpci_read_reg8(&cfg->bus, TCPC_REG_RX_BUFFER + 1, &rxftype);
// 	if (ret) {
// 		k_mutex_unlock(&data->bus_lock);
// 		return ret;
// 	}

// 	/* Read PD header (2 bytes) */
// 	ret = tcpci_read_reg16(&cfg->bus, TCPC_REG_RX_BUFFER + 2, &rxhead);
// 	if (ret) {
// 		k_mutex_unlock(&data->bus_lock);
// 		return ret;
// 	}

// 	/* Data objects size = total bytes - header(2) - frame_type(1) - byte_cnt(1) */
// 	rx_data_size = rxbcnt - 3;
// 	if (rx_data_size < 0 || rx_data_size > (int)sizeof(msg->data)) {
// 		LOG_WRN("Invalid RX byte count %d", rxbcnt);
// 		tcpci_write_reg8(&cfg->bus, TCPC_REG_COMMAND,
// 				 TCPC_REG_COMMAND_RESET_RECEIVE_BUF);
// 		k_mutex_unlock(&data->bus_lock);
// 		return -EMSGSIZE;
// 	}

// 	/* Rx frame type */
// 	msg->type = rxftype;

// 	/* Rx header */
// 	msg->header.raw_value = (uint16_t)rxhead;

// 	/* Rx data size */
// 	msg->len = rx_data_size;

// 	/* Rx data objects — burst read starting at offset +4 from TCPC_REG_RX_BUFFER */
// 	if (rx_data_size > 0) {
// 		ret = i2c_burst_read_dt(&cfg->bus,
// 					TCPC_REG_RX_BUFFER + 4,
// 					msg->data,
// 					rx_data_size);
// 		if (ret) {
// 			LOG_ERR("Failed to read Rx data: %d", ret);
// 			k_mutex_unlock(&data->bus_lock);
// 			return ret;
// 		}
// 	}

// 	/* Clear RX_STATUS alert bit (W1C) */
// 	tcpci_write_reg16(&cfg->bus, TCPC_REG_ALERT, TCPC_REG_ALERT_RX_STATUS);

// 	data->msg_pending = false;
// 	k_mutex_unlock(&data->bus_lock);
// 	return 0;
// }

/*
 * Read and decode the TCPCI receive buffer beginning at 0x30.
 * This Zephyr revision does not provide a
 * tcpci_tcpm_get_rx_pending_msg() helper.
 */
// #include <zephyr/sys/byteorder.h>
// #include <string.h>

// static int raa489400_get_rx_pending_msg(const struct device *dev,
//                     struct pd_msg *msg)
// {
//     const struct raa489400_cfg *cfg = dev->config;
//     struct raa489400_data *data = dev->data;
//     uint8_t buf[4 + sizeof(msg->data)];
//     uint8_t byte_count;
//     size_t transfer_size;
//     size_t data_size;
//     int ret;

//     k_mutex_lock(&data->bus_lock, K_FOREVER);
//     if (!data->msg_pending) {
//         ret = -ENODATA;
//         goto out;
//     }
//     /*
//      * TCPCI RX buffer:
//      *
//      * 0x30: READABLE_BYTE_COUNT
//      * 0x31: RX_BUF_FRAME_TYPE
//      * 0x32: PD header byte 0
//      * 0x33: PD header byte 1
//      * 0x34: first data-object byte
//      *
//      * READABLE_BYTE_COUNT covers:
//      *   frame type + 2-byte PD header + PD data bytes.
//      */
//     ret = tcpci_read_reg8(&cfg->bus, TCPC_REG_RX_BUFFER,
//                   &byte_count);
//     if (ret != 0) {
//         LOG_ERR("Failed to read RX byte count: %d", ret);
//         goto out;
//     }

//     if (byte_count < 3U ||
//         byte_count > (3U + sizeof(msg->data))) {
//         LOG_ERR("Invalid RX byte count: %u", byte_count);

//         (void)tcpci_write_reg8(
//             &cfg->bus,
//             TCPC_REG_COMMAND,
//             TCPC_REG_COMMAND_RESET_RECEIVE_BUF);

//         ret = -EMSGSIZE;
//         goto out;
//     }

//     /*
//      * Include the READABLE_BYTE_COUNT byte itself.
//      */
//     transfer_size = (size_t)byte_count + 1U;

//     /*
//      * Read the complete RX message from 0x30 in one burst. This
//      * avoids separately reading frame type, header, and payload.
//      */
//     ret = i2c_burst_read_dt(&cfg->bus, TCPC_REG_RX_BUFFER,
//                 buf, transfer_size);
//     if (ret != 0) {
//         LOG_ERR("Failed to burst-read RX buffer: %d", ret);
//         goto out;
//     }

//     /*
//      * Verify that the count did not change between the count read
//      * and the complete buffer read.
//      */
//     if (buf[0] != byte_count) {
//         LOG_WRN("RX byte count changed: first=%u burst=%u",
//             byte_count, buf[0]);
//         ret = -EAGAIN;
//         goto out;
//     }

//     data_size = (size_t)byte_count - 3U;

//     msg->type = buf[1];
//     msg->header.raw_value = sys_get_le16(&buf[2]);
//     msg->len = data_size;

//     if (data_size != 0U) {
//         memcpy(msg->data, &buf[4], data_size);
//     }

//     data->msg_pending = false;

//     LOG_DBG("RX: count=%u type=0x%02x header=0x%04x data=%u",
//         byte_count, msg->type, msg->header.raw_value,
//         (unsigned int)data_size);

//     ret = 0;

// out:
//     k_mutex_unlock(&data->bus_lock);
//     return ret;
// }
#include <zephyr/sys/byteorder.h>
#include <string.h>

static int raa489400_get_rx_pending_msg(const struct device *dev,
                    struct pd_msg *msg)
{
    const struct raa489400_cfg *cfg = dev->config;
    struct raa489400_data *data = dev->data;
    uint8_t buf[4 + sizeof(msg->data)];
    uint8_t count;
    size_t total;
    size_t payload_len;
    int ret;

    k_mutex_lock(&data->bus_lock, K_FOREVER);

    if (!data->msg_pending) {
        ret = -ENODATA;
        goto out;
    }

    /*
     * Read READABLE_BYTE_COUNT first. Do not clear RX_STATUS
     * until the complete message has been copied.
     */
    ret = tcpci_read_reg8(&cfg->bus, TCPC_REG_RX_BUFFER, &count);
    if (ret != 0) {
        LOG_ERR("Failed to read RX byte count: %d", ret);
        goto out;
    }

    /*
     * Count includes:
     *   RX_BUF_FRAME_TYPE: 1 byte
     *   PD header:         2 bytes
     *   payload:           0..28 bytes
     */
    if ((count < 3U) || (count > (3U + sizeof(msg->data)))) {
        LOG_WRN("Invalid RX byte count: %u", count);

        /*
         * Discard the invalid RX buffer and acknowledge RX_STATUS,
         * otherwise the stack may repeatedly request the same
         * invalid message.
         */
        (void)tcpci_write_reg8(&cfg->bus,
                      TCPC_REG_COMMAND,
                      TCPC_REG_COMMAND_RESET_RECEIVE_BUF);

        (void)tcpci_write_reg16(&cfg->bus,
                       TCPC_REG_ALERT,
                       TCPC_REG_ALERT_RX_STATUS);

        data->msg_pending = false;
        ret = -EBADMSG;
        goto out;
    }

    /*
     * Include the count byte itself:
     *   buf[0] = count
     *   buf[1] = frame type
     *   buf[2..3] = header
     *   buf[4..] = data objects
     */
    total = (size_t)count + 1U;

    ret = i2c_burst_read_dt(&cfg->bus,
                TCPC_REG_RX_BUFFER,
                buf,
                total);
    if (ret != 0) {
        LOG_ERR("Failed to read RX buffer: %d", ret);
        goto out;
    }

    if (buf[0] != count) {
        LOG_WRN("RX count changed: initial=%u burst=%u",
            count, buf[0]);
        ret = -EAGAIN;
        goto out;
    }

    payload_len = (size_t)count - 3U;

    msg->type = buf[1];
    msg->header.raw_value = sys_get_le16(&buf[2]);
    msg->len = payload_len;

    if (payload_len != 0U) {
        memcpy(msg->data, &buf[4], payload_len);
    }

    // LOG_INF("RX count=%u type=0x%02x header=0x%04x payload=%u",
    //     count, msg->type, msg->header.raw_value,
    //     (unsigned int)payload_len);
	LOG_INF("RX count=%u type=0x%02x header=0x%04x payload=%u PDO0=0x%08x",
		count,
		msg->type,
		msg->header.raw_value,
		(unsigned int)payload_len,
		payload_len >= 4U ? msg->data[0] : 0U);

	uint8_t ndo = (msg->header.raw_value >> 12) & 0x7U;

	if ((size_t)ndo * sizeof(uint32_t) != payload_len) {
		LOG_ERR("RX length mismatch: NDO=%u payload=%u",
			ndo, (unsigned int)payload_len);
		ret = -EBADMSG;
		goto out;
	}
    /*
     * Message has now been copied safely. Release the TCPC RX
     * buffer by clearing RX_STATUS.
     */
    ret = tcpci_write_reg16(&cfg->bus,
                TCPC_REG_ALERT,
                TCPC_REG_ALERT_RX_STATUS);
    if (ret != 0) {
        LOG_ERR("Failed to clear RX_STATUS: %d", ret);
        goto out;
    }

	LOG_INF("RX returning success: type=0x%02x header=0x%04x len=%u",
		msg->type, msg->header.raw_value, msg->len);
	
	data->msg_pending = false;
	ret = 0;

out:
    k_mutex_unlock(&data->bus_lock);
    return ret;
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
	struct raa489400_data *data = dev->data;
	int ret;

    LOG_INF("TX request: type=0x%02x header=0x%04x len=%u",
        msg->type, msg->header.raw_value, msg->len);

	/* 3 retries per USB PD specification */
	k_mutex_lock(&data->bus_lock, K_FOREVER);
	ret = tcpci_tcpm_transmit_data(&cfg->bus, msg, 3);
	k_mutex_unlock(&data->bus_lock);
	return ret;
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
	struct raa489400_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->bus_lock, K_FOREVER);
	if (reg == TCPC_VENDOR_DEFINED_STATUS) {
		ret = tcpci_read_reg16(&cfg->bus,
				       RAA489400_REG_VENDOR_STATUS,
				       (uint16_t *)status);
	} else {
		ret = tcpci_tcpm_get_status_register(&cfg->bus, reg,
						     (uint16_t *)status);
	}
	k_mutex_unlock(&data->bus_lock);
	return ret;
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
	uint16_t serviced;
	int ret;

	/* Do not touch the bus until init has fully completed, or this
	 * handler's I2C will collide with init's own I2C traffic. */
	if (!data->initialized) {
		return;
	}

	/* Serialize against the USB-C stack thread's TCPC calls. k_mutex is
	 * recursive, so the synchronous data->alert_handler() callbacks
	 * below (which re-enter this driver's locked API functions on the
	 * same thread) are safe. */
	k_mutex_lock(&data->bus_lock, K_FOREVER);

	/* Read the ALERT register */
	ret = tcpci_read_reg16(&cfg->bus, TCPC_REG_ALERT, &alert_reg);
	if (ret || alert_reg == 0) {
		k_mutex_unlock(&data->bus_lock);
		return;
	}

	/* Remember what we saw this pass, so the re-drain below can tell a
	 * genuinely new event apart from a persistent bit we already
	 * serviced (a persistent bit must NOT trigger an immediate
	 * resubmit, or the handler spins and starves the bus). */
	serviced = alert_reg;

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
		case TCPC_ALERT_POWER_STATUS: {
			// uint8_t pstat = 0;
			// tcpci_read_reg8(&cfg->bus,
			// 		TCPC_REG_POWER_STATUS, &pstat);
			// LOG_DBG("POWER_STATUS = 0x%02x", pstat);
			bit = TCPC_REG_ALERT_POWER_STATUS;
			break;
		}
		// case TCPC_ALERT_MSG_STATUS:
		// 	bit = TCPC_REG_ALERT_RX_STATUS;
		// 	data->msg_pending = true;
		// 	break;
		case TCPC_ALERT_MSG_STATUS:
			/*
			* Do not clear RX_STATUS here. Clearing it may release the
			* TCPC RX buffer before get_rx_pending_msg() reads it.
			*/
			data->msg_pending = true;

			if (data->alert_handler) {
				data->alert_handler(dev, data->alert_handler_data,
							TCPC_ALERT_MSG_STATUS);
			}

			alert_reg &= ~TCPC_REG_ALERT_RX_STATUS;
    		continue;		
		case TCPC_ALERT_HARD_RESET_RECEIVED:
			bit = TCPC_REG_ALERT_RX_HARD_RST;
			break;
		case TCPC_ALERT_TRANSMIT_MSG_SUCCESS:
		case TCPC_ALERT_TRANSMIT_MSG_FAILED:
		case TCPC_ALERT_TRANSMIT_MSG_DISCARDED:
			bit = TCPC_REG_ALERT_TX_COMPLETE;
			break;
		case TCPC_ALERT_FAULT_STATUS: {
			/* The FAULT alert bit only re-arms when the underlying
			 * FAULT_STATUS (0x1F) condition is acknowledged. Read
			 * it, log it once, and W1C-clear it at the source —
			 * otherwise the fault re-asserts ALERT immediately and
			 * the handler spins forever, starving the I2C bus. */
			uint8_t fault = 0;

			tcpci_read_reg8(&cfg->bus,
					RAA489400_REG_FAULT_STATUS, &fault);
			LOG_WRN("FAULT_STATUS = 0x%02x", fault);
			if (fault) {
				tcpci_write_reg8(&cfg->bus,
						 RAA489400_REG_FAULT_STATUS,
						 fault);
			}
			bit = TCPC_REG_ALERT_FAULT;
			break;
		}
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

	/* Level-triggered drain: ALERT# stays asserted (low) while any
	 * unmasked event is still pending. With an edge interrupt, an
	 * alert that arrived while we were servicing this pass produces no
	 * new falling edge, so re-check and re-submit — but ONLY if a bit
	 * we did not already service this pass is now set. A persistent
	 * bit that we just serviced (e.g. an un-clearable fault condition)
	 * must not trigger another resubmit, or the handler spins forever
	 * and starves the shared I2C bus. */
	if (tcpci_read_reg16(&cfg->bus, TCPC_REG_ALERT, &alert_reg) == 0) {
		if (alert_reg & ~serviced) {
			k_mutex_unlock(&data->bus_lock);
			k_work_submit(&data->alert_work);
			return;
		}
	}

	k_mutex_unlock(&data->bus_lock);
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
	// .init                   = raa489400_init,
	.init                   = raa489400_tcpc_init,
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
