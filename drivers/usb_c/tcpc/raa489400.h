/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * RAA489400 USB Type-C Port Controller (TCPC) Driver
 * Based on RAA489400 Datasheet R16DS0292EU0100 Rev.1.00
 */

#ifndef ZEPHYR_DRIVERS_USB_C_TCPC_RAA489400_H_
#define ZEPHYR_DRIVERS_USB_C_TCPC_RAA489400_H_

/* ----------------------------------------------------------------
 * TCPC Standard Registers (TCPCi Revision 2.0)
 * ---------------------------------------------------------------- */
#define RAA489400_REG_VENDOR_ID                 0x00
#define RAA489400_REG_PRODUCT_ID                0x02
#define RAA489400_REG_DEVICE_ID                 0x04
#define RAA489400_REG_USBTYPEC_REV              0x06
#define RAA489400_REG_USBPD_REV_VER            0x08
#define RAA489400_REG_PD_INTERFACE_REV          0x0A
#define RAA489400_REG_ALERT                     0x10
#define RAA489400_REG_ALERT_MASK                0x12
#define RAA489400_REG_POWER_STATUS_MASK         0x14
#define RAA489400_REG_FAULT_STATUS_MASK         0x15
#define RAA489400_REG_EXTENDED_STATUS_MASK      0x16
#define RAA489400_REG_ALERT_EXTENDED_MASK       0x17
#define RAA489400_REG_TCPC_CONTROL              0x19
#define RAA489400_REG_ROLE_CONTROL              0x1A
#define RAA489400_REG_FAULT_CONTROL             0x1B
#define RAA489400_REG_POWER_CONTROL             0x1C
#define RAA489400_REG_CC_STATUS                 0x1D
#define RAA489400_REG_POWER_STATUS              0x1E
#define RAA489400_REG_FAULT_STATUS              0x1F
#define RAA489400_REG_EXTENDED_STATUS           0x20
#define RAA489400_REG_ALERT_EXTENDED            0x21
#define RAA489400_REG_COMMAND                   0x23
#define RAA489400_REG_DEV_CAP_1                 0x24
#define RAA489400_REG_DEV_CAP_2                 0x26
#define RAA489400_REG_STD_INPUT_CAP             0x28
#define RAA489400_REG_STD_OUTPUT_CAP            0x29
#define RAA489400_REG_MSG_HEADER_INFO           0x2E
#define RAA489400_REG_RECEIVE_DETECT            0x2F
#define RAA489400_REG_RX_BUFFER                 0x30  /* READABLE_BYTE_COUNT */
#define RAA489400_REG_TRANSMIT                  0x50
#define RAA489400_REG_TX_BUFFER                 0x51  /* I2C_WRITE_BYTE_COUNT */
#define RAA489400_REG_VBUS_VOLTAGE              0x70
#define RAA489400_REG_VBUS_SNK_DISC_THRESH      0x72
#define RAA489400_REG_VBUS_STOP_DISC_THRESH     0x74
#define RAA489400_REG_VBUS_VOLT_ALARM_HI        0x76
#define RAA489400_REG_VBUS_VOLT_ALARM_LO        0x78
#define RAA489400_REG_DEV_CAP_3                 0x7C

/* ----------------------------------------------------------------
 * RAA489400 Vendor-Defined Registers
 * ---------------------------------------------------------------- */
#define RAA489400_REG_CHIP_REVISION             0x80
#define RAA489400_REG_GPIO1_CTRL                0x82
#define RAA489400_REG_GPIO2_CTRL                0x83
#define RAA489400_REG_GPIO3_CTRL                0x84
#define RAA489400_REG_GPIO4_CTRL                0x85
#define RAA489400_REG_VBUS_PATH_CTRL            0x86
#define RAA489400_REG_VBUS_GPIO_CTRL            0x87
#define RAA489400_REG_VBUS_CP_CTRL              0x88
#define RAA489400_REG_GPIO_OC_EN                0x89
#define RAA489400_REG_VBUS_CTRL                 0x90
#define RAA489400_REG_VBUS_CURRENT              0x92
#define RAA489400_REG_VBUS_PEAK_CURRENT         0x94
#define RAA489400_REG_VBUS_AVE_CURRENT          0x96
#define RAA489400_REG_FRS_CTRL                  0x98
#define RAA489400_REG_SINK_PATH_DISCHG          0x9A
#define RAA489400_REG_SNK_DETACH                0x9B
#define RAA489400_REG_VENDOR_STATUS             0xA0
#define RAA489400_REG_VENDOR_STATUS_MASK        0xA2
#define RAA489400_REG_VBUS_FAULT_CTRL           0xA4
#define RAA489400_REG_VCONN_FAULT_CTRL          0xA6
#define RAA489400_REG_OCP_OUTPUT_CTRL           0xA8
#define RAA489400_REG_PROCHOT_EN                0xAA
#define RAA489400_REG_CONTROL1                  0xB1
#define RAA489400_REG_TYPE_C_PARAMETER          0xE2

/* ----------------------------------------------------------------
 * VENDOR_ID Register — reset value 0x045B (Renesas)
 * ---------------------------------------------------------------- */
#define RAA489400_VENDOR_ID                     0x045B
#define RAA489400_PRODUCT_ID                    0x026D

/* ----------------------------------------------------------------
 * ALERT Register (0x10) — bit masks
 * ---------------------------------------------------------------- */
#define RAA489400_ALERT_VENDOR_DEFINED          BIT(15)
#define RAA489400_ALERT_EXTENDED                BIT(14)
#define RAA489400_ALERT_EXTENDED_STATUS         BIT(13)
#define RAA489400_ALERT_VBUS_SNK_DISCONNECT     BIT(11)
#define RAA489400_ALERT_RX_BUF_OVERFLOW         BIT(10)
#define RAA489400_ALERT_FAULT                   BIT(9)
#define RAA489400_ALERT_VBUS_ALARM_LO           BIT(8)
#define RAA489400_ALERT_VBUS_ALARM_HI           BIT(7)
#define RAA489400_ALERT_TX_SUCCESS              BIT(6)
#define RAA489400_ALERT_TX_DISCARDED            BIT(5)
#define RAA489400_ALERT_TX_FAILED               BIT(4)
#define RAA489400_ALERT_RX_HARD_RESET           BIT(3)
#define RAA489400_ALERT_RX_STATUS               BIT(2)
#define RAA489400_ALERT_POWER_STATUS            BIT(1)
#define RAA489400_ALERT_CC_STATUS               BIT(0)

/* ----------------------------------------------------------------
 * TCPC_CONTROL Register (0x19)
 * ---------------------------------------------------------------- */
#define RAA489400_TCPC_CTRL_EN_LOOK4CONN_ALERT  BIT(6)
#define RAA489400_TCPC_CTRL_BIST_TEST_MODE      BIT(1)
#define RAA489400_TCPC_CTRL_PLUG_ORIENTATION    BIT(0)

/* ----------------------------------------------------------------
 * ROLE_CONTROL Register (0x1A)
 * ---------------------------------------------------------------- */
#define RAA489400_ROLE_CTRL_DRP                 BIT(6)
#define RAA489400_ROLE_CTRL_RP_MASK             (0x3 << 4)
#define RAA489400_ROLE_CTRL_RP_DEFAULT          (0x0 << 4)
#define RAA489400_ROLE_CTRL_RP_1P5A             (0x1 << 4)
#define RAA489400_ROLE_CTRL_RP_3P0A             (0x2 << 4)
#define RAA489400_ROLE_CTRL_CC2_MASK            (0x3 << 2)
#define RAA489400_ROLE_CTRL_CC2_RP              (0x1 << 2)
#define RAA489400_ROLE_CTRL_CC2_RD              (0x2 << 2)
#define RAA489400_ROLE_CTRL_CC2_OPEN            (0x3 << 2)
#define RAA489400_ROLE_CTRL_CC1_MASK            (0x3 << 0)
#define RAA489400_ROLE_CTRL_CC1_RP              (0x1 << 0)
#define RAA489400_ROLE_CTRL_CC1_RD              (0x2 << 0)
#define RAA489400_ROLE_CTRL_CC1_OPEN            (0x3 << 0)

/* ----------------------------------------------------------------
 * FAULT_CONTROL Register (0x1B)
 * ---------------------------------------------------------------- */
#define RAA489400_FAULT_CTRL_VBUS_DISC_TIMER_DIS BIT(3)
#define RAA489400_FAULT_CTRL_VBUS_OCP_DIS        BIT(2)
#define RAA489400_FAULT_CTRL_VBUS_OVP_DIS        BIT(1)
#define RAA489400_FAULT_CTRL_VCONN_OCP_DIS       BIT(0)

/* ----------------------------------------------------------------
 * POWER_CONTROL Register (0x1C)
 * ---------------------------------------------------------------- */
#define RAA489400_PWR_CTRL_FRS_EN               BIT(7)
#define RAA489400_PWR_CTRL_VBUS_MON_DIS         BIT(6)  /* 1 = disabled (default) */
#define RAA489400_PWR_CTRL_VOLT_ALARM_DIS       BIT(5)  /* 1 = disabled (default) */
#define RAA489400_PWR_CTRL_AUTO_DISC_DISC       BIT(4)
#define RAA489400_PWR_CTRL_BLEED_DISCHARGE      BIT(3)
#define RAA489400_PWR_CTRL_FORCE_DISCHARGE      BIT(2)
#define RAA489400_PWR_CTRL_VCONN_PWR_SUPP       BIT(1)
#define RAA489400_PWR_CTRL_EN_VCONN             BIT(0)

/* ----------------------------------------------------------------
 * CC_STATUS Register (0x1D)
 * ---------------------------------------------------------------- */
#define RAA489400_CC_STATUS_LOOKING4CONN        BIT(5)
#define RAA489400_CC_STATUS_CONNECT_RESULT      BIT(4)
#define RAA489400_CC_STATUS_CC2_MASK            (0x3 << 2)
#define RAA489400_CC_STATUS_CC2_SHIFT           2
#define RAA489400_CC_STATUS_CC1_MASK            (0x3 << 0)
#define RAA489400_CC_STATUS_CC1_SHIFT           0

/* CC voltage state values (source role) */
#define RAA489400_CC_SRC_OPEN                   0x0  /* SRC.Open */
#define RAA489400_CC_SRC_RA                     0x1  /* SRC.Ra   */
#define RAA489400_CC_SRC_RD                     0x2  /* SRC.Rd   */

/* CC voltage state values (sink role) */
#define RAA489400_CC_SNK_OPEN                   0x0  /* SNK.Open    */
#define RAA489400_CC_SNK_DEFAULT                0x1  /* SNK.Default */
#define RAA489400_CC_SNK_POWER_1P5A             0x2  /* SNK.Power1.5 — Rp 1.5A */
#define RAA489400_CC_SNK_POWER_3P0A             0x3  /* SNK.Power3.0 — Rp 3.0A */

/* ----------------------------------------------------------------
 * POWER_STATUS Register (0x1E)
 * ---------------------------------------------------------------- */
#define RAA489400_PWR_STATUS_DEBUG_ACC          BIT(7)
#define RAA489400_PWR_STATUS_INIT               BIT(6)  /* 1 = still initializing */
#define RAA489400_PWR_STATUS_SRC_NONDEFAULT     BIT(5)
#define RAA489400_PWR_STATUS_SOURCING_VBUS      BIT(4)
#define RAA489400_PWR_STATUS_VBUS_DET_EN        BIT(3)
#define RAA489400_PWR_STATUS_VBUS_PRESENT       BIT(2)
#define RAA489400_PWR_STATUS_VCONN_PRESENT      BIT(1)
#define RAA489400_PWR_STATUS_SINKING_VBUS       BIT(0)

/* ----------------------------------------------------------------
 * FAULT_STATUS Register (0x1F)
 * ---------------------------------------------------------------- */
#define RAA489400_FAULT_STATUS_ALL_REGS_RESET   BIT(7)
#define RAA489400_FAULT_STATUS_AUTO_DISC_FAIL   BIT(5)
#define RAA489400_FAULT_STATUS_FORCE_DISC_FAIL  BIT(4)
#define RAA489400_FAULT_STATUS_VBUS_OCP         BIT(3)
#define RAA489400_FAULT_STATUS_VBUS_OVP         BIT(2)
#define RAA489400_FAULT_STATUS_VCONN_OCP        BIT(1)
#define RAA489400_FAULT_STATUS_I2C_ERROR        BIT(0)

/* ----------------------------------------------------------------
 * EXTENDED_STATUS Register (0x20)
 * ---------------------------------------------------------------- */
#define RAA489400_EXT_STATUS_VSAFE0V            BIT(0)

/* ----------------------------------------------------------------
 * COMMAND Register (0x23) — command bytes
 * ---------------------------------------------------------------- */
#define RAA489400_CMD_WAKE_I2C                  0x11
#define RAA489400_CMD_DISABLE_VBUS_DETECT       0x22
#define RAA489400_CMD_ENABLE_VBUS_DETECT        0x33
#define RAA489400_CMD_DISABLE_SINK_VBUS         0x44
#define RAA489400_CMD_SINK_VBUS                 0x55
#define RAA489400_CMD_DISABLE_SOURCE_VBUS       0x66
#define RAA489400_CMD_SOURCE_VBUS_DEFAULT       0x77
#define RAA489400_CMD_LOOK4CONNECTION           0x99
#define RAA489400_CMD_RX_ONE_MORE               0xAA
#define RAA489400_CMD_RESET_TRANSMIT_BUF        0xDD
#define RAA489400_CMD_RESET_RECEIVE_BUF         0xEE

/* ----------------------------------------------------------------
 * MESSAGE_HEADER_INFO Register (0x2E)
 * ---------------------------------------------------------------- */
#define RAA489400_MSG_HDR_CABLE_PLUG            BIT(4)
#define RAA489400_MSG_HDR_DATA_ROLE_DFP         BIT(3)
#define RAA489400_MSG_HDR_PD_REV_MASK          (0x3 << 1)
#define RAA489400_MSG_HDR_PD_REV_1P0           (0x0 << 1)
#define RAA489400_MSG_HDR_PD_REV_2P0           (0x1 << 1)
#define RAA489400_MSG_HDR_PD_REV_3P0           (0x2 << 1)
#define RAA489400_MSG_HDR_PWR_ROLE_SRC         BIT(0)

/* ----------------------------------------------------------------
 * TRANSMIT Register (0x50)
 * ---------------------------------------------------------------- */
#define RAA489400_TX_RETRY_MASK                 (0x3 << 4)
#define RAA489400_TX_RETRY_0                    (0x0 << 4)
#define RAA489400_TX_RETRY_1                    (0x1 << 4)
#define RAA489400_TX_RETRY_2                    (0x2 << 4)
#define RAA489400_TX_RETRY_3                    (0x3 << 4)
#define RAA489400_TX_SOP_TYPE_MASK              0x7
#define RAA489400_TX_SOP                        0x0
#define RAA489400_TX_SOP_PRIME                  0x1
#define RAA489400_TX_SOP_DPRIME                 0x2
#define RAA489400_TX_SOP_DBG_PRIME              0x3
#define RAA489400_TX_SOP_DBG_DPRIME             0x4
#define RAA489400_TX_HARD_RESET                 0x5
#define RAA489400_TX_CABLE_RESET                0x6
#define RAA489400_TX_BIST_CM2                   0x7

/* ----------------------------------------------------------------
 * VBUS_VOLTAGE Register (0x70)
 * ---------------------------------------------------------------- */
#define RAA489400_VBUS_VOLT_SCALE_MASK          (0x3 << 10)
#define RAA489400_VBUS_VOLT_SCALE_1X            (0x0 << 10)
#define RAA489400_VBUS_VOLT_MEAS_MASK           0x3FF
#define RAA489400_VBUS_VOLT_LSB_MV              25   /* 25 mV per LSB */

/* ----------------------------------------------------------------
 * VBUS_PATH_CTRL Register (0x86)
 * ---------------------------------------------------------------- */
#define RAA489400_VBUS_PATH_SRC_SEL_MASK        (0x7 << 5)
#define RAA489400_VBUS_PATH_SRC_SEL_VSRC_GATE   (0x0 << 5)
#define RAA489400_VBUS_PATH_SRC_SEL_GPIO1        (0x1 << 5)
#define RAA489400_VBUS_PATH_SRC_EN              BIT(4)
#define RAA489400_VBUS_PATH_SNK_SEL_MASK        (0x7 << 1)
#define RAA489400_VBUS_PATH_SNK_SEL_VSNK_GATE   (0x0 << 1)
#define RAA489400_VBUS_PATH_SNK_SEL_GPIO1        (0x1 << 1)
#define RAA489400_VBUS_PATH_SNK_EN              BIT(0)

/* ----------------------------------------------------------------
 * VBUS_CP_CTRL Register (0x88)
 * ---------------------------------------------------------------- */
#define RAA489400_VBUS_CP_VSRC_CP_EN            BIT(1)

/* ----------------------------------------------------------------
 * VENDOR_STATUS Register (0xA0) — key bits
 * ---------------------------------------------------------------- */
#define RAA489400_VS_SNK_PATH_DISC_TIMEOUT      BIT(15)
#define RAA489400_VS_SNK_PATH_DISC_DONE         BIT(14)
#define RAA489400_VS_VCONN_OTP                  BIT(13)
#define RAA489400_VS_VCONN_RVP                  BIT(12)
#define RAA489400_VS_VCONN_UVP                  BIT(11)
#define RAA489400_VS_VCONN_OVP                  BIT(10)
#define RAA489400_VS_FAULT_STATE_CHANGED        BIT(6)
#define RAA489400_VS_FAULT_STATE                BIT(5)
#define RAA489400_VS_SRC_VBUS_RVP               BIT(4)
#define RAA489400_VS_SRC_VBUS_UVP               BIT(3)
#define RAA489400_VS_SNK_DETACH_PROCHOT         BIT(1)
#define RAA489400_VS_SNK_DETACH                 BIT(0)

/* ----------------------------------------------------------------
 * VBUS_FAULT_CTRL Register (0xA4)
 * ---------------------------------------------------------------- */
#define RAA489400_VBUS_FAULT_OVP_TYPE_EPR       BIT(7)
#define RAA489400_VBUS_FAULT_SRC_RVP_DIS        BIT(4)
#define RAA489400_VBUS_FAULT_SRC_UVP_DIS        BIT(3)
#define RAA489400_VBUS_FAULT_SRC_OVP_DIS        BIT(2)
#define RAA489400_VBUS_FAULT_OC_MASK            0x3
#define RAA489400_VBUS_FAULT_OC_1P8A            0x0
#define RAA489400_VBUS_FAULT_OC_3P6A            0x1  /* default */
#define RAA489400_VBUS_FAULT_OC_4P8A            0x2
#define RAA489400_VBUS_FAULT_OC_6P0A            0x3

/* ----------------------------------------------------------------
 * TYPE_C_PARAMETER Register (0xE2)
 * ---------------------------------------------------------------- */
#define RAA489400_TYPE_C_FORCE_24MHZ            BIT(14)
#define RAA489400_TYPE_C_STOP_24MHZ_OSC         BIT(13)
/* bits [8:2] must be written as reset value 0b110_0100 */
#define RAA489400_TYPE_C_RESERVED_BITS          (0x64 << 2)

/* ----------------------------------------------------------------
 * CONTROL1 Register (0xB1) — bits [2:0] must be set to 001b
 * ---------------------------------------------------------------- */
#define RAA489400_CONTROL1_OSC_CAL_MASK         0x7
#define RAA489400_CONTROL1_OSC_CAL_VALUE        0x1

/* ----------------------------------------------------------------
 * I2C address — set by PROG resistor (Table 1 in datasheet)
 * ---------------------------------------------------------------- */
#define RAA489400_I2C_ADDR_0x22                 0x22  /* PROG = 1.0kΩ  */
#define RAA489400_I2C_ADDR_0x23                 0x23  /* PROG = 1.5kΩ  */
#define RAA489400_I2C_ADDR_0x24                 0x24  /* PROG = 2.2kΩ  */
#define RAA489400_I2C_ADDR_0x25                 0x25  /* PROG = 3.3kΩ  */
#define RAA489400_I2C_ADDR_0x26                 0x26  /* PROG = 4.7kΩ  */
#define RAA489400_I2C_ADDR_0x27                 0x27  /* PROG = 6.8kΩ  */

#endif /* ZEPHYR_DRIVERS_USB_C_TCPC_RAA489400_H_ */
