/*
 * Driver for decaWave DW1000 802.15.4 Wireless-PAN Networking controller
 *
 * Copyright (C) 2018 Xue Liu <liuxuenetmail@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _DW1000_H
#define _DW1000_H

#define RG_DEV_ID	0x00
#define RG_EUI		0x01
#define RG_PANADR	0x03
#define RG_SYS_CFG	0x04
#define RG_SYS_TIME	0x06
#define RG_TX_FCTRL	0x08
#define RG_TX_BUFFER	0x09
#define RG_DX_TIME	0x0A
#define RG_RX_FWTO	0x0C
#define RG_SYS_CTRL	0x0D
#define RG_SYS_MASK	0x0E
#define RG_SYS_STATUS	0x0F
#define RG_RX_FINFO	0x10
#define RG_RX_BUFFER	0x11
#define RG_RX_FQUAL	0x12
#define RG_RX_TTCKI	0x13
#define RG_RX_TTCKO	0x14
#define RG_RX_TIME	0x15
#define RG_TX_TIME	0x17
#define RG_TX_ANTD	0x18
#define RG_SYS_STATE	0x19
#define RG_ACK_RESP_T	0x1A
#define RG_RX_SNIFF	0x1D
#define RG_TX_POWER	0x1E
#define RG_CHAN_CTRL	0x1F
#define RG_USR_SFD	0x21
#define RG_ACG_CTRL	0x23
#define RG_EXT_SYNC	0x24
#define RG_ACC_MEM	0x25
#define RG_GPIO_CTRL	0x26
#define RG_DRX_CONF	0x27
#define	RG_RF_CONF	0x28
#define RG_TX_CAL	0x2A
#define RG_FS_CTRL	0x2B
#define RG_AON		0x2C
#define RG_OTP_IF	0x2D
#define RG_LDE_CTRL	0x2E
#define RG_DIG_DIAG	0x2F
#define RG_PMSC		0x36

#endif /* _DW1000_H */

