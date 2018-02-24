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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/ieee802154.h>
#include <linux/debugfs.h>

#include <net/mac802154.h>
#include <net/cfg802154.h>

#include <linux/device.h>

#include "dw1000.h"

#define printdev(X) (&X->spi->dev)

#define	SPI_COMMAND_BUFFER	5

#define REG_READ	(0)
#define REG_WRITE	BIT(7)
#define SUB_INDEX	BIT(6)
#define ADDR_EXT	BIT(7)

#define DW1000_REG_READ(x)	(x)
#define DW1000_REG_WRITE(x)	(REG_WRITE | (x))

struct dw1000_local {
	struct spi_device *spi;

	struct ieee802154_hw *hw;
	struct regmap *regmap;
	int slp_tr;
	int exton;
	int wakeup;
	bool sleep;

	u8 *buf;

	unsigned long cal_timeout;
	bool is_tx;
	bool is_tx_from_off;
	u8 tx_retry;
	struct sk_buff *tx_skb;

	/* SPI transaction with 3-octet header */
	struct spi_message reg_msg;
	u8 reg_addr[3];
	struct spi_transfer reg_addr_xfer;
	u8 reg_val[16];
	struct spi_transfer reg_val_xfer;
};

/* DW1000 operational states */
enum {
	STATE_OFF	= 0x00,
	STATE_WAKEUP	= 0x01,
	STATE_INIT	= 0x02,
	STATE_IDLE	= 0x03,
	STATE_SLEEP	= 0x04,
	STATE_DEEPSLEEP	= 0x05,
	STATE_TX	= 0x06,
	STATE_RX	= 0x07,
	STATE_SNOOZE	= 0x08,
};

// static bool
// dw1000_reg_writeable(struct device *dev, unsigned int reg)
// {
// 	switch (reg) {
// 	case RG_EUI:
// 		return true;
// 	default:
// 		return false;
// 	}
// }

// static bool
// dw1000_reg_readable(struct device *dev, unsigned int reg)
// {
// 	bool rc;

// 	/* all writeable are also readable */
// 	rc = dw1000_reg_writeable(dev, reg);
// 	if (rc)
// 		return rc;

// 	/* readonly regs */
// 	switch (reg) {
// 	case RG_DEV_ID:
// 		return true;
// 	default:
// 		return false;
// 	}
// }

// static const struct regmap_config dw1000_regmap_spi_config = {
// 	.reg_bits = 8,
// 	.val_bits = 8,
// 	.write_flag_mask = CMD_REG | CMD_WRITE,
// 	.read_flag_mask = CMD_REG,
// 	.cache_type = REGCACHE_RBTREE,
// 	.max_register = AT86RF2XX_NUMREGS,
// 	.writeable_reg = dw1000_reg_writeable,
// 	.readable_reg = dw1000_reg_readable,
// 	.volatile_reg = dw1000_reg_volatile,
// 	.precious_reg = dw1000_reg_precious,
// };


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_reg()
 *
 * @brief  this function is used to read from the DW1000 device registers
 * Notes:
 *        1. Firstly we create a header (the first byte is a header byte)
 *        a. check if sub index is used, if subindexing is used - set bit-6 to 1 to signify that the sub-index address follows the register index byte
 *        b. set bit-7 (or with 0x80) for write operation
 *        c. if extended sub address index is used (i.e. if index > 127) set bit-7 of the first sub-index byte following the first header byte
 *
 *        2. Write the header followed by the data bytes to the DW1000 device
 *        3. Store the read data in the input buffer
 *
 * input parameters:
 * @param lp 		- struct dw1000_local
 * @param addr  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being read
 * @param data		- pointer to buffer in which to return the read data.
 *
 * output parameters
 *
 * returns SPI transmission status
 */
static int
dw1000_read_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 length, void *data)
{
	u8 header[3] = { 0, 0, 0}; // Buffer to compose header in
	int   cnt = 0; // Counter for length of header
	int i;
	struct spi_message msg;
	struct spi_transfer header_xfer = {
		.tx_buf = header,
	};

	struct spi_transfer data_xfer = {
		.len = length,
		.rx_buf = data,
	};

	// Write message header selecting READ operation and addresses as appropriate (this is one to three bytes long)
	if (index == 0) { // For index of 0, no sub-index is required
		header[cnt++] = (u8)addr; // Bit-7 zero is READ operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
	} else {
		// Bit-7 zero is READ operation, bit-6 one=sub-address follows, bits 5-0 is reg file id
		header[cnt++] = (u8)(0x40 | addr);

		if (index <= 127) { // For non-zero index < 127, just a single sub-index byte is required
			header[cnt++] = (u8)index; // Bit-7 zero means no extension, bits 6-0 is index.
		} else {
			header[cnt++] = 0x80 | (u8)(index); // Bit-7 one means extended index, bits 6-0 is low seven bits of index.
			header[cnt++] =  (u8)(index >> 7); // 8-bit value = high eight bits of index.
		}
	}

	header_xfer.len = cnt;

	for (i = 0; i < cnt; i++) {
		dev_dbg(printdev(lp), "addr[%d]:0x%x\n", i, header[i]);
	}

	spi_message_init(&msg);
	spi_message_add_tail(&header_xfer, &msg);
	spi_message_add_tail(&data_xfer, &msg);

	return spi_sync(lp->spi, &msg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_8bit_reg()
 *
 * @brief  this function is used to read an 8-bit value from the DW1000 device registers
 *
 * input parameters:
 * @param lp - dw1000_local
 * @param addr - ID of register file or buffer being accessed
 * @param index - the index into register file or buffer being accessed
 * @param data - u8 data read from register
 *
 * output parameters
 *
 * returns SPI transmission status
 */
static int
dw1000_read_8bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u8 *data) {

	// Read 1 bytes (8-bits) register into buffer
	return dw1000_read_reg(lp, addr, index, sizeof(data), data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_16bit_reg()
 *
 * @brief  this function is used to read an 16-bit value from the DW1000 device registers
 *
 * input parameters:
 * @param lp - dw1000_local
 * @param addr - ID of register file or buffer being accessed
 * @param index - the index into register file or buffer being accessed
 * @param data - u16 data read from register
 *
 * output parameters
 *
 * returns SPI transmission status
 */
static int
dw1000_read_16bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u16 *data) {
	int ret;

	// Read 2 bytes (16-bits) register into buffer
	ret = dw1000_read_reg(lp, addr, index, sizeof(data), data);

	if (ret) {
		return ret;
	}

	le16_to_cpus(data);

	return ret;
}

static int
dw1000_read_32bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 *data) {
	int ret;

	// Read 4 bytes (32-bits) register into buffer
	ret = dw1000_read_reg(lp, addr, index, sizeof(data), data);

	if (ret) {
		return ret;
	}

	le32_to_cpus(data);

	return ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_write_reg()
 *
 * @brief  this function is used to write to the DW1000 device registers
 * Notes:
 *        1. Firstly we create a header (the first byte is a header byte)
 *        a. check if sub index is used, if subindexing is used - set bit-6 to 1 to signify that the sub-index address follows the register index byte
 *        b. set bit-7 (or with 0x80) for write operation
 *        c. if extended sub address index is used (i.e. if index > 127) set bit-7 of the first sub-index byte following the first header byte
 *
 *        2. Write the header followed by the data bytes to the DW1000 device
 *
 *
 * input parameters:
 * @param lp		- struct dw1000_local
 * @param addr		- ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param data		- pointer to buffer containing the 'length' bytes to be written
 *
 * output parameters
 *
 * returns
 */
static int
dw1000_write_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 length, void *data)
{
	u8 header[3] = { 0, 0, 0 }; // Buffer to compose header in
	int   cnt = 0; // Counter for length of header
	int i;
	struct spi_message msg;
	struct spi_transfer header_xfer = {
		.tx_buf = header,
	};

	struct spi_transfer data_xfer = {
		.len = length,
		.tx_buf = data,
	};

	// Write message header selecting WRITE operation and addresses as appropriate (this is one to three bytes long)
	if (index == 0) { // For index of 0, no sub-index is required
		// Bit-7 zero is READ operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
		header[cnt++] = REG_WRITE | (u8)addr;
	} else {
		// Bit-7 is WRITE operation, bit-6 one=sub-address follows, bits 5-0 is reg file id
		header[cnt++] = (u8)(REG_WRITE | SUB_INDEX | addr);

		if (index <= 127) { // For non-zero index < 127, just a single sub-index byte is required
			header[cnt++] = (u8)index; // Bit-7 zero means no extension, bits 6-0 is index.
		} else {
			header[cnt++] = REG_WRITE | (u8)(index); // Bit-7 one means extended index, bits 6-0 is low seven bits of index.
			header[cnt++] =  (u8)(index >> 7); // 8-bit value = high eight bits of index.
		}
	}

	header_xfer.len = cnt;

	for (i = 0; i < cnt; i++) {
		dev_dbg(printdev(lp), "write addr[%d]:0x%x\n", i, header[i]);
	}

	spi_message_init(&msg);
	spi_message_add_tail(&header_xfer, &msg);
	spi_message_add_tail(&data_xfer, &msg);

	return spi_sync(lp->spi, &msg);
}

static int
dw1000_write_8bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u8 *data)
{
	return dw1000_write_reg(lp, addr, index, sizeof(data), data);
}

static int
dw1000_write_16bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u16 *data)
{
	cpu_to_le16s(&data);
	return dw1000_write_reg(lp, addr, index, sizeof(data), data);
}


static int
dw1000_write_32bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 *data)
{
	cpu_to_le32s(&data);
	return dw1000_write_reg(lp, addr, index, sizeof(data), data);
}

static int
dw1000_regmap_write(void *context, const void *data,
			       size_t count)
{
	struct spi_device *spi = context;
	u8 buf[3];

	if (count > 3)
		return -EINVAL;

	memcpy(buf, data, count);
	buf[1] |= (1 << 4);

	return spi_write(spi, buf, count);
}

static int
dw1000_regmap_read(void *context, const void *reg, size_t reg_size,
		   void *val, size_t val_size)
{
	struct spi_device *spi = context;

	return spi_write_then_read(spi, reg, reg_size, val, val_size);
}


static const struct regmap_bus dw1000_regmap_bus = {
	.write = dw1000_regmap_write,
	.read = dw1000_regmap_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static inline void
dw1000_sleep(struct dw1000_local *lp)
{

}

static inline void
dw1000_awake(struct dw1000_local *lp)
{

}

/*
 * SPI read with a 3-octet header
*/
//static inline int
//dw1000_read_subreg(struct dw1000_local *lp,
//	unsigned int addr, unsigned int index, unsigned int mask,
//	unsigned int shift, unsigned int *data)
//{
//	int ret;
//
//	u8 index_low = index & 0x00ff;
//	u8 index_high = ((index >> 8) & 0xff00) * 2;
//
//	lp->reg_addr[0] = REG_READ(addr) | SUB_INDEX;
//	dev_dbg(printdev(lp), "reg_addr[0]:0x%x\n", lp->reg_addr[0]);
//
//	lp->reg_addr[1] = ADDR_EXT | index_low;
//	dev_dbg(printdev(lp), "reg_addr[1]:0x%x\n", lp->reg_addr[1]);
//
//	lp->reg_addr[2] = index_high;
//	dev_dbg(printdev(lp), "reg_addr[2]:0x%x\n", lp->reg_addr[2]);
//
//	lp->reg_val_xfer.len = 1;
//
//	ret = spi_sync(lp->spi, &lp->reg_msg);
//	if (ret)
//		dev_err(printdev(lp), "failed to read subreg\n");
//
//	dev_dbg(printdev(lp), "reg_val:0x%x\n", lp->reg_val[0]);
//
//	if (!ret)
//		*data = (*lp->reg_val & mask) >> shift;
//
//	return ret;
//}
//
//static inline int
//dw1000_write_subreg(struct at86rf230_local *lp,
//	unsigned int addr, unsigned int mask,
//	unsigned int shift, unsigned int data)
//{
//	bool sleep = lp->sleep;
//	int ret;
//
//	/* awake for register setting if sleep */
//	if (sleep)
//		dw1000_awake(lp);
//
//	ret = regmap_update_bits(lp->regmap, addr, mask, data << shift);
//
//	/* sleep again if was sleeping */
//	if (sleep)
//		at86rf230_sleep(lp);
//
//	return ret;
//}

static irqreturn_t dw1000_isr(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int
dw1000_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	return 0;
}

static int
dw1000_ed(struct ieee802154_hw *hw, u8 *level)
{
	return 0;
}

static int
dw1000_start(struct ieee802154_hw *hw)
{
	return 0;
}

static void
dw1000_stop(struct ieee802154_hw *hw)
{}

static int
dw1000_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct dw1000_local *lp = hw->priv;

	dev_dbg(printdev(lp), "%s\n", __func__);
	return 0;
}

#define DW1000_MAX_ED_LEVELS 0xF
static const s32 dw1000_ed_levels[DW1000_MAX_ED_LEVELS + 1] = {
	-9400, -9200, -9000, -8800, -8600, -8400, -8200, -8000, -7800, -7600,
	-7400, -7200, -7000, -6800, -6600, -6400,
};

static inline int
dw1000_update_cca_ed_level(struct dw1000_local *lp, int rssi_base_val)
{

	return 0;
}

static int
dw1000_set_hw_addr_filt(struct ieee802154_hw *hw,
			   struct ieee802154_hw_addr_filt *filt,
			   unsigned long changed)
{
	return 0;
}

#define DW1000_MAX_TX_POWERS 0x1F
static const s32 dw1000_powers[DW1000_MAX_TX_POWERS + 1] = {
	500, 400, 300, 200, 100, 0, -100, -200, -300, -400, -500, -600, -700,
	-800, -900, -1000, -1100, -1200, -1300, -1400, -1500, -1600, -1700,
	-1800, -1900, -2000, -2100, -2200, -2300, -2400, -2500, -2600,
};


static int
dw1000_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	return 0;
}

static int
dw1000_set_lbt(struct ieee802154_hw *hw, bool on)
{

	return 0;
}

static int
dw1000_set_cca_mode(struct ieee802154_hw *hw,
		       const struct wpan_phy_cca *cca)
{
	return 0;
}

static int
dw1000_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	return 0;
}
static int
dw1000_set_csma_params(struct ieee802154_hw *hw, u8 min_be, u8 max_be,
			  u8 retries)
{
	return 0;
}

static int
dw1000_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	return 0;
}

static int
dw1000_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	return 0;
}

static const struct ieee802154_ops dw1000_ops = {
	.owner = THIS_MODULE,
	.xmit_async = dw1000_xmit,
	.ed = dw1000_ed,
	.set_channel = dw1000_set_channel,
	.start = dw1000_start,
	.stop = dw1000_stop,
	.set_hw_addr_filt = dw1000_set_hw_addr_filt,
	.set_txpower = dw1000_set_txpower,
	.set_lbt = dw1000_set_lbt,
	.set_cca_mode = dw1000_set_cca_mode,
	.set_cca_ed_level = dw1000_set_cca_ed_level,
	.set_csma_params = dw1000_set_csma_params,
	.set_frame_retries = dw1000_set_frame_retries,
	.set_promiscuous_mode = dw1000_set_promiscuous_mode,
};

static void
dw1000_setup_reg_messages(struct dw1000_local *lp)
{
	spi_message_init(&lp->reg_msg);
	lp->reg_msg.context = lp;

	lp->reg_addr_xfer.len = 3;
	lp->reg_addr_xfer.tx_buf = lp->reg_addr;

	lp->reg_val_xfer.tx_buf = lp->reg_val;
	lp->reg_val_xfer.rx_buf = lp->reg_val;

	spi_message_add_tail(&lp->reg_addr_xfer, &lp->reg_msg);
	spi_message_add_tail(&lp->reg_val_xfer, &lp->reg_msg);
}


static int dw1000_hw_init(struct dw1000_local *lp)
{
//	int rc, irq_type, irq_pol = IRQ_ACTIVE_HIGH;
//	unsigned int dvdd;
//	u8 csma_seed[2];
//
	dev_dbg(printdev(lp), "%s\n", __func__);
//
//	rc = dw1000_sync_state_change(lp, STATE_FORCE_TRX_OFF);
//	if (rc)
//		return rc;
//
//	irq_type = irq_get_trigger_type(lp->spi->irq);
//	if (irq_type == IRQ_TYPE_EDGE_FALLING ||
//	    irq_type == IRQ_TYPE_LEVEL_LOW)
//		irq_pol = IRQ_ACTIVE_LOW;
//
//	rc = dw1000_write_subreg(lp, SR_IRQ_POLARITY, irq_pol);
//	if (rc)
//		return rc;
//
//	rc = dw1000_write_subreg(lp, SR_RX_SAFE_MODE, 1);
//	if (rc)
//		return rc;
//
//	rc = dw1000_write_subreg(lp, SR_IRQ_MASK, IRQ_TRX_END);
//	if (rc)
//		return rc;
//
//	/* reset values differs in at86rf231 and at86rf233 */
//	rc = dw1000_write_subreg(lp, SR_IRQ_MASK_MODE, 0);
//	if (rc)
//		return rc;
//
//	get_random_bytes(csma_seed, ARRAY_SIZE(csma_seed));
//	rc = dw1000_write_subreg(lp, SR_CSMA_SEED_0, csma_seed[0]);
//	if (rc)
//		return rc;
//	rc = dw1000_write_subreg(lp, SR_CSMA_SEED_1, csma_seed[1]);
//	if (rc)
//		return rc;
//
//	/* CLKM changes are applied immediately */
//	rc = dw1000_write_subreg(lp, SR_CLKM_SHA_SEL, 0x00);
//	if (rc)
//		return rc;
//
//	/* Turn CLKM Off */
//	rc = dw1000_write_subreg(lp, SR_CLKM_CTRL, 0x00);
//	if (rc)
//		return rc;
//	/* Wait the next SLEEP cycle */
//	usleep_range(lp->data->t_sleep_cycle,
//		     lp->data->t_sleep_cycle + 100);

	/* xtal_trim value is calculated by:
	 * CL = 0.5 * (CX + CTRIM + CPAR)
	 *
	 * whereas:
	 * CL = capacitor of used crystal
	 * CX = connected capacitors at xtal pins
	 * CPAR = in all at86rf2xx datasheets this is a constant value 3 pF,
	 *	  but this is different on each board setup. You need to fine
	 *	  tuning this value via CTRIM.
	 * CTRIM = variable capacitor setting. Resolution is 0.3 pF range is
	 *	   0 pF upto 4.5 pF.
	 *
	 * Examples:
	 * atben transceiver:
	 *
	 * CL = 8 pF
	 * CX = 12 pF
	 * CPAR = 3 pF (We assume the magic constant from datasheet)
	 * CTRIM = 0.9 pF
	 *
	 * (12+0.9+3)/2 = 7.95 which is nearly at 8 pF
	 *
	 * xtal_trim = 0x3
	 *
	 * openlabs transceiver:
	 *
	 * CL = 16 pF
	 * CX = 22 pF
	 * CPAR = 3 pF (We assume the magic constant from datasheet)
	 * CTRIM = 4.5 pF
	 *
	 * (22+4.5+3)/2 = 14.75 which is the nearest value to 16 pF
	 *
	 * xtal_trim = 0xf
	 */
//	rc = dw1000_write_subreg(lp, SR_XTAL_TRIM, xtal_trim);
//	if (rc)
//		return rc;
//
//	rc = dw1000_read_subreg(lp, SR_DVDD_OK, &dvdd);
//	if (rc)
//		return rc;
//	if (!dvdd) {
//		dev_err(&lp->spi->dev, "DVDD error\n");
//		return -EINVAL;
//	}

	/* Force setting slotted operation bit to 0. Sometimes the atben
	 * sets this bit and I don't know why. We set this always force
	 * to zero while probing.
	 */
	return 0;
}

static int
dw1000_get_pdata(struct spi_device *spi, int *rstn)
{
//	struct dw1000_platform_data *pdata = spi->dev.platform_data;
//	int ret;
//
	dev_info(&spi->dev, "%s\n", __func__);

//	if (!IS_ENABLED(CONFIG_OF) || !spi->dev.of_node) {
//		if (!pdata)
//			return -ENOENT;
//
//		*rstn = pdata->rstn;
//		*slp_tr = pdata->slp_tr;
//		*xtal_trim = pdata->xtal_trim;
//		return 0;
//	}
//
//	*rstn = of_get_named_gpio(spi->dev.of_node, "reset-gpio", 0);
//	*slp_tr = of_get_named_gpio(spi->dev.of_node, "sleep-gpio", 0);
//	ret = of_property_read_u8(spi->dev.of_node, "xtal-trim", xtal_trim);
//	if (ret < 0 && ret != -EINVAL)
//		return ret;

	*rstn = 27;

	return 0;
}

static int
dw1000_detect_device(struct dw1000_local *lp)
{
//	unsigned int part, version, val;
//	u16 man_id = 0;
	const char *chip;
//	int rc;
	int status;

	u8 data[4] = {0, 0, 0, 0};
	u32 dev_id = 0;

//	unsigned int reg_val = 0x0;

//	int ret;
	struct spi_message msg;

	struct spi_transfer xfer_head = {
		.len = 2,
		.tx_buf = lp->buf,
//		.rx_buf = priv->buf,
	};
	struct spi_transfer xfer_buf = {
		.len = 4,
		.rx_buf = data,
	};

	dev_dbg(printdev(lp), "%s\n", __func__);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	lp->buf[0] = 0x40;
	lp->buf[1] = 0x80;
	lp->buf[2] = 0x00;

	dev_dbg(&lp->spi->dev, "read DEVID command[0] = %02x\n", lp->buf[0]);

	status = spi_sync(lp->spi, &msg);

	if (msg.status)
		status = msg.status;
	dev_dbg(&lp->spi->dev, "return data data[0] = %02x\n", data[0]);
	dev_dbg(&lp->spi->dev, "return data data[1] = %02x\n", data[1]);
	dev_dbg(&lp->spi->dev, "return data data[2] = %02x\n", data[2]);
	dev_dbg(&lp->spi->dev, "return data data[3] = %02x\n", data[3]);

//	dev_dbg(&lp->spi->dev, "read subreg test start >>>>>>\n");
//
//	dw1000_read_subreg(lp, SG_REV, &reg_val);
//	dev_dbg(&lp->spi->dev, "SG_REV: 0x%x", reg_val);
//
//	dw1000_read_subreg(lp, SG_VER, &reg_val);
//	dev_dbg(&lp->spi->dev, "SG_VER: 0x%x", reg_val);
//
//	dev_dbg(&lp->spi->dev, "read subreg test stop <<<<<<\n");

	dw1000_read_32bit_reg(lp, RG_DEV_ID, 0, &dev_id);
	dev_dbg(&lp->spi->dev, "DEV_ID:0x%x\n", dev_id);


	lp->hw->flags = IEEE802154_HW_TX_OMIT_CKSUM |
			IEEE802154_HW_CSMA_PARAMS |
			IEEE802154_HW_FRAME_RETRIES | IEEE802154_HW_AFILT |
			IEEE802154_HW_PROMISCUOUS;

	lp->hw->phy->flags = WPAN_PHY_FLAG_TXPOWER |
			     WPAN_PHY_FLAG_CCA_ED_LEVEL |
			     WPAN_PHY_FLAG_CCA_MODE;

	lp->hw->phy->supported.cca_modes = BIT(NL802154_CCA_ENERGY) |
		BIT(NL802154_CCA_CARRIER) | BIT(NL802154_CCA_ENERGY_CARRIER);
	lp->hw->phy->supported.cca_opts = BIT(NL802154_CCA_OPT_ENERGY_CARRIER_AND) |
		BIT(NL802154_CCA_OPT_ENERGY_CARRIER_OR);

	lp->hw->phy->cca.mode = NL802154_CCA_ENERGY;
		chip = "dw1000";
//		lp->data = &at86rf212_data;
		lp->hw->flags |= IEEE802154_HW_LBT;
		lp->hw->phy->supported.channels[4] = 0x1e;
		lp->hw->phy->current_channel = 1;
		lp->hw->phy->current_page = 4;
		lp->hw->phy->symbol_duration = 25;
		lp->hw->phy->supported.lbt = NL802154_SUPPORTED_BOOL_BOTH;
		lp->hw->phy->supported.tx_powers = dw1000_powers;
		lp->hw->phy->supported.tx_powers_size = ARRAY_SIZE(dw1000_powers);
		lp->hw->phy->supported.cca_ed_levels = dw1000_ed_levels;
		lp->hw->phy->supported.cca_ed_levels_size = ARRAY_SIZE(dw1000_ed_levels);

	lp->hw->phy->cca_ed_level = lp->hw->phy->supported.cca_ed_levels[7];
	lp->hw->phy->transmit_power = lp->hw->phy->supported.tx_powers[0];

//not_supp:
//	dev_info(&lp->spi->dev, "Detected %s chip version %d\n", chip, version);

//	return rc;
	return 0;
}

static int dw1000_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct dw1000_local *lp;
//	unsigned int status;
	int rc, irq_type, rstn;

	dev_info(&spi->dev, "%s\n", __func__);

	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ specified\n");
		return -EINVAL;
	}

	rc = dw1000_get_pdata(spi, &rstn);
	if (rc < 0) {
		dev_err(&spi->dev, "failed to parse platform_data: %d\n", rc);
		return rc;
	}

	if (gpio_is_valid(rstn)) {
		rc = devm_gpio_request_one(&spi->dev, rstn,
					   GPIOF_OUT_INIT_HIGH, "rstn");
		if (rc)
			return rc;
	}

//	if (gpio_is_valid(exton)) {
//		rc = devm_gpio_request_one(&spi->dev, exton,
//					   GPIOF_OUT_INIT_LOW, "exton");
//		if (rc)
//			return rc;
//	}
//
//	if (gpio_is_valid(wakeup)) {
//		rc = devm_gpio_request_one(&spi->dev, wakeup,
//					   GPIOF_OUT_INIT_LOW, "wakeup");
//		if (rc)
//			return rc;
//	}

	/* Reset */
	if (gpio_is_valid(rstn)) {
		udelay(1);
		gpio_set_value_cansleep(rstn, 0);
		udelay(1);
		gpio_set_value_cansleep(rstn, 1);
		usleep_range(120, 240);
	}

	hw = ieee802154_alloc_hw(sizeof(*lp), &dw1000_ops);
	if (!hw)
		return -ENOMEM;

	lp = hw->priv;
	lp->hw = hw;
	lp->spi = spi;
//	lp->slp_tr = slp_tr;
	hw->parent = &spi->dev;
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

//	lp->regmap = devm_regmap_init_spi(spi, &dw1000_regmap_spi_config);
//	if (IS_ERR(lp->regmap)) {
//		rc = PTR_ERR(lp->regmap);
//		dev_err(&spi->dev, "Failed to allocate register map: %d\n",
//			rc);
//		goto free_dev;
//	}

//	dw1000_setup_spi_messages(lp, &lp->state);
//	dw1000__setup_spi_messages(lp, &lp->tx);

	lp->buf = devm_kzalloc(&spi->dev,
				 SPI_COMMAND_BUFFER, GFP_KERNEL);
	if (!lp->buf)
		return -ENOMEM;

	dw1000_setup_reg_messages(lp);

	rc = dw1000_detect_device(lp);
	if (rc < 0)
		goto free_dev;

//	init_completion(&lp->state_complete);

	spi_set_drvdata(spi, lp);

	rc = dw1000_hw_init(lp);
	if (rc)
		goto free_dev;

	/* Read irq status register to reset irq line */
//	rc = dw1000_read_subreg(lp, RG_IRQ_STATUS, 0xff, 0, &status);
//	if (rc)
//		goto free_dev;

	irq_type = irq_get_trigger_type(spi->irq);
	if (!irq_type)
		irq_type = IRQF_TRIGGER_HIGH;

	rc = devm_request_irq(&spi->dev, spi->irq, dw1000_isr,
			      IRQF_SHARED | irq_type, dev_name(&spi->dev), lp);
	if (rc)
		goto free_dev;

	/* disable_irq by default and wait for starting hardware */
	disable_irq(spi->irq);

	/* going into sleep by default */
	//dw1000_sleep(lp);

	rc = ieee802154_register_hw(lp->hw);
	if (rc)
		goto free_dev;

	return rc;

free_dev:
	ieee802154_free_hw(lp->hw);

	return rc;
}

static int
dw1000_remove(struct spi_device *spi)
{
	struct dw1000_local *lp = spi_get_drvdata(spi);

	ieee802154_unregister_hw(lp->hw);
	ieee802154_free_hw(lp->hw);
	dev_info(&spi->dev, "unregistered dw1000\n");

	return 0;
}

static const struct of_device_id dw1000_of_match[] = {
	{ .compatible = "decaWave,dw1000", },
	{ },
};
MODULE_DEVICE_TABLE(of, dw1000_of_match);

static const struct spi_device_id dw1000_device_id[] = {
	{ .name = "dw1000", },
	{ },
};
MODULE_DEVICE_TABLE(spi, dw1000_device_id);

static struct spi_driver dw1000_driver = {
	.id_table = dw1000_device_id,
	.driver = {
		.of_match_table = of_match_ptr(dw1000_of_match),
		.name	= "dw1000",
	},
	.probe      = dw1000_probe,
	.remove     = dw1000_remove,
};

module_spi_driver(dw1000_driver);

MODULE_DESCRIPTION("DW1000 IEEE 802.15.4 Transceiver Driver");
MODULE_LICENSE("GPL v2");
