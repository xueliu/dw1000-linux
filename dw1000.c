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
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/ieee802154.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>

#include <net/mac802154.h>
#include <net/cfg802154.h>

#include <linux/device.h>

#include "dw1000.h"
#include "dw1000_regs.h"

#define printdev(X) (&X->spi->dev)

#define	SPI_COMMAND_BUFFER	5

#define REG_READ	(0)
#define REG_WRITE	BIT(7)
#define SUB_INDEX	BIT(6)
#define ADDR_EXT	BIT(7)

#define DW1000_REG_READ(x)	(x)
#define DW1000_REG_WRITE(x)	(REG_WRITE | (x))

/* Defines for enable_clocks function */
#define FORCE_SYS_XTI  0
#define ENABLE_ALL_SEQ 1
#define FORCE_SYS_PLL  2
#define READ_ACC_ON    7
#define READ_ACC_OFF   8
#define FORCE_OTP_ON   11
#define FORCE_OTP_OFF  12
#define FORCE_TX_PLL   13
#define FORCE_LDE      14

/* Defines for ACK request bitmask in DATA and MAC COMMAND frame control (first byte) - Used to detect AAT bit wrongly set. */
#define FCTRL_ACK_REQ_MASK 0x20
/* Frame control maximum length in bytes. */
#define FCTRL_LEN_MAX 2

/* OTP addresses definitions */
#define LDOTUNE_ADDRESS (0x04)
#define PARTID_ADDRESS (0x06)
#define LOTID_ADDRESS  (0x07)
#define VBAT_ADDRESS   (0x08)
#define VTEMP_ADDRESS  (0x09)
#define XTRIM_ADDRESS  (0x1E)

#define DW1000_SUCCESS (0)
#define DW1000_ERROR   (-1)

#define DW1000_TIME_UNITS          (1.0/499.2e6/128.0) //!< = 15.65e-12 s */

#define DW1000_DEVICE_ID   (0xDECA0130)        //!< DW1000 MP device ID */

/* constants for selecting the bit rate for data TX (and RX) */
/* These are defined for write (with just a shift) the TX_FCTRL register */
#define DW1000_BR_110K     0   /* UWB bit rate 110 kbits/s */
#define DW1000_BR_850K     1   /* UWB bit rate 850 kbits/s */
#define DW1000_BR_6M8      2   /* UWB bit rate 6.8 Mbits/s */

/* constants for specifying the (Nominal) mean Pulse Repetition Frequency */
/* These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs */
#define DW1000_PRF_16M     1   /* UWB PRF 16 MHz */
#define DW1000_PRF_64M     2   /* UWB PRF 64 MHz */

/* constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols */
#define DW1000_PAC8        0   /* PAC  8 (recommended for RX of preamble length  128 and below */
#define DW1000_PAC16       1   /* PAC 16 (recommended for RX of preamble length  256 */
#define DW1000_PAC32       2   /* PAC 32 (recommended for RX of preamble length  512 */
#define DW1000_PAC64       3   /* PAC 64 (recommended for RX of preamble length 1024 and up */

/* constants for specifying TX Preamble length in symbols */
/* These are defined to allow them be directly written into byte 2 of the TX_FCTRL register */
/* (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment) */
#define DW1000_PLEN_4096   0x0C    /* Standard preamble length 4096 symbols */
#define DW1000_PLEN_2048   0x28    /* Non-standard preamble length 2048 symbols */
#define DW1000_PLEN_1536   0x18    /* Non-standard preamble length 1536 symbols */
#define DW1000_PLEN_1024   0x08    /* Standard preamble length 1024 symbols */
#define DW1000_PLEN_512    0x34    /* Non-standard preamble length 512 symbols */
#define DW1000_PLEN_256    0x24    /* Non-standard preamble length 256 symbols */
#define DW1000_PLEN_128    0x14    /* Non-standard preamble length 128 symbols */
#define DW1000_PLEN_64     0x04    /* Standard preamble length 64 symbols */

#define DW1000_SFDTOC_DEF	0x1041  /* default SFD timeout value */

#define DW1000_PHRMODE_STD	0x0     /* standard PHR mode */
#define DW1000_PHRMODE_EXT	0x3     /* DW proprietary extended frames PHR mode */

// Defined constants for "mode" bitmask parameter passed into dwt_starttx() function. */
#define DW1000_START_TX_IMMEDIATE      0
#define DW1000_START_TX_DELAYED        1
#define DW1000_RESPONSE_EXPECTED       2

#define DW1000_START_RX_IMMEDIATE  0
#define DW1000_START_RX_DELAYED    1    // Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately */
#define DW1000_IDLE_ON_DLY_ERR     2    // If delayed RX failed due to "late" error then if this */
// flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits */
#define DW1000_NO_SYNC_PTRS        4    // Do not try to sync IC side and Host side buffer pointers when enabling RX. This is used to perform manual RX */
// re-enabling when receiving a frame in double buffer mode.

// Defined constants for "mode" bit field parameter passed to dwt_setleds() function.
#define DW1000_LEDS_DISABLE     0x00
#define DW1000_LEDS_ENABLE      0x01
#define DW1000_LEDS_INIT_BLINK  0x02

/* frame filtering configuration options */
#define DW1000_FF_NOTYPE_EN	0x000	/* no frame types allowed (FF disabled) */
#define DW1000_FF_COORD_EN	0x002	/* behave as coordinator (can receive frames with no dest address (PAN ID has to match)) */
#define DW1000_FF_BEACON_EN	0x004	/* beacon frames allowed */
#define DW1000_FF_DATA_EN	0x008	/* data frames allowed */
#define DW1000_FF_ACK_EN	0x010	/* ack frames allowed */
#define DW1000_FF_MAC_EN	0x020	/* mac control frames allowed */
#define DW1000_FF_RSVD_EN	0x040	/* reserved frame types allowed */

/* DW1000 interrupt events */
#define DW1000_INT_TFRS		0x00000080 /* frame sent */
#define DW1000_INT_LDED		0x00000400 /* micro-code has finished execution */
#define DW1000_INT_RFCG		0x00004000 /* frame received with good CRC */
#define DW1000_INT_RPHE		0x00001000 /* receiver PHY header error */
#define DW1000_INT_RFCE		0x00008000 /* receiver CRC error */
#define DW1000_INT_RFSL		0x00010000 /* receiver sync loss error */
#define DW1000_INT_RFTO		0x00020000 /* frame wait timeout */
#define DW1000_INT_RXOVRR	0x00100000 /* receiver overrun */
#define DW1000_INT_RXPTO	0x00200000 /* preamble detect timeout */
#define DW1000_INT_SFDT		0x04000000 /* SFD timeout */
#define DW1000_INT_ARFE		0x20000000 /* frame rejected (due to frame filtering configuration) */


/* DW1000 SLEEP and WAKEUP configuration parameters */
#define DW1000_PRESRV_SLEEP	0x0100 /* PRES_SLEEP - on wakeup preserve sleep bit */
#define DW1000_LOADOPSET	0x0080 /* ONW_L64P - on wakeup load operating parameter set for 64 PSR */
#define DW1000_CONFIG		0x0040 /* ONW_LDC - on wakeup restore (load) the saved configurations (from AON array into HIF) */
#define DW1000_RX_EN		0x0002 /* ONW_RX - on wakeup activate reception */
#define DW1000_TANDV		0x0001 /* ONW_RADC - on wakeup run ADC to sample temperature and voltage sensor values */

#define DW1000_XTAL_EN      0x10 /* keep XTAL running during sleep */
#define DW1000_WAKE_SLPCNT  0x8 /* wake up after sleep count */
#define DW1000_WAKE_CS      0x4 /* wake up on chip select */
#define DW1000_WAKE_WK      0x2 /* wake up on WAKEUP PIN */
#define DW1000_SLP_EN       0x1 /* enable sleep/deep sleep functionality */

/* DW1000 INIT configuration parameters */
#define DW1000_LOADUCODE     0x1
#define DW1000_LOADNONE      0x0

/* DW1000 OTP operating parameter set selection */
#define DW1000_OPSET_64LEN   0x0
#define DW1000_OPSET_TIGHT   0x1
#define DW1000_OPSET_DEFLT   0x2

/* Call-back data RX frames flags */
#define DW1000_CB_DATA_RX_FLAG_RNG 0x1 // Ranging bit */

/* TX/RX call-back data */
struct dw1000_cb_data{
	u32 status;      /* initial value of register as ISR is entered */
	u16 datalength;  /* length of frame */
	u8  fctrl[2];    /* frame control bytes */
	u8  rx_flags;    /* RX frame flags, see above */
};

#define NUM_BR		3
#define NUM_PRF		2
#define NUM_PACS	4
#define NUM_BW		2 /* two bandwidths are supported */
#define NUM_SFD		2 /* supported number of SFDs - standard = 0, non-standard = 1 */
#define NUM_CH		6 /* supported channels are 1, 2, 3, 4, 5, 7 */
#define NUM_CH_SUPPORTED 8 /* supported channels are '0', 1, 2, 3, 4, 5, '6', 7 */
#define PCODES		25 /* supported preamble codes */

#define PEAK_MULTPLIER  (0x60) //3 -> (0x3 * 32) & 0x00E0
#define N_STD_FACTOR    (13)
#define LDE_PARAM1      (PEAK_MULTPLIER | N_STD_FACTOR)

#define LDE_PARAM3_16 (0x1607)
#define LDE_PARAM3_64 (0x0607)

#define MIXER_GAIN_STEP (0.5)
#define DA_ATTN_STEP    (2.5)

#define DW1000_MAX_BUF 127

//-----------------------------------------
// map the channel number to the index in the configuration arrays below
// 0th element is chan 1, 1st is chan 2, 2nd is chan 3, 3rd is chan 4, 4th is chan 5, 5th is chan 7
const u8 CHAN_IDX[NUM_CH_SUPPORTED] = { 0, 0, 1, 2, 3, 4, 0, 5 };

//-----------------------------------------
const u32 TX_CONFIG[NUM_CH] =
{
	RF_TXCTRL_CH1,
	RF_TXCTRL_CH2,
	RF_TXCTRL_CH3,
	RF_TXCTRL_CH4,
	RF_TXCTRL_CH5,
	RF_TXCTRL_CH7,
};

//Frequency Synthesiser - PLL configuration
const u32 FS_PLL_CFG[NUM_CH] =
{
	FS_PLLCFG_CH1,
	FS_PLLCFG_CH2,
	FS_PLLCFG_CH3,
	FS_PLLCFG_CH4,
	FS_PLLCFG_CH5,
	FS_PLLCFG_CH7
};

//Frequency Synthesiser - PLL tuning
const u8 FS_PLL_TUNE[NUM_CH] =
{
	FS_PLLTUNE_CH1,
	FS_PLLTUNE_CH2,
	FS_PLLTUNE_CH3,
	FS_PLLTUNE_CH4,
	FS_PLLTUNE_CH5,
	FS_PLLTUNE_CH7
};

//bandwidth configuration
const u8 RX_CONFIG[NUM_BW] =
{
	RF_RXCTRLH_NBW,
	RF_RXCTRLH_WBW
};

struct agc_cfg_struct {
	u32 lo32;
	u16 target[NUM_PRF];
};

const struct agc_cfg_struct agc_config =
{
	AGC_TUNE2_VAL,
	{ AGC_TUNE1_16M, AGC_TUNE1_64M }  //adc target
};

/* DW1000 non-standard SFD length for 110k, 850k and 6.81M */
const u8 dw_ns_SFD_len[NUM_BR] =
{
	DW_NS_SFD_LEN_110K,
	DW_NS_SFD_LEN_850K,
	DW_NS_SFD_LEN_6M8
};

/* SFD Threshold */
const u16 SFD_THRESHOLD[NUM_BR][NUM_SFD] =
{
	{
		DRX_TUNE0b_110K_STD,
		DRX_TUNE0b_110K_NSTD
	},
	{
		DRX_TUNE0b_850K_STD,
		DRX_TUNE0b_850K_NSTD
	},
	{
		DRX_TUNE0b_6M8_STD,
		DRX_TUNE0b_6M8_NSTD
	}
};

const u16 DTUNE1[NUM_PRF] =
{
	DRX_TUNE1a_PRF16,
	DRX_TUNE1a_PRF64
};

const u32 DIGITAL_BB_CONFIG[NUM_PRF][NUM_PACS] =
{
	{
		DRX_TUNE2_PRF16_PAC8,
		DRX_TUNE2_PRF16_PAC16,
		DRX_TUNE2_PRF16_PAC32,
		DRX_TUNE2_PRF16_PAC64
	},
	{
		DRX_TUNE2_PRF64_PAC8,
		DRX_TUNE2_PRF64_PAC16,
		DRX_TUNE2_PRF64_PAC32,
		DRX_TUNE2_PRF64_PAC64
	}
};

const u16 LDE_REPLICA_COEFF[PCODES] =
{
	0, // No preamble code 0
	LDE_REPC_PCODE_1,
	LDE_REPC_PCODE_2,
	LDE_REPC_PCODE_3,
	LDE_REPC_PCODE_4,
	LDE_REPC_PCODE_5,
	LDE_REPC_PCODE_6,
	LDE_REPC_PCODE_7,
	LDE_REPC_PCODE_8,
	LDE_REPC_PCODE_9,
	LDE_REPC_PCODE_10,
	LDE_REPC_PCODE_11,
	LDE_REPC_PCODE_12,
	LDE_REPC_PCODE_13,
	LDE_REPC_PCODE_14,
	LDE_REPC_PCODE_15,
	LDE_REPC_PCODE_16,
	LDE_REPC_PCODE_17,
	LDE_REPC_PCODE_18,
	LDE_REPC_PCODE_19,
	LDE_REPC_PCODE_20,
	LDE_REPC_PCODE_21,
	LDE_REPC_PCODE_22,
	LDE_REPC_PCODE_23,
	LDE_REPC_PCODE_24
};

const double txpwr_compensation[NUM_CH] = { // TX_PWR_COMPENSATION
	0.0,
	0.035,
	0.0,
	0.0,
	0.065,
	0.0
};

typedef struct
{
    u8 channel ;
    u8 prf ;
    u8 datarate ;
    u8 preambleCode ;
    u8 preambleLength ;
    u8 pacSize ;
    u8 nsSFD ;
    u16 sfdTO ;
} chConfig_t ;


//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
chConfig_t chConfig[8] ={
	//mode 1 - S1: 7 off, 6 off, 5 off
	{
                        2,              // channel
                        DW1000_PRF_16M,    // prf
		DW1000_BR_110K,    // datarate
                        3,             // preambleCode
		DW1000_PLEN_1024,  // preambleLength
		DW1000_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
		DW1000_PRF_16M,    // prf
		DW1000_BR_6M8,    // datarate
                        3,             // preambleCode
		DW1000_PLEN_128,   // preambleLength
		DW1000_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
		DW1000_PRF_64M,    // prf
		DW1000_BR_110K,    // datarate
                        9,             // preambleCode
		DW1000_PLEN_1024,  // preambleLength
		DW1000_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
		DW1000_PRF_64M,    // prf
		DW1000_BR_6M8,    // datarate
                        9,             // preambleCode
		DW1000_PLEN_128,   // preambleLength
		DW1000_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
		DW1000_PRF_16M,    // prf
		DW1000_BR_110K,    // datarate
                        3,             // preambleCode
		DW1000_PLEN_1024,  // preambleLength
		DW1000_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
		DW1000_PRF_16M,    // prf
        DW1000_BR_6M8,    // datarate
                        3,             // preambleCode
        DW1000_PLEN_128,   // preambleLength
        DW1000_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,              // channel
        DW1000_PRF_64M,    // prf
        DW1000_BR_110K,    // datarate
                        9,             // preambleCode
        DW1000_PLEN_1024,  // preambleLength
        DW1000_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
                    {
                        5,              // channel
        DW1000_PRF_64M,    // prf
        DW1000_BR_6M8,    // datarate
                        9,             // preambleCode
        DW1000_PLEN_128,   // preambleLength
        DW1000_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    }
};

// Structure to hold device data
struct dw1000_data {
       u32 part_id;		/* IC Part ID - read during initialisation */
       u32 lot_id;		/* IC Lot ID - read during initialisation */
       u8 long_frame;		/* Flag in non-standard long frame mode */
       u8 otprev;		/* OTP revision number (read during initialisation) */
       u32 tx_fctrl_reg;	/* Keep TX_FCTRL register config */
       u8 init_xtrim;		/* initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed) */
       u8 dbl_buff_on;    	/* Double RX buffer mode flag */
       u32 sys_cfg_reg;   	/* Local copy of system config register */
       u16 sleep_mode;  	/* Used for automatic reloading of LDO tune and microcode at wake-up */
       u8 wait_for_resp;    	/* wait4response was set with last TX start command */
       struct dw1000_cb_data cb_data;	/* Callback data structure */
};

/*! ------------------------------------------------------------------------------------------------------------------
 * Structure typedef: dwt_config_t
 *
 * Structure for setting device configuration via dwt_configure() function
 *
 */
struct dw1000_config {
	u8 chan;		/* channel number {1, 2, 3, 4, 5, 7 } */
	u8 prf;			/* Pulse Repetition Frequency */
	u8 tx_preamble_length;	/* TX preamble length */
	u8 rx_pac;		/* Acquisition Chunk Size (Relates to RX preamble length) */
	u8 tx_code;		/* TX preamble code */
	u8 rx_code;		/* RX preamble code */
	u8 ns_sfd;		/* Boolean should we use non-standard SFD for better performance */
	u8 data_rate;		/* Data Rate */
	u8 phr_mode;		/* PHR mode */
	u16 sfd_timeout;	/* SFD timeout value (in symbols) */
	u16 short_addr;		/* Short address */
	u16 pan_id;		/* PAN ID */
};

struct dw1000_local {
	struct spi_device *spi;

	struct dw1000_data pdata;
	struct dw1000_config config;

	struct ieee802154_hw *hw;
	struct regmap *regmap;
	int slp_tr;
	int exton;
	int wakeup;
	bool sleep;

	u8 *buf;
	u8 rx_buf[DW1000_MAX_BUF];

	unsigned long cal_timeout;
	bool is_tx;
	bool is_tx_from_off;
	struct sk_buff *tx_skb;

	/* async spi write */
	struct spi_message reg_msg;
	u8 reg_addr[3];
	struct spi_transfer reg_addr_xfer;
	u32 reg_val;
	struct spi_transfer reg_val_xfer;

	/* async spi read */
	struct spi_message async_read_msg;
	u8 async_read_header[3];
	struct spi_transfer async_read_header_xfer;
//	u8 async_read_data[16];
	struct spi_transfer async_read_data_xfer;

};

/* DW1000 operational states */
//enum {
//    STATE_OFF   = 0x00,
//    STATE_WAKEUP    = 0x01,
//    STATE_INIT  = 0x02,
//    STATE_IDLE  = 0x03,
//    STATE_SLEEP = 0x04,
//    STATE_DEEPSLEEP = 0x05,
//    STATE_TX    = 0x06,
//    STATE_RX    = 0x07,
//    STATE_SNOOZE    = 0x08,
//};

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_async_read_reg() / dwt_readfromdevice
 *
 * @brief  this function is used to async read from the DW1000 device registers
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
 * @param lp		- struct dw1000_local
 * @param addr		- ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being read
 * @param data		- pointer to buffer in which to return the read data.
 *
 * output parameters
 *
 * returns SPI transmission status
 */
static int
dw1000_async_read_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 length, void *data, void (*complete)(void *context)) {
	int   cnt = 0;

	lp->async_read_data_xfer.rx_buf = data;
	lp->async_read_data_xfer.len = length;

	/* Write message header selecting READ operation and addresses as appropriate
	 *(this is one to three bytes long)
	 */
	/* For index of 0, no sub-index is required */
	if (index == 0) {
		/* Bit-7 zero is READ operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id */
		lp->async_read_header[cnt++] = (u8)addr;
	} else {
		/* Bit-7 zero is READ operation, bit-6 one=sub-address follows, bits 5-0 is reg file id */
		lp->async_read_header[cnt++] = (u8)(SUB_INDEX | addr);
		/* For non-zero index < 127, just a single sub-index byte is required */
		if (index <= 127) {
			/* Bit-7 zero means no extension, bits 6-0 is index */
			lp->async_read_header[cnt++] = (u8)index;

		} else {
			/* Bit-7 one means extended index, bits 6-0 is low seven bits of index */
			lp->async_read_header[cnt++] = ADDR_EXT | (u8)(index);
			/* 8-bit value = high eight bits of index */
			lp->async_read_header[cnt++] =  (u8)(index >> 7);
		}
	}

	lp->async_read_header_xfer.len = cnt;

	lp->async_read_msg.complete = complete;

	return spi_async(lp->spi, &lp->async_read_msg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_async_read_8bit_reg()
 *
 * @brief  this function is used to async read an 8-bit value from the DW1000 device registers
 *
 * input parameters:
 * @param lp - dw1000_local
 * @param addr - ID of register file or buffer being accessed
 * @param index - the index into register file or buffer being accessed
 * @param data - u8 data read from register
 * @param complete - spi callback function
 *
 * output parameters
 *
 * returns SPI transmission status
 */
static int
dw1000_async_read_8bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u8 *data, void (*complete)(void *context)) {
	// Read 1 bytes (8-bits) register into buffer
	return dw1000_async_read_reg(lp, addr, index, 1, data, complete);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_async_read_16bit_reg()
 *
 * @brief  this function is used to async read an 16-bit value from the DW1000 device registers
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
dw1000_async_read_16bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u16 *data, void (*complete)(void *context)) {
	int ret;

	// Read 2 bytes (16-bits) register into buffer
	ret = dw1000_async_read_reg(lp, addr, index, 2, data, complete);

	if (ret) {
		return ret;
	}

//	le16_to_cpus(data);

	return ret;
}

static int
dw1000_async_read_32bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 *data, void (*complete)(void *context)) {
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

	// Read 4 bytes (32-bits) register into buffer
	ret = dw1000_async_read_reg(lp, addr, index, 4, data, complete);

	if (ret) {
		return ret;
	}

//	le32_to_cpus(data);

	return ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_reg() / dwt_readfromdevice
 *
 * @brief  this function is used to sync read from the DW1000 device registers
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
	/* Buffer to compose header in */
	u8 header[3] = { 0, 0, 0};
	/* Counter for length of header */
	int   cnt = 0;
//	int i;
	struct spi_message msg;
	struct spi_transfer header_xfer = {
		.tx_buf = header,
	};

	struct spi_transfer data_xfer = {
		.len = length,
		.rx_buf = data,
	};

	/* Write message header selecting READ operation and addresses as appropriate (this is one to three bytes long) */
	/* For index of 0, no sub-index is required*/
	if (index == 0) {
		/* Bit-7 zero is READ operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id*/
		header[cnt++] = (u8)addr;
	} else {
		/* Bit-7 zero is READ operation, bit-6 one=sub-address follows, bits 5-0 is reg file id */
		header[cnt++] = (u8)(SUB_INDEX | addr); // 0x40

		/* For non-zero index < 127, just a single sub-index byte is required */
		if (index <= 127) {
			/* Bit-7 zero means no extension, bits 6-0 is index */
			header[cnt++] = (u8)index;
		} else {
			/* Bit-7 one means extended index, bits 6-0 is low seven bits of index */
			header[cnt++] = ADDR_EXT | (u8)(index);
			/* 8-bit value = high eight bits of index */
			header[cnt++] =  (u8)(index >> 7);
		}
	}

	header_xfer.len = cnt;

//	for (i = 0; i < cnt; i++) {
//		dev_dbg(printdev(lp), "addr[%d]:0x%x\n", i, header[i]);
//	}

	spi_message_init(&msg);
	spi_message_add_tail(&header_xfer, &msg);
	spi_message_add_tail(&data_xfer, &msg);

	return spi_sync(lp->spi, &msg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_8bit_reg()
 *
 * @brief  this function is used to sync read an 8-bit value from the DW1000 device registers
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
	return dw1000_read_reg(lp, addr, index, 1, data);
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
	ret = dw1000_read_reg(lp, addr, index, 2, data);

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
	ret = dw1000_read_reg(lp, addr, index, 4, data);

	if (ret) {
		return ret;
	}

	le32_to_cpus(data);

	return ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_async_write_reg()
 *
 * @brief  this function is used to async write to the DW1000 device registers
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
 * @param index		- byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param data		- pointer to buffer containing the 'length' bytes to be written
 * @param complete	- async spi callback function
 *
 * output parameters
 *
 * returns
 */
static int
dw1000_async_write_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 length, const void *data, void (*complete)(void *context)) {

	// Counter for length of header
	int   cnt = 0;

	dev_dbg(printdev(lp), "%s\n", __func__);

//  dev_dbg(printdev(lp), "data:0x%x\n", *(u32*)(data));

	lp->reg_val_xfer.len = length;
	lp->reg_val_xfer.tx_buf = data;

	// Write message header selecting WRITE operation and addresses as appropriate (this is one to three bytes long)
	// For index of 0, no sub-index is required
	if (index == 0) {
		// Bit-7 zero is WRITE operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
		lp->reg_addr[cnt++] = REG_WRITE | (u8)addr;
	} else {
		// Bit-7 is WRITE operation, bit-6 one=sub-address follows, bits 5-0 is reg file id
		lp->reg_addr[cnt++] = (u8)(REG_WRITE | SUB_INDEX | addr);

		// For non-zero index < 127, just a single sub-index byte is required
		if (index <= 127) {
			// Bit-7 zero means no extension, bits 6-0 is index.
			lp->reg_addr[cnt++] = (u8)index;
		} else {
			// Bit-7 one means extended index, bits 6-0 is low seven bits of index.
			lp->reg_addr[cnt++] = REG_WRITE | (u8)(index);
			// 8-bit value = high eight bits of index.
			lp->reg_addr[cnt++] =  (u8)(index >> 7);
		}
	}

	lp->reg_addr_xfer.len = cnt;

	lp->reg_msg.complete = complete;

	return spi_async(lp->spi, &lp->reg_msg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_async_write_8bit_reg / dwt_write8bitoffsetreg()
 *
 * @brief  this function is used to write an 8-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param lp	- struct dw1000_local
 * @param addr	- ID of register file or buffer being accessed
 * @param index	- the index into register file or buffer being accessed
 * @param data	- the value to write
 * @param complete - async spi callback function
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_async_write_8bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u8 *data, void (*complete)(void *context)) {
	return dw1000_async_write_reg(lp, addr, index, 1, data, complete);
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_async_write_16bit_reg / dwt_write16bitoffsetreg()
 *
 * @brief  this function is used to write 16-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param lp
 * @param addr	- ID of register file or buffer being accessed
 * @param index	- the index into register file or buffer being accessed
 * @param data	- the value to write
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_async_write_16bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u16 *data, void (*complete)(void *context)) {
	cpu_to_le16s(&data);
	return dw1000_async_write_reg(lp, addr, index, 2, data, complete);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_write_32bit_reg / dwt_write32bitoffsetreg()
 *
 * @brief  this function is used to write 32-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param lp	- struct dw1000_local
 * @param addr	- ID of register file or buffer being accessed
 * @param index	- the index into register file or buffer being accessed
 * @param data	- the value to write
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_async_write_32bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 *data, void (*complete)(void *context)) {
	cpu_to_le32s(&data);
	return dw1000_async_write_reg(lp, addr, index, 4, data, complete);
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
 * @param index		- byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param data		- pointer to buffer containing the 'length' bytes to be written
 *
 * output parameters
 *
 * returns
 */
static int
dw1000_write_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 length, const void *data)
{
	// Buffer to compose header in
	u8 header[3] = { 0, 0, 0 };
	// Counter for length of header
	int   cnt = 0;
//	int i;
	struct spi_message msg;

	struct spi_transfer header_xfer = {
		.tx_buf = header,
	};

	struct spi_transfer data_xfer = {
		.len = length,
		.tx_buf = data,
	};

//	dev_dbg(printdev(lp), "%s\n", __func__);

	// Write message header selecting WRITE operation and addresses as appropriate (this is one to three bytes long)
	// For index of 0, no sub-index is required
	if (index == 0) {
		// Bit-7 zero is WRITE operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
		header[cnt++] = REG_WRITE | (u8)addr;
	} else {
		// Bit-7 is WRITE operation, bit-6 one=sub-address follows, bits 5-0 is reg file id
		header[cnt++] = (u8)(REG_WRITE | SUB_INDEX | addr);

		// For non-zero index < 127, just a single sub-index byte is required
		if (index <= 127) {
			// Bit-7 zero means no extension, bits 6-0 is index.
			header[cnt++] = (u8)index;
		} else {
			// Bit-7 one means extended index, bits 6-0 is low seven bits of index.
			header[cnt++] = REG_WRITE | (u8)(index);
			// 8-bit value = high eight bits of index.
			header[cnt++] =  (u8)(index >> 7);
		}
	}

	header_xfer.len = cnt;

//	for (i = 0; i < cnt; i++) {
//		dev_dbg(printdev(lp), "write addr[%d]:0x%x\n", i, header[i]);
//	}

	spi_message_init(&msg);
	spi_message_add_tail(&header_xfer, &msg);
	spi_message_add_tail(&data_xfer, &msg);

	return spi_sync(lp->spi, &msg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_write_8bit_reg / dwt_write8bitoffsetreg()
 *
 * @brief  this function is used to write an 8-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param lp	- struct dw1000_local
 * @param addr	- ID of register file or buffer being accessed
 * @param index	- the index into register file or buffer being accessed
 * @param data	- the value to write
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_write_8bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u8 data)
{
	return dw1000_write_reg(lp, addr, index, 1, &data);
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_write_16bit_reg / dwt_write16bitoffsetreg()
 *
 * @brief  this function is used to write 16-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param lp
 * @param addr	- ID of register file or buffer being accessed
 * @param index	- the index into register file or buffer being accessed
 * @param data	- the value to write
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_write_16bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u16 data)
{
	cpu_to_le16s(&data);
	return dw1000_write_reg(lp, addr, index, 2, &data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_write_32bit_reg / dwt_write32bitoffsetreg()
 *
 * @brief  this function is used to write 32-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param lp	- struct dw1000_local
 * @param addr	- ID of register file or buffer being accessed
 * @param index	- the index into register file or buffer being accessed
 * @param data	- the value to write
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_write_32bit_reg(struct dw1000_local *lp, u16 addr, u16 index, u32 data)
{
	cpu_to_le32s(&data);
	return dw1000_write_reg(lp, addr, index, 4, &data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_enable_clocks()
 *
 * @brief function to enable/disable clocks to particular digital blocks/system
 *
 * input parameters
 * @param clocks - set of clocks to enable/disable
 *
 * output parameters none
 *
 * no return value
 */
void _dw1000_enable_clocks(struct dw1000_local *lp, int clocks)
{
	u8 reg[2];

	dev_dbg(printdev(lp), "%s\n", __func__);

	dw1000_read_reg(lp, PMSC_ID, PMSC_CTRL0_OFFSET, 2, reg);
	switch (clocks) {
	case ENABLE_ALL_SEQ:
		reg[0] = 0x00;
		reg[1] = reg[1] & 0xfe;
		break;
	case FORCE_SYS_XTI:
		// System and RX
		reg[0] = 0x01 | (reg[0] & 0xfc);
		break;
	case FORCE_SYS_PLL:
		// System
		reg[0] = 0x02 | (reg[0] & 0xfc);
		break;
	case READ_ACC_ON:
		reg[0] = 0x48 | (reg[0] & 0xb3);
		reg[1] = 0x80 | reg[1];
		break;
	case READ_ACC_OFF:
		reg[0] = reg[0] & 0xb3;
		reg[1] = 0x7f & reg[1];
		break;
	case FORCE_OTP_ON:
		reg[1] = 0x02 | reg[1];
		break;
	case FORCE_OTP_OFF:
		reg[1] = reg[1] & 0xfd;
		break;
	case FORCE_TX_PLL:
		reg[0] = 0x20 | (reg[0] & 0xcf);
		break;
	case FORCE_LDE:
		reg[0] = 0x01;
		reg[1] = 0x03;
		break;
	default:
		break;
	}


	// Need to write lower byte separately before setting the higher byte(s)
	dw1000_write_reg(lp, PMSC_ID, PMSC_CTRL0_OFFSET, 1, &reg[0]);
	dw1000_write_reg(lp, PMSC_ID, 0x1, 1, &reg[1]);

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_read_otp() / _dwt_otpread()
 *
 * @brief function to read the OTP memory. Ensure that MR,MRa,MRb are reset to 0.
 *
 * input parameters
 * @param address - address to read at
 *
 * output parameters
 *
 * returns the 32bit of read data
 */
static int
_dw1000_read_otp(struct dw1000_local *lp, u16 address, u32 *data)
{
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

	// Write the address
	ret = dw1000_write_16bit_reg(lp, OTP_IF_ID, OTP_ADDR, address);

	// Perform OTP Read - Manual read mode has to be set
	ret = dw1000_write_8bit_reg(lp, OTP_IF_ID, OTP_CTRL, OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN);
	// OTPREAD is self clearing but OTPRDEN is not
	ret = dw1000_write_8bit_reg(lp, OTP_IF_ID, OTP_CTRL, 0x00);

	// Read read data, available 40ns after rising edge of OTP_READ
	ret = dw1000_read_32bit_reg(lp, OTP_IF_ID, OTP_RDAT, data);

	return ret;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_otp() / dwt_otpread()
 *
 * @brief This is used to read the OTP data from given address into provided array
 *
 * input parameters
 * @param address - this is the OTP address to read from
 * @param array - this is the pointer to the array into which to read the data
 * @param length - this is the number of 32 bit words to read (array needs to be at least this length)
 *
 * output parameters
 *
 * no return value
 */
static void
dw1000_read_otp(struct dw1000_local *lp, u32 address, u8 length, u32 *array)
{
	int i;

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* NOTE: Set system clock to XTAL - this is necessary to make sure the values read by _dwt_otpread are reliable */
	_dw1000_enable_clocks(lp, FORCE_SYS_XTI);

	for (i = 0; i < length; i++) {
		_dw1000_read_otp(lp, address + i, (array +i));
	}

	/* Restore system clock to PLL */
	_dw1000_enable_clocks(lp, ENABLE_ALL_SEQ);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_set_otp_mr_regs / _dwt_otpsetmrregs()
 *
 * @brief Configure the MR registers for initial programming (enable charge pump).
 * Read margin is used to stress the read back from the
 * programmed bit. In normal operation this is relaxed.
 *
 * input parameters
 * @param mode - "0" : Reset all to 0x0:           MRA=0x0000, MRB=0x0000, MR=0x0000
 *               "1" : Set for inital programming: MRA=0x9220, MRB=0x000E, MR=0x1024
 *               "2" : Set for soak programming:   MRA=0x9220, MRB=0x0003, MR=0x1824
 *               "3" : High Vpp:                   MRA=0x9220, MRB=0x004E, MR=0x1824
 *               "4" : Low Read Margin:            MRA=0x0000, MRB=0x0003, MR=0x0000
 *               "5" : Array Clean:                MRA=0x0049, MRB=0x0003, MR=0x0024
 *               "4" : Very Low Read Margin:       MRA=0x0000, MRB=0x0003, MR=0x0000
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
static int
_dw1000_set_otp_mr_regs(struct dw1000_local *lp, int mode)
{
	u8 rd_buf[4];
	u8 wr_buf[4];
	u32 mra = 0, mrb = 0, mr = 0;

	dev_dbg(printdev(lp), "%s\n", __func__);

	// PROGRAMME MRA
	// Set MRA, MODE_SEL
	wr_buf[0] = 0x03;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

	// Load data
	switch (mode & 0x0f) {
	case 0x0 :
		mr = 0x0000;
		mra = 0x0000;
		mrb = 0x0000;
		break;
	case 0x1 :
		mr = 0x1024;
		mra = 0x9220; // Enable CPP mon
		mrb = 0x000e;
		break;
	case 0x2 :
		mr = 0x1824;
		mra = 0x9220;
		mrb = 0x0003;
		break;
	case 0x3 :
		mr = 0x1824;
		mra = 0x9220;
		mrb = 0x004e;
		break;
	case 0x4 :
		mr = 0x0000;
		mra = 0x0000;
		mrb = 0x0003;
		break;
	case 0x5 :
		mr = 0x0024;
		mra = 0x0000;
		mrb = 0x0003;
		break;
	default :
		return DW1000_ERROR;
	}

	wr_buf[0] = mra & 0x00ff;
	wr_buf[1] = (mra & 0xff00) >> 8;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_WDAT, 2, wr_buf);


	// Set WRITE_MR
	wr_buf[0] = 0x08;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);

	// Wait?

	// Set Clear Mode sel
	wr_buf[0] = 0x02;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

	// Set AUX update, write MR
	wr_buf[0] = 0x88;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);
	// Clear write MR
	wr_buf[0] = 0x80;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);
	// Clear AUX update
	wr_buf[0] = 0x00;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);

	///////////////////////////////////////////
	// PROGRAM MRB
	// Set SLOW, MRB, MODE_SEL
	wr_buf[0] = 0x05;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

	wr_buf[0] = mrb & 0x00ff;
	wr_buf[1] = (mrb & 0xff00) >> 8;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_WDAT, 2, wr_buf);

	// Set WRITE_MR
	wr_buf[0] = 0x08;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);

	// Wait?

	// Set Clear Mode sel
	wr_buf[0] = 0x04;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

	// Set AUX update, write MR
	wr_buf[0] = 0x88;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);
	// Clear write MR
	wr_buf[0] = 0x80;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);
	// Clear AUX update
	wr_buf[0] = 0x00;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);

	///////////////////////////////////////////
	// PROGRAM MR
	// Set SLOW, MODE_SEL
	wr_buf[0] = 0x01;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
	// Load data

	wr_buf[0] = mr & 0x00ff;
	wr_buf[1] = (mr & 0xff00) >> 8;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_WDAT, 2, wr_buf);

	// Set WRITE_MR
	wr_buf[0] = 0x08;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);

	// Wait?
	usleep_range(100, 120);
	// Set Clear Mode sel
	wr_buf[0] = 0x00;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

	// Read confirm mode writes.
	// Set man override, MRA_SEL
	wr_buf[0] = OTP_CTRL_OTPRDEN;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);
	wr_buf[0] = 0x02;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
	// MRB_SEL
	wr_buf[0] = 0x04;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
	usleep_range(100, 120);

	// Clear mode sel
	wr_buf[0] = 0x00;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
	// Clear MAN_OVERRIDE
	wr_buf[0] = 0x00;
	dw1000_write_reg(lp, OTP_IF_ID, OTP_CTRL, 1, wr_buf);

	usleep_range(10, 16);

	if (((mode & 0x0f) == 0x1) || ((mode & 0x0f) == 0x2)) {
		// Read status register
		dw1000_read_reg(lp, OTP_IF_ID, OTP_STAT, 1, rd_buf);
	}

	return DW1000_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_sync_rx_buf_ptrs / dwt_syncrxbufptrs()
 *
 * @brief this function synchronizes rx buffer pointers
 * need to make sure that the host/IC buffer pointers are aligned before starting RX
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dw1000_sync_rx_buf_ptrs(struct dw1000_local *lp) {
	u8  buff;
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Need to make sure that the host/IC buffer pointers are aligned before starting RX */
	/* Read 1 byte at offset 3 to get the 4th byte out of 5 */
	ret = dw1000_read_8bit_reg(lp, SYS_STATUS_ID, 3, &buff);

	/* IC side Receive Buffer Pointer */
	if ((buff & (SYS_STATUS_ICRBP >> 24)) !=
	    /* Host Side Receive Buffer Pointer */
		((buff & (SYS_STATUS_HSRBP >> 24)) << 1)) {
		/* We need to swap RX buffer status reg (write one to toggle internally) */
		dw1000_write_8bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 0x01);
	}
}

static inline void
dw1000_sleep(struct dw1000_local *lp)
{

}

static inline void
dw1000_awake(struct dw1000_local *lp)
{

}

static void
dw1000_irq_enable_rx_complete(void *context) {

    struct dw1000_local *lp = context;

    dev_dbg(printdev(lp), "%s\n", __func__);

//  enable_irq(lp->spi->irq);
}

static void
dw1000_irq_read_rx_buf_complete(void *context)
{
	int ret;
	struct sk_buff *skb;
	struct dw1000_local *lp = context;

	u16 len = lp->pdata.cb_data.datalength;
//  u32 status = lp->pdata.cb_data.status;

	dev_dbg(printdev(lp), "%s\n", __func__);

	// Report frame control - First bytes of the received frame.
//	dwt_readfromdevice(RX_BUFFER_ID, 0, FCTRL_LEN_MAX, pdw1000local->cbData.fctrl);
	memcpy(lp->pdata.cb_data.fctrl, lp->rx_buf, FCTRL_LEN_MAX);

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "dw1000 rx: ", DUMP_PREFIX_OFFSET, 16, 1,
		       lp->rx_buf, len, 0);
//	pr_info("dw1000 rx: lqi: %02hhx\n", lp->rx_lqi[0]);
#endif

	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_vdbg(&lp->spi->dev, "corrupted frame received\n");
		len = IEEE802154_MTU;
	}

	len = len - 2;  /* get rid of frame check field */

	skb = dev_alloc_skb(len);
	if (!skb)
		return;

	memcpy(skb_put(skb, len), lp->rx_buf, len);

    ieee802154_rx_irqsafe(lp->hw, skb, 0);

//  enable_irq(lp->spi->irq);

    /* Enable rx */
    lp->reg_val = SYS_CTRL_RXENAB;
    ret = dw1000_async_write_16bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, (u16 *)&lp->reg_val, dw1000_irq_enable_rx_complete);
    if (ret)
        dev_err(printdev(lp), "failed to enable rx\n");



	// Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
	// acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
	// implementation works only for IEEE802.15.4-2011 compliant frames).
	// This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
//  if ((status & SYS_STATUS_AAT) && ((lp->pdata.cb_data.fctrl[0] & FCTRL_ACK_REQ_MASK) == 0)) {
		/* Clear AAT status bit in register */
//		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_AAT);
//  	lp->reg_val = SYS_STATUS_AAT;
//  	dw1000_async_write_32bit_reg(lp, SYS_STATUS_ID, 0, &lp->reg_val, NULL);
//
//  	/* Clear AAT status bit in callback data register copy */
//  	lp->pdata.cb_data.status &= ~SYS_STATUS_AAT;
//  	lp->pdata.wait_for_resp = 0;
//  }

	/* Read diagnostics data. */
//	dwt_readdiagnostics(&rx_diag);

	/* Read accumulator. See NOTES 2 and 6. */
//	uint16 fp_int = rx_diag.firstPath / 64;
//	dwt_readaccdata(accum_data, ACCUM_DATA_LEN, (fp_int - 2) * 4);
}

static void
dw1000_read_rx_fifo_info_complete(void *context)
{
	int ret;
	struct dw1000_local *lp = context;
	u16 *len = &lp->pdata.cb_data.datalength;

	dev_dbg(printdev(lp), "%s\n", __func__);

	// Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
//	lp->pdata.cbData.datalength &= RX_FINFO_RXFL_MASK_1023;
	if (lp->pdata.long_frame == 0) {
		*len &= RX_FINFO_RXFLEN_MASK;
	}
//	pdw1000local->cbData.datalength = len;

	dev_dbg(printdev(lp), "received frame len : %d\n", *len);

	// Report ranging bit
	if (*len & RX_FINFO_RNG) {
		lp->pdata.cb_data.rx_flags |= DW1000_CB_DATA_RX_FLAG_RNG;
	}

	ret = dw1000_async_read_reg(lp, RX_BUFFER_ID, 0, *len, &lp->rx_buf, dw1000_irq_read_rx_buf_complete);
	if (ret)
		dev_err(printdev(lp), "failed to read rx buf\n");
}

static void
dw1000_clear_rx_status_complete(void *context)
{
	int ret;
	struct dw1000_local *lp = context;

	dev_dbg(printdev(lp), "%s\n", __func__);

    enable_irq(lp->spi->irq);

	lp->pdata.cb_data.rx_flags = 0;

	/* Read frame info - Only the first two bytes of the register are used here */
	ret = dw1000_async_read_16bit_reg(lp, RX_FINFO_ID, RX_FINFO_OFFSET, &lp->pdata.cb_data.datalength, dw1000_read_rx_fifo_info_complete);

	if (ret)
		dev_err(printdev(lp), "failed to read rx fifo info\n");
}

static void
dw1000_clear_rx_reset_complete(void *context)
{
//	int ret;
	struct dw1000_local *lp = context;

	dev_dbg(printdev(lp), "%s\n", __func__);

	ieee802154_xmit_complete(lp->hw, lp->tx_skb, false);
}

static void
dw1000_set_rx_reset_complete(void *context)
{
	int ret;
	struct dw1000_local *lp = context;

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Clear RX reset */
	lp->reg_val = PMSC_CTRL0_RESET_CLEAR;
	ret = dw1000_async_write_8bit_reg(lp, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, (u8 *)&lp->reg_val, dw1000_clear_rx_reset_complete);
	if (ret)
		dev_err(printdev(lp), "failed to clear rx reset\n");

}

static void
dw1000_clear_tx_status_complete(void *context)
{
	int ret;
	struct dw1000_local *lp = context;
	u32 status = lp->pdata.cb_data.status;

	dev_dbg(printdev(lp), "%s\n", __func__);

    enable_irq(lp->spi->irq);

	// In the case where this TXFRS interrupt is due to the automatic transmission of an ACK solicited by a response (with ACK request bit set)
	// that we receive through using wait4resp to a previous TX (and assuming that the IRQ processing of that TX has already been handled), then
	// we need to handle the IC issue which turns on the RX again in this situation (i.e. because it is wrongly applying the wait4resp after the
	// ACK TX).
	// See section "Transmit and automatically wait for response" in DW1000 User Manual
//  if ((status & SYS_STATUS_AAT) && lp->pdata.wait_for_resp) {

//		dwt_forcetrxoff(); // Turn the RX off

//  	lp->pdata.wait_for_resp = 0;
//
//  	/* Reset RX in case we were late and a frame was already being received */
//  	/* Set RX reset */
//  	lp->reg_val = PMSC_CTRL0_RESET_RX;
//  	ret = dw1000_async_write_8bit_reg(lp, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET,  (u8 *)&lp->reg_val, dw1000_set_rx_reset_complete);
//  	if (ret)
//  		dev_err(printdev(lp), "failed to set rx reset\n");
//  } else {
		lp->is_tx = 0;
		ieee802154_xmit_complete(lp->hw, lp->tx_skb, false);

//      enable_irq(lp->spi->irq);

        /* Enable rx */
        lp->reg_val = SYS_CTRL_RXENAB;
        ret = dw1000_async_write_16bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, (u16 *)&lp->reg_val, dw1000_irq_enable_rx_complete);
        if (ret)
            dev_err(printdev(lp), "failed to enable rx\n");
//  }
}

static void
dw1000_clear_rx_err_complete(void *context)
{
	int ret;
	struct dw1000_local *lp = context;

	dev_dbg(printdev(lp), "%s\n", __func__);


	// Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
	// the next good frame's timestamp is computed correctly.
	// See section "RX Message timestamp" in DW1000 User Manual.

	/* Set RX reset */
	lp->reg_val = PMSC_CTRL0_RESET_RX;
	ret = dw1000_async_write_8bit_reg(lp, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, (u8 *)&lp->reg_val, dw1000_set_rx_reset_complete);

	if (ret)
		dev_err(printdev(lp), "failed to set rx reset\n");
}

static void
dw1000_irq_read_status_complete(void *context)
{
	int ret;
	u32 status;

	struct dw1000_local *lp = context;

	le32_to_cpus(&lp->pdata.cb_data.status);
	status = lp->pdata.cb_data.status;

	dev_dbg(printdev(lp), "%s\n", __func__);
	dev_dbg(printdev(lp), "SYS_STATUS: 0x%x\n", status);

	/* Handle RX good frame event */
	if (status & SYS_STATUS_RXFCG) {
		dev_dbg(printdev(lp), "RX is done\n");
		/* Clear all receive status bits */

		lp->reg_val = SYS_STATUS_ALL_RX_GOOD;
		ret = dw1000_async_write_32bit_reg(lp, SYS_STATUS_ID, 0, &lp->reg_val, dw1000_clear_rx_status_complete);
		if (ret)
			dev_err(printdev(lp), "failed to clear all receive status bits\n");
	}

	/* Handle TX confirmation event */
	if (status & SYS_STATUS_TXFRS) {
        if (lp->is_tx) {
            dev_dbg(printdev(lp), "TX is done\n");
            /* Clear TX event bits */
            lp->reg_val = SYS_STATUS_ALL_TX;
            ret = dw1000_async_write_32bit_reg(lp, SYS_STATUS_ID, 0, &lp->reg_val, dw1000_clear_tx_status_complete);
            if (ret)
                dev_err(printdev(lp), "failed to clear TX event bits\n");
        } else {
            dev_dbg(printdev(lp), "ACK is done\n");
        }
	}

	// Handle frame reception/preamble detect timeout events
//	if (lp->irq_status & SYS_STATUS_ALL_RX_TO) {
//		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO); // Clear RX timeout event bits
//
//		pdw1000local->wait4resp = 0;

		// Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
		// the next good frame's timestamp is computed correctly.
		// See section "RX Message timestamp" in DW1000 User Manual.
//		dwt_forcetrxoff();
//		dwt_rxreset();

		// Call the corresponding callback if present
//		if (pdw1000local->cbRxTo != NULL) {
//			pdw1000local->cbRxTo(&pdw1000local->cbData);
//		}
//	}

	/* Handle RX errors events */
//  if (status & SYS_STATUS_ALL_RX_ERR) {
//
//      lp->pdata.wait_for_resp = 0;
//
//      /* Clear RX error event bits */
//      lp->reg_val = SYS_STATUS_ALL_RX_ERR;
//      ret = dw1000_async_write_32bit_reg(lp, SYS_STATUS_ID, 0, &lp->reg_val, dw1000_clear_rx_err_complete);
//      if (ret)
//          dev_err(printdev(lp), "failed to clear rx error event bits\n");
//  }

	/* Clear events */
//	dev_dbg(printdev(lp), "clear not concerned events\n");

//	ret = dw1000_async_write_32bit_reg(lp, SYS_STATUS_ID, 0, status, NULL);
//	if (ret)
//		dev_err(printdev(lp), "failed to clear not concerned events\n");
}

static irqreturn_t
dw1000_isr(int irq, void *data)
{
	struct dw1000_local *lp = data;
	int ret;

	disable_irq_nosync(irq);

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Read status register low 32bits */
	ret = dw1000_async_read_32bit_reg(lp, SYS_STATUS_ID, 0, &lp->pdata.cb_data.status, dw1000_irq_read_status_complete);

	if (ret) {
		dev_dbg(printdev(lp), "failed to async spi read\n");
		enable_irq(irq);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static void
dw1000_set_tx_fctrl_complete(void *context)
{
	int ret;
	struct dw1000_local *lp = context;

	dev_dbg(printdev(lp), "%s\n", __func__);

	lp->reg_val = SYS_CTRL_TXSTRT;
	ret = dw1000_async_write_8bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, (u8 *)&lp->reg_val, NULL);
	if (ret)
		dev_err(printdev(lp), "failed to start tx\n");
}

static void
dw1000_write_tx_buff_complete(void *context)
{
	int ret;
	u16 txBufferOffset = 0;
	/* no ranging */
	int ranging = 0;
	struct dw1000_local *lp = context;
	unsigned int len = lp->tx_skb->len;


	len = len + 2;

	/* Zero offset in TX buffer, no ranging. */
	lp->reg_val = lp->pdata.tx_fctrl_reg | len | (txBufferOffset << TX_FCTRL_TXBOFFS_SHFT) | (ranging << TX_FCTRL_TR_SHFT);

	dev_dbg(printdev(lp), "%s\n", __func__);

	dev_dbg(printdev(lp), "TX_FCTRL:0x%x\n", lp->reg_val);

	ret = dw1000_async_write_32bit_reg(lp, TX_FCTRL_ID, 0, &lp->reg_val, dw1000_set_tx_fctrl_complete);
	if (ret)
		dev_err(printdev(lp), "failed to set tx fctrl\n");

}

static void
dw1000_set_trxoff_complete(void *context)
{
	int ret;
	struct dw1000_local *lp = context;

	dev_dbg(printdev(lp), "%s\n", __func__);

	lp->is_tx = 1;

	/* Write frame data to DW1000 and prepare transmission */
	ret = dw1000_async_write_reg(lp, TX_BUFFER_ID, 0, lp->tx_skb->len, lp->tx_skb->data, dw1000_write_tx_buff_complete);
	if (ret)
		dev_err(printdev(lp), "failed to write tx buff\n");
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_xmit / dwt_starttx()
 *
 * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
 *
 * input parameters:
 * @param mode - if 0 immediate TX (no response expected)
 *               if 1 delayed TX (no response expected)
 *               if 2 immediate TX (response expected - so the receiver will be automatically turned on after TX is done)
 *               if 3 delayed TX (response expected - so the receiver will be automatically turned on after TX is done)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed transmission will fail if the delayed time has passed)
 */
static int
dw1000_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	int ret;
	struct dw1000_local *lp = hw->priv;

	dev_dbg(printdev(lp), "%s\n", __func__);

	lp->tx_skb = skb;

	print_hex_dump_debug("dw1000 tx: ", DUMP_PREFIX_OFFSET, 16, 1,
			     skb->data, skb->len, 0);

	lp->reg_val = SYS_CTRL_TRXOFF;
	/* Turn off rx at first */
	ret = dw1000_async_write_8bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, (u8 *)&lp->reg_val, dw1000_set_trxoff_complete);
	if (ret)
		dev_err(printdev(lp), "failed to turn off rx\n");

	return 0;
}

static int
dw1000_ed(struct ieee802154_hw *hw, u8 *level)
{
	return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_enable_frame_filter / dwt_enableframefilter()
 *
 * @brief This is used to enable the frame filtering - (the default option is to
 * accept any data and ACK frames with correct destination address
 *
 * input parameters
 * @param - bitmask - enables/disables the frame filtering options according to
 *      DWT_FF_NOTYPE_EN        0x000   no frame types allowed
 *      DWT_FF_COORD_EN         0x002   behave as coordinator (can receive frames with no destination address (PAN ID has to match))
 *      DWT_FF_BEACON_EN        0x004   beacon frames allowed
 *      DWT_FF_DATA_EN          0x008   data frames allowed
 *      DWT_FF_ACK_EN           0x010   ack frames allowed
 *      DWT_FF_MAC_EN           0x020   mac control frames allowed
 *      DWT_FF_RSVD_EN          0x040   reserved frame types allowed
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_enable_frame_filter(struct dw1000_local *lp, u16 enable)
{
	int ret;
	u32 sysconfig;

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Read sysconfig register */
	ret = dw1000_read_32bit_reg(lp, SYS_CFG_ID, 0, &sysconfig);
	if (ret) {
		return ret;
	}
	sysconfig &= SYS_CFG_MASK;

	if (enable) {
		/* Enable frame filtering and configure frame types */
		sysconfig &= ~(SYS_CFG_FF_ALL_EN); /* Clear all */
		sysconfig |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
	} else {
		sysconfig &= ~(SYS_CFG_FFE);
	}

	lp->pdata.sys_cfg_reg = sysconfig;
	return dw1000_write_32bit_reg(lp, SYS_CFG_ID, 0, sysconfig);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_enable_auto_ack / dwt_enableautoack()
 *
 * @brief This call enables the auto-ACK feature. If the responseDelayTime (parameter) is 0, the ACK will be sent a.s.a.p.
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well
 *
 * input parameters
 * @param response_delay_time - if non-zero the ACK is sent after this delay, max is 255.
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_enable_auto_ack(struct dw1000_local *lp, u8 response_delay_time)
{
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Set auto ACK reply delay */
	/* In symbols */
	ret = dw1000_write_8bit_reg(lp, ACK_RESP_T_ID, ACK_RESP_T_ACK_TIM_OFFSET, response_delay_time);
	if (ret) {
		return ret;
	}

	/* Enable auto ACK */
	lp->pdata.sys_cfg_reg |= SYS_CFG_AUTOACK;
	return dw1000_write_32bit_reg(lp, SYS_CFG_ID, 0, lp->pdata.sys_cfg_reg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_setinterrupt / dwt_setinterrupt()
 *
 * @brief This function enables the specified events to trigger an interrupt.
 * The following events can be enabled:
 * DWT_INT_TFRS         0x00000080          // frame sent
 * DWT_INT_RFCG         0x00004000          // frame received with good CRC
 * DWT_INT_RPHE         0x00001000          // receiver PHY header error
 * DWT_INT_RFCE         0x00008000          // receiver CRC error
 * DWT_INT_RFSL         0x00010000          // receiver sync loss error
 * DWT_INT_RFTO         0x00020000          // frame wait timeout
 * DWT_INT_RXPTO        0x00200000          // preamble detect timeout
 * DWT_INT_SFDT         0x04000000          // SFD timeout
 * DWT_INT_ARFE         0x20000000          // frame rejected (due to frame filtering configuration)
 *
 *
 * input parameters:
 * @param bitmask - sets the events which will generate interrupt
 * @param enable - if set the interrupts are enabled else they are cleared
 *
 * output parameters
 *
 * no return value
 */
static int
dw1000_setinterrupt(struct dw1000_local *lp, u32 bitmask, u8 enable) {

	int ret;
//	decaIrqStatus_t stat;
	u32 mask;

	/* Need to beware of interrupts occurring in the middle of following read modify write cycle */
//	stat = decamutexon();

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Read irq mask */
	ret = dw1000_read_32bit_reg(lp, SYS_MASK_ID, 0, &mask);

	if (enable) {
		mask |= bitmask;
	} else {
		/* Clear the bit */
		mask &= ~bitmask;
	}

	/* write new value */
	return dw1000_write_32bit_reg(lp, SYS_MASK_ID, 0, mask);

//	decamutexoff(stat);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_lna_pa_mode / dwt_setlnapamode()
 *
 * @brief This is used to enable GPIO for external LNA or PA functionality - HW dependent, consult the DW1000 User Manual.
 *        This can also be used for debug as enabling TX and RX GPIOs is quite handy to monitor DW1000's activity.
 *
 * NOTE: Enabling PA functionality requires that fine grain TX sequencing is deactivated. This can be done using
 *       dwt_setfinegraintxseq().
 *
 * input parameters
 * @param lna - 1 to enable LNA functionality, 0 to disable it
 * @param pa - 1 to enable PA functionality, 0 to disable it
 *
 * output parameters
 *
 * no return value
 */
void dw1000_set_lna_pa_mode(struct dw1000_local *lp, int lna, int pa) {
	u32 gpio_mode;
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

//	uint32 gpio_mode = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
	ret = dw1000_read_32bit_reg(lp, GPIO_CTRL_ID, GPIO_MODE_OFFSET, &gpio_mode);

	gpio_mode &= ~(GPIO_MSGP4_MASK | GPIO_MSGP5_MASK | GPIO_MSGP6_MASK);
	if (lna) {
		gpio_mode |= GPIO_PIN6_EXTRXE;
	}
	if (pa) {
		gpio_mode |= (GPIO_PIN5_EXTTXE | GPIO_PIN4_EXTPA);
	}
//	dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, gpio_mode);
	ret = dw1000_write_32bit_reg(lp, GPIO_CTRL_ID, GPIO_MODE_OFFSET, gpio_mode);
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_leds / dwt_setleds()
 *
 * @brief This is used to set up Tx/Rx GPIOs which could be used to control LEDs
 * Note: not completely IC dependent, also needs board with LEDS fitted on right I/O lines
 *       this function enables GPIOs 2 and 3 which are connected to LED3 and LED4 on EVB1000
 *
 * input parameters
 * @param mode - this is a bit field interpreted as follows:
 *          - bit 0: 1 to enable LEDs, 0 to disable them
 *          - bit 1: 1 to make LEDs blink once on init. Only valid if bit 0 is set (enable LEDs)
 *          - bit 2 to 7: reserved
 *
 * output parameters none
 *
 * no return value
 */
void dw1000_set_leds(struct dw1000_local *lp, u8 mode) {
	u32 reg;
	int ret;

	dev_dbg(printdev(lp), "%s\n", __func__);

	if (mode & DW1000_LEDS_ENABLE) {
		/* Set up MFIO for LED output */
//		reg = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
		ret = dw1000_read_32bit_reg(lp, GPIO_CTRL_ID, GPIO_MODE_OFFSET, &reg);
		reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
		reg |= (GPIO_PIN2_RXLED | GPIO_PIN3_TXLED);
//		dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);
		ret = dw1000_write_32bit_reg(lp, GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);

		/* Enable LP Oscillator to run from counter and turn on de-bounce clock */
//		reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
		ret = dw1000_read_32bit_reg(lp, PMSC_ID, PMSC_CTRL0_OFFSET, &reg);
		reg |= (PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLEN);
//		dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, reg);
		ret = dw1000_write_32bit_reg(lp, PMSC_ID, PMSC_CTRL0_OFFSET, reg);

		/* Enable LEDs to blink and set default blink time. */
		reg = PMSC_LEDC_BLNKEN | PMSC_LEDC_BLINK_TIME_DEF;
		/* Make LEDs blink once if requested. */
		if (mode & DW1000_LEDS_INIT_BLINK) {
			reg |= PMSC_LEDC_BLINK_NOW_ALL;
		}
//		dwt_write32bitoffsetreg(PMSC_ID, PMSC_LEDC_OFFSET, reg);
		ret = dw1000_write_32bit_reg(lp, PMSC_ID, PMSC_LEDC_OFFSET, reg);
		/* Clear force blink bits if needed. */
		if (mode & DW1000_LEDS_INIT_BLINK) {
			reg &= ~PMSC_LEDC_BLINK_NOW_ALL;
//			dwt_write32bitoffsetreg(PMSC_ID, PMSC_LEDC_OFFSET, reg);
			ret = dw1000_write_32bit_reg(lp, PMSC_ID, PMSC_LEDC_OFFSET, reg);
		}
	} else {
		/* Clear the GPIO bits that are used for LED control. */
//		reg = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
		ret = dw1000_read_32bit_reg(lp, GPIO_CTRL_ID, GPIO_MODE_OFFSET, &reg);
		reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
//		dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);
		ret = dw1000_write_32bit_reg(lp, GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_force_trx_off / dwt_forcetrxoff()
 *
 * @brief This is used to turn off the transceiver
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
static void
dw1000_force_trx_off(struct dw1000_local *lp)
{
//	decaIrqStatus_t stat;
	u32 mask;

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Read set interrupt mask */
	dw1000_read_32bit_reg(lp, SYS_MASK_ID, 0, &mask);

	// Need to beware of interrupts occurring in the middle of following read modify write cycle
	// We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
	// event has just happened before the radio was disabled)
	// thus we need to disable interrupt during this

	disable_irq(lp->spi->irq);

//	stat = decamutexon();

	/* Clear interrupt mask - so we don't get any unwanted events */
	dw1000_write_32bit_reg(lp, SYS_MASK_ID, 0, 0);

	/* Disable the radio */
	dw1000_write_8bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, (u8)SYS_CTRL_TRXOFF);

	/* Forcing Transceiver off - so we do not want to see any new events that may have happened */
	dw1000_write_32bit_reg(lp, SYS_STATUS_ID, 0, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD));

	dw1000_sync_rx_buf_ptrs(lp);

	dw1000_write_32bit_reg(lp, SYS_MASK_ID, 0, mask); // Set interrupt mask to what it was */

	// Enable/restore interrupts again... */
//	decamutexoff(stat);
	enable_irq(lp->spi->irq);

	lp->pdata.wait_for_resp = 0;

} // end deviceforcetrxoff()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_start / dwt_rxenable()
 *
 * @brief This call turns on the receiver, can be immediate or delayed (depending on the mode parameter). In the case of a
 * "late" error the receiver will only be turned on if the DWT_IDLE_ON_DLY_ERR is not set.
 * The receiver will stay turned on, listening to any messages until
 * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or  it times out (SFD, Preamble or Frame).
 *
 * input parameters
 * @param mode - this can be one of the following allowed values:
 *
 * DWT_START_RX_IMMEDIATE      0 used to enbale receiver immediately
 * DWT_START_RX_DELAYED        1 used to set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
 * (DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR) 3 used to disable re-enabling of receiver if delayed RX failed due to "late" error
 * (DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS) 4 used to re-enable RX without trying to sync IC and host side buffer pointers, typically when
 *                                               performing manual RX re-enabling in double buffering mode
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed receive enable will be too far in the future if delayed time has passed)
 */
static int
dw1000_start(struct ieee802154_hw *hw)
{
//  int mode = DW1000_START_RX_IMMEDIATE;
//  u16 mode = (u16)SYS_CTRL_RXENAB;

  struct dw1000_local *lp = hw->priv;

	dev_dbg(printdev(lp), "%s\n", __func__);

//	if ((mode & DW1000_NO_SYNC_PTRS) == 0) {
//		dw1000_sync_rx_buf_ptrs(lp);
//	}

	/* Configure frame filtering. Frame filtering must be enabled for Auto ACK to work. */
    dw1000_enable_frame_filter(lp, DW1000_FF_DATA_EN | DW1000_FF_BEACON_EN | DW1000_FF_MAC_EN | DW1000_FF_ACK_EN);

	/* Activate auto-acknowledgement. Time is set to 0 so that the ACK is sent as soon as possible after reception of a frame. */
	//dw1000_enable_auto_ack(lp, 0);

    dw1000_setinterrupt(lp, DW1000_INT_TFRS | DW1000_INT_RFCG, 1);

	enable_irq(lp->spi->irq);

    return dw1000_write_16bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB);
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_stop / dwt_forcetrxoff()
 *
 * @brief This is used to turn off the transceiver
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
static void
dw1000_stop(struct ieee802154_hw *hw)
{
//	decaIrqStatus_t stat;
	u32 mask;

	struct dw1000_local *lp = hw->priv;

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Read set interrupt mask */
	dw1000_read_32bit_reg(lp, SYS_MASK_ID, 0, &mask);

	// Need to beware of interrupts occurring in the middle of following read modify write cycle
	// We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
	// event has just happened before the radio was disabled)
	// thus we need to disable interrupt during this

	disable_irq(lp->spi->irq);

//	stat = decamutexon();

	/* Clear interrupt mask - so we don't get any unwanted events */
	dw1000_write_32bit_reg(lp, SYS_MASK_ID, 0, 0);

	/* Disable the radio */
	dw1000_write_8bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, (u8)SYS_CTRL_TRXOFF);

	/* Forcing Transceiver off - so we do not want to see any new events that may have happened */
	dw1000_write_32bit_reg(lp, SYS_STATUS_ID, 0, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD));

	dw1000_sync_rx_buf_ptrs(lp);

	dw1000_write_32bit_reg(lp, SYS_MASK_ID, 0, mask); // Set interrupt mask to what it was */

	// Enable/restore interrupts again... */
//	decamutexoff(stat);
//	enable_irq(lp->spi->irq);

	lp->pdata.wait_for_resp = 0;
}

static int
dw1000_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct dw1000_local *lp = hw->priv;

	/* Select wide or narrow band */
	u8 bandwidth = ((channel == 4) || (channel == 7)) ? 1 : 0;

	BUG_ON(page != 4);

	dev_dbg(printdev(lp), "%s(%d)\n", __func__, channel);

//	dev_dbg(printdev(lp), "channel: %d\n", channel);
//	dev_dbg(printdev(lp), "channel index: %d\n", CHAN_IDX[channel]);

	// Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
	dw1000_write_32bit_reg(lp, FS_CTRL_ID, FS_PLLCFG_OFFSET, FS_PLL_CFG[CHAN_IDX[channel]]);
	dw1000_write_8bit_reg(lp, FS_CTRL_ID, FS_PLLTUNE_OFFSET, FS_PLL_TUNE[CHAN_IDX[channel]]);

	// Configure RF RX blocks (for specified channel/bandwidth)
	dw1000_write_8bit_reg(lp, RF_CONF_ID, RF_RXCTRLH_OFFSET, RX_CONFIG[bandwidth]);

	// Configure RF TX blocks (for specified channel and PRF)
	// Configure RF TX control
	dw1000_write_32bit_reg(lp, RF_CONF_ID, RF_TXCTRL_OFFSET, TX_CONFIG[CHAN_IDX[channel]]);

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
	struct dw1000_local *lp = hw->priv;

	dev_dbg(printdev(lp), "%s\n", __func__);

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		u16 addr = le16_to_cpu(filt->short_addr);
		/* Short address into low 16 bits */
		dw1000_write_16bit_reg(lp, PANADR_ID, PANADR_SHORT_ADDR_OFFSET, addr);
		lp->config.short_addr = addr;
	}

	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		u16 pan = le16_to_cpu(filt->pan_id);

		/* PAN ID is high 16 bits of register */
		dw1000_write_16bit_reg(lp, PANADR_ID, PANADR_PAN_ID_OFFSET, pan);
		lp->config.pan_id = pan;
	}

	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		u8 addr[EUI_64_LEN];

		memcpy(addr, &filt->ieee_addr, EUI_64_LEN);

		dw1000_write_reg(lp, EUI_64_ID, EUI_64_OFFSET, EUI_64_LEN, addr);
	}

	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		if (filt->pan_coord) {
			dw1000_enable_frame_filter(lp, DW1000_FF_DATA_EN | DW1000_FF_BEACON_EN | DW1000_FF_MAC_EN | DW1000_FF_ACK_EN | DW1000_FF_COORD_EN);
		} else {
			dw1000_enable_frame_filter(lp, DW1000_FF_DATA_EN | DW1000_FF_BEACON_EN | DW1000_FF_MAC_EN | DW1000_FF_ACK_EN);
		}
	}

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
dw1000_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	struct dw1000_local *lp = hw->priv;
	int ret;
	dev_dbg(printdev(lp), "%s(%d)\n", __func__, on);

	if(on) {
		ret = dw1000_write_16bit_reg(lp, PANADR_ID, PANADR_SHORT_ADDR_OFFSET, 0xffff);
		if (ret < 0)
			return ret;
		ret = dw1000_write_16bit_reg(lp, PANADR_ID, PANADR_PAN_ID_OFFSET, 0xffff);
		if (ret < 0)
			return ret;
		// TODO: Disable all frame filter
	} else {
		ret = dw1000_write_16bit_reg(lp, PANADR_ID, PANADR_SHORT_ADDR_OFFSET, lp->config.short_addr);
		if (ret < 0)
			return ret;
		ret = dw1000_write_16bit_reg(lp, PANADR_ID, PANADR_PAN_ID_OFFSET, lp->config.pan_id);
		if (ret < 0)
			return ret;
	}

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
	.set_promiscuous_mode = dw1000_set_promiscuous_mode,
};

static void
dw1000_setup_reg_messages(struct dw1000_local *lp)
{
	spi_message_init(&lp->reg_msg);
	lp->reg_msg.context = lp;

	lp->reg_addr_xfer.tx_buf = lp->reg_addr;

//	lp->reg_val_xfer.tx_buf = lp->reg_val;
//	lp->reg_val_xfer.rx_buf = lp->reg_val;

	spi_message_add_tail(&lp->reg_addr_xfer, &lp->reg_msg);
	spi_message_add_tail(&lp->reg_val_xfer, &lp->reg_msg);
}

static void
dw1000_setup_async_read_messages(struct dw1000_local *lp) {
	spi_message_init(&lp->async_read_msg);
	lp->async_read_msg.context = lp;

	lp->async_read_header_xfer.tx_buf = lp->async_read_header;

//	lp->async_read_data_xfer.tx_buf = lp->reg_val;
//	lp->async_read_data_xfer.rx_buf = lp->irq_reg_data;

	spi_message_add_tail(&lp->async_read_header_xfer, &lp->async_read_msg);
	spi_message_add_tail(&lp->async_read_data_xfer, &lp->async_read_msg);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_disable_sequencing()
 *
 * @brief This function disables the TX blocks sequencing and go to state "INIT",
 * it disables PMSC control of RF blocks, system clock is also set to XTAL
 *
 * input parameters none
 *
 * output parameters none
 *
 * no return value
 */
static void
_dw1000_disable_sequencing(struct dw1000_local *lp)
{

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Set system clock to XTI */
	_dw1000_enable_clocks(lp, FORCE_SYS_XTI);

	/* Disable PMSC ctrl of RF and RX clk blocks */
	dw1000_write_16bit_reg(lp, PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_config_lde / _dwt_configlde()
 *
 * @brief configure LDE algorithm parameters
 *
 * input parameters
 * @param prf   -   this is the PRF index (0 or 1) 0 corresponds to 16 and 1 to 64 PRF
 *
 * output parameters
 *
 * no return value
 */
void _dw1000_config_lde(struct dw1000_local *lp, int prf_index) {

	dev_dbg(printdev(lp), "%s\n", __func__);

	/* 8-bit configuration register */
	dw1000_write_8bit_reg(lp, LDE_IF_ID, LDE_CFG1_OFFSET, LDE_PARAM1);

	if (prf_index) {
		dw1000_write_16bit_reg(lp, LDE_IF_ID, LDE_CFG2_OFFSET, (u16)LDE_PARAM3_64); // 16-bit LDE configuration tuning register
	} else {
		dw1000_write_16bit_reg(lp, LDE_IF_ID, LDE_CFG2_OFFSET, (u16)LDE_PARAM3_16);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_upload_aon_config()
 *
 * @brief This function uploads always on (AON) configuration, as set in the AON_CFG0_OFFSET register.
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dw1000_upload_aon_config(struct dw1000_local *lp)
{

	dev_dbg(printdev(lp), "%s\n", __func__);

	dw1000_write_8bit_reg(lp, AON_ID, AON_CTRL_OFFSET, AON_CTRL_UPL_CFG);
	/* Clear the register */
	dw1000_write_8bit_reg(lp, AON_ID, AON_CTRL_OFFSET, 0x00);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_upload_aon()
 *
 * @brief This function uploads always on (AON) data array and configuration. Thus if this function is used, then _dwt_aonconfigupload
 * is not necessary. The DW1000 will go so SLEEP straight after this if the DWT_SLP_EN has been set.
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dw1000_upload_aon_array(struct dw1000_local *lp)
{
	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Clear the register */
	dw1000_write_8bit_reg(lp, AON_ID, AON_CTRL_OFFSET, 0x00);
	dw1000_write_8bit_reg(lp, AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_soft_reset()
 *
 * @brief this function resets the DW1000
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void
dw1000_soft_reset(struct dw1000_local *lp)
{

	dev_dbg(printdev(lp), "%s\n", __func__);

	_dw1000_disable_sequencing(lp);

	/* Clear any AON auto download bits (as reset will trigger AON download) */
	dw1000_write_16bit_reg(lp, AON_ID, AON_WCFG_OFFSET, 0x00);

	/* Clear the wake-up configuration */
	dw1000_write_8bit_reg(lp, AON_ID, AON_CFG0_OFFSET, 0x00);

	/* Upload the new configuration */
	_dw1000_upload_aon_array(lp);

	/* Reset HIF, TX, RX and PMSC */
	dw1000_write_8bit_reg(lp, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_ALL);

	/* DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
	 * Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier
	 */
	usleep_range(16, 20);

	/* Clear reset */
	dw1000_write_8bit_reg(lp, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_xtal_trim()
 *
 * @brief This is used to adjust the crystal frequency
 *
 * input parameters:
 * @param   value - crystal trim value (in range 0x0 to 0x1F) 31 steps (~1.5ppm per step)
 *
 * output parameters
 *
 * no return value
 */
void dw1000_set_xtal_trim(struct dw1000_local *lp, u8 value)
{
	// The 3 MSb in this 8-bit register must be kept to 0b011 to avoid any malfunction.
	u8 reg_val = (3 << 5) | (value & FS_XTALT_MASK);

	dev_dbg(printdev(lp), "%s\n", __func__);

	dw1000_write_8bit_reg(lp, FS_CTRL_ID, FS_XTALT_OFFSET, reg_val);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_loaducodefromrom()
 *
 * @brief  load ucode from OTP MEMORY or ROM
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dw1000_load_ucode_from_rom(struct dw1000_local *lp)
{
	dev_dbg(printdev(lp), "%s\n", __func__);

	/* Set up clocks */
	_dw1000_enable_clocks(lp, FORCE_LDE);

	/* Kick off the LDE load */
	/* Set load LDE kick bit */
	dw1000_write_16bit_reg(lp, OTP_IF_ID, OTP_CTRL, OTP_CTRL_LDELOAD);

	msleep(1); // Allow time for code to upload (should take up to 120 us) */

	/* Default clocks (ENABLE_ALL_SEQ) */
	/* Enable clocks for sequencing */
	_dw1000_enable_clocks(lp, ENABLE_ALL_SEQ);
}

//#ifdef CONFIG_IEEE802154_DW1000_DEBUGFS
static struct dentry *dw1000_debugfs_root;

static int dw1000_reg_show(struct seq_file *file, void *offset) {
	struct dw1000_local *lp = file->private;
	u32 sys_status;
	u32 sys_cfg;
	u32 sys_ctrl;
	u32 sys_mask;
	u32 gpio_ctrl;

	dw1000_read_32bit_reg(lp, SYS_STATUS_ID, 0, &sys_status);
	dw1000_read_32bit_reg(lp, SYS_CFG_ID, 0, &sys_cfg);
	dw1000_read_32bit_reg(lp, SYS_CTRL_ID, 0, &sys_ctrl);
	dw1000_read_32bit_reg(lp, SYS_MASK_ID, 0, &sys_mask);
	dw1000_read_32bit_reg(lp, GPIO_CTRL_ID, 0, &gpio_ctrl);

	seq_printf(file, "SYS_STATUS:\t0x%x\n", sys_status);
	seq_printf(file, "SYS_CFG:\t0x%x\n", sys_cfg);
	seq_printf(file, "SYS_CTRL:\t0x%x\n", sys_ctrl);
	seq_printf(file, "SYS_MASK:\t0x%x\n", sys_mask);
	seq_printf(file, "GPIO_CTRL:\t0x%x\n", gpio_ctrl);

//	dw1000_write_32bit_reg(lp, SYS_STATUS_ID, 0, sys_status);

//  dw1000_write_16bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB);

	return 0;
}

static int dw1000_reg_open(struct inode *inode, struct file *file) {
	return single_open(file, dw1000_reg_show, inode->i_private);
}

static const struct file_operations dw1000_reg_fops = {
	.open		= dw1000_reg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dw1000_configs_show(struct seq_file *file, void *offset)
{
	struct dw1000_local *lp = file->private;
	seq_printf(file, "Data Rate:\t\t\t%d\n", lp->config.data_rate);
	seq_printf(file, "Use Non-standard SFD:\t\t%d\n", lp->config.ns_sfd);
	seq_printf(file, "PHR Mode:\t\t\t%d\n", lp->config.phr_mode);
	seq_printf(file, "Pulse Repetition Frequency:\t%d\n", lp->config.prf);
	seq_printf(file, "RX Preamble Code:\t\t%d\n", lp->config.rx_code);
	seq_printf(file, "RX Acquisition Chunk Size:\t%d\n", lp->config.rx_pac);
	seq_printf(file, "SFD Timeout:\t\t\t%d\n", lp->config.sfd_timeout);
	seq_printf(file, "TX Preamble Code:\t\t%d\n", lp->config.tx_code);
	seq_printf(file, "TX Preamble Length:\t\t%d\n", lp->config.tx_preamble_length);
	return 0;
}

static int dw1000_configs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dw1000_configs_show, inode->i_private);
}

static const struct file_operations dw1000_configs_fops = {
	.open		= dw1000_configs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dw1000_dev_data_show(struct seq_file *file, void *offset)
{
	struct dw1000_local *lp = file->private;

	seq_printf(file, "IC Part ID:\t\t\t0x%x\n", lp->pdata.part_id);
	seq_printf(file, "IC Lot ID:\t\t\t0x%x\n", lp->pdata.lot_id);
	seq_printf(file, "Non-standard Long Frame Mode:\t%d\n", lp->pdata.long_frame);
	seq_printf(file, "OTP Revision Number:\t\t0x%x\n", lp->pdata.otprev);
	seq_printf(file, "TX_FCTRL Reg:\t\t\t0x%x\n", lp->pdata.tx_fctrl_reg);
	seq_printf(file, "Initial XTAL Trim Value:\t%d\n", lp->pdata.init_xtrim);
	seq_printf(file, "Double RX Buffer Mode:\t\t%d\n", lp->pdata.dbl_buff_on);
	seq_printf(file, "System Config Reg:\t\t0x%x\n", lp->pdata.sys_cfg_reg);
	seq_printf(file, "Sleep Mode:\t\t\t%d\n", lp->pdata.sleep_mode);
	return 0;
}

static int dw1000_dev_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, dw1000_dev_data_show, inode->i_private);
}

static const struct file_operations dw1000_dev_data_fops = {
	.open		= dw1000_dev_data_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dw1000_debugfs_init(struct dw1000_local *lp)
{
	char debugfs_dir_name[DNAME_INLINE_LEN + 1] = "dw1000-";
	struct dentry *stats;

	strncat(debugfs_dir_name, dev_name(&lp->spi->dev), DNAME_INLINE_LEN);

	dw1000_debugfs_root = debugfs_create_dir(debugfs_dir_name, NULL);
	if (!dw1000_debugfs_root)
		return -ENOMEM;

	stats = debugfs_create_file("reg", S_IRUGO,
		dw1000_debugfs_root, lp,
		&dw1000_reg_fops);
	if (!stats)
		return -ENOMEM;

	stats = debugfs_create_file("config", S_IRUGO,
		dw1000_debugfs_root, lp,
		&dw1000_configs_fops);
	if (!stats)
		return -ENOMEM;

	stats = debugfs_create_file("data", S_IRUGO,
		dw1000_debugfs_root, lp,
		&dw1000_dev_data_fops);
	if (!stats)
		return -ENOMEM;

	return 0;
}

static void dw1000_debugfs_remove(void)
{
	debugfs_remove_recursive(dw1000_debugfs_root);
}
//#else
//static int at86rf230_debugfs_init(struct at86rf230_local *lp) { return 0; }
//static void at86rf230_debugfs_remove(void) { }
//#endif

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_configure / dwt_configure()
 *
 * @brief This function provides the main API for the configuration of the
 * DW1000 and this low-level driver.  The input is a pointer to the data structure
 * of type dwt_config_t that holds all the configurable items.
 * The dwt_config_t structure shows which ones are supported
 *
 * input parameters
 * @param config    -   pointer to the configuration structure, which contains the device configuration data.
 *
 * output parameters
 *
 * no return value
 */
void dw1000_configure(struct dw1000_local *lp) {
	struct dw1000_config *config = &lp->config;
	u8 nsSfd_result  = 0;
	u8 useDWnsSFD = 0;
	u8 chan = config->chan;
	u32 regval;
    u16 reg16 = LDE_REPLICA_COEFF[config->rx_code];
	u8 prfIndex = config->prf - DW1000_PRF_16M;
	u8 bw = ((chan == 4) || (chan == 7)) ? 1 : 0; // Select wide or narrow band

	dev_dbg(printdev(lp), "%s\n", __func__);

	// For 110 kbps we need a special setup */
	if (DW1000_BR_110K == config->data_rate) {
		lp->pdata.sys_cfg_reg |= SYS_CFG_RXM110K;
        reg16 >>= 3; // lde_replicaCoeff must be divided by 8 */
	} else {
		lp->pdata.sys_cfg_reg &= (~SYS_CFG_RXM110K);
	}

	lp->pdata.long_frame = config->phr_mode;

	lp->pdata.sys_cfg_reg &= ~SYS_CFG_PHR_MODE_11;
	lp->pdata.sys_cfg_reg |= (SYS_CFG_PHR_MODE_11 & (config->phr_mode << SYS_CFG_PHR_MODE_SHFT));

    /* enable auto rx */
    lp->pdata.sys_cfg_reg |= SYS_CFG_RXAUTR;

	dw1000_write_32bit_reg(lp, SYS_CFG_ID, 0, lp->pdata.sys_cfg_reg);
    /* Set the lde_replicaCoeff */
	dw1000_write_16bit_reg(lp, LDE_IF_ID, LDE_REPC_OFFSET, reg16);

	_dw1000_config_lde(lp, prfIndex);

    /* Configure PLL2/RF PLL block CFG/TUNE (for a given channel) */
    dw1000_write_32bit_reg(lp, FS_CTRL_ID, FS_PLLCFG_OFFSET, FS_PLL_CFG[CHAN_IDX[chan]]);
    dw1000_write_8bit_reg(lp, FS_CTRL_ID, FS_PLLTUNE_OFFSET, FS_PLL_TUNE[CHAN_IDX[chan]]);

    /* Configure RF RX blocks (for specified channel/bandwidth) */
	dw1000_write_8bit_reg(lp, RF_CONF_ID, RF_RXCTRLH_OFFSET, RX_CONFIG[bw]);

    /* Configure RF TX blocks (for specified channel and PRF) */
    /* Configure RF TX control */
	dw1000_write_32bit_reg(lp, RF_CONF_ID, RF_TXCTRL_OFFSET, TX_CONFIG[CHAN_IDX[chan]]);

    /* Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings) */
    /* DTUNE0 */
    dw1000_write_16bit_reg(lp, DRX_CONF_ID, DRX_TUNE0b_OFFSET, SFD_THRESHOLD[config->data_rate][config->ns_sfd]);

    /* DTUNE1 */
    dw1000_write_16bit_reg(lp, DRX_CONF_ID, DRX_TUNE1a_OFFSET, DTUNE1[prfIndex]);

	if (config->data_rate == DW1000_BR_110K) {
		dw1000_write_16bit_reg(lp, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_110K);
	} else {
		if (config->tx_preamble_length == DW1000_PLEN_64) {
			dw1000_write_16bit_reg(lp, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_6M8_PRE64);
			dw1000_write_16bit_reg(lp, DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE64);
		} else {
			dw1000_write_16bit_reg(lp, DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_850K_6M8);
			dw1000_write_8bit_reg(lp, DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE128PLUS);
		}
	}

    /* DTUNE2 */
    dw1000_write_32bit_reg(lp, DRX_CONF_ID, DRX_TUNE2_OFFSET, DIGITAL_BB_CONFIG[prfIndex][config->rx_pac]);

    /* DTUNE3 (SFD timeout) */
	/* Don't allow 0 - SFD timeout will always be enabled */
    if (config->sfd_timeout == 0) {
        config->sfd_timeout = DW1000_SFDTOC_DEF;
    }
	dw1000_write_16bit_reg(lp, DRX_CONF_ID, DRX_SFDTOC_OFFSET, config->sfd_timeout);

    /* Configure AGC parameters  */
	dw1000_write_32bit_reg(lp, AGC_CFG_STS_ID, 0xC, agc_config.lo32);
	dw1000_write_16bit_reg(lp, AGC_CFG_STS_ID, 0x4, agc_config.target[prfIndex]);

    /* Set (non-standard) user SFD for improved performance */
	if (config->ns_sfd) {
        // Write non standard (DW) SFD length */
		dw1000_write_8bit_reg(lp, USR_SFD_ID, 0x00, dw_ns_SFD_len[config->data_rate]);
		nsSfd_result = 3;
		useDWnsSFD = 1;
	}
	regval =  (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | // Transmit Channel
		(CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | // Receive Channel
		(CHAN_CTRL_RXFPRF_MASK & (config->prf << CHAN_CTRL_RXFPRF_SHIFT)) | // RX PRF
		((CHAN_CTRL_TNSSFD | CHAN_CTRL_RNSSFD) & (nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
		(CHAN_CTRL_DWSFD & (useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | // Use DW nsSFD
		(CHAN_CTRL_TX_PCOD_MASK & (config->tx_code << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
		(CHAN_CTRL_RX_PCOD_MASK & (config->rx_code << CHAN_CTRL_RX_PCOD_SHIFT)); // RX Preamble Code

	dev_dbg(printdev(lp), "CHAN_CTRL:0x%x\n", regval);

	dw1000_write_32bit_reg(lp, CHAN_CTRL_ID, 0, regval);

    /* Set up TX Preamble Size, PRF and Data Rate  */
	lp->pdata.tx_fctrl_reg = ((config->tx_preamble_length | config->prf) << TX_FCTRL_TXPRF_SHFT) | (config->data_rate << TX_FCTRL_TXBR_SHFT);

	dev_dbg(printdev(lp), "TX_FCTRL:0x%x\n", lp->pdata.tx_fctrl_reg);

	dw1000_write_32bit_reg(lp, TX_FCTRL_ID, 0, lp->pdata.tx_fctrl_reg);

	// The SFD transmit pattern is initialised by the DW1000 upon a user TX request, but (due to an IC issue) it is not done for an auto-ACK TX. The
	// SYS_CTRL write below works around this issue, by simultaneously initiating and aborting a transmission, which correctly initialises the SFD
	// after its configuration or reconfiguration.
	// This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
	dw1000_write_8bit_reg(lp, SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF); // Request TX start and TRX off at the same time
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_set_spi_low / spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
static void
_dw1000_set_spi_low(struct spi_device *spi)
{
	dev_dbg(&spi->dev, "%s\n", __func__);

	dev_dbg(&spi->dev, "current max_speed is %d Hz\n", spi->max_speed_hz);

	spi->max_speed_hz = 3000000;

	dev_dbg(&spi->dev, "low max_speed is %d Hz\n", spi->max_speed_hz);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn _dw1000_set_spi_high / spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */

static void
_dw1000_set_spi_high(struct spi_device *spi) {
	dev_dbg(&spi->dev, "%s\n", __func__);

	dev_dbg(&spi->dev, "current max_speed is %d Hz\n", spi->max_speed_hz);

	spi->max_speed_hz = 20000000;

	dev_dbg(&spi->dev, "high max_speed is %d Hz\n", spi->max_speed_hz);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_hw_init() / dwt_initialise
 *
 * @brief This function initiates communications with the DW1000 transceiver
 * and reads its DEV_ID register (address 0x00) to verify the IC is one supported
 * by this software (e.g. DW1000 32-bit device ID value is 0xDECA0130).  Then it
 * does any initial once only device configurations needed for use and initialises
 * as necessary any static data items belonging to this low-level driver.
 *
 * NOTES:
 * 1.this function needs to be run before dwt_configuresleep, also the SPI frequency has to be < 3MHz
 * 2.it also reads and applies LDO tune and crystal trim values from OTP memory
 *
 * input parameters
 * @param config    -   specifies what configuration to load
 *                  DW1000_LOADUCODE     0x1 - load the LDE microcode from ROM - enabled accurate RX timestamp
 *                  DW1000_LOADNONE      0x0 - do not load any values from OTP memory
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
static int
dw1000_hw_init(struct dw1000_local *lp, u16 config)
{
	u16 otp_addr = 0;
	u32 ldo_tune = 0;

	dev_dbg(printdev(lp), "%s\n", __func__);

	lp->pdata.dbl_buff_on = 0; /* Double buffer mode off by default */
	lp->pdata.wait_for_resp = 0;
	lp->pdata.sleep_mode = 0;

//	lp->pdata.cbTxDone = NULL;
//	lp->pdata.cbRxOk = NULL;
//	lp->pdata.cbRxTo = NULL;
//	lp->pdata.cbRxErr = NULL;

	/* Make sure the device is completely reset before starting initialisation */
	dw1000_soft_reset(lp);

	/* NOTE: set system clock to XTI - this is necessary to make sure the values read by _dwt_otpread are reliable */
	_dw1000_enable_clocks(lp, FORCE_SYS_XTI);

	/* Configure the CPLL lock detect */
	dw1000_write_8bit_reg(lp, EXT_SYNC_ID, EC_CTRL_OFFSET, EC_CTRL_PLLLCK);

	/* Read OTP revision number */
	// Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
	// TODO: otp_addr u16 or u32
	_dw1000_read_otp(lp, XTRIM_ADDRESS, (u32*) &otp_addr);
	otp_addr = otp_addr & 0xffff;

	// OTP revision is next byte
	lp->pdata.otprev = (otp_addr >> 8) & 0xff;

	// Load LDO tune from OTP and kick it if there is a value actually programmed.
	_dw1000_read_otp(lp, LDOTUNE_ADDRESS, &ldo_tune);
	if ((ldo_tune & 0xFF) != 0) {
		dev_info(printdev(lp), "the device has been calibrated\n");
		// Kick LDO tune
		// Set load LDE kick bit
		dw1000_write_8bit_reg(lp, OTP_IF_ID, OTP_SF, OTP_SF_LDO_KICK);
		// LDO tune must be kicked at wake-up
		lp->pdata.sleep_mode |= AON_WCFG_ONW_LLDO;
	}

	// Load Part and Lot ID from OTP
	_dw1000_read_otp(lp, PARTID_ADDRESS, &lp->pdata.part_id);
	_dw1000_read_otp(lp, LOTID_ADDRESS, &lp->pdata.lot_id);

	// XTAL trim value is set in OTP for DW1000 module and EVK/TREK boards but that might not be the case in a custom design
	lp->pdata.init_xtrim = otp_addr & 0x1F;
	// A value of 0 means that the crystal has not been trimmed
	if (!lp->pdata.init_xtrim) {
		dev_info(printdev(lp), "the crystal has not been trimmed\n");
		// Set to mid-range if no calibration value inside
		lp->pdata.init_xtrim = FS_XTALT_MIDRANGE;
	}
	dev_info(printdev(lp), "the crystal has been trimmed\n");

	// Configure XTAL trim
	dw1000_set_xtal_trim(lp, lp->pdata.init_xtrim);

	// Load leading edge detect code
	if (config & DW1000_LOADUCODE) {
		_dw1000_load_ucode_from_rom(lp);
		lp->pdata.sleep_mode |= AON_WCFG_ONW_LLDE; // microcode must be loaded at wake-up
		dev_info(printdev(lp), "Microcode is loaded\n");
	} else {
		// Should disable the LDERUN enable bit in 0x36, 0x4
		u16 rega;
		dw1000_read_16bit_reg(lp, PMSC_ID, PMSC_CTRL1_OFFSET + 1, &rega);
		rega &= 0xFDFF; // Clear LDERUN bit
		dw1000_write_16bit_reg(lp, PMSC_ID, PMSC_CTRL1_OFFSET + 1, rega);
	}

	// Enable clocks for sequencing
	_dw1000_enable_clocks(lp, ENABLE_ALL_SEQ);

	// The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode.
	dw1000_write_8bit_reg(lp, AON_ID, AON_CFG1_OFFSET, 0x00);

	// Read system register / store local copy
	// Read sysconfig register
	dw1000_read_32bit_reg(lp, SYS_CFG_ID, 0, &lp->pdata.sys_cfg_reg);

	return DW1000_SUCCESS;
}

static int
dw1000_detect_device(struct dw1000_local *lp)
{
	const char *chip;
	int ret = 0;
	u32 dev_id = 0;

	struct wpan_phy *phy = lp->hw->phy;

	dev_info(printdev(lp), "%s\n", __func__);

	dw1000_read_32bit_reg(lp, RG_DEV_ID, 0, &dev_id);

	// Read and validate device ID return -1 if not recognised
	if (DW1000_DEVICE_ID != dev_id) { // MP IC ONLY (i.e. DW1000) FOR THIS CODE
		goto not_supp;
	}

	lp->hw->flags = IEEE802154_HW_TX_OMIT_CKSUM |
//			IEEE802154_HW_CSMA_PARAMS |
//			IEEE802154_HW_FRAME_RETRIES |
		IEEE802154_HW_AFILT |
			IEEE802154_HW_PROMISCUOUS;

	phy->flags = WPAN_PHY_FLAG_TXPOWER |
			     WPAN_PHY_FLAG_CCA_ED_LEVEL |
			     WPAN_PHY_FLAG_CCA_MODE;

	phy->supported.cca_modes = BIT(NL802154_CCA_ENERGY) |
		BIT(NL802154_CCA_CARRIER) | BIT(NL802154_CCA_ENERGY_CARRIER);
	phy->supported.cca_opts = BIT(NL802154_CCA_OPT_ENERGY_CARRIER_AND) |
		BIT(NL802154_CCA_OPT_ENERGY_CARRIER_OR);

	phy->cca.mode = NL802154_CCA_ENERGY;
	chip = "dw1000";
	lp->hw->flags |= IEEE802154_HW_LBT;
	phy->supported.channels[4] = 0xbe;
	phy->current_channel = 2;
	phy->current_page = 4;
	phy->symbol_duration = 25;
	phy->supported.lbt = NL802154_SUPPORTED_BOOL_BOTH;
	phy->supported.tx_powers = dw1000_powers;
	phy->supported.tx_powers_size = ARRAY_SIZE(dw1000_powers);
	phy->supported.cca_ed_levels = dw1000_ed_levels;
	phy->supported.cca_ed_levels_size = ARRAY_SIZE(dw1000_ed_levels);

	phy->cca_ed_level = lp->hw->phy->supported.cca_ed_levels[7];
	phy->transmit_power = lp->hw->phy->supported.tx_powers[0];

not_supp:
	dev_info(&lp->spi->dev, "Detected %s chip id 0x%x\n", chip, dev_id);

	return ret;
}

static int dw1000_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct dw1000_local *lp;
//	unsigned int status;
	struct gpio_desc *rstn;
	int rc;

	dev_info(&spi->dev, "%s\n", __func__);

	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ specified\n");
		return -EINVAL;
	}

	rstn = devm_gpiod_get(&spi->dev, "rstn", GPIOD_OUT_LOW);
	if (IS_ERR(rstn)) {
		rc = PTR_ERR(rstn);
		if (rc != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to get 'rstn' gpio: %d", rc);
		return rc;
	}

	/* Reset */
	gpiod_set_value_cansleep(rstn, 1);
	udelay(4);
	gpiod_set_value_cansleep(rstn, 0);
	usleep_range(120, 240);

	hw = ieee802154_alloc_hw(sizeof(*lp), &dw1000_ops);
	if (!hw)
		return -ENOMEM;

	lp = hw->priv;
	lp->hw = hw;
	lp->spi = spi;
//	lp->slp_tr = slp_tr;
	hw->parent = &spi->dev;
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	lp->buf = devm_kzalloc(&spi->dev,
				 SPI_COMMAND_BUFFER, GFP_KERNEL);
	if (!lp->buf)
		return -ENOMEM;

	dw1000_setup_reg_messages(lp);
	dw1000_setup_async_read_messages(lp);

	rc = dw1000_detect_device(lp);
	if (rc < 0)
		goto free_dev;

	spi_set_drvdata(spi, lp);

	_dw1000_set_spi_low(spi);

	rc = dw1000_hw_init(lp, DW1000_LOADUCODE);
	if (rc)
		goto free_dev;

	_dw1000_set_spi_high(spi);

	/* Configure GPIOs to show TX/RX activity */
//  dw1000_set_lna_pa_mode(lp, 1, 1);

	/* Configure LEDs management */
//  dw1000_set_leds(lp, DW1000_LEDS_ENABLE);

	rc = devm_request_irq(&spi->dev, spi->irq, dw1000_isr,
			      IRQF_TRIGGER_HIGH,
			      dev_name(&spi->dev), lp);
	if (rc)
		goto free_dev;

	/* Disable_irq by default and wait for starting hardware */
	disable_irq(spi->irq);

    /* Default mode 2 */
	lp->config.chan = 2;                            /* Channel number. */
	lp->config.prf = DW1000_PRF_64M;                /* Pulse repetition frequency. */
	lp->config.tx_preamble_length = DW1000_PLEN_128;   /* Preamble length. Used in TX only. */
	lp->config.rx_pac = DW1000_PAC8;                /* Preamble acquisition chunk size. Used in RX only. */
	lp->config.tx_code = 9;                          /* TX preamble code. Used in TX only. */
	lp->config.rx_code = 9;                          /* RX preamble code. Used in RX only. */
	lp->config.ns_sfd = 0;                           /* 0 to use standard SFD, 1 to use non-standard SFD. */
	lp->config.data_rate = DW1000_BR_6M8;           /* Data rate. */
	lp->config.phr_mode = DW1000_PHRMODE_STD;        /* PHY header mode. */
    lp->config.sfd_timeout = (129 + 8 - 8);   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */

	dw1000_configure(lp);

	/* going into sleep by default */
	//dw1000_sleep(lp);

	rc = dw1000_debugfs_init(lp);
	if (rc)
		goto free_dev;

	rc = ieee802154_register_hw(lp->hw);
	if (rc)
		goto free_debugfs;

	return rc;

free_debugfs:
	dw1000_debugfs_remove();

free_dev:
	ieee802154_free_hw(lp->hw);

	return rc;
}

static int
dw1000_remove(struct spi_device *spi)
{
	struct dw1000_local *lp = spi_get_drvdata(spi);

	// TODO: how to disbale the chip
	ieee802154_unregister_hw(lp->hw);
	ieee802154_free_hw(lp->hw);

	dw1000_debugfs_remove();
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
		.of_match_table = dw1000_of_match,
		.name	= "dw1000",
	},
	.probe      = dw1000_probe,
	.remove     = dw1000_remove,
};

module_spi_driver(dw1000_driver);

MODULE_DESCRIPTION("DW1000 IEEE 802.15.4 UWB Transceiver Driver");
MODULE_AUTHOR("Xue Liu");
MODULE_LICENSE("GPL v2");
