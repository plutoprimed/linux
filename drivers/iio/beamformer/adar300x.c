// SPDX-License-Identifier: GPL-2.0
/*
 * ADAR3000, ADAR3001, ADAR3002, ADAR3003 device driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/iio/buffer-dma.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include "adar300x.h"

#define ADAR300x_REG_SPI_CONFIG			0x00
#define ADAR300x_REG_1				0x01
#define ADAR300x_REG_CHIPTYPE			0x03
#define ADAR300x_REG_PRODUCT_ID_L		0x04
#define ADAR300x_REG_PRODUCT_ID_H		0x05
#define ADAR300x_REG_ADDRESS_PAGE		0x08
#define ADAR300x_REG_SCRATCHPAD			0x0A
#define ADAR300x_REG_SPI_REV			0x0B
#define	ADAR300x_REG_BEAM0_MAP			0x10
#define	ADAR300x_REG_BEAM1_MAP			0x11
#define	ADAR300x_REG_BEAM2_MAP			0x12
#define	ADAR300x_REG_BEAM3_MAP			0x13
#define ADAR300x_REG_BEAMFORMER_MODE		0x14
#define ADAR300x_REG_BEAMSTATE_MODE		0x15
#define ADAR300x_REG_BEAM_SLEEP			0x16
#define ADAR300x_REG_MEM_SEQPTR(x)		(0x17 + x)
#define ADAR300x_REG_MEM_SEQPTR0_START		0x17
#define ADAR300x_REG_MEM_SEQPTR0_STOP		0x18
#define ADAR300x_REG_MEM_SEQPTR1_START		0x19
#define ADAR300x_REG_MEM_SEQPTR1_STOP		0x1A
#define ADAR300x_REG_MEM_SEQPTR2_START		0x1B
#define ADAR300x_REG_MEM_SEQPTR2_STOP		0x1C
#define ADAR300x_REG_MEM_SEQPTR3_START		0x1D
#define ADAR300x_REG_MEM_SEQPTR3_STOP		0x1E
#define ADAR300x_REG_ADC_CONTROL		0x20
#define ADAR300x_REG_ADC_CONTROL2		0x21
#define ADAR300x_REG_ADC_DATA_OUT		0x22
#define ADAR300x_REG_DAC_DATA_MSB		0x23
#define ADAR300x_REG_DAC_DATA_LSB		0x24
#define ADAR300x_REG_DAC_CONTROL		0x25
#define ADAR300x_REG_PIN_OR_SPI_CTL		0x30
#define ADAR300x_REG_BEAMWISE_UPDATE_CODE	0x32
#define ADAR300x_REG_BEAMWISE_UPDATE		0x33

#define ADAR300x_REG_FIFO_POINTER(x)		(0x50 + x)
#define ADAR300x_REG_FIFO_WRITE_POINTER0	0x50
#define ADAR300x_REG_FIFO_READ_POINTER0		0x51
#define ADAR300x_REG_FIFO_WRITE_POINTER1	0x52
#define ADAR300x_REG_FIFO_READ_POINTER1		0x53
#define ADAR300x_REG_FIFO_WRITE_POINTER2	0x54
#define ADAR300x_REG_FIFO_READ_POINTER2		0x55
#define ADAR300x_REG_FIFO_WRITE_POINTER3	0x56
#define ADAR300x_REG_FIFO_READ_POINTER3		0x57

#define ADAR3002_REG_RESET(x)			(0x080 + x)
#define ADAR3002_REG_MUTE(x)			(0x0A0 + x)
#define ADAR300x_REG_AMP_BIAS(x)		(0xC0 + x)
#define ADAR3002_REG_DRCT_CNTRL(x)		(0x100 + x)

/* ADAR3002 direct control registers H/V BEAM0 to BEAM1 */
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL0H(x)	(0x100 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL0H(x)	(0x101 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL1H(x)	(0x102 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL1H(x)	(0x103 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL2H(x)	(0x104 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL2H(x)	(0x105 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL3H(x)	(0x106 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL3H(x)	(0x107 + (x) * (0x03 << 3))

#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL0V(x)	(0x108 + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL0V(x)	(0x109 + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL1V(x)	(0x10A + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL1V(x)	(0x10B + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL2V(x)	(0x10C + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL2V(x)	(0x10D + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL3V(x)	(0x10E + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL3V(x)	(0x10F + (x) * 0x08)

/* ADAR3003 direct control registers only one BEAM from EL0 to EL3 */
#define ADAR3003_REG_DRCT_CNTRL_DELAY_ELV(x)	(0x100 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_ATTN_ELV(x)	(0x101 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_DELAY_ELH(x)	(0x102 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_ATTN_ELH(x)	(0x103 + (x) * 0x08)


#define ADAR300x_REG_BM0_SEQ_PTR(x)	(0x200 + x)
#define ADAR300x_REG_BM1_SEQ_PTR(x)	(0x240 + x)
#define ADAR300x_REG_BM2_SEQ_PTR(x)	(0x280 + x)
#define ADAR300x_REG_BM3_SEQ_PTR(x)	(0x2C0 + x)

#define ADAR300x_REG_RESET_BM0_STREAM_IN(x)	(0x300 + x)
#define ADAR300x_REG_RESET_BM1_STREAM_IN(x)	(0x306 + x)
#define ADAR300x_REG_RESET_BM2_STREAM_IN(x)	(0x30C + x)
#define ADAR300x_REG_RESET_BM3_STREAM_IN(x)	(0x312 + x)

#define ADAR300x_REG_MUTE_BM0_STREAM_IN(x)	(0x318 + x)
#define ADAR300x_REG_MUTE_BM1_STREAM_IN(x)	(0x31E + x)
#define ADAR300x_REG_MUTE_BM2_STREAM_IN(x)	(0x324 + x)
#define ADAR300x_REG_MUTE_BM3_STREAM_IN(x)	(0x32A + x)

#define ADAR300x_REG_DRCT_CNTRL_BM0_STREAM_IN(x)	(0x330 + x)
#define ADAR300x_REG_DRCT_CNTRL_BM1_STREAM_IN(x)	(0x336 + x)
#define ADAR300x_REG_DRCT_CNTRL_BM2_STREAM_IN(x)	(0x33C + x)
#define ADAR300x_REG_DRCT_CNTRL_BM3_STREAM_IN(x)	(0x342 + x)

/* Beam state RAM
 * state - beam state number 0 - 63
  */
#define ADAR300x_RAM_BEAM_STATE(state)			(0x100 + (state * 6))
#define ADAR300x_RAM_MAX_ADDR				0x27F

/* Beam state FIFO load
 * beam - position in memory location 0 - 3
 */
#define ADAR300x_FIFO_LOAD(beam)			(0x100 + (beam * 0x10))

/* ADAR300x_REG_SPI_CONFIG */
#define ADAR300x_SPI_CONFIG_RESET_	BIT(7)
#define ADAR300x_SPI_CONFIG_BIG_ENDIAN_	BIT(5)
#define ADAR300x_SPI_CONFIG_SDOACTIVE_	BIT(4)
#define ADAR300x_SPI_CONFIG_SDOACTIVE	BIT(3)
#define ADAR300x_SPI_CONFIG_BIG_ENDIAN	BIT(2)
#define ADAR300x_SPI_CONFIG_RESET	BIT(0)

/* ADAR300x_REG_BEAMSTATE_MODE */
#define ADAR300x_MODE0	0x03
#define ADAR300x_MODE1	0x0C
#define ADAR300x_MODE2	0x30
#define ADAR300x_MODE3	0xC0

#define ADAR300x_ADDRESS_PAGE_MASK	0x0F
#define ADAR300x_SPI_ADDR_MSK		GENMASK(13, 10)
#define ADAR300x_SPI_ADDR(x)		FIELD_PREP(ADAR300x_SPI_ADDR_MSK, x)

#define ADAR300x_BEAMS_PER_DEVICE	4
#define ADAR300x_ELEMENTS_PER_BEAM	4
#define ADAR300x_CHANNELS_PER_BEAM	8
#define ADAR300x_PACKED_BEAMSTATE_LEN	6	/* bytes */
#define ADAR300x_UNPACKED_BEAMSTATE_LEN	8	/* bytes */
#define ADAR300x_MAX_RAM_STATES	64
#define ADAR300x_MAX_FIFO_STATES	16
#define ADAR300x_MAX_DEV		16
#define ADAR300x_MAX_RAW		0x3f
#define ADAR300x_MAX_GAIN		31	/* dB */
#define ADAR300x_MAX_PHASE		360	/* degree */

enum adar300x_ADC_sel {
	ADAR300x_ADC_ANALG0,
	ADAR300x_ADC_ANLG1,
	ADAR300x_ADC_TEMPERATURE,
};

enum adar300x_pages {
	ADAR300x_CONFIG_PAGE,
	ADAR300x_BEAM0H_PAGE,
	ADAR300x_BEAM0V_PAGE,
	ADAR300x_BEAM1H_PAGE,
	ADAR300x_BEAM1V_PAGE,
	ADAR300x_FIFO_PAGE,
};

enum adar300x_update_intf_control {
	ADAR300x_UPDATE_PIN_CONTROL,
	ADAR300x_UPDATE_SPI_CONTROL,
};

static const char *const adar300x_update_intf_ctrl[] = {
	[ADAR300x_UPDATE_PIN_CONTROL] = "pin",
	[ADAR300x_UPDATE_SPI_CONTROL] = "SPI",
};

static const char *const adar300x_mode_ctrl[] = {
	[ADAR300x_DIRECT_CTRL] = "direct",
	[ADAR300x_MEMORY_CTRL] = "memory",
	[ADAR300x_FIFO_CTRL] = "fifo",
	[ADAR300x_INST_DIRECT_CTRL] = "instant_direct",
	[ADAR300x_MUTE] = "mute",
	[ADAR300x_RESET] = "reset"
};

static const struct regmap_config adar300x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int adar300x_reg_read(struct adar300x_state *st, u32 reg, u32 *val)
{
	return regmap_read(st->regmap, st->dev_addr | reg, val);
}

static int adar300x_reg_write(struct adar300x_state *st, u32 reg, u32 val)
{
	return regmap_write(st->regmap, st->dev_addr | reg, val);
}

static int adar300x_reg_update(struct adar300x_state *st, u32 reg, u32 mask, u32 val)
{
	int ret;
	u32 readval;

	ret = adar300x_reg_read(st, reg, &readval);
	if (ret < 0)
		return ret;

	readval &= ~mask;
	readval |= (val & mask);

	return adar300x_reg_write(st, reg, readval);
}

static int adar300x_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval, u32 *readval)
{
	struct adar300x_state *st = iio_priv(indio_dev);

	if (readval)
		return adar300x_reg_read(st, reg, readval);
	else
		return adar300x_reg_write(st, reg, writeval);
}

static int adar300x_adc_read(struct adar300x_state *st, u32 *value)
{
	u32 val;
	int ret;

	ret = adar300x_reg_read(st, ADAR300x_REG_ADC_CONTROL2, &val);
	if (ret < 0)
		return ret;
#ifdef DEBUG_ADAR300x
	pr_err("-----adar300x_adc_read0:%x", val);
#endif // DEBUG_ADAR300x
	val = (~val) & 0x01;
#ifdef DEBUG_ADAR300x
	pr_err("adar300x_adc_read1:%x", val);
#endif // DEBUG_ADAR300x
	ret = adar300x_reg_update(st, ADAR300x_REG_ADC_CONTROL2, 0x01, val);
	if (ret < 0)
		return ret;

	do {
		ret = adar300x_reg_read(st, ADAR300x_REG_ADC_CONTROL2, &val);
		if (ret < 0)
			return ret;
#ifdef DEBUG_ADAR300x
		pr_err("adar300x_adc_read2:%x", val);
#endif // DEBUG_ADAR300x
		val = (0x10 & val) ? 0 : 1;
#ifdef DEBUG_ADAR300x
		pr_err("adar300x_adc_read3:%x", val);
#endif // DEBUG_ADAR300x
	} while(val);

	return adar300x_reg_read(st, ADAR300x_REG_ADC_DATA_OUT, value);
}

static int adar300x_adc_setup(struct adar300x_state *st, enum adar300x_ADC_sel sel)
{
	int ret;

	ret = adar300x_reg_update(st, ADAR300x_REG_ADC_CONTROL, 0x80, 0x80);
	if (ret < 0)
		return ret;

	ret = adar300x_reg_update(st, ADAR300x_REG_ADC_CONTROL, 0x80, 0x00);
	if (ret < 0)
		return ret;

	return adar300x_reg_update(st, ADAR300x_REG_ADC_CONTROL, 0x03, sel);
}

static int adar300x_set_page(struct adar300x_state *st, enum adar300x_pages page)
{
	return adar300x_reg_update(st, ADAR300x_REG_ADDRESS_PAGE, ADAR300x_ADDRESS_PAGE_MASK, page);
}

static int adar300x_ram_read(struct adar300x_state *st, enum adar300x_pages page, u16 addr, char *data)
{
	u32 val;
	int ret;

	ret = adar300x_set_page(st, page);
	if (ret < 0)
		return ret;

	ret = adar300x_reg_read(st, addr, &val);
	if (ret < 0)
		return ret;

	/* Second read for valid data */
	ret = adar300x_reg_read(st, addr, &val);
	if (ret < 0)
		return ret;

	*data = val;

	return 0;
}

static enum adar300x_beamstate_mode_ctrl adar300x_get_bem(struct adar300x_state *st, struct iio_chan_spec const *chan)
{
	if (st->chip_info->chip_id == ID_ADAR3003)
		return chan->address / ADAR300x_ELEMENTS_PER_BEAM;
	else
		return chan->address / (ADAR300x_ELEMENTS_PER_BEAM * 2);
}

static void adar300x_unpack_data(char *packed, char *unpacked, int unpacked_len)
{
	int i, j;

	for(i = 0, j = 0; i < unpacked_len;) {
		unpacked[i + 0]= (packed[j + 0] >> 2) & ADAR300x_MAX_RAW;
		unpacked[i + 1]= ((packed[j + 0] << 4) | (packed[j + 1] >> 4)) & ADAR300x_MAX_RAW;
		unpacked[i + 2]= ((packed[j + 1] << 2) | (packed[j + 2] >> 6)) & ADAR300x_MAX_RAW;
		unpacked[i + 3]= (packed[j + 2]) & ADAR300x_MAX_RAW;

		unpacked[i + 4]= (packed[j + 3] >> 2) & ADAR300x_MAX_RAW;
		unpacked[i + 5]= ((packed[j + 3] << 4) | (packed[j + 4] >> 4)) & ADAR300x_MAX_RAW;
		unpacked[i + 6]= ((packed[j + 4] << 2) | (packed[j + 5] >> 6)) & ADAR300x_MAX_RAW;
		unpacked[i + 7]= (packed[j + 5]) & ADAR300x_MAX_RAW;

		i = i + ADAR300x_UNPACKED_BEAMSTATE_LEN;
		j = j + ADAR300x_PACKED_BEAMSTATE_LEN;
	}
}

static void adar300x_pack_data(char *packed, const char *unpacked, int unpacked_len)
{
	int i, j;

	for(i = 0, j = 0; i < unpacked_len;) {
		packed[j + 0] = unpacked[i + 0] << 2 | unpacked[i + 1] >> 4;
		packed[j + 1] = unpacked[i + 1] << 4 | unpacked[i + 2] >> 2;
		packed[j + 2] = unpacked[i + 2] << 6 | unpacked[i + 3];
		packed[j + 3] = unpacked[i + 4] << 2 | unpacked[i + 5] >> 4;
		packed[j + 4] = unpacked[i + 5] << 4 | unpacked[i + 6] >> 2;
		packed[j + 5] = unpacked[i + 6] << 6 | unpacked[i + 7];

		i = i + ADAR300x_UNPACKED_BEAMSTATE_LEN;
		j = j + ADAR300x_PACKED_BEAMSTATE_LEN;
	}
}

static int adar300x_set_fifo_value(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, u32 val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	enum adar300x_beamstate_mode_ctrl beam;
	char packed[ADAR300x_PACKED_BEAMSTATE_LEN];
	int ret, i;
	u16 addr, element;

	beam = adar300x_get_bem(st, chan);
	element = chan->address % st->chip_info->unpacked_beamst_len;
	st->state_buf[beam][element] = val;

	// if (st->chip_info->unpacked_beamst_len != chan->address)
	// 	return 0;

	adar300x_pack_data(packed, st->state_buf[beam], ADAR300x_UNPACKED_BEAMSTATE_LEN);
	addr = ADAR300x_FIFO_LOAD(beam);
#ifdef DEBUG_ADAR300x
	pr_err("adar300x_set_fifo_value chan->address: %ld, beam %d, addr %x, element %d, val %d\n", chan->address, beam, addr, element, val);
#endif // DEBUG_ADAR300x
	if (element != (ADAR300x_UNPACKED_BEAMSTATE_LEN - 1))
		return 0;

	pr_err("adar300x_set_fifo_value save");
	ret = adar300x_set_page(st, ADAR300x_FIFO_PAGE);
	if (ret < 0)
		return ret;

	for (i = 0; i < ADAR300x_PACKED_BEAMSTATE_LEN; i++) {
		ret = adar300x_reg_write(st, addr + i, packed[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int adar300x_get_fifo_value(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, u32 *val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	enum adar300x_beamstate_mode_ctrl beam;
	u8 element;

	beam = adar300x_get_bem(st, chan);
	element = chan->address % st->chip_info->unpacked_beamst_len;
	*val = st->state_buf[beam][element];
#ifdef DEBUG_ADAR300x
	pr_err("adar300x_get_fifo_value chan->address: %ld, beam %d, element %d, val %d\n", chan->address, beam, element, *val);
#endif // DEBUG_ADAR300x
	return 0;
}

static int adar300x_set_mem_value(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, u32 val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	enum adar300x_beamstate_mode_ctrl beam;
	char packed[ADAR300x_PACKED_BEAMSTATE_LEN];
	char unpacked[ADAR300x_UNPACKED_BEAMSTATE_LEN];
	u16 addr;
	u8 state;
	int ret, i;

	beam = adar300x_get_bem(st, chan);
	state = st->beam_index[beam];
	addr = ADAR300x_RAM_BEAM_STATE(state);

	for (i = 0; i < ADAR300x_PACKED_BEAMSTATE_LEN; i++) {
		ret = adar300x_ram_read(st, ADAR300x_BEAM0H_PAGE + beam, addr + i, &packed[i]);
		if (ret < 0)
			return ret;
	}

	adar300x_unpack_data(packed, unpacked, ADAR300x_UNPACKED_BEAMSTATE_LEN);

	unpacked[chan->address] = val;

	adar300x_pack_data(packed, unpacked, ADAR300x_UNPACKED_BEAMSTATE_LEN);

	for (i = 0; i < ADAR300x_PACKED_BEAMSTATE_LEN; i++) {
		ret = adar300x_reg_write(st, addr + i, packed[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int adar300x_get_mem_value(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, u32 *val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	enum adar300x_beamstate_mode_ctrl beam;
	char packed[ADAR300x_PACKED_BEAMSTATE_LEN];
	char unpacked[ADAR300x_UNPACKED_BEAMSTATE_LEN];
	u16 addr;
	u8 state;
	int ret, i;

	beam = adar300x_get_bem(st, chan);
	state = st->beam_index[beam];
	addr = ADAR300x_RAM_BEAM_STATE(state);

	for (i = 0; i < ADAR300x_PACKED_BEAMSTATE_LEN; i++) {
		ret = adar300x_ram_read(st, ADAR300x_BEAM0H_PAGE + beam, addr + i, &packed[i]);
		if (ret < 0)
			return ret;
	}

	adar300x_unpack_data(packed, unpacked, ADAR300x_UNPACKED_BEAMSTATE_LEN);

	*val = unpacked[chan->address];

	return 0;
}

static int adar300x_get_mode_address(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    u16 *address)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	enum adar300x_beamstate_mode_ctrl beam;
	u8 chan_addr;

	beam = adar300x_get_bem(st, chan);
	if (st->chip_info->chip_id == ID_ADAR3003)
		chan_addr = chan->address + (4 * (chan->address / 4));
	else
		chan_addr = chan->address;


	switch(st->beam_load_mode[beam]) {
	case ADAR300x_DIRECT_CTRL:
	case ADAR300x_INST_DIRECT_CTRL:
		*address = ADAR3002_REG_DRCT_CNTRL(chan_addr);
		return 0;
	case ADAR300x_RESET:
		*address = ADAR3002_REG_RESET(chan_addr);
		return 0;
	case ADAR300x_MUTE:
		*address = ADAR3002_REG_MUTE(chan_addr);
		return 0;
	default:
		return -EINVAL;
	}
}

static int adar300x_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long m)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	enum adar300x_beamstate_mode_ctrl beam;
	u16 address;
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		switch(chan->type) {
			case IIO_POWER:
			case IIO_PHASE:
				beam = adar300x_get_bem(st, chan);
				if (st->beam_load_mode[beam] == ADAR300x_MEMORY_CTRL) {
					ret = adar300x_get_mem_value(indio_dev, chan, val);
					if (ret)
						return ret;
				} else if (st->beam_load_mode[beam] == ADAR300x_FIFO_CTRL) {
					ret = adar300x_get_fifo_value(indio_dev, chan, val);
					if (ret)
						return ret;
				} else {
					ret = adar300x_get_mode_address(indio_dev, chan, &address);
					if (ret)
						return ret;

					ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
					if (ret)
						return ret;

					ret = adar300x_reg_read(st, address, val);
					if (ret)
						return ret;
				}
#ifdef DEBUG_ADAR300x
		pr_err("adar300x_read_raw channel: %lud, address:%x, value: %d, ret: %d\n", chan->address, address, *val, ret);
#endif // DEBUG_ADAR300x
				return IIO_VAL_INT;
			case IIO_TEMP:
				ret = adar300x_adc_read(st, val);
				if (ret)
					return ret;

				return IIO_VAL_INT;
			break;
			default:
				return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch(chan->type) {
		case IIO_POWER:
			*val = ADAR300x_MAX_GAIN;
			*val2 = ADAR300x_MAX_RAW;

			return IIO_VAL_FRACTIONAL;
		case IIO_PHASE:
			*val = ADAR300x_MAX_PHASE;
			*val2 = ADAR300x_MAX_RAW;

			return IIO_VAL_FRACTIONAL;
		case IIO_TEMP:
			*val = 0;
			*val2 = 911500;

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch(chan->type) {
		case IIO_TEMP:
			*val = 86;
			*val2 = 362000;

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
};

static int adar300x_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	enum adar300x_beamstate_mode_ctrl beam;
	u16 address;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val > ADAR300x_MAX_RAW)
			return -EINVAL;
		beam = adar300x_get_bem(st, chan);
		if (st->beam_load_mode[beam] == ADAR300x_MEMORY_CTRL) {
			ret = adar300x_set_mem_value(indio_dev, chan, val);
			if (ret)
				return ret;
		} else if (st->beam_load_mode[beam] == ADAR300x_FIFO_CTRL) {
			ret = adar300x_set_fifo_value(indio_dev, chan, val);
			if (ret)
				return ret;
		} else {
			ret = adar300x_get_mode_address(indio_dev, chan, &address);
			if (ret)
				return ret;

			ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
			if (ret)
				return ret;

			ret = adar300x_reg_write(st, address, val);
		}
#ifdef DEBUG_ADAR300x
		pr_err("adar300x_write_raw channel: %lud, address:%x, value: %d, ret:%d\n", chan->address, address, val, ret);
#endif // DEBUG_ADAR300x
		if (ret < 0)
			return ret;

		return 0;
	default:
		return -EINVAL;
	};
}

ssize_t adar300x_amp_en_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, ch;
	u8 readval;

	ret = kstrtou8(buf, 10, &readval);
	if (ret < 0)
		return ret;

	ch = (u32)this_attr->address;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	ret = adar300x_reg_update(st, ADAR300x_REG_AMP_BIAS(ch), BIT(3), (readval << 3));
	if (ret < 0)
		return ret;
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "adar300x_amp_bias_store ch:%d %x", ch, readval);
#endif // DEBUG_ADAR300x
	return ret ? ret : len;
}

ssize_t adar300x_amp_en_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, ch;
	u32 readval;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	ch = (u32)this_attr->address;

	ret = adar300x_reg_read(st, ADAR300x_REG_AMP_BIAS(ch), &readval);
	if (ret < 0)
		return ret;
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "adar300x_amp_en_show");
#endif // DEBUG_ADAR300x
	return sprintf(buf, "%d\n", ((readval >> 3) & 0x01));
}

ssize_t adar300x_amp_bias_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, ch;
	u8 readval;

	ret = kstrtou8(buf, 10, &readval);
	if (ret < 0)
		return ret;

	ch = (u32)this_attr->address;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	ret = adar300x_reg_update(st, ADAR300x_REG_AMP_BIAS(ch), 0x07, (readval & 0x07));
	if (ret < 0)
		return ret;
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "adar300x_amp_bias_store ch:%d %x", ch, readval);
#endif // DEBUG_ADAR300x

	return ret ? ret : len;
}

ssize_t adar300x_amp_bias_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, ch;
	u32 readval;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	ch = (u32)this_attr->address;

	ret = adar300x_reg_read(st, ADAR300x_REG_AMP_BIAS(ch), &readval);
	if (ret < 0)
		return ret;
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "adar300x_amp_bias_show");
#endif // DEBUG_ADAR300x

	return sprintf(buf, "%d\n", (readval & 0x07));
}

ssize_t adar300x_update_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, beam;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	beam = (u32)this_attr->address;
	if (beam > 3)
		return -EINVAL;

	ret = adar300x_reg_write(st, ADAR300x_REG_BEAMWISE_UPDATE, BIT(beam));
	if (ret < 0)
		return ret;
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "adar300x_update beam:%d", beam);
#endif // DEBUG_ADAR300x
	return ret ? ret : len;
}

ssize_t adar300x_update_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, beam;
	u32 readval;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	beam = (u32)this_attr->address;
	if (beam > 3)
		return -EINVAL;


	ret = adar300x_reg_read(st, ADAR300x_REG_BEAMWISE_UPDATE, &readval);
	if (ret < 0)
		return ret;
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "adar300x_update_show");
#endif // DEBUG_ADAR300x
	return sprintf(buf, "%d\n", readval);
}

ssize_t adar300x_ram_index_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u8 readval;
	int ret = 0, beam;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	beam = this_attr->address;
	ret = kstrtou8(buf, 10, &readval);
	if (readval > (ADAR300x_MAX_RAM_STATES - 1))
		return -EINVAL;

	st->beam_index[beam] = readval;

	return len;
}

ssize_t adar300x_ram_index_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u32 readval;

	readval = st->beam_index[this_attr->address];

	return sprintf(buf, "%d\n", readval);
}

ssize_t adar300x_ram_range_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u8 readval;
	int ret = 0, beam;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	beam = this_attr->address;
	ret = kstrtou8(buf, 10, &readval);
	if (readval > (ADAR300x_MAX_RAM_STATES - 1))
		return -EINVAL;

	ret = adar300x_reg_write(st, ADAR300x_REG_MEM_SEQPTR(beam), readval);
	if (ret <0)
		return ret;
	
	return len;
}

ssize_t adar300x_ram_range_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	int ret = 0;
	u16 beam;
	u32 readval;
	beam = this_attr->address;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	ret = adar300x_reg_read(st, ADAR300x_REG_MEM_SEQPTR0_START + beam, &readval);
	if (ret <0)
		return ret;

	return sprintf(buf, "%d\n", readval);
}

ssize_t adar300x_fifo_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	int ret = 0;
	u16 fifo_attr, fifo_ptr;
	u32 readval;

	fifo_attr = this_attr->address;

	fifo_ptr = ADAR300x_REG_FIFO_POINTER(fifo_attr);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	ret = adar300x_reg_read(st, fifo_ptr, &readval);
	if (ret < 0)
		return ret;
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "fifo_ptr%x: %x", fifo_ptr, readval);
#endif // DEBUG_ADAR300x
	return sprintf(buf, "%d\n", readval);
}

ssize_t adar300x_show_update_intf_ctrl_available(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(adar300x_update_intf_ctrl); ++i) {
		if (adar300x_update_intf_ctrl[i])
			len += sprintf(buf + len, "%s ", adar300x_update_intf_ctrl[i]);
	}

	return len;
}

ssize_t adar300x_update_intf_ctrl_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	unsigned int mode = 0, i;
	int ret;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(adar300x_update_intf_ctrl); ++i) {
		if (adar300x_update_intf_ctrl[i] && sysfs_streq(buf, adar300x_update_intf_ctrl[i])) {
			mode = i;
			break;
		}
	}
	
	ret = adar300x_reg_write(st, ADAR300x_REG_PIN_OR_SPI_CTL, mode);
	if (ret < 0)
		return ret;

	return len;
}

ssize_t adar300x_update_intf_ctrl_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	u32 readval;
	int ret;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;
	
	ret = adar300x_reg_read(st, ADAR300x_REG_PIN_OR_SPI_CTL, &readval);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%s\n", adar300x_update_intf_ctrl[readval]);
}

ssize_t adar300x_load_mode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	unsigned int mode = 0, i;

	for (i = 0; i < ARRAY_SIZE(adar300x_mode_ctrl); ++i) {
		if (adar300x_mode_ctrl[i] && sysfs_streq(buf, adar300x_mode_ctrl[i])) {
			mode = i;
			break;
		}
	}

	st->beam_load_mode[(u32)this_attr->address] = mode;

	return len;
}

ssize_t adar300x_load_mode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u32 readval;

	readval = st->beam_load_mode[(u32)this_attr->address];

	return sprintf(buf, "%s\n", adar300x_mode_ctrl[readval]);
}

ssize_t adar300x_show_mode_available(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(adar300x_mode_ctrl); ++i) {
		if (adar300x_mode_ctrl[i])
			len += sprintf(buf + len, "%s ", adar300x_mode_ctrl[i]);
	}

	return len;
}

ssize_t adar300x_mode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	unsigned int mode = 0, i;
	int ret = 0, ch, beam, beam_mask;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(adar300x_mode_ctrl); ++i) {
		if (adar300x_mode_ctrl[i] && sysfs_streq(buf, adar300x_mode_ctrl[i])) {
			mode = i;
			break;
		}
	}

	ch = this_attr->address;
	beam_mask = ADAR300x_MODE0 << (ch * 2);
	if (mode <= ADAR300x_INST_DIRECT_CTRL) {
#ifdef DEBUG_ADAR300x
		dev_err(indio_dev->dev.parent, "adar300x_mode_store ch: %d,  mode %d", ch, mode);
#endif // DEBUG_ADAR300x
		beam = 0;
		ret = adar300x_reg_update(st, ADAR300x_REG_BEAMSTATE_MODE,
					  beam_mask, mode << (2 * ch));
		if (ret < 0)
			return ret;
	}
	else {
		/* In Instantaneous Direct Control, the ADAR3002 cannot go into Reset or
		Mute beamstates*/
		if (st->beam_mode[ch] == ADAR300x_INST_DIRECT_CTRL)
			return -EINVAL;

		beam = BIT(ch * 2) << (mode == ADAR300x_MUTE);
	}

	ret = adar300x_reg_update(st, ADAR300x_REG_BEAMWISE_UPDATE_CODE, beam_mask, beam);
	if (ret < 0)
		return ret;

	st->beam_mode[ch] = mode;

	return len;
}

ssize_t adar300x_mode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u16 ch;

	ch = this_attr->address;
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "adar300x_mode_show ch: %d,  mode %d", ch, st->beam_mode[ch]);
#endif // DEBUG_ADAR300x
	return sprintf(buf, "%s\n", adar300x_mode_ctrl[st->beam_mode[ch]]);
}

static const unsigned long adar300x_available_scan_masks[] = {
	0x000000FF, 0x0000FF00, 0x0000FFFF, 0x00FF0000,
	0x00FF00FF, 0x00FFFF00, 0x00FFFFFF, 0xFF000000,
	0xFF0000FF, 0xFF00FF00, 0xFF00FFFF, 0xFFFF0000,
	0xFFFF00FF, 0xFFFFFF00, 0xFFFFFFFF, 0x00000000,
};

// static const struct adar300x_chip_info adar300x_chip_info_tbl[] = {
// 	[ID_ADAR3000] = {
// 		.chip_id = ID_ADAR3000,
// 		.channels = adar300x_channels,
// 		.num_channels = ARRAY_SIZE(adar300x_channels),
// 	},
// 	[ID_ADAR3002] = {
// 		.chip_id = ID_ADAR3002,
// 		.channels = adar3002_channels,
// 		.num_channels = ARRAY_SIZE(adar3002_channels),
// 	},
// 	[ID_ADAR3003] = {
// 		.chip_id = ID_ADAR3003,
// 		.channels = adar3003_channels,
// 		.num_channels = ARRAY_SIZE(adar3003_channels),
// 	},
// };

static int adar300x_setup(struct iio_dev *indio_dev)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	u32 val = 0;

	int ret;

	/* Software reset and activate SDO */
	ret = adar300x_reg_write(st, ADAR300x_REG_SPI_CONFIG,
				 ADAR300x_SPI_CONFIG_RESET_ |
				 ADAR300x_SPI_CONFIG_SDOACTIVE_ |
				 ADAR300x_SPI_CONFIG_SDOACTIVE |
				 ADAR300x_SPI_CONFIG_RESET);
	if (ret < 0)
	       return ret;

	ret = adar300x_reg_write(st, ADAR300x_REG_SCRATCHPAD, 0xAD);
	if (ret < 0)
		return ret;

	ret = adar300x_reg_read(st, ADAR300x_REG_SCRATCHPAD, &val);
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "ADAR300x_REG_SCRATCHPAD: %x", val);
#endif // DEBUG_ADAR300x
	if (ret < 0)
	       return ret;

	if (val != 0xAD) {
		dev_err(indio_dev->dev.parent, "Failed to read/write scratchpad");
		return -EIO;
	}

	ret = adar300x_reg_write(st, ADAR300x_REG_SCRATCHPAD, 0xEA);
	if (ret < 0)
		return ret;

	ret = adar300x_reg_read(st, ADAR300x_REG_SCRATCHPAD, &val);
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "ADAR300x_REG_SCRATCHPAD: %x", val);
#endif // DEBUG_ADAR300x
	if (ret < 0)
	       return ret;

	if (val != 0xEA) {
		dev_err(indio_dev->dev.parent, "Failed to read/write scratchpad");
		return -EIO;
	}

	ret = adar300x_reg_read(st, ADAR300x_REG_CHIPTYPE, &val);
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "ADAR300x_REG_CHIPTYPE: %x", val);
#endif // DEBUG_ADAR300x
	if (ret < 0)
	       return ret;
	
	if (val != 0x01) {
		dev_err(indio_dev->dev.parent, "Failed to read ADAR300x_REG_CHIPTYPE %x", val);
		return -EIO;
	}

	ret = adar300x_reg_read(st, ADAR300x_REG_PRODUCT_ID_L, &val);
#ifdef DEBUG_ADAR300x
	dev_err(indio_dev->dev.parent, "ADAR300x_REG_CHIPTYPE: %x", val);
#endif // DEBUG_ADAR300x
	if (ret < 0)
	       return ret;

	if (val != 0x01) {
		dev_err(indio_dev->dev.parent, "Failed to read PRODUCT_ID_L %x", val);
		return -EIO;
	}
#ifdef DEBUG_ADAR300x
	pr_info("%s: scratchpad val: %d\n", __func__, val);
	pr_info("%s: done\n", __func__);
#endif // DEBUG_ADAR300x

	ret = adar300x_adc_setup(st, ADAR300x_ADC_TEMPERATURE);
	if (ret < 0)
	       return ret;

	return 0;
}

static bool adar300x_is_beam_active(u8 beam, u32 mask)
{
	/* Only one channel in beam needs to be tested since we provide a mask table: 
	adar300x_available_scan_masks */
	return (mask & (1 << (ADAR300x_CHANNELS_PER_BEAM * beam)));
}

static int adar300x_ram_write(struct adar300x_state *dac, struct iio_dma_buffer_block *block, u32 mask)
{
	char *data = block->vaddr;
	char packed[ADAR300x_PACKED_BEAMSTATE_LEN];
	int ret;
	int i, j, beam, ram_beam_state;
	u16 addr, unp_bst_len, p_bst_len;

	unp_bst_len = dac->chip_info->unpacked_beamst_len;
	p_bst_len = dac->chip_info->packed_beamst_len;

	for (i = 0; i < block->block.bytes_used; i++)	
		data[i] &= ADAR300x_MAX_RAW;

	for (i = 0, beam = 0, ram_beam_state = 0; ((i + unp_bst_len) <= block->block.bytes_used) && ram_beam_state < ADAR300x_MAX_RAM_STATES;)
	{
		if (beam == ADAR300x_BEAMS_PER_DEVICE)
			ram_beam_state++;

		beam %= ADAR300x_BEAMS_PER_DEVICE;
		if (!adar300x_is_beam_active(beam, mask)) {
			beam++;
			continue;
		}
		adar300x_pack_data(packed, &data[i], ADAR300x_UNPACKED_BEAMSTATE_LEN);

#ifdef DEBUG_ADAR300x
		dev_err(&dac->spi->dev, "data: %x %x %x %x %x %x %x, %x",data[i+0], data[i+1], data[i+2], data[i+3], data[i+4], data[i+5], data[i+6], data[i+7]);
		dev_err(&dac->spi->dev, "packed: %x %x %x %x %x %x", packed[0], packed[1], packed[2], packed[3], packed[4], packed[5]);
#endif // DEBUG_ADAR300x
		if (dac->beam_load_mode[beam] == ADAR300x_MEMORY_CTRL)
		{
			ret = adar300x_set_page(dac, beam + 1);
			if (ret < 0)
				return ret;

			addr = ADAR300x_RAM_BEAM_STATE(ram_beam_state);
			addr += dac->beam_index[beam];
#ifdef DEBUG_ADAR300x
			dev_err(&dac->spi->dev, "ram beam: %d, addr: %x, i:%d, data + i: %08x, beam_state: %d", beam, addr, i, *(data + i), ram_beam_state);
#endif // DEBUG_ADAR300x
			// ret = adar300x_bulk_write(dac, addr, (data + i), ADAR300x_BEAMSTATE_LEN);
			if ((addr + 5) > ADAR300x_RAM_MAX_ADDR)
				return -EINVAL;

		} else if(dac->beam_load_mode[beam] == ADAR300x_FIFO_CTRL)
		{
			ret = adar300x_set_page(dac, ADAR300x_FIFO_PAGE);
			if (ret < 0)
				return ret;
			// ret = adar300x_bulk_write(dac, ADAR300x_FIFO_LOAD(j), (data + i), ADAR300x_BEAMSTATE_LEN);
			// if (ret < 0)
			// 	goto error;
			addr = ADAR300x_FIFO_LOAD(beam);
#ifdef DEBUG_ADAR300x
			dev_err(&dac->spi->dev, "fifo beam: %d, addr %x, i:%d, data + i: %08x, data + i: %p", beam, ADAR300x_FIFO_LOAD(beam), i, *(data + i), (data + i));
#endif // DEBUG_ADAR300x
		}
		else
			return -EINVAL;

		for (j = 0; j < p_bst_len; j++) {
			ret = adar300x_reg_write(dac, addr + j, (packed[j]));
			if (ret < 0)
				return ret;
		}

		i += unp_bst_len;
		beam++;
	}
#ifdef DEBUG_ADAR300x
	dev_err(&dac->spi->dev, "adar300x_ram_write exit OK");
#endif // DEBUG_ADAR300x

	return 0;
}

/* Will be called only when blocksize if full */
static int hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct adar300x_state *dac = queue->driver_data;
	int ret;
	u32 ch_mask = *queue->buffer.scan_mask;

#ifdef DEBUG_ADAR300x
	char *data = block->vaddr;
	char rd_data;
	int i;

	dev_err(&dac->spi->dev, "Writing data: offest: %d -- used: %d -- size: %d -- id: %d",
		block->block.data.offset, block->block.bytes_used,
		block->block.size, block->block.id);

	dev_err(&dac->spi->dev, "type: %x, flags: %x", block->block.type, block->block.flags);

	dev_err(&dac->spi->dev, "queue->buffer.length: %x, queue->buffer.scan_mask: %x, queue->buffer.channel_mask: %x",
		queue->buffer.length, (unsigned int)(*queue->buffer.scan_mask), (unsigned int)(*queue->buffer.channel_mask));
#endif // DEBUG_ADAR300x
	block->block.bytes_used = block->block.size;
#ifdef DEBUG_ADAR300x
	pr_err("%s: %d: hw_submit_block \n", __func__, __LINE__);
#endif // DEBUG_ADAR300x
	
	ret = adar300x_ram_write(dac, block, ch_mask);
	if (ret < 0)
		return ret;

#ifdef DEBUG_ADAR300x
	for(i = 0; i < 24; i++)
	{
		ret = adar300x_ram_read(dac, 1, 0x100 + i, &rd_data);
		if (ret < 0)
			return ret;

		dev_err(&dac->spi->dev, "rd_data i: %d addr:%x read data: %x",i , 0x100+i, rd_data);
	}

	for (i = 0; i < block->block.bytes_used; i++) {
		printk(KERN_INFO "%d: %x ", i, data[i]);
	}
#endif // DEBUG_ADAR300x

	iio_dma_buffer_block_done(block);

	return 0;
}

static void hw_abort(struct iio_dma_buffer_queue *queue)
{
#ifdef DEBUG_ADAR300x
	struct adar300x_state *dac = queue->driver_data;
	dev_err(&dac->spi->dev, "Aborting");
#endif // DEBUG_ADAR300x
}
static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = hw_abort, 
};
#define MAX_TRIGER_NAME_LENGTH 50

static void iio_dmaengine_buffer_release(struct iio_buffer *buf)
{
#ifdef DEBUG_ADAR300x
	pr_err("Dma release");
#endif // DEBUG_ADAR300x
}

//#include "buffer_wrappers.cc"

static const struct iio_buffer_access_funcs iio_dmaengine_buffer_ops = {
	.read = iio_dma_buffer_read,
	.write = iio_dma_buffer_write,
	.set_bytes_per_datum = iio_dma_buffer_set_bytes_per_datum,
	.set_length = iio_dma_buffer_set_length,
	.enable = iio_dma_buffer_enable,
	.disable = iio_dma_buffer_disable,
	.data_available = iio_dma_buffer_data_available,
	.space_available = iio_dma_buffer_space_available,
	.release = iio_dmaengine_buffer_release,

	.modes = INDIO_BUFFER_HARDWARE,
	.flags = INDIO_BUFFER_FLAG_FIXED_WATERMARK,
};

static int adar300x_remove(struct spi_device *spi)
{
	struct adar300x_state	*dac;
#ifdef DEBUG_ADAR300x
	dev_err(&spi->dev, "Cleaning driver resources\n");
#endif // DEBUG_ADAR300x
	dac = dev_get_drvdata(&spi->dev);

	if (dac->dma_buffer) {
		//dev_err(&spi->dev, "Cleaning dma buffer\n");
		iio_buffer_put(&dac->queue.buffer);
	}

	//dev_err(&spi->dev, "Cleaning success\n");
	return 0;
}

static int adar300x_setup_buffer(struct device *dev, struct iio_dev *indio_dev,
				int irq)
{
	struct adar300x_state	*dac;
	int			err;

	dac = iio_priv(indio_dev);

	/* Allocatate dma buffer for adar300x_UPDATE_DAC mode */
	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	err = iio_dma_buffer_init(&dac->queue, dev, &dma_buffer_ops, dac);
	if (err)
		goto error;

	dac->queue.buffer.access = &iio_dmaengine_buffer_ops;
	dac->dma_buffer = iio_buffer_get(&dac->queue.buffer);
		indio_dev->buffer = dac->dma_buffer;
#ifdef DEBUG_ADAR300x
	dev_err(dev, "Dma: %p\n", (void *)dac->dma_buffer);
#endif // DEBUG_ADAR300x

	return 0;
error:
	adar300x_remove(dac->spi);
	return err;
}

static struct iio_info adar300x_info = {
	.read_raw = &adar300x_read_raw,
	.write_raw = &adar300x_write_raw,
	.debugfs_reg_access = &adar300x_reg_access,
};

int adar300x_probe(struct spi_device *spi, const struct attribute_group *attr_group)
{
	struct adar300x_state		**st;
	struct iio_dev			**indio_dev;
	const struct adar300x_chip_info *info;
	struct device_node 		*child, *np = spi->dev.of_node;
	struct regmap 			*regmap;
	int				err = 0;
	int 				ret, cnt = 0, num_dev;
	u32				tmp;

	pr_err("%s: %d: adar300x_probe \n", __func__, __LINE__);

	num_dev = of_get_available_child_count(np);
	if (num_dev < 1 || num_dev > ADAR300x_MAX_DEV) {
		dev_err(&spi->dev, "Number of devices is incorrect (%d)\n", num_dev);
		return -ENODEV;
	}

	indio_dev = devm_kzalloc(&spi->dev, num_dev * sizeof(*indio_dev), GFP_KERNEL);
	if (!indio_dev)
		return -ENOMEM;

	st = devm_kzalloc(&spi->dev, num_dev * sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &adar300x_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	for_each_available_child_of_node(np, child) {
		indio_dev[cnt] = devm_iio_device_alloc(&spi->dev,
						       sizeof(**st));
		if (!indio_dev[cnt])
			return -ENOMEM;

		info = of_device_get_match_data(&spi->dev);
		if (!info)
			return -ENODEV;

		st[cnt] = iio_priv(indio_dev[cnt]);
		//memset(st, 0, sizeof(*st));
		// st[cnt]->indio_dev = indio_dev[cnt];
		st[cnt]->spi = spi;
		dev_set_drvdata(&spi->dev, st[cnt]);
		st[cnt]->chip_info = info;
		st[cnt]->regmap = regmap;

		ret = of_property_read_u32(child, "reg", &tmp);
		if (ret < 0)
			return ret;

		st[cnt]->dev_addr = ADAR300x_SPI_ADDR(tmp);

		indio_dev[cnt]->dev.parent = &spi->dev;
		indio_dev[cnt]->name = child->name;
		indio_dev[cnt]->channels = st[cnt]->chip_info->channels;
		indio_dev[cnt]->num_channels = st[cnt]->chip_info->num_channels;
		adar300x_info.attrs = attr_group;
		indio_dev[cnt]->info = &adar300x_info;
		indio_dev[cnt]->modes = INDIO_DIRECT_MODE;
		indio_dev[cnt]->direction = IIO_DEVICE_DIRECTION_OUT;
		indio_dev[cnt]->available_scan_masks = adar300x_available_scan_masks;
		ret = adar300x_setup(indio_dev[cnt]);
		if (ret < 0) {
			dev_err(&spi->dev, "Setup failed (%d), label: %s, dev: %d, cnt: %d\n", ret, indio_dev[cnt]->label, tmp, cnt);
			return ret;
		}

		/* Do setup for each device */
		err = adar300x_setup_buffer(&spi->dev, indio_dev[cnt], spi->irq);
		if (err) {
			dev_err(&spi->dev, "Error buffer setup\n");
			return ret;
		}

		ret = devm_iio_device_register(&spi->dev, indio_dev[cnt]);
		if (ret < 0)
			return ret;
	
		cnt++;
	}

	return 0;
}
