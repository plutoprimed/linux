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

enum adar3003_iio_dev_attr {
	ADAR3003_EL0VH,
	ADAR3003_EL1VH,
	ADAR3003_EL2VH,
	ADAR3003_EL3VH,
	ADAR3003_ELEM_NO,
};

static IIO_DEVICE_ATTR(el0vh_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3003_EL0VH);
static IIO_DEVICE_ATTR(el1vh_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3003_EL1VH);
static IIO_DEVICE_ATTR(el2vh_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3003_EL2VH);
static IIO_DEVICE_ATTR(el3vh_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3003_EL3VH);

static IIO_DEVICE_ATTR(update_intf_ctrl_available, 0444, adar300x_show_update_intf_ctrl_available, NULL, 0);

static IIO_DEVICE_ATTR(update_intf_ctrl, 0644, adar300x_update_intf_ctrl_show, adar300x_update_intf_ctrl_store, 0);

static struct attribute *adar3003_attributes[] = {
	&iio_dev_attr_el0vh_update.dev_attr.attr,
	&iio_dev_attr_el1vh_update.dev_attr.attr,
	&iio_dev_attr_el2vh_update.dev_attr.attr,
	&iio_dev_attr_el3vh_update.dev_attr.attr,

	&iio_dev_attr_update_intf_ctrl.dev_attr.attr,
	&iio_dev_attr_update_intf_ctrl_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group adar3003_attribute_group = {
	.attrs = adar3003_attributes,
};

#define DECLARE_ADAR3003_CHANNELS(name)					\
static const struct iio_chan_spec name[] = {				\
	ADAR300x_DELAY_CH(0, 0, "BEAM_V_EL0_DELAY"),			\
	ADAR300x_ATTEN_CH(1, 0, "BEAM_V_EL0_ATTENUATION"),		\
	ADAR300x_DELAY_CH(2, 1, "BEAM_H_EL0_DELAY"),			\
	ADAR300x_ATTEN_CH(3, 1, "BEAM_H_EL0_ATTENUATION"),		\
	ADAR300x_DELAY_CH(4, 2, "BEAM_V_EL1_DELAY"),			\
	ADAR300x_ATTEN_CH(5, 2, "BEAM_V_EL1_ATTENUATION"),		\
	ADAR300x_DELAY_CH(6, 3, "BEAM_H_EL1_DELAY"),			\
	ADAR300x_ATTEN_CH(7, 3, "BEAM_H_EL1_ATTENUATION"),		\
	ADAR300x_DELAY_CH(8, 4, "BEAM_V_EL2_DELAY"),			\
	ADAR300x_ATTEN_CH(9, 4, "BEAM_V_EL2_ATTENUATION"),		\
	ADAR300x_DELAY_CH(10, 5, "BEAM_H_EL2_DELAY"),			\
	ADAR300x_ATTEN_CH(11, 5, "BEAM_H_EL2_ATTENUATION"),		\
	ADAR300x_DELAY_CH(12, 6, "BEAM_V_EL3_DELAY"),			\
	ADAR300x_ATTEN_CH(13, 6, "BEAM_V_EL3_ATTENUATION"),		\
	ADAR300x_DELAY_CH(14, 7, "BEAM_H_EL3_DELAY"),			\
	ADAR300x_ATTEN_CH(15, 7, "BEAM_H_EL3_ATTENUATION"),		\
									\
	ADAR300x_TEMP(32, 16, TEMP),					\
};

DECLARE_ADAR3003_CHANNELS(adar3003_channels);

static const struct adar300x_chip_info adar3003_chip_info_tbl[] = {
	[ID_ADAR3003] = {
		.chip_id = ID_ADAR3003,
		.channels = adar3003_channels,
		.num_channels = ARRAY_SIZE(adar3003_channels),
	},
};
static const struct of_device_id adar3003_of_match[] = {
	{ .compatible = "adi,adar3003",
		.data = &adar3003_chip_info_tbl[ID_ADAR3003], },
	{ }
};

MODULE_DEVICE_TABLE(of, adar3000_of_match);

int adar3003_probe(struct spi_device *spi) {
	return adar300x_probe(spi, &adar3003_attribute_group);
}

static struct spi_driver adar3003_driver = {
	.driver = {
		.name	= "adar3003",
		.of_match_table = adar3003_of_match,
	},
	.probe = adar3003_probe,
};
module_spi_driver(adar3003_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADAR3000 Beamformer");
MODULE_LICENSE("GPL v2");
