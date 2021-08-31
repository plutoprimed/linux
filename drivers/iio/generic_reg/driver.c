#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

static const struct of_device_id of_match_generic_reg[] = {
	{ .compatible = "generic-reg" },
	{},
};

struct generic_reg_data {
//	struct device *dev;
	void __iomem *base;
	char name[32];
};

static ssize_t generic_reg_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct iio_dev *idev = dev_to_iio_dev(dev);
	struct generic_reg_data *data = iio_priv(idev);

	return sprintf(buf, "%u\n", readl(data->base));
}

static IIO_DEVICE_ATTR(regval,
	S_IRUGO, generic_reg_get, NULL, 0);
static struct attribute *generic_reg_attributes[] = {
	&iio_dev_attr_regval.dev_attr.attr,
	NULL,
};
static const struct attribute_group generic_reg_attribute_group = {
	.attrs = generic_reg_attributes,
};
static const struct iio_info generic_reg_info = {
	.attrs = &generic_reg_attribute_group,
};

static int generic_reg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct generic_reg_data *data;
	struct iio_dev *idev;
	struct device_node *dt_node = pdev->dev.of_node;

	idev = devm_iio_device_alloc(dev, sizeof(struct generic_reg_data));
	if (!idev) {
		dev_err(dev, "failed to allocate IIO device\n");
		return -ENOMEM;
	}
	data = iio_priv(idev);

	data->base = of_iomap(dt_node, 0);
	if (!data->base)
		return -ENXIO;

	if (of_property_read_string(dt_node, "id", &idev->name))
		idev->name = "regval";

	idev->num_channels = 0;
	idev->modes = INDIO_DIRECT_MODE;
	idev->info = &generic_reg_info;

	return devm_iio_device_register(dev, idev);
}

static struct platform_driver generic_reg_iio_driver= {
	.probe = generic_reg_probe,
	.driver = {
		.name = "generic-reg",
		.of_match_table = of_match_generic_reg,
	}
};

module_platform_driver(generic_reg_iio_driver);

MODULE_AUTHOR("Martin Povišer <poviser@ujf.cas.cz>");
MODULE_DESCRIPTION("IIO generic mmapped-register driver");
MODULE_LICENSE("Dual BSD/GPL");
