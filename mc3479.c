#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

struct mc3479_prv
{
    struct device *dev;
    struct iio_dev *indio_dev;
    struct spi_device *spi;
};


static int mc3479_probe(struct spi_device *spi){
	int ret;

    struct device *dev = &spi->dev;
    struct iio_dev *indio_dev;
    struct mc3479_prv *prv;

    indio_dev = devm_iio_device_alloc(dev, sizeof(*prv));
    if(!indio_dev)
		return -ENOMEM;

    prv = iio_priv(indio_dev);

    prv->dev = dev;
    prv->indio_dev = indio_dev;
    prv->spi = spi;
    
    spi->mode = SPI_MODE_3;
    ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret,
							"spi setup failed for mc3479!");

    return 0;
}

static const struct of_device_id mc3479_spi_of_id[] = {
	{ .compatible = "memsic,mc3479" },
	{ }
};
MODULE_DEVICE_TABLE(of, mc3479_spi_of_id);

static const struct spi_device_id mc3479_id[] = {
	{ "mc3479" },
	{ }
};
MODULE_DEVICE_TABLE(spi, mc3479_id);

static struct spi_driver mc3479_driver = {
	.driver = {
		.name	= "mc3479",
		.of_match_table = mc3479_spi_of_id,
	},
	.probe		= mc3479_probe,
	.id_table	= mc3479_id,
};
module_spi_driver(mc3479_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ibrahim Bagriyanik");
MODULE_DESCRIPTION("Device driver for accel MC3479.");