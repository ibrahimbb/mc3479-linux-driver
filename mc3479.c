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

/**
 * mc3479_read_reg - reads 8 bit value from register.
 * 
 * @param prv: private structure.
 * @param addr: unsigned 8 bit address of register and a dummy byte.
 * @param rx_buf: unsigned 8 bit receive buffer.
 * 
 * Returns: zero on success, else a negative error code.
 */
static int mc3479_read_reg(struct mc3479_prv *prv, u16 addr, u8 *rx_buf){
    int ret;
    
    //read mask is 1
    addr = addr | 0x80;

    //address is written in 2 bytes, response read in a byte
    struct spi_transfer xfers[] = {
        {
            .tx_buf = &addr,
            .len = 2
        },
        {
            .rx_buf = rx_buf,
            .len = 1
        }
    };

    struct spi_message message;
	spi_message_init_with_transfers(&message, xfers, ARRAY_SIZE(xfers));

	ret = spi_sync(prv->spi, &message);
    if (ret)
        return ret;

    return 0;
}

/**
 * mc3479_write_reg - writes 8 bit value to the register.
 * 
 * @param prv: private structure.
 * @param addr: unsigned 8 bit address of register.
 * @param rx_buf: unsigned 8 bit value to write.
 * 
 * Returns: zero on success, else a negative error code.
 */
static int mc3479_write_reg(struct mc3479_prv *prv, u8 addr, u8 cmd){
    int ret = 0;

    u16 tx_buf = (cmd << 8) | addr;

    //address is written in 2 bytes, response read in a byte
    struct spi_transfer xfer = {
        .tx_buf = &tx_buf,
        .len = 2
    };

    struct spi_message message;
	spi_message_init_with_transfers(&message, &xfer, 1);

	ret = spi_sync(prv->spi, &message);
    if (ret)
        return ret;

    return ret;
}

static int mc3479_probe(struct spi_device *spi){
	int ret;

    struct device *dev = &spi->dev;
    struct iio_dev *indio_dev;
    struct mc3479_prv *prv;

    indio_dev = devm_iio_device_alloc(dev, sizeof(*prv));
    if(!indio_dev)
		return -ENOMEM;

    //Fill private structure
    prv = iio_priv(indio_dev);
    prv->dev = dev;
    prv->indio_dev = indio_dev;
    prv->spi = spi;
    
    //Setup SPI mode
    spi->mode = SPI_MODE_3;
    ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret,
							"spi setup failed for mc3479!");

    //Fill iio_dev
    indio_dev->name = "mc3479";
    indio_dev->modes = INDIO_DIRECT_MODE;

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