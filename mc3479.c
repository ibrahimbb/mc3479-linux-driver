#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "mc3479.h"

enum mc3479_device_state {
    MC3479_STANDBY = 0x00,
    MC3479_WAKE = 0x01,
};

enum mc3479_sampling_rate {
    MC3479_RATE50   = 0x08,
    MC3479_RATE100  = 0x09,
    MC3479_RATE125  = 0x0A,
    MC3479_RATE200  = 0x0B,
    MC3479_RATE250  = 0x0C,
    MC3479_RATE500  = 0x0D,
    MC3479_RATE1000 = 0x0E,
    MC3479_RATE2000 = 0x0F
};

struct mc3479_prv
{
    struct device *dev;
    struct iio_dev *indio_dev;
    struct spi_device *spi;

    enum mc3479_device_state state;
};

/**
 * mc3479_read_reg - reads 8 bit value from register.
 * 
 * @param indio_dev
 * @param addr: unsigned 8 bit address of register and a dummy byte.
 * @param rx_buf: unsigned 8 bit receive buffer.
 * 
 * Returns: zero on success, else a negative error code.
 */
static int mc3479_read_reg(struct iio_dev *indio_dev, u16 addr, u8 *rx_buf){
    int ret;
    struct mc3479_prv *prv = iio_priv(indio_dev);
    
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
 * @param indio_dev
 * @param addr: unsigned 8 bit address of register.
 * @param rx_buf: unsigned 8 bit value to write.
 * 
 * Returns: zero on success, else a negative error code.
 */
static int mc3479_write_reg(struct iio_dev *indio_dev, u8 addr, u8 cmd){
    int ret = 0;
    struct mc3479_prv *prv = iio_priv(indio_dev);

    u8 tx_buf[2];

    tx_buf[0] = addr;
    tx_buf[1] = cmd;

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

/**
 * mc3479_burst_read - reads registers sequentially.
 * 
 * "A burst (multi-byte) register write cycle uses the address 
 * specified at the beginning of the transaction as the starting 
 * register address. Internally the address will auto-increment 
 * to the next consecutive address for each additional byte (8-clocks) 
 * of data written beyond clock 8." per datasheet.
 * 
 * @param indio_dev
 * @param addr: 8 bit addr of the starting register and a dummy byte
 * @param rx_buf: pointer to s16 type array
 * @param len: length of the read in bytes
 */
static int mc3479_burst_read(struct iio_dev *indio_dev, u16 addr, 
                                                     s16 *rx_buf, u8 len){
    int ret;
    struct mc3479_prv *prv = iio_priv(indio_dev);

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
            .len = len
        }
    };

    //Using spi_message to make transfers atomic
    struct spi_message message;
	spi_message_init_with_transfers(&message, xfers, ARRAY_SIZE(xfers));

	ret = spi_sync(prv->spi, &message);
    if (ret)
        return ret;

    return 0;
}

/**
 * mc3479_change_operation_state - change operation mode
 * 
 * MC3479 has 2 modes:
 * STANDBY. Clocks are not running and X, Y, and Z-axis data are not sampled.
 * WAKE. Clocks are running and X, Y, and Z-axis data are acquired at the 
 * sample rate. per datasheet.
 * 
 * @param indio_dev: iio_dev
 * @param state: one of two states. 0 for standby, 1 for wake.
 */
static int mc3479_change_operation_state(struct iio_dev *indio_dev, 
                                            unsigned int state){
    int ret;
    struct mc3479_prv *prv = iio_priv(indio_dev);

    switch (state)
    {
    case MC3479_STANDBY:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_MODE, MC3479_STANDBY);
        if (ret)
            return ret;

        prv->state = MC3479_STANDBY;
        break;
    
    case MC3479_WAKE:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_MODE, MC3479_WAKE);
        if (ret)
            return ret;

        prv->state = MC3479_WAKE;
        break;
    
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static int mc3479_set_sampling_rate(struct iio_dev *indio_dev, unsigned int odr){
    int ret;
    struct mc3479_prv *prv = iio_priv(indio_dev);

    switch (odr)
    {
    case MC3479_RATE50:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
        return ret;
        break;

    case MC3479_RATE100:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
        return ret;
        break;

    case MC3479_RATE125:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
        return ret;
        break;

    case MC3479_RATE200:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
        return ret;
        break;

    case MC3479_RATE250:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
        return ret;
        break;

    case MC3479_RATE500:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
        return ret;
        break;

    case MC3479_RATE1000:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
        return ret;
        break;

    case MC3479_RATE2000:
        ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
        return ret;
        break;

    default:
        return -EINVAL;
        break;
    }
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