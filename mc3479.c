#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "mc3479.h"

// clang-format off
#define MC3479_PROD_ID 			0xA4
#define MC3479_REG_GPIO_CTRL 	0x33
#define  MC3479_INT_PP_DRIVE	(BIT(7) | BIT(3))
#define  MC3479_INT_POL_LOW 	(0x00 | MC3479_INT_PP_DRIVE)
#define  MC3479_INT_POL_HIG 	(BIT(6) | BIT(2) | MC3479_INT_PP_DRIVE)
// clang-format on

enum mc3479_device_state {
	MC3479_STANDBY = 0x00,
	MC3479_WAKE = 0x01,
	MC3479_INIT_STATE
};

enum mc3479_odr {
	MC3479_RATE50 = 0x08,
	MC3479_RATE100 = 0x09,
	MC3479_RATE125 = 0x0A,
	MC3479_RATE200 = 0x0B,
	MC3479_RATE250 = 0x0C,
	MC3479_RATE500 = 0x0D,
	MC3479_RATE1000 = 0x0E,
	MC3479_RATE2000 = 0x0F,
	MC3479_INIT_ODR
};

struct mc3479_prv {
	struct device *dev;
	struct spi_device *spi;
	struct iio_dev *indio_dev;
	struct iio_trigger *trig;

	struct mutex mtx; //Lock to protect below enumerators for state
	enum mc3479_device_state state;
	enum mc3479_odr odr;
};

#define MC3479_ACCEL_CHANNEL(index, reg, axis)                            \
	{                                                                 \
		.type = IIO_ACCEL, .address = reg, .modified = 1,         \
		.channel2 = IIO_MOD_##axis,                               \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),             \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.scan_index = index, .scan_type = {                       \
			.sign = 's',                                      \
			.realbits = 16,                                   \
			.storagebits = 32,                                \
			.shift = 0,                                       \
			.endianness = IIO_LE,                             \
		}                                                         \
	}

static const struct iio_chan_spec mc3479_channels[] = {
	MC3479_ACCEL_CHANNEL(0, MC3479_REG_XOUT_LSB, X),
	MC3479_ACCEL_CHANNEL(1, MC3479_REG_YOUT_LSB, Y),
	MC3479_ACCEL_CHANNEL(2, MC3479_REG_ZOUT_LSB, Z),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const int mc3479_odr_comm_table[8] = {
	MC3479_RATE50,	MC3479_RATE100, MC3479_RATE125,	 MC3479_RATE200,
	MC3479_RATE250, MC3479_RATE500, MC3479_RATE1000, MC3479_RATE2000,
};

static const int mc3479_odr_table[8] = {
	50, 100, 125, 200, 250, 500, 1000, 2000
};

static int mc3479_get_odr(int odr_comm)
{
	for (int i = 0; i < ARRAY_SIZE(mc3479_odr_comm_table); i++) {
		//Return the odr
		if (mc3479_odr_comm_table[i] == odr_comm)
			return mc3479_odr_table[i];
	}

	return -EINVAL;
}

static int mc3479_get_odr_comm(int odr)
{
	for (int i = 0; i < ARRAY_SIZE(mc3479_odr_table); i++) {
		if (mc3479_odr_comm_table[i] == odr)
			return mc3479_odr_table[i];
	}

	return -EINVAL;
}

/**
 * mc3479_read_reg - reads 8 bit value from register.
 * 
 * @param indio_dev
 * @param addr: unsigned 8 bit address of register and a dummy byte.
 * @param rx_buf: unsigned 8 bit receive buffer.
 * 
 * Returns: zero on success, else a negative error code.
 */
static int mc3479_read_reg(struct iio_dev *indio_dev, u8 addr, u8 *rx_buf)
{
	int ret;
	struct mc3479_prv *prv = iio_priv(indio_dev);

	//read mask
	addr = addr | 0x80;

	u8 regval[2];

	regval[0] = addr;
	regval[1] = 0x00; //Dummy byte

	//address is written in 2 bytes, response read in a byte
	struct spi_transfer xfers[] = { { .tx_buf = regval, .len = 2 },
					{ .rx_buf = rx_buf, .len = 1 } };

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
static int mc3479_write_reg(struct iio_dev *indio_dev, u8 addr, u8 cmd)
{
	int ret = 0;
	struct mc3479_prv *prv = iio_priv(indio_dev);

	u8 tx_buf[2];

	tx_buf[0] = addr;
	tx_buf[1] = cmd;

	struct spi_transfer xfer = { .tx_buf = &tx_buf, .len = 2 };

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
 * @param addr: 8 bit addr of the register to write
 * @param rx_buf: pointer to u8 type array
 * @param len: length of the read in bytes
 */
static int mc3479_burst_read(struct iio_dev *indio_dev, u8 addr, u16 *rx_buf,
			     u8 len)
{
	int ret;
	struct mc3479_prv *prv = iio_priv(indio_dev);

	//read mask
	addr = addr | 0x80;

	u8 regval[2];

	regval[0] = addr;
	regval[1] = 0x00; //Dummy byte

	//address is written in 2 bytes, response read in a byte
	struct spi_transfer xfers[] = { { .tx_buf = regval, .len = 2 },
					{ .rx_buf = rx_buf, .len = len } };

	//Using spi_message to make transfers atomic
	struct spi_message message;
	spi_message_init_with_transfers(&message, xfers, ARRAY_SIZE(xfers));

	ret = spi_sync(prv->spi, &message);
	if (ret)
		return ret;

	return 0;
}

/**
 * mc3479_set_operation_state - change operation mode
 * 
 * MC3479 has 2 modes:
 * 1) STANDBY. Clocks are not running and X, Y, and Z-axis data are not sampled.
 * All registers are writeable and readable.
 * 
 * 2) WAKE. Clocks are running and X, Y, and Z-axis data are acquired at the 
 * sample rate. per datasheet. Only mode register is writable, all registers are
 * readable.
 * 
 * Is not thread-safe. Should be called when prv->mtx is locked.

 */
static int mc3479_set_operation_state(struct iio_dev *indio_dev,
				      unsigned int state)
{
	int ret = 0;
	struct mc3479_prv *prv = iio_priv(indio_dev);

	if (state == prv->state)
		return ret;

	switch (state) {
	case MC3479_STANDBY:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_MODE,
				       MC3479_STANDBY);
		break;

	case MC3479_WAKE:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_MODE, MC3479_WAKE);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if (ret)
		return ret;

	prv->state = state;
	return ret;
}

static int mc3479_set_sampling_rate(struct iio_dev *indio_dev, unsigned int odr)
{
	int ret = 0;
	struct mc3479_prv *prv = iio_priv(indio_dev);

	mutex_lock(&prv->mtx);

	if (odr == prv->odr)
		goto unlock_and_ret;

	ret = mc3479_set_operation_state(indio_dev, MC3479_STANDBY);
	if (ret)
		goto unlock_and_ret;

	switch (odr) {
	case MC3479_RATE50:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
		break;

	case MC3479_RATE100:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
		break;

	case MC3479_RATE125:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
		break;

	case MC3479_RATE200:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
		break;

	case MC3479_RATE250:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
		break;

	case MC3479_RATE500:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
		break;

	case MC3479_RATE1000:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
		break;

	case MC3479_RATE2000:
		ret = mc3479_write_reg(indio_dev, MC3479_REG_SR, odr);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if (ret)
		goto unlock_and_ret;

	prv->odr = odr;

unlock_and_ret:
	mutex_unlock(&prv->mtx);
	return ret;
}

static int mc3479_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	int ret;
	struct mc3479_prv *prv = iio_priv(indio_dev);

	mutex_lock(&prv->mtx);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			goto unlock_and_ret;

		ret = mc3479_set_operation_state(indio_dev, MC3479_WAKE);
		if (ret)
			goto release_unlock_and_ret;

		u16 regval;
		ret = mc3479_burst_read(indio_dev, chan->address, &regval, 2);
		if (ret)
			goto release_unlock_and_ret;

		u32 val_u32 = (0x0000 << 16) | regval;
		s32 val_s32 = sign_extend32(val_u32, 15);

		*val = val_s32;

		ret = IIO_VAL_INT;
		goto release_unlock_and_ret;
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = mc3479_get_odr(prv->odr);
		if (ret < 0)
			goto unlock_and_ret;

		*val = ret;

		ret = IIO_VAL_INT;
		goto unlock_and_ret;
		break;

	default:
		ret = -EINVAL;
		goto unlock_and_ret;
		break;
	}

release_unlock_and_ret:
	iio_device_release_direct_mode(indio_dev);
unlock_and_ret:
	mutex_unlock(&prv->mtx);
	return ret;
}

struct iio_info mc3479_info = { .read_raw = &mc3479_read_raw };

static int mc3479_setup_irq(struct iio_dev *indio_dev)
{
	int ret;
	struct mc3479_prv *prv = iio_priv(indio_dev);

	int irq = fwnode_irq_get_byname(dev_fwnode(prv->dev), "INT1");
	if (irq > 0) {
		u32 irq_type = irq_get_trigger_type(irq);

		switch (irq_type) {
		case IRQ_TYPE_EDGE_FALLING:
		case IRQ_TYPE_LEVEL_LOW:
			ret = mc3479_write_reg(indio_dev, MC3479_REG_GPIO_CTRL,
					       MC3479_INT_POL_HIG);
			break;

		case IRQ_TYPE_EDGE_RISING:
		case IRQ_TYPE_LEVEL_HIGH:
			ret = mc3479_write_reg(indio_dev, MC3479_REG_GPIO_CTRL,
					       MC3479_INT_POL_LOW);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		if (ret)
			return ret;

		ret = mc3479_write_reg(indio_dev, MC3479_REG_INTR_CTRL, 0xC0);
		if (ret)
			return ret;
	}

	return irq;
}

irqreturn_t mc3479_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct mc3479_prv *prv = iio_priv(indio_dev);
	mutex_lock(&prv->mtx);

	//Do trigger handling stuff here.

	mutex_unlock(&prv->mtx);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int mc3479_setup(struct iio_dev *indio_dev)
{
	int ret;
	u8 regval;
	struct mc3479_prv *prv = iio_priv(indio_dev);

	mutex_lock(&prv->mtx);
	ret = mc3479_set_operation_state(indio_dev, MC3479_STANDBY);
	mutex_unlock(&prv->mtx);
	if (ret)
		return ret;

	//Read chip identification register
	ret = mc3479_read_reg(indio_dev, MC3479_REG_PROD, &regval);
	if (ret)
		return ret;

	if (regval != MC3479_PROD_ID)
		dev_warn(prv->dev, "Invalid DEV ID 0x%02x\n", regval);

	ret = mc3479_set_sampling_rate(indio_dev, MC3479_RATE2000);
	if (ret)
		return ret;

	return 0;
}

static int mc3479_probe(struct spi_device *spi)
{
	int ret;

	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct mc3479_prv *prv;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*prv));
	if (!indio_dev)
		return -ENOMEM;

	//Setup SPI mode
	spi->mode = SPI_MODE_3;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi setup failed for mc3479!");

	//Fill iio_dev
	indio_dev->name = "mc3479";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mc3479_info;
	indio_dev->channels = mc3479_channels;
	indio_dev->num_channels = ARRAY_SIZE(mc3479_channels);

	//Fill private structure
	prv = iio_priv(indio_dev);
	prv->dev = dev;
	prv->indio_dev = indio_dev;
	prv->spi = spi;
	prv->state = MC3479_INIT_STATE;
	prv->odr = MC3479_INIT_ODR;

	mutex_init(&prv->mtx);

	ret = mc3479_setup(indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "MC3479 setup failed!\n");

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &mc3479_trigger_handler, NULL);

	int irq = mc3479_setup_irq(indio_dev);
	if (irq > 0) {
		prv->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
						   indio_dev->name,
						   iio_device_id(indio_dev));
		if (!prv->trig)
			return dev_err_probe(
				dev, -ENOMEM,
				"MC3479 trigger allocation failed!");

		iio_trigger_set_drvdata(prv->trig, indio_dev);

		ret = devm_request_irq(dev, irq,
				       &iio_trigger_generic_data_rdy_poll,
				       IRQF_ONESHOT, "mc3479_irq", prv->trig);
		if (ret)
			return dev_err_probe(dev, ret,
					     "request irq %d failed\n", irq);

		ret = devm_iio_trigger_register(dev, prv->trig);
		if (ret)
			return dev_err_probe(
				dev, ret, "MC36479 trigger register failed\n");
	}

	mc3479_set_operation_state(indio_dev, MC3479_WAKE);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id mc3479_spi_of_id[] = {
	{ .compatible = "memsic,mc3479" },
	{}
};
MODULE_DEVICE_TABLE(of, mc3479_spi_of_id);

static const struct spi_device_id mc3479_id[] = { { "mc3479" }, {} };
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