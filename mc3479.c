#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>

static int mc3479_probe(struct spi_device *spi){
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