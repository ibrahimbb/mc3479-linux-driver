#include <linux/init.h>

static int max11040k_probe(struct spi_device *spi){
	return 0;
}

static const struct of_device_id max11040k_spi_of_id[] = {
	{ .compatible = "adi,max11040k" },
	{ }
};
MODULE_DEVICE_TABLE(of, max11040k_spi_of_id);

static const struct spi_device_id max11040k_id[] = {
	{ "max11040k" },
	{ }
};
MODULE_DEVICE_TABLE(spi, max11040k_id);

static struct spi_driver max11040k_driver = {
	.driver = {
		.name	= "max11040k",
		.of_match_table = max11040k_spi_of_id,
	},
	.probe		= max11040k_probe,
	.id_table	= max11040k_id,
};
module_spi_driver(max11040k_driver);