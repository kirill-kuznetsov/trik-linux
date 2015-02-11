#include <linux/input.h>	/* BUS_SPI */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/types.h>
#include "l3g42xxd.h"
static int l3g42xxd_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3g42xxd_chip *chip = i2c_get_clientdata(client);
	l3g42xxd_suspend(chip);
	return 0;
}

static int l3g42xxd_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct l3g42xxd_chip *chip = i2c_get_clientdata(client);
	l3g42xxd_resume(chip);
	return 0;
}

static SIMPLE_DEV_PM_OPS(l3g42xxd_i2c_pm, l3g42xxd_i2c_suspend, l3g42xxd_i2c_resume);


static int l3g42xxd_i2c_write(struct device *dev, unsigned char reg, unsigned char data)
{
	struct i2c_client *client = to_i2c_client(dev);
	
	return i2c_smbus_write_byte_data(client, reg, data);
}

static int l3g42xxd_i2c_read(struct device *dev, unsigned char reg)
{
	struct i2c_client *client = to_i2c_client(dev);

	return i2c_smbus_read_byte_data(client, reg);
}
static  int l3g42xxd_i2c_read_block (struct device * dev, unsigned char reg, int count, void *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return i2c_smbus_read_i2c_block_data(client, reg, count, buf);
}

static int __devexit l3g42xxd_i2c_remove(struct i2c_client *client)
{
	struct l3g42xxd_chip *chip = i2c_get_clientdata(client);
	
	l3g42xxd_remove(chip);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct l3g42xxd_bus_op l3g42xxd_i2c_ops = {
	.bustype	= BUS_I2C,
	.write		= l3g42xxd_i2c_write,
	.read		= l3g42xxd_i2c_read,
	.read_block	= l3g42xxd_i2c_read_block,
};

static int __devinit l3g42xxd_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct l3g42xxd_chip* chip;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		pr_err("%s :client not i2c capable\n",__func__);
		return -ENODEV;
	}

	chip = l3g42xxd_probe(&client->dev, &l3g42xxd_i2c_ops);
		if (IS_ERR(chip)){
			pr_err("%s: l3g42xxd_probe failed\n",__func__);
			return PTR_ERR(chip);
	}
	i2c_set_clientdata(client, chip);

	return 0;
}
static const struct i2c_device_id l3g42xxd_id[] = {
	{"l3g42xxd",0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, l3g42xxd_id);

static struct i2c_driver l3g42xxd_i2c_driver = {
	.driver = {
		.name = "l3g42xxd",
		.pm   = &l3g42xxd_i2c_pm,
	},
	.probe    = l3g42xxd_i2c_probe,
	.remove   = __devexit_p(l3g42xxd_i2c_remove),
	.id_table = l3g42xxd_id,
};

module_i2c_driver(l3g42xxd_i2c_driver);

MODULE_AUTHOR("Roman Meshkevich <romik.momik@trikset.com>");
MODULE_DESCRIPTION("L3gd20 Three-Axis Digital Gyroscope I2C Bus Driver");
MODULE_LICENSE("GPL");

