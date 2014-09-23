/*
 * @file bd71805.c  --  RoHM BD71805MWV mfd driver
 * 
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * @author: Tony Luo <luofc@embedinfo.com>
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/bd71805.h>

/** @brief bd71805 multi function cells */
static struct mfd_cell bd71805_mfd_cells[] = {
	{
		.name = "bd71805-pmic",
	},
	{
		.name = "bd71805-power",
	},
};

static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	struct bd71805 *bd71805 = dev_get_drvdata(dev);

	/*
	 * Caching all regulator registers.
	 */
	return true;
}

/** @brief regmap configures */
static const struct regmap_config bd71805_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = is_volatile_reg,
	.max_register = BD71805_MAX_REGISTER - 1,
	.cache_type = REGCACHE_RBTREE,
};

#ifdef CONFIG_OF
static struct of_device_id bd71805_of_match[] = {
	{ .compatible = "rohm,bd71805", .data = (void *)0},
	{ },
};
MODULE_DEVICE_TABLE(of, bd71805_of_match);


/** @brief parse device tree data of bd71805
 *  @param client client object provided by system
 *  @param chip_id return chip id back to caller
 *  @return board initialize data
 */
static struct bd71805_board *bd71805_parse_dt(struct i2c_client *client,
						int *chip_id)
{
	struct device_node *np = client->dev.of_node;
	struct bd71805_board *board_info;
	unsigned int prop;
	const struct of_device_id *match;
	int ret = 0;

	match = of_match_device(bd71805_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	*chip_id  = (int)match->data;

	board_info = devm_kzalloc(&client->dev, sizeof(*board_info),
			GFP_KERNEL);
	if (!board_info) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	return board_info;
}
#else
static inline
struct bd71805_board *bd71805_parse_dt(struct i2c_client *client,
					 int *chip_id)
{
	return NULL;
}
#endif

/** @brief probe bd71805 device
 *  @param i2c client object provided by system
 *  @param id chip id
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71805_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct bd71805 *bd71805;
	struct bd71805_board *pmic_plat_data;
	struct bd71805_board *of_pmic_plat_data = NULL;
	int chip_id = id->driver_data;
	int ret = 0;

	pmic_plat_data = dev_get_platdata(&i2c->dev);

	if (!pmic_plat_data && i2c->dev.of_node) {
		pmic_plat_data = bd71805_parse_dt(i2c, &chip_id);
		of_pmic_plat_data = pmic_plat_data;
	}

	if (!pmic_plat_data)
		return -EINVAL;

	bd71805 = kzalloc(sizeof(struct bd71805), GFP_KERNEL);
	if (bd71805 == NULL)
		return -ENOMEM;

	bd71805->of_plat_data = of_pmic_plat_data;
	i2c_set_clientdata(i2c, bd71805);
	bd71805->dev = &i2c->dev;
	bd71805->i2c_client = i2c;
	bd71805->id = chip_id;
	mutex_init(&bd71805->io_mutex);

	bd71805->regmap = devm_regmap_init_i2c(i2c, &bd71805_regmap_config);
	if (IS_ERR(bd71805->regmap)) {
		ret = PTR_ERR(bd71805->regmap);
		dev_err(&i2c->dev, "regmap initialization failed: %d\n", ret);
		return ret;
	}

	ret = mfd_add_devices(bd71805->dev, -1,
			      bd71805_mfd_cells, ARRAY_SIZE(bd71805_mfd_cells),
			      NULL, 0, NULL);
	if (ret < 0)
		goto err;

	return ret;

err:
	mfd_remove_devices(bd71805->dev);
	kfree(bd71805);
	return ret;
}

/** @brief remove bd71805 device
 *  @param i2c client object provided by system
 *  @return 0
 */
static int bd71805_i2c_remove(struct i2c_client *i2c)
{
	struct bd71805 *bd71805 = i2c_get_clientdata(i2c);

	mfd_remove_devices(bd71805->dev);
	kfree(bd71805);

	return 0;
}

static const struct i2c_device_id bd71805_i2c_id[] = {
       { "bd71805", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, bd71805_i2c_id);


static struct i2c_driver bd71805_i2c_driver = {
	.driver = {
		.name = "bd71805",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bd71805_of_match),
	},
	.probe = bd71805_i2c_probe,
	.remove = bd71805_i2c_remove,
	.id_table = bd71805_i2c_id,
};

static int __init bd71805_i2c_init(void)
{
	return i2c_add_driver(&bd71805_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(bd71805_i2c_init);

static void __exit bd71805_i2c_exit(void)
{
	i2c_del_driver(&bd71805_i2c_driver);
}
module_exit(bd71805_i2c_exit);

MODULE_AUTHOR("Tony Luo <luofc@embest-tech.com>");
MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_DESCRIPTION("BD71805MWV chip multi-function driver");
MODULE_LICENSE("GPL");
