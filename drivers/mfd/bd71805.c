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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/bd71805.h>

/** @brief bd71805 irq resource */
static struct resource rtc_resources[] = {
	{
		.start  = BD71805_IRQ_ALARM_12,
		.end    = BD71805_IRQ_ALARM_12,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource power_resources[] = {
	{
		.start	= BD71805_IRQ_DCIN_03,
		.end	= BD71805_IRQ_DCIN_03,
		.flags	= IORESOURCE_IRQ,
	}
};

/** @brief bd71805 multi function cells */
static struct mfd_cell bd71805_mfd_cells[] = {
	{
		.name = "bd71805-pmic",
	},
	{
		.name = "bd71805-power",
		.num_resources = ARRAY_SIZE(power_resources),
		.resources = &power_resources[0],
	},
	{
		.name = "bd71805-gpo",
	},
	{
		.name = "bd71805-rtc",
		.num_resources = ARRAY_SIZE(rtc_resources),
		.resources = &rtc_resources[0],
	},
};

/** @brief bd71805 irqs */
static const struct regmap_irq bd71805_irqs[] = {
	[BD71805_IRQ_BUCK_01] = {
		.mask = BD71805_INT_EN_01_BUCKAST_MASK,
		.reg_offset = 1,
	},
	[BD71805_IRQ_DCIN_02] = {
		.mask = BD71805_INT_EN_02_DCINAST_MASK,
		.reg_offset = 2,
	},
	[BD71805_IRQ_DCIN_03] = {
		.mask = BD71805_INT_EN_03_DCINAST_MASK,
		.reg_offset = 3,
	},
	[BD71805_IRQ_VSYS_04] = {
		.mask = BD71805_INT_EN_04_VSYSAST_MASK,
		.reg_offset = 4,
	},
	[BD71805_IRQ_CHARGE_05] = {
		.mask = BD71805_INT_EN_05_CHGAST_MASK,
		.reg_offset = 5,
	},
	[BD71805_IRQ_BAT_06] = {
		.mask = BD71805_INT_EN_06_BATAST_MASK,
		.reg_offset = 6,
	},
	[BD71805_IRQ_BAT_MON_07] = {
		.mask = BD71805_INT_EN_07_BMONAST_MASK,
		.reg_offset = 7,
	},
	[BD71805_IRQ_BAT_MON_08] = {
		.mask = BD71805_INT_EN_08_BMONAST_MASK,
		.reg_offset = 8,
	},
	[BD71805_IRQ_BAT_MON_09] = {
		.mask = BD71805_INT_EN_09_BMONAST_MASK,
		.reg_offset = 9,
	},
	[BD71805_IRQ_BAT_MON_10] = {
		.mask = BD71805_INT_EN_10_BMONAST_MASK,
		.reg_offset = 10,
	},
	[BD71805_IRQ_TEMPERATURE_11] = {
		.mask = BD71805_INT_EN_11_TMPAST_MASK,
		.reg_offset = 11,
	},
	[BD71805_IRQ_ALARM_12] = {
		.mask = BD71805_INT_EN_12_ALMAST_MASK,
		.reg_offset = 12,
	},
};

/** @brief bd71805 irq chip definition */
static struct regmap_irq_chip bd71805_irq_chip = {
	.name = "bd71805",
	.irqs = bd71805_irqs,
	.num_irqs = ARRAY_SIZE(bd71805_irqs),
	.num_regs = 13,
	.irq_reg_stride = 1,
	.status_base = BD71805_REG_INT_STAT_00,
	.mask_base = BD71805_REG_INT_EN_00,
	.mask_invert = true,
	// .ack_base = BD71805_REG_INT_STAT_00,
};

/** @brief bd71805 irq initialize 
 *  @param bd71805 bd71805 device to init
 *  @param bdinfo platform init data
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71805_irq_init(struct bd71805 *bd71805, struct bd71805_board* bdinfo) {
	int irq;
	int ret = 0;

	if (!bdinfo) {
		dev_warn(bd71805->dev, "No interrupt support, no pdata\n");
		return -EINVAL;
	}
	
	irq = gpio_to_irq(bdinfo->gpio_intr);

	bd71805->chip_irq = irq;
	ret = regmap_add_irq_chip(bd71805->regmap, bd71805->chip_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_FALLING, bdinfo->irq_base,
		&bd71805_irq_chip, &bd71805->irq_data);
	if (ret < 0)
		dev_warn(bd71805->dev, "Failed to add irq_chip %d\n", ret);
	return ret;
}

/** @brief bd71805 irq initialize 
 *  @param bd71805 bd71805 device to init
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71805_irq_exit(struct bd71805 *bd71805)
{
	if (bd71805->chip_irq > 0)
		regmap_del_irq_chip(bd71805->chip_irq, bd71805->irq_data);
	return 0;
}

/** @brief check whether volatile register 
 *  @param dev kernel device pointer
 *  @param reg register index
 */
static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	// struct bd71805 *bd71805 = dev_get_drvdata(dev);

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
	int r = 0;

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

	board_info->gpio_intr = of_get_named_gpio(np, "gpio_intr", 0);
        if (!gpio_is_valid(board_info->gpio_intr)) {
		dev_err(&client->dev, "no pmic intr pin available\n");
		goto err_intr;
        }

        r = of_property_read_u32(np, "irq_base", &prop);
        if (!r) {
		board_info->irq_base = prop;
        } else {
		board_info->irq_base = -1;
        }

	return board_info;

err_intr:
	devm_kfree(&client->dev, board_info);
	return NULL;
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

	bd71805_irq_init(bd71805, of_pmic_plat_data);

	ret = mfd_add_devices(bd71805->dev, -1,
			      bd71805_mfd_cells, ARRAY_SIZE(bd71805_mfd_cells),
			      NULL, 0,
			      regmap_irq_get_domain(bd71805->irq_data));
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

	bd71805_irq_exit(bd71805);
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
