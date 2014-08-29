/*
 * bd71805.c  --  RoHM bd71805
 *
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 *
 * Author: Tony Luo <luofc@embedinfo.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mfd/bd71805.h>
#include <linux/regulator/of_regulator.h>


/* supported BUCK1 voltages in milivolts */
static const u16 BUCK1_VSEL_table[] = {
	800, 825, 850, 875, 900, 925, 950, 975, 
	1000, 1025, 1050, 1075, 1100, 1125, 1150, 1175,
	1200, 1225, 1250, 1275, 1300, 1325, 1350, 1375,
	1400, 1425, 1450, 1475, 1500, 1525, 1550, 1575,
	1600, 1625, 1650, 1675, 1700, 1725, 1750, 1775,
	1800, 1825, 1850, 1875, 1900, 1925, 1950, 1975,
	2000,
};

/* supported BUCK2 voltages in milivolts */
static const u16 BUCK2_VSEL_table[] = {
	800, 825, 850, 875, 900, 925, 950, 975,
	1000, 1025, 1050, 1075, 1100, 1125, 1150, 1175,
	1200, 1225, 1250, 1275, 1300, 1325, 1350, 1375,
	1400, 1425, 1450, 1475, 1500, 1525, 1550, 1575,
	1600, 1625, 1650, 1675, 1700, 1725, 1750, 1775,
	1800, 1825, 1850, 1875, 1900, 1925, 1950, 1975,
	2000,
};

/* supported BUCK3 voltages in milivolts */
static const u16 BUCK3_VSEL_table[] = {
	2600, 2650, 2700, 2750, 2800, 2850, 2900, 2950,
	3000, 3050, 3100, 3150, 3200, 3250, 3300, 3350,
};

/* supported BUCK4 voltages in milivolts */
static const u16 BUCK4_VSEL_table[] = {
	1000, 1050, 1100, 1150, 1200, 1250, 1300, 1350,
	1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750,
	1800, 1850, 1900, 1950, 2000, 2050, 2100, 2150,
	2200, 2250, 2300, 2350, 2400, 2450, 2500, 2550,
	2600, 2650, 2700,
};

/* supported LDO1 voltages in milivolts */
static const u16 LDO1_VSEL_table[] = {
	800, 850, 900, 950, 1000, 1050, 1100, 1150,
	1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550,
	1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950,
	2000, 2050, 2100, 2150, 2200, 2250, 2300, 2350,
	2400, 2450, 2500, 2550, 2600, 2650, 2700, 2750,
	2800, 2850, 2900, 2950, 3000, 3050, 3100, 3150,
	3200, 3250, 3300,
};

/* supported LDO2 voltages in milivolts */
static const u16 LDO2_VSEL_table[] = {
	800, 850, 900, 950, 1000, 1050, 1100, 1150,
	1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550,
	1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950,
	2000, 2050, 2100, 2150, 2200, 2250, 2300, 2350,
	2400, 2450, 2500, 2550, 2600, 2650, 2700, 2750,
	2800, 2850, 2900, 2950, 3000, 3050, 3100, 3150,
	3200, 3250, 3300,
};

/* supported LDO2 voltages in milivolts */
static const u16 LDO3_VSEL_table[] = {
	800, 850, 900, 950, 1000, 1050, 1100, 1150,
	1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550,
	1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950,
	2000, 2050, 2100, 2150, 2200, 2250, 2300, 2350,
	2400, 2450, 2500, 2550, 2600, 2650, 2700, 2750,
	2800, 2850, 2900, 2950, 3000, 3050, 3100, 3150,
	3200, 3250, 3300,
};

struct bd71805_info {
	const char *name;
	unsigned min_uV;
	unsigned max_uV;
	u8 table_len;
	const u16 *table;
};

static struct bd71805_info bd71805_regs[] = {
	{
		.name = "buck1",
		.min_uV	= 800000,
		.max_uV	= 2000000,
		.table_len = ARRAY_SIZE(BUCK1_VSEL_table),
		.table = BUCK1_VSEL_table,
	},
	{
		.name = "buck2",
		.min_uV = 800000,
		.max_uV = 2000000,
		.table_len = ARRAY_SIZE(BUCK2_VSEL_table),
		.table = BUCK2_VSEL_table,
	},
	{
		.name = "buck3",
		.min_uV = 2600000,
		.max_uV = 3350000,
		.table_len = ARRAY_SIZE(BUCK3_VSEL_table),
		.table = BUCK3_VSEL_table,
	},
	{
		.name = "buck4",
		.min_uV = 1000000,
		.max_uV = 2700000,
		.table_len = ARRAY_SIZE(BUCK4_VSEL_table),
		.table = BUCK4_VSEL_table,
	},
	{
		.name = "ldo1",
		.min_uV = 800000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO1_VSEL_table),
		.table = LDO1_VSEL_table,
	},
	{
		.name = "ldo2",
		.min_uV = 800000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO2_VSEL_table),
		.table = LDO2_VSEL_table,
	},
	{
		.name = "ldo3",
		.min_uV = 800000,
		.max_uV = 3300000,
		.table_len = ARRAY_SIZE(LDO3_VSEL_table),
		.table = LDO3_VSEL_table,
	},
	{
		.name = "vodvref",
		.min_uV = 500000,
		.max_uV = 1350000,
	},
	{
		.name = "snvs",
		.min_uV = 3000000,
		.max_uV = 3000000,
	},
};

struct bd71805_pmic {
	struct regulator_desc desc[BD71805_NUM_REGULATOR];
	struct bd71805 *mfd;
	struct regulator_dev *rdev[BD71805_NUM_REGULATOR];
	struct bd71805_info *info[BD71805_NUM_REGULATOR];
	struct mutex mutex;
	int mode;
	int  (*get_ctrl_reg)(int);
};

static int bd71805_get_ctrl_register(int id)
{
	switch (id) {
	case BD71805_BUCK1:
		return BD71805_REG_BUCK1_ON;
	case BD71805_BUCK2:
		return BD71805_REG_BUCK2_ON;
	case BD71805_BUCK3:
		return BD71805_REG_BUCK3_VOLT;
	case BD71805_BUCK4:
		return BD71805_REG_BUCK4_VOLT;
	case BD71805_LDO1:
	case BD71805_LDO2:
	case BD71805_LDO3:
		return BD71805_REG_LDO1_CTRL;
#if 0
	case BD71805_VODVREF:
		return BD71805_REG_VODVREF;
	case BD71805_VOSNVS:
		return BD71805_REG_VOSNVS;
#endif
	default:
		return -EINVAL;
	}
}

static int bd71805_modify_bits(struct bd71805_pmic *pmic, u8 reg,
					u8 set_mask, u8 clear_mask)
{
	struct bd71805 *mfd = pmic->mfd;
	int err, data;

	printk("----bd71805_modify_bits: reg= 0x%x, set_mask= 0x%x, clear_mask= 0x%x\n",
				reg, set_mask, clear_mask);

	mutex_lock(&pmic->mutex);

	data = bd71805_reg_read(mfd, reg);
	if (data < 0) {
		dev_err(pmic->mfd->dev, "Read from reg 0x%x failed\n", reg);
		err = data;
		goto out;
	}

	data &= ~clear_mask;
	data |= set_mask;
	err = bd71805_reg_write(mfd, reg, data);
	if (err)
		dev_err(pmic->mfd->dev, "Write for reg 0x%x failed\n", reg);

out:
	mutex_unlock(&pmic->mutex);
	return err;
}

static int bd71805_is_enabled_ldo(struct regulator_dev *dev)
{
	struct bd71805_pmic *pmic = rdev_get_drvdata(dev);
	struct bd71805 *mfd = pmic->mfd;
	int reg, value, id = rdev_get_id(dev);

	reg = pmic->get_ctrl_reg(id);
	if (reg < 0)
		return reg;

	value = bd71805_reg_read(mfd, reg);
	if (value < 0)
		return value;

	switch(id) {
	case BD71805_LDO1:
		return value & LDO1_EN;
	case BD71805_LDO2:
		return value & LDO2_EN;
	case BD71805_LDO3:
		return value & LDO3_EN;
	default:
		return -EINVAL;
	}
}

static int bd71805_enable_ldo(struct regulator_dev *dev)
{
	struct bd71805_pmic *pmic = rdev_get_drvdata(dev);
	struct bd71805 *mfd = pmic->mfd;
	int reg, id = rdev_get_id(dev);

	printk("----bd71805_enable_ldo %d\n", id);

	reg = pmic->get_ctrl_reg(id);
	if (reg < 0)
		return reg;

	switch(id) {
	case BD71805_LDO1:
		return bd71805_set_bits(mfd, reg, LDO1_EN);
	case BD71805_LDO2:
		return bd71805_set_bits(mfd, reg, LDO2_EN);
	case BD71805_LDO3:
		return bd71805_set_bits(mfd, reg, LDO3_EN);
	default:
		return -EINVAL;
	}
}

static int bd71805_disable_ldo(struct regulator_dev *dev)
{
	struct bd71805_pmic *pmic = rdev_get_drvdata(dev);
	struct bd71805 *mfd = pmic->mfd;
	int reg, id = rdev_get_id(dev);

	printk("----bd71805_disable_ldo %d\n", id);

	reg = pmic->get_ctrl_reg(id);
	if (reg < 0)
		return reg;

	switch(id) {
	case BD71805_LDO1:
		return bd71805_clear_bits(mfd, reg, LDO1_EN);
	case BD71805_LDO2:
		return bd71805_clear_bits(mfd, reg, LDO2_EN);
	case BD71805_LDO3:
		return bd71805_clear_bits(mfd, reg, LDO3_EN);
	default:
		return -EINVAL;
	}
}

static int bd71805_get_voltage(struct regulator_dev *dev)
{
	struct bd71805_pmic *pmic = rdev_get_drvdata(dev);
	struct bd71805 *mfd = pmic->mfd;
	int reg, value, id = rdev_get_id(dev), voltage = 0;

	reg = pmic->get_ctrl_reg(id);
	if (reg < 0)
		return reg;

	switch(id) {
	case BD71805_LDO3:
		++reg;
	case BD71805_LDO2:
		++reg;
	case BD71805_LDO1:
		reg += 2;
		break;
	}

	value = bd71805_reg_read(mfd, reg);
	if (value < 0)
		return value;

	value &= VOLT_MASK;
	voltage = pmic->info[id]->table[value] * 1000;

	return voltage;
}

static int bd71805_set_voltage(struct regulator_dev *dev, unsigned selector)
{
	struct bd71805_pmic *pmic = rdev_get_drvdata(dev);
	int reg, id = rdev_get_id(dev);

	reg = pmic->get_ctrl_reg(id);
	if (reg < 0)
		return reg;

	switch(id) {
	case BD71805_LDO3:
		++reg;
	case BD71805_LDO2:
		++reg;
	case BD71805_LDO1:
		reg += 2;
		break;
	}

	return bd71805_modify_bits(pmic, reg, selector, VOLT_MASK);
}

static int bd71805_list_voltage(struct regulator_dev *dev,
					unsigned selector)
{
	struct bd71805_pmic *pmic = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev), voltage;

	if (id < BD71805_BUCK1 || id > BD71805_VOSNVS)
		return -EINVAL;

	if (selector >= pmic->info[id]->table_len)
		return -EINVAL;
	else
		voltage = pmic->info[id]->table[selector] * 1000;

	return voltage;
}

/* Regulator ops */
static struct regulator_ops bd71805_ops_ldo = {
	.is_enabled		= bd71805_is_enabled_ldo,
	.enable			= bd71805_enable_ldo,
	.disable		= bd71805_disable_ldo,
	.get_voltage		= bd71805_get_voltage,
	.set_voltage_sel	= bd71805_set_voltage,
	.list_voltage		= bd71805_list_voltage,
};

static struct regulator_ops bd71805_ops = {
	.get_voltage		= bd71805_get_voltage,
	.set_voltage_sel	= bd71805_set_voltage,
	.list_voltage		= bd71805_list_voltage,
};

#ifdef CONFIG_OF

static struct of_regulator_match bd71805_matches[] = {
	{ .name = "buck1",	.driver_data = (void *) &bd71805_regs[0] },
	{ .name = "buck2",	.driver_data = (void *) &bd71805_regs[1] },
	{ .name = "buck3",	.driver_data = (void *) &bd71805_regs[2] },
	{ .name = "buck4",	.driver_data = (void *) &bd71805_regs[3] },
	{ .name = "ldo1",	.driver_data = (void *) &bd71805_regs[4] },
	{ .name = "ldo2",	.driver_data = (void *) &bd71805_regs[5] },
	{ .name = "ldo3",	.driver_data = (void *) &bd71805_regs[6] },
	{ .name = "vodvref",	.driver_data = (void *) &bd71805_regs[7] },
	{ .name = "snvs",	.driver_data = (void *) &bd71805_regs[8] },
};

static struct bd71805_board *bd71805_parse_dt_reg_data(
		struct platform_device *pdev,
		struct of_regulator_match **bd71805_reg_matches)
{
	struct bd71805_board *pmic_plat_data;
	struct bd71805 *bd71805 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np, *regulators;
	struct of_regulator_match *matches;
	int idx = 0, ret, count;

	pmic_plat_data = devm_kzalloc(&pdev->dev, sizeof(*pmic_plat_data),
					GFP_KERNEL);

	if (!pmic_plat_data) {
		dev_err(&pdev->dev, "Failure to alloc pdata for regulators.\n");
		return NULL;
	}

	np = of_node_get(pdev->dev.parent->of_node);
	regulators = of_find_node_by_name(np, "regulators");
	if (!regulators) {
		dev_err(&pdev->dev, "regulator node not found\n");
		return NULL;
	}

	count = ARRAY_SIZE(bd71805_matches);
	matches = bd71805_matches;

	ret = of_regulator_match(&pdev->dev, regulators, matches, count);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error parsing regulator init data: %d\n",
			ret);
		return NULL;
	}

	*bd71805_reg_matches = matches;

	for (idx = 0; idx < count; idx++) {
		if (!matches[idx].init_data || !matches[idx].of_node)
			continue;

		pmic_plat_data->bd71805_pmic_init_data[idx] = matches[idx].init_data;
	}

	return pmic_plat_data;
}
#else
static inline struct bd71805_board *bd71805_parse_dt_reg_data(
			struct platform_device *pdev,
			struct of_regulator_match **bd71805_reg_matches)
{
	*bd71805_reg_matches = NULL;
	return NULL;
}
#endif

static __init int bd71805_probe(struct platform_device *pdev)
{
	struct bd71805 *bd71805 = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = {};
	struct bd71805_info *info;
	struct regulator_init_data *reg_data;
	struct regulator_dev *rdev;
	struct bd71805_pmic *pmic;
	struct bd71805_board *pmic_plat_data;
	struct of_regulator_match *bd71805_reg_matches = NULL;
	int i, err;

	pmic_plat_data = dev_get_platdata(bd71805->dev);
	if (!pmic_plat_data && bd71805->dev->of_node) {
		pmic_plat_data = bd71805_parse_dt_reg_data(pdev,
						&bd71805_reg_matches);
	}
	
	if (!pmic_plat_data) {
		dev_err(&pdev->dev, "Platform data not found\n");
		return -EINVAL;
	}

	pmic = kzalloc(sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		dev_err(&pdev->dev, "Memory allocation failed for pmic\n");
		return -ENOMEM;
	}

	mutex_init(&pmic->mutex);
	pmic->mfd = bd71805;
	platform_set_drvdata(pdev, pmic);

	pmic->get_ctrl_reg = &bd71805_get_ctrl_register;
	info = bd71805_regs;

	for (i = 0; i < BD71805_NUM_REGULATOR; i++, info++) {
		
		reg_data = pmic_plat_data->bd71805_pmic_init_data[i];
		if(!reg_data)
			continue;

		/* Register the regulators */
		pmic->info[i] = info;

		pmic->desc[i].name = info->name;
		pmic->desc[i].supply_name = info->name;
		pmic->desc[i].id = i;
		pmic->desc[i].n_voltages = info->table_len;
		if (i == BD71805_LDO1 || i == BD71805_LDO2 || i == BD71805_LDO3) { 
			pmic->desc[i].ops = &bd71805_ops_ldo;
		} else {
			pmic->desc[i].ops = &bd71805_ops;
		}
		pmic->desc[i].type = REGULATOR_VOLTAGE;
		pmic->desc[i].owner = THIS_MODULE;

		reg_data->constraints.min_uV = info->min_uV;
		reg_data->constraints.max_uV = info->max_uV;

		config.dev = bd71805->dev;
		config.init_data = reg_data;
		config.driver_data = pmic;
		config.regmap = bd71805->regmap;

		if (bd71805_reg_matches) {
			config.of_node = bd71805_reg_matches[i].of_node;
		}

		rdev = regulator_register(&pmic->desc[i], &config);
		if (IS_ERR(rdev)) {
			dev_err(bd71805->dev,
				"failed to register %s regulator\n",
				pdev->name);
			err = PTR_ERR(rdev);
			goto err;
		}

		/* Save regulator for cleanup */
		pmic->rdev[i] = rdev;
	}
	return 0;

err:
	while (--i >= 0)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return err;
}

static int __exit bd71805_remove(struct platform_device *pdev)
{
	struct bd71805_pmic *pmic = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < BD71805_NUM_REGULATOR; i++)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return 0;
}

static struct platform_driver bd71805_driver = {
	.driver = {
		.name = "bd71805-pmic",
		.owner = THIS_MODULE,
	},
	.probe = bd71805_probe,
	.remove = bd71805_remove,
};

static int __init bd71805_init(void)
{
	return platform_driver_register(&bd71805_driver);
}
subsys_initcall(bd71805_init);

static void __exit bd71805_cleanup(void)
{
	platform_driver_unregister(&bd71805_driver);
}
module_exit(bd71805_cleanup);

MODULE_AUTHOR("Tony Luo <luofc@embedinfo.com>");
MODULE_DESCRIPTION("BD71805MWV voltage regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bd71805-pmic");
