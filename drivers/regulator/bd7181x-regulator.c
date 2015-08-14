/*
 * @file bd7181x-regulator.c RoHM BD71815/BD71817 regulator driver
 *
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 *
 * @author Tony Luo <luofc@embedinfo.com>
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
#include <linux/mfd/bd7181x.h>
#include <linux/regulator/of_regulator.h>

#define BD7181X_VOL_OFFSET		0
#define BD7181X_STANDBY_OFFSET		0

struct bd7181x_regulator {
	struct regulator_desc desc;
	unsigned char stby_reg;
	unsigned char stby_mask;
};

/** @brief bd7181x regulator type */
struct bd7181x_pmic {
	struct bd7181x_regulator descs[BD7181X_REGULATOR_CNT];	/**< regulator description to system */
	struct bd7181x *mfd;					/**< parent device */
	struct device *dev;					/**< regulator kernel device */
	struct regulator_dev *rdev[BD7181X_REGULATOR_CNT];	/**< regulator device of system */
};

static const int bd7181x_wled_currents[] = {
	// 0x00
	10, 20, 30, 50,
	70, 100, 200, 300,
	500, 700, 1000, 2000,
	3000, 4000, 5000, 6000,
	// 0x10
	7000, 8000, 9000, 10000,
	11000, 12000, 13000, 14000,
	15000, 16000, 17000, 18000,
	19000, 20000, 21000, 22000,
	// 0x20
	23000, 24000, 25000,
};

static int bd7181x_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay) {
	struct bd7181x_pmic *pmic = rdev_get_drvdata(rdev);
	struct bd7181x *mfd = pmic->mfd;
	// int id = rdev->desc->id;
	unsigned int ramp_bits;
	int ret;

	if (1 /*TODO*/ ) {
		ramp_delay = 12500 / ramp_delay;
		ramp_bits = (ramp_delay >> 1) - (ramp_delay >> 3);
		ret = regmap_update_bits(mfd->regmap,
					 rdev->desc->vsel_reg + 4,
					 0xc0, ramp_bits << 6);
		if (ret < 0)
			dev_err(pmic->dev, "ramp failed, err %d\n", ret);
	} else
		ret = -EACCES;

	return ret;
}

static int bd7181x_led_set_current_limit(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	struct bd7181x_pmic* pmic = rdev_get_drvdata(rdev);
	struct bd7181x* mfd = pmic->mfd;
	u8 addr;
	// int id = rdev_get_id(rdev);
	int i;

	addr = BD7181X_REG_LED_DIMM;

	for (i = ARRAY_SIZE(bd7181x_wled_currents) - 1 ; i >= 0; i--) {
		if (bd7181x_wled_currents[i] >= min_uA &&
			bd7181x_wled_currents[i] <= max_uA)
			return bd7181x_update_bits(mfd, addr, 0x3F, i);
	}

	return -EINVAL;
}

static int bd7181x_led_get_current_limit(struct regulator_dev *rdev)
{
	struct bd7181x_pmic* pmic = rdev_get_drvdata(rdev);
	struct bd7181x* mfd = pmic->mfd;
	// int id = rdev_get_id(rdev);
	u8 addr;
	int r;

	addr = BD7181X_REG_LED_DIMM;

	r = bd7181x_reg_read(mfd, addr);
	if (r < 0) {
		return r;
	}

	r = r & 0x3F;

	return (r < ARRAY_SIZE(bd7181x_wled_currents)) ?
			bd7181x_wled_currents[r] : -EINVAL;
}

static struct regulator_ops bd7181x_ldo_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops bd7181x_fixed_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
};

static struct regulator_ops bd7181x_buck_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd7181x_set_ramp_delay,
};

static struct regulator_ops bd7181x_led_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_ascend,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_current_limit = bd7181x_led_set_current_limit,
	.get_current_limit = bd7181x_led_get_current_limit,
};

#define BD7181X_FIXED_REG(_name, ereg, emsk, voltage)	\
	[BD7181X_ ## _name] = {	\
		.desc = {	\
			.name = #_name,	\
			.n_voltages = 1,	\
			.ops = &bd7181x_fixed_regulator_ops,	\
			.type = REGULATOR_VOLTAGE,	\
			.id = BD7181X_ ## _name,	\
			.owner = THIS_MODULE,	\
			.min_uV = (voltage),	\
			.enable_reg = (ereg),	\
			.enable_mask = (emsk),	\
		},	\
	}

#define BD7181X_BUCK_REG(_name, base, ereg, min, max, step)	\
	[BD7181X_ ## _name] = {	\
		.desc = {	\
			.name = #_name,\
			.n_voltages = ((max) - (min)) / (step) + 1,	\
			.ops = &bd7181x_buck_regulator_ops,	\
			.type = REGULATOR_VOLTAGE,	\
			.id = BD7181X_ ## _name,	\
			.owner = THIS_MODULE,	\
			.min_uV = (min),	\
			.uV_step = (step),	\
			.vsel_reg = (base) + BD7181X_VOL_OFFSET,	\
			.vsel_mask = 0x3f,	\
			.enable_reg = (ereg),	\
			.enable_mask = 0x04,	\
		},	\
		.stby_reg = (base) + BD7181X_STANDBY_OFFSET,	\
		.stby_mask = 0x3f,	\
	}

#define BD7181X_LED_REG(_name, base, mask, ereg, emsk, voltages)	\
	[BD7181X_ ## _name] = {	\
		.desc = {	\
			.name = #_name,	\
			.n_voltages = ARRAY_SIZE(voltages),	\
			.ops = &bd7181x_led_regulator_ops,	\
			.type = REGULATOR_CURRENT,	\
			.id = BD7181X_ ## _name,	\
			.owner = THIS_MODULE,	\
			.volt_table = voltages,	\
			.vsel_reg = (base),	\
			.vsel_mask = (mask),	\
			.enable_reg = (ereg),	\
			.enable_mask = (emsk),	\
		},	\
	}

#define BD7181X_LDO_REG(_name, base, ereg, emsk, min, max, step)	\
	[BD7181X_ ## _name] = {	\
		.desc = {	\
			.name = #_name,	\
			.n_voltages = ((max) - (min)) / (step) + 1,	\
			.ops = &bd7181x_ldo_regulator_ops,	\
			.type = REGULATOR_VOLTAGE,	\
			.id = BD7181X_ ## _name,	\
			.owner = THIS_MODULE,	\
			.min_uV = (min),	\
			.uV_step = (step),	\
			.vsel_reg = (base),	\
			.vsel_mask = 0x3f,	\
			.enable_reg = (ereg),	\
			.enable_mask = (emsk),	\
		},	\
		.stby_reg = (base),	\
		.stby_mask = 0x20,	\
	}

static struct bd7181x_regulator bd7181x_regulators[] = {
	BD7181X_BUCK_REG(BUCK1, BD7181X_REG_BUCK1_VOLT_H, BD7181X_REG_BUCK1_MODE, 800000, 2000000, 25000),
	BD7181X_BUCK_REG(BUCK2, BD7181X_REG_BUCK2_VOLT_H, BD7181X_REG_BUCK2_MODE, 800000, 2000000, 25000),
	BD7181X_BUCK_REG(BUCK3, BD7181X_REG_BUCK3_VOLT, BD7181X_REG_BUCK3_MODE,  1200000, 2700000, 50000),
	BD7181X_BUCK_REG(BUCK4, BD7181X_REG_BUCK4_VOLT, BD7181X_REG_BUCK4_MODE,  1100000, 1850000, 25000),
	BD7181X_BUCK_REG(BUCK5, BD7181X_REG_BUCK5_VOLT, BD7181X_REG_BUCK5_MODE,  1800000, 3300000, 50000),
	BD7181X_LDO_REG(LDO1, BD7181X_REG_LDO1_VOLT, BD7181X_REG_LDO_MODE1, 0x40, 800000, 3300000, 50000),
	BD7181X_LDO_REG(LDO2, BD7181X_REG_LDO2_VOLT, BD7181X_REG_LDO_MODE2, 0x04, 800000, 3300000, 50000),
	BD7181X_LDO_REG(LDO3, BD7181X_REG_LDO3_VOLT, BD7181X_REG_LDO_MODE2, 0x40, 800000, 3300000, 50000),
	BD7181X_LDO_REG(LDO4, BD7181X_REG_LDO4_VOLT, BD7181X_REG_LDO_MODE3, 0x04, 800000, 3300000, 50000),
	BD7181X_LDO_REG(LDO5, BD7181X_REG_LDO5_VOLT_H,BD7181X_REG_LDO_MODE3,0x40, 800000, 3300000, 50000),
	BD7181X_FIXED_REG(LDODVREF, BD7181X_REG_LDO_MODE4, 0x40, 3000000),
	BD7181X_FIXED_REG(LDOLPSR, BD7181X_REG_LDO_MODE4,  0x04, 1800000),
	BD7181X_LED_REG(WLED, BD7181X_REG_LED_DIMM, 0x3F, BD7181X_REG_LED_CTRL, 0x04, bd7181x_wled_currents),
};

#ifdef CONFIG_OF

static struct of_regulator_match bd7181x_matches[] = {
	{ .name = "buck1",	},
	{ .name = "buck2",	},
	{ .name = "buck3",	},
	{ .name = "buck4",	},
	{ .name = "buck5",	},
	{ .name = "ldo1",	},
	{ .name = "ldo2",	},
	{ .name = "ldo3",	},
	{ .name = "ldo4",	},
	{ .name = "ldo5",	},
	{ .name = "dvref",	},
	{ .name = "lpsr",	},
	{ .name = "wled",	},
};

/**@brief parse bd7181x regulator device tree
 * @param pdev platform device of bd7181x regulator
 * @param bd7181x_reg_matches return regualtor matches
 * @retval 0 parse success
 * @retval NULL parse fail
 */
static int bd7181x_parse_dt_reg_data(
		struct platform_device *pdev,
		struct of_regulator_match **reg_matches)
{
	// struct bd7181x *bd7181x = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np, *regulators;
	struct of_regulator_match *matches;
	int ret, count;

	np = of_node_get(pdev->dev.parent->of_node);
	regulators = of_find_node_by_name(np, "regulators");
	if (!regulators) {
		dev_err(&pdev->dev, "regulator node not found\n");
		return -EINVAL;
	}

	count = ARRAY_SIZE(bd7181x_matches);
	matches = bd7181x_matches;

	ret = of_regulator_match(&pdev->dev, regulators, matches, count);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error parsing regulator init data: %d\n",
			ret);
		return ret;
	}

	*reg_matches = matches;

	return 0;
}
#else
static inline int bd7181x_parse_dt_reg_data(
			struct platform_device *pdev,
			struct of_regulator_match **reg_matches)
{
	*reg_matches = NULL;
	return 0;
}
#endif

/** @brief out32k mode constants */
static const char* out32k_modes[] = {"open_drain", "cmos"};

/** @brief retrive out32k output mode */
static ssize_t show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd7181x_pmic *pmic = dev_get_drvdata(dev);
	int o;

	o = bd7181x_reg_read(pmic->mfd, BD7181X_REG_OUT32K);
	o = (o & OUT32K_MODE) != 0;

	return sprintf(buf, "%s\n", out32k_modes[o]);
}

/** @brief set out32k output mode */
static ssize_t set_mode(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd7181x_pmic *pmic = dev_get_drvdata(dev);
	int o, r;

	if (strncmp(buf, out32k_modes[0], strlen(out32k_modes[0])) == 0) {
		o = 0;
	} else {
		o = OUT32K_MODE;
	}

	r = bd7181x_update_bits(pmic->mfd, BD7181X_REG_OUT32K, OUT32K_MODE, o);
	if (r < 0) {
		return r;
	}
	return count;
}

/** @brief retrive out32k output value */
static ssize_t show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd7181x_pmic *pmic = dev_get_drvdata(dev);
	int o;

	o = bd7181x_reg_read(pmic->mfd, BD7181X_REG_OUT32K);
	o = (o & OUT32K_EN) != 0;

	return sprintf(buf, "%d\n", o);
}

/** @brief set o output value */
static ssize_t set_value(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd7181x_pmic *pmic = dev_get_drvdata(dev);
	int o, r;

	if (sscanf(buf, "%d", &o) < 1) {
		return -EINVAL;
	}

	if (o != 0) {
		o = OUT32K_EN;
	}
	r = bd7181x_update_bits(pmic->mfd, BD7181X_REG_OUT32K, OUT32K_EN, o);
	if (r < 0) {
		return r;
	}
	return count;
}

/** @brief list all supported modes */
static ssize_t available_modes(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, r;

	r = 0;
	for (i = 0; i < ARRAY_SIZE(out32k_modes) && r >= 0; i++) {
		r += sprintf(buf + r, "%s ", out32k_modes[i]);
	}
	r += sprintf(buf + r, "\n");

	return r;
}

/** @brief list all supported values */
static ssize_t available_values(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0 1 \n");
}

static DEVICE_ATTR(out32k_mode, 0666, show_mode, set_mode);
static DEVICE_ATTR(out32k_value, 0666, show_value, set_value);
static DEVICE_ATTR(available_mode, 0444, available_modes, NULL);
static DEVICE_ATTR(available_value, 0444, available_values, NULL);

/** @brief device sysfs attribute table, about o */
static struct attribute *gpo_attributes[] = {
	&dev_attr_out32k_mode.attr,
	&dev_attr_out32k_value.attr,
	&dev_attr_available_mode.attr,
	&dev_attr_available_value.attr,
	NULL
};

static const struct attribute_group gpo_attr_group = {
	.attrs	= gpo_attributes,
};


/**@brief probe bd7181x regulator device
 @param pdev bd7181x regulator platform device
 @retval 0 success
 @retval negative fail
*/
static __init int bd7181x_probe(struct platform_device *pdev)
{
	struct bd7181x_pmic *pmic;
	struct bd7181x_board *pdata;
	struct regulator_config config = {};
	struct bd7181x *bd7181x = dev_get_drvdata(pdev->dev.parent);
	struct of_regulator_match *matches = NULL;
	int i, err;

	pmic = kzalloc(sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		dev_err(&pdev->dev, "Memory allocation failed for pmic\n");
		return -ENOMEM;
	}

	memcpy(pmic->descs, bd7181x_regulators,	sizeof(pmic->descs));

	pmic->dev = &pdev->dev;
	pmic->mfd = bd7181x;
	platform_set_drvdata(pdev, pmic);

	pdata = dev_get_platdata(bd7181x->dev);
	if (!pdata && bd7181x->dev->of_node) {
		bd7181x_parse_dt_reg_data(pdev,	&matches);
		if (matches == NULL) {
			dev_err(&pdev->dev, "Platform data not found\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < BD7181X_REGULATOR_CNT; i++) {
		struct regulator_init_data *init_data;
		struct regulator_desc *desc;
		struct regulator_dev *rdev;

		desc = &pmic->descs[i].desc;
		desc->name = bd7181x_matches[i].name;
		
		if (pdata) {
			init_data = pdata->init_data[i];
		} else {
			init_data = matches[i].init_data;
		}

		config.dev = pmic->dev;
		config.init_data = init_data;
		config.driver_data = pmic;
		config.regmap = bd7181x->regmap;
		config.of_node = matches[i].of_node;

		rdev = regulator_register(desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(bd7181x->dev,
				"failed to register %s regulator\n",
				desc->name);
			err = PTR_ERR(rdev);
			goto err;
		}
		pmic->rdev[i] = rdev;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &gpo_attr_group);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to create attribute group: %d\n", err);
		goto err;
	}

	return 0;

err:
	while (--i >= 0)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return err;
}

/**@brief remove bd7181x regulator device
 @param pdev bd7181x regulator platform device
 @return 0
*/
static int __exit bd7181x_remove(struct platform_device *pdev)
{
	struct bd7181x_pmic *pmic = platform_get_drvdata(pdev);
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &gpo_attr_group);

	for (i = 0; i < BD7181X_REGULATOR_CNT; i++)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return 0;
}

static struct platform_driver bd7181x_driver = {
	.driver = {
		.name = "bd7181x-pmic",
		.owner = THIS_MODULE,
	},
	.probe = bd7181x_probe,
	.remove = bd7181x_remove,
};

/**@brief module initialize function */
static int __init bd7181x_init(void)
{
	return platform_driver_register(&bd7181x_driver);
}
subsys_initcall(bd7181x_init);

/**@brief module deinitialize function */
static void __exit bd7181x_cleanup(void)
{
	platform_driver_unregister(&bd7181x_driver);
}
module_exit(bd7181x_cleanup);

MODULE_AUTHOR("Tony Luo <luofc@embedinfo.com>");
MODULE_DESCRIPTION("BD71815/BD71817 voltage regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bd7181x-pmic");
