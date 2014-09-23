/*
 * bd71805-power.c
 * @file ROHM BD71805MWV Charger driver
 *
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/mfd/bd71805.h>

#define JITTER_DEFAULT		50		/* hope 50ms is enough */
#define JITTER_REPORT_CAP	30000		/* 30 seconds */

/** @brief power deivce */
struct bd71805_power {
	struct device *dev;
	struct bd71805 *mfd;			/**< parent for access register */
	struct power_supply ac;			/**< alternating current power */
	struct power_supply bat;		/**< battery power */
	struct delayed_work bd_work;		/**< delayed work for timed work */
	int    reg_index;			/**< register address saved for sysfs */
	int    vbus_status;			/**< last vbus status */
	int    bat_status;			/**< last bat status */
	int    charge_status;			/**< last charge status */
};

/** @brief read a register group once
 *  @param power power device
 *  @param reg	 register address of lower register
 *  @return register value
 */
static u16 bd71805_reg_read16(struct bd71805_power *power, int reg) {
	u16 v;

	v = (u16)bd71805_reg_read(power->mfd, reg) << 8;
	v |= (u16)bd71805_reg_read(power->mfd, reg + 1) << 0;
	return v;
}

/** @brief get property of power supply ac
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */
static int bd71805_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd71805_power *power = dev_get_drvdata(psy->dev->parent);
	u32 vot;
	u8 ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = bd71805_reg_read(power->mfd, BD71805_REG_VBUS_STAT);
		val->intval = ret & VBUS_DET;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd71805_reg_read16(power, BD71805_REG_VM_VBUS_U);
		val->intval = 5 * vot;		// 5 milli volt steps
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief get property of power supply bat
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */

static int bd71805_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd71805_power *power = dev_get_drvdata(psy->dev->parent);
	u32 cap;
	u32 vot;
	u32 r;
	u8 ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		r = bd71805_reg_read(power->mfd, BD71805_REG_CHG_STATE);
		printk("CHG_STATE = 0x%.2X\n", r);
		switch(r) {
		case CHG_STATE_SUSPEND:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case CHG_STATE_TRICKLE_CHARGE:
		case CHG_STATE_PRE_CHARGE:
		case CHG_STATE_FAST_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case CHG_STATE_TOP_OFF:
		case CHG_STATE_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		do {
			ret = bd71805_reg_read(power->mfd, BD71805_REG_BAT_STAT);
		} while (!(ret & BAT_DET_DONE));
		val->intval = (ret & BAT_DET) >> BAT_DET_OFFSET;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bd71805_reg_read(power->mfd, BD71805_REG_BAT_STAT);
		if (ret & DBAT_DET)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else if (ret & VBAT_OV)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		cap = bd71805_reg_read16(power, BD71805_REG_CC_BATCAP_U);
		printk("CC_BATCAP = 0x%.4X\n", cap);
		val->intval = cap * 100 / 0x1FFF;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd71805_reg_read16(power, BD71805_REG_VM_VBAT_U);
		val->intval = vot;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**@brief timed work function called by system
 *  read battery capacity,
 *  sense change of charge status, etc.
 * @param work work struct
 * @return  void
 */

static void bd_work_callback(struct work_struct *work)
{
	struct bd71805_power *power;
	struct delayed_work *delayed_work;
	int status, changed = 0;
	static int cap_counter = 0;

	delayed_work = container_of(work, struct delayed_work, work);
	power = container_of(delayed_work, struct bd71805_power, bd_work);

	status = bd71805_reg_read(power->mfd, BD71805_REG_VBUS_STAT);
	if (status != power->vbus_status) {
		printk("VBUS_STAT CHANGED from 0x%X to 0x%X\n", power->vbus_status, status);
		power->vbus_status = status;
		changed = 1;
	}

	status = bd71805_reg_read(power->mfd, BD71805_REG_BAT_STAT);
	status &= ~BAT_DET_DONE;
	if (status != power->bat_status) {
		printk("BAT_STAT CHANGED from 0x%X to 0x%X\n", power->bat_status, status);
		power->bat_status = status;
		changed = 1;
	}

	status = bd71805_reg_read(power->mfd, BD71805_REG_CHG_STATE);
	if (status != power->charge_status) {
		printk("CHG_STATE CHANGED from 0x%X to 0x%X\n", power->charge_status, status);
		power->charge_status = status;
		changed = 1;
	}

	if (changed || cap_counter++ > JITTER_REPORT_CAP / JITTER_DEFAULT) {
		power_supply_changed(&power->ac);
		power_supply_changed(&power->bat);
		cap_counter = 0;
	}

	schedule_delayed_work(&power->bd_work, msecs_to_jiffies(JITTER_DEFAULT));
}

/** @brief ac properties */
static enum power_supply_property bd71805_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/** @brief bat properies */
static enum power_supply_property bd71805_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/** @brief directly set raw value to chip register, format: 'register value' */
static ssize_t bd71805_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71805_power *power = container_of(psy, struct bd71805_power, bat);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		power->reg_index = -1;
		return count;
	}

	if (ret == 1 && reg <= BD71805_MAX_REGISTER) {
		power->reg_index = reg;
		return count;
	}
	if (reg > BD71805_MAX_REGISTER || val > 255)
		return -EINVAL;

	ret = bd71805_reg_write(power->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/** @brief print value of chip register, format: 'register=value' */
static ssize_t bd71805_sysfs_print_reg(struct bd71805_power *power,
				       u8 reg,
				       char *buf)
{
	int ret = bd71805_reg_read(power->mfd, reg);

	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/** @brief show all raw values of chip register, format per line: 'register=value' */
static ssize_t bd71805_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71805_power *power = container_of(psy, struct bd71805_power, bat);
	ssize_t ret = 0;
	int i;

	if (power->reg_index > 0) {
		ret += bd71805_sysfs_print_reg(power, power->reg_index, buf + ret);
	} else {
		for (i = 0; i <= BD71805_MAX_REGISTER; i++) {
			ret += bd71805_sysfs_print_reg(power, i, buf + ret);
		}
	}
	return ret;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bd71805_sysfs_show_registers, bd71805_sysfs_set_registers);

static struct attribute *bd71805_sysfs_attributes[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use power supply class props.
	 */
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bd71805_sysfs_attr_group = {
	.attrs = bd71805_sysfs_attributes,
};


/** @brief probe power device 
 * @param pdev platform deivce of bd71805_power
 * @retval 0 success
 * @retval negative fail
 */
static int __init bd71805_power_probe(struct platform_device *pdev)
{
	struct bd71805 *bd71805 = dev_get_drvdata(pdev->dev.parent);
	struct bd71805_power *power;
	int ret;

	power = kzalloc(sizeof(*power), GFP_KERNEL);
	if (power == NULL)
		return -ENOMEM;

	power->dev = &pdev->dev;
	power->mfd = bd71805;

	platform_set_drvdata(pdev, power);

	power->ac.name = "bd71805_ac";
	power->ac.type = POWER_SUPPLY_TYPE_MAINS;
	power->ac.properties = bd71805_charger_props;
	power->ac.num_properties = ARRAY_SIZE(bd71805_charger_props);
	power->ac.get_property = bd71805_charger_get_property;

	ret = power_supply_register(&pdev->dev, &power->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ac: %d\n", ret);
		goto fail_register_ac;
	}

	power->bat.name = "bd71805_bat";
	power->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	power->bat.properties = bd71805_battery_props;
	power->bat.num_properties = ARRAY_SIZE(bd71805_battery_props);
	power->bat.get_property = bd71805_battery_get_property;

	ret = power_supply_register(&pdev->dev, &power->bat);
	if (ret) {
		dev_err(&pdev->dev, "failed to register usb: %d\n", ret);
		goto fail_register_bat;
	}

	/*
	// Enable to control charging voltage/current by thermal condition
	ret = bd71805_reg_read(power->mfd, BD71805_REG_CHG_SET1);
	ret &= ~0x6;
	bd71805_reg_write(power->mfd, BD71805_REG_CHG_SET1, ret);
	ret = bd71805_reg_read(power->mfd, BD71805_REG_CHG_SET1);
	printk("CHG_SET1 = 0x%x\n", ret);

	bd71805_reg_write(power->mfd, BD71805_REG_VM_BTMP, 80);

	// disable temperature monitor
	bd71805_reg_write(power->mfd, BD71805_REG_BAT_TEMP, 0x06);
	ret = bd71805_reg_read(power->mfd, BD71805_REG_BAT_TEMP);
	printk("BAT_TEMP = 0x%x\n", ret);
	*/

	ret = sysfs_create_group(&power->bat.dev->kobj, &bd71805_sysfs_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register sysfs interface\n");
	}

	power->reg_index = -1;

	INIT_DELAYED_WORK(&power->bd_work, bd_work_callback);

	/* Schedule timer to check current status */
	schedule_delayed_work(&power->bd_work, msecs_to_jiffies(0));

	return 0;

      //error_exit:
	power_supply_unregister(&power->bat);
      fail_register_bat:
	power_supply_unregister(&power->ac);
      fail_register_ac:
	platform_set_drvdata(pdev, NULL);
	kfree(power);

	return ret;
}

/** @brief remove power device
 * @param pdev platform deivce of bd71805_power
 * @return 0
 */

static int __exit bd71805_power_remove(struct platform_device *pdev)
{
	struct bd71805_power *power = platform_get_drvdata(pdev);

	sysfs_remove_group(&power->bat.dev->kobj, &bd71805_sysfs_attr_group);

	cancel_delayed_work(&power->bd_work);

	power_supply_unregister(&power->bat);
	power_supply_unregister(&power->ac);
	platform_set_drvdata(pdev, NULL);
	kfree(power);

	return 0;
}

static struct platform_driver bd71805_power_driver = {
	.driver = {
		   .name = "bd71805-power",
		   .owner = THIS_MODULE,
		   },
	.remove = __exit_p(bd71805_power_remove),
};

/** @brief module initialize function */
static int __init bd71805_power_init(void)
{
	return platform_driver_probe(&bd71805_power_driver, bd71805_power_probe);
}

module_init(bd71805_power_init);

/** @brief module deinitialize function */
static void __exit bd71805_power_exit(void)
{
	platform_driver_unregister(&bd71805_power_driver);
}

module_exit(bd71805_power_exit);

MODULE_AUTHOR("Tony Luo <luofc@embest-tech.com>");
MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_DESCRIPTION("BD71805MWV Battery Charger Power driver");
MODULE_LICENSE("GPL");
