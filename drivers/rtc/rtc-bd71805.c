/*
 * @file RoHM BD71805 Real Time Clock interface
 *
 * Copyright (C) 2014 Embest Technology Co. Ltd. Inc.
 *
 * @author Peter Yang <yanglsh@embest-tech.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/mfd/bd71805.h>

/** @brief bd71805 rtc struct */
struct bd71805_rtc {
	struct rtc_device	*rtc;		/**< system rtc device */
	int irq;				/**< rtc irq */
};

static int bd71805_regmap_read(struct bd71805 *mfd, u8 reg, int* val_p) {
	int r;

	r = bd71805_reg_read(mfd, reg);
	if (r < 0) {
		return r;
	}
	if (val_p) {
		*val_p = r;
	}
	return 0;
}

/* @brief Total number of RTC registers needed to set time*/
// #define NUM_TIME_REGS	(BD71805_REG_YEAR - BD71805_REG_SEC + 1)

/**@brief enable or disable rtc alarm irq
 * @param dev rtc device of system
 * @param enabled enable if non-zero
 * @retval 0 success
 * @retval negative error number
 */
static int bd71805_rtc_alarm_irq_enable(struct device *dev, unsigned enabled)
{
	struct bd71805 *mfd = dev_get_drvdata(dev->parent);
	u8 val = 0;

	if (enabled)
		val = ALM0;

	return bd71805_reg_write(mfd, BD71805_REG_INT_EN_12, val);
}

/**@brief bd71805 rtc time convert to linux time
 * @param tm linux rtc time
 * @param hw_rtc bd71805 rtc time
 * @return argument tm
 */
static struct rtc_time* hw_to_rtc_time(struct rtc_time* tm, const struct bd71805_rtc_alarm* hw_rtc) {
	u8 hour;

	tm->tm_sec = bcd2bin(hw_rtc->sec);
	tm->tm_min = bcd2bin(hw_rtc->min);
	hour = hw_rtc->hour & ~HOUR_24HOUR;
	tm->tm_hour = bcd2bin(hour);
	tm->tm_mday = bcd2bin(hw_rtc->day);
	tm->tm_mon = bcd2bin(hw_rtc->month) - 1;
	tm->tm_year = bcd2bin(hw_rtc->year) + 100;
	return tm;
}

/**@brief linux time convert bd71805 rtc time
 * @param hw_rtc bd71805 rtc time
 * @param tm linux rtc time
 * @return argument hw_rtc
 */
static struct bd71805_rtc_alarm* rtc_time_to_hw(struct bd71805_rtc_alarm* hw_rtc, const struct rtc_time* tm) {
	hw_rtc->sec = bin2bcd(tm->tm_sec);
	hw_rtc->min = bin2bcd(tm->tm_min);
	hw_rtc->hour = HOUR_24HOUR | bin2bcd(tm->tm_hour);
	hw_rtc->day = bin2bcd(tm->tm_mday);
	hw_rtc->month = bin2bcd(tm->tm_mon + 1);
	hw_rtc->year = bin2bcd(tm->tm_year - 100);

	return hw_rtc;
}

/*
 * Gets current bd71805 RTC time and date parameters.
 *
 * The RTC's time/alarm representation is not what gmtime(3) requires
 * Linux to use:
 *
 *  - Months are 1..12 vs Linux 0-11
 *  - Years are 0..99 vs Linux 1900..N (we assume 21st century)
 */
/**@brief read date/time from bd71805 rtc
 * @param dev rtc device of system
 * @param tm date/time store target
 * @retval 0 success
 * @retval negative error number
 */
static int bd71805_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct bd71805_rtc_alarm rtc_data[1];
	struct bd71805 *mfd = dev_get_drvdata(dev->parent);
	int ret;

	// ret = regmap_bulk_read(mfd->regmap, BD71805_REG_SEC, rtc_data, sizeof rtc_data);
	ret = mfd->read(mfd, BD71805_REG_SEC, sizeof rtc_data, rtc_data);
	if (ret < 0) {
		dev_err(dev, "reading from RTC failed with err:%d\n", ret);
		return ret;
	}

	hw_to_rtc_time(tm, rtc_data);

	return ret;
}

/**@brief write date/time to bd71805 rtc
 * @param dev rtc device of system
 * @param tm date/time source
 * @retval 0 success
 * @retval negative error number
 */
static int bd71805_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct bd71805_rtc_alarm rtc_data[1];
	struct bd71805 *mfd = dev_get_drvdata(dev->parent);
	int ret;

	rtc_time_to_hw(rtc_data, tm);

	/* update all the time registers in one shot */
	// ret = regmap_bulk_write(mfd->regmap, BD71805_REG_SEC, rtc_data, sizeof rtc_data);
	ret = mfd->write(mfd, BD71805_REG_SEC, sizeof rtc_data, rtc_data);
	if (ret < 0) {
		dev_err(dev, "rtc_set_time error %d\n", ret);
		return ret;
	}

	return ret;
}

/**@brief Gets current bd71805 RTC alarm time.
 * @param dev rtc device of system
 * @param alm alarm date/time store target
 * @retval 0 success
 * @retval negative error number
 */
static int bd71805_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct bd71805_rtc_alarm rtc_data[1];
	u32 int_val;
	struct bd71805 *mfd = dev_get_drvdata(dev->parent);
	int ret;

	// ret = regmap_bulk_read(mfd->regmap, BD71805_REG_ALM0_SEC, rtc_data, sizeof rtc_data);
	ret = mfd->read(mfd, BD71805_REG_ALM0_SEC, sizeof rtc_data, rtc_data);
	if (ret < 0) {
		dev_err(dev, "rtc_read_alarm error %d\n", ret);
		return ret;
	}
	
	hw_to_rtc_time(&alm->time, rtc_data);

	ret = bd71805_regmap_read(mfd, BD71805_REG_INT_EN_12, &int_val);
	if (ret < 0)
		return ret;

	if (int_val & ALM0)
		alm->enabled = 1;

	return ret;
}

/**@brief Set current bd71805 RTC alarm time
 * @param dev rtc device of system
 * @param alm alarm date/time to set
 * @retval 0 success
 * @retval negative error number
 */
static int bd71805_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct bd71805_rtc_alarm rtc_data[1];
	struct bd71805 *mfd = dev_get_drvdata(dev->parent);
	int ret;

	// printk("%s() L%d\n", __func__, __LINE__);

	ret = bd71805_rtc_alarm_irq_enable(dev, 0);
	if (ret)
		return ret;

	rtc_time_to_hw(rtc_data, &alm->time);

	/* update all the alarm registers in one shot */
	// ret = regmap_bulk_write(mfd->regmap, BD71805_REG_ALM0_SEC, rtc_data, sizeof rtc_data);
	ret = mfd->write(mfd, BD71805_REG_ALM0_SEC, sizeof rtc_data, rtc_data);
	if (ret) {
		dev_err(dev, "rtc_set_alarm error %d\n", ret);
		return ret;
	}

	if (alm->enabled)
		ret = bd71805_rtc_alarm_irq_enable(dev, 1);

	return ret;
}

/**@brief bd71805 rtc alarm interrupt
 * @param irq system irq
 * @param rtc rtc device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t bd71805_rtc_interrupt(int irq, void *rtc)
{
	struct device *dev = rtc;
	unsigned long events = 0;
	struct bd71805 *mfd = dev_get_drvdata(dev->parent);
	struct bd71805_rtc *bd_rtc = dev_get_drvdata(dev);
	int ret;
	u32 rtc_reg;

	ret = bd71805_regmap_read(mfd, BD71805_REG_INT_STAT_12, &rtc_reg);
	if (ret)
		return IRQ_NONE;

	if (rtc_reg & ALM0)
		events = RTC_IRQF | RTC_AF;

	ret = bd71805_reg_write(mfd, BD71805_REG_INT_STAT_12, rtc_reg);
	if (ret)
		return IRQ_NONE;

	printk("\n~~~IRQ ALARM.\n");

	/* Notify RTC core on event */
	rtc_update_irq(bd_rtc->rtc, 1, events);

	return IRQ_HANDLED;
}

/** @brief function operations definition */
static struct rtc_class_ops bd71805_rtc_ops = {
	.read_time	= bd71805_rtc_read_time,
	.set_time	= bd71805_rtc_set_time,
	.read_alarm	= bd71805_rtc_read_alarm,
	.set_alarm	= bd71805_rtc_set_alarm,
	.alarm_irq_enable = bd71805_rtc_alarm_irq_enable,
};

/**@brief probe bd71805 rtc device
 @param pdev bd71805 rtc platform device
 @retval 0 success
 @retval negative fail
*/
static int bd71805_rtc_probe(struct platform_device *pdev)
{
	struct bd71805 *bd71805 = NULL;
	struct bd71805_rtc *bd_rtc = NULL;
	int ret;
	int irq;
	u32 rtc_reg;

	bd71805 = dev_get_drvdata(pdev->dev.parent);

	bd_rtc = devm_kzalloc(&pdev->dev, sizeof(struct bd71805_rtc),
			GFP_KERNEL);
	if (!bd_rtc)
		return -ENOMEM;

	/* Clear pending interrupts */
	ret = bd71805_regmap_read(bd71805, BD71805_REG_INT_STAT_12, &rtc_reg);
	if (ret < 0)
		return ret;

	ret = bd71805_reg_write(bd71805, BD71805_REG_INT_STAT_12, rtc_reg);
	if (ret < 0)
		return ret;

	dev_dbg(&pdev->dev, "Enabling rtc-bd71805.\n");

	#if 0
	/* Enable RTC alarm interrupt */
	// ret = regmap_update_bits(bd71805->regmap, BD71805_REG_INT_EN_00, ALMALE, ALMALE);
	ret = bd71805_update_bits(bd71805, BD71805_REG_INT_EN_00, ALMALE, ALMALE);
	if (ret < 0) {
		return ret;
	}
	#endif

	/* Disable ALM0 mask */
	ret = bd71805_reg_write(bd71805, BD71805_REG_ALM0_MASK, 0x77);
	if (ret < 0)
		return ret;

	irq = platform_get_irq(pdev, 0);
	irq += bd71805->irq_base;
	if (irq <= 0) {
		dev_warn(&pdev->dev, "Wake up is not possible as irq = %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd71805_rtc_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ is not free.\n");
		return ret;
	}
	bd_rtc->irq = irq;
	device_set_wakeup_capable(&pdev->dev, 1);

	/* bd_rtc->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
		&bd71805_rtc_ops, THIS_MODULE); */
	bd_rtc->rtc = rtc_device_register(pdev->name, &pdev->dev,
		&bd71805_rtc_ops, THIS_MODULE);
	if (IS_ERR(bd_rtc->rtc)) {
		ret = PTR_ERR(bd_rtc->rtc);
		dev_err(&pdev->dev, "RTC device register: err %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, bd_rtc);

	return 0;
}

/*
 * Disable bd71805 RTC interrupts.
 * Sets status flag to free.
 */
/**@brief remove bd71805 rtc device
 @param pdev bd71805 rtc platform device
 @return 0
*/
static int bd71805_rtc_remove(struct platform_device *pdev)
{
	bd71805_rtc_alarm_irq_enable(&pdev->dev, 0);

	return 0;
}

/**@brief shutdown bd71805 rtc device
 @param pdev bd71805 rtc platform device
 @return void
*/
static void bd71805_rtc_shutdown(struct platform_device *pdev)
{
	/* mask timer interrupts, but leave alarm interrupts on to enable
	   power-on when alarm is triggered */
	bd71805_rtc_alarm_irq_enable(&pdev->dev, 0);
}

#ifdef CONFIG_PM_SLEEP
/**@brief suspend bd71805 rtc device
 * @param dev rtc device of system
 * @retval 0
 */
static int bd71805_rtc_suspend(struct device *dev)
{
	struct bd71805_rtc *bd_rtc = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(bd_rtc->irq);
	return 0;
}

/**@brief resume bd71805 rtc device
 * @param dev rtc device of system
 * @retval 0
 */
static int bd71805_rtc_resume(struct device *dev)
{
	struct bd71805_rtc *bd_rtc = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(bd_rtc->irq);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(bd71805_rtc_pm_ops, bd71805_rtc_suspend, bd71805_rtc_resume);

#ifdef CONFIG_OF
static const struct of_device_id bd71805_rtc_of_match[] = {
	{.compatible = "ti,bd71805-rtc", },
	{ },
};
MODULE_DEVICE_TABLE(of, bd71805_rtc_of_match);
#endif

static struct platform_driver bd71805rtc_driver = {
	.probe		= bd71805_rtc_probe,
	.remove		= bd71805_rtc_remove,
	.shutdown	= bd71805_rtc_shutdown,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "bd71805-rtc",
		.pm		= &bd71805_rtc_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(bd71805_rtc_of_match),
#endif
	},
};

/**@brief module initialize function */
static int __init bd71805_rtc_init(void)
{
	return platform_driver_register(&bd71805rtc_driver);
}
module_init(bd71805_rtc_init);

/**@brief module deinitialize function */
static void __exit bd71805_rtc_exit(void)
{
	platform_driver_unregister(&bd71805rtc_driver);
}
module_exit(bd71805_rtc_exit);

MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bd71805-rtc");
