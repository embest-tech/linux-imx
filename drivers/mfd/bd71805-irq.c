/*
 * @file bd71805-irq.c  --  RoHM BD71805MWV mfd driver irq funcs
 * 
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * @author: Peter Yang <yanglsh@embest-tech.com>
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/bd71805.h>

#define BD71805_INT_STS		BD71805_REG_INT_STAT_00
#define BD71805_INT_MSK		BD71805_REG_INT_EN_00

static inline int irq_to_bd71805_irq(struct bd71805 *bd71805, int irq)
{
	return (irq - bd71805->irq_base);
}

/*
 * This is a threaded IRQ handler so can access I2C/SPI.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.  We're also assuming that
 * it's rare to get lots of interrupts firing simultaneously so try to
 * minimise I/O.
 */
static irqreturn_t bd71805_irq(int irq, void *irq_data)
{
	struct bd71805 *bd71805 = irq_data;
	u32 irq_sts;
	u32 irq_mask;
	u8 reg;
	int i;

	bd71805->read(bd71805, BD71805_INT_STS, 1, &reg);
	irq_sts = reg;

	bd71805->read(bd71805, BD71805_INT_MSK, 1, &reg);
	irq_mask = reg;

	irq_sts &= irq_mask;

	if (!irq_sts)
		return IRQ_NONE;

	for (i = 0; i < bd71805->irq_num; i++) {

		if (!(irq_sts & (1 << i)))
			continue;

		handle_nested_irq(bd71805->irq_base + i);
	}

	/* Write the STS register back to clear IRQs we handled */
	reg = irq_sts & 0xFF;
	irq_sts >>= 8;
	bd71805->write(bd71805, BD71805_INT_STS, 1, &reg);

	return IRQ_HANDLED;
}

static void bd71805_irq_lock(struct irq_data *data)
{
	struct bd71805 *bd71805 = irq_data_get_irq_chip_data(data);

	mutex_lock(&bd71805->irq_lock);
}

static void bd71805_irq_sync_unlock(struct irq_data *data)
{
	struct bd71805 *bd71805 = irq_data_get_irq_chip_data(data);
	u32 reg_mask;
	u8 reg;

	bd71805->read(bd71805, BD71805_INT_MSK, 1, &reg);
	reg_mask = reg;

	if (bd71805->irq_mask != reg_mask) {
		reg = bd71805->irq_mask & 0xFF;
		bd71805->write(bd71805, BD71805_INT_MSK, 1, &reg);
	}
	mutex_unlock(&bd71805->irq_lock);
}

static void bd71805_irq_enable(struct irq_data *data)
{
	struct bd71805 *bd71805 = irq_data_get_irq_chip_data(data);

	bd71805->irq_mask |= ( 1 << irq_to_bd71805_irq(bd71805, data->irq));
}

static void bd71805_irq_disable(struct irq_data *data)
{
	struct bd71805 *bd71805 = irq_data_get_irq_chip_data(data);

	bd71805->irq_mask &= ~( 1 << irq_to_bd71805_irq(bd71805, data->irq));
}

static struct irq_chip bd71805_irq_chip = {
	.name = "bd71805",
	.irq_bus_lock = bd71805_irq_lock,
	.irq_bus_sync_unlock = bd71805_irq_sync_unlock,
	.irq_disable = bd71805_irq_disable,
	.irq_enable = bd71805_irq_enable,
};

int bd71805_irq_init(struct bd71805 *bd71805, struct bd71805_board *pdata)
{
	int ret, cur_irq;
	int flags = IRQF_ONESHOT | IRQF_TRIGGER_LOW;
	int irq;

	if (!pdata || !pdata->irq_base) {
		dev_warn(bd71805->dev, "No interrupt support, no IRQ base\n");
		return -EINVAL;
	}

	irq = bd71805->chip_irq;
	if (irq < 0) {
		dev_warn(bd71805->dev, "No interrupt support, no core IRQ\n");
		return -EINVAL;
	}

	bd71805->irq_mask = 0x0;

	mutex_init(&bd71805->irq_lock);

	bd71805->irq_base = pdata->irq_base;
	bd71805->irq_num = BD71805_NUM_IRQ;

	/* Register with genirq */
	for (cur_irq = bd71805->irq_base;
	     cur_irq < bd71805->irq_num + bd71805->irq_base;
	     cur_irq++) {
		irq_set_chip_data(cur_irq, bd71805);
		irq_set_chip_and_handler(cur_irq, &bd71805_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);

		/* ARM needs us to explicitly flag the IRQ as valid
		 * and will set them noprobe when we do so. */
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}

	ret = request_threaded_irq(irq, NULL, bd71805_irq, flags,
				   "bd71805", bd71805);

	irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);

	if (ret != 0)
		dev_err(bd71805->dev, "Failed to request IRQ %d: %d\n", irq, ret);

	return ret;
}

int bd71805_irq_exit(struct bd71805 *bd71805)
{
	free_irq(bd71805->chip_irq, bd71805);
	return 0;
}
