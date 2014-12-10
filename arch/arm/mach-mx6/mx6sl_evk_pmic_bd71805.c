/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/bd71805.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include "cpu_op-mx6.h"

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

#define BD71805_I2C_DEVICE_NAME  "bd71805"
/* 7-bit I2C bus slave address */
#define BD71805_I2C_ADDR         (0x4B)

static struct regulator_consumer_supply buck1_consumers[] = {
	{
		.supply = "VDDCORE",
	 },
};

static struct regulator_consumer_supply buck2_consumers[] = {
	{
		.supply = "VDDSOC",
	 },
};

static struct regulator_consumer_supply buck3_consumers[] = {
	{
		.supply		= "MICVDD",
		.dev_name	= "1-001a",
	},
};

static struct regulator_consumer_supply buck4_consumers[] = {
	{
	 .supply = "VDDR_1V2",
	 },
};

static struct regulator_consumer_supply ldo1_consumers[] = {
       {
	.supply = "VGEN5_2V8",
	},

};

static struct regulator_consumer_supply ldo2_consumers[] = {
       {
	.supply = "AUD_1V8",
	},
	{
		.supply    = "AVDD",
		.dev_name	= "1-001a",
	},
	{
		.supply    = "DCVDD",
		.dev_name	= "1-001a",
	},
	{
		.supply    = "CPVDD",
		.dev_name	= "1-001a",
	},
	{
		.supply    = "PLLVDD",
		.dev_name	= "1-001a",
	},
	{
		.supply		= "DBVDD",
		.dev_name	= "1-001a",
	},
       {
	.supply = "VGEN4_1V8",
	}
};

static struct regulator_consumer_supply ldo3_consumers[] = {
	{
	 .supply = "VGEN1_1V5",
	 },
};

void bd71805_dump_regs(struct bd71805 *mfd)
{
	int reg;
	u8 val;

	for (reg = BD71805_REG_PWRCTRL; reg < BD71805_REG_BUCK_PDDIS; reg++) {
		val = bd71805_reg_read(mfd, reg);
		printk("----0x%x= 0x%x\n", reg, val);
	}
}

static int bd71805_init(struct bd71805 *mfd)
{
	u8 id;

	id = bd71805_reg_read(mfd, BD71805_REG_DEVICE);
	if (id) {
		printk(KERN_ERR "BD71805 device id: 0x%X\n", id);
		// return -ENODEV;
	}
	//  bd71805_dump_regs(mfd);
	return 0;
}

static struct regulator_init_data bd71805_buck1_init = {
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.boot_on = 1,
			.always_on = 1,
			},
	.consumer_supplies = buck1_consumers,
	.num_consumer_supplies = ARRAY_SIZE(buck1_consumers),
};

static struct regulator_init_data bd71805_buck2_init = {
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.boot_on = 1,
			.always_on = 1,
			},
	.consumer_supplies = buck2_consumers,
	.num_consumer_supplies = ARRAY_SIZE(buck2_consumers),
};

static struct regulator_init_data bd71805_buck3_init = {
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.boot_on = 1,
			.always_on = 1,
			},
	.consumer_supplies = buck3_consumers,
	.num_consumer_supplies = ARRAY_SIZE(buck3_consumers),
};

static struct regulator_init_data bd71805_buck4_init = {
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.boot_on = 1,
			.always_on = 1,
			},
	.consumer_supplies = buck4_consumers,
	.num_consumer_supplies = ARRAY_SIZE(buck4_consumers),
};

static struct regulator_init_data bd71805_ldo1_init = {
	.supply_regulator = "buck3",
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.always_on = 1,
			},
	.consumer_supplies = ldo1_consumers,
	.num_consumer_supplies = ARRAY_SIZE(ldo1_consumers),
};

static struct regulator_init_data bd71805_ldo2_init = {
	.supply_regulator = "buck3",
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = 0,
			.always_on = 1,
			},
	.consumer_supplies = ldo2_consumers,
	.num_consumer_supplies = ARRAY_SIZE(ldo2_consumers),
};

static struct regulator_init_data bd71805_ldo3_init = {
	.supply_regulator = "buck3",
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.boot_on = 1,
			},
	.consumer_supplies = ldo3_consumers,
	.num_consumer_supplies = ARRAY_SIZE(ldo3_consumers),
};

static struct regulator_init_data bd71805_init_data = {
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
			.boot_on = 1,
			.always_on = 1,
			},
};

static struct bd71805_board bd71805_info = {
	.bd71805_pmic_init_data[BD71805_BUCK1] = &bd71805_buck1_init,
	.bd71805_pmic_init_data[BD71805_BUCK2] = &bd71805_buck2_init,
	.bd71805_pmic_init_data[BD71805_BUCK3] = &bd71805_buck3_init,
	.bd71805_pmic_init_data[BD71805_BUCK4] = &bd71805_buck4_init,
	.bd71805_pmic_init_data[BD71805_LDO1] = &bd71805_ldo1_init,
	.bd71805_pmic_init_data[BD71805_LDO2] = &bd71805_ldo2_init,
	.bd71805_pmic_init_data[BD71805_LDO3] = &bd71805_ldo3_init,
	.bd71805_pmic_init_data[BD71805_VODVREF] = &bd71805_init_data,
	.bd71805_pmic_init_data[BD71805_VOSNVS] = &bd71805_init_data,
	.bd71805_init = bd71805_init,
	.irq_base = MXC_BOARD_IRQ_START,
};

static struct i2c_board_info __initdata bd71805_i2c_device = {
	I2C_BOARD_INFO(BD71805_I2C_DEVICE_NAME, BD71805_I2C_ADDR),
	.platform_data = &bd71805_info,
};

int __init mx6sl_evk_init_bd71805(u32 int_gpio)
{
	if (int_gpio)
		bd71805_i2c_device.irq = gpio_to_irq(int_gpio);	/*update INT gpio */
	return i2c_register_board_info(0, &bd71805_i2c_device, 1);
}
