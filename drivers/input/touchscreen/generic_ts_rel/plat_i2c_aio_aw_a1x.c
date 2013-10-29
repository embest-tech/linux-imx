/* 
 * drivers/input/touchscreen/generic_ts.c
 *
 * generic touchscreen driver. 
 *
 * Copyright (c) 2012.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * George Chen, 2012-06-15
 */

// ****************************************************************************
// Includes
// ****************************************************************************
#include <linux/slab.h>
#include "core_i2c.h"
#include "plat_i2c.h"

#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_DEBUG_PLATFORM
#define	PRINT_PLAT_MSG(msg...) printk(msg)
#else
#define	PRINT_PLAT_MSG(msg...)
#endif

// ****************************************************************************
// Globel or static variables
// ****************************************************************************
static 
struct i2c_board_info i2c_board_info[] = {
	{
		I2C_BOARD_INFO(DRIVER_NAME, PLATFORM_I2C_ADDR),
		.platform_data = NULL,
	},
};


// ****************************************************************************
// Function declaration
// ****************************************************************************
int platform_i2c_read(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = addr,
			.flags = 0x00,  // 0x00: write 0x01:read 
			.len = 1,
			.buf = buf,
			//.scl_rate = PLATFORM_I2C_SPEED,
		},
		{
			.addr = addr,
			.flags = 0x01,  // 0x00: write 0x01:read 
			.len = len,
			.buf = buf,
			//.scl_rate = PLATFORM_I2C_SPEED,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);

	return (ret != 2);
}

int platform_i2c_write(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = addr,
			.flags = 0x00,  // 0x00: write 0x01:read 
			.len = len,
			.buf = buf,
			//.scl_rate = PLATFORM_I2C_SPEED,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 1);

	return (ret != 1);
}

void platform_i2c_get_cfg(struct struct_platform_i2c_var *platform_i2c_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	/* I2C config */
	platform_i2c_data->i2c_bus = PLATFORM_I2C_BUS;
	platform_i2c_data->i2c_addr = PLATFORM_I2C_ADDR;

	/* GPIO config */
	//platform_i2c_data->pwr = PLATFORM_PWR_PIN;
	//platform_i2c_data->rst = PLATFORM_RST_PIN;
	//platform_i2c_data->ss = PLATFORM_IRQ_PIN;

	/* IRQ config*/
	platform_i2c_data->irq = SW_INT_IRQNO_PIO;
}

int platform_i2c_set_dev(struct struct_platform_i2c_var *platform_i2c_data)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	adapter = i2c_get_adapter(platform_i2c_data->i2c_bus);
	if ( !adapter ) {
		printk("Unable to get i2c adapter on bus %d.\n", platform_i2c_data->i2c_bus);
		return -ENODEV;
	}

	client = i2c_new_device(adapter, i2c_board_info);
	i2c_put_adapter(adapter);
	if (!client) {
		printk("Unable to create i2c device on bus %d.\n", platform_i2c_data->i2c_bus);
		return -ENODEV;
	}

	platform_i2c_data->client = client;

	return 0;
}

int platform_i2c_get_rsrc(struct struct_platform_i2c_var *platform_i2c_data)
{
	int err = -1;
	int reg_num;
	int reg_addr;
	int reg_val;
	int int_cfg_addr[] = {
		PLATFORM_AW_PIO_INT_CFG0_OFFSET, PLATFORM_AW_PIO_INT_CFG1_OFFSET, 
		PLATFORM_AW_PIO_INT_CFG2_OFFSET, PLATFORM_AW_PIO_INT_CFG3_OFFSET, 
	};

	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	// gpio_address_remap
	platform_i2c_data->gpio_addr = ioremap(PLATFORM_AW_PIO_BASE_ADDR, PLATFORM_AW_PIO_RANGE_SIZE);
	if ( !platform_i2c_data->gpio_addr ) {
		return -EIO;
	}

	// Init PwrEn pin
	if ( PLATFORM_AW_PWR_PIN_STR ) {
		platform_i2c_data->pwr = gpio_request_ex(PLATFORM_AW_CTP_PARA_STR, PLATFORM_AW_PWR_PIN_STR);
		if ( !platform_i2c_data->pwr ) {
			return -EIO;
		}
		//gpio_direction_output(platform_i2c_data->pwr, 1);
		//gpio_set_value(platform_i2c_data->pwr, 1);
	}

	// Init Reset pin
	if ( PLATFORM_AW_RST_PIN_STR ) {
		platform_i2c_data->rst = gpio_request_ex(PLATFORM_AW_CTP_PARA_STR, PLATFORM_AW_RST_PIN_STR);
		if ( !platform_i2c_data->rst ) {
			return -EIO;
		}
		//gpio_direction_output(platform_i2c_data->rst, 1);
		//gpio_set_value(platform_i2c_data->rst, 1);
	}

	// Init Int pin
	if ( PLATFORM_AW_IRQ_PIN_STR ) {
		platform_i2c_data->ss = gpio_request_ex(PLATFORM_AW_CTP_PARA_STR, PLATFORM_AW_IRQ_PIN_STR);
		if ( !platform_i2c_data->ss ) {
			return -EIO;
		}
		//gpio_direction_input(platform_i2c_data->ss);
	}

	/* config irq mode */
	reg_num = PLATFORM_AW_CTP_IRQ_NUM % 8;
	reg_addr = PLATFORM_AW_CTP_IRQ_NUM / 8;
	// falling edge triggering
	reg_val = readl(platform_i2c_data->gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= ~(7 << (reg_num * 4));	// clear trigger mode
	reg_val |= ( PLATFORM_AW_CTP_IRQ_MOD << (reg_num * 4));	// set trigger mode
	writel(reg_val, platform_i2c_data->gpio_addr + int_cfg_addr[reg_addr]);
	// clear pending interrupt
	reg_val = readl(platform_i2c_data->gpio_addr + PLATFORM_AW_PIO_INT_STAT_OFFSET);
	if ( reg_val & (1 << PLATFORM_AW_CTP_IRQ_NUM) ) {
		writel(reg_val & (1 << PLATFORM_AW_CTP_IRQ_NUM),  platform_i2c_data->gpio_addr + PLATFORM_AW_PIO_INT_STAT_OFFSET);
	}
	// enable irq
	reg_val = readl(platform_i2c_data->gpio_addr + PLATFORM_AW_PIO_INT_CTRL_OFFSET);
	reg_val |= (1 << PLATFORM_AW_CTP_IRQ_NUM);
	writel(reg_val, platform_i2c_data->gpio_addr + PLATFORM_AW_PIO_INT_CTRL_OFFSET);

	return 0;
}

void platform_i2c_put_rsrc(struct struct_platform_i2c_var *platform_i2c_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( platform_i2c_data->gpio_addr )
		iounmap(platform_i2c_data->gpio_addr);
	if ( platform_i2c_data->pwr )
		gpio_release(platform_i2c_data->pwr, 2);
	if ( platform_i2c_data->rst )
		gpio_release(platform_i2c_data->rst, 2);
	if ( platform_i2c_data->ss )
		gpio_release(platform_i2c_data->ss, 2);
}

void platform_i2c_hw_reset(struct struct_platform_i2c_var *platform_i2c_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	mdelay(500);
	gpio_write_one_pin_value(platform_i2c_data->rst, 0, PLATFORM_AW_RST_PIN_STR);
	mdelay(5);
	gpio_write_one_pin_value(platform_i2c_data->rst, 1, PLATFORM_AW_RST_PIN_STR);
	mdelay(500);
}

int platform_i2c_chk_irq(struct struct_platform_i2c_var *platform_i2c_data)
{
	int ret = 0;
	int reg_val;
	
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	// read irq trigger flag
	reg_val = readl(platform_i2c_data->gpio_addr + PLATFORM_AW_PIO_INT_STAT_OFFSET);
	// if hit, clear, return true
	if ( reg_val & (1 << PLATFORM_AW_CTP_IRQ_NUM) ) {
		writel(reg_val & (1 << PLATFORM_AW_CTP_IRQ_NUM),  platform_i2c_data->gpio_addr + PLATFORM_AW_PIO_INT_STAT_OFFSET);
		ret = 1;
	}

	return ret;
}

struct struct_platform_i2c_var* platform_i2c_var_init(void)
{
	struct struct_platform_i2c_var* platform_i2c_data = NULL;

	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	platform_i2c_data = kzalloc(sizeof(struct struct_platform_i2c_var), GFP_KERNEL);
	if ( platform_i2c_data ) {
		PRINT_PLAT_MSG("%s: kzalloc OK for variable track.\n", __FUNCTION__);
		platform_i2c_data->ops.i2c_read = platform_i2c_read;
		platform_i2c_data->ops.i2c_write = platform_i2c_write;
		platform_i2c_data->ops.get_cfg = platform_i2c_get_cfg;
		platform_i2c_data->ops.set_dev = platform_i2c_set_dev;
		platform_i2c_data->ops.get_rsrc = platform_i2c_get_rsrc;
		platform_i2c_data->ops.put_rsrc = platform_i2c_put_rsrc;
		platform_i2c_data->ops.hw_reset = platform_i2c_hw_reset;
		platform_i2c_data->ops.chk_irq = platform_i2c_chk_irq;
	}

	return platform_i2c_data;
}

void platform_i2c_var_exit(struct struct_platform_i2c_var* platform_i2c_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( platform_i2c_data )
		kfree(platform_i2c_data);
}

