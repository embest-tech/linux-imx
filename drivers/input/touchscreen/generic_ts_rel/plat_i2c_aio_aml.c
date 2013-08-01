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
void platform_i2c_read(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
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

	i2c_transfer(client->adapter, msgs, 2);
}

void platform_i2c_write(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = addr,
			.flags = 0x00,  // 0x00: write 0x01:read 
			.len = len,
			.buf = buf,
			//.scl_rate = PLATFORM_I2C_SPEED,
		},
	};

	i2c_transfer(client->adapter, msgs, 1);
}

void platform_i2c_get_cfg(struct struct_platform_i2c_var *platform_i2c_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	/* I2C config */
	platform_i2c_data->i2c_bus = PLATFORM_I2C_BUS;
	platform_i2c_data->i2c_addr = PLATFORM_I2C_ADDR;

	/* GPIO config */
	platform_i2c_data->pwr = PLATFORM_AML_PWR_PIN;
	platform_i2c_data->rst = PLATFORM_AML_RST_PIN;
	platform_i2c_data->ss = PLATFORM_AML_IRQ_PIN;

	/* IRQ config*/
	//platform_i2c_data->irq = gpio_to_irq(platform_i2c_data->ss);
	platform_i2c_data->irq = PLATFORM_AML_IRQ_NUM;
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

	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	// Init PwrEn pin
	if ( platform_i2c_data->pwr ) {
		err = gpio_request(platform_i2c_data->pwr, "generic_ts_pwr");
		if ( err ) {
			return -EIO;
		}
		//gpio_direction_output(platform_i2c_data->pwr, 1);
		//gpio_set_value(platform_i2c_data->pwr, 1);
		gpio_set_status(platform_i2c_data->pwr, gpio_status_out);
		gpio_out(platform_i2c_data->pwr, 1);
	}

	// Init Reset pin
	err = gpio_request(platform_i2c_data->rst, "generic_ts_rst");
	if ( err ) {
		return -EIO;
	}
	//gpio_direction_output(platform_i2c_data->rst, 1);
	//gpio_set_value(platform_i2c_data->rst, 1);
	gpio_set_status(platform_i2c_data->rst, gpio_status_out);
	gpio_out(platform_i2c_data->rst, 1);

	// Init Int pin
	err = gpio_request(platform_i2c_data->ss, "generic_ts_int");
	if ( err ) {
		return -EIO;
	}
	//gpio_direction_input(platform_i2c_data->ss);
	gpio_set_status(platform_i2c_data->ss, gpio_status_in);
	gpio_irq_set(170, GPIO_IRQ(platform_i2c_data->irq-INT_GPIO_0, GPIO_IRQ_FALLING));

	return 0;
}

void platform_i2c_put_rsrc(struct struct_platform_i2c_var *platform_i2c_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	gpio_free(platform_i2c_data->pwr);
	gpio_free(platform_i2c_data->rst);
	gpio_free(platform_i2c_data->ss);
}

void platform_i2c_hw_reset(struct struct_platform_i2c_var *platform_i2c_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	mdelay(500);
	gpio_set_value(platform_i2c_data->rst, 0);
	//mdelay(50);
	mdelay(5);
	gpio_set_value(platform_i2c_data->rst, 1);
	mdelay(500);
}

int platform_i2c_chk_irq(struct struct_platform_i2c_var *platform_i2c_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	return 1;
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

