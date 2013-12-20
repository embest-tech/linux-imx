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

#include "platform.h"

#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_DEBUG_PLATFORM
#define	PRINT_PLAT_MSG(msg...) printk(msg)
#else
#define	PRINT_PLAT_MSG(msg...)
#endif

// ****************************************************************************
// Globel or static variables
// ****************************************************************************


// ****************************************************************************
// Function declaration
// ****************************************************************************
struct struct_platform_var* platform_var_init(struct struct_platform_param *param)
{
	struct struct_platform_var* platform_data = NULL;

	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	platform_data = kzalloc(sizeof(struct struct_platform_var), GFP_KERNEL);
	if ( platform_data ) {
		platform_data->bustype = param->bustype;

		switch ( platform_data->bustype ) {
			#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_INTERFACE_I2C
			case PLATFORM_BUS_I2C:
				platform_data->ops.platform_bus_var_init = platform_i2c_var_init;
				platform_data->ops.platform_bus_var_exit = platform_i2c_var_exit;
				
				platform_data->i2c = platform_data->ops.platform_bus_var_init();
				break;
			#endif

			#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_INTERFACE_USB
			case PLATFORM_BUS_USB:
				platform_data->ops.platform_bus_var_init = platform_usb_var_init;
				platform_data->ops.platform_bus_var_exit = platform_usb_var_exit;
				
				platform_data->usb = platform_data->ops.platform_bus_var_init();
				break;
			#endif
		}

		PRINT_PLAT_MSG("%s: kzalloc OK for variable platform.\n", __FUNCTION__);
	}

	return platform_data;
}

void platform_var_exit(struct struct_platform_var* platform_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( platform_data ) {
		if ( platform_data->ops.platform_bus_var_exit )
		switch ( platform_data->bustype ) {
			#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_INTERFACE_I2C
			case PLATFORM_BUS_I2C:
				platform_data->ops.platform_bus_var_exit(platform_data->i2c);
				break;
			#endif

			#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_INTERFACE_USB
			case PLATFORM_BUS_USB:
				platform_data->ops.platform_bus_var_exit(platform_data->usb);
				break;
			#endif
		}
		kfree(platform_data);
	}
}

