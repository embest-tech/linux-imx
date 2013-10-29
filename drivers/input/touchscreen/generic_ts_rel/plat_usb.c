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
#include <linux/usb.h>
#include <linux/hid.h>

#include "core_usb.h"
#include "plat_usb.h"


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

void platform_usb_proc_pkg(/*struct i2c_client *client, unsigned short addr, */char *dst, char *src, int len)
{
	//PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);
	//
	//

	if (src[0] == 0x01) {
		//PRINT_PLAT_MSG("usb package: %d \n", len);
		//PRINT_PLAT_MSG("buf[0]: %d \n", buf[0]);
		//PRINT_PLAT_MSG("buf[61]: %d \n", buf[61]);
		memcpy(dst, src+1, len);
	}
}

void platform_usb_write(/*struct i2c_client *client, unsigned short addr, */char *buf, int len)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);
}

void platform_usb_get_cfg(struct struct_platform_usb_var *platform_usb_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	/* USB config */
	platform_usb_data->idvend = PLATFORM_USB_IDVEND;
	platform_usb_data->idprod = PLATFORM_USB_IDPROD;

	/* GPIO config */

	/* IRQ config*/
}

int platform_usb_set_dev(struct struct_platform_usb_var *platform_usb_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( platform_usb_data->intf != NULL ) {
		platform_usb_data->udev = interface_to_usbdev(platform_usb_data->intf);
		platform_usb_data->epdesc = &platform_usb_data->intf->cur_altsetting->endpoint[0].desc;
	
		return 0;
	}

	return -1;
}

int platform_usb_get_rsrc(struct struct_platform_usb_var *platform_usb_data)
{
	int err = -1;

	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);
	
	platform_usb_data->buf = usb_alloc_coherent(platform_usb_data->udev, 256, GFP_KERNEL, &platform_usb_data->dma_addr);
	platform_usb_data->irq = usb_alloc_urb(0, GFP_KERNEL);

	/* */
	
	//platform_data->irq->dev = platform_data->udev;
	//platform_data->irq->transfer_dma = platform_data->dma_addr;
	//platform_data->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	return 0;
}

void platform_usb_put_rsrc(struct struct_platform_usb_var *platform_usb_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

}

void platform_usb_hw_reset(struct struct_platform_usb_var *platform_usb_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);


}

int platform_usb_chk_irq(void)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	return 1;
}

struct struct_platform_usb_var* platform_usb_var_init(void)
{
	struct struct_platform_usb_var* platform_usb_data = NULL;

	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	platform_usb_data = kzalloc(sizeof(struct struct_platform_usb_var), GFP_KERNEL);
	if ( platform_usb_data ) {
		platform_usb_data->ops.usb_proc_pkg = platform_usb_proc_pkg;
		//platform_usb_data->ops.usb_write = platform_usb_write;
		platform_usb_data->ops.get_cfg = platform_usb_get_cfg;
		platform_usb_data->ops.set_dev = platform_usb_set_dev;
		platform_usb_data->ops.get_rsrc = platform_usb_get_rsrc;
		platform_usb_data->ops.put_rsrc = platform_usb_put_rsrc;
		platform_usb_data->ops.hw_reset = platform_usb_hw_reset;

		PRINT_PLAT_MSG("%s: kzalloc OK for variable track.\n", __FUNCTION__);
	}

	return platform_usb_data;
}

void platform_usb_var_exit(struct struct_platform_usb_var* platform_usb_data)
{
	PRINT_PLAT_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( platform_usb_data )
		kfree(platform_usb_data);
}


