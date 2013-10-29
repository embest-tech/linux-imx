/* 
 * drivers/input/touchscreen/generic_ts.c
 *
 * generic touchscreen driver. 
 *
 * Copyright (c) 2012 ~ 2013.
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

 /*
	TODO: 
		xxxx_data validation check
		file system access
*/

// ****************************************************************************
// Includes
// ****************************************************************************
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/hid.h>


#include <linux/interrupt.h>

#include "core_usb.h"

#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_DEBUG_CORE
#define	PRINT_CORE_MSG(msg...) printk(msg)
#else
#define	PRINT_CORE_MSG(msg...)
#endif

// ****************************************************************************
// Globel or static variables
// ****************************************************************************
struct struct_core_usb_var *core_data = NULL;

static 
struct usb_device_id usb_dev_id[] = {
	{ USB_DEVICE(PLATFORM_USB_IDVEND, PLATFORM_USB_IDPROD), .driver_info = 0 },
	{}
};

static int core_probe(struct usb_interface *intf, const struct usb_device_id *id);
static void core_disconn(struct usb_interface *intf);
static int core_suspend(struct usb_interface *intf, pm_message_t message);
static int core_resume(struct usb_interface *intf);

static
struct usb_driver generic_ts_driver  = {
	.name		= DRIVER_NAME,
	.id_table	= usb_dev_id,
	.probe		= core_probe,
	.disconnect	= core_disconn,
	.suspend	= core_suspend,
	.resume		= core_resume,
};



// ****************************************************************************
// Function declaration
// ****************************************************************************
static 
struct struct_core_usb_var* core_var_init(void)
{
	struct struct_core_usb_var* core_data = NULL;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	core_data = kzalloc(sizeof(struct struct_core_usb_var), GFP_KERNEL);
	if ( core_data ) {
		core_data->ops.platform_var_init = platform_var_init;
		core_data->ops.platform_var_exit = platform_var_exit;

		core_data->ops.chip_var_init = chip_var_init;
		core_data->ops.chip_var_exit = chip_var_exit;

		core_data->ops.touch_var_init = touch_var_init;
		core_data->ops.touch_var_exit = touch_var_exit;

		core_data->ops.enhance_var_init = enhance_var_init;
		core_data->ops.enhance_var_exit = enhance_var_exit;
	}

	return core_data;
}

static 
void core_var_exit(struct struct_core_usb_var* core_usb_data)
{
	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	if ( core_usb_data ) {
		if ( core_usb_data->ops.enhance_var_exit )
			enhance_var_exit(core_usb_data->edata);
		if ( core_usb_data->ops.touch_var_exit )
			touch_var_exit(core_usb_data->tdata);
		if ( core_usb_data->ops.chip_var_exit )
			chip_var_exit(core_usb_data->cdata);
		if ( core_usb_data->ops.platform_var_exit )
			platform_var_exit(core_usb_data->pdata);

		kfree(core_usb_data);
	}
}


static
void core_irq_handler(struct urb *urb)
{
	struct struct_core_usb_var *core_data = urb->context;
	int retval;

	//PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ETIME:
		/* this urb is timing out */
		dbg("%s - urb timed out - was the device unplugged?",
		    __func__);
		return;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
	case -EPIPE:
		/* this urb is terminated, clean up */
		dbg("%s - urb shutting down with status: %d",
		    __func__, urb->status);
		return;
	default:
		dbg("%s - nonzero urb status received: %d",
		    __func__, urb->status);
		goto exit;
	}

	usb_mark_last_busy(core_data->pdata->usb->udev);
	core_data->pdata->usb->ops.usb_proc_pkg(core_data->cdata->pts_data, core_data->pdata->usb->buf, core_data->cdata->pts_num*core_data->cdata->pt_sz);

	if ( core_data->tdata->ops.obtain_pts )
		core_data->tdata->ops.obtain_pts(
			core_data->tdata, 
			core_data->cdata->pts_data, 
			core_data->cdata->ops.cpy_valid_pt, 
			core_data->cdata->pt_sz);

	/* report touch points */
	if ( core_data->tdata->ops.report_pts ) {
		core_data->tdata->ops.report_pts(core_data->tdata);
	}

exit:
	usb_submit_urb(urb, GFP_ATOMIC);
}


static
int core_probe(struct usb_interface *intf, const struct usb_device_id *id)
{	
	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	/* create core_data */
	core_data = core_var_init();
	//if ( core_data == NULL )
	//	goto ERR_VAR;

	core_data->state = CORE_USB_STATE_UNKNOWN;
	core_data->state = CORE_USB_STATE_INIT;

	usb_set_intfdata(intf, core_data);

	if ( core_data->ops.platform_var_init ) {
		struct struct_platform_param param;
		param.bustype = PLATFORM_BUS_USB;
		core_data->pdata = core_data->ops.platform_var_init(&param);

		if ( core_data->pdata == NULL ) {
			
		}
	}

	core_data->pdata->usb->intf = intf;

	if ( core_data->pdata->usb->ops.get_cfg ) {
		core_data->pdata->usb->ops.get_cfg(core_data->pdata->usb);
	}

	if ( core_data->pdata->usb->ops.set_dev ) {
		core_data->pdata->usb->ops.set_dev(core_data->pdata->usb);
		//if ( err ) goto ERR_INIT;
	}

	if ( core_data->pdata->usb->ops.get_rsrc ) {
		core_data->pdata->usb->ops.get_rsrc(core_data->pdata->usb);
	}

	/* chip data allocation */
	if ( core_data->ops.chip_var_init ) {
		struct struct_chip_param param;
	#if 1
		param.chip = CHIP_USBVC;
		param.chn_x_num = USBVC_CHN_X_MAX;
		param.chn_y_num = USBVC_CHN_Y_MAX;
		param.chn_res = USBVC_CHN_RES;
		param.x_max = USBVC_ABS_X_MAX;
		param.y_max = USBVC_ABS_Y_MAX;
		param.xy_swap = CHIP_XY_SWAP;
		param.pts_num = USBVC_POINT_NUM;
	#else
		#error No Chip Parameter defined
	#endif
		core_data->cdata = core_data->ops.chip_var_init(&param);
		}

	
#if 1




	//g_data = usb_alloc_coherent(udev, 256,
	//				    GFP_KERNEL, &g_dma_addr);
//
	//g_irq = usb_alloc_urb(0, GFP_KERNEL);
	//if ( !g_irq ) {
	//	printk("usb_alloc_urb failed!!! \n");
	//}
//
	usb_fill_int_urb(core_data->pdata->usb->irq, 
					core_data->pdata->usb->udev,
					usb_rcvintpipe(core_data->pdata->usb->udev, core_data->pdata->usb->epdesc->bEndpointAddress),
					core_data->pdata->usb->buf, 
					256,
					core_irq_handler, 
					core_data, 
					core_data->pdata->usb->epdesc->bInterval);

	core_data->pdata->usb->irq->dev = core_data->pdata->usb->udev;
	core_data->pdata->usb->irq->transfer_dma = core_data->pdata->usb->dma_addr;
	core_data->pdata->usb->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* create touch data */
	if ( core_data->ops.touch_var_init ) {
		// parameters for touch
		struct struct_touch_param param;
		param.valid_max_pts_num = core_data->cdata->pts_num;
		param.x_max = core_data->cdata->x_max;
		param.y_max = core_data->cdata->y_max;
		param.x_rvs = TOUCH_X_REVERSE;
		param.y_rvs = TOUCH_Y_REVERSE;
		param.xy_swp = CHIP_XY_SWAP;

		core_data->tdata = core_data->ops.touch_var_init(&param);

		//if ( core_data->tdata == NULL )
			//goto ERR_TDATA_ALLOC;
	}

	/* enable usb polling */
	if ( usb_submit_urb(core_data->pdata->usb->irq, GFP_KERNEL) )
		return -EIO;
	return 0;
#if 0
	struct usbtouch_usb *usbtouch;
	struct input_dev *input_dev;
	struct usb_host_interface *interface;
	struct usbtouch_device_info *type;
	struct usb_device *udev = interface_to_usbdev(intf);
	int err = -ENOMEM;

	/* some devices are ignored */
	if (id->driver_info == DEVTYPE_IGNORE)
		return -ENODEV;

	interface = intf->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;
	usbtouch = kzalloc(sizeof(struct usbtouch_usb), GFP_KERNEL);
	input_dev=NULL;
	if (id->driver_info == DEVTYPE_NVT){
		input_dev = input_allocate_device();
		if (!usbtouch || !input_dev)
			goto out_free;
	}

	type = &usbtouch_dev_info[id->driver_info];
	usbtouch->type = type;
	if (!type->process_pkt)
		type->process_pkt = usbtouch_process_pkt;

	if (id->driver_info == DEVTYPE_NVT){
		
		usbtouch->data = usb_alloc_coherent(udev, type->rept_size,
	                                  GFP_KERNEL, &usbtouch->data_dma);
		usbtouch->data3 = usb_alloc_coherent(udev, type->rept_size,
	                                  GFP_KERNEL, &usbtouch->data_dma3);

/*		usbtouch->data = usb_buffer_alloc(udev, type->rept_size,
	                                  GFP_KERNEL, &usbtouch->data_dma);
		usbtouch->data3 = usb_buffer_alloc(udev, type->rept_size,
	                                  GFP_KERNEL, &usbtouch->data_dma3);*/
	}

	if (id->driver_info == DEVTYPE_NVT){
		if (!usbtouch->data||!usbtouch->data3)
			goto out_free;
	}

	if (id->driver_info == DEVTYPE_NVT){
		if (type->get_pkt_len) {
			usbtouch->buffer = kmalloc(type->rept_size, GFP_KERNEL);
			if (!usbtouch->buffer)
				goto out_free_buffers;
		}
	}

	if (id->driver_info == DEVTYPE_NVT){
		usbtouch->irq = usb_alloc_urb(0, GFP_KERNEL);
		usbtouch->control = usb_alloc_urb(0, GFP_KERNEL);
	
		if (!usbtouch->irq) {
			dbg("%s - usb_alloc_urb failed: usbtouch->irq", __func__);
			goto out_free_buffers;
		}
		if (!usbtouch->control) {
			dbg("%s - usb_alloc_urb failed: usbtouch->control", __func__);
			goto out_free_buffers;
		}

	}

	usbtouch->udev = udev;
	usbtouch->input=NULL;
	if (id->driver_info == DEVTYPE_NVT)
		usbtouch->input = input_dev;

	if (udev->manufacturer)
		strlcpy(usbtouch->name, udev->manufacturer, sizeof(usbtouch->name));

	if (udev->product) {
		if (udev->manufacturer)
			strlcat(usbtouch->name, " ", sizeof(usbtouch->name));
		strlcat(usbtouch->name, udev->product, sizeof(usbtouch->name));
	}

	if (!strlen(usbtouch->name))
		snprintf(usbtouch->name, sizeof(usbtouch->name),
			"USB Touchscreen %04x:%04x",
			 le16_to_cpu(udev->descriptor.idVendor),
			 le16_to_cpu(udev->descriptor.idProduct));

	usb_make_path(udev, usbtouch->phys, sizeof(usbtouch->phys));
	strlcat(usbtouch->phys, "/input0", sizeof(usbtouch->phys));


	if (id->driver_info == DEVTYPE_NVT){
		input_dev->name = usbtouch->name;
		input_dev->phys = usbtouch->phys;
		usb_to_input_id(udev, &input_dev->id);
		input_dev->dev.parent = &intf->dev;

		input_set_drvdata(input_dev, usbtouch);
	
		input_dev->open = usbtouch_open;
		input_dev->close = usbtouch_close;
		set_bit(EV_SYN, input_dev->evbit);
		set_bit(EV_KEY, input_dev->evbit);
		set_bit(BTN_TOUCH, input_dev->keybit);
		set_bit(BTN_2, input_dev->keybit);
		set_bit(EV_ABS, input_dev->evbit);
		input_set_abs_params(input_dev, ABS_X, type->min_xc, type->max_xc, 0, 0);
		input_set_abs_params(input_dev, ABS_Y, type->min_yc, type->max_yc, 0, 0);
		input_set_abs_params(input_dev, ABS_HAT0X, type->min_xc, type->max_xc, 0, 0);
		input_set_abs_params(input_dev, ABS_HAT0Y, type->min_yc, type->max_yc, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_X, type->min_xc, type->max_xc, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y, type->min_yc, type->max_yc, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 20, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 250, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 50, 0, 0);
		if (type->max_press)
			input_set_abs_params(input_dev, ABS_PRESSURE, type->min_press,
			                     type->max_press, 0, 0);
		usb_fill_int_urb(usbtouch->irq, usbtouch->udev,
				 usb_rcvintpipe(usbtouch->udev, endpoint->bEndpointAddress),
				 usbtouch->data, type->rept_size,
				 usbtouch_irq, usbtouch, endpoint->bInterval);

		usbtouch->irq->dev = usbtouch->udev;
		usbtouch->irq->transfer_dma = usbtouch->data_dma;
		usbtouch->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		 //usb_kill_urb(usbtouch->irq);
	}

	/* device specific init */
	if (type->init) {
		err = type->init(usbtouch);
		if (err) {
			dbg("%s - type->init() failed, err: %d", __func__, err);
			goto out_free_buffers;
		}
	}

	if (id->driver_info == DEVTYPE_NVT){
		err = input_register_device(usbtouch->input);
		if (err) {
			dbg("%s - input_register_device failed, err: %d", __func__, err);
			goto out_free_buffers;
		}
	}
	usb_set_intfdata(intf, usbtouch);
//	flash_priv->usbtouch=usbtouch;
	return 0;

out_free_buffers:
	usbtouch_free_buffers(udev, usbtouch);
out_free:
	if (id->driver_info == DEVTYPE_NVT)
		input_free_device(input_dev);
	kfree(usbtouch);
	return err;
#endif

#endif

	return 0;
}

static 
void core_disconn(struct usb_interface *intf)
{
	struct struct_core_usb_var *core_data = usb_get_intfdata(intf);
	
	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	usb_kill_urb(core_data->pdata->usb->irq);
	//if(usbtouch->input!=NULL)
	//	input_unregister_device(usbtouch->input);
	usb_free_urb(core_data->pdata->usb->irq);
	//usbtouch_free_buffers(interface_to_usbdev(intf), usbtouch);
	usb_free_coherent(core_data->pdata->usb->udev, 256, core_data->pdata->usb->buf, core_data->pdata->usb->dma_addr);

	usb_set_intfdata(intf, NULL);
	core_var_exit(core_data);
}

static
int core_suspend(struct usb_interface *intf, pm_message_t message)
{
	//struct usbtouch_usb *usbtouch = usb_get_intfdata(intf);
	int ret=0;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	return ret;
}

static 
int core_resume(struct usb_interface *intf)
{	
	//struct usbtouch_usb *usbtouch = usb_get_intfdata(intf);
	int ret=0;

	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);
	
	return ret;
}


static
int __init core_init(void)
{
	printk("generic touchscreen usb driver, <@>.\n");

	return usb_register(&generic_ts_driver);
}

static
void __exit core_exit(void)
{
	PRINT_CORE_MSG("%s: >>>>>>>>>>>>>>>>>>>\n", __FUNCTION__);

	usb_deregister(&generic_ts_driver);
}

module_init(core_init);
module_exit(core_exit);

MODULE_AUTHOR("<@>");
MODULE_DESCRIPTION("generic touchscreen usb driver");
MODULE_LICENSE("GPL");


