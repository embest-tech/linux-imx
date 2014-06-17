#ifndef PLAT_USB_H
#define PLAT_USB_H


// ****************************************************************************
// Includes
// ****************************************************************************
#include <linux/usb.h>
#include <linux/hid.h>

// ****************************************************************************
// Defines
// ****************************************************************************
/* USB config */
#define PLATFORM_USB_IDVEND			0x0306
#define PLATFORM_USB_IDPROD			0xFF3F

// ****************************************************************************
// Types
// ****************************************************************************
// Platform usb data
struct struct_platform_usb_var;
struct struct_platform_usb_ops {
	void (*usb_proc_pkg)(char *dst, char *src, int len);
	//void (*usb_write)(char *buf, int len);
	void (*get_cfg)(struct struct_platform_usb_var *platform_usb_data);
	int (*set_dev)(struct struct_platform_usb_var *platform_usb_data);
	int (*get_rsrc)(struct struct_platform_usb_var *platform_usb_data);
	void (*put_rsrc)(struct struct_platform_usb_var *platform_usb_data);
	void (*hw_reset)(struct struct_platform_usb_var *platform_usb_data);
};

struct struct_platform_usb_var {
	/* Hardware IO settings */
	char * buf;
	dma_addr_t dma_addr;
	struct urb *irq;

	/* Communication settings */
	int idvend;
	int idprod;

	struct usb_device *udev;
	struct usb_interface *intf;
	struct usb_endpoint_descriptor *epdesc;

	/* Platform ops */
	struct struct_platform_usb_ops	ops;
};

// ****************************************************************************
// Globel or static variables
// ****************************************************************************


// ****************************************************************************
// Function prototypes
// ****************************************************************************
struct struct_platform_usb_var* platform_usb_var_init(void);
void platform_usb_var_exit(struct struct_platform_usb_var *platform_usb_data);

#endif
