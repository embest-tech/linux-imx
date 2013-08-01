#ifndef CORE_USB_H
#define CORE_USB_H


#include "platform.h"


#include "chip.h"
#include "touch.h"
#include "enhance.h"

#define DRIVER_NAME		"generic_ts"

// ****************************************************************************
// Defines
// ****************************************************************************
enum enum_core_usb_drv_state {
	CORE_USB_STATE_UNKNOWN,
	CORE_USB_STATE_INIT,
	CORE_USB_STATE_NORMAL,
	CORE_USB_STATE_SLEEP,
	CORE_USB_STATE_DEBUG,
};

enum enum_core_usb_fs_cmds {
	CORE_USB_FS_CHIP_ID,
	CORE_USB_FS_CHIP_RESET,
	CORE_USB_FS_FW_VER,
	CORE_USB_FS_FW_CHKSUM,
	CORE_USB_FS_FW_UPDATE,
	CORE_USB_FS_BIN_LOAD,
	CORE_USB_FS_BIN_VER,
	CORE_USB_FS_BIN_CHKSUM,
};

// ****************************************************************************
// Types
// ****************************************************************************
struct struct_core_usb_var;
struct struct_core_usb_ops {
	struct struct_platform_var* (*platform_var_init)(struct struct_platform_param *param);
	void (*platform_var_exit)(struct struct_platform_var *pdata);

	struct struct_chip_var* (*chip_var_init)(struct struct_chip_param *param);
	void (*chip_var_exit)(struct struct_chip_var *cdata);

	struct struct_touch_var* (*touch_var_init)(struct struct_touch_param *param);
	void (*touch_var_exit)(struct struct_touch_var *tdata);

	struct struct_enhance_var* (*enhance_var_init)(struct struct_enhance_param *param);
	void (*enhance_var_exit)(struct struct_enhance_var *edata);
};

struct struct_core_usb_var {
	/* hw config */
	struct struct_platform_var	*pdata;

	/* chip config */
	struct struct_chip_var		*cdata;

	/* touch config */
	struct struct_touch_var		*tdata;

	/* enhance config */
	struct struct_enhance_var	*edata;

	/* driver status */
	char				state;

	struct struct_core_usb_ops ops;
};

// ****************************************************************************
// Globel or static variables
// ****************************************************************************


#endif
