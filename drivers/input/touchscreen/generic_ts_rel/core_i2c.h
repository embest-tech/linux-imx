#ifndef CORE_I2C_H
#define CORE_I2C_H


#include <linux/i2c.h>
#include <linux/input.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "platform.h"
#include "chip.h"
#include "touch.h"
#include "fsutils.h"
#include "enhance.h"

#define DRIVER_NAME		"generic_ts"

// ****************************************************************************
// Defines
// ****************************************************************************
enum enum_core_drv_state {
	CORE_STATE_UNKNOWN,
	CORE_STATE_INIT,
	CORE_STATE_NORMAL,
	CORE_STATE_SLEEP,
	CORE_STATE_DEBUG,
};

// ****************************************************************************
// Types
// ****************************************************************************
struct struct_core_i2c_var;
struct struct_core_i2c_ops {
	struct struct_platform_var* (*platform_var_init)(struct struct_platform_param *param);
	void (*platform_var_exit)(struct struct_platform_var *pdata);

	struct struct_chip_var* (*chip_var_init)(struct struct_chip_param *param);
	void (*chip_var_exit)(struct struct_chip_var *cdata);

	struct struct_touch_var* (*touch_var_init)(struct struct_touch_param *param);
	void (*touch_var_exit)(struct struct_touch_var *tdata);

	struct struct_fsutils_var* (*fsutils_var_init)(struct struct_fsutils_param *param);
	void (*fsutils_var_exit)(struct struct_fsutils_var *fdata);

	struct struct_enhance_var* (*enhance_var_init)(struct struct_enhance_param *param);
	void (*enhance_var_exit)(struct struct_enhance_var *edata);
};

struct struct_core_i2c_var {
	/* hw config */
	struct struct_platform_var	*pdata;

	/* chip config */
	struct struct_chip_var		*cdata;

	/* touch config */
	struct struct_touch_var		*tdata;

	/* fsys config */
	struct struct_fsutils_var	*fdata;

	/* enhance config */
	struct struct_enhance_var	*edata;

	/* driver status */
	char				state;

#ifdef CONFIG_HAS_EARLYSUSPEND
	// Early suspend
	struct early_suspend		early_suspend;
#endif

	// Work thread settings
	struct work_struct		event_work;
	struct workqueue_struct 	*workqueue;

	struct struct_core_i2c_ops ops;
};

// ****************************************************************************
// Globel or static variables
// ****************************************************************************
//extern struct i2c_board_info i2c_board_info[];

#endif
