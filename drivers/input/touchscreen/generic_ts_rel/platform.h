#ifndef PLATFORM_H
#define PLATFORM_H


// ****************************************************************************
// Includes
// ****************************************************************************
#include "plat_i2c.h"
#include "plat_usb.h"

// ****************************************************************************
// Defines
// ****************************************************************************

// ****************************************************************************
// Types
// ****************************************************************************

struct struct_platform_var;
struct struct_platform_ops {
	void* (*platform_bus_var_init)(void);
	void (*platform_bus_var_exit)(void* platform_bus_data);
};

struct struct_platform_var {
	int bustype;
	struct struct_platform_i2c_var *i2c;
	struct struct_platform_usb_var *usb;

	struct struct_platform_ops ops;
};

struct struct_platform_param {
	/* parameters for platform */
	int bustype;
};

enum enum_platform_bus_type {
	PLATFORM_BUS_I2C,
	PLATFORM_BUS_USB,
};
// ****************************************************************************
// Globel or static variables
// ****************************************************************************


// ****************************************************************************
// Function prototypes
// ****************************************************************************
struct struct_platform_var* platform_var_init(struct struct_platform_param *param);
void platform_var_exit(struct struct_platform_var *platform_data);

#endif
