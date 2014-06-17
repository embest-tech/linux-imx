#ifndef CHIP_H
#define CHIP_H


#include "ct360.h"
#include "ct365.h"

#include "usbvc.h"



#define CHIP_XY_SWAP	0


struct struct_chip_ops {
	int (*get_bin_chksum)(unsigned char *buf, unsigned char *bin);
	int (*get_fw_chksum)(struct i2c_client *client, unsigned char *buf);
	int (*get_ver)(struct i2c_client *client, unsigned char *buf);
	int (*get_vendor)(struct i2c_client *client, unsigned char *buf);
	void (*go_sleep)(struct i2c_client *client, unsigned char *buf);
	int (*update_fw)(struct i2c_client *client, unsigned char *buf);
	void (*set_charger_on)(struct i2c_client *client, unsigned char *buf);
	void (*set_charger_off)(struct i2c_client *client, unsigned char *buf);

	int (*cpy_valid_pt)(int *x, int *y, char *id, void *pts_data);
};

struct struct_chip_var {
	int chip;		// CHIP ID
	int xy_swap;	// enable xy swap
	int x_max;		// max resolution of x
	int y_max;		// max resolution y y
	int chn_x_num;	// channel num of x
	int chn_y_num;	// channel num of y
	int chn_res;	// resolution of channel

	int bin_sz;		// fw bin size
	int pts_num;	// max point num of support
	int pt_sz;		// point size

	char *bin_data;	// fw bin data
	char *pts_data;	// point data

	struct struct_chip_ops ops;
};

struct struct_chip_param {
	/* parameters for chip */
	int pts_num;
	int chip;
	int xy_swap;
	int x_max;
	int y_max;
	int chn_x_num;
	int chn_y_num;
	int chn_res;
};


// ****************************************************************************
// Globel or static variables
// ****************************************************************************
enum enum_chip_var {
	CHIP_CT365 = 0x01,
	CHIP_CT360 = 0x02,
	CHIP_USBVC,
	CHIP_MAX,
};

// ****************************************************************************
// Function prototypes
// ****************************************************************************
struct struct_chip_var* chip_var_init(struct struct_chip_param *param);
void chip_var_exit(struct struct_chip_var *chip_data);
#endif
