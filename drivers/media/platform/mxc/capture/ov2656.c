/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV2656_XCLK_MIN 6000000
#define OV2656_XCLK_MAX 24000000

#define OV2656_PIDH	0x300A
#define OV2656_PIDL	0x300B

enum ov2656_mode {
	ov2656_mode_MIN = 0,
	ov2656_mode_VGA_640_480 = 0,
	ov2656_mode_UXGA_1600_1200 = 1,
	ov2656_mode_MAX = 1
};

enum ov2656_frame_rate {
	ov2656_15_fps,
	ov2656_30_fps
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov2656_mode_info {
	enum ov2656_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
struct sensor_data ov2656_data;
static int pwn_gpio, rst_gpio;

static struct reg_value ov2656_setting_init_data[] = {
		/* reset */
		{0x3012,0x80, 0, 0},
		/* IO & Clock & Analog Setup */
		{0x308c,0x80, 0, 0},
	 	{0x308d,0x0e, 0, 0},
	 	{0x360b,0x00, 0, 0},
	 	{0x30b0,0xff, 0, 0},
	 	{0x30b1,0xff, 0 ,0},
		{0x30b2,0x04, 0, 0},

	 	{0x300e,0x34, 0, 0},
        {0x300f,0xa6, 0, 0},
	 	{0x3010,0x81, 0, 0},
	 	{0x3082,0x01, 0, 0},
	 	{0x30f4,0x01, 0, 0},
	 	{0x3091,0xc0, 0, 0},
	 	{0x30ac,0x42, 0, 0},

	 	{0x30d1,0x08, 0, 0},
	 	{0x30a8,0x55, 0, 0},
	 	{0x3015,0x02, 0, 0},
	 	{0x3093,0x00, 0, 0},
	 	{0x307e,0xe5, 0, 0},
	 	{0x3079,0x00, 0, 0},
	 	{0x30aa,0x42, 0, 0},
	 	{0x3017,0x40, 0, 0},
	 	{0x30f3,0x83, 0, 0},
        {0x306a,0x0c, 0, 0},
	 	{0x306d,0x00, 0, 0},
	 	{0x336a,0x3c, 0, 0},
	 	{0x3076,0x6a, 0, 0},
	 	{0x30d9,0x95, 0, 0},
        {0x3016,0x82, 0, 0},
	 	{0x3601,0x30, 0, 0},
	 	{0x304e,0x88, 0, 0},
	 	{0x30f1,0x82, 0, 0},
	 	{0x306f,0x14, 0, 0},

	 	{0x3012,0x10, 0, 0},
	 	{0x3011,0x01, 0, 0},
        {0x302a,0x02, 0, 0},
	 	{0x302b,0xe6, 0, 0},
        {0x3028,0x07, 0, 0},
	 	{0x3029,0x93, 0, 0},

        {0x3391,0x06, 0, 0},
        {0x3394,0x38, 0, 0},
	 	{0x3395,0x38, 0, 0},

	 	/*AEC/AGC*/
	 	{0x3013,0xf7, 0, 0},
	 	{0x3018,0x78, 0, 0},
	 	{0x3019,0x68, 0, 0},
	 	{0x301a,0xd4, 0, 0},

		/*D5060*/
	 	{0x30af,0x00, 0, 0},
	 	{0x3048,0x1f, 0, 0},
	 	{0x3049,0x4e, 0, 0},
	 	{0x304a,0x20, 0, 0},
	 	{0x304f,0x20, 0, 0},
	 	{0x304b,0x02, 0, 0},
	 	{0x304c,0x00, 0, 0},
	 	{0x304d,0x02, 0, 0},
	 	{0x304f,0x20, 0, 0},
	 	{0x30a3,0x10, 0, 0},
	 	{0x3013,0xf7, 0, 0},
	 	{0x3014,0x84, 0, 0},
	 	{0x3071,0x00, 0, 0},
	 	{0x3070,0x5d, 0, 0},
	 	{0x3073,0x00, 0, 0},
	 	{0x3072,0x4d, 0, 0},
	 	{0x301c,0x07, 0, 0},
	 	{0x301d,0x08, 0, 0},
	 	{0x304d,0x42, 0, 0},
	 	{0x304a,0x40, 0, 0},
	 	{0x304f,0x40, 0, 0},
	 	{0x3095,0x07, 0, 0},
	 	{0x3096,0x16, 0, 0},
	 	{0x3097,0x1d, 0, 0},

		/*Window Setup*/
		{0x3020,0x01, 0, 0}, 
		{0x3021,0x18, 0, 0},
		{0x3022,0x00, 0, 0},
		{0x3023,0x06, 0, 0},
		{0x3024,0x06, 0, 0},
		{0x3025,0x58, 0, 0},
		{0x3026,0x02, 0, 0},
		{0x3027,0x5e/*0x58*/, 0, 0},	//??
	 	{0x3088,0x02, 0, 0},
	 	{0x3089,0x80, 0, 0},
	 	{0x308a,0x01, 0, 0},
	 	{0x308b,0xe0, 0, 0},
	 	{0x3316,0x64, 0, 0},
	 	{0x3317,0x25, 0, 0},
	 	{0x3318,0x80, 0, 0},
	 	{0x3319,0x08/*0x00*/, 0, 0},	//??
		{0x331a,0x64, 0, 0},
		{0x331b,0x4b, 0, 0},
		{0x331c,0x00, 0, 0},
		{0x331d,0x38/*0x00*/, 0, 0},	//??
		{0x3100,0x00, 0, 0},

	    /*AWB*/
		{0x3320,0xfa, 0, 0},   
		{0x3321,0x11, 0, 0},   
		{0x3322,0x92, 0, 0},   
		{0x3323,0x01, 0, 0},   
		{0x3324,0x97, 0, 0},   
		{0x3325,0x02, 0, 0},   
		{0x3326,0xff, 0, 0},   
		{0x3327,0x0c, 0, 0},   
		{0x3328,0x10, 0, 0},   
		{0x3329,0x10, 0, 0},   
		{0x332a,0x54, 0, 0},   
		{0x332b,0x52, 0, 0},  
		{0x332c,0xbe, 0, 0},   
		{0x332d,0xe1, 0, 0},  
		{0x332e,0x3a, 0, 0}, 
		{0x332f,0x36, 0, 0},   
		{0x3330,0x4d, 0, 0},   
		{0x3331,0x44, 0, 0},   
		{0x3332,0xf8, 0, 0},   
		{0x3333,0x0a, 0, 0},   
		{0x3334,0xf0, 0, 0},   
		{0x3335,0xf0, 0, 0},   
		{0x3336,0xf0, 0, 0},   
		{0x3337,0x40, 0, 0},   
		{0x3338,0x40, 0, 0},   
		{0x3339,0x40, 0, 0},   
		{0x333a,0x00, 0, 0},   
		{0x333b,0x00, 0, 0},   
		/*Color Matrix*/
	 	{0x3380,0x28, 0, 0},
		{0x3381,0x48, 0, 0}, 
		{0x3382,0x10, 0, 0},
		{0x3383,0x22, 0, 0},  
		{0x3384,0xc0, 0, 0},  
		{0x3385,0xe2, 0, 0},  
		{0x3386,0xe2, 0, 0},  
		{0x3387,0xf2, 0, 0},  
		{0x3388,0x10, 0, 0},  
		{0x3389,0x98, 0, 0},
		{0x338a,0x00, 0, 0},	
        /*Gamma*/
        {0x3340,0x04, 0, 0},  
		{0x3341,0x07, 0, 0},  
		{0x3342,0x19, 0, 0},
		{0x3343,0x34, 0, 0},
		{0x3344,0x4a, 0, 0},  
		{0x3345,0x5a, 0, 0},  
		{0x3346,0x67, 0, 0},  
		{0x3347,0x71, 0, 0},  
		{0x3348,0x7c, 0, 0},  
		{0x3349,0x8c, 0, 0},
		{0x334a,0x9b, 0, 0},
		{0x334b,0xa9, 0, 0},
		{0x334c,0xc0, 0, 0},
		{0x334d,0xd5, 0, 0},
		{0x334e,0xe8, 0, 0},
        {0x334f,0x20, 0, 0},
	    /*Lens correction{largon 9310}*/
        {0x3090,0x03, 0, 0},
        {0x307c,0x10/*0x11*/, 0, 0},	//??
	    /*R*/
        {0x3350,0x33, 0, 0},
	    {0x3351,0x28, 0, 0},
		{0x3352,0x00, 0, 0},
		{0x3353,0x14, 0, 0},
		{0x3354,0x00, 0, 0},
		{0x3355,0x85, 0, 0},
		/*G*/
		{0x3356,0x35, 0, 0},
		{0x3357,0x28, 0, 0},
		{0x3358,0x00, 0, 0},
		{0x3359,0x13, 0, 0},
		{0x335a,0x00, 0, 0},
		{0x335b,0x85, 0, 0},
		/*B*/
		{0x335c,0x34, 0, 0},
		{0x335d,0x28, 0, 0},
		{0x335e,0x00, 0, 0},
		{0x335f,0x13, 0, 0},
		{0x3360,0x00, 0, 0},
		{0x3361,0x85, 0, 0},
     	{0x3363,0x70, 0, 0},
     	{0x3364,0x7f, 0, 0},
    	{0x3365,0x00, 0, 0},
	    {0x3366,0x00, 0, 0},
        {0x3362,0x90, 0, 0}, 
		/*UVadjust*/
	 	{0x3301,0xff, 0, 0},
		{0x338B,0x13, 0, 0},
		{0x338c,0x10, 0, 0},
		{0x338d,0x40, 0, 0},

		/*Sharpness/De-noise*/
		{0x3370,0xd0, 0, 0},
		{0x3371,0x00, 0, 0},
		{0x3372,0x00, 0, 0},
		{0x3373,0x30, 0, 0},
		{0x3374,0x10, 0, 0},
		{0x3375,0x10, 0, 0},
		{0x3376,0x07, 0, 0},
		{0x3377,0x00, 0, 0},
		{0x3378,0x04, 0, 0},
       	{0x3379,0x40, 0, 0},

		/*BLC*/
        {0x3069,0x86, 0, 0}, 	
		{0x3087,0x02, 0, 0},

		/*Other functions*/
		{0x3300,0xf8, 0, 0},
		{0x3302,0x11, 0, 0},
		{0x3400,0x00/*0x02*/, 0, 0},	//??
		{0x3606,0x20, 0, 0},
		{0x3601,0x30, 0, 0},
		{0x30f3,0x83, 0, 0},
		{0x304e,0x88, 0, 0},

        {0x3015,0x02, 0, 0}, 
		{0x302d,0x00, 0, 0}, 
		{0x302e,0x00, 0, 0},
	    {0x3306,0x00, 0, 0},

        {0x363b,0x01, 0, 0},
	    {0x363c,0xf2, 0, 0},
	
		{0x3086,0x0f, 0, 0},
		{0x3086,0x00, 0, 0},

		{0x30a1,0x41, 0, 0},
		{0x30a3,0x80, 0, 0},
		{0x30a8,0x56, 0, 0}, 
		{0x30aa,0x72, 0, 0}, 
		{0x30af,0x10, 0, 0}, 
		{0x30b2,0x2c, 0, 0}, 
		{0x30d9,0x8c, 0, 0},
};

static struct reg_value ov2656_setting_VGA_640_480[] = {
    {0x300E, 0x34, 0, 0},
    {0x3011, 0x01, 0, 0},
    {0x3012, 0x10, 0, 0},
    {0x302a, 0x02, 0, 0},
    {0x302b, 0xE6, 0, 0},
    {0x306f, 0x14, 0, 0},
    {0x3362, 0x90, 0, 0},

    {0x3070, 0x5D, 0, 0},
    {0x3072, 0x5D, 0, 0},
    {0x301c, 0x07, 0, 0},
    {0x301d, 0x07, 0, 0},

    {0x3020, 0x01, 0, 0},
    {0x3021, 0x18, 0, 0},
    {0x3022, 0x00, 0, 0},
    {0x3023, 0x06, 0, 0},
    {0x3024, 0x06, 0, 0},
    {0x3025, 0x58, 0, 0},
    {0x3026, 0x02, 0, 0},
    {0x3027, 0x61, 0, 0},
    {0x3088, 0x02, 0, 0},
    {0x3089, 0x80, 0, 0},
    {0x308A, 0x01, 0, 0},
    {0x308B, 0xe0, 0, 0},
    {0x3316, 0x64, 0, 0},
    {0x3317, 0x25, 0, 0},
    {0x3318, 0x80, 0, 0},
    {0x3319, 0x08, 0, 0},
    {0x331A, 0x28, 0, 0},
    {0x331B, 0x1e, 0, 0},
    {0x331C, 0x08, 0, 0},
    {0x331D, 0x38, 0, 0},
    {0x3302, 0x11, 0, 0},
};

static struct reg_value ov2656_setting_UXGA_1600_1200[] = {
    {0x300E, 0x34, 0, 0},
    {0x3011, 0x01, 0, 0},
    {0x3012, 0x00, 0, 0},
    {0x302a, 0x05, 0, 0},
    {0x302b, 0xCB, 0, 0},
    {0x306f, 0x54, 0, 0},
    {0x3362, 0x80, 0, 0},

    {0x3070, 0x5d, 0, 0},
    {0x3072, 0x5d, 0, 0},
    {0x301c, 0x0f, 0, 0},
    {0x301d, 0x0f, 0, 0},

    {0x3020, 0x01, 0, 0},
    {0x3021, 0x18, 0, 0},
    {0x3022, 0x00, 0, 0},
    {0x3023, 0x0A, 0, 0},
    {0x3024, 0x06, 0, 0},
    {0x3025, 0x58, 0, 0},
    {0x3026, 0x04, 0, 0},
    {0x3027, 0xbc, 0, 0},
    {0x3088, 0x06, 0, 0},
    {0x3089, 0x40, 0, 0},
    {0x308A, 0x04, 0, 0},
    {0x308B, 0xB0, 0, 0},
    {0x3316, 0x64, 0, 0},
    {0x3317, 0x4B, 0, 0},
    {0x3318, 0x00, 0, 0},
    {0x3319, 0x6C, 0, 0},
    {0x331A, 0x64, 0, 0},
    {0x331B, 0x4B, 0, 0},
    {0x331C, 0x00, 0, 0},
    {0x331D, 0x6C, 0, 0},
    {0x3302, 0x01, 0, 0},
};

static struct ov2656_mode_info ov2656_mode_info_data[ov2656_mode_MAX + 1] = {
	{
		ov2656_mode_VGA_640_480,    640,  480,
		ov2656_setting_VGA_640_480,
		ARRAY_SIZE(ov2656_setting_VGA_640_480)
	},
	{
		ov2656_mode_UXGA_1600_1200,    1600,  1200,
        ov2656_setting_UXGA_1600_1200,
        ARRAY_SIZE(ov2656_setting_UXGA_1600_1200)
	},
};

static int ov2656_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov2656_remove(struct i2c_client *client);

static s32 ov2656_read_reg(u16 reg, u8 *val);
static s32 ov2656_write_reg(u16 reg, u8 val);

static const struct i2c_device_id ov2656_id[] = {
	{"ov2656", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov2656_id);

static struct i2c_driver ov2656_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov2656",
		  },
	.probe  = ov2656_probe,
	.remove = ov2656_remove,
	.id_table = ov2656_id,
};

static inline void ov2656_power_down(int enable)
{
    gpio_set_value(pwn_gpio, enable);

    msleep(2);
}

static inline void ov2656_reset(void)
{
    /* camera reset */
    gpio_set_value(rst_gpio, 1);

    /* camera power down */
    gpio_set_value(pwn_gpio, 1);
    msleep(5);
    gpio_set_value(pwn_gpio, 0);
    msleep(5);
    gpio_set_value(rst_gpio, 0);
    msleep(1);
    gpio_set_value(rst_gpio, 1);
    msleep(5);
    gpio_set_value(pwn_gpio, 1);
}

static s32 ov2656_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(ov2656_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 ov2656_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ov2656_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov2656_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

static int ov2656_change_mode(enum ov2656_frame_rate frame_rate,
		enum ov2656_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

	if (mode > ov2656_mode_MAX || mode < ov2656_mode_MIN) {
		pr_err("Wrong ov2656 mode detected!\n");
		return -1;
	}

	pModeSetting = ov2656_mode_info_data[mode].init_data_ptr;
	iModeSettingArySize = ov2656_mode_info_data[mode].init_data_size;

    ov2656_data.pix.width = ov2656_mode_info_data[mode].width;
    ov2656_data.pix.height = ov2656_mode_info_data[mode].height;

	if (ov2656_data.pix.width == 0 || ov2656_data.pix.height == 0 ||
			pModeSetting == NULL || iModeSettingArySize == 0)
		return -EINVAL;

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov2656_read_reg(RegAddr, &RegVal);
			if (retval < 0) {
				pr_err("read reg error addr=0x%x", RegAddr);
				goto err;
			}

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov2656_write_reg(RegAddr, Val);
		if (retval < 0) {
			pr_err("write reg error addr=0x%x", RegAddr);
			goto err;
		}

		if (Delay_ms)
			msleep(Delay_ms);
	}

	/* wait for stable */
	msleep(1500);

err:
	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov2656_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ov2656_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV2656_XCLK_MIN;
	p->u.bt656.clock_max = OV2656_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {
		/* Make sure power on */
		ov2656_power_down(0);
	} else if (!on && sensor->on) {
		ov2656_power_down(1);
	}

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov2656_frame_rate frame_rate;
	int ret = 0;

	/* Make sure power on */
	ov2656_power_down(0);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			frame_rate = ov2656_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ov2656_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		ret =  ov2656_change_mode(frame_rate, a->parm.capture.capturemode);
		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = (u32)a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{

	return 0;	
}

static int ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;
	int ret = 0;

	ret = ioctl_try_fmt_cap(s, f);
	if (ret)
		return ret;

	sensor->pix = f->fmt.pix;

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov2656_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov2656_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov2656_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov2656_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov2656_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov2656_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov2656_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In ov2656:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ov2656_mode_MAX)
		return -EINVAL;

//	fsize->pixel_format = ov2656_data.pix.pixelformat;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width =
			    ov2656_mode_info_data[fsize->index].width;
	fsize->discrete.height =
			    ov2656_mode_info_data[fsize->index].height;
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ov2656_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > ov2656_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ov2656_data.pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov2656_frame_rate frame_rate;

	ov2656_data.on = true;

	/* mclk */
	tgt_xclk = ov2656_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV2656_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV2656_XCLK_MIN);
	ov2656_data.mclk = tgt_xclk;

	pr_debug("Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	clk_set_rate(ov2656_data.sensor_clk, ov2656_data.mclk);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ov2656_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov2656_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	pModeSetting = ov2656_setting_init_data;
	iModeSettingArySize = ARRAY_SIZE(ov2656_setting_init_data);

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;
		if (Mask) {
			retval = ov2656_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov2656_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov2656_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, 
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave ov2656_slave = {
	.ioctls = ov2656_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov2656_ioctl_desc),
};

static struct v4l2_int_device ov2656_int_device = {
	.module = THIS_MODULE,
	.name = "ov2656",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov2656_slave,
	},
};

/*!
 * ov2656 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov2656_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
    struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high = 0, chip_id_low = 0;

    /* pinctrl */
    pinctrl = devm_pinctrl_get_select_default(dev);
    if (IS_ERR(pinctrl)) {
        dev_err(dev, "setup pinctrl failed\n");
        return PTR_ERR(pinctrl);
    }
	
    /* request power down pin */
    pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
    if (!gpio_is_valid(pwn_gpio)) {
        dev_err(dev, "no sensor pwdn pin available\n");
        return -ENODEV;
    }
    retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
                    "ov2656_pwdn");
    if (retval < 0)
        return retval;

    /* request reset pin */
    rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
    if (!gpio_is_valid(rst_gpio)) {
        dev_err(dev, "no sensor reset pin available\n");
        return -EINVAL;
    }
    retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
                    "ov2656_reset");
    if (retval < 0)
        return retval;

	/* Set initial values for the sensor struct. */
	memset(&ov2656_data, 0, sizeof(ov2656_data));
    ov2656_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
    if (IS_ERR(ov2656_data.sensor_clk)) {
        dev_err(dev, "get mclk failed\n");
        return PTR_ERR(ov2656_data.sensor_clk);
    }

    retval = of_property_read_u32(dev->of_node, "mclk",
                    &ov2656_data.mclk);
    if (retval) {
        dev_err(dev, "mclk frequency is invalid\n");
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "mclk_source",
                    (u32 *) &(ov2656_data.mclk_source));
    if (retval) {
        dev_err(dev, "mclk_source invalid\n");
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "csi_id",
                    &(ov2656_data.csi));
    if (retval) {
        dev_err(dev, "csi_id invalid\n");
        return retval;
    }

    clk_prepare_enable(ov2656_data.sensor_clk);
		

	ov2656_data.io_init = ov2656_reset;
	ov2656_data.i2c_client = client;
	ov2656_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ov2656_data.pix.width = 640;
	ov2656_data.pix.height = 480;
	ov2656_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov2656_data.streamcap.capturemode = 0;
	ov2656_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov2656_data.streamcap.timeperframe.numerator = 1;

	ov2656_reset();
	
	ov2656_power_down(0);
	
    retval = ov2656_read_reg(OV2656_PIDH, &chip_id_high);
    if (retval < 0 || chip_id_high != 0x26) {
		clk_disable_unprepare(ov2656_data.sensor_clk);
        pr_warning("camera ov2656 is not found\n");
        retval = -ENODEV;
        goto err;
    }

    retval = ov2656_read_reg(OV2656_PIDL, &chip_id_low);
    if (retval < 0 || chip_id_low != 0x56) {
		clk_disable_unprepare(ov2656_data.sensor_clk);
        pr_warning("camera ov2656 is not found\n");
        retval = -ENODEV;
        goto err;
    }

	pr_info("camera ov2656 is found\n");

    ov2656_power_down(1);

    clk_disable_unprepare(ov2656_data.sensor_clk);

	ov2656_int_device.priv = &ov2656_data;
	retval = v4l2_int_device_register(&ov2656_int_device);

err:
	return retval;
}

/*!
 * ov2656 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov2656_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ov2656_int_device);

	return 0;
}

/*!
 * ov2656 init function
 * Called by insmod ov2656_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov2656_init(void)
{
	u8 err;

	err = i2c_add_driver(&ov2656_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * OV2656 cleanup function
 * Called on rmmod ov2656_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov2656_clean(void)
{
	i2c_del_driver(&ov2656_i2c_driver);
}

module_init(ov2656_init);
module_exit(ov2656_clean);

MODULE_AUTHOR("Embest, Inc.");
MODULE_DESCRIPTION("OV2656 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
