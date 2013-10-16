#ifndef USBVC_H
#define USBVC_H

//#include <linux/i2c.h>

/* max touch points supported */
#define USBVC_POINT_NUM			10

#define USBVC_CHN_X_MAX			17
#define USBVC_CHN_Y_MAX			13
#define USBVC_CHN_RES			64//32

#define USBVC_ABS_X_MAX			1024
#define USBVC_ABS_Y_MAX			768

#define USBVC_FLASH_SECTOR_NUM		256
#define USBVC_FLASH_SECTOR_SIZE		128
#define USBVC_FLASH_SOURCE_SIZE		8

/* data structure of point event */
/* Old Touch Points Protocol
---------+-+-+-+-+-+-+-+-+
Byte0|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Finger ID|Statu|
---------+-+-+-+-+-+-+-+-+
Byte1|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |X High         |
---------+-+-+-+-+-+-+-+-+
Byte2|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Y High         |
---------+-+-+-+-+-+-+-+-+
Byte3|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |X Low  |X High |
---------+-+-+-+-+-+-+-+-+
Byte4|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Area           |
---------+-+-+-+-+-+-+-+-+
Byte5|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Pressure       |
---------+-+-+-+-+-+-+-+-+
*/
/* New Touch Points Protocol
---------+-+-+-+-+-+-+-+-+
Byte0|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |X High         |
---------+-+-+-+-+-+-+-+-+
Byte1|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Y High         |
---------+-+-+-+-+-+-+-+-+
Byte2|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |X Low  |X High |
---------+-+-+-+-+-+-+-+-+
Byte3|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Finger ID|Statu|
---------+-+-+-+-+-+-+-+-+
Byte4|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Area           |
---------+-+-+-+-+-+-+-+-+
Byte5|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Pressure       |
---------+-+-+-+-+-+-+-+-+
*/

struct struct_usbvc_pts_data {
//#ifndef CONFIG_TOUCHSCREEN_GENERIC_TS_MISC_PTS_FORMAT_V01
	unsigned char	status;		// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id;			// ID information, from 1 to CFG_MAX_POINT_NUM
//#endif
	unsigned char	xlo;		// X coordinate Lo
	unsigned char	xhi;			// X coordinate Hi
	unsigned char	ylo;		// Y coordinate Lo
	unsigned char	yhi;			// Y coordinate Hi
	
	
//#ifdef CONFIG_TOUCHSCREEN_GENERIC_TS_MISC_PTS_FORMAT_V01
//	unsigned char	status : 3;		// Action information, 1: Down; 2: Move; 3: Up
//	unsigned char	id : 5;			// ID information, from 1 to CFG_MAX_POINT_NUM
//#endif
//	unsigned char	area;			// Touch area
//	unsigned char	pressure;		// Touch Pressure
};

//int ct365_get_bin_chksum(unsigned char *buf, unsigned char *bin);
//int ct365_get_fw_chksum(struct i2c_client *client, unsigned char *buf);
//int ct365_get_ver(struct i2c_client *client, unsigned char *buf);
//int ct365_get_vendor(struct i2c_client *client, unsigned char *buf);
//void ct365_go_sleep(struct i2c_client *client, unsigned char *buf);
//int ct365_update_fw(struct i2c_client *client, unsigned char *buf);
//void ct365_set_adapter_on(struct i2c_client *client, unsigned char *buf);
//void ct365_set_adapter_off(struct i2c_client *client, unsigned char *buf);
#endif
