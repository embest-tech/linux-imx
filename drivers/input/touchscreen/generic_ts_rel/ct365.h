#ifndef CT365_H
#define CT365_H

#include <linux/i2c.h>

/* max touch points supported */
#define CT365_POINT_NUM			10

#define CT365_CHN_X_MAX			17
#define CT365_CHN_Y_MAX			13
#define CT365_CHN_RES			64

#define CT365_ABS_X_MAX			1024
#define CT365_ABS_Y_MAX			768

#define CT365_FLASH_SECTOR_NUM		256
#define CT365_FLASH_SECTOR_SIZE		128
#define CT365_FLASH_SOURCE_SIZE		8

//#define CT365_PTS_FORMAT_V01
#define CT365_PTS_FORMAT_V02

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

struct struct_ct365_pts_data {
#ifdef CT365_PTS_FORMAT_V01
	unsigned char	status : 3;		// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 5;			// ID information, from 1 to CFG_MAX_POINT_NUM
#endif
	unsigned char	xhi;			// X coordinate Hi
	unsigned char	yhi;			// Y coordinate Hi
	unsigned char	ylo : 4;		// Y coordinate Lo
	unsigned char	xlo : 4;		// X coordinate Lo
#ifdef CT365_PTS_FORMAT_V02
	unsigned char	status : 3;		// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 5;			// ID information, from 1 to CFG_MAX_POINT_NUM
#endif
	unsigned char	area;			// Touch area
	unsigned char	pressure;		// Touch Pressure
};

int ct365_get_bin_chksum(unsigned char *buf, unsigned char *bin);
int ct365_get_fw_chksum(struct i2c_client *client, unsigned char *buf);
int ct365_get_ver(struct i2c_client *client, unsigned char *buf);
int ct365_get_vendor(struct i2c_client *client, unsigned char *buf);
void ct365_go_sleep(struct i2c_client *client, unsigned char *buf);
int ct365_update_fw(struct i2c_client *client, unsigned char *buf);
void ct365_set_charger_on(struct i2c_client *client, unsigned char *buf);
void ct365_set_charger_off(struct i2c_client *client, unsigned char *buf);
#endif
