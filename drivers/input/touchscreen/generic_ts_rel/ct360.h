#ifndef CT360_H
#define CT360_H

#include <linux/i2c.h>

/* max touch points supported */
#define CT360_POINT_NUM			5

#define CT360_CHN_X_MAX			17
#define CT360_CHN_Y_MAX			13
#define CT360_CHN_RES			64//32

#define CT360_ABS_X_MAX			1024
#define CT360_ABS_Y_MAX			600

#define CT360_FLASH_SECTOR_NUM		8
#define CT360_FLASH_SECTOR_SIZE		2048
#define CT360_FLASH_SOURCE_SIZE		8


//#define CT360_PTS_FORMAT_V01
#define CT360_PTS_FORMAT_V02


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
*/
struct struct_ct360_pts_data {
#ifdef CT360_PTS_FORMAT_V01
	unsigned char	status : 4; 	// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 4; 		// ID information, from 1 to CFG_MAX_POINT_NUM
#endif
	unsigned char	xhi;			// X coordinate Hi
	unsigned char	yhi;			// Y coordinate Hi
	unsigned char	ylo : 4;		// Y coordinate Lo
	unsigned char	xlo : 4;		// X coordinate Lo
#ifdef CT360_PTS_FORMAT_V02
	unsigned char	status : 4;		// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 4;			// ID information, from 1 to CFG_MAX_POINT_NUM
#endif
};

int ct360_get_bin_chksum(unsigned char *buf, unsigned char *bin);
int ct360_get_fw_chksum(struct i2c_client *client, unsigned char *buf);
int ct360_get_ver(struct i2c_client *client, unsigned char *buf);
int ct360_get_vendor(struct i2c_client *client, unsigned char *buf);
void ct360_go_sleep(struct i2c_client *client, unsigned char *buf);
int ct360_update_fw(struct i2c_client *client, unsigned char *buf);
void ct360_set_charger_on(struct i2c_client *client, unsigned char *buf);
void ct360_set_charger_off(struct i2c_client *client, unsigned char *buf);
#endif
