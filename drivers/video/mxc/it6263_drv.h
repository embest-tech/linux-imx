/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _IT626X_H_
#define _IT626X_H_

/////////////////////////////////////////
// DDC Address
/////////////////////////////////////////
#define DDC_HDCP_ADDRESS 0x74
#define DDC_EDID_ADDRESS 0xA0
#define DDC_FIFO_MAXREQ 0x20

// I2C address
//#define HDMI_TX_I2C_SLAVE_ADDR 0x98

////////////////////////////////////////
//type define
////////////////////////////////////////

#define abso(x) (((x)>0)?(x):(-(x)))
typedef unsigned int UINT, uint;
typedef bool BOOL;
typedef char CHAR, *PCHAR;
typedef unsigned char uchar, *puchar;
typedef unsigned char UCHAR, *PUCHAR;
typedef short SHORT, *PSHORT;
typedef unsigned short ushort, *pushort;
typedef unsigned short USHORT, *PUSHORT;
typedef long LONG, *PLONG;
typedef unsigned long ulong, *pulong;
typedef unsigned long ULONG, *PULONG;
typedef unsigned char _u8, *p_u8;
typedef unsigned short _u16, *p_u16;
typedef unsigned long _u32, *p_u32;
typedef signed char _s8, *p_s8;
typedef signed short _s16, *p_s16;
typedef signed long _s32, *p_s32;

#define FALSE 0
#define TRUE 1

//#define SUCCESS 0
#define FAIL -1

#define ON 1
#define OFF 0

#define LO_ACTIVE TRUE
#define HI_ACTIVE FALSE
#define Debug_message
#define PR_INFO(fmt, ...) printk("IT6263:"fmt, ##__VA_ARGS__)
#define HDMITX_DEBUG_PRINTF(x) PR_INFO x
typedef enum tagHDMI_Aspec { HDMI_4x3, HDMI_16x9
} HDMI_Aspec;
typedef enum tagHDMI_OutputColorMode { HDMI_RGB444, HDMI_YUV444, HDMI_YUV422
} HDMI_OutputColorMode;
typedef enum tagHDMI_Colorimetry { HDMI_ITU601, HDMI_ITU709
} HDMI_Colorimetry;
typedef enum _SYS_STATUS { ER_SUCCESS = 0, ER_FAIL, ER_RESERVED
} SYS_STATUS;

///////////////////////////////////////////////////////////////////////
//Config
///////////////////////////////////////////////////////////////////////
#define SUPPORT_EDID
//#define SUPPORT_HDCP
#define SUPPORT_INPUTRGB
//#define Powerdown
#define SUPPORT_LVDS_6_BIT
//#define SUPPORT_LVDS_8_BIT
//#define SUPPORT_LVDS_10_BIT

//#define SUPPORT_INPUTYUV444
//#define SUPPORT_INPUTYUV422
//#define IT626X_SSC
//#define SUPPORT_HBR_AUDIO
#define USE_SPDIF_CHSTAT
#define SUPPORT_MAP3
//#define SUPPORT_DISO
//#define SUPPORT_REGEN_TIMING
//#define SUPPORT_SPLASH
#define SUPPORT_AUDIO_MONITOR
#define SUPPORT_AUDI_AudSWL 24	//18;20;24
//#define Force_CTS

#define AudioOutDelayCnt 36	//60

// #define SUPPORT_HBR_AUDIO
// #define SUPPORT_I2S_AUDIO
// #define SUPPORT_SPDIF_AUDIO

// modify the input audio sample frequence here.
#define INPUT_AUDIO_SAMPLE_FREQ 48000L
#define INPUT_SAMPLE_FREQ AUDFS_48KHz

// #define INPUT_AUDIO_SAMPLE_FREQ 32000L
// #define INPUT_SAMPLE_FREQ AUDFS_32KHz

// #define INPUT_AUDIO_SAMPLE_FREQ 44100L
// #define INPUT_SAMPLE_FREQ AUDFS_44p1KHz

// #define INPUT_AUDIO_SAMPLE_FREQ 88200L
// #define INPUT_SAMPLE_FREQ AUDFS_88p2KHz

// #define INPUT_AUDIO_SAMPLE_FREQ 96000L
// #define INPUT_SAMPLE_FREQ AUDFS_96KHz

// #define INPUT_AUDIO_SAMPLE_FREQ 176400L
// #define INPUT_SAMPLE_FREQ AUDFS_176p4KHz

// #define INPUT_AUDIO_SAMPLE_FREQ 192000L
// #define INPUT_SAMPLE_FREQ AUDFS_192KHz

#define OUTPUT_CHANNEL 2

//#define SUPPORT_GPIO_MUTE

#ifndef _MCU_8051_		// DSSSHA need large computation data rather than 8051 supported.
#define SUPPORT_DSSSHA
#endif /*  */

#if defined(SUPPORT_INPUTYUV444) || defined(SUPPORT_INPUTYUV422)
#define SUPPORT_INPUTYUV
#endif /*  */
///////////////////////////////////////////////////////////////////////
// Output Mode Type
///////////////////////////////////////////////////////////////////////

#define RES_ASPEC_4x3 0
#define RES_ASPEC_16x9 1
#define F_MODE_REPT_NO 0
#define F_MODE_REPT_TWICE 1
#define F_MODE_REPT_QUATRO 3
#define F_MODE_CSC_ITU601 0
#define F_MODE_CSC_ITU709 1

///////////////////////////////////////////////////////////////////////
//HDMITX.h
///////////////////////////////////////////////////////////////////////

#define HDMITX_INSTANCE_MAX 1

#define SIZEOF_CSCMTX 18
#define SIZEOF_CSCGAIN 6
#define SIZEOF_CSCOFFSET 3

///////////////////////////////////////////////////////////////////////
// Output Mode Type
///////////////////////////////////////////////////////////////////////

#define RES_ASPEC_4x3 0
#define RES_ASPEC_16x9 1
#define F_MODE_REPT_NO 0
#define F_MODE_REPT_TWICE 1
#define F_MODE_REPT_QUATRO 3
#define F_MODE_CSC_ITU601 0
#define F_MODE_CSC_ITU709 1

///////////////////////////////////////////////////////////////////////
// ROM OFFSET
///////////////////////////////////////////////////////////////////////
#define ROMOFF_INT_TYPE 0
#define ROMOFF_INPUT_VIDEO_TYPE 1
#define ROMOFF_OUTPUT_AUDIO_MODE 8
#define ROMOFF_AUDIO_CH_SWAP 9

#define TIMER_LOOP_LEN 10
#define MS(x) (((x)+(TIMER_LOOP_LEN-1))/TIMER_LOOP_LEN);	// for timer loop
typedef enum { PCLK_LOW = 0, PCLK_MEDIUM, PCLK_HIGH
} VIDEOPCLKLEVEL;

///////////////////////////////////////////////////////////////////////
// Video Data Type
///////////////////////////////////////////////////////////////////////
#define F_MODE_RGB24  0
#define F_MODE_RGB444  0
#define F_MODE_YUV422 1
#define F_MODE_YUV444 2
#define F_MODE_CLRMOD_MASK 3

#define F_MODE_INTERLACE  1

#define F_MODE_ITU709  (1<<4)
#define F_MODE_ITU601  0

#define F_MODE_0_255   0
#define F_MODE_16_235  (1<<5)

#define F_MODE_EN_UDFILT (1<<6)	// output mode only,and loaded from EEPROM
#define F_MODE_EN_DITHER (1<<7)	// output mode only,and loaded from EEPROM
typedef union _VideoFormatCode {
	struct _VFC {
		_u8 colorfmt:2;
		_u8 interlace:1;
		_u8 Colorimetry:1;
		_u8 Quantization:1;
		_u8 UpDownFilter:1;
		_u8 Dither:1;
	} VFCCode;
	unsigned char VFCByte;
} VideoFormatCode;
typedef enum tagHDMI_Video_Type { HDMI_Unknown = 0, HDMI_640x480p60 =
	    1, HDMI_480p60, HDMI_480p60_16x9, HDMI_720p60, HDMI_1080i60,
	HDMI_480i60, HDMI_480i60_16x9, HDMI_1080p60 =
	    16, HDMI_576p50, HDMI_576p50_16x9, HDMI_720p50, HDMI_1080i50,
	HDMI_576i50, HDMI_576i50_16x9, HDMI_1080p50 =
	    31, HDMI_1080p24, HDMI_1080p48, HDMI_1080p25, HDMI_1080p30,
	HDMI_720p60_FP3D = 0x84, HDMI_720p50_FP3D =
	    0x93, HDMI_1080p24_FP3D = 0xA0,
} HDMI_Video_Type;

#define T_MODE_CCIR656 (1<<0)
#define T_MODE_SYNCEMB (1<<1)
#define T_MODE_INDDR   (1<<2)
#define T_MODE_PCLKDIV2 (1<<3)
#define T_MODE_DEGEN (1<<4)
#define T_MODE_SYNCGEN (1<<5)
//////////////////////////////////////////////////////////////////
// Audio relate definition and macro.
//////////////////////////////////////////////////////////////////

// 2008/08/15 added by jj_tseng@chipadvanced
#define F_AUDIO_ON  (1<<7)
#define F_AUDIO_HBR (1<<6)
#define F_AUDIO_DSD (1<<5)
#define F_AUDIO_NLPCM (1<<4)
#define F_AUDIO_LAYOUT_1 (1<<3)
#define F_AUDIO_LAYOUT_0 (0<<3)

// HBR - 1100
// DSD - 1010
// NLPCM - 1001
// LPCM - 1000

#define T_AUDIO_MASK 0xF0
#define T_AUDIO_OFF 0
#define T_AUDIO_HBR (F_AUDIO_ON|F_AUDIO_HBR)
#define T_AUDIO_DSD (F_AUDIO_ON|F_AUDIO_DSD)
#define T_AUDIO_NLPCM (F_AUDIO_ON|F_AUDIO_NLPCM)
#define T_AUDIO_LPCM (F_AUDIO_ON)

// for sample clock
#define AUDFS_22p05KHz  4
#define AUDFS_44p1KHz 0
#define AUDFS_88p2KHz 8
#define AUDFS_176p4KHz    12

#define AUDFS_24KHz  6
#define AUDFS_48KHz  2
#define AUDFS_96KHz  10
#define AUDFS_192KHz 14

#define AUDFS_768KHz 9

#define AUDFS_32KHz  3
#define AUDFS_OTHER    1

// Audio Enable
#define ENABLE_SPDIF    (1<<4)
#define ENABLE_I2S_SRC3  (1<<3)
#define ENABLE_I2S_SRC2  (1<<2)
#define ENABLE_I2S_SRC1  (1<<1)
#define ENABLE_I2S_SRC0  (1<<0)

#define AUD_SWL_NOINDICATE  0x0
#define AUD_SWL_16          0x2
#define AUD_SWL_17          0xC
#define AUD_SWL_18          0x4
#define AUD_SWL_20          0xA	// for maximum 20 bit
#define AUD_SWL_21          0xD
#define AUD_SWL_22          0x5
#define AUD_SWL_23          0x9
#define AUD_SWL_24          0xB

#define Frame_Packing 0
#define Top_and_Botton 6
#define Side_by_Side 8

/////////////////////////////////////////////////////////////////////
// Packet and Info Frame definition and datastructure.
/////////////////////////////////////////////////////////////////////

#define VENDORSPEC_INFOFRAME_TYPE 0x01
#define AVI_INFOFRAME_TYPE  0x02
#define SPD_INFOFRAME_TYPE 0x03
#define AUDIO_INFOFRAME_TYPE 0x04
#define MPEG_INFOFRAME_TYPE 0x05

#define VENDORSPEC_INFOFRAME_VER 0x01
#define AVI_INFOFRAME_VER  0x02
#define SPD_INFOFRAME_VER 0x01
#define AUDIO_INFOFRAME_VER 0x01
#define MPEG_INFOFRAME_VER 0x01

#define VENDORSPEC_INFOFRAME_LEN 6
#define AVI_INFOFRAME_LEN 13
#define SPD_INFOFRAME_LEN 25
#define AUDIO_INFOFRAME_LEN 10
#define MPEG_INFOFRAME_LEN 10

#define ACP_PKT_LEN 9
#define ISRC1_PKT_LEN 16
#define ISRC2_PKT_LEN 16
typedef union _VendorSpecific_InfoFrame {
	struct {
		_u8 Type;
		_u8 Ver;
		_u8 Len;
		_u8 CheckSum;
		_u8 IEEE_0;	//PB1
		_u8 IEEE_1;	//PB2
		_u8 IEEE_2;	//PB3
		_u8 Rsvd:5;	//PB4
		_u8 HDMI_Video_Format:3;
		_u8 Reserved_PB5:4;	//PB5
		_u8 _3D_Structure:4;
		_u8 Reserved_PB6:4;	//PB6
		_u8 _3D_Ext_Data:4;
	} info;
	struct {
		_u8 VS_HB[3];
		_u8 VS_DB[28];
	} pktbyte;
} VendorSpecific_InfoFrame;
typedef union _AVI_InfoFrame {
	struct {
		_u8 Type;
		_u8 Ver;
		_u8 Len;
		_u8 checksum;
		_u8 Scan:2;
		_u8 BarInfo:2;
		_u8 ActiveFmtInfoPresent:1;
		_u8 ColorMode:2;
		_u8 FU1:1;
		_u8 ActiveFormatAspectRatio:4;
		_u8 PictureAspectRatio:2;
		_u8 Colorimetry:2;
		_u8 Scaling:2;
		_u8 FU2:6;
		_u8 VIC:7;
		_u8 FU3:1;
		_u8 PixelRepetition:4;
		_u8 FU4:4;
		SHORT Ln_End_Top;
		SHORT Ln_Start_Bottom;
		SHORT Pix_End_Left;
		SHORT Pix_Start_Right;
	} info;
	struct {
		_u8 AVI_HB[3];
		_u8 checksum;
		_u8 AVI_DB[AVI_INFOFRAME_LEN];
	} pktbyte;
} AVI_InfoFrame;
typedef union _Audio_InfoFrame {
	struct {
		_u8 Type;
		_u8 Ver;
		_u8 Len;
		_u8 checksum;
		_u8 AudioChannelCount:3;
		_u8 RSVD1:1;
		_u8 AudioCodingType:4;
		_u8 SampleSize:2;
		_u8 SampleFreq:3;
		_u8 Rsvd2:3;
		_u8 FmtCoding;
		_u8 SpeakerPlacement;
		_u8 Rsvd3:3;
		_u8 LevelShiftValue:4;
		_u8 DM_INH:1;
	} info;
	struct {
		_u8 AUD_HB[3];
		_u8 checksum;
		_u8 AUD_DB[AUDIO_INFOFRAME_LEN];
	} pktbyte;
} Audio_InfoFrame;
typedef union _MPEG_InfoFrame {
	struct {
		_u8 Type;
		_u8 Ver;
		_u8 Len;
		_u8 checksum;
		_u32 MpegBitRate;
		_u8 MpegFrame:2;
		_u8 Rvsd1:2;
		_u8 FieldRepeat:1;
		_u8 Rvsd2:3;
	} info;
	struct {
		_u8 MPG_HB[3];
		_u8 checksum;
		_u8 MPG_DB[MPEG_INFOFRAME_LEN];
	} pktbyte;
} MPEG_InfoFrame;

// Source Product Description
typedef union _SPD_InfoFrame {
	struct {
		_u8 Type;
		_u8 Ver;
		_u8 Len;
		_u8 checksum;
		char VN[8];	// vendor name character in 7bit ascii characters
		char PD[16];	// product description character in 7bit ascii characters
		_u8 SourceDeviceInfomation;
	} info;
	struct {
		_u8 SPD_HB[3];
		_u8 checksum;
		_u8 SPD_DB[SPD_INFOFRAME_LEN];
	} pktbyte;
} SPD_InfoFrame;

///////////////////////////////////////////////////////////////////////////
// Using for interface.
///////////////////////////////////////////////////////////////////////////
struct VideoTiming {
	_u32 VideoPixelClock;
	_u8 VIC;
	_u8 pixelrep;
	_u8 outputVideoMode;
};

/////////////////////////////////////////
// RX Capability.
/////////////////////////////////////////
typedef struct {
	_u8 b16bit:1;
	_u8 b20bit:1;
	_u8 b24bit:1;
	_u8 Rsrv:5;
} LPCM_BitWidth;
typedef enum { AUD_RESERVED_0 =
	    0, AUD_LPCM, AUD_AC3, AUD_MPEG1, AUD_MP3, AUD_MPEG2, AUD_AAC,
	AUD_DTS, AUD_ATRAC, AUD_ONE_BIT_AUDIO, AUD_DOLBY_DIGITAL_PLUS,
	AUD_DTS_HD, AUD_MAT_MLP, AUD_DST, AUD_WMA_PRO, AUD_RESERVED_15
} AUDIO_FORMAT_CODE;
typedef union {
	struct {
		_u8 channel:3;
		_u8 AudioFormatCode:4;
		_u8 Rsrv1:1;
		_u8 b32KHz:1;
		_u8 b44_1KHz:1;
		_u8 b48KHz:1;
		_u8 b88_2KHz:1;
		_u8 b96KHz:1;
		_u8 b176_4KHz:1;
		_u8 b192KHz:1;
		_u8 Rsrv2:1;
		_u8 ucCode;
	} s;
	_u8 uc[3];
} AUDDESCRIPTOR;
typedef union {
	struct {
		_u8 FL_FR:1;
		_u8 LFE:1;
		_u8 FC:1;
		_u8 RL_RR:1;
		_u8 RC:1;
		_u8 FLC_FRC:1;
		_u8 RLC_RRC:1;
		_u8 Reserve:1;
		_u8 Unuse[2];
	} s;
	_u8 uc[3];
} SPK_ALLOC;

#define CEA_SUPPORT_UNDERSCAN (1<<7)
#define CEA_SUPPORT_AUDIO (1<<6)
#define CEA_SUPPORT_YUV444 (1<<5)
#define CEA_SUPPORT_YUV422 (1<<4)
#define CEA_NATIVE_MASK 0xF
typedef union _tag_DCSUPPORT {
	struct {
		_u8 DVI_Dual:1;
		_u8 Rsvd:2;
		_u8 DC_Y444:1;
		_u8 DC_30Bit:1;
		_u8 DC_36Bit:1;
		_u8 DC_48Bit:1;
		_u8 SUPPORT_AI:1;
	} info;
	_u8 uc;
} DCSUPPORT;
typedef union _LATENCY_SUPPORT {
	struct {
		_u8 Rsvd:6;
		_u8 I_Latency_Present:1;
		_u8 Latency_Present:1;
	} info;
	_u8 uc;
} LATENCY_SUPPORT;

#define HDMI_IEEEOUI 0x0c03
typedef struct _RX_CAP {
	_u8 HDMI_VSDB[32];
	_u8 VideoMode;
	_u8 VDOModeCount;
	_u8 idxNativeVDOMode;
	_u8 VDOMode[32];	//32
	_u8 AUDDesCount;
	AUDDESCRIPTOR AUDDes[8];
	_u32 IEEEOUI;
	DCSUPPORT dc;
	_u8 MaxTMDSClock;
	LATENCY_SUPPORT lsupport;
	_u8 V_Latency;
	_u8 A_Latency;
	_u8 V_I_Latency;
	_u8 A_I_Latency;
	SPK_ALLOC SpeakerAllocBlk;
	_u8 ValidCEA:1;
	_u8 ValidHDMI:1;
} RX_CAP;

///////////////////////////////////////////////////////////////////////
// Register offset
///////////////////////////////////////////////////////////////////////

#define REG_TX_VENDOR_ID0   0x00
#define REG_TX_VENDOR_ID1   0x01
#define REG_TX_DEVICE_ID0   0x02
#define REG_TX_DEVICE_ID1   0x03

#define O_DEVID 0
#define M_DEVID 0xF
#define O_REVID 4
#define M_REVID 0xF

#define REG_TX_SW_RST       0x04
#define B_ENTEST    (1<<7)
#define B_REF_RST (1<<5)
#define B_AREF_RST (1<<4)
#define B_VID_RST (1<<3)
#define B_AUD_RST (1<<2)
#define B_HDMI_RST (1<<1)
#define B_HDCP_RST (1<<0)

#define REG_TX_INT_CTRL 0x05
#define B_INTPOL_ACTL 0
#define B_INTPOL_ACTH (1<<7)
#define B_INT_PUSHPULL 0
#define B_INT_OPENDRAIN (1<<6)

#define REG_TX_INT_STAT1    0x06
#define B_INT_AUD_OVERFLOW  (1<<7)
#define B_INT_ROMACQ_NOACK  (1<<6)
#define B_INT_RDDC_NOACK    (1<<5)
#define B_INT_DDCFIFO_ERR   (1<<4)
#define B_INT_ROMACQ_BUS_HANG   (1<<3)
#define B_INT_DDC_BUS_HANG  (1<<2)
#define B_INT_RX_SENSE  (1<<1)
#define B_INT_HPD_PLUG  (1<<0)

#define REG_TX_INT_STAT2    0x07
#define B_INT_HDCP_SYNC_DET_FAIL  (1<<7)
#define B_INT_VID_UNSTABLE  (1<<6)
#define B_INT_PKTACP    (1<<5)
#define B_INT_PKTNULL  (1<<4)
#define B_INT_PKTGENERAL   (1<<3)
#define B_INT_KSVLIST_CHK   (1<<2)
#define B_INT_AUTH_DONE (1<<1)
#define B_INT_AUTH_FAIL (1<<0)

#define REG_TX_INT_STAT3    0x08
#define B_INT_AUD_CTS   (1<<6)
#define B_INT_VSYNC     (1<<5)
#define B_INT_VIDSTABLE (1<<4)
#define B_INT_PKTMPG    (1<<3)
#define B_INT_PKTSPD    (1<<2)
#define B_INT_PKTAUD    (1<<1)
#define B_INT_PKTAVI    (1<<0)

#define REG_TX_INT_MASK1    0x09
#define B_AUDIO_OVFLW_MASK (1<<7)
#define B_DDC_NOACK_MASK (1<<5)
#define B_DDC_FIFO_ERR_MASK (1<<4)
#define B_DDC_BUS_HANG_MASK (1<<2)
#define B_RXSEN_MASK (1<<1)
#define B_HPD_MASK (1<<0)

#define REG_TX_INT_MASK2    0x0A
#define B_PKT_AVI_MASK (1<<7)
#define B_PKT_VID_UNSTABLE_MASK (1<<6)
#define B_PKT_ACP_MASK (1<<5)
#define B_PKT_NULL_MASK (1<<4)
#define B_PKT_GEN_MASK (1<<3)
#define B_KSVLISTCHK_MASK (1<<2)
#define B_T_AUTH_DONE_MASK (1<<1)
#define B_AUTH_FAIL_MASK (1<<0)

#define REG_TX_INT_MASK3    0x0B
#define B_HDCP_SYNC_DET_FAIL_MASK (1<<6)
#define B_AUDCTS_MASK (1<<5)
#define B_VSYNC_MASK (1<<4)
#define B_VIDSTABLE_MASK (1<<3)
#define B_PKT_MPG_MASK (1<<2)
#define B_PKT_SPD_MASK (1<<1)
#define B_PKT_AUD_MASK (1<<0)

#define REG_TX_INT_CLR0      0x0C
#define B_CLR_PKTACP    (1<<7)
#define B_CLR_PKTNULL   (1<<6)
#define B_CLR_PKTGENERAL    (1<<5)
#define B_CLR_KSVLISTCHK    (1<<4)
#define B_CLR_AUTH_DONE  (1<<3)
#define B_CLR_AUTH_FAIL  (1<<2)
#define B_CLR_RXSENSE   (1<<1)
#define B_CLR_HPD       (1<<0)

#define REG_TX_INT_CLR1       0x0D
#define B_CLR_VSYNC (1<<7)
#define B_CLR_VIDSTABLE (1<<6)
#define B_CLR_PKTMPG    (1<<5)
#define B_CLR_PKTSPD    (1<<4)
#define B_CLR_PKTAUD    (1<<3)
#define B_CLR_PKTAVI    (1<<2)
#define B_CLR_HDCP_SYNC_DET_FAIL  (1<<1)
#define B_CLR_VID_UNSTABLE        (1<<0)

#define REG_TX_SYS_STATUS     0x0E
    // readonly
#define B_INT_ACTIVE    (1<<7)
#define B_HPDETECT      (1<<6)
#define B_RXSENDETECT   (1<<5)
#define B_TXVIDSTABLE   (1<<4)
    // read/write
#define O_CTSINTSTEP    2
#define M_CTSINTSTEP    (3<<2)
#define B_CLR_AUD_CTS     (1<<1)
#define B_INTACTDONE    (1<<0)

#define REG_TX_BANK_CTRL        0x0F
#define B_BANK0 0
#define B_BANK1 1

// DDC

#define REG_TX_DDC_MASTER_CTRL   0x10
#define B_MASTERROM (1<<1)
#define B_MASTERDDC (0<<1)
#define B_MASTERHOST    (1<<0)
#define B_MASTERHDCP    (0<<0)

#define REG_TX_DDC_HEADER  0x11
#define REG_TX_DDC_REQOFF  0x12
#define REG_TX_DDC_REQCOUNT    0x13
#define REG_TX_DDC_EDIDSEG 0x14
#define REG_TX_DDC_CMD 0x15
#define CMD_DDC_SEQ_BURSTREAD 0
#define CMD_LINK_CHKREAD  2
#define CMD_EDID_READ   3
#define CMD_FIFO_CLR    9
#define CMD_GEN_SCLCLK  0xA
#define CMD_DDC_ABORT   0xF

#define REG_TX_DDC_STATUS  0x16
#define B_DDC_DONE  (1<<7)
#define B_DDC_ACT   (1<<6)
#define B_DDC_NOACK (1<<5)
#define B_DDC_WAITBUS   (1<<4)
#define B_DDC_ARBILOSE  (1<<3)
#define B_DDC_ERROR     (B_DDC_NOACK|B_DDC_WAITBUS|B_DDC_ARBILOSE)
#define B_DDC_FIFOFULL  (1<<2)
#define B_DDC_FIFOEMPTY (1<<1)

#define REG_TX_DDC_READFIFO    0x17
#define REG_TX_ROM_STARTADDR   0x18
#define REG_TX_HDCP_HEADER 0x19
#define REG_TX_ROM_HEADER  0x1A
#define REG_TX_BUSHOLD_T   0x1B
#define REG_TX_ROM_STAT    0x1C
#define B_ROM_DONE  (1<<7)
#define B_ROM_ACTIVE	(1<<6)
#define B_ROM_NOACK	(1<<5)
#define B_ROM_WAITBUS	(1<<4)
#define B_ROM_ARBILOSE	(1<<3)
#define B_ROM_BUSHANG	(1<<2)

// HDCP
#define REG_TX_AN_GENERATE 0x1F
#define B_START_CIPHER_GEN  1
#define B_STOP_CIPHER_GEN   0

#define REG_TX_HDCP_DESIRE 0x20
#define B_ENABLE_HDPC11 (1<<1)
#define B_CPDESIRE  (1<<0)

#define REG_TX_AUTHFIRE    0x21
#define REG_TX_LISTCTRL    0x22
#define B_LISTFAIL  (1<<1)
#define B_LISTDONE  (1<<0)

#define REG_TX_AKSV    0x23
#define REG_TX_AKSV0   0x23
#define REG_TX_AKSV1   0x24
#define REG_TX_AKSV2   0x25
#define REG_TX_AKSV3   0x26
#define REG_TX_AKSV4   0x27

#define REG_TX_AN  0x28
#define REG_TX_AN_GEN  0x30
#define REG_TX_ARI     0x38
#define REG_TX_ARI0    0x38
#define REG_TX_ARI1    0x39
#define REG_TX_APJ     0x3A

#define REG_TX_BKSV    0x3B
#define REG_TX_BRI     0x40
#define REG_TX_BRI0    0x40
#define REG_TX_BRI1    0x41
#define REG_TX_BPJ     0x42
#define REG_TX_BCAP    0x43
#define B_CAP_HDMI_REPEATER (1<<6)
#define B_CAP_KSV_FIFO_RDY  (1<<5)
#define B_CAP_HDMI_FAST_MODE    (1<<4)
#define B_CAP_HDCP_1p1  (1<<1)
#define B_CAP_FAST_REAUTH   (1<<0)
#define REG_TX_BSTAT   0x44
#define REG_TX_BSTAT0   0x44
#define REG_TX_BSTAT1   0x45
#define B_CAP_HDMI_MODE (1<<12)
#define B_CAP_DVI_MODE (0<<12)
#define B_MAX_CASCADE_EXCEEDED  (1<<11)
#define M_REPEATER_DEPTH    (0x7<<8)
#define O_REPEATER_DEPTH    8
#define B_DOWNSTREAM_OVER   (1<<7)
#define M_DOWNSTREAM_COUNT  0x7F

#define REG_TX_AUTH_STAT 0x46
#define B_T_AUTH_DONE (1<<7)
#define REG_TX_CLK_CTRL0 0x58
#define O_OSCLK_SEL 5
#define M_OSCLK_SEL 3
#define B_AUTO_OVER_SAMPLING_CLOCK (1<<4)
#define O_EXT_MCLK_SEL  2
#define M_EXT_MCLK_SEL  (3<<O_EXT_MCLK_SEL)
#define B_EXT_128FS (0<<O_EXT_MCLK_SEL)
#define B_EXT_256FS (1<<O_EXT_MCLK_SEL)
#define B_EXT_512FS (2<<O_EXT_MCLK_SEL)
#define B_EXT_1024FS (3<<O_EXT_MCLK_SEL)

#define REG_TX_SHA_SEL       0x50
#define REG_TX_SHA_RD_BYTE1  0x51
#define REG_TX_SHA_RD_BYTE2  0x52
#define REG_TX_SHA_RD_BYTE3  0x53
#define REG_TX_SHA_RD_BYTE4  0x54
#define REG_TX_AKSV_RD_BYTE5 0x55

#define REG_TX_CLK_CTRL1 0x59
#define B_EN_TXCLK_COUNT    (1<<5)
#define B_VDO_LATCH_EDGE    (1<<3)
#define M_AUD_DIV           3
#define B_AUD_NODIV         0
#define B_AUD_DIV2          1
#define B_AUD_DIV4          3
#define B_AUD_NODEF         2
#define REG_TX_CLK_STATUS1 0x5E
#define REG_TX_CLK_STATUS2 0x5F
#define B_IP_LOCK (1<<7)
#define B_XP_LOCK (1<<6)
#define B_OSF_LOCK (1<<5)

#define REG_TX_AFE_DRV_CTRL 0x61

#define B_AFE_DRV_PWD    (1<<5)
#define B_AFE_DRV_RST    (1<<4)
#define B_AFE_DRV_PDRXDET    (1<<2)
#define B_AFE_DRV_TERMON    (1<<1)
#define B_AFE_DRV_ENCAL    (1<<0)

#define REG_TX_AFE_XP_CTRL 0x62
#define B_AFE_XP_GAINBIT    (1<<7)
#define B_AFE_XP_PWDPLL    (1<<6)
#define B_AFE_XP_ENI    (1<<5)
#define B_AFE_XP_ER0    (1<<4)
#define B_AFE_XP_RESETB    (1<<3)
#define B_AFE_XP_PWDI    (1<<2)
#define B_AFE_XP_DEI    (1<<1)
#define B_AFE_XP_DER    (1<<0)

#define REG_TX_AFE_ISW_CTRL  0x63
#define B_AFE_RTERM_SEL  (1<<7)
#define B_AFE_IP_BYPASS  (1<<6)
#define M_AFE_DRV_ISW    (7<<3)
#define O_AFE_DRV_ISW    3
#define B_AFE_DRV_ISWK   7

#define REG_TX_AFE_IP_CTRL 0x64

#define B_AFE_IP_GAINBIT    (1<<7)
#define B_AFE_IP_PWDPLL    (1<<6)
#define M_AFE_IP_CKSEL    (3<<4)
#define O_AFE_IP_CKSEL    4
#define B_AFE_IP_ER0    (1<<3)
#define B_AFE_IP_RESETB    (1<<2)
#define B_AFE_IP_ENC    (1<<1)
#define B_AFE_IP_EC1    (1<<0)

#define REG_TX_AFE_RING    0x65
#define B_AFE_CAL_UPDATE    (1<<7)
#define B_AFE_CAL_MANUAL    (1<<6)
#define M_AFE_CAL_CLK_MODE    (3<<4)
#define O_AFE_CAL_CLK_MODE    4
#define O_AFE_DRV_VSW   2
#define M_AFE_DRV_VSW   (3<<2)
#define B_AFE_RING_SLOW    (1<<1)
#define B_AFE_RING_FAST    (1<<0)
#define REG_TX_AFE_TEST    0x66
#define B_AFE_AFE_ENTEST    (1<<6)
#define B_AFE_AFE_ENBIST    (1<<5)
#define M_AFE_CAL_RTERM_MANUAL    0x1F
#define REG_TX_AFE_LFSR     0x67
#define B_AFE_AFELFSR_VAL	(1<<7)
#define B_AFE_DIS_AFELFSR	(1<<6)
#define M_AFE_RTERM_VAOUE    0xF

//
//#define REG_TX_AFE_DRV_CTRL    0x61
//    #define M_AFE_DRV_SR (3<<2)
//    #define O_AFE_DRV_SR 2
//      #define B_AFE_DRV_RST (1<<4)
//      #define B_AFE_DRV_PWD (1<<5)
//      #define B_AFE_DRV_ENBIST (1<<6)
//
//#define REG_TX_AFE_XP_CTRL1 0x62
//      #define B_AFE_XP_GAINBIT (1<<7)
//      #define B_AFE_XP_PWDPLL (1<<6)
//      #define B_AFE_XP_ENI (1<<5)
//      #define B_AFE_XP_ER0 (1<<4)
//      #define B_AFE_XP_RESETB (1<<3)
//      #define B_AFE_XP_PWDI (1<<2)
//      #define B_AFE_XP_DEI (1<<1)
//      #define B_AFE_XP_BYPASS (1<<0)
//
//#define REG_TX_AFE_XP_CTRL2 0x63
//    #define B_XP_ENCLKX5    (1<<3)
//    #define M_XP_CLKSEL 3
//    #define B_XP_CLKSEL_HALF_PCLKHV 0
//    #define B_XP_CLKSEL_1_PCLKHV 1
//    #define B_XP_CLKSEL_2_PCLKHV 2
//    #define B_XP_CLKSEL_4_PCLKHV 3
//
//
//#define REG_TX_AFE_IP_CTRL 0x64
//      #define B_AFE_IP_GAINBIT (1<<6)
//      #define B_AFE_IP_PWDPLL (1<<5)
//      #define B_AFE_IP_SEDB (1<<4)
//      #define B_AFE_IP_ER0 (1<<3)
//      #define B_AFE_IP_RESETB (1<<2)
//      #define B_AFE_IP_PDIV1 (1<<1)
//      #define B_AFE_IP_ENCB (1<<0)
//
//#define REG_TX_AFE_RING    0x65
//      #define B_AFE_RING_FAST (1<<0)
//      #define B_AFE_RING_SLOW (1<<1)
//    #define M_AFE_DRV_VSW (3<<2)
//    #define B_AFE_VSW_NOENH 0
//    #define B_AFE_VSW_12ENH (1<<2)
//    #define B_AFE_VSW_24ENH (2<<2)
//    #define B_AFE_VSW_35ENH (3<<2)

// Input Data Format Register
#define REG_TX_INPUT_MODE  0x70
#define O_INCLKDLY	0
#define M_INCLKDLY	3
#define B_INDDR	    (1<<2)
#define B_SYNCEMB	(1<<3)
#define B_2X656CLK	(1<<4)
#define B_PCLKDIV2  (1<<5)
#define M_INCOLMOD	(3<<6)
#define B_IN_RGB    0
#define B_IN_YUV422 (1<<6)
#define B_IN_YUV444 (2<<6)

#define REG_TX_TXFIFO_RST  0x71
#define B_ENAVMUTERST	1
#define B_TXFFRST	(1<<1)

#define REG_TX_CSC_CTRL    0x72
#define B_CSC_BYPASS    0
#define B_CSC_RGB2YUV   2
#define B_CSC_YUV2RGB   3
#define M_CSC_SEL       3
#define B_TX_EN_DITHER      (1<<7)
#define B_TX_EN_UDFILTER    (1<<6)
#define B_TX_DNFREE_GO      (1<<5)

#define REG_TX_CSC_YOFF 0x73
#define REG_TX_CSC_COFF 0x74
#define REG_TX_CSC_RGBOFF 0x75

#define REG_TX_CSC_MTX11_L 0x76
#define REG_TX_CSC_MTX11_H 0x77
#define REG_TX_CSC_MTX12_L 0x78
#define REG_TX_CSC_MTX12_H 0x79
#define REG_TX_CSC_MTX13_L 0x7A
#define REG_TX_CSC_MTX13_H 0x7B
#define REG_TX_CSC_MTX21_L 0x7C
#define REG_TX_CSC_MTX21_H 0x7D
#define REG_TX_CSC_MTX22_L 0x7E
#define REG_TX_CSC_MTX22_H 0x7F
#define REG_TX_CSC_MTX23_L 0x80
#define REG_TX_CSC_MTX23_H 0x81
#define REG_TX_CSC_MTX31_L 0x82
#define REG_TX_CSC_MTX31_H 0x83
#define REG_TX_CSC_MTX32_L 0x84
#define REG_TX_CSC_MTX32_H 0x85
#define REG_TX_CSC_MTX33_L 0x86
#define REG_TX_CSC_MTX33_H 0x87

#define REG_TX_CSC_GAIN1V_L 0x88
#define REG_TX_CSC_GAIN1V_H 0x89
#define REG_TX_CSC_GAIN2V_L 0x8A
#define REG_TX_CSC_GAIN2V_H 0x8B
#define REG_TX_CSC_GAIN3V_L 0x8C
#define REG_TX_CSC_GAIN3V_H 0x8D

#define REG_TX_HVPol 0x90
#define REG_TX_HfPixel 0x91
#define REG_TX_HSSL 0x95
#define REG_TX_HSEL 0x96
#define REG_TX_HSH 0x97
#define REG_TX_VSS1 0xA0
#define REG_TX_VSE1 0xA1
#define REG_TX_VSS2 0xA2
#define REG_TX_VSE2 0xA3

// HDMI General Control Registers

#define REG_TX_HDMI_MODE   0xC0
#define B_TX_HDMI_MODE 1
#define B_TX_DVI_MODE  0
#define REG_TX_AV_MUTE 0xC1
#define REG_TX_GCP     0xC1
#define B_CLR_AVMUTE    0
#define B_SET_AVMUTE    1
#define B_TX_SETAVMUTE        (1<<0)
#define B_BLUE_SCR_MUTE   (1<<1)
#define B_NODEF_PHASE    (1<<2)
#define B_PHASE_RESYNC   (1<<3)

#define O_COLOR_DEPTH     4
#define M_COLOR_DEPTH     7
#define B_COLOR_DEPTH_MASK (M_COLOR_DEPTH<<O_COLOR_DEPTH)
#define B_CD_NODEF  0
#define B_CD_24     (4<<4)
#define B_CD_30     (5<<4)
#define B_CD_36     (6<<4)
#define B_CD_48     (7<<4)

#define REG_TX_OESS_CYCLE  0xC3
#define REG_TX_ENCRYPTION  0xC4
#define B_DISABLE_ENCRYPTION    1
#define B_ENABLE_ENCRYPTION 0
#define REG_TX_PKT_SINGLE_CTRL 0xC5
#define B_SINGLE_PKT    1
#define B_BURST_PKT
#define B_SW_CTS    (1<<1)
#define REG_TX_PKT_GENERAL_CTRL    0xC6

#define REG_TX_NULL_CTRL 0xC9
#define REG_TX_ACP_CTRL 0xCA
#define REG_TX_ISRC1_CTRL 0xCB
#define REG_TX_ISRC2_CTRL 0xCC
#define REG_TX_AVI_INFOFRM_CTRL 0xCD
#define REG_TX_AUD_INFOFRM_CTRL 0xCE
#define REG_TX_SPD_INFOFRM_CTRL 0xCF
#define REG_TX_MPG_INFOFRM_CTRL 0xD0
#define B_ENABLE_PKT    1
#define B_REPEAT_PKT    (1<<1)

// Audio Channel Control
#define REG_TX_AUDIO_CTRL0 0xE0
#define M_AUD_SWL (3<<6)
#define M_AUD_16BIT (0<<6)
#define M_AUD_18BIT (1<<6)
#define M_AUD_20BIT (2<<6)
#define M_AUD_24BIT (3<<6)

#define B_SPDIFTC (1<<5)

#define B_AUD_SPDIF (1<<4)
#define B_AUD_I2S (0<<4)
#define B_AUD_EN_I2S3   (1<<3)
#define B_AUD_EN_I2S2   (1<<2)
#define B_AUD_EN_I2S1   (1<<1)
#define B_AUD_EN_I2S0   (1<<0)
#define B_AUD_EN_SPDIF  1

#define REG_TX_AUDIO_CTRL1 0xE1
#define B_AUD_FULLPKT (1<<6)

#define B_AUDFMT_STD_I2S (0<<0)
#define B_AUDFMT_32BIT_I2S (1<<0)
#define B_AUDFMT_LEFT_JUSTIFY (0<<1)
#define B_AUDFMT_RIGHT_JUSTIFY (1<<1)
#define B_AUDFMT_DELAY_1T_TO_WS (0<<2)
#define B_AUDFMT_NO_DELAY_TO_WS (1<<2)
#define B_AUDFMT_WS0_LEFT   (0<<3)
#define B_AUDFMT_WS0_RIGHT   (1<<3)
#define B_AUDFMT_MSB_SHIFT_FIRST (0<<4)
#define B_AUDFMT_LSB_SHIFT_FIRST (1<<4)
#define B_AUDFMT_RISE_EDGE_SAMPLE_WS (0<<5)
#define B_AUDFMT_FALL_EDGE_SAMPLE_WS (0<<5)

#define REG_TX_AUDIO_FIFOMAP 0xE2
#define O_FIFO3SEL 6
#define O_FIFO2SEL 4
#define O_FIFO1SEL 2
#define O_FIFO0SEL 0
#define B_SELSRC3  3
#define B_SELSRC2  2
#define B_SELSRC1  1
#define B_SELSRC0  0

#define REG_TX_AUDIO_CTRL3 0xE3
#define B_AUD_MULCH (1<<7)
#define B_EN_ZERO_CTS (1<<6)
#define B_CHSTSEL (1<<4)
#define B_S3RLCHG (1<<3)
#define B_S2RLCHG (1<<2)
#define B_S1RLCHG (1<<1)
#define B_S0RLCHG (1<<0)

#define REG_TX_AUD_SRCVALID_FLAT 0xE4
#define B_AUD_SPXFLAT_SRC3 (1<<7)
#define B_AUD_SPXFLAT_SRC2 (1<<6)
#define B_AUD_SPXFLAT_SRC1 (1<<5)
#define B_AUD_SPXFLAT_SRC0 (1<<4)
#define B_AUD_ERR2FLAT (1<<3)
#define B_AUD_S3VALID (1<<2)
#define B_AUD_S2VALID (1<<1)
#define B_AUD_S1VALID (1<<0)

#define REG_TX_AUD_HDAUDIO 0xE5
#define B_HBR   (1<<3)
#define B_DSD   (1<<1)

//////////////////////////////////////////
// Bank 1
//////////////////////////////////////////

#define REGPktAudCTS0 0x30	// 7:0
#define REGPktAudCTS1 0x31	// 15:8
#define REGPktAudCTS2 0x32	// 19:16
#define REGPktAudN0 0x33	// 7:0
#define REGPktAudN1 0x34	// 15:8
#define REGPktAudN2 0x35	// 19:16
#define REGPktAudCTSCnt0 0x35	// 3:0
#define REGPktAudCTSCnt1 0x36	// 11:4
#define REGPktAudCTSCnt2 0x37	// 19:12

//////////////////////////////////////////
// COMMON PACKET for NULL,ISRC1,ISRC2,SPD
//////////////////////////////////////////

#define	REG_TX_PKT_HB00 0x38
#define	REG_TX_PKT_HB01 0x39
#define	REG_TX_PKT_HB02 0x3A

#define	REG_TX_PKT_PB00 0x3B
#define	REG_TX_PKT_PB01 0x3C
#define	REG_TX_PKT_PB02 0x3D
#define	REG_TX_PKT_PB03 0x3E
#define	REG_TX_PKT_PB04 0x3F
#define	REG_TX_PKT_PB05 0x40
#define	REG_TX_PKT_PB06 0x41
#define	REG_TX_PKT_PB07 0x42
#define	REG_TX_PKT_PB08 0x43
#define	REG_TX_PKT_PB09 0x44
#define	REG_TX_PKT_PB10 0x45
#define	REG_TX_PKT_PB11 0x46
#define	REG_TX_PKT_PB12 0x47
#define	REG_TX_PKT_PB13 0x48
#define	REG_TX_PKT_PB14 0x49
#define	REG_TX_PKT_PB15 0x4A
#define	REG_TX_PKT_PB16 0x4B
#define	REG_TX_PKT_PB17 0x4C
#define	REG_TX_PKT_PB18 0x4D
#define	REG_TX_PKT_PB19 0x4E
#define	REG_TX_PKT_PB20 0x4F
#define	REG_TX_PKT_PB21 0x50
#define	REG_TX_PKT_PB22 0x51
#define	REG_TX_PKT_PB23 0x52
#define	REG_TX_PKT_PB24 0x53
#define	REG_TX_PKT_PB25 0x54
#define	REG_TX_PKT_PB26 0x55
#define	REG_TX_PKT_PB27 0x56

#define REG_TX_AVIINFO_DB1 0x58
#define REG_TX_AVIINFO_DB2 0x59
#define REG_TX_AVIINFO_DB3 0x5A
#define REG_TX_AVIINFO_DB4 0x5B
#define REG_TX_AVIINFO_DB5 0x5C
#define REG_TX_AVIINFO_DB6 0x5E
#define REG_TX_AVIINFO_DB7 0x5F
#define REG_TX_AVIINFO_DB8 0x60
#define REG_TX_AVIINFO_DB9 0x61
#define REG_TX_AVIINFO_DB10 0x62
#define REG_TX_AVIINFO_DB11 0x63
#define REG_TX_AVIINFO_DB12 0x64
#define REG_TX_AVIINFO_DB13 0x65
#define REG_TX_AVIINFO_SUM 0x5D

#define REG_TX_PKT_AUDINFO_CC 0x68	// [2:0]
#define REG_TX_PKT_AUDINFO_SF 0x69	// [4:2]
#define REG_TX_PKT_AUDINFO_CA 0x6B	// [7:0]

#define REG_TX_PKT_AUDINFO_DM_LSV 0x6C	// [7][6:3]
#define REG_TX_PKT_AUDINFO_SUM 0x6D	// [7:0]

// Source Product Description Info Frame
#define REG_TX_PKT_SPDINFO_SUM 0x70
#define REG_TX_PKT_SPDINFO_PB1 0x71
#define REG_TX_PKT_SPDINFO_PB2 0x72
#define REG_TX_PKT_SPDINFO_PB3 0x73
#define REG_TX_PKT_SPDINFO_PB4 0x74
#define REG_TX_PKT_SPDINFO_PB5 0x75
#define REG_TX_PKT_SPDINFO_PB6 0x76
#define REG_TX_PKT_SPDINFO_PB7 0x77
#define REG_TX_PKT_SPDINFO_PB8 0x78
#define REG_TX_PKT_SPDINFO_PB9 0x79
#define REG_TX_PKT_SPDINFO_PB10 0x7A
#define REG_TX_PKT_SPDINFO_PB11 0x7B
#define REG_TX_PKT_SPDINFO_PB12 0x7C
#define REG_TX_PKT_SPDINFO_PB13 0x7D
#define REG_TX_PKT_SPDINFO_PB14 0x7E
#define REG_TX_PKT_SPDINFO_PB15 0x7F
#define REG_TX_PKT_SPDINFO_PB16 0x80
#define REG_TX_PKT_SPDINFO_PB17 0x81
#define REG_TX_PKT_SPDINFO_PB18 0x82
#define REG_TX_PKT_SPDINFO_PB19 0x83
#define REG_TX_PKT_SPDINFO_PB20 0x84
#define REG_TX_PKT_SPDINFO_PB21 0x85
#define REG_TX_PKT_SPDINFO_PB22 0x86
#define REG_TX_PKT_SPDINFO_PB23 0x87
#define REG_TX_PKT_SPDINFO_PB24 0x88
#define REG_TX_PKT_SPDINFO_PB25 0x89

#define REG_TX_PKT_MPGINFO_FMT 0x8A
#define B_MPG_FR 1
#define B_MPG_MF_I  (1<<1)
#define B_MPG_MF_B  (2<<1)
#define B_MPG_MF_P  (3<<1)
#define B_MPG_MF_MASK (3<<1)
#define REG_TX_PKG_MPGINFO_DB0 0x8B
#define REG_TX_PKG_MPGINFO_DB1 0x8C
#define REG_TX_PKG_MPGINFO_DB2 0x8D
#define REG_TX_PKG_MPGINFO_DB3 0x8E
#define REG_TX_PKG_MPGINFO_SUM 0x8F

#define REG_TX_AUDCHST_MODE    0x91	// 191 REG_TX_AUD_CHSTD[2:0] 6:4
    //     REG_TX_AUD_CHSTC 3
    //     REG_TX_AUD_NLPCM 2
    //     REG_TX_AUD_MONO 0
#define REG_TX_AUDCHST_CAT     0x92	// 192 REG_TX_AUD_CHSTCAT 7:0
#define REG_TX_AUDCHST_SRCNUM  0x93	// 193 REG_TX_AUD_CHSTSRC 3:0
#define REG_TX_AUD0CHST_CHTNUM 0x94	// 194 REG_TX_AUD0_CHSTCHR 7:4
    //     REG_TX_AUD0_CHSTCHL 3:0
#define REG_TX_AUD1CHST_CHTNUM 0x95	// 195 REG_TX_AUD1_CHSTCHR 7:4
    //     REG_TX_AUD1_CHSTCHL 3:0
#define REG_TX_AUD2CHST_CHTNUM 0x96	// 196 REG_TX_AUD2_CHSTCHR 7:4
    //     REG_TX_AUD2_CHSTCHL 3:0
#define REG_TX_AUD3CHST_CHTNUM 0x97	// 197 REG_TX_AUD3_CHSTCHR 7:4
    //     REG_TX_AUD3_CHSTCHL 3:0
#define REG_TX_AUDCHST_CA_FS   0x98	// 198 REG_TX_AUD_CHSTCA 5:4
    //     REG_TX_AUD_CHSTFS 3:0
#define REG_TX_AUDCHST_OFS_WL  0x99	// 199 REG_TX_AUD_CHSTOFS 7:4
    //     REG_TX_AUD_CHSTWL 3:0

/////////////////////////////////////////////////////////////////////
// Macro
/////////////////////////////////////////////////////////////////////

#define Switch_HDMITX_Bank(x)   HDMITX_WriteI2C_Byte(0x0f,(x)&1)

#define HDMI_OrREG_TX_Byte(reg,ormask) HDMITX_WriteI2C_Byte(reg,(HDMITX_ReadI2C_Byte(reg) | (ormask)))
#define HDMI_AndREG_TX_Byte(reg,andmask) HDMITX_WriteI2C_Byte(reg,(HDMITX_ReadI2C_Byte(reg) & (andmask)))
#define HDMI_SetREG_TX_Byte(reg,andmask,ormask) HDMITX_WriteI2C_Byte(reg,((HDMITX_ReadI2C_Byte(reg) & (andmask))|(ormask)))

#define LVDS_OrREG_TX_Byte(reg,ormask) LVDS_WriteI2C_Byte(reg,(LVDS_ReadI2C_Byte(reg) | (ormask)))
#define LVDS_AndREG_TX_Byte(reg,andmask) LVDS_WriteI2C_Byte(reg,(LVDS_ReadI2C_Byte(reg) & (andmask)))
#define LVDS_SetREG_TX_Byte(reg,andmask,ormask) LVDS_WriteI2C_Byte(reg,((LVDS_ReadI2C_Byte(reg) & (andmask))|(ormask)))

/////////////////////////////////////////////////////////////////////
// data structure
/////////////////////////////////////////////////////////////////////
typedef struct _INSTANCE_STRUCT {
	_u8 I2C_DEV;
	_u8 I2C_ADDR;

	/////////////////////////////////////////////////
	// Interrupt Type
	/////////////////////////////////////////////////
	_u8 bIntType;		// = 0 ;
	/////////////////////////////////////////////////
	// Video Property
	/////////////////////////////////////////////////
	_u8 bInputVideoSignalType;	// for Sync Embedded,CCIR656,InputDDR
	/////////////////////////////////////////////////
	// Audio Property
	/////////////////////////////////////////////////
	_u8 bOutputAudioMode;	// = 0 ;
	_u8 bAudioChannelSwap;	// = 0 ;
	_u8 bAudioChannelEnable;
	_u8 bAudFs;
	unsigned long TMDSClock;
	_u8 bAuthenticated:1;
	_u8 bHDMIMode:1;
	_u8 bIntPOL:1;		// 0 = Low Active
	_u8 bHPD:1;

	// 2009/11/11 added by jj_tseng@ite.com.tw
	_u8 TxEMEMStatus:1;
	_u8 bSPDIF_OUT;
	_u8 bVideoOut;		//~Mingchih.Lung@ite.com.tw 2009/11/11
} INSTANCE;

// 2008/02/27 added by jj_tseng@chipadvanced.com
typedef enum _mode_id { UNKNOWN_MODE =
	    0, CEA_640x480p60, CEA_720x480p60, CEA_1280x720p60,
	CEA_1920x1080i60, CEA_720x480i60, CEA_720x240p60,
	CEA_1440x480i60, CEA_1440x240p60, CEA_2880x480i60,
	CEA_2880x240p60, CEA_1440x480p60, CEA_1920x1080p60,
	CEA_720x576p50, CEA_1280x720p50, CEA_1920x1080i50,
	CEA_720x576i50, CEA_1440x576i50, CEA_720x288p50,
	CEA_1440x288p50, CEA_2880x576i50, CEA_2880x288p50,
	CEA_1440x576p50, CEA_1920x1080p50, CEA_1920x1080p24,
	CEA_1920x1080p25, CEA_1920x1080p30, VESA_640x350p85,
	VESA_640x400p85, VESA_720x400p85, VESA_640x480p60,
	VESA_640x480p72, VESA_640x480p75, VESA_640x480p85,
	VESA_800x600p56, VESA_800x600p60, VESA_800x600p72,
	VESA_800x600p75, VESA_800X600p85, VESA_840X480p60,
	VESA_1024x768p60, VESA_1024x768p70, VESA_1024x768p75,
	VESA_1024x768p85, VESA_1152x864p75, VESA_1280x768p60R,
	VESA_1280x768p60, VESA_1280x768p75, VESA_1280x768p85,
	VESA_1280x960p60, VESA_1280x960p85, VESA_1280x1024p60,
	VESA_1280x1024p75, VESA_1280X1024p85, VESA_1360X768p60,
	VESA_1400x768p60R, VESA_1400x768p60, VESA_1400x1050p75,
	VESA_1400x1050p85, VESA_1440x900p60R, VESA_1440x900p60,
	VESA_1440x900p75, VESA_1440x900p85, VESA_1600x1200p60,
	VESA_1600x1200p65, VESA_1600x1200p70, VESA_1600x1200p75,
	VESA_1600x1200p85, VESA_1680x1050p60R, VESA_1680x1050p60,
	VESA_1680x1050p75, VESA_1680x1050p85, VESA_1792x1344p60,
	VESA_1792x1344p75, VESA_1856x1392p60, VESA_1856x1392p75,
	VESA_1920x1200p60R, VESA_1920x1200p60, VESA_1920x1200p75,
	VESA_1920x1200p85, VESA_1920x1440p60, VESA_1920x1440p75,
} MODE_ID;

//~jj_tseng@chipadvanced.com

//////////////////////////////////////////////////////////////////////
// External Interface
//////////////////////////////////////////////////////////////////////
void InitIT626X(void);
void HDMITX_InitInstance(INSTANCE * pInstance);
BOOL EnableVideoOutput( /*VIDEOPCLKLEVEL*/ _u8 level, _u8 inputColorMode,
		       _u8 outputColorMode, _u8 bHDMI);
BOOL SetupVideoInputSignal(_u8 inputSignalType);
BOOL EnableAudioOutput(unsigned long cVideoPixelClock, _u8 bAudioSampleFreq,
		       _u8 ChannelNumber, _u8 bAudSWL, _u8 bSPDIF);
void DisableIT626X(void);
void DisableVideoOutput(void);
void DisableAudioOutput(void);
BOOL GetEDIDData(int EDIDBlockID, _u8 * pEDIDData);
_u8 CheckHDMITX(_u8 * pHPD, _u8 * pHPDChange);

#ifdef SUPPORT_HDCP
BOOL EnableHDCP(_u8 bEnable);

#endif /*  */
BOOL EnableVSInfoFrame(_u8 bEnable, _u8 * pVSInfoFrame);
BOOL EnableAVIInfoFrame(_u8 bEnable, _u8 * pAVIInfoFrame);
BOOL EnableAudioInfoFrame(_u8 bEnable, _u8 * pAudioInfoFrame);

// BOOL EnableVideoOutputIndirect(_u8 xCnt,_u8 inputColorMode,_u8 outputColorMode,_u8 bHDMI);
void SetAVMute(_u8 bEnable);
void SetOutputColorDepthPhase(_u8 ColorDepth, _u8 bPhase);
void Get6613Reg(_u8 * pReg);
_u8 LVDS_ReadI2C_Byte(_u8 RegAddr);
int LVDS_WriteI2C_Byte(_u8 RegAddr, _u8 d);
_u8 HDMITX_DetectVIC(void);

// 2008/08/18 added by jj_tseng@chipadvanced.com
/////////////////////////////////////////////////////////////////////////////////////
// IT626X part
/////////////////////////////////////////////////////////////////////////////////////
void setIT626X_ChStat(_u8 ucIEC60958ChStat[]);
void setIT626X_UpdateChStatFs(_u32 Fs);
void setIT626X_LPCMAudio(_u8 AudioSrcNum, _u8 AudSWL, BOOL bSPDIF);
void setIT626X_NLPCMAudio(void);
void setIT626X_HBRAudio(BOOL bSPDIF);
void setIT626X_DSDAudio(void);
void EnableHDMIAudio(_u8 AudioType, BOOL bSPDIF, _u32 SampleFreq, _u8 ChNum,
		     _u8 * pIEC60958ChStat, _u32 TMDSClock);

////////////////////////////////////////////////////////////////////
// Required Interfance
////////////////////////////////////////////////////////////////////
#define HDMITX_ReadI2C_Byte ReadI2C_Byte
#define HDMITX_WriteI2C_Byte WriteI2C_Byte
//#define HDMITX_ReadI2C_ByteN ReadI2C_ByteN
//#define HDMITX_WriteI2C_ByteN WriteI2C_ByteN
#define I2C_OrReg_Byte(reg,ormask) WriteI2C_Byte(reg,(ReadI2C_Byte(reg) | (ormask)))
#define I2C_AndReg_Byte(reg,andmask) WriteI2C_Byte(reg,(ReadI2C_Byte(reg) & (andmask)))
#define I2C_SetReg_Byte(reg,andmask,ormask) WriteI2C_Byte(reg,((ReadI2C_Byte(reg) & (andmask))|(ormask)))
_u16 GetInputClockCount(void);
_u32 GetInputPclk(void);
UINT GetHTotal(void);
UINT GetHActive(void);
UINT GetHBlank(void);
UINT GetVTotal(void);
UINT GetVActive(void);
UINT GetVBlank(void);
BOOL SetLVDSinterface(void);
BOOL CheckLVDS(void);
void ResetLVDS(void);
void SetLVDS_AFE(void);
void EnableHVToolDetect(BOOL HVenable);
void SoftWareVideoReset(void);

#ifdef SUPPORT_REGEN_TIMING
BOOL ReGenTiming(_u8 fmt_assign);
void EnableDeOnly(BOOL DeOnly);

#endif /*  */
void UpDateAFE(_u32 PixelClock);

#ifdef DEBUG
void DumpCatHDMITXReg(void);
void DumpLVDSReg(void);

#endif // DEBUG
#ifdef SUPPORT_SPLASH
void FastOutput(void);

#endif /*  */
void SetupAudioChannel(BOOL EnableAudio_b);
BOOL GetVideoStatus(void);
BOOL GetAVMute(void);
void CheckAudioVideoInput(void);
void HDMITX_ChangeDisplayOption(HDMI_Video_Type VideoMode,
				HDMI_OutputColorMode OutputColorMode);
void InitIT626X_Instance(void);
void HDMITX_SetOutput(void);
void HDMITX_DevLoopProc(void);
CHAR VideoModeDetect(void);
void ConfigfHdmiVendorSpecificInfoFrame(_u8 _3D_Stru);
static int init_it6263_loop_kthread(void);

    /*
       _u8 I2C_Read_Byte(_u8 Addr,_u8 RegAddr);
       SYS_STATUS I2C_Write_Byte(_u8 Addr,_u8 RegAddr,_u8 Data);
       SYS_STATUS I2C_Read_ByteN(_u8 Addr,_u8 RegAddr,_u8 *pData,int N);
       SYS_STATUS I2C_Write_ByteN(_u8 Addr,_u8 RegAddr,_u8 *pData,int N);

       _u8 HDMITX_ReadI2C_Byte(_u8 RegAddr);
       SYS_STATUS HDMITX_WriteI2C_Byte(_u8 RegAddr,_u8 val);
       SYS_STATUS HDMITX_ReadI2C_ByteN(_u8 RegAddr,_u8 *pData,int N);
       SYS_STATUS HDMITX_WriteI2C_ByteN(_u8 RegAddr,_u8 *pData,int N);
     */
#define Switch_HDMITX_Bank(x)   HDMITX_WriteI2C_Byte(0x0f,(x)&1)

#define HDMITX_OrREG_Byte(reg,ormask) HDMITX_WriteI2C_Byte(reg,(HDMITX_ReadI2C_Byte(reg) | (ormask)))
#define HDMITX_AndREG_Byte(reg,andmask) HDMITX_WriteI2C_Byte(reg,(HDMITX_ReadI2C_Byte(reg) & (andmask)))
#define HDMITX_SetREG_Byte(reg,andmask,ormask) HDMITX_WriteI2C_Byte(reg,((HDMITX_ReadI2C_Byte(reg) & (andmask))|(ormask)))

#endif // _IT626X_H_
