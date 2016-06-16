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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/printk.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <asm/atomic.h>
#include <linux/it6263.h>
#include "it6263_drv.h"

MODULE_LICENSE("GPL");

static struct device *it6263_dev;
struct it6263_data *g_it6263_data;
struct it6263_platform_data *it6263_platform;

struct it6263_data *get_it6263_data(void)
{
	return g_it6263_data;
}

void delay1ms(unsigned short ms)
{
	msleep(ms);
}

static int i2c_write_reg(struct i2c_client *client, unsigned int offset,
			 u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, offset, value);
	if (ret < 0) {
		printk("i2c_write_reg error %d, offset=0x%02x, val=0x%02x\n",
		       ret, offset, (int)value);
	}

	return ret;
}

static int i2c_read_reg(struct i2c_client *client, unsigned int offset,
			u8 * value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(client, offset);
	if (ret < 0) {
		printk("i2c_read_reg write error %d, offset=0x%02x\n", ret,
		       offset);
		return ret;
	}

	ret = i2c_smbus_read_byte(client);
	if (ret < 0) {
		printk("i2c_read_reg read error %d, offset=0x%02x\n", ret,
		       offset);
		return ret;
	}

	*value = ret & 0x000000FF;

	return 0;
}

_u8 ReadI2C_Byte(_u8 RegAddr)
{
	_u8 p_data = 0;
	int ret = 0;
	i2c_read_reg(it6263_platform->hdmi_client, (unsigned int)RegAddr,
		     &p_data);
	if (ret < 0)
		HDMITX_DEBUG_PRINTF(("HDMI Read 0x%x fail\n",
				     (unsigned int)RegAddr));
	return p_data;
}

int WriteI2C_Byte(_u8 RegAddr, _u8 d)
{
	int ret = 0;
	ret =
	    i2c_write_reg(it6263_platform->hdmi_client, (unsigned int)RegAddr,
			  d);
	if (ret < 0)
		HDMITX_DEBUG_PRINTF(("HDMI write 0x%x fail\n",
				     (unsigned int)RegAddr));
	return ret;
}

#define INPUT_SIGNAL_TYPE 0	// 24 bit sync seperate
#define I2S 0
#define SPDIF 1
INSTANCE InstanceData = {

	0,			// _u8 I2C_DEV ;
	0x98,			// _u8 I2C_ADDR ;

	/////////////////////////////////////////////////
	// Interrupt Type
	/////////////////////////////////////////////////
	0x40,			// _u8 bIntType ; // = 0 ;
	/////////////////////////////////////////////////
	// Video Property
	/////////////////////////////////////////////////
	INPUT_SIGNAL_TYPE,	// _u8 bInputVideoSignalType ; // for Sync Embedded,CCIR656,InputDDR

	/////////////////////////////////////////////////
	// Audio Property
	/////////////////////////////////////////////////
	I2S,			// _u8 bOutputAudioMode ; // = 0 ;
	FALSE,			// _u8 bAudioChannelSwap ; // = 0 ;
	0x01,			// _u8 bAudioChannelEnable ;
	AUDFS_48KHz,		// _u8 bAudFs ;
	0,			// unsigned long TMDSClock ;
	FALSE,			// _u8 bAuthenticated:1 ;
	FALSE,			// _u8 bHDMIMode: 1;
	FALSE,			// _u8 bIntPOL:1 ; // 0 = Low Active
	FALSE,			// _u8 bHPD:1 ;

};

////////////////////////////////////////////////////////////////////////////////
// EDID
////////////////////////////////////////////////////////////////////////////////
static RX_CAP RxCapability;
BOOL bChangeMode = FALSE;
BOOL bForceCTS = FALSE;
AVI_InfoFrame AviInfo;
Audio_InfoFrame AudioInfo;
_u16 LastInputPclk;
VendorSpecific_InfoFrame VS_Info;
////////////////////////////////////////////////////////////////////////////////
// Program utility.
////////////////////////////////////////////////////////////////////////////////
// 2012/08/16 modified by jjtseng
#ifdef SUPPORT_HDCP
static _u8 bAuthenticated = FALSE;
#endif
_u8 cBuf[128];
INSTANCE Instance[HDMITX_INSTANCE_MAX];
_u8 bInputColorMode = F_MODE_RGB444;
//_u8 bInputColorMode = F_MODE_YUV422 ;
//_u8 bInputColorMode = F_MODE_YUV444 ;
_u8 OutputColorDepth = 24;
//_u8 bOutputColorMode = F_MODE_YUV422 ;
//_u8 bOutputColorMode = F_MODE_YUV444 ;
_u8 bOutputColorMode = F_MODE_RGB444;
_u8 iVideoModeSelect = 0;
_u32 VideoPixelClock;
_u8 VIC;			// 480p60
_u8 HDMI3DFormat = 0xFF;	// 3D Format
_u8 pixelrep;			// no pixelrepeating
HDMI_Aspec aspec;
HDMI_Colorimetry Colorimetry;
_u8 bAudioSampleFreq = INPUT_SAMPLE_FREQ;
BOOL bHDMIMode, bAudioEnable;
BOOL ReGenTimingEnable = FALSE;
_u8 HPDStatus = FALSE;
_u8 HPDChangeStatus = FALSE;
_u8 LastRefaudfreqnum = 0;
BOOL BootFlag = TRUE;
extern BOOL ReGenTimingEnable;
extern BOOL bChangeMode;
extern BOOL bForceCTS;
_u8 AudioDelayCnt = 0;

typedef enum VindorName {
	DELL_2408WPF = 0,
	UNKNOW_Monitor
} VindorName;

struct EDID_Vindor {
	VindorName Name;
	_u8 VIDC[8];
};
struct EDID_Vindor ForceCTS_EDID_Vindeo[] = {
	{DELL_2408WPF, {0x10, 0xAC, 0x2C, 0xA0, 0x53, 0x33, 0x47, 0x31}},
};

struct CRT_TimingSetting {
	_u8 fmt;
	_u8 interlaced;
	UINT VEC[11];
};
//   VDEE_L,   VDEE_H, VRS2S_L, VRS2S_H, VRS2E_L, VRS2E_H, HalfL_L, HalfL_H, VDE2S_L, VDE2S_H, HVP&Progress
struct CRT_TimingSetting TimingTable[] = {
// {    FMT             ,  Int,      HT   ,      H_DEW,    H_FBH,  H_SyncW,   H_BBH,   VTotal,   V_DEW,  V_FBH,  V_SyncW,  V_BBH,   Sync_pol,   }},
	{HDMI_480p60, 0, {858, 720, 16, 62, 60, 525, 480, 9, 6, 30, 0}},
	{HDMI_480p60_16x9, 0, {858, 720, 16, 62, 60, 525, 480, 9, 6, 30, 0}},
	{HDMI_720p60, 0, {1650, 1280, 110, 40, 220, 750, 720, 5, 5, 20, 3}},
	{HDMI_1080i60, 1, {2200, 1920, 88, 44, 148, 1125, 540, 2, 5, 15, 3}},
	{HDMI_480i60_16x9, 1, {858, 720, 19, 62, 57, 525, 240, 4, 3, 15, 0}},
	{HDMI_1080p60, 0, {2200, 1920, 88, 44, 148, 1125, 1080, 4, 5, 36, 3}},

	{HDMI_576p50, 0, {864, 720, 12, 64, 68, 525, 480, 5, 5, 36, 0}},
	{HDMI_720p50, 0, {1980, 1280, 440, 40, 220, 750, 720, 5, 5, 20, 3}},
	{HDMI_1080i50, 1, {2640, 1920, 528, 44, 148, 1125, 540, 2, 5, 15, 3}},
	{HDMI_1080p50, 0, {2640, 1920, 528, 44, 148, 1125, 1080, 4, 5, 36, 3}},
	{HDMI_1080p24, 0, {2750, 1920, 638, 44, 148, 1125, 1080, 4, 5, 36, 3}},
	{HDMI_720p60_FP3D, 0,
	 {1650, 1280, 110, 40, 220, 2 * 750, 750 + 720, 5, 5, 20, 3}},
	{HDMI_720p50_FP3D, 0,
	 {1980, 1280, 440, 40, 220, 2 * 750, 750 + 720, 5, 5, 20, 3}},
	{HDMI_1080p24_FP3D, 0,
	 {2750, 1920, 638, 44, 148, 2 * 1125, 1125 + 1080, 4, 5, 36, 3}},

};

#define MaxIndex (sizeof(TimingTable)/sizeof(struct CRT_TimingSetting))

////////////////////////////////////////////////////////////////////////////////
// Function Body.
////////////////////////////////////////////////////////////////////////////////
void HDMITX_ChangeDisplayOption(HDMI_Video_Type VideoMode,
				HDMI_OutputColorMode OutputColorMode);
void HDMITX_SetOutput(void);
void HDMITX_DevLoopProc(void);
_u8 ParseEDID(void);
static BOOL ParseCEAEDID(_u8 * pCEAEDID);
void ConfigAVIInfoFrame(_u8 VIC, _u8 pixelrep);
void ConfigAudioInfoFrm(void);
//static void SetInputMode(_u8 InputMode,_u8 bInputSignalType);
//static void SetCSCScale(_u8 bInputMode,_u8 bOutputMode);
// static void SetupAFE(_u8 ucFreqInMHz);
static void SetupAFE(VIDEOPCLKLEVEL PCLKLevel);
static void FireAFE(void);
//atic SYS_STATUS SetAudioFormat(_u8 NumChannel,_u8 AudioEnable,_u8 bSampleFreq,_u8 AudSWL,_u8 AudioCatCode);
static SYS_STATUS SetNCTS( /*_u32 PCLK,*/ _u8 Fs);
static void AutoAdjustAudio(void);
static SYS_STATUS SetAVIInfoFrame(AVI_InfoFrame * pAVIInfoFrame);
static SYS_STATUS SetAudioInfoFrame(Audio_InfoFrame * pAudioInfoFrame);
//static SYS_STATUS SetSPDInfoFrame(SPD_InfoFrame *pSPDInfoFrame);
//static SYS_STATUS SetMPEGInfoFrame(MPEG_InfoFrame *pMPGInfoFrame);
static SYS_STATUS ReadEDID(_u8 * pData, _u8 bSegment, _u8 offset, SHORT Count);
static void AbortDDC(void);
static void ClearDDCFIFO(void);
static void GenerateDDCSCLK(void);
static SYS_STATUS HDCP_EnableEncryption(void);
static void HDCP_ResetAuth(void);
static void HDCP_Auth_Fire(void);
static void HDCP_StartAnCipher(void);
static void HDCP_StopAnCipher(void);
static void HDCP_GenerateAn(void);
static SYS_STATUS HDCP_GetVr(_u8 * pVr);
static SYS_STATUS HDCP_GetBCaps(PUCHAR pBCaps, PUSHORT pBStatus);
static SYS_STATUS HDCP_GetBKSV(_u8 * pBKSV);
static SYS_STATUS HDCP_Authenticate(void);
static SYS_STATUS HDCP_Authenticate_Repeater(void);
static SYS_STATUS HDCP_VerifyIntegration(void);
static SYS_STATUS HDCP_GetKSVList(_u8 * pKSVList, _u8 cDownStream);
static SYS_STATUS HDCP_CheckSHA(_u8 M0[], _u16 BStatus, _u8 KSVList[],
				int devno, _u8 Vr[]);
static void HDCP_ResumeAuthentication(void);
static void HDCP_Reset(void);
static void HDCP_CancelRepeaterAuthenticate(void);
static void ENABLE_NULL_PKT(void);
static void ENABLE_ACP_PKT(void);
static void ENABLE_ISRC1_PKT(void);
static void ENABLE_ISRC2_PKT(void);
static void ENABLE_AVI_INFOFRM_PKT(void);
static void ENABLE_AUD_INFOFRM_PKT(void);
static void ENABLE_SPD_INFOFRM_PKT(void);
static void ENABLE_MPG_INFOFRM_PKT(void);

static void DISABLE_NULL_PKT(void);
static void DISABLE_ACP_PKT(void);
static void DISABLE_ISRC1_PKT(void);
static void DISABLE_ISRC2_PKT(void);
static void DISABLE_AVI_INFOFRM_PKT(void);
static void DISABLE_AUD_INFOFRM_PKT(void);
static void DISABLE_SPD_INFOFRM_PKT(void);
static void DISABLE_MPG_INFOFRM_PKT(void);
static _u8 countbit(_u8 b);
#ifdef Powerdown
void Power_Down(void);
void Power_Resume(void);
#endif

//_IDATA _u32 CurrCTS;
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////

BOOL AudioModeDetect(void)
{
	SetupAudioChannel(bAudioEnable);
	return TRUE;
}

CHAR VideoModeDetect(void)
{
	static BOOL ForceUpdate = TRUE;
	if (CheckLVDS() == FALSE) {
		ForceUpdate = TRUE;
		return FALSE;
	}
	// VIC = HDMI_1080p60;
	if (ForceUpdate) {
		HDMITX_DEBUG_PRINTF(("VideoModeDetect()\n"));
		VIC = HDMI_Unknown;	// turn off Regentiming at first.
		ForceUpdate = FALSE;
#ifdef SUPPORT_REGEN_TIMING
		if (ReGenTiming(VIC)) {
			ReGenTimingEnable = TRUE;
			HDMITX_DEBUG_PRINTF(("USE RE GEN TIMING FORMAT %d\n",
					     (UINT) VIC));
		} else {
			ReGenTimingEnable = FALSE;
			HDMITX_DEBUG_PRINTF(("NO RE GEN TIMING MTACH %d\n",
					     (UINT) VIC));
		}
#else
		VIC = HDMITX_DetectVIC();
#endif
		HDMITX_ChangeDisplayOption(VIC, HDMI_RGB444);
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("VIC = %d\n", (_u16) VIC));
		HDMITX_DEBUG_PRINTF(("H TOTAL = %d\n", GetHTotal()));
		HDMITX_DEBUG_PRINTF(("V TOTAL = %d\n", GetVTotal()));
		HDMITX_DEBUG_PRINTF(("H_Active = %d\n", GetHActive()));
		HDMITX_DEBUG_PRINTF(("V_Active = %d\n", GetVActive()));
		HDMITX_DEBUG_PRINTF(("PCLK = %u0,000\n",
				     (_u16) (VideoPixelClock / 10000)));
#endif
	}

	return TRUE;
}

void InitIT626X_Instance(void)
{
	HDMITX_InitInstance(&InstanceData);
	HPDStatus = FALSE;
	HPDChangeStatus = FALSE;
}

void HDMITX_SetOutput(void)
{
	_u8 uc;
	 /*VIDEOPCLKLEVEL*/ _u8 level;
	unsigned long cTMDSClock = VideoPixelClock * (pixelrep + 1);
	DisableAudioOutput();
	uc = HDMITX_ReadI2C_Byte(0xc1);
	switch (uc & 0x70) {
	case 0x50:
		cTMDSClock *= 5;
		cTMDSClock /= 4;
		break;
	case 0x60:
		cTMDSClock *= 3;
		cTMDSClock /= 2;
	}

	if (cTMDSClock > 80000000L) {
		level = PCLK_HIGH;
	} else if (cTMDSClock > 20000000L) {
		level = PCLK_MEDIUM;
	} else {
		level = PCLK_LOW;
	}

#ifdef SUPPORT_HDCP
	EnableHDCP(FALSE);
	bAuthenticated = FALSE;
#endif
	SetOutputColorDepthPhase(OutputColorDepth, 0);
	SetupVideoInputSignal(InstanceData.bInputVideoSignalType);
	EnableVideoOutput(level, F_MODE_RGB444, F_MODE_RGB444, (_u8) bHDMIMode);
	if (bHDMIMode) {
		ConfigAVIInfoFrame(VIC, pixelrep);
		// ConfigfHdmiVendorSpecificInfoFrame(Side_by_Side);
		ConfigfHdmiVendorSpecificInfoFrame(HDMI3DFormat);
		// ConfigfHdmiVendorSpecificInfoFrame(Frame_Packing);
		// ConfigfHdmiVendorSpecificInfoFrame(Top_and_Botton);
		// #ifdef SUPPORT_HDCP
		//         EnableHDCP(TRUE);
		// #endif
		if (bAudioEnable) {
#ifdef SUPPORT_HBR_AUDIO
			EnableHDMIAudio(T_AUDIO_HBR, FALSE, 768000L,
					OUTPUT_CHANNEL, NULL, cTMDSClock);
#else
			// 2012/07/10 modified by jjtseng
			// found by Tranmin, fix.
			// #ifndef SUPPORT_I2S_AUDIO
#ifdef SUPPORT_I2S_AUDIO
			EnableHDMIAudio(T_AUDIO_LPCM, FALSE,
					INPUT_AUDIO_SAMPLE_FREQ, OUTPUT_CHANNEL,
					NULL, cTMDSClock);
#else
			EnableHDMIAudio(T_AUDIO_LPCM, TRUE,
					INPUT_AUDIO_SAMPLE_FREQ, OUTPUT_CHANNEL,
					NULL, cTMDSClock);
#endif
#endif
			ConfigAudioInfoFrm();
		}
		// 2012/08/16 modified by jjtseng
#ifdef SUPPORT_HDCP
		SetAVMute(TRUE);
#else
		SetAVMute(FALSE);
#endif
		//~jjtseng 2012/08/16
	} else {
		EnableAVIInfoFrame(FALSE, NULL);
		EnableVSInfoFrame(FALSE, NULL);
		SetAVMute(FALSE);
	}
	bChangeMode = FALSE;
#if 0
	while (1) {
		uc = (LVDS_ReadI2C_Byte(0x0b) & (1 << 6));
		if (uc != (1 << 6))
			LVDS_OrREG_TX_Byte(0x0b, (1 << 6));

		uc = (LVDS_ReadI2C_Byte(0x09) & (0x03));
		if (uc != 0x03)
			LVDS_OrREG_TX_Byte(0x09, (0x01));
	}
#endif
}

void MuteUpdata(void)
{
	if (GetAVMute()) {
		if (GetVideoStatus()) {
			HDMITX_AndREG_Byte(0x04, ~(0x14));
			SetAVMute(FALSE);
		}
	}
}

void HDMITX_DevLoopProc(void)
{
	//~jjtseng 2012/08/16
	static _u8 LoopCnt = 0;
	CheckHDMITX(&HPDStatus, &HPDChangeStatus);
	if (HPDChangeStatus) {
		// 2012/08/16 modified by jjtseng
#ifdef SUPPORT_HDCP
		bAuthenticated = FALSE;
#endif
		//~jjtseng 2012/08/16
		LoopCnt = 0;
		if (HPDStatus) {
			ParseEDID();
			bOutputColorMode = F_MODE_RGB444;

			if (RxCapability.ValidHDMI) {
				bHDMIMode = TRUE;

				if (RxCapability.VideoMode & (1 << 6)) {
					bAudioEnable = TRUE;
				}
			} else {
				bHDMIMode = FALSE;
				bAudioEnable = FALSE;
			}
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HPD change HDMITX_SetOutput();\n"));
#endif
			bChangeMode = TRUE;
			//HDMITX_SetOutput();//if system have use VideoModeDetect function

		} else {
			// unplug mode, ...
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HPD OFF DisableVideoOutput()\n"));
#endif
			DisableAudioOutput();
			DisableVideoOutput();
#ifdef SUPPORT_HDCP
			EnableHDCP(FALSE);
#endif
			bAudioEnable = FALSE;
		}
	} else			// no stable but need to process mode change procedure
	{
		LoopCnt++;
		CheckAudioVideoInput();
		if (0 == (LoopCnt & 0x0F) || bChangeMode) {
			if (FALSE == VideoModeDetect()) {
				bChangeMode = TRUE;
				return;
			}
		}
		//===================================
		if (0 == (LoopCnt & 0x03)) {
			AudioModeDetect();
		}

		if (bChangeMode && HPDStatus) {
			HDMITX_SetOutput();
		}
		// 2012/08/16 modified by jjtseng
#ifdef SUPPORT_HDCP
		if (HPDStatus && bAuthenticated == FALSE) {
			bAuthenticated = EnableHDCP(TRUE);
			if (bAuthenticated == TRUE) {
				SetAVMute(FALSE);
			}
		}
#endif
		//~jjtseng 2012/08/16
		if (LoopCnt >= 0xFF)
			LoopCnt = 0;
	}
}

void HDMITX_ChangeDisplayOption(HDMI_Video_Type OutputVideoTiming,
				HDMI_OutputColorMode OutputColorMode)
{
	//HDMI_Video_Type  t=HDMI_480i60_16x9;
	HDMITX_DEBUG_PRINTF(("HDMITX_ChangeDisplayOption(%d)\n",
			     (int)OutputVideoTiming));
	HDMI3DFormat = 0xFF;
	switch (OutputVideoTiming) {
	case HDMI_640x480p60:
		VIC = 1;
		VideoPixelClock = 25000000L;
		pixelrep = 0;
		aspec = HDMI_4x3;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_480p60:
		VIC = 2;
		VideoPixelClock = 27000000L;
		pixelrep = 0;
		aspec = HDMI_4x3;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_480p60_16x9:
		VIC = 3;
		VideoPixelClock = 27000000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_720p60:
		VIC = 4;
		VideoPixelClock = 74250000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	case HDMI_720p60_FP3D:
		VIC = 4;
		VideoPixelClock = 148500000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		HDMI3DFormat = Frame_Packing;
		break;
	case HDMI_1080i60:
		VIC = 5;
		VideoPixelClock = 74250000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	case HDMI_480i60:
		VIC = 6;
		VideoPixelClock = 13500000L;
		pixelrep = 1;
		aspec = HDMI_4x3;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_480i60_16x9:
		VIC = 7;
		VideoPixelClock = 13500000L;
		pixelrep = 1;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_1080p60:
		VIC = 16;
		VideoPixelClock = 148500000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	case HDMI_576p50:
		VIC = 17;
		VideoPixelClock = 27000000L;
		pixelrep = 0;
		aspec = HDMI_4x3;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_576p50_16x9:
		VIC = 18;
		VideoPixelClock = 27000000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_720p50:
		VIC = 19;
		VideoPixelClock = 74250000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	case HDMI_720p50_FP3D:
		VIC = 19;
		VideoPixelClock = 148500000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		HDMI3DFormat = Frame_Packing;
		break;
	case HDMI_1080i50:
		VIC = 20;
		VideoPixelClock = 74250000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	case HDMI_576i50:
		VIC = 21;
		VideoPixelClock = 13500000L;
		pixelrep = 1;
		aspec = HDMI_4x3;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_576i50_16x9:
		VIC = 22;
		VideoPixelClock = 13500000L;
		pixelrep = 1;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU601;
		break;
	case HDMI_1080p50:
		VIC = 31;
		VideoPixelClock = 148500000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	case HDMI_1080p24:
		VIC = 32;
		VideoPixelClock = 74250000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	case HDMI_1080p24_FP3D:
		VIC = 32;
		VideoPixelClock = 2 * 74250000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		HDMI3DFormat = Frame_Packing;
		break;
	case HDMI_1080p25:
		VIC = 33;
		VideoPixelClock = 74250000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	case HDMI_1080p30:
		VIC = 34;
		VideoPixelClock = 74250000L;
		pixelrep = 0;
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
		break;
	default:
		VIC = 0;
		VideoPixelClock = GetInputPclk();
		pixelrep = 0;
		aspec = HDMI_4x3;
		Colorimetry = HDMI_ITU601;
		break;
		//bChangeMode = FALSE ;
		//return ;
	}
	bOutputColorMode = F_MODE_RGB444;

	if (Colorimetry == HDMI_ITU709) {
		bInputColorMode |= F_MODE_ITU709;
	} else {
		bInputColorMode &= ~F_MODE_ITU709;
	}

	if (OutputVideoTiming != HDMI_640x480p60) {
		bInputColorMode |= F_MODE_16_235;
	} else {
		bInputColorMode &= ~F_MODE_16_235;
	}

	bChangeMode = TRUE;
}

void ConfigAVIInfoFrame(_u8 VIC, _u8 pixelrep)
{
//     AVI_InfoFrame AviInfo;

	HDMITX_DEBUG_PRINTF(("ConfigAVIInfoFrame(%d)\n", (int)VIC));
	AviInfo.pktbyte.AVI_HB[0] = AVI_INFOFRAME_TYPE | 0x80;
	AviInfo.pktbyte.AVI_HB[1] = AVI_INFOFRAME_VER;
	AviInfo.pktbyte.AVI_HB[2] = AVI_INFOFRAME_LEN;

	AviInfo.pktbyte.AVI_DB[0] = (0 << 5) | (1 << 4);	// RGB Only

	AviInfo.pktbyte.AVI_DB[1] = 8;
	if (VIC)
		AviInfo.pktbyte.AVI_DB[1] |= (aspec != HDMI_16x9) ? (1 << 4) : (2 << 4);	// 4:3 or 16:9
	else
		AviInfo.pktbyte.AVI_DB[1] &= (0xcf);

	AviInfo.pktbyte.AVI_DB[1] |= (Colorimetry != HDMI_ITU709) ? (1 << 6) : (2 << 6);	// 4:3 or 16:9
	AviInfo.pktbyte.AVI_DB[2] = 0;
	AviInfo.pktbyte.AVI_DB[3] = VIC;
	AviInfo.pktbyte.AVI_DB[4] = pixelrep & 3;
	AviInfo.pktbyte.AVI_DB[5] = 0;
	AviInfo.pktbyte.AVI_DB[6] = 0;
	AviInfo.pktbyte.AVI_DB[7] = 0;
	AviInfo.pktbyte.AVI_DB[8] = 0;
	AviInfo.pktbyte.AVI_DB[9] = 0;
	AviInfo.pktbyte.AVI_DB[10] = 0;
	AviInfo.pktbyte.AVI_DB[11] = 0;
	AviInfo.pktbyte.AVI_DB[12] = 0;

	EnableAVIInfoFrame(TRUE, (unsigned char *)&AviInfo);
}

void ConfigfHdmiVendorSpecificInfoFrame(_u8 _3D_Stru)
{
	VS_Info.pktbyte.VS_HB[0] = VENDORSPEC_INFOFRAME_TYPE | 0x80;
	VS_Info.pktbyte.VS_HB[1] = VENDORSPEC_INFOFRAME_VER;

	VS_Info.pktbyte.VS_DB[1] = 0x03;
	VS_Info.pktbyte.VS_DB[2] = 0x0C;
	VS_Info.pktbyte.VS_DB[3] = 0x00;
	VS_Info.pktbyte.VS_DB[4] = 0x40;
	switch (_3D_Stru) {
	case Side_by_Side:
		VS_Info.pktbyte.VS_HB[2] = 6;
		VS_Info.pktbyte.VS_DB[5] = (_3D_Stru << 4);
		VS_Info.pktbyte.VS_DB[6] = 0x00;
		break;

	case Top_and_Botton:
	case Frame_Packing:
		VS_Info.pktbyte.VS_HB[2] = 5;
		VS_Info.pktbyte.VS_DB[5] = (_3D_Stru << 4);
		break;
	default:
		EnableVSInfoFrame(FALSE, NULL);
		return;
	}

	EnableVSInfoFrame(TRUE, (_u8 *) & VS_Info);
}

////////////////////////////////////////////////////////////////////////////////
// Function: ConfigAudioInfoFrm
// Parameter: NumChannel, number from 1 to 8
// Return: ER_SUCCESS for successfull.
// Remark: Evaluate. The speakerplacement is only for reference.
//         For production, the caller of SetAudioInfoFrame should program
//         Speaker placement by actual status.
// Side-Effect:
////////////////////////////////////////////////////////////////////////////////

void ConfigAudioInfoFrm(void)
{
	int i;
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("ConfigAudioInfoFrm(%d)\n", (_u16) 2));
#endif
	AudioInfo.pktbyte.AUD_HB[0] = AUDIO_INFOFRAME_TYPE;
	AudioInfo.pktbyte.AUD_HB[1] = 1;
	AudioInfo.pktbyte.AUD_HB[2] = AUDIO_INFOFRAME_LEN;
	AudioInfo.pktbyte.AUD_DB[0] = 1;
	for (i = 1; i < AUDIO_INFOFRAME_LEN; i++) {
		AudioInfo.pktbyte.AUD_DB[i] = 0;
	}
	//AudioInfo.pktbyte.AUD_DB[3] = 0x1f ;
	EnableAudioInfoFrame(TRUE, (unsigned char *)&AudioInfo);
}

/////////////////////////////////////////////////////////////////////
// ParseEDID()
// Check EDID check sum and EDID 1.3 extended segment.
/////////////////////////////////////////////////////////////////////

_u8 ParseEDID(void)
{
	// collect the EDID ucdata of segment 0
	_u8 CheckSum;
	_u8 BlockCount;
	_u8 err;
	_u8 bValidCEA = FALSE;
	_u8 i;
	_u8 MoniCnt;
	RxCapability.ValidCEA = FALSE;
	RxCapability.ValidHDMI = FALSE;
	bForceCTS = FALSE;

	GetEDIDData(0, cBuf);

	for (i = 0, CheckSum = 0; i < 128; i++) {
		CheckSum += cBuf[i];
		CheckSum &= 0xFF;
	}

	//Eep_Write(0x80, 0x80, EDID_Buf);
	if (CheckSum != 0) {
		return FALSE;
	}

	if (cBuf[0] != 0x00 ||
	    cBuf[1] != 0xFF ||
	    cBuf[2] != 0xFF ||
	    cBuf[3] != 0xFF || cBuf[4] != 0xFF || cBuf[5] != 0xFF
	    || cBuf[6] != 0xFF || cBuf[7] != 0x00) {
		return FALSE;
	}
	HDMITX_DEBUG_PRINTF(("[0x08+0]=%X\n[0x08+1]=%X\n[0x08+2]=%X\n[0x08+3]=%X\n[0x08+4]=%X\n[0x08+5]=%X\n[0x08+6]=%X\n[0x08+7]=%X\n", (_u16) cBuf[0x08 + 0], (_u16) cBuf[0x08 + 1], (_u16) cBuf[0x08 + 2], (_u16) cBuf[0x08 + 3], (_u16) cBuf[0x08 + 4], (_u16) cBuf[0x08 + 5], (_u16) cBuf[0x08 + 6], (_u16) cBuf[0x08 + 7]));
	for (MoniCnt = 0; MoniCnt < UNKNOW_Monitor; MoniCnt++) {
		_u8 ccCnt;
		for (ccCnt = 0; ccCnt < 8; ccCnt++) {
			if (cBuf[0x08 + ccCnt] !=
			    ForceCTS_EDID_Vindeo[MoniCnt].VIDC[ccCnt]) {
				break;
			}

		}
		if (ccCnt == 8) {
			bForceCTS = TRUE;
			break;
		}
	}
	if (bForceCTS) {
		HDMITX_DEBUG_PRINTF(("bForceCTS = TRUE\n"));
	} else {
		HDMITX_DEBUG_PRINTF(("bForceCTS = FALSE\n"));
	}

	BlockCount = cBuf[0x7E];

	if (BlockCount == 0) {
		return TRUE;	// do nothing.
	} else if (BlockCount > 4) {
		BlockCount = 4;
	}
	RxCapability.ValidHDMI = FALSE;
	// read all segment for test
	for (i = 1; i <= BlockCount; i++) {
		err = GetEDIDData(i, cBuf);

		if (err) {
			if (!RxCapability.ValidHDMI && cBuf[0] == 0x2
			    && cBuf[1] == 0x3) {
				err = ParseCEAEDID(cBuf);
				if (err) {
					if (RxCapability.IEEEOUI == 0x0c03) {
						bValidCEA = TRUE;
					}
				}
			}
		}
	}
	for (i = 0; i < 128; i++)
		cBuf[i] = 0;
	return err;

}

static BOOL ParseCEAEDID(_u8 * pCEAEDID)
{
	_u8 offset, End;
	_u8 count;
	_u8 tag;
	int i;

	if (pCEAEDID[0] != 0x02 || pCEAEDID[1] != 0x03)
		return ER_SUCCESS;	// not a CEA BLOCK.
	End = pCEAEDID[2];	// CEA description.
	RxCapability.VideoMode = pCEAEDID[3];

	RxCapability.VDOModeCount = 0;
	RxCapability.idxNativeVDOMode = 0xff;

	for (offset = 4; offset < End;) {
		tag = pCEAEDID[offset] >> 5;
		count = pCEAEDID[offset] & 0x1f;
		switch (tag) {
		case 0x01:	// Audio Data Block ;
			RxCapability.AUDDesCount = count / 3;
			offset++;
			for (i = 0; i < RxCapability.AUDDesCount; i++) {
				RxCapability.AUDDes[i].uc[0] =
				    pCEAEDID[offset++];
				RxCapability.AUDDes[i].uc[1] =
				    pCEAEDID[offset++];
				RxCapability.AUDDes[i].uc[2] =
				    pCEAEDID[offset++];
			}

			break;

		case 0x02:	// Video Data Block ;
			//RxCapability.VDOModeCount = 0 ;
			offset++;
			for (i = 0, RxCapability.idxNativeVDOMode = 0xff;
			     i < count; i++, offset++) {
				_u8 VIC;
				VIC = pCEAEDID[offset] & (~0x80);
				// if( FindModeTableEntryByVIC(VIC) != -1 )
				{
					RxCapability.VDOMode[RxCapability.
							     VDOModeCount] =
					    VIC;
					if (pCEAEDID[offset] & 0x80) {
						RxCapability.idxNativeVDOMode =
						    (_u8)
						    RxCapability.VDOModeCount;
						iVideoModeSelect =
						    RxCapability.VDOModeCount;
					}

					RxCapability.VDOModeCount++;
				}
			}
			break;

		case 0x03:	// Vendor Specific Data Block ;
			for (i = 0; i < count; i++) {
				RxCapability.HDMI_VSDB[i] =
				    pCEAEDID[i + offset];
			}
#ifdef Debug_message
			for (i = 0; i < count; i++) {
				HDMITX_DEBUG_PRINTF(("HDMI_VSDB[%d]= %X\n", i,
						     (_u16)
						     RxCapability.HDMI_VSDB
						     [i]));
			}
#endif

			offset++;
			RxCapability.IEEEOUI = (_u32) pCEAEDID[offset + 2];
			RxCapability.IEEEOUI <<= 8;
			RxCapability.IEEEOUI += (_u32) pCEAEDID[offset + 1];
			RxCapability.IEEEOUI <<= 8;
			RxCapability.IEEEOUI += (_u32) pCEAEDID[offset];
			offset += count;	// ignore the remaind.
			if (RxCapability.IEEEOUI == 0x0c03) {
				RxCapability.ValidHDMI = TRUE;
			}
			break;

		case 0x04:	// Speaker Data Block ;
			offset++;
			RxCapability.SpeakerAllocBlk.uc[0] = pCEAEDID[offset];
			RxCapability.SpeakerAllocBlk.uc[1] =
			    pCEAEDID[offset + 1];
			RxCapability.SpeakerAllocBlk.uc[2] =
			    pCEAEDID[offset + 2];
			offset += 3;
			break;
		case 0x05:	// VESA Data Block ;
			offset += count + 1;
			break;
		case 0x07:	// Extended Data Block ;
			offset += count + 1;	//ignore
			break;
		default:
			offset += count + 1;	// ignore
		}
	}
	RxCapability.ValidCEA = TRUE;
	return TRUE;
}

//////////////////////////////////////////////////////////////////////
// utility function for main..
//////////////////////////////////////////////////////////////////////
/*
struct DE_ONLY_Setting {
    _u8 fmt ;
    _u8 Reg90 ; // Reg90
    _u8 Reg91 ; // Reg91
    _u8 Reg95 ; // Reg95
    _u8 Reg96 ; // Reg96
    _u8 Reg97 ; // Reg97
    _u8 RegA0 ; // RegA0
    _u8 RegA1 ; // RegA1
    _u8 RegA2 ; // RegA2
    _u8 RegA3 ; // RegA3

    _u32 PCLK ;
    _u8 VFreq ;
}DE_ONLY_Setting ;

static _CODE struct DE_ONLY_Setting AONVISION_DE_ONLY_Table[] =
{
    {   0,0x08,0x31,0x58,0x3c,0x03,0x0b,0x42,0xFF,0xFF,27000000,60},
    {   1,0x08,0x31,0x58,0x3c,0x03,0x0b,0x42,0xFF,0xFF,27000000,60},
    {   2,0x08,0x31,0x58,0x3c,0x03,0x0b,0x42,0xFF,0xFF,27000000,60},
    {   3,0x08,0x31,0x58,0x3c,0x03,0x0b,0x42,0xFF,0xFF,27000000,60},
};
*/
//////////////////////////////////////////////////////////////////////
// external Interface                                                         //
//////////////////////////////////////////////////////////////////////
_u8 LVDS_ReadI2C_Byte(_u8 RegAddr)
{
	//i2c_read_byte(0x66, RegAddr, 1, &p_data, I2CDEV);
	_u8 p_data = 0;
	int ret = 0;
	ret =
	    i2c_read_reg(it6263_platform->LVDS_client, (unsigned int)RegAddr,
			 &p_data);
	if (ret < 0)
		HDMITX_DEBUG_PRINTF(("LVDS Read 0x%x fail\n",
				     (unsigned int)RegAddr));
	return p_data;
}

int LVDS_WriteI2C_Byte(_u8 RegAddr, _u8 d)
{
	int ret = 0;
	ret =
	    i2c_write_reg(it6263_platform->LVDS_client, (unsigned int)RegAddr,
			  d);
	if (ret < 0)
		HDMITX_DEBUG_PRINTF(("LVDS write 0x%x fail\n",
				     (unsigned int)RegAddr));
	return ret;
}

void InitLVDS(void)
{
	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(0x1d, 0x66);
	HDMITX_WriteI2C_Byte(0x1E, 0x01);
#ifdef SUPPORT_REGEN_TIMING
	EnableDeOnly(FALSE);
#endif
	ResetLVDS();
	delay1ms(10);
	SetLVDSinterface();
	delay1ms(10);
	SetLVDS_AFE();
	delay1ms(500);
}

void ResetLVDS(void)
{
	HDMITX_DEBUG_PRINTF(("reset lvds  \n"));
	LVDS_AndREG_TX_Byte(0x3c, 0xfe);	//LVDS AFE PLL RESET
	delay1ms(1);
	LVDS_OrREG_TX_Byte(0x3c, 0x01);

	LVDS_OrREG_TX_Byte(0x05, 0x02);	//RESET LVDS PCLK
	delay1ms(1);
	LVDS_AndREG_TX_Byte(0x05, 0xfd);
#ifdef IT626X_SSC
	LVDS_OrREG_TX_Byte(0x2c, 0x40);	//SRAM RESET
	delay1ms(1);
	LVDS_AndREG_TX_Byte(0x2c, 0xbf);
#endif
	HDMITX_DEBUG_PRINTF(("reset lvds end \n"));

}

void SetLVDS_AFE(void)
{
	LVDS_WriteI2C_Byte(0x3e, 0xaa);	//dont ask me "What is it"
	LVDS_WriteI2C_Byte(0x3f, 0x02);	//dont ask me "What is it"
	LVDS_WriteI2C_Byte(0x47, 0xaa);	//dont ask me "What is it"
	LVDS_WriteI2C_Byte(0x48, 0x02);	//dont ask me "What is it"
	LVDS_WriteI2C_Byte(0x4f, 0x11);	//dont ask me "What is it"

	LVDS_OrREG_TX_Byte(0x0b, 0x01);
#ifdef IT626X_SSC
	LVDS_OrREG_TX_Byte(0x3c, 0x07);	//[1]PLL POWER DOWN [2]RCLK SELECT
	LVDS_AndREG_TX_Byte(0x2c, (~(1 << 6)));

	LVDS_OrREG_TX_Byte(0x2c, 0x40);	//SRAM RESET
	delay1ms(1);
	LVDS_AndREG_TX_Byte(0x2c, 0xbf);
#else
	LVDS_AndREG_TX_Byte(0x3c, (~0x07));
	LVDS_OrREG_TX_Byte(0x2c, (1 << 6));
#endif
	if (100000000L < GetInputPclk())
		LVDS_AndREG_TX_Byte(0x39, 0x3f);
	else
		LVDS_OrREG_TX_Byte(0x39, 0xc0);

#ifdef SUPPORT_MAP3
	LVDS_OrREG_TX_Byte(0x2c, (1 << 4));
#else
	LVDS_AndREG_TX_Byte(0x2c, ~(1 << 4));
#endif

	LVDS_OrREG_TX_Byte(0x39, 0x02);
	LVDS_AndREG_TX_Byte(0x39, 0xfd);

//    LVDS_WriteI2C_Byte(0x2c,0x51);//dont ask me "What is it"
	delay1ms(20);

}

BOOL SetLVDSinterface(void)
{
	_u8 uc;
	uc = (LVDS_ReadI2C_Byte(0x2c) & 0x03);
	// Switch_HDMITX_Bank(0);
	// HDMITX_WriteI2C_Byte(0x1d,0x66);
#ifdef SUPPORT_LVDS_8_BIT
	if (0x01 != uc) {
		LVDS_AndREG_TX_Byte(0x2c, 0xfc);	//[1:0]=("00"-6 bit),("01"-8 bit),("10"-10 bit)
		LVDS_OrREG_TX_Byte(0x2c, 0x01);
	}
#endif
#ifdef SUPPORT_LVDS_6_BIT
	if (0x00 != uc) {
		LVDS_AndREG_TX_Byte(0x2c, 0xfc);	//[1:0]=("00"-6 bit),("01"-8 bit),("10"-10 bit)
		LVDS_OrREG_TX_Byte(0x2c, 0x00);
	}
#endif
#ifdef SUPPORT_LVDS_10_BIT
	if (0x02 != uc) {
		LVDS_AndREG_TX_Byte(0x2c, 0xfc);	//[1:0]=("00"-6 bit),("01"-8 bit),("10"-10 bit)
		LVDS_OrREG_TX_Byte(0x2c, 0x02);
	}
#endif

	uc = (LVDS_ReadI2C_Byte(0x2c) & (1 << 7));
#ifdef SUPPORT_DISO
	if (uc != (1 << 7)) {
		LVDS_OrREG_TX_Byte(0x2c, (1 << 7));
		LVDS_OrREG_TX_Byte(0x52, (1 << 1));
	}
#else
	if (uc != (0 << 7)) {
		LVDS_AndREG_TX_Byte(0x2c, (~(1 << 7)));
		LVDS_AndREG_TX_Byte(0x52, (~(1 << 1)));
	}
#endif
	return TRUE;
}

_u16 GetInputClockCount(void)
{
	_u8 u1, u2;
	_u16 temp1;
	u1 = LVDS_ReadI2C_Byte(0x58);
	u2 = LVDS_ReadI2C_Byte(0x57);
	temp1 = ((_u16) u1 << 8);
	temp1 += u2;
	return temp1;
}

_u32 GetInputPclk(void)
{
	_u8 u3, StableCnt = 0, LoopCnt = 20;
	_u16 LastTemp = 0;
	_u16 temp1;

	while (LoopCnt--) {
		temp1 = GetInputClockCount();
		if ((temp1 < (LastTemp - (LastTemp >> 6)))
		    || (temp1 > (LastTemp + (LastTemp >> 6)))) {
			StableCnt = 0;
		} else {
			StableCnt++;
			//delay1ms(temp1%5);
		}
		if (StableCnt > 5)
			break;
		LastTemp = temp1;
	}

	u3 = LVDS_ReadI2C_Byte(0x2c);
	if (u3 & 0x80)
		temp1 /= 2;
	if (0x40 & u3) {
		return ((49000000L / temp1) << 8);
	} else {
		return ((27000000L / temp1) << 10);
	}
}

BOOL CheckLVDS(void)
{
	_u8 u1;
	static _u8 LVDS_UnStable_Cnt = 0;
	SetLVDSinterface();
	u1 = (0x03 & LVDS_ReadI2C_Byte(0x30));
	if (0x03 != u1)		//0x30[1] LVDS Lock [0] LVDS Stable
	{
		//DisableAudioOutput();

		LVDS_UnStable_Cnt++;
	} else {
		LVDS_UnStable_Cnt = 0;
		return TRUE;
	}
	if (LVDS_UnStable_Cnt > 0) {
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("Reset LVDS\n"));
#endif
		// InitIT626X();
		InitLVDS();
		LVDS_UnStable_Cnt = 0;
	}
	return FALSE;
}

//===================================================================

_u8 HDMITX_DetectVIC(void)
{
	_u8 i;
	UINT HTotal, VTotal, HActive, VActive;

	for (i = 0; i < MaxIndex; i++) {
		EnableHVToolDetect(TRUE);
		HTotal = GetHTotal();
		VTotal = GetVTotal();
		HActive = GetHActive();
		VActive = GetVActive();

		if ((TimingTable[i].VEC[0] == HTotal) &&
		    (TimingTable[i].VEC[1] == HActive) &&
		    ((TimingTable[i].VEC[5] % 2048) == VTotal)
		    && ((TimingTable[i].VEC[6] % 2048) == VActive)) {
			return TimingTable[i].fmt;
		}

	}
	return 0;
}

#ifdef SUPPORT_REGEN_TIMING

BOOL ReGenTiming(_u8 fmt_assign)
{
	UINT HRS, HRE;
	UINT HDES, HDEE;
	UINT VRS, VRE;
	UINT VDES, VDEE;
	UINT VRS2ndRise, VRS2nd, VRE2nd;
	UINT VDES2nd, VDEE2nd;
	UINT HTotal, VTotal;
	UINT i;
	UINT H_Total, H_FBH, H_SyncW, H_BBH, H_DEW;
	UINT V_Total, V_FBH, V_SyncW, V_BBH, V_DEW;
	UINT SyncPol;
	_u8 fmt_index;
	_u8 interlaced;
	_u8 reg90, reg91, reg92, reg93, reg94, reg95, reg96, reg97, reg98;
	_u8 reg99, reg9A, reg9B, reg9C, reg9D, reg9E, reg9F, regA0, regA1,
	    regA2, regA3;
	_u8 regA4, regA5;
	fmt_index = 0;
	for (i = 0; i < MaxIndex; i++) {
		if (TimingTable[i].fmt == fmt_assign)
			fmt_index = i;
	}
	if (i == MaxIndex && fmt_index == 0) {
		EnableDeOnly(FALSE);
		return FALSE;
	}
	interlaced = TimingTable[fmt_index].interlaced;

	H_Total = TimingTable[fmt_index].VEC[0];
	H_DEW = TimingTable[fmt_index].VEC[1];
	H_FBH = TimingTable[fmt_index].VEC[2];
	H_SyncW = TimingTable[fmt_index].VEC[3];
	H_BBH = TimingTable[fmt_index].VEC[4];
	V_Total = TimingTable[fmt_index].VEC[5];
	V_DEW = TimingTable[fmt_index].VEC[6];
	V_FBH = TimingTable[fmt_index].VEC[7];
	V_SyncW = TimingTable[fmt_index].VEC[8];
	V_BBH = TimingTable[fmt_index].VEC[9];
	SyncPol = TimingTable[fmt_index].VEC[10];

	HTotal = H_Total - 1;
	VTotal = V_Total - 1;
	HRS = H_FBH - 1;
	HRE = HRS + H_SyncW;
	HDES = HRE + H_BBH;
	HDEE = HDES + H_DEW;

	VRS = V_FBH;
	VRE = VRS + V_SyncW;
	VDES = VRE + V_BBH;
	VDEE = (VDES + V_DEW) % V_Total;
	VRS2nd = VDEE + V_FBH;
	VRE2nd = VRS2nd + V_SyncW;
	VDES2nd = VRS2nd + V_BBH;
	VDEE2nd = (VDES2nd + V_DEW) % V_Total;
	VRS2ndRise = H_Total / 2 + HRS;
	VDES = VRE + V_BBH - 1;
	VDEE = (VDES + V_DEW) % V_Total;
	VDES2nd = VRS2nd + V_SyncW + V_BBH;
	VDEE2nd = (VDES2nd + V_DEW) % V_Total;

	reg90 = ((HTotal & 0xF) << 4) + (SyncPol << 1);
	reg91 = ((HTotal & 0xFF0) >> 4);
	reg92 = HDES & 0xFF;
	reg93 = HDEE & 0xFF;
	reg94 = ((HDEE & 0xF00) >> 4) + ((HDES & 0xF00) >> 8);

	reg95 = HRS & 0xFF;
	reg96 = HRE & 0xFF;
	reg97 = ((HRE & 0xF00) >> 4) + ((HRS & 0xF00) >> 8);
	reg98 = VTotal & 0xFF;
	reg99 = (VTotal & 0xF00) >> 8;
	reg9A = VDES & 0xFF;
	reg9B = VDEE & 0xFF;
	reg9C = ((VDEE & 0xF00) >> 4) + ((VDES & 0xF00) >> 8);
	reg9D = VDES2nd & 0xFF;
	reg9E = VDEE2nd & 0xFF;
	reg9F = ((VDEE2nd & 0xF00) >> 4) + ((VDES2nd & 0xF00) >> 8);
	regA0 = VRS & 0xFF;
	regA1 = ((VRE & 0xF) << 4) + ((VRS & 0xF00) >> 8);

	regA2 = VRS2nd & 0xFF;
	if (interlaced == 1)
		regA3 = ((VRE2nd & 0xF) << 4) + ((VRS2nd & 0xF00) >> 8);
	else
		regA3 = 0xFF;

	regA4 = VRS2ndRise & 0xFF;
	regA5 = 0x20 + ((VRS2ndRise & 0xF00) >> 8);
	EnableHVToolDetect(FALSE);

	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(0x90, reg90);
	HDMITX_WriteI2C_Byte(0x91, reg91);
	HDMITX_WriteI2C_Byte(0x92, reg92);
	HDMITX_WriteI2C_Byte(0x93, reg93);
	HDMITX_WriteI2C_Byte(0x94, reg94);
	HDMITX_WriteI2C_Byte(0x95, reg95);
	HDMITX_WriteI2C_Byte(0x96, reg96);
	HDMITX_WriteI2C_Byte(0x97, reg97);
	HDMITX_WriteI2C_Byte(0x98, reg98);
	HDMITX_WriteI2C_Byte(0x99, reg99);
	HDMITX_WriteI2C_Byte(0x9a, reg9A);
	HDMITX_WriteI2C_Byte(0x9b, reg9B);
	HDMITX_WriteI2C_Byte(0x9c, reg9C);
	HDMITX_WriteI2C_Byte(0x9d, reg9D);
	HDMITX_WriteI2C_Byte(0x9e, reg9E);
	HDMITX_WriteI2C_Byte(0x9f, reg9F);
	HDMITX_WriteI2C_Byte(0xa0, regA0);
	HDMITX_WriteI2C_Byte(0xa1, regA1);
	HDMITX_WriteI2C_Byte(0xa2, regA2);
	HDMITX_WriteI2C_Byte(0xa3, regA3);
	HDMITX_WriteI2C_Byte(0xa4, regA4);
	//EnableDeOnly(TRUE);
	HDMITX_WriteI2C_Byte(0xa5, regA5);
	HDMITX_AndREG_Byte(0xa8, ~(1 << 3));
	return TRUE;
}
#endif
//===================================================================
void HDMITX_InitInstance(INSTANCE * pInstance)
{
	if (pInstance && 0 < HDMITX_INSTANCE_MAX) {
		Instance[0] = *pInstance;
	}
	Instance[0].bAudFs = AUDFS_OTHER;
}

void InitIT626X(void)
{
	_u8 intclr;
	HDMITX_DEBUG_PRINTF(("InitIT626X start \n"));
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST,
			     B_REF_RST | B_VID_RST | B_AUD_RST | B_AREF_RST |
			     B_HDCP_RST);
	delay1ms(1);

	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(REG_TX_INT_CTRL, Instance[0].bIntType);
	Instance[0].bIntPOL =
	    (Instance[0].bIntType & B_INTPOL_ACTH) ? TRUE : FALSE;
	Instance[0].TxEMEMStatus = TRUE;

	HDMITX_WriteI2C_Byte(REG_TX_SW_RST,
			     B_AUD_RST | B_AREF_RST | B_HDCP_RST);
	InitLVDS();
	// Avoid power loading in un play status.
	HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL,
			     B_AFE_DRV_RST | B_AFE_DRV_PWD);
	//////////////////////////////////////////////////////////////////
	// Setup HDCP ROM
	//////////////////////////////////////////////////////////////////

	//InitIT626X_HDCPROM();
	HDMITX_AndREG_Byte(0xF3, ~0x30);
	HDMITX_OrREG_Byte(0xF3, 0x20);
	// set interrupt mask,mask value 0 is interrupt available.
	//HDMITX_WriteI2C_Byte(REG_TX_INT_MASK1,0xB2);
	HDMITX_WriteI2C_Byte(REG_TX_INT_MASK1, 0x30);
	HDMITX_WriteI2C_Byte(REG_TX_INT_MASK2, 0xF8);
	HDMITX_WriteI2C_Byte(REG_TX_INT_MASK3, 0x37);
	Switch_HDMITX_Bank(0);
	DISABLE_NULL_PKT();
	DISABLE_ACP_PKT();
	DISABLE_ISRC1_PKT();
	DISABLE_ISRC2_PKT();
	DISABLE_AVI_INFOFRM_PKT();
	DISABLE_AUD_INFOFRM_PKT();
	DISABLE_SPD_INFOFRM_PKT();
	DISABLE_MPG_INFOFRM_PKT();
	Switch_HDMITX_Bank(1);
	HDMITX_AndREG_Byte(REG_TX_AVIINFO_DB1, ~(3 << 5));
	Switch_HDMITX_Bank(0);
	//////////////////////////////////////////////////////////////////
	// Setup Output Audio format.
	//////////////////////////////////////////////////////////////////
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL1, Instance[0].bOutputAudioMode);	// regE1 bOutputAudioMode should be loaded from ROM image.

	HDMITX_WriteI2C_Byte(REG_TX_INT_CLR0, 0xFF);
	HDMITX_WriteI2C_Byte(REG_TX_INT_CLR1, 0xFF);
	intclr =
	    (HDMITX_ReadI2C_Byte(REG_TX_SYS_STATUS)) | B_CLR_AUD_CTS |
	    B_INTACTDONE;
	HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, intclr);	// clear interrupt.
	intclr &= ~(B_INTACTDONE);
	HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, intclr);	// INTACTDONE reset to zero.

	{
		_u32 n = 6144L;
		Switch_HDMITX_Bank(1);
		HDMITX_WriteI2C_Byte(REGPktAudN0, (_u8) ((n) & 0xFF));
		HDMITX_WriteI2C_Byte(REGPktAudN1, (_u8) ((n >> 8) & 0xFF));
		HDMITX_WriteI2C_Byte(REGPktAudN2, (_u8) ((n >> 16) & 0xF));
		Switch_HDMITX_Bank(0);
		HDMITX_WriteI2C_Byte(0xc4, 0xfe);
		HDMITX_OrREG_Byte(REG_TX_PKT_SINGLE_CTRL, (1 << 4) | (1 << 5));
	}
	HDMITX_DEBUG_PRINTF(("InitIT626X  end \n"));
}

//////////////////////////////////////////////////////////////////////
// export this for dynamic change input signal
//////////////////////////////////////////////////////////////////////
BOOL SetupVideoInputSignal(_u8 inputSignalType)
{
	Instance[0].bInputVideoSignalType = inputSignalType;
	// SetInputMode(inputColorMode,Instance[0].bInputVideoSignalType);
	return TRUE;
}

BOOL EnableVideoOutput( /*VIDEOPCLKLEVEL*/ _u8 level, _u8 inputColorMode,
		       _u8 outputColorMode, _u8 bHDMI)
{
	// bInputVideoMode,bOutputVideoMode,Instance[0].bInputVideoSignalType,bAudioInputType,should be configured by upper F/W or loaded from EEPROM.
	// should be configured by initsys.c
	// VIDEOPCLKLEVEL level ;

	Instance[0].bHDMIMode = (_u8) bHDMI;

	SetAVMute(TRUE);
	//SetInputMode(inputColorMode,Instance[0].bInputVideoSignalType);
	//SetCSCScale(inputColorMode,outputColorMode);

	Switch_HDMITX_Bank(1);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB1, 0x10);
	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(0x70, 0);
	HDMITX_WriteI2C_Byte(0x71, 0);
	if (Instance[0].bHDMIMode) {
		HDMITX_WriteI2C_Byte(REG_TX_HDMI_MODE, B_TX_HDMI_MODE);
	} else {
		HDMITX_WriteI2C_Byte(REG_TX_HDMI_MODE, B_TX_DVI_MODE);
	}

#ifdef INVERT_VID_LATCHEDGE
	uc = HDMITX_ReadI2C_Byte(REG_TX_CLK_CTRL1);
	uc |= B_VDO_LATCH_EDGE;
	HDMITX_WriteI2C_Byte(REG_TX_CLK_CTRL1, uc);
#endif

	SetupAFE(level);	// pass if High Freq request
	FireAFE();

	HDMITX_WriteI2C_Byte(REG_TX_INT_CLR0, 0);
	HDMITX_WriteI2C_Byte(REG_TX_INT_CLR1, B_CLR_VIDSTABLE);
	HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, B_INTACTDONE);

	return TRUE;
}

BOOL GetEDIDData(int EDIDBlockID, _u8 * pEDIDData)
{
	if (!pEDIDData) {
		return FALSE;
	}

	if (ReadEDID(pEDIDData, EDIDBlockID / 2, (EDIDBlockID % 2) * 128, 128)
	    == ER_FAIL) {
		return FALSE;
	}

	return TRUE;
}

#ifdef SUPPORT_HDCP
BOOL EnableHDCP(_u8 bEnable)
{
	if (bEnable) {
		if (ER_FAIL == HDCP_Authenticate()) {
			// 2012/08/16 modified by jj
			// HDCP_ResetAuth();
			// when HDCP fail, do not reset Auth.
			//~ 2012/08/16 modified by jj
			return FALSE;
		}
	} else {
		HDCP_CancelRepeaterAuthenticate();
		HDCP_ResetAuth();
		Instance[0].bAuthenticated = FALSE;
	}
	return TRUE;
}
#endif

_u8 CheckHDMITX(_u8 * pHPD, _u8 * pHPDChange)
{
	_u8 intdata1, intdata2, intdata3, sysstat;
	_u8 intclr3 = 0;
	_u8 PrevHPD = Instance[0].bHPD;
	_u8 HPD;

	sysstat = HDMITX_ReadI2C_Byte(REG_TX_SYS_STATUS);
#ifdef Debug_message
	//HDMITX_DEBUG_PRINTF(("REG_TX_SYS_STATUS = %X \n",(_u16)sysstat));
#endif
	//if((sysstat&(1<<4)))
	//    SetupAudioChannel();

	if ((sysstat & (B_HPDETECT /*|B_RXSENDETECT */ )) ==
	    (B_HPDETECT /*|B_RXSENDETECT */ )) {
		HPD = TRUE;
	} else {
		HPD = FALSE;
	}
	// 2007/06/20 added by jj_tseng@chipadvanced.com

	if (pHPDChange) {
		*pHPDChange = FALSE;
	}
	//~jj_tseng@chipadvanced.com 2007/06/20

	if (HPD == FALSE) {
		Instance[0].bAuthenticated = FALSE;
	}

	if (sysstat & B_INT_ACTIVE) {

		intdata1 = HDMITX_ReadI2C_Byte(REG_TX_INT_STAT1);
		if (intdata1 & B_INT_AUD_OVERFLOW) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("B_INT_AUD_OVERFLOW.\n"));
#endif
			HDMITX_OrREG_Byte(0xc5, (1 << 4));
			HDMITX_OrREG_Byte(REG_TX_SW_RST,
					  (B_AUD_RST | B_AREF_RST));
			HDMITX_AndREG_Byte(REG_TX_SW_RST,
					   ~(B_AUD_RST | B_AREF_RST));
			AudioDelayCnt = AudioOutDelayCnt;
			LastRefaudfreqnum = 0;
		}
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("INT_Handler: reg%X = %X\n",
				     (_u16) REG_TX_INT_STAT1, (_u16) intdata1));
#endif
		if (intdata1 & B_INT_DDCFIFO_ERR) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("DDC FIFO Error.\n"));
#endif
			ClearDDCFIFO();
		}

		if (intdata1 & B_INT_DDC_BUS_HANG) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("DDC BUS HANG.\n"));
#endif
			AbortDDC();
#ifdef SUPPORT_HDCP
			if (Instance[0].bAuthenticated) {
#ifdef Debug_message
				HDMITX_DEBUG_PRINTF(("when DDC hang,and aborted DDC,the HDCP authentication need to restart.\n"));
#endif
				HDCP_ResumeAuthentication();
			}
#endif
		}

		if (intdata1 & (B_INT_HPD_PLUG | B_INT_RX_SENSE)) {
			if (pHPDChange && (Instance[0].bHPD != HPD)) {
				*pHPDChange = TRUE;
#ifdef Debug_message
				HDMITX_DEBUG_PRINTF(("2-*pHPDChange = %X\n",
						     (_u16) * pHPDChange));
#endif
			}
			if (HPD == FALSE) {
#ifdef Debug_message
				HDMITX_DEBUG_PRINTF(("HPD = %X\n", (_u16) HPD));
#endif
				HDMITX_WriteI2C_Byte(REG_TX_SW_RST,
						     B_AREF_RST | B_VID_RST |
						     B_AUD_RST | B_HDCP_RST);
				delay1ms(1);
				HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL,
						     B_AFE_DRV_RST |
						     B_AFE_DRV_PWD);
				//HDMITX_DEBUG_PRINTF(("Unplug,%x %x\n",HDMITX_ReadI2C_Byte(REG_TX_SW_RST),HDMITX_ReadI2C_Byte(REG_TX_AFE_DRV_CTRL)));
				// VState = TXVSTATE_Unplug ;
				LastRefaudfreqnum = 0;
			}
		}

		intdata2 = HDMITX_ReadI2C_Byte(REG_TX_INT_STAT2);
		HDMITX_DEBUG_PRINTF(("INT_Handler: reg%X = %X\n",
				     (_u16) REG_TX_INT_STAT2, (_u16) intdata2));

#ifdef SUPPORT_HDCP
		if (intdata2 & B_INT_AUTH_DONE) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("interrupt Authenticate Done.\n"));
#endif
			HDMITX_OrREG_Byte(REG_TX_INT_MASK2,
					  (_u8) B_T_AUTH_DONE_MASK);
			Instance[0].bAuthenticated = TRUE;
			SetAVMute(FALSE);
		}

		if (intdata2 & B_INT_AUTH_FAIL) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("reg46 = %02x reg16 = %02x reg1A = %02x\n", (_u16) HDMITX_ReadI2C_Byte(0x46), (_u16) HDMITX_ReadI2C_Byte(0x16), (_u16) HDMITX_ReadI2C_Byte(0x1A)));
			HDMITX_DEBUG_PRINTF(("interrupt Authenticate Fail.\n"));
#endif
			SetAVMute(TRUE);
			AbortDDC();	// @emily add
			HDCP_ResumeAuthentication();
		}
#endif // SUPPORT_HDCP

		intdata3 = HDMITX_ReadI2C_Byte(REG_TX_INT_STAT3);
		if (intdata3 & B_INT_VIDSTABLE) {
			sysstat = HDMITX_ReadI2C_Byte(REG_TX_SYS_STATUS);
			if (Instance[0].bVideoOut == FALSE
			    || sysstat & B_TXVIDSTABLE) {
				//CurrCTS=0;
				//LastRefaudfreqnum=0;
				bChangeMode = TRUE;
//#ifdef Debug_message
//                HDMITX_DEBUG_PRINTF(("B_INT_VIDSTABLE Instance[0].bVideoOut==FALSE\n"));
//#endif
			} else {
				//DisableAudioOutput();
				Instance[0].bVideoOut = FALSE;
//#ifdef Debug_message
//                HDMITX_DEBUG_PRINTF(("B_INT_VIDSTABLE Instance[0].bVideoOut==TRUE\n"));
//#endif
			}
		}
		HDMITX_WriteI2C_Byte(REG_TX_INT_CLR0, 0xFF);
		HDMITX_WriteI2C_Byte(REG_TX_INT_CLR1, 0xFF);
		intclr3 =
		    (HDMITX_ReadI2C_Byte(REG_TX_SYS_STATUS)) | B_CLR_AUD_CTS |
		    B_INTACTDONE;
		HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, intclr3);	// clear interrupt.
		intclr3 &= ~(B_INTACTDONE);
		HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, intclr3);	// INTACTDONE reset to zero.
	} else {
		if (pHPDChange) {
			if (HPD != PrevHPD) {
				*pHPDChange = TRUE;
			} else {
				*pHPDChange = FALSE;
			}
		}
	}
	if (pHPDChange) {
		if ((*pHPDChange == TRUE) && (HPD == FALSE)) {
			HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL,
					     B_AFE_DRV_RST | B_AFE_DRV_PWD);
		}
	}

	if (pHPD) {
		*pHPD = HPD;
	}

	Instance[0].bHPD = HPD;
	return HPD;
}

void DisableIT626X(void)
{
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST,
			     B_AREF_RST | B_VID_RST | B_AUD_RST | B_HDCP_RST);
	delay1ms(1);
	HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL,
			     B_AFE_DRV_RST | B_AFE_DRV_PWD);
}

void DisableVideoOutput(void)
{
	_u8 uc = HDMITX_ReadI2C_Byte(REG_TX_SW_RST) | B_VID_RST;
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST, uc);
	HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL,
			     B_AFE_DRV_RST | B_AFE_DRV_PWD);
	Instance[0].bVideoOut = FALSE;
}

void DisableAudioOutput(void)
{
	//UINT msec,count;
	HDMITX_OrREG_Byte(REG_TX_PKT_SINGLE_CTRL, (1 << 5));
	SetAVMute(TRUE);
	/*
	   for(msec=0;msec<32;msec++)
	   {
	   for(count=0;count<2048;count++)
	   {
	   _nop_();_nop_();_nop_();_nop_();
	   _nop_();_nop_();_nop_();_nop_();
	   }
	   }
	 */
	HDMITX_OrREG_Byte(REG_TX_SW_RST, B_AUD_RST | B_AREF_RST);
	//HDMITX_AndREG_Byte(0x59,~(1<<2)); //for test
	AudioDelayCnt = AudioOutDelayCnt;
	LastRefaudfreqnum = 0;
}

static SYS_STATUS SetVSIInfoFrame(VendorSpecific_InfoFrame * pVSIInfoFrame)
{
	int i;
	_u8 ucData = 0;

	if (!pVSIInfoFrame) {
		return ER_FAIL;
	}
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("SetVSIInfo(): "));
	HDMITX_DEBUG_PRINTF(("\n"));
#endif
	Switch_HDMITX_Bank(1);
	// for(i=0x38;i<0x57;i++)
	// {
	//     HDMITX_WriteI2C_Byte(i,0);
	// }
	if (pVSIInfoFrame->pktbyte.VS_HB[2] > 27)	// HB[2] = Checksum ;
	{
		pVSIInfoFrame->pktbyte.VS_HB[2] = 27;
	}

	HDMITX_WriteI2C_Byte(REG_TX_PKT_HB00, pVSIInfoFrame->pktbyte.VS_HB[0]);
	ucData -= pVSIInfoFrame->pktbyte.VS_HB[0];
	HDMITX_WriteI2C_Byte(REG_TX_PKT_HB01, pVSIInfoFrame->pktbyte.VS_HB[1]);
	ucData -= pVSIInfoFrame->pktbyte.VS_HB[1];
	HDMITX_WriteI2C_Byte(REG_TX_PKT_HB02, pVSIInfoFrame->pktbyte.VS_HB[2]);
	ucData -= pVSIInfoFrame->pktbyte.VS_HB[2];

	for (i = 0; i < (int)(pVSIInfoFrame->pktbyte.VS_HB[2]); i++) {
		HDMITX_WriteI2C_Byte(REG_TX_PKT_PB01 + i,
				     pVSIInfoFrame->pktbyte.VS_DB[1 + i]);
		ucData -= pVSIInfoFrame->pktbyte.VS_DB[1 + i];
	}

	for (; i < 27; i++) {
		HDMITX_WriteI2C_Byte(REG_TX_PKT_PB01 + i, 0);
	}

	// HDMITX_WriteI2C_Byte(REG_TX_PKT_PB01,pVSIInfoFrame->pktbyte.VS_DB[1]);
	// HDMITX_WriteI2C_Byte(REG_TX_PKT_PB02,pVSIInfoFrame->pktbyte.VS_DB[2]);
	// HDMITX_WriteI2C_Byte(REG_TX_PKT_PB03,pVSIInfoFrame->pktbyte.VS_DB[3]);
	// HDMITX_WriteI2C_Byte(REG_TX_PKT_PB04,pVSIInfoFrame->pktbyte.VS_DB[4]);
	// HDMITX_WriteI2C_Byte(REG_TX_PKT_PB05,pVSIInfoFrame->pktbyte.VS_DB[5]);
	// HDMITX_WriteI2C_Byte(REG_TX_PKT_PB06,pVSIInfoFrame->pktbyte.VS_DB[6]);

	// for(i = 1; i < VENDORSPEC_INFOFRAME_LEN+1 ; i++)
	// {
	//     ucData -= pVSIInfoFrame->pktbyte.VS_DB[i] ;
	// }
	//
	// ucData -= 0x80+VENDORSPEC_INFOFRAME_VER+VENDORSPEC_INFOFRAME_TYPE+VENDORSPEC_INFOFRAME_LEN ;
	ucData &= 0xFF;
	pVSIInfoFrame->pktbyte.VS_DB[0] = ucData;

	HDMITX_WriteI2C_Byte(REG_TX_PKT_PB00, pVSIInfoFrame->pktbyte.VS_DB[0]);
	Switch_HDMITX_Bank(0);
	ENABLE_NULL_PKT();
	return ER_SUCCESS;
}

BOOL EnableVSInfoFrame(_u8 bEnable, _u8 * pVSInfoFrame)
{
	if (!bEnable) {
		DISABLE_NULL_PKT();
		return TRUE;
	}

	if (SetVSIInfoFrame((VendorSpecific_InfoFrame *) pVSInfoFrame) ==
	    ER_SUCCESS) {
		return TRUE;
	}

	return FALSE;
}

BOOL EnableAVIInfoFrame(_u8 bEnable, _u8 * pAVIInfoFrame)
{
	if (!bEnable) {
		DISABLE_AVI_INFOFRM_PKT();
		Switch_HDMITX_Bank(1);
		HDMITX_AndREG_Byte(REG_TX_AVIINFO_DB1, ~(3 << 5));
		Switch_HDMITX_Bank(0);
		return TRUE;
	}

	if (SetAVIInfoFrame((AVI_InfoFrame *) pAVIInfoFrame) == ER_SUCCESS) {
		return TRUE;
	}

	return FALSE;
}

BOOL EnableAudioInfoFrame(_u8 bEnable, _u8 * pAudioInfoFrame)
{
	if (!bEnable) {
		DISABLE_AVI_INFOFRM_PKT();
		return TRUE;
	}

	if (SetAudioInfoFrame((Audio_InfoFrame *) pAudioInfoFrame) ==
	    ER_SUCCESS) {
		return TRUE;
	}

	return FALSE;
}

void SetAVMute(_u8 bEnable)
{
	_u8 uc;
	Switch_HDMITX_Bank(0);
	uc = HDMITX_ReadI2C_Byte(REG_TX_GCP);
	uc &= ~B_TX_SETAVMUTE;
	uc |= bEnable ? B_TX_SETAVMUTE : 0;
	HDMITX_WriteI2C_Byte(REG_TX_GCP, uc);
	HDMITX_WriteI2C_Byte(REG_TX_PKT_GENERAL_CTRL,
			     B_ENABLE_PKT | B_REPEAT_PKT);
	if (bEnable) {
		HDMITX_DEBUG_PRINTF(("==========Set AV Mute============\n"));
	} else {
		HDMITX_DEBUG_PRINTF(("==========Clear AV Mute============\n"));
	}
}

BOOL GetAVMute(void)
{
	if (B_SET_AVMUTE & HDMITX_ReadI2C_Byte(REG_TX_GCP)) {
		return TRUE;
	} else {
		return FALSE;
	}
}

void SetOutputColorDepthPhase(_u8 ColorDepth, _u8 bPhase)
{
	_u8 uc;
	_u8 bColorDepth;
	bPhase = (~bPhase);
	if (ColorDepth == 30) {
		bColorDepth = B_CD_30;
	} else if (ColorDepth == 36) {
		bColorDepth = B_CD_36;
	}			/*
				   else if (ColorDepth == 24)
				   {
				   bColorDepth = B_CD_24 ;
				   } */
	else {
		bColorDepth = 0;	// not indicated
	}

	Switch_HDMITX_Bank(0);
	uc = HDMITX_ReadI2C_Byte(REG_TX_GCP);
	uc &= ~B_COLOR_DEPTH_MASK;
	uc |= bColorDepth & B_COLOR_DEPTH_MASK;
	HDMITX_WriteI2C_Byte(REG_TX_GCP, uc);
}

#if 0
void Get6613Reg(_u8 * pReg)
{
	int i;
	_u8 reg;
	Switch_HDMITX_Bank(0);
	for (i = 0; i < 0x100; i++) {
		reg = i & 0xFF;
		pReg[i] = HDMITX_ReadI2C_Byte(reg);
	}
	Switch_HDMITX_Bank(1);
	for (reg = 0x30; reg < 0xB0; i++, reg++) {
		pReg[i] = HDMITX_ReadI2C_Byte(reg);
	}
	Switch_HDMITX_Bank(0);

}
#endif
//////////////////////////////////////////////////////////////////////
// SubProcedure process                                                       //
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Function: SetupAFE
// Parameter: VIDEOPCLKLEVEL level
//            PCLK_LOW - for 13.5MHz (for mode less than 1080p)
//            PCLK MEDIUM - for 25MHz~74MHz
//            PCLK HIGH - PCLK > 80Hz (for 1080p mode or above)
// Return: N/A
// Remark: set reg62~reg65 depended on HighFreqMode
//         reg61 have to be programmed at last and after video stable input.
// Side-Effect:
//////////////////////////////////////////////////////////////////////

static void
// SetupAFE(_u8 ucFreqInMHz)
SetupAFE(VIDEOPCLKLEVEL level)
{
	_u8 uc = 0x00;
	// @emily turn off reg61 before SetupAFE parameters.
	HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL, B_AFE_DRV_RST);	/* 0x10 */
#ifdef Debug_message
	// HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL,0x3);
	HDMITX_DEBUG_PRINTF(("SetupAFE(TX_AFE_ISW = 0x%2x)\n", uc));
#endif
	//TMDS Clock < 80MHz    TMDS Clock > 80MHz
	//Reg61    0x03    0x03

	//Reg62    0x18    0x88
	//Reg63    Default    Default
	//Reg64    0x08    0x80
	//Reg65    Default    Default
	//Reg66    Default    Default
	//Reg67    Default    Default

	switch (level) {
	case PCLK_HIGH:
		HDMITX_WriteI2C_Byte(REG_TX_AFE_XP_CTRL, 0x88);	// reg62
		HDMITX_WriteI2C_Byte(REG_TX_AFE_ISW_CTRL, uc);	// reg63
		HDMITX_WriteI2C_Byte(REG_TX_AFE_IP_CTRL, 0x84);	// reg64
		break;
	default:
		HDMITX_WriteI2C_Byte(REG_TX_AFE_XP_CTRL, 0x18);	// reg62
		HDMITX_WriteI2C_Byte(REG_TX_AFE_ISW_CTRL, uc);	// reg63
		HDMITX_WriteI2C_Byte(REG_TX_AFE_IP_CTRL, 0x0C);	// reg64
		break;
	}
}

//////////////////////////////////////////////////////////////////////
// Function: FireAFE
// Parameter: N/A
// Return: N/A
// Remark: write reg61 with 0x04
//         When program reg61 with 0x04,then audio and video circuit work.
// Side-Effect: N/A
//////////////////////////////////////////////////////////////////////
static void FireAFE(void)
{
	_u8 reg;
	SoftWareVideoReset();
	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL, 0);
#ifdef Debug_message
	for (reg = 0x61; reg <= 0x67; reg++) {
		HDMITX_DEBUG_PRINTF(("Reg[%02X] = %02X\n", (_u16) reg,
				     (_u16) HDMITX_ReadI2C_Byte(reg)));
	}
#endif
	Instance[0].bVideoOut = TRUE;
	//EnableHVToolDetect(FALSE);
	//EnableHVToolDetect(TRUE);
}

//////////////////////////////////////////////////////////////////////
// Audio Output
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Function: SetAudioFormat
// Parameter:
//    NumChannel - number of channel,from 1 to 8
//    AudioEnable - Audio source and type bit field,value of bit field are
//        ENABLE_SPDIF    (1<<4)
//        ENABLE_I2S_SRC3  (1<<3)
//        ENABLE_I2S_SRC2  (1<<2)
//        ENABLE_I2S_SRC1  (1<<1)
//        ENABLE_I2S_SRC0  (1<<0)
//    SampleFreq - the audio sample frequence in Hz
//    AudSWL - Audio sample width,only support 16,18,20,or 24.
//    AudioCatCode - The audio channel catalogy code defined in IEC 60958-3
// Return: ER_SUCCESS if done,ER_FAIL for otherwise.
// Remark: program audio channel control register and audio channel registers
//         to enable audio by input.
// Side-Effect: register bank will keep in bank zero.
//////////////////////////////////////////////////////////////////////
static void AutoAdjustAudio(void)
{
	unsigned long SampleFreq, cTMDSClock;
	unsigned long N;
	_u32 aCTS = 0;
	_u8 fs, uc, LoopCnt = 10;
	if (bForceCTS) {
		Switch_HDMITX_Bank(0);
		HDMITX_WriteI2C_Byte(0xF8, 0xC3);
		HDMITX_WriteI2C_Byte(0xF8, 0xA5);
		HDMITX_AndREG_Byte(REG_TX_PKT_SINGLE_CTRL, ~B_SW_CTS);	// D[1] = 0, HW auto count CTS
		HDMITX_WriteI2C_Byte(0xF8, 0xFF);
	}
	//delay1ms(50);
	Switch_HDMITX_Bank(1);
	N = ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudN2) & 0xF) << 16;
	N |= ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudN1)) << 8;
	N |= ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudN0));

	while (LoopCnt--) {
		_u32 TempCTS = 0;
		aCTS =
		    ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudCTSCnt2)) <<
		    12;
		aCTS |=
		    ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudCTSCnt1)) << 4;
		aCTS |=
		    ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudCTSCnt0) &
		     0xf0) >> 4;
		if (aCTS == TempCTS) {
			break;
		}
		TempCTS = aCTS;
	}
	Switch_HDMITX_Bank(0);
	if (aCTS == 0) {
		HDMITX_DEBUG_PRINTF(("aCTS== 0"));
		return;
	}

	uc = HDMITX_ReadI2C_Byte(0xc1);

	cTMDSClock = GetInputPclk();	//Instance[0].TMDSClock ;
	//TMDSClock=GetInputPclk();
	HDMITX_DEBUG_PRINTF(("PCLK = %u0,000\n", (_u16) (cTMDSClock / 10000)));
	switch (uc & 0x70) {
	case 0x50:
		cTMDSClock *= 5;
		cTMDSClock /= 4;
		break;
	case 0x60:
		cTMDSClock *= 3;
		cTMDSClock /= 2;
	}
	SampleFreq = cTMDSClock / aCTS;
	SampleFreq *= N;
	SampleFreq /= 128;
	//SampleFreq=48000;

	HDMITX_DEBUG_PRINTF(("SampleFreq = %u0\n", (_u16) (SampleFreq / 10)));
	if (SampleFreq > 31000L && SampleFreq <= 38050L) {
		fs = AUDFS_32KHz;
	} else if (SampleFreq < 46550L) {
		fs = AUDFS_44p1KHz;
	}			//46050
	else if (SampleFreq < 68100L) {
		fs = AUDFS_48KHz;
	} else if (SampleFreq < 92100L) {
		fs = AUDFS_88p2KHz;
	} else if (SampleFreq < 136200L) {
		fs = AUDFS_96KHz;
	} else if (SampleFreq < 184200L) {
		fs = AUDFS_176p4KHz;
	} else if (SampleFreq < 240200L) {
		fs = AUDFS_192KHz;
	} else if (SampleFreq < 800000L) {
		fs = AUDFS_768KHz;
	} else {
		fs = AUDFS_OTHER;
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("fs = AUDFS_OTHER\n"));
#endif
	}
	if (Instance[0].bAudFs != fs) {
		Instance[0].bAudFs = fs;
		SetNCTS( /*Instance[0].TMDSClock, */ Instance[0].bAudFs);	// set N, CTS by new generated clock.
		//CurrCTS=0;
		return;
	}
	return;
}

BOOL IsAudioChang(void)
{
	//_u32 pCTS=0;
	_u8 FreDiff = 0, Refaudfreqnum;

	//Switch_HDMITX_Bank(1);
	//pCTS = ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudCTSCnt2)) << 12 ;
	//pCTS |= ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudCTSCnt1)) <<4 ;
	//pCTS |= ((unsigned long)HDMITX_ReadI2C_Byte(REGPktAudCTSCnt0)&0xf0)>>4  ;
	//Switch_HDMITX_Bank(0);
	Switch_HDMITX_Bank(0);
	Refaudfreqnum = HDMITX_ReadI2C_Byte(0x60);
	//HDMITX_DEBUG_PRINTF(("Refaudfreqnum=%X    pCTS= %u",(_u16)Refaudfreqnum,(_u16)(pCTS/10000)));
	//if((pCTS%10000)<1000)HDMITX_DEBUG_PRINTF(("0"));
	//if((pCTS%10000)<100)HDMITX_DEBUG_PRINTF(("0"));
	//if((pCTS%10000)<10)HDMITX_DEBUG_PRINTF(("0"));
	//HDMITX_DEBUG_PRINTF(("%u\n",(_u16)(pCTS%10000)));
	if ((1 << 4) & HDMITX_ReadI2C_Byte(0x5f)) {
		//printf("=======XXXXXXXXXXX=========\n");
		return FALSE;
	}

	if (LastRefaudfreqnum > Refaudfreqnum) {
		FreDiff = LastRefaudfreqnum - Refaudfreqnum;
	} else {
		FreDiff = Refaudfreqnum - LastRefaudfreqnum;
	}
	LastRefaudfreqnum = Refaudfreqnum;
	if (2 < FreDiff) {
		HDMITX_OrREG_Byte(REG_TX_PKT_SINGLE_CTRL, (1 << 5));
		HDMITX_AndREG_Byte(REG_TX_AUDIO_CTRL0, 0xF0);
		return TRUE;
	} else {
		return FALSE;
	}

}

//void RampOn()
//{
    //HDMITX_OrREG_Byte(REG_TX_SW_RST,(B_AUD_RST|B_AREF_RST));
    //HDMITX_OrREG_Byte(REG_TX_PKT_SINGLE_CTRL,(1<<2));
    //HDMITX_AndREG_Byte(REG_TX_PKT_SINGLE_CTRL,(~0x3C));
    //HDMITX_AndREG_Byte(REG_TX_SW_RST,~(B_AUD_RST|B_AREF_RST));
//}
void SetupAudioChannel(BOOL EnableAudio_b)
{
	static BOOL AudioOutStatus = FALSE;
	if (EnableAudio_b) {
		if (AudioDelayCnt == 0) {
			//if(Instance[0].bAuthenticated==FALSE)
			//{EnableHDCP(TRUE);}
#ifdef SUPPORT_AUDIO_MONITOR
			if (IsAudioChang()) {
				AutoAdjustAudio();
#else
			if (AudioOutStatus == FALSE) {
				SetNCTS(Instance[0].bAudFs);
#endif
				HDMITX_WriteI2C_Byte(REG_TX_AUD_SRCVALID_FLAT,
						     0);
				HDMITX_OrREG_Byte(REG_TX_PKT_SINGLE_CTRL,
						  (1 << 5));
				HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0,
						     Instance
						     [0].bAudioChannelEnable);
				//HDMITX_OrREG_Byte(0x59,(1<<2));  //for test
				HDMITX_AndREG_Byte(REG_TX_PKT_SINGLE_CTRL,
						   (~0x3C));
				HDMITX_AndREG_Byte(REG_TX_PKT_SINGLE_CTRL,
						   (~(1 << 5)));
				HDMITX_DEBUG_PRINTF(("Audio Out Enable\n"));
#ifndef SUPPORT_AUDIO_MONITOR
				AudioOutStatus = TRUE;
			}
#endif
		}
	} else {
		AudioOutStatus = FALSE;
		if (((Instance[0].bAudioChannelEnable & B_AUD_SPDIF) == 0) ||	// if I2S , ignore the reg5F[5] check.
		    (0x20 == (HDMITX_ReadI2C_Byte(REG_TX_CLK_STATUS2) & 0x30))) {
			AudioDelayCnt--;
			HDMITX_DEBUG_PRINTF(("AudioDelayCnt=%u\n",
					     (_u16) AudioDelayCnt));
		} else {
			AudioDelayCnt = AudioOutDelayCnt;
		}
	}
}

else
{
	// CurrCTS=0;
}
}

//////////////////////////////////////////////////////////////////////
// Function: SetNCTS
// Parameter: PCLK - video clock in Hz.
//            Fs - Encoded audio sample rate
//                          AUDFS_22p05KHz  4
//                          AUDFS_44p1KHz 0
//                          AUDFS_88p2KHz 8
//                          AUDFS_176p4KHz    12
//
//                          AUDFS_24KHz  6
//                          AUDFS_48KHz  2
//                          AUDFS_96KHz  10
//                          AUDFS_192KHz 14
//
//                          AUDFS_768KHz 9
//
//                          AUDFS_32KHz  3
//                          AUDFS_OTHER    1

// Return: ER_SUCCESS if success
// Remark: set N value,the CTS will be auto generated by HW.
// Side-Effect: register bank will reset to bank 0.
//////////////////////////////////////////////////////////////////////

static SYS_STATUS SetNCTS( /*_u32 PCLK,*/ _u8 Fs)
{
	_u32 n, MCLK, SampleFreq;
	_u8 LoopCnt = 255, CTSStableCnt = 0;
	_u32 diff;
	_u32 CTS = 0, LastCTS = 0;
	BOOL HBR_mode;
	_u8 aVIC;
	if (B_HBR & HDMITX_ReadI2C_Byte(REG_TX_AUD_HDAUDIO)) {
		HBR_mode = TRUE;
	} else {
		HBR_mode = FALSE;
	}

	Switch_HDMITX_Bank(1);
	aVIC = (HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB4) & 0x7f);
	Switch_HDMITX_Bank(0);

	if (aVIC) {
		switch (Fs) {
		case AUDFS_32KHz:
			n = 4096;
			break;
		case AUDFS_44p1KHz:
			n = 6272;
			break;
		case AUDFS_48KHz:
			n = 6144;
			break;
		case AUDFS_88p2KHz:
			n = 12544;
			break;
		case AUDFS_96KHz:
			n = 12288;
			break;
		case AUDFS_176p4KHz:
			n = 25088;
			break;
		case AUDFS_192KHz:
			n = 24576;
			break;
		default:
			n = 6144;
		}
	} else {
		switch (Fs) {
		case AUDFS_32KHz:
			SampleFreq = 32000L;
			break;
		case AUDFS_44p1KHz:
			SampleFreq = 44100L;
			break;
		case AUDFS_48KHz:
			SampleFreq = 48000L;
			break;
		case AUDFS_88p2KHz:
			SampleFreq = 88200L;
			break;
		case AUDFS_96KHz:
			SampleFreq = 96000L;
			break;
		case AUDFS_176p4KHz:
			SampleFreq = 176000L;
			break;
		case AUDFS_192KHz:
			SampleFreq = 192000L;
			break;
		default:
			SampleFreq = 768000L;
		}
		MCLK = SampleFreq * 256;	// MCLK = fs * 256 ;
		n = MCLK / 2000;
	}
	// tr_printf((" n = %ld\n",n)) ;
	Switch_HDMITX_Bank(1);
	HDMITX_WriteI2C_Byte(REGPktAudN0, (_u8) ((n) & 0xFF));
	HDMITX_WriteI2C_Byte(REGPktAudN1, (_u8) ((n >> 8) & 0xFF));
	HDMITX_WriteI2C_Byte(REGPktAudN2, (_u8) ((n >> 16) & 0xF));
	//Switch_HDMITX_Bank(0) ;
	//delay1ms(200);
	//Switch_HDMITX_Bank(1) ;
	if (bForceCTS) {
		_u32 SumCTS = 0;
		while (LoopCnt--) {
			delay1ms(30);
			CTS = ((unsigned long)
			       HDMITX_ReadI2C_Byte(REGPktAudCTSCnt2)) << 12;
			CTS |= ((unsigned long)
				HDMITX_ReadI2C_Byte(REGPktAudCTSCnt1)) << 4;
			CTS |= ((unsigned long)
				HDMITX_ReadI2C_Byte(REGPktAudCTSCnt0) & 0xf0) >>
			    4;
			if (CTS == 0) {
				continue;
			} else {
				if (LastCTS > CTS) {
					diff = LastCTS - CTS;
				} else {
					diff = CTS - LastCTS;
				}
				//HDMITX_DEBUG_PRINTF(("LastCTS= %u%u",(_u16)(LastCTS/10000),(_u16)(LastCTS%10000)));
				//HDMITX_DEBUG_PRINTF(("       CTS= %u%u\n",(_u16)(CTS/10000),(_u16)(CTS%10000)));
				LastCTS = CTS;
				if (5 > diff) {
					CTSStableCnt++;
					SumCTS += CTS;
				} else {
					CTSStableCnt = 0;
					SumCTS = 0;
					continue;
				}
				if (CTSStableCnt >= 32) {
					LastCTS = (SumCTS >> 5);
					break;
				}
			}
		}
	}
	HDMITX_WriteI2C_Byte(REGPktAudCTS0, (_u8) ((LastCTS) & 0xFF));
	HDMITX_WriteI2C_Byte(REGPktAudCTS1, (_u8) ((LastCTS >> 8) & 0xFF));
	HDMITX_WriteI2C_Byte(REGPktAudCTS2, (_u8) ((LastCTS >> 16) & 0xF));
	Switch_HDMITX_Bank(0);
#ifdef Force_CTS
	bForceCTS = TRUE;
#endif
	HDMITX_WriteI2C_Byte(0xF8, 0xC3);
	HDMITX_WriteI2C_Byte(0xF8, 0xA5);
	if (bForceCTS) {
		HDMITX_OrREG_Byte(REG_TX_PKT_SINGLE_CTRL, B_SW_CTS);	// D[1] = 0, HW auto count CTS
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("bForceCTS = TRUE  [LoopCnt=%u]\n Write CTS= %u", (_u16) LoopCnt, (_u16) (LastCTS / 10000)));
		HDMITX_DEBUG_PRINTF(("%u\n", (_u16) (LastCTS % 10000)));
#endif
	} else {
		HDMITX_AndREG_Byte(REG_TX_PKT_SINGLE_CTRL, ~B_SW_CTS);	// D[1] = 0, HW auto count CTS
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("bForceCTS = FALSE\n"));
#endif
	}
	HDMITX_WriteI2C_Byte(0xF8, 0xFF);

	if (FALSE == HBR_mode)	//LPCM
	{
		_u8 uData;
		Switch_HDMITX_Bank(1);
		HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_CA_FS, 0x00 | Fs);
		Fs = ~Fs;	// OFS is the one's complement of FS
		uData = (0x0f & HDMITX_ReadI2C_Byte(REG_TX_AUDCHST_OFS_WL));
		HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_OFS_WL, (Fs << 4) | uData);
		Switch_HDMITX_Bank(0);
	}
#ifdef Debug_message
	//   HDMITX_DEBUG_PRINTF(("CurrCTS = %u",(_u16)(LastCTS/10000)));
	//   HDMITX_DEBUG_PRINTF(("%u\n",(_u16)(CurrCTS%10000)));
	HDMITX_DEBUG_PRINTF(("N = %d \n", (_u16) n));
	//HDMITX_DEBUG_PRINTF(("SampleFreq = %d 000\n",(_u16)(SampleFreq/1000)));
	HDMITX_DEBUG_PRINTF(("**************************************\n"));
#endif

	return ER_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// DDC Function.
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Function: ClearDDCFIFO
// Parameter: N/A
// Return: N/A
// Remark: clear the DDC FIFO.
// Side-Effect: DDC master will set to be HOST.
//////////////////////////////////////////////////////////////////////

static void ClearDDCFIFO(void)
{
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
			     B_MASTERDDC | B_MASTERHOST);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_FIFO_CLR);
}

static void GenerateDDCSCLK(void)
{
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
			     B_MASTERDDC | B_MASTERHOST);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_GEN_SCLCLK);
}

//////////////////////////////////////////////////////////////////////
// Function: AbortDDC
// Parameter: N/A
// Return: N/A
// Remark: Force abort DDC and reset DDC bus.
// Side-Effect:
//////////////////////////////////////////////////////////////////////

static void AbortDDC(void)
{
	_u8 CPDesire, SWReset, DDCMaster;
	_u8 uc, timeout, i;
	// save the SW reset,DDC master,and CP Desire setting.
	SWReset = HDMITX_ReadI2C_Byte(REG_TX_SW_RST);
	CPDesire = HDMITX_ReadI2C_Byte(REG_TX_HDCP_DESIRE);
	DDCMaster = HDMITX_ReadI2C_Byte(REG_TX_DDC_MASTER_CTRL);

	HDMITX_WriteI2C_Byte(REG_TX_HDCP_DESIRE, 0x08 | (CPDesire & (~B_CPDESIRE)));	// @emily change order
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST, SWReset | B_HDCP_RST);	// @emily change order
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
			     B_MASTERDDC | B_MASTERHOST);

	// 2009/01/15 modified by Mingchih.Lung@ite.com.tw
	// do abort DDC twice.
	for (i = 0; i < 2; i++) {
		HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_DDC_ABORT);

		for (timeout = 0; timeout < 200; timeout++) {
			uc = HDMITX_ReadI2C_Byte(REG_TX_DDC_STATUS);
			if (uc & B_DDC_DONE) {
				break;	// success
			}

			if (uc & (B_DDC_NOACK | B_DDC_WAITBUS | B_DDC_ARBILOSE)) {
#ifdef Debug_message
				HDMITX_DEBUG_PRINTF(("AbortDDC Fail by reg16=%02X\n", uc));
#endif
				break;
			}
			delay1ms(1);	// delay 1 ms to stable.
		}
	}
	//~Mingchih.Lung@ite.com.tw

	// 2009/01/15 modified by Mingchih.Lung@ite.com.tw
	//// restore the SW reset,DDC master,and CP Desire setting.
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST, SWReset);
	HDMITX_WriteI2C_Byte(REG_TX_HDCP_DESIRE, 0x08 | CPDesire);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL, DDCMaster);
	//~Mingchih.Lung@ite.com.tw

}

//////////////////////////////////////////////////////////////////////
// Packet and InfoFrame
//////////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////////
// // Function: SetAVMute()
// // Parameter: N/A
// // Return: N/A
// // Remark: set AVMute as TRUE and enable GCP sending.
// // Side-Effect: N/A
// ////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// void
// SetAVMute()
// {
//     Switch_HDMITX_Bank(0);
//     HDMITX_WriteI2C_Byte(REG_TX_GCP,B_SET_AVMUTE);
//     HDMITX_WriteI2C_Byte(REG_TX_PKT_GENERAL_CTRL,B_ENABLE_PKT|B_REPEAT_PKT);
// }

// ////////////////////////////////////////////////////////////////////////////////
// // Function: SetAVMute(FALSE)
// // Parameter: N/A
// // Return: N/A
// // Remark: clear AVMute as TRUE and enable GCP sending.
// // Side-Effect: N/A
// ////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// void
// SetAVMute(FALSE)
// {
//     Switch_HDMITX_Bank(0);
//     HDMITX_WriteI2C_Byte(REG_TX_GCP,B_CLR_AVMUTE);
//     HDMITX_WriteI2C_Byte(REG_TX_PKT_GENERAL_CTRL,B_ENABLE_PKT|B_REPEAT_PKT);
// }

//////////////////////////////////////////////////////////////////////
// Function: ReadEDID
// Parameter: pData - the pointer of buffer to receive EDID ucdata.
//            bSegment - the segment of EDID readback.
//            offset - the offset of EDID ucdata in the segment. in _u8.
//            count - the read back bytes count,cannot exceed 32
// Return: ER_SUCCESS if successfully getting EDID. ER_FAIL otherwise.
// Remark: function for read EDID ucdata from reciever.
// Side-Effect: DDC master will set to be HOST. DDC FIFO will be used and dirty.
//////////////////////////////////////////////////////////////////////

static SYS_STATUS ReadEDID(_u8 * pData, _u8 bSegment, _u8 offset, SHORT Count)
{
	SHORT RemainedCount, ReqCount;
	_u8 bCurrOffset;
	SHORT TimeOut;
	_u8 *pBuff = pData;
	_u8 ucdata;

#ifdef Debug_message
	// HDMITX_DEBUG_PRINTF(("ReadEDID(%08lX,%d,%d,%d)\n",(_u32)pData,bSegment,offset,Count));
#endif
	if (!pData) {
#ifdef Debug_message
//        HDMITX_DEBUG_PRINTF(("ReadEDID(): Invallid pData pointer %08lX\n",(_u32)pData));
#endif
		return ER_FAIL;
	}

	if (HDMITX_ReadI2C_Byte(REG_TX_INT_STAT1) & B_INT_DDC_BUS_HANG) {
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("Called AboutDDC()\n"));
#endif
		AbortDDC();

	}

	ClearDDCFIFO();

	RemainedCount = Count;
	bCurrOffset = offset;

	Switch_HDMITX_Bank(0);

	while (RemainedCount > 0) {

		ReqCount =
		    (RemainedCount >
		     DDC_FIFO_MAXREQ) ? DDC_FIFO_MAXREQ : RemainedCount;
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("ReadEDID(): ReqCount = %d,bCurrOffset = %d\n", (_u16) ReqCount, (_u16) bCurrOffset));
#endif
		HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
				     B_MASTERDDC | B_MASTERHOST);
		HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_FIFO_CLR);

		for (TimeOut = 0; TimeOut < 200; TimeOut++) {
			ucdata = HDMITX_ReadI2C_Byte(REG_TX_DDC_STATUS);

			if (ucdata & B_DDC_DONE) {
				break;
			}

			if ((ucdata & B_DDC_ERROR)
			    || (HDMITX_ReadI2C_Byte(REG_TX_INT_STAT1) &
				B_INT_DDC_BUS_HANG)) {
#ifdef Debug_message
				HDMITX_DEBUG_PRINTF(("Called AboutDDC()\n"));
#endif
				AbortDDC();
				return ER_FAIL;
			}
		}

		HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
				     B_MASTERDDC | B_MASTERHOST);
		HDMITX_WriteI2C_Byte(REG_TX_DDC_HEADER, DDC_EDID_ADDRESS);	// for EDID ucdata get
		HDMITX_WriteI2C_Byte(REG_TX_DDC_REQOFF, bCurrOffset);
		HDMITX_WriteI2C_Byte(REG_TX_DDC_REQCOUNT, (_u8) ReqCount);
		HDMITX_WriteI2C_Byte(REG_TX_DDC_EDIDSEG, bSegment);
		HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_EDID_READ);

		bCurrOffset += ReqCount;
		RemainedCount -= ReqCount;

		for (TimeOut = 250; TimeOut > 0; TimeOut--) {
			delay1ms(1);
			ucdata = HDMITX_ReadI2C_Byte(REG_TX_DDC_STATUS);
			if (ucdata & B_DDC_DONE) {
				break;
			}

			if (ucdata & B_DDC_ERROR) {
#ifdef Debug_message
//                HDMITX_DEBUG_PRINTF(("ReadEDID(): DDC_STATUS = %02X,fail.\n",ucdata));
#endif
				return ER_FAIL;
			}
		}

		if (TimeOut == 0) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("ReadEDID(): DDC TimeOut. %02X\n",
					     (_u16) ucdata));
#endif
			return ER_FAIL;
		}

		do {
			*(pBuff++) = HDMITX_ReadI2C_Byte(REG_TX_DDC_READFIFO);
			ReqCount--;
		} while (ReqCount > 0);

	}

	return ER_SUCCESS;
}

#ifdef SUPPORT_HDCP
//////////////////////////////////////////////////////////////////////
// Authentication
//////////////////////////////////////////////////////////////////////
static void HDCP_ClearAuthInterrupt(void)
{
	_u8 uc;
	uc = HDMITX_ReadI2C_Byte(REG_TX_INT_MASK2) &
	    (~(B_KSVLISTCHK_MASK | B_T_AUTH_DONE_MASK | B_AUTH_FAIL_MASK));
	HDMITX_WriteI2C_Byte(REG_TX_INT_CLR0,
			     B_CLR_AUTH_FAIL | B_CLR_AUTH_DONE |
			     B_CLR_KSVLISTCHK);
	HDMITX_WriteI2C_Byte(REG_TX_INT_CLR1, 0);
	HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, B_INTACTDONE);
}

static void HDCP_ResetAuth(void)
{
	HDMITX_WriteI2C_Byte(REG_TX_LISTCTRL, 0);
	HDMITX_WriteI2C_Byte(REG_TX_HDCP_DESIRE, 0x08);
	HDMITX_OrREG_Byte(REG_TX_SW_RST, B_HDCP_RST);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
			     B_MASTERDDC | B_MASTERHOST);
	HDCP_ClearAuthInterrupt();
	AbortDDC();
}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_EnableEncryption
// Parameter: N/A
// Return: ER_SUCCESS if done.
// Remark: Set regC1 as zero to enable continue authentication.
// Side-Effect: register bank will reset to zero.
//////////////////////////////////////////////////////////////////////

static SYS_STATUS HDCP_EnableEncryption(void)
{
	Switch_HDMITX_Bank(0);
	return HDMITX_WriteI2C_Byte(REG_TX_ENCRYPTION, B_ENABLE_ENCRYPTION);
}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_Auth_Fire()
// Parameter: N/A
// Return: N/A
// Remark: write anything to reg21 to enable HDCP authentication by HW
// Side-Effect: N/A
//////////////////////////////////////////////////////////////////////

static void HDCP_Auth_Fire(void)
{
#ifdef Debug_message
	// HDMITX_DEBUG_PRINTF(("HDCP_Auth_Fire():\n"));
#endif
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL, B_MASTERDDC | B_MASTERHDCP);	// MASTERHDCP,no need command but fire.
	HDMITX_WriteI2C_Byte(REG_TX_AUTHFIRE, 1);
}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_StartAnCipher
// Parameter: N/A
// Return: N/A
// Remark: Start the Cipher to free run for random number. When stop,An is
//         ready in Reg30.
// Side-Effect: N/A
//////////////////////////////////////////////////////////////////////

static void HDCP_StartAnCipher(void)
{
	HDMITX_WriteI2C_Byte(REG_TX_AN_GENERATE, B_START_CIPHER_GEN);
	delay1ms(1);		// delay 1 ms
}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_StopAnCipher
// Parameter: N/A
// Return: N/A
// Remark: Stop the Cipher,and An is ready in Reg30.
// Side-Effect: N/A
//////////////////////////////////////////////////////////////////////

static void HDCP_StopAnCipher(void)
{
	HDMITX_WriteI2C_Byte(REG_TX_AN_GENERATE, B_STOP_CIPHER_GEN);
}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_GenerateAn
// Parameter: N/A
// Return: N/A
// Remark: start An ciper random run at first,then stop it. Software can get
//         an in reg30~reg38,the write to reg28~2F
// Side-Effect:
//////////////////////////////////////////////////////////////////////

static void HDCP_GenerateAn(void)
{
	_u8 Data[8];
	_u8 i = 0;
#if 1
	HDCP_StartAnCipher();
	// HDMITX_WriteI2C_Byte(REG_TX_AN_GENERATE,B_START_CIPHER_GEN);
	// delay1ms(1); // delay 1 ms
	// HDMITX_WriteI2C_Byte(REG_TX_AN_GENERATE,B_STOP_CIPHER_GEN);

	HDCP_StopAnCipher();

	Switch_HDMITX_Bank(0);
	// new An is ready in reg30
	//HDMITX_ReadI2C_ByteN(REG_TX_AN_GEN,Data,8);
	for (i = 0; i < 8; i++)
		Data[i] = HDMITX_ReadI2C_Byte(REG_TX_AN_GEN);
#else
	Data[0] = 0;
	Data[1] = 0;
	Data[2] = 0;
	Data[3] = 0;
	Data[4] = 0;
	Data[5] = 0;
	Data[6] = 0;
	Data[7] = 0;
#endif
	for (i = 0; i < 8; i++)
		HDMITX_WriteI2C_Byte(REG_TX_AN, Data[i]);

}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_GetBCaps
// Parameter: pBCaps - pointer of _u8 to get BCaps.
//            pBStatus - pointer of two bytes to get BStatus
// Return: ER_SUCCESS if successfully got BCaps and BStatus.
// Remark: get B status and capability from HDCP reciever via DDC bus.
// Side-Effect:
//////////////////////////////////////////////////////////////////////

static SYS_STATUS HDCP_GetBCaps(PUCHAR pBCaps, PUSHORT pBStatus)
{
	_u8 ucdata;
	_u8 TimeOut;

	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
			     B_MASTERDDC | B_MASTERHOST);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_HEADER, DDC_HDCP_ADDRESS);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_REQOFF, 0x40);	// BCaps offset
	HDMITX_WriteI2C_Byte(REG_TX_DDC_REQCOUNT, 3);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_DDC_SEQ_BURSTREAD);

	for (TimeOut = 200; TimeOut > 0; TimeOut--) {
		delay1ms(1);

		ucdata = HDMITX_ReadI2C_Byte(REG_TX_DDC_STATUS);
		if (ucdata & B_DDC_DONE) {
#ifdef Debug_message
			//HDMITX_DEBUG_PRINTF(("HDCP_GetBCaps(): DDC Done.\n"));
#endif
			break;
		}

		if (ucdata & B_DDC_ERROR) {
#ifdef Debug_message
//            HDMITX_DEBUG_PRINTF(("HDCP_GetBCaps(): DDC fail by reg16=%02X.\n",ucdata));
#endif
			return ER_FAIL;
		}
	}

	if (TimeOut == 0) {
		return ER_FAIL;
	}
	//HDMITX_ReadI2C_ByteN(REG_TX_BSTAT,(PUCHAR)pBStatus,2);
	*pBStatus &= 0x0000;
	*pBStatus = (HDMITX_ReadI2C_Byte(REG_TX_BSTAT) & 0xff);
	*pBStatus += ((HDMITX_ReadI2C_Byte(REG_TX_BSTAT1) & 0xff) << 8);
	*pBCaps = HDMITX_ReadI2C_Byte(REG_TX_BCAP);
	return ER_SUCCESS;

}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_GetBKSV
// Parameter: pBKSV - pointer of 5 bytes buffer for getting BKSV
// Return: ER_SUCCESS if successfuly got BKSV from Rx.
// Remark: Get BKSV from HDCP reciever.
// Side-Effect: N/A
//////////////////////////////////////////////////////////////////////

static SYS_STATUS HDCP_GetBKSV(_u8 * pBKSV)
{
	_u8 ucdata, i = 0;
	_u8 TimeOut;

	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
			     B_MASTERDDC | B_MASTERHOST);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_HEADER, DDC_HDCP_ADDRESS);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_REQOFF, 0x00);	// BKSV offset
	HDMITX_WriteI2C_Byte(REG_TX_DDC_REQCOUNT, 5);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_DDC_SEQ_BURSTREAD);

	for (TimeOut = 200; TimeOut > 0; TimeOut--) {
		delay1ms(1);

		ucdata = HDMITX_ReadI2C_Byte(REG_TX_DDC_STATUS);
		if (ucdata & B_DDC_DONE) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_GetBCaps(): DDC Done.\n"));
#endif
			break;
		}

		if (ucdata & B_DDC_ERROR) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_GetBCaps(): DDC No ack or arbilose,%x,maybe cable did not connected. Fail.\n", (_u16) ucdata));
#endif
			return ER_FAIL;
		}
	}

	if (TimeOut == 0) {
		return ER_FAIL;
	}
	//HDMITX_ReadI2C_ByteN(REG_TX_BKSV,(PUCHAR)pBKSV,5);
	for (i = 0; i < 5; i++) {
		*pBKSV = HDMITX_ReadI2C_Byte(REG_TX_BKSV + i);
		pBKSV++;
	}
	return ER_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// Function:HDCP_Authenticate
// Parameter: N/A
// Return: ER_SUCCESS if Authenticated without error.
// Remark: do Authentication with Rx
// Side-Effect:
//  1. Instance[0].bAuthenticated global variable will be TRUE when authenticated.
//  2. Auth_done interrupt and AUTH_FAIL interrupt will be enabled.
//////////////////////////////////////////////////////////////////////
static _u8 countbit(_u8 b)
{
	_u8 i, count;
	for (i = 0, count = 0; i < 8; i++) {
		if (b & (1 << i)) {
			count++;
		}
	}
	return count;
}

static void HDCP_Reset(void)
{
	_u8 uc;
	uc = HDMITX_ReadI2C_Byte(REG_TX_SW_RST) | B_HDCP_RST;
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST, uc);
	HDMITX_WriteI2C_Byte(REG_TX_HDCP_DESIRE, 0x08);
	HDMITX_WriteI2C_Byte(REG_TX_LISTCTRL, 0);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL, B_MASTERHOST);
	ClearDDCFIFO();
	AbortDDC();
}

static SYS_STATUS HDCP_Authenticate(void)
{
	_u8 ucdata;
	_u8 BCaps;
	_u16 BStatus;
	_u16 TimeOut;

	//_u8 revoked = FALSE ;
	_u8 BKSV[5];

	Instance[0].bAuthenticated = FALSE;

	// Authenticate should be called after AFE setup up.
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("HDCP_Authenticate():\n"));
#endif
	HDCP_Reset();
	// ClearDDCFIFO();
	// AbortDDC();

	Switch_HDMITX_Bank(0);

	for (TimeOut = 0; TimeOut < 20; TimeOut++) {
		delay1ms(15);

		if (HDCP_GetBCaps(&BCaps, &BStatus) != ER_SUCCESS) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_GetBCaps fail.\n"));
#endif
			return ER_FAIL;
		}

		if (B_TX_HDMI_MODE ==
		    (HDMITX_ReadI2C_Byte(REG_TX_HDMI_MODE) & B_TX_HDMI_MODE)) {
			if ((BStatus & B_CAP_HDMI_MODE) == B_CAP_HDMI_MODE) {
				break;
			}
		} else {
			if ((BStatus & B_CAP_HDMI_MODE) != B_CAP_HDMI_MODE) {
				break;
			}
		}
	}
#ifdef Debug_message
//    HDMITX_DEBUG_PRINTF(("BCaps = %02X BStatus = %04X\n",BCaps,BStatus));
#endif
	/*
	   if((BStatus & M_DOWNSTREAM_COUNT)> 6)
	   {
	   #ifdef Debug_message
	   HDMITX_DEBUG_PRINTF(("Down Stream Count %d is over maximum supported number 6,fail.\n",(BStatus & M_DOWNSTREAM_COUNT)));
	   #endif
	   return ER_FAIL ;
	   }
	 */

	HDCP_GetBKSV(BKSV);
#ifdef Debug_message
//    HDMITX_DEBUG_PRINTF(("BKSV %02X %02X %02X %02X %02X\n",BKSV[0],BKSV[1],BKSV[2],BKSV[3],BKSV[4]));
#endif
	for (TimeOut = 0, ucdata = 0; TimeOut < 5; TimeOut++) {
		ucdata += countbit(BKSV[TimeOut]);
	}
	if (ucdata != 20)
		return ER_FAIL;

	Switch_HDMITX_Bank(0);	// switch bank action should start on direct register writting of each function.

	// 2006/08/11 added by jjtseng
	// enable HDCP on CPDired enabled.
	HDMITX_AndREG_Byte(REG_TX_SW_RST, ~(B_HDCP_RST));
	//~jjtseng 2006/08/11

//    if(BCaps & B_CAP_HDCP_1p1)
//    {
#ifdef Debug_message
//        HDMITX_DEBUG_PRINTF(("RX support HDCP 1.1\n"));
#endif
//        HDMITX_WriteI2C_Byte(REG_TX_HDCP_DESIRE,B_ENABLE_HDPC11|B_CPDESIRE);
//    }
//    else
//    {
#ifdef Debug_message
//        HDMITX_DEBUG_PRINTF(("RX not support HDCP 1.1\n"));
#endif
	HDMITX_WriteI2C_Byte(REG_TX_HDCP_DESIRE, 0x08 | B_CPDESIRE);
//    }

	// HDMITX_WriteI2C_Byte(REG_TX_INT_CLR0,B_CLR_AUTH_DONE|B_CLR_AUTH_FAIL|B_CLR_KSVLISTCHK);
	// HDMITX_WriteI2C_Byte(REG_TX_INT_CLR1,0); // don't clear other settings.
	// ucdata = HDMITX_ReadI2C_Byte(REG_TX_SYS_STATUS);
	// ucdata = (ucdata & M_CTSINTSTEP) | B_INTACTDONE ;
	// HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS,ucdata); // clear action.

	// HDMITX_AndREG_Byte(REG_TX_INT_MASK2,~(B_AUTH_FAIL_MASK|B_T_AUTH_DONE_MASK));    // enable GetBCaps Interrupt
	HDCP_ClearAuthInterrupt();
#ifdef Debug_message
//    HDMITX_DEBUG_PRINTF(("int2 = %02X DDC_Status = %02X\n",HDMITX_ReadI2C_Byte(REG_TX_INT_STAT2),HDMITX_ReadI2C_Byte(REG_TX_DDC_STATUS)));
#endif

	HDCP_GenerateAn();
	HDMITX_WriteI2C_Byte(REG_TX_LISTCTRL, 0);
	Instance[0].bAuthenticated = FALSE;
	//DumpCatHDMITXReg();
	if ((BCaps & B_CAP_HDMI_REPEATER) == 0) {
		HDCP_Auth_Fire();
		// wait for status ;
		for (TimeOut = 250; TimeOut > 0; TimeOut--) {
			delay1ms(5);	// delay 1ms
			ucdata = HDMITX_ReadI2C_Byte(REG_TX_AUTH_STAT);
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("reg46 = %02x reg16 = %02x reg1A = %02x\n", (_u16) ucdata, (_u16) HDMITX_ReadI2C_Byte(0x16), (_u16) HDMITX_ReadI2C_Byte(0x1A)));
#endif
			if (ucdata & B_T_AUTH_DONE) {
				Instance[0].bAuthenticated = TRUE;
				break;
			}

			ucdata = HDMITX_ReadI2C_Byte(REG_TX_INT_STAT2);
			if (ucdata & B_INT_AUTH_FAIL) {
				/*
				   HDMITX_WriteI2C_Byte(REG_TX_INT_CLR0,B_CLR_AUTH_FAIL);
				   HDMITX_WriteI2C_Byte(REG_TX_INT_CLR1,0);
				   HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS,B_INTACTDONE);
				   HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS,0);
				 */
#ifdef Debug_message
				HDMITX_DEBUG_PRINTF(("HDCP_Authenticate(): Authenticate fail\n"));
#endif
				Instance[0].bAuthenticated = FALSE;
				return ER_FAIL;
			}
		}

		if (TimeOut == 0) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_Authenticate(): Time out. return fail\n"));
#endif
			Instance[0].bAuthenticated = FALSE;
			return ER_FAIL;
		}
		return ER_SUCCESS;
	}

	return HDCP_Authenticate_Repeater();
}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_VerifyIntegration
// Parameter: N/A
// Return: ER_SUCCESS if success,if AUTH_FAIL interrupt status,return fail.
// Remark: no used now.
// Side-Effect:
//////////////////////////////////////////////////////////////////////

static SYS_STATUS HDCP_VerifyIntegration(void)
{
	// if any interrupt issued a Auth fail,returned the Verify Integration fail.

	if (HDMITX_ReadI2C_Byte(REG_TX_INT_STAT1) & B_INT_AUTH_FAIL) {
		HDCP_ClearAuthInterrupt();
		Instance[0].bAuthenticated = FALSE;
		return ER_FAIL;
	}

	if (Instance[0].bAuthenticated == TRUE) {
		return ER_SUCCESS;
	}

	return ER_FAIL;
}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_Authenticate_Repeater
// Parameter: BCaps and BStatus
// Return: ER_SUCCESS if success,if AUTH_FAIL interrupt status,return fail.
// Remark:
// Side-Effect: as Authentication
//////////////////////////////////////////////////////////////////////
//static _IDATA _u8 KSVList[32] ;
static _u8 Vr[20];
static _u8 M0[8];
//static _XDATA _u8 SHABuff[64] ;
extern _u8 cBuf[128];

static _u8 V[20];
#define SHA_BUFF_COUNT 17
static _u32 w[SHA_BUFF_COUNT];
static _u32 sha[5];

#define rol(x,y) (((x) << (y)) | (((_u32)x) >> (32-y)))

static void HDCP_CancelRepeaterAuthenticate(void)
{
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("HDCP_CancelRepeaterAuthenticate"));
#endif
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL,
			     B_MASTERDDC | B_MASTERHOST);
	AbortDDC();
	HDMITX_WriteI2C_Byte(REG_TX_LISTCTRL, B_LISTFAIL | B_LISTDONE);
	HDCP_ClearAuthInterrupt();
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST,
			     B_HDCP_RST | HDMITX_ReadI2C_Byte(REG_TX_SW_RST));
}

static void HDCP_ResumeRepeaterAuthenticate(void)
{
	HDMITX_WriteI2C_Byte(REG_TX_LISTCTRL, B_LISTDONE);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL, B_MASTERHDCP);
}

static SYS_STATUS HDCP_GetKSVList(_u8 * pKSVList, _u8 cDownStream)
{
	_u8 TimeOut = 100;
	_u8 ucdata;

	if (cDownStream == 0) {
		return ER_SUCCESS;
	}

	if ( /* cDownStream == 0 || */ pKSVList == NULL) {
		return ER_FAIL;
	}

	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL, B_MASTERHOST);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_HEADER, 0x74);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_REQOFF, 0x43);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_REQCOUNT, cDownStream * 5);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_DDC_SEQ_BURSTREAD);

	for (TimeOut = 200; TimeOut > 0; TimeOut--) {

		ucdata = HDMITX_ReadI2C_Byte(REG_TX_DDC_STATUS);
		if (ucdata & B_DDC_DONE) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_GetKSVList(): DDC Done.\n"));
#endif
			break;
		}

		if (ucdata & B_DDC_ERROR) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_GetKSVList(): DDC Fail by REG_TX_DDC_STATUS = %x.\n", (_u16) ucdata));
#endif
			return ER_FAIL;
		}
		delay1ms(5);
	}

	if (TimeOut == 0) {
		return ER_FAIL;
	}
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("HDCP_GetKSVList(): KSV"));
#endif
	for (TimeOut = 0; TimeOut < cDownStream * 5; TimeOut++) {
		pKSVList[TimeOut] = HDMITX_ReadI2C_Byte(REG_TX_DDC_READFIFO);
#ifdef Debug_message
//        HDMITX_DEBUG_PRINTF((" %02X",pKSVList[TimeOut]));
#endif
	}
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("\n"));
#endif
	return ER_SUCCESS;
}

static SYS_STATUS HDCP_GetVr(_u8 * pVr)
{
	_u8 TimeOut;
	_u8 ucdata;

	if (pVr == NULL) {
		return ER_FAIL;
	}

	HDMITX_WriteI2C_Byte(REG_TX_DDC_MASTER_CTRL, B_MASTERHOST);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_HEADER, 0x74);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_REQOFF, 0x20);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_REQCOUNT, 20);
	HDMITX_WriteI2C_Byte(REG_TX_DDC_CMD, CMD_DDC_SEQ_BURSTREAD);

	for (TimeOut = 200; TimeOut > 0; TimeOut--) {
		ucdata = HDMITX_ReadI2C_Byte(REG_TX_DDC_STATUS);
		if (ucdata & B_DDC_DONE) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_GetVr(): DDC Done.\n"));
#endif
			break;
		}

		if (ucdata & B_DDC_ERROR) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_GetVr(): DDC fail by REG_TX_DDC_STATUS = %x.\n", (_u16) ucdata));
#endif
			return ER_FAIL;
		}
		delay1ms(5);
	}

	if (TimeOut == 0) {
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("HDCP_GetVr(): DDC fail by timeout.%x\n",
				     (_u16) ucdata));
#endif
		return ER_FAIL;
	}

	Switch_HDMITX_Bank(0);

	for (TimeOut = 0; TimeOut < 5; TimeOut++) {
		HDMITX_WriteI2C_Byte(REG_TX_SHA_SEL, TimeOut);
		pVr[TimeOut * 4 + 3] =
		    (_u32) HDMITX_ReadI2C_Byte(REG_TX_SHA_RD_BYTE1);
		pVr[TimeOut * 4 + 2] =
		    (_u32) HDMITX_ReadI2C_Byte(REG_TX_SHA_RD_BYTE2);
		pVr[TimeOut * 4 + 1] =
		    (_u32) HDMITX_ReadI2C_Byte(REG_TX_SHA_RD_BYTE3);
		pVr[TimeOut * 4] =
		    (_u32) HDMITX_ReadI2C_Byte(REG_TX_SHA_RD_BYTE4);
#ifdef Debug_message
		//HDMITX_DEBUG_PRINTF(("Read V' = %02X %02X %02X %02X\n",(_u16)pVr[TimeOut*4],(_u16)pVr[TimeOut*4+1],(_u16)pVr[TimeOut*4+2],(_u16)pVr[TimeOut*4+3]));
#endif
	}

	return ER_SUCCESS;
}

static SYS_STATUS HDCP_GetM0(_u8 * pM0)
{
	int i;

	if (!pM0) {
		return ER_FAIL;
	}

	HDMITX_WriteI2C_Byte(REG_TX_SHA_SEL, 5);	// read m0[31:0] from reg51~reg54
	pM0[0] = HDMITX_ReadI2C_Byte(REG_TX_SHA_RD_BYTE1);
	pM0[1] = HDMITX_ReadI2C_Byte(REG_TX_SHA_RD_BYTE2);
	pM0[2] = HDMITX_ReadI2C_Byte(REG_TX_SHA_RD_BYTE3);
	pM0[3] = HDMITX_ReadI2C_Byte(REG_TX_SHA_RD_BYTE4);
	HDMITX_WriteI2C_Byte(REG_TX_SHA_SEL, 0);	// read m0[39:32] from reg55
	pM0[4] = HDMITX_ReadI2C_Byte(REG_TX_AKSV_RD_BYTE5);
	HDMITX_WriteI2C_Byte(REG_TX_SHA_SEL, 1);	// read m0[47:40] from reg55
	pM0[5] = HDMITX_ReadI2C_Byte(REG_TX_AKSV_RD_BYTE5);
	HDMITX_WriteI2C_Byte(REG_TX_SHA_SEL, 2);	// read m0[55:48] from reg55
	pM0[6] = HDMITX_ReadI2C_Byte(REG_TX_AKSV_RD_BYTE5);
	HDMITX_WriteI2C_Byte(REG_TX_SHA_SEL, 3);	// read m0[63:56] from reg55
	pM0[7] = HDMITX_ReadI2C_Byte(REG_TX_AKSV_RD_BYTE5);
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("M[] ="));
#endif
	for (i = 0; i < 8; i++) {
//        HDMITX_DEBUG_PRINTF(("0x%02x,",pM0[i]));
	}
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("\n"));
#endif
	return ER_SUCCESS;
}

void SHATransform(_u32 * h)
{
	_u32 t;
	_u32 tmp;

	h[0] = 0x67452301;
	h[1] = 0xefcdab89;
	h[2] = 0x98badcfe;
	h[3] = 0x10325476;
	h[4] = 0xc3d2e1f0;

	for (t = 0; t < 80; t++) {
		if ((t >= 16) && (t < 80)) {
			tmp =
			    w[(t + SHA_BUFF_COUNT -
			       3) % SHA_BUFF_COUNT] ^ w[(t + SHA_BUFF_COUNT -
							 8) %
							SHA_BUFF_COUNT] ^ w[(t +
									     SHA_BUFF_COUNT
									     -
									     14)
									    %
									    SHA_BUFF_COUNT]
			    ^ w[(t + SHA_BUFF_COUNT - 16) % SHA_BUFF_COUNT];
			w[t % SHA_BUFF_COUNT] = rol(tmp, 1);
		}

		if ((t >= 0) && (t < 20)) {
			tmp =
			    rol(h[0],
				5) + ((h[1] & h[2]) | (h[3] & ~h[1])) + h[4] +
			    w[t % SHA_BUFF_COUNT] + 0x5a827999;
#ifdef Debug_message
			// TXHDCP_DEBUG_PRINT(("%08lX %08lX %08lX %08lX %08lX\n",h[0],h[1],h[2],h[3],h[4]));
#endif
			h[4] = h[3];
			h[3] = h[2];
			h[2] = rol(h[1], 30);
			h[1] = h[0];
			h[0] = tmp;
		}
		if ((t >= 20) && (t < 40)) {
			tmp =
			    rol(h[0],
				5) + (h[1] ^ h[2] ^ h[3]) + h[4] +
			    w[t % SHA_BUFF_COUNT] + 0x6ed9eba1;
#ifdef Debug_message
			// TXHDCP_DEBUG_PRINT(("%08lX %08lX %08lX %08lX %08lX\n",h[0],h[1],h[2],h[3],h[4]));
#endif
			h[4] = h[3];
			h[3] = h[2];
			h[2] = rol(h[1], 30);
			h[1] = h[0];
			h[0] = tmp;
		}
		if ((t >= 40) && (t < 60)) {
			tmp =
			    rol(h[0],
				5) +
			    ((h[1] & h[2]) | (h[1] & h[3]) | (h[2] & h[3])) +
			    h[4] + w[t % SHA_BUFF_COUNT] + 0x8f1bbcdc;
#ifdef Debug_message
			//TXHDCP_DEBUG_PRINT(("%08lX %08lX %08lX %08lX %08lX\n",h[0],h[1],h[2],h[3],h[4]));
#endif
			h[4] = h[3];
			h[3] = h[2];
			h[2] = rol(h[1], 30);
			h[1] = h[0];
			h[0] = tmp;
		}
		if ((t >= 60) && (t < 80)) {
			tmp =
			    rol(h[0],
				5) + (h[1] ^ h[2] ^ h[3]) + h[4] +
			    w[t % SHA_BUFF_COUNT] + 0xca62c1d6;
#ifdef Debug_message
			//TXHDCP_DEBUG_PRINT(("%08lX %08lX %08lX %08lX %08lX\n",h[0],h[1],h[2],h[3],h[4]));
#endif
			h[4] = h[3];
			h[3] = h[2];
			h[2] = rol(h[1], 30);
			h[1] = h[0];
			h[0] = tmp;
		}
	}
#ifdef Debug_message
	//TXHDCP_DEBUG_PRINT(("%08lX %08lX %08lX %08lX %08lX\n",h[0],h[1],h[2],h[3],h[4]));
#endif
	h[0] += 0x67452301;
	h[1] += 0xefcdab89;
	h[2] += 0x98badcfe;
	h[3] += 0x10325476;
	h[4] += 0xc3d2e1f0;

#ifdef Debug_message
	//TXHDCP_DEBUG_PRINT(("%08lX %08lX %08lX %08lX %08lX\n",h[0],h[1],h[2],h[3],h[4]));
#endif
}

/* ----------------------------------------------------------------------
 * Outer SHA algorithm: take an arbitrary length _u8 string,
 * convert it into 16-_u16 blocks with the prescribed padding at
 * the end,and pass those blocks to the core SHA algorithm.
 */

void SHA_Simple(void *p, LONG len, _u8 * output)
{
	// SHA_State s;
	_u16 i, t;
	_u32 c;
	_u8 *pBuff = p;
	for (i = 0; i < len; i++) {
		t = i / 4;
		if (i % 4 == 0) {
			w[t] = 0;
		}
		c = pBuff[i];
		c <<= (3 - (i % 4)) * 8;
		w[t] |= c;
	}
	t = i / 4;
	if (i % 4 == 0) {
		w[t] = 0;
	}
	//c=0x80 << ((3-i%4)*24);
	c = 0x80;
	c <<= ((3 - i % 4) * 8);
	w[t] |= c;
	t++;
	for (; t < 15; t++) {
		w[t] = 0;
	}
	w[15] = len * 8;
	for (i = 0; i < len; i++) {
		t = i / 4;
#ifdef Debug_message
		//HDMITX_DEBUG_PRINTF(("pBuff[%d]=%X,w[%d]=%08lX\n",(_u16)i,(_u16)pBuff[i],t,w[t]));
#endif
	}

	SHATransform(sha);
	for (i = 0; i < 5; i++) {
		output[i * 4] = (_u8) ((sha[i] >> 24) & 0xFF);
		output[i * 4 + 1] = (_u8) ((sha[i] >> 16) & 0xFF);
		output[i * 4 + 2] = (_u8) ((sha[i] >> 8) & 0xFF);
		output[i * 4 + 3] = (_u8) (sha[i] & 0xFF);
#ifdef Debug_message
		//HDMITX_DEBUG_PRINTF(("Read V' = %02X %02X %02X %02X\n",(_u16)output[i*4],(_u16)output[i*4+1],(_u16)output[i*4+2],(_u16)output[i*4+3]));
#endif
	}
}

static SYS_STATUS HDCP_CheckSHA(_u8 pM0[], _u16 BStatus, _u8 pKSVList[],
				int cDownStream, _u8 Vr[])
{
	int i, n;
	for (i = 0; i < cDownStream * 5; i++) {
		cBuf[i] = pKSVList[i];
	}
	i = cDownStream * 5;
	cBuf[i++] = BStatus & 0xFF;
	cBuf[i++] = (BStatus >> 8) & 0xFF;
	for (n = 0; n < 8; n++, i++) {
		cBuf[i] = pM0[n];
	}
	n = i;
	// SHABuff[i++] = 0x80 ; // end mask
	for (; i < 64; i++) {
		cBuf[i] = 0;
	}
	// n = cDownStream * 5 + 2 /* for BStatus */ + 8 /* for M0 */ ;
	// n *= 8 ;
	// SHABuff[62] = (n>>8) & 0xff ;
	// SHABuff[63] = (n>>8) & 0xff ;
	/*
	   #ifdef Debug_message
	   for(i = 0 ; i < 64 ; i++)
	   {
	   if(i % 16 == 0)
	   {
	   HDMITX_DEBUG_PRINTF(("SHA[]: "));
	   }
	   HDMITX_DEBUG_PRINTF((" %02X",(_u16)cBuf[i]));
	   if((i%16)==15)
	   {
	   HDMITX_DEBUG_PRINTF(("\n"));
	   }
	   }
	   #endif
	 */
	SHA_Simple(cBuf, n, V);
	/*
	   #ifdef Debug_message
	   HDMITX_DEBUG_PRINTF(("V[] ="));
	   for(i = 0 ; i < 20 ; i++)
	   {
	   HDMITX_DEBUG_PRINTF((" %02X",(_u16)V[i]));
	   }
	   HDMITX_DEBUG_PRINTF(("\nVr[] ="));
	   for(i = 0 ; i < 20 ; i++)
	   {
	   HDMITX_DEBUG_PRINTF((" %02X",(_u16)Vr[i]));
	   }
	   #endif
	 */
	for (i = 0; i < 20; i++) {
		if (V[i] != Vr[i]) {
			return ER_FAIL;
		}
	}
	return ER_SUCCESS;
}

static SYS_STATUS HDCP_Authenticate_Repeater(void)
{
	_u8 uc;
//    _u8 revoked ;
	// int i ;
	_u8 cDownStream;

	_u8 BCaps;
	_u16 BStatus;
	_u16 TimeOut;
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("Authentication for repeater\n"));
#endif
	// emily add for test,abort HDCP
	// 2007/10/01 marked by jj_tseng@chipadvanced.com
	// HDMITX_WriteI2C_Byte(0x20,0x00);
	// HDMITX_WriteI2C_Byte(0x04,0x01);
	// HDMITX_WriteI2C_Byte(0x10,0x01);
	// HDMITX_WriteI2C_Byte(0x15,0x0F);
	// delay1ms(100);
	// HDMITX_WriteI2C_Byte(0x04,0x00);
	// HDMITX_WriteI2C_Byte(0x10,0x00);
	// HDMITX_WriteI2C_Byte(0x20,0x01);
	// delay1ms(100);
	// test07 = HDMITX_ReadI2C_Byte(0x7);
	// test06 = HDMITX_ReadI2C_Byte(0x6);
	// test08 = HDMITX_ReadI2C_Byte(0x8);
	//~jj_tseng@chipadvanced.com
	// end emily add for test
	//////////////////////////////////////
	// Authenticate Fired
	//////////////////////////////////////

	HDCP_GetBCaps(&BCaps, &BStatus);
	delay1ms(2);
	HDCP_Auth_Fire();
	delay1ms(550);		// emily add for test

	for (TimeOut = 250 * 6; TimeOut > 0; TimeOut--) {

		uc = HDMITX_ReadI2C_Byte(REG_TX_INT_STAT1);
		if (uc & B_INT_DDC_BUS_HANG) {
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("DDC Bus hang\n"));
#endif
			goto HDCP_Repeater_Fail;
		}

		uc = HDMITX_ReadI2C_Byte(REG_TX_INT_STAT2);

		if (uc & B_INT_AUTH_FAIL) {
			/*
			   HDMITX_WriteI2C_Byte(REG_TX_INT_CLR0,B_CLR_AUTH_FAIL);
			   HDMITX_WriteI2C_Byte(REG_TX_INT_CLR1,0);
			   HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS,B_INTACTDONE);
			   HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS,0);
			 */
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("HDCP_Authenticate_Repeater(): B_INT_AUTH_FAIL.\n"));
#endif
			goto HDCP_Repeater_Fail;
		}
		// emily add for test
		// test =(HDMITX_ReadI2C_Byte(0x7)&0x4)>>2 ;
		if (uc & B_INT_KSVLIST_CHK) {
			HDMITX_WriteI2C_Byte(REG_TX_INT_CLR0, B_CLR_KSVLISTCHK);
			HDMITX_WriteI2C_Byte(REG_TX_INT_CLR1, 0);
			HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, B_INTACTDONE);
			HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, 0);
#ifdef Debug_message
			HDMITX_DEBUG_PRINTF(("B_INT_KSVLIST_CHK\n"));
#endif
			break;
		}

		if (HDCP_GetBCaps(&BCaps, &BStatus) == ER_FAIL) {
			goto HDCP_Repeater_Fail;
		}
		if (BCaps & B_CAP_KSV_FIFO_RDY) {
			break;
		}

		delay1ms(5);
	}

	if (TimeOut == 0) {
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("Time out for wait KSV List checking interrupt\n"));
#endif
		goto HDCP_Repeater_Fail;
	}
	///////////////////////////////////////
	// clear KSVList check interrupt.
	///////////////////////////////////////

	for (TimeOut = (500); TimeOut > 0; TimeOut--) {
		/*
		   #ifdef Debug_message
		   if((TimeOut % 100) == 0)
		   {
		   HDMITX_DEBUG_PRINTF(("Wait KSV FIFO Ready %d\n",(_u16)TimeOut));
		   }
		   #endif
		 */
		if (HDCP_GetBCaps(&BCaps, &BStatus) == ER_FAIL) {
#ifdef Debug_message
			//HDMITX_DEBUG_PRINTF(("Get BCaps fail\n"));
#endif
			goto HDCP_Repeater_Fail;
		}
#ifdef Debug_message
		//HDMITX_DEBUG_PRINTF(("BCaps = %X , BStatus = %X\n",(_u16)BCaps,BStatus));
#endif
		if (BCaps & B_CAP_KSV_FIFO_RDY) {
#ifdef Debug_message
			//HDMITX_DEBUG_PRINTF(("FIFO Ready\n"));
#endif
			break;
		}
		delay1ms(5);

	}

	if (TimeOut == 0) {
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("Get KSV FIFO ready TimeOut\n"));
#endif
		goto HDCP_Repeater_Fail;
	}
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("Wait timeout = %d\n", (_u16) TimeOut));
#endif
	ClearDDCFIFO();
	GenerateDDCSCLK();
	cDownStream = (BStatus & M_DOWNSTREAM_COUNT);

	if ( /*cDownStream == 0 || */ cDownStream > 6
	    || BStatus & (B_MAX_CASCADE_EXCEEDED | B_DOWNSTREAM_OVER)) {
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF(("Invalid Down stream count,fail\n"));
#endif
		goto HDCP_Repeater_Fail;
	}

	if (HDCP_GetKSVList(&cBuf[64], cDownStream) == ER_FAIL) {
		goto HDCP_Repeater_Fail;
	}
#if 0
	for (i = 0; i < cDownStream; i++) {
		revoked = FALSE;
		uc = 0;
		for (TimeOut = 0; TimeOut < 5; TimeOut++) {
			// check bit count
			uc += countbit(KSVList[i * 5 + TimeOut]);
		}
		if (uc != 20)
			revoked = TRUE;

		if (revoked) {
#ifdef Debug_message
//            HDMITX_DEBUG_PRINTF(("KSVFIFO[%d] = %02X %02X %02X %02X %02X is revoked\n",i,KSVList[i*5],KSVList[i*5+1],KSVList[i*5+2],KSVList[i*5+3],KSVList[i*5+4]));
#endif
			goto HDCP_Repeater_Fail;
		}
	}
#endif

	if (HDCP_GetVr(Vr) == ER_FAIL) {
		goto HDCP_Repeater_Fail;
	}

	if (HDCP_GetM0(M0) == ER_FAIL) {
		goto HDCP_Repeater_Fail;
	}
	// do check SHA
	if (HDCP_CheckSHA(M0, BStatus, &cBuf[64], cDownStream, Vr) == ER_FAIL) {
		goto HDCP_Repeater_Fail;
	}

	HDCP_ResumeRepeaterAuthenticate();
	Instance[0].bAuthenticated = TRUE;
	return ER_SUCCESS;

HDCP_Repeater_Fail:
	HDCP_CancelRepeaterAuthenticate();
	return ER_FAIL;
}

//////////////////////////////////////////////////////////////////////
// Function: HDCP_ResumeAuthentication
// Parameter: N/A
// Return: N/A
// Remark: called by interrupt handler to restart Authentication and Encryption.
// Side-Effect: as Authentication and Encryption.
//////////////////////////////////////////////////////////////////////

static void HDCP_ResumeAuthentication(void)
{
	SetAVMute(TRUE);
	if (HDCP_Authenticate() == ER_SUCCESS) {
		HDCP_EnableEncryption();
	}
	SetAVMute(FALSE);
}

#endif // SUPPORT_HDCP

static void ENABLE_NULL_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_NULL_CTRL, B_ENABLE_PKT | B_REPEAT_PKT);
}

static void ENABLE_ACP_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_ACP_CTRL, B_ENABLE_PKT | B_REPEAT_PKT);
}

static void ENABLE_ISRC1_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_ISRC1_CTRL, B_ENABLE_PKT | B_REPEAT_PKT);
}

static void ENABLE_ISRC2_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_ISRC2_CTRL, B_ENABLE_PKT | B_REPEAT_PKT);
}

static void ENABLE_AVI_INFOFRM_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_AVI_INFOFRM_CTRL,
			     B_ENABLE_PKT | B_REPEAT_PKT);
}

static void ENABLE_AUD_INFOFRM_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_AUD_INFOFRM_CTRL,
			     B_ENABLE_PKT | B_REPEAT_PKT);
}

static void ENABLE_SPD_INFOFRM_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_SPD_INFOFRM_CTRL,
			     B_ENABLE_PKT | B_REPEAT_PKT);
}

static void ENABLE_MPG_INFOFRM_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_MPG_INFOFRM_CTRL,
			     B_ENABLE_PKT | B_REPEAT_PKT);
}

static void DISABLE_NULL_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_NULL_CTRL, 0);
}

static void DISABLE_ACP_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_ACP_CTRL, 0);
}

static void DISABLE_ISRC1_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_ISRC1_CTRL, 0);
}

static void DISABLE_ISRC2_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_ISRC2_CTRL, 0);
}

static void DISABLE_AVI_INFOFRM_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_AVI_INFOFRM_CTRL, 0);
}

static void DISABLE_AUD_INFOFRM_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_AUD_INFOFRM_CTRL, 0);
}

static void DISABLE_SPD_INFOFRM_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_SPD_INFOFRM_CTRL, 0);
}

static void DISABLE_MPG_INFOFRM_PKT(void)
{

	HDMITX_WriteI2C_Byte(REG_TX_MPG_INFOFRM_CTRL, 0);
}

//////////////////////////////////////////////////////////////////////
// Function: SetAVIInfoFrame()
// Parameter: pAVIInfoFrame - the pointer to HDMI AVI Infoframe ucData
// Return: N/A
// Remark: Fill the AVI InfoFrame ucData,and count checksum,then fill into
//         AVI InfoFrame registers.
// Side-Effect: N/A
//////////////////////////////////////////////////////////////////////

static SYS_STATUS SetAVIInfoFrame(AVI_InfoFrame * pAVIInfoFrame)
{
	int i;
	_u8 ucData;

	if (!pAVIInfoFrame) {
		return ER_FAIL;
	}

	Switch_HDMITX_Bank(1);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB1,
			     pAVIInfoFrame->pktbyte.AVI_DB[0]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB2,
			     pAVIInfoFrame->pktbyte.AVI_DB[1]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB3,
			     pAVIInfoFrame->pktbyte.AVI_DB[2]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB4,
			     pAVIInfoFrame->pktbyte.AVI_DB[3]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB5,
			     pAVIInfoFrame->pktbyte.AVI_DB[4]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB6,
			     pAVIInfoFrame->pktbyte.AVI_DB[5]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB7,
			     pAVIInfoFrame->pktbyte.AVI_DB[6]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB8,
			     pAVIInfoFrame->pktbyte.AVI_DB[7]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB9,
			     pAVIInfoFrame->pktbyte.AVI_DB[8]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB10,
			     pAVIInfoFrame->pktbyte.AVI_DB[9]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB11,
			     pAVIInfoFrame->pktbyte.AVI_DB[10]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB12,
			     pAVIInfoFrame->pktbyte.AVI_DB[11]);
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_DB13,
			     pAVIInfoFrame->pktbyte.AVI_DB[12]);
	for (i = 0, ucData = 0; i < 13; i++) {
		ucData -= pAVIInfoFrame->pktbyte.AVI_DB[i];
	}
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF(("SetAVIInfo(): "));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB1)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB2)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB3)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB4)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB5)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB6)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB7)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB8)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB9)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB10)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB11)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB12)));
	HDMITX_DEBUG_PRINTF(("%02X ",
			     (int)HDMITX_ReadI2C_Byte(REG_TX_AVIINFO_DB13)));
	HDMITX_DEBUG_PRINTF(("\n"));
#endif
	ucData -=
	    0x80 + AVI_INFOFRAME_VER + AVI_INFOFRAME_TYPE + AVI_INFOFRAME_LEN;
	HDMITX_WriteI2C_Byte(REG_TX_AVIINFO_SUM, ucData);

	Switch_HDMITX_Bank(0);
	ENABLE_AVI_INFOFRM_PKT();
	return ER_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// Function: SetAudioInfoFrame()
// Parameter: pAudioInfoFrame - the pointer to HDMI Audio Infoframe ucData
// Return: N/A
// Remark: Fill the Audio InfoFrame ucData,and count checksum,then fill into
//         Audio InfoFrame registers.
// Side-Effect: N/A
//////////////////////////////////////////////////////////////////////

static SYS_STATUS SetAudioInfoFrame(Audio_InfoFrame * pAudioInfoFrame)
{
//    int i ;
	_u8 uc;

	if (!pAudioInfoFrame) {
		return ER_FAIL;
	}

	Switch_HDMITX_Bank(1);
	uc = 0x80 - (AUDIO_INFOFRAME_VER + AUDIO_INFOFRAME_TYPE +
		     AUDIO_INFOFRAME_LEN);
	HDMITX_WriteI2C_Byte(REG_TX_PKT_AUDINFO_CC,
			     pAudioInfoFrame->pktbyte.AUD_DB[0]);
	uc -= HDMITX_ReadI2C_Byte(REG_TX_PKT_AUDINFO_CC);
	uc &= 0xFF;
	HDMITX_WriteI2C_Byte(REG_TX_PKT_AUDINFO_SF,
			     pAudioInfoFrame->pktbyte.AUD_DB[1]);
	uc -= HDMITX_ReadI2C_Byte(REG_TX_PKT_AUDINFO_SF);
	uc &= 0xFF;
	HDMITX_WriteI2C_Byte(REG_TX_PKT_AUDINFO_CA,
			     pAudioInfoFrame->pktbyte.AUD_DB[3]);
	uc -= HDMITX_ReadI2C_Byte(REG_TX_PKT_AUDINFO_CA);
	uc &= 0xFF;

	HDMITX_WriteI2C_Byte(REG_TX_PKT_AUDINFO_DM_LSV,
			     pAudioInfoFrame->pktbyte.AUD_DB[4]);
	uc -= HDMITX_ReadI2C_Byte(REG_TX_PKT_AUDINFO_DM_LSV);
	uc &= 0xFF;

	HDMITX_WriteI2C_Byte(REG_TX_PKT_AUDINFO_SUM, uc);

	Switch_HDMITX_Bank(0);
	ENABLE_AUD_INFOFRM_PKT();
	return ER_SUCCESS;
}

#if 0
// //////////////////////////////////////////////////////////////////////
// // Function: SetSPDInfoFrame()
// // Parameter: pSPDInfoFrame - the pointer to HDMI SPD Infoframe ucData
// // Return: N/A
// // Remark: Fill the SPD InfoFrame ucData,and count checksum,then fill into
// //         SPD InfoFrame registers.
// // Side-Effect: N/A
// //////////////////////////////////////////////////////////////////////
//
// static SYS_STATUS
// SetSPDInfoFrame(SPD_InfoFrame *pSPDInfoFrame)
// {
//     int i ;
//     _u8 ucData ;
//
//     if(!pSPDInfoFrame)
//     {
//         return ER_FAIL ;
//     }
//
//     Switch_HDMITX_Bank(1);
//     for(i = 0,ucData = 0 ; i < 25 ; i++)
//     {
//         ucData -= pSPDInfoFrame->pktbyte.SPD_DB[i] ;
//         HDMITX_WriteI2C_Byte(REG_TX_PKT_SPDINFO_PB1+i,pSPDInfoFrame->pktbyte.SPD_DB[i]);
//     }
//     ucData -= 0x80+SPD_INFOFRAME_VER+SPD_INFOFRAME_TYPE+SPD_INFOFRAME_LEN ;
//     HDMITX_WriteI2C_Byte(REG_TX_PKT_SPDINFO_SUM,ucData); // checksum
//     Switch_HDMITX_Bank(0);
//     ENABLE_SPD_INFOFRM_PKT();
//     return ER_SUCCESS ;
// }
//
// //////////////////////////////////////////////////////////////////////
// // Function: SetMPEGInfoFrame()
// // Parameter: pMPEGInfoFrame - the pointer to HDMI MPEG Infoframe ucData
// // Return: N/A
// // Remark: Fill the MPEG InfoFrame ucData,and count checksum,then fill into
// //         MPEG InfoFrame registers.
// // Side-Effect: N/A
// //////////////////////////////////////////////////////////////////////
//
// static SYS_STATUS
// SetMPEGInfoFrame(MPEG_InfoFrame *pMPGInfoFrame)
// {
//     int i ;
//     _u8 ucData ;
//
//     if(!pMPGInfoFrame)
//     {
//         return ER_FAIL ;
//     }
//
//     Switch_HDMITX_Bank(1);
//
//     HDMITX_WriteI2C_Byte(REG_TX_PKT_MPGINFO_FMT,pMPGInfoFrame->info.FieldRepeat|(pMPGInfoFrame->info.MpegFrame<<1));
//     HDMITX_WriteI2C_Byte(REG_TX_PKG_MPGINFO_DB0,pMPGInfoFrame->pktbyte.MPG_DB[0]);
//     HDMITX_WriteI2C_Byte(REG_TX_PKG_MPGINFO_DB1,pMPGInfoFrame->pktbyte.MPG_DB[1]);
//     HDMITX_WriteI2C_Byte(REG_TX_PKG_MPGINFO_DB2,pMPGInfoFrame->pktbyte.MPG_DB[2]);
//     HDMITX_WriteI2C_Byte(REG_TX_PKG_MPGINFO_DB3,pMPGInfoFrame->pktbyte.MPG_DB[3]);
//
//     for(ucData = 0,i = 0 ; i < 5 ; i++)
//     {
//         ucData -= pMPGInfoFrame->pktbyte.MPG_DB[i] ;
//     }
//     ucData -= 0x80+MPEG_INFOFRAME_VER+MPEG_INFOFRAME_TYPE+MPEG_INFOFRAME_LEN ;
//
//     HDMITX_WriteI2C_Byte(REG_TX_PKG_MPGINFO_SUM,ucData);
//
//     Switch_HDMITX_Bank(0);
//     ENABLE_SPD_INFOFRM_PKT();
//
//     return ER_SUCCESS ;
// }
#endif

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

void setIT626X_ChStat(_u8 ucIEC60958ChStat[])
{
	_u8 uc;

	Switch_HDMITX_Bank(1);
	uc = (ucIEC60958ChStat[0] << 1) & 0x7C;
	HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_MODE, uc);
	HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_CAT, ucIEC60958ChStat[1]);	// 192, audio CATEGORY
	HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_SRCNUM, ucIEC60958ChStat[2] & 0xF);
	HDMITX_WriteI2C_Byte(REG_TX_AUD0CHST_CHTNUM,
			     (ucIEC60958ChStat[2] >> 4) & 0xF);
	HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_CA_FS, ucIEC60958ChStat[3]);	// choose clock
	HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_OFS_WL, ucIEC60958ChStat[4]);
	Switch_HDMITX_Bank(0);
}

void setIT626X_UpdateChStatFs(_u32 Fs)
{
	_u8 uc;

	/////////////////////////////////////
	// Fs should be the following value.
	// #define AUDFS_22p05KHz  4
	// #define AUDFS_44p1KHz 0
	// #define AUDFS_88p2KHz 8
	// #define AUDFS_176p4KHz    12
	//
	// #define AUDFS_24KHz  6
	// #define AUDFS_48KHz  2
	// #define AUDFS_96KHz  10
	// #define AUDFS_192KHz 14
	//
	// #define AUDFS_768KHz 9
	//
	// #define AUDFS_32KHz  3
	// #define AUDFS_OTHER    1
	/////////////////////////////////////

	Switch_HDMITX_Bank(1);
	uc = HDMITX_ReadI2C_Byte(REG_TX_AUDCHST_CA_FS);	// choose clock
	HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_CA_FS, uc);	// choose clock
	uc &= 0xF0;
	uc |= (Fs & 0xF);

	uc = HDMITX_ReadI2C_Byte(REG_TX_AUDCHST_OFS_WL);
	uc &= 0xF;
	uc |= ((~Fs) << 4) & 0xF0;
	HDMITX_WriteI2C_Byte(REG_TX_AUDCHST_OFS_WL, uc);

	Switch_HDMITX_Bank(0);

}

void setIT626X_LPCMAudio(_u8 AudioSrcNum, _u8 AudSWL, BOOL bSPDIF)
{

	_u8 AudioEnable, AudioFormat;

	AudioEnable = 0;
	AudioFormat = Instance[0].bOutputAudioMode;

	switch (AudSWL) {
	case 16:
		AudioEnable |= M_AUD_16BIT;
		break;
	case 18:
		AudioEnable |= M_AUD_18BIT;
		break;
	case 20:
		AudioEnable |= M_AUD_20BIT;
		break;
	case 24:
	default:
		AudioEnable |= M_AUD_24BIT;
		break;
	}

	if (bSPDIF) {
		AudioFormat &= ~0x40;
		AudioEnable |= B_AUD_SPDIF | B_AUD_EN_I2S0;
	} else {
		AudioFormat |= 0x40;
		switch (AudioSrcNum) {
		case 4:
			AudioEnable |=
			    B_AUD_EN_I2S3 | B_AUD_EN_I2S2 | B_AUD_EN_I2S1 |
			    B_AUD_EN_I2S0;
			break;

		case 3:
			AudioEnable |=
			    B_AUD_EN_I2S2 | B_AUD_EN_I2S1 | B_AUD_EN_I2S0;
			break;

		case 2:
			AudioEnable |= B_AUD_EN_I2S1 | B_AUD_EN_I2S0;
			break;

		case 1:
		default:
			AudioFormat &= ~0x40;
			AudioEnable |= B_AUD_EN_I2S0;
			break;

		}
	}
	AudioFormat |= 0x01;	//mingchih add
	Instance[0].bAudioChannelEnable = AudioEnable;

	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0, AudioEnable & 0xF0);

	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL1, AudioFormat);	// regE1 bOutputAudioMode should be loaded from ROM image.
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_FIFOMAP, 0xE4);	// default mapping.
#ifdef USE_SPDIF_CHSTAT
	if (bSPDIF) {
		HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL3, B_CHSTSEL);
	} else {
		HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL3, 0);
	}
#else // not USE_SPDIF_CHSTAT
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL3, 0);
#endif // USE_SPDIF_CHSTAT

	HDMITX_WriteI2C_Byte(REG_TX_AUD_SRCVALID_FLAT, 0x00);
	HDMITX_WriteI2C_Byte(REG_TX_AUD_HDAUDIO, 0x00);	// regE5 = 0 ;

	if (bSPDIF) {
		_u8 i;
		HDMI_OrREG_TX_Byte(0x5c, (1 << 6));
		for (i = 0; i < 100; i++) {
			if (HDMITX_ReadI2C_Byte(REG_TX_CLK_STATUS2) &
			    B_OSF_LOCK) {
				break;	// stable clock.
			}
		}
	}
//    HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0, Instance[0].bAudioChannelEnable);
}

void setIT626X_NLPCMAudio(void)	// no Source Num, no I2S.
{
	_u8 AudioEnable, AudioFormat;
	_u8 i;

	AudioFormat = 0x01;	// NLPCM must use standard I2S mode.
	AudioEnable = M_AUD_24BIT | B_AUD_SPDIF | B_AUD_EN_I2S0;

	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0, M_AUD_24BIT | B_AUD_SPDIF);
	//HDMITX_AndREG_Byte(REG_TX_SW_RST,~(B_AUD_RST|B_AREF_RST));

	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL1, 0x01);	// regE1 bOutputAudioMode should be loaded from ROM image.
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_FIFOMAP, 0xE4);	// default mapping.

#ifdef USE_SPDIF_CHSTAT
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL3, B_CHSTSEL);
#else // not USE_SPDIF_CHSTAT
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL3, 0);
#endif // USE_SPDIF_CHSTAT

	HDMITX_WriteI2C_Byte(REG_TX_AUD_SRCVALID_FLAT, 0x00);
	HDMITX_WriteI2C_Byte(REG_TX_AUD_HDAUDIO, 0x00);	// regE5 = 0 ;

	for (i = 0; i < 100; i++) {
		if (HDMITX_ReadI2C_Byte(REG_TX_CLK_STATUS2) & B_OSF_LOCK) {
			break;	// stable clock.
		}
	}
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0,
			     M_AUD_24BIT | B_AUD_SPDIF | B_AUD_EN_I2S0);
}

void setIT626X_HBRAudio(BOOL bSPDIF)
{
	_u8 rst, uc;
	Switch_HDMITX_Bank(0);

	rst = HDMITX_ReadI2C_Byte(REG_TX_SW_RST);

	//HDMITX_WriteI2C_Byte(REG_TX_SW_RST, rst | (B_AUD_RST|B_AREF_RST) );

	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL1, 0x47);	// regE1 bOutputAudioMode should be loaded from ROM image.
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_FIFOMAP, 0xE4);	// default mapping.

	if (bSPDIF) {
		HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0,
				     M_AUD_24BIT | B_AUD_SPDIF);
		HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL3, B_CHSTSEL);
	} else {
		HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0, M_AUD_24BIT);
		HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL3, 0);
	}

	HDMITX_WriteI2C_Byte(REG_TX_AUD_SRCVALID_FLAT, 0x08);
	HDMITX_WriteI2C_Byte(REG_TX_AUD_HDAUDIO, B_HBR);	// regE5 = 0 ;
	uc = HDMITX_ReadI2C_Byte(REG_TX_CLK_CTRL1);
	uc &= ~M_AUD_DIV;
	HDMITX_WriteI2C_Byte(REG_TX_CLK_CTRL1, uc);

	if (bSPDIF) {
		_u8 i;
		for (i = 0; i < 100; i++) {
			if (HDMITX_ReadI2C_Byte(REG_TX_CLK_STATUS2) &
			    B_OSF_LOCK) {
				break;	// stable clock.
			}
		}
		HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0,
				     M_AUD_24BIT | B_AUD_SPDIF |
				     B_AUD_EN_SPDIF);
	} else {
		HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0,
				     M_AUD_24BIT | B_AUD_EN_I2S3 | B_AUD_EN_I2S2
				     | B_AUD_EN_I2S1 | B_AUD_EN_I2S0);
	}
	HDMI_AndREG_TX_Byte(0x5c, ~(1 << 6));
	Instance[0].bAudioChannelEnable =
	    HDMITX_ReadI2C_Byte(REG_TX_AUDIO_CTRL0);
	//HDMITX_WriteI2C_Byte(REG_TX_SW_RST, rst & ~(B_AUD_RST|B_AREF_RST) );
}

void setIT626X_DSDAudio(void)
{
	// to be continue
	_u8 rst, uc;
	rst = HDMITX_ReadI2C_Byte(REG_TX_SW_RST);

	//HDMITX_WriteI2C_Byte(REG_TX_SW_RST, rst | (B_AUD_RST|B_AREF_RST) );

	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL1, 0x41);	// regE1 bOutputAudioMode should be loaded from ROM image.
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_FIFOMAP, 0xE4);	// default mapping.

	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0, M_AUD_24BIT);
	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL3, 0);

	HDMITX_WriteI2C_Byte(REG_TX_AUD_SRCVALID_FLAT, 0x00);
	HDMITX_WriteI2C_Byte(REG_TX_AUD_HDAUDIO, B_DSD);	// regE5 = 0 ;
	//HDMITX_WriteI2C_Byte(REG_TX_SW_RST, rst & ~(B_AUD_RST|B_AREF_RST) );

	uc = HDMITX_ReadI2C_Byte(REG_TX_CLK_CTRL1);
	uc &= ~M_AUD_DIV;
	HDMITX_WriteI2C_Byte(REG_TX_CLK_CTRL1, uc);

	HDMITX_WriteI2C_Byte(REG_TX_AUDIO_CTRL0,
			     M_AUD_24BIT | B_AUD_EN_I2S3 | B_AUD_EN_I2S2 |
			     B_AUD_EN_I2S1 | B_AUD_EN_I2S0);
}

void EnableHDMIAudio(_u8 AudioType, BOOL bSPDIF, _u32 SampleFreq, _u8 ChNum,
		     _u8 * pIEC60958ChStat, _u32 TMDSClock)
{
	static _u8 ucIEC60958ChStat[5];
	_u8 Fs;
	Instance[0].TMDSClock = TMDSClock;
	Instance[0].bAudioChannelEnable = 0;
	Instance[0].bSPDIF_OUT = bSPDIF;
	HDMITX_WriteI2C_Byte(REG_TX_CLK_CTRL0,
			     B_AUTO_OVER_SAMPLING_CLOCK | B_EXT_256FS | 0x01);
	if (bSPDIF) {
		if (AudioType == T_AUDIO_HBR) {
			HDMITX_WriteI2C_Byte(REG_TX_CLK_CTRL0, 0x81);
		}
		HDMITX_OrREG_Byte(REG_TX_AUDIO_CTRL0, B_AUD_SPDIF);
	} else {
		HDMITX_AndREG_Byte(REG_TX_AUDIO_CTRL0, (~B_AUD_SPDIF));
	}

	if (AudioType != T_AUDIO_DSD) {
		// one bit audio have no channel status.
		switch (SampleFreq) {
		case 44100L:
			Fs = AUDFS_44p1KHz;
			break;
		case 88200L:
			Fs = AUDFS_88p2KHz;
			break;
		case 176400L:
			Fs = AUDFS_176p4KHz;
			break;
		case 32000L:
			Fs = AUDFS_32KHz;
			break;
		case 48000L:
			Fs = AUDFS_48KHz;
			break;
		case 96000L:
			Fs = AUDFS_96KHz;
			break;
		case 192000L:
			Fs = AUDFS_192KHz;
			break;
		case 768000L:
			Fs = AUDFS_768KHz;
			break;
		default:
			SampleFreq = 48000L;
			Fs = AUDFS_48KHz;
			break;	// default, set Fs = 48KHz.
		}

#ifdef SUPPORT_AUDIO_MONITOR
		Instance[0].bAudFs = AUDFS_OTHER;
#else
		Instance[0].bAudFs = Fs;
#endif

		if (pIEC60958ChStat == NULL) {
			ucIEC60958ChStat[0] = 0;
			ucIEC60958ChStat[1] = 0;
			ucIEC60958ChStat[2] = (ChNum + 1) / 2;

			if (ucIEC60958ChStat[2] < 1) {
				ucIEC60958ChStat[2] = 1;
			} else if (ucIEC60958ChStat[2] > 4) {
				ucIEC60958ChStat[2] = 4;
			}

			ucIEC60958ChStat[3] = Fs;
#if(SUPPORT_AUDI_AudSWL==16)
			ucIEC60958ChStat[4] = (((~Fs) << 4) & 0xF0) | 0x02;	// Fs | 24bit _u16 length
#elif(SUPPORT_AUDI_AudSWL==18)
			ucIEC60958ChStat[4] = (((~Fs) << 4) & 0xF0) | 0x04;	// Fs | 24bit _u16 length
#elif(SUPPORT_AUDI_AudSWL==20)
			ucIEC60958ChStat[4] = (((~Fs) << 4) & 0xF0) | 0x03;	// Fs | 24bit _u16 length
#else
			ucIEC60958ChStat[4] = (((~Fs) << 4) & 0xF0) | 0x0B;	// Fs | 24bit _u16 length
#endif
			pIEC60958ChStat = ucIEC60958ChStat;
		}
	}

	switch (AudioType) {
	case T_AUDIO_HBR:
		HDMITX_DEBUG_PRINTF(("T_AUDIO_HBR\n"));
		pIEC60958ChStat[0] |= 1 << 1;
		pIEC60958ChStat[2] = 0;
		pIEC60958ChStat[3] &= 0xF0;
		pIEC60958ChStat[3] |= AUDFS_768KHz;
		pIEC60958ChStat[4] |= (((~AUDFS_768KHz) << 4) & 0xF0) | 0xB;
		setIT626X_ChStat(pIEC60958ChStat);
		setIT626X_HBRAudio(bSPDIF);

		break;
	case T_AUDIO_DSD:
		HDMITX_DEBUG_PRINTF(("T_AUDIO_DSD\n"));
		setIT626X_DSDAudio();
		break;
	case T_AUDIO_NLPCM:
		HDMITX_DEBUG_PRINTF(("T_AUDIO_NLPCM\n"));
		pIEC60958ChStat[0] |= 1 << 1;
		setIT626X_ChStat(pIEC60958ChStat);
		setIT626X_NLPCMAudio();
		break;
	case T_AUDIO_LPCM:
		HDMITX_DEBUG_PRINTF(("T_AUDIO_LPCM\n"));
		pIEC60958ChStat[0] &= ~(1 << 1);

		setIT626X_ChStat(pIEC60958ChStat);
		setIT626X_LPCMAudio((ChNum + 1) / 2,	/*24 */
				    SUPPORT_AUDI_AudSWL, bSPDIF);
		// can add auto adjust
		break;
	}
	HDMITX_AndREG_Byte(REG_TX_INT_MASK1, (~B_AUDIO_OVFLW_MASK));
	HDMITX_AndREG_Byte(REG_TX_SW_RST, ~(B_AUD_RST | B_AREF_RST));
}

void SoftWareVideoReset(void)
{
	_u8 ResetCount;
	_u8 intclr = 0;

	// Switch_HDMITX_Bank(0) ;
	// CurrCTS=0;
	ResetCount = 5;
	I2C_OrReg_Byte(REG_TX_SW_RST, B_VID_RST);	//add 20080702  hermes
	delay1ms(1);
	I2C_AndReg_Byte(REG_TX_SW_RST, ~(B_VID_RST));
	while (0 == (HDMITX_ReadI2C_Byte(REG_TX_SYS_STATUS) & B_TXVIDSTABLE)) {
		delay1ms(100);
		ResetCount--;
		if (ResetCount == 0)
			break;
#ifdef Debug_message
		HDMITX_DEBUG_PRINTF((" wait video stable \n"));
#endif
	}
#ifdef Debug_message
	HDMITX_DEBUG_PRINTF((" =======================reset video==========================\n"));
#endif
	I2C_OrReg_Byte(REG_TX_INT_CLR1, B_CLR_VIDSTABLE);
	intclr |=
	    (HDMITX_ReadI2C_Byte(REG_TX_SYS_STATUS) & (~B_CLR_AUD_CTS)) |
	    B_INTACTDONE;
	HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, intclr);
	intclr &= ~B_INTACTDONE;
	HDMITX_WriteI2C_Byte(REG_TX_SYS_STATUS, intclr);	// INTACTDONE reset to zero.

}

BOOL GetVideoStatus(void)
{
	if (HDMITX_ReadI2C_Byte(REG_TX_SYS_STATUS) & B_TXVIDSTABLE) {
		return TRUE;
	} else {
		return FALSE;
	}
}

void UpDateAFE(_u32 PixelClock)
{
	_u8 uc, level;
	unsigned long cTMDSClock = PixelClock;	//*(pixelrep+1);
	uc = HDMITX_ReadI2C_Byte(0xc1);
	switch (uc & 0x70) {
	case 0x50:
		cTMDSClock *= 5;
		cTMDSClock /= 4;
		break;
	case 0x60:
		cTMDSClock *= 3;
		cTMDSClock /= 2;
	}

	if (cTMDSClock > 80000000L) {
		level = PCLK_HIGH;
	} else if (cTMDSClock > 20000000L) {
		level = PCLK_MEDIUM;
	} else {
		level = PCLK_LOW;
	}
	SetupAFE(level);
	//FireAFE();
}

void EnableDeOnly(BOOL DeOnly)
{
	if (DeOnly) {
		EnableHVToolDetect(FALSE);
		HDMITX_OrREG_Byte(0xa5, (1 << 5));
		ReGenTimingEnable = TRUE;
	} else {
		EnableHVToolDetect(TRUE);
		HDMITX_AndREG_Byte(0xa5, ~(1 << 5));
		ReGenTimingEnable = FALSE;
	}
}

void EnableHVToolDetect(BOOL HVenable)
{
	if (HVenable)
		HDMITX_OrREG_Byte(0xa8, (1 << 3));
	else
		HDMITX_AndREG_Byte(0xa8, ~(1 << 3));
}

UINT GetHTotal(void)
{
	return (((_u16) HDMITX_ReadI2C_Byte(0x91)) << 4) +
	    ((_u16) (HDMITX_ReadI2C_Byte(0x90) >> 4)) + 1;
}

UINT GetHActive(void)
{
	return
	    abso(((((_u16) (HDMITX_ReadI2C_Byte(0x94) & 0xf0)) << 4) +
		  ((_u16) HDMITX_ReadI2C_Byte(0x93))) -
		 ((((_u16) (HDMITX_ReadI2C_Byte(0x94) & 0x0f)) << 8) +
		  ((_u16) HDMITX_ReadI2C_Byte(0x92))));
}

UINT GetHBlank(void)
{
	return (GetHTotal() - GetHActive());
}

UINT GetVTotal(void)
{
	return ((_u16) (HDMITX_ReadI2C_Byte(0x99) & 0x07) << 8) +
	    ((_u16) HDMITX_ReadI2C_Byte(0x98)) + 1;
}

UINT GetVActive(void)
{
	return
	    abso(((((_u16) (HDMITX_ReadI2C_Byte(0x9C) & 0x70)) << 4) +
		  ((_u16) HDMITX_ReadI2C_Byte(0x9B))) -
		 ((((_u16) (HDMITX_ReadI2C_Byte(0x9C) & 0x07)) << 8) +
		  ((_u16) HDMITX_ReadI2C_Byte(0x9A))));
}

UINT GetVBlank(void)
{
	return (GetVTotal() - GetVActive());
}

#ifdef SUPPORT_SPLASH
void FastOutput(void)
{
	Switch_HDMITX_Bank(0);
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST,
			     B_VID_RST | B_AUD_RST | B_AREF_RST | B_HDCP_RST |
			     B_REF_RST);
	HDMITX_WriteI2C_Byte(REG_TX_SW_RST,
			     B_AUD_RST | B_AREF_RST | B_HDCP_RST);
	InitLVDS();
	SetLVDSinterface();
	EnableAVIInfoFrame(FALSE, NULL);
	HDMITX_WriteI2C_Byte(REG_TX_HDMI_MODE, B_TX_DVI_MODE);
	UpDateAFE(GetInputPclk());
	FireAFE();
	SetAVMute(FALSE);
	delay1ms(4200);
	//DisableVideoOutput();
}
#endif
/*  marked   
#ifdef SUPPORT_GPIO_MUTE
BOOL LastGPIO_Status=TRUE;
void GPIO_Panel_En()
{
    if(LastGPIO_Status == TRUE &&
        P1_7 == FALSE)
    {
        DisableAudioOutput();
        printf("P1_7 == FALSE\n");
    }

    if(LastGPIO_Status == FALSE &&
        P1_7 == TRUE)
    {
        HDMITX_AndREG_Byte(0x04,~(0x14));
        SetAVMute(FALSE);
        printf("P1_7 == TRUE\n");
    }

    LastGPIO_Status=P1_7;
}
#endif
*/
void CheckAudioVideoInput(void)
{
	_u8 LVDS_Lock;
	_u8 NoAudio_flag, AudioOFF_flag;
#ifdef SUPPORT_GPIO_MUTE
	GPIO_Panel_En();
#else
	LVDS_Lock = (0x03 & LVDS_ReadI2C_Byte(0x30));
	NoAudio_flag = ((1 << 4) & HDMITX_ReadI2C_Byte(0x5f));
	AudioOFF_flag = ((1 << 4) & HDMITX_ReadI2C_Byte(0xc5));
	if (0x03 != LVDS_Lock) {
#ifdef SUPPORT_HDCP
		if (Instance[0].bAuthenticated) {
			EnableHDCP(FALSE);
		}
#endif
		DisableAudioOutput();
	}
	if (NoAudio_flag != AudioOFF_flag) {
		HDMITX_WriteI2C_Byte(0xc5,
				     (((0xEF) & HDMITX_ReadI2C_Byte(0xc5)) |
				      NoAudio_flag));
	}
#endif
}

#ifdef Powerdown
void Power_Down(void)
{
	_u8 temp;
	HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL,
			     B_AFE_DRV_RST | B_AFE_DRV_PWD);
	temp = LVDS_ReadI2C_Byte(0x0B);
	LVDS_WriteI2C_Byte(0x0B, (temp & 0x7F));
	HDMITX_DEBUG_PRINTF(("it6263 power down "));
}

void Power_Resume(void)
{
	_u8 temp;
	HDMITX_WriteI2C_Byte(REG_TX_AFE_DRV_CTRL, 0x00);
	temp = LVDS_ReadI2C_Byte(0x0B);
	LVDS_WriteI2C_Byte(0x0B, (temp | 0x80));

	HDMITX_DEBUG_PRINTF(("it6263 power resume "));
	delay1ms(100);
	InitIT626X_Instance();
	InitIT626X();
#ifdef SUPPORT_SPLASH
	FastOutput();
#endif
	HDMITX_ChangeDisplayOption(HDMI_1080p24, HDMI_RGB444);
	VideoModeDetect();
}
#endif
//~jj_tseng@chipadvanced.com 2008/08/18
//////////////////////////////////////////////////////////////////////
// Function: DumpCatHDMITXReg()
// Parameter: N/A
// Return: N/A
// Remark: Debug function,dumps the registers of CAT6611.
// Side-Effect: N/A
//////////////////////////////////////////////////////////////////////

#ifdef Debug_message
void DumpLVDSReg(void)
{
	int i, j;
	_u8 ucData;
	HDMITX_DEBUG_PRINTF(("        "));
	for (j = 0; j < 16; j++) {

		if (j < 0x10)
			HDMITX_DEBUG_PRINTF(("0%X ", j));
		else
			HDMITX_DEBUG_PRINTF(("%X ", j));
		if ((j == 3) || (j == 7) || (j == 11)) {
			HDMITX_DEBUG_PRINTF(("  "));
		}
	}
	HDMITX_DEBUG_PRINTF(("\n        -----------------------------------------------------\n"));

	Switch_HDMITX_Bank(0);

	for (i = 0; i < 0x100; i += 16) {
		if (i < 0x10)
			HDMITX_DEBUG_PRINTF(("[ 0%X ]  ", i));
		else
			HDMITX_DEBUG_PRINTF(("[ %X ]  ", i));
		for (j = 0; j < 16; j++) {
			ucData = LVDS_ReadI2C_Byte(i + j);
			//i2c_read_byte(0x66, (i+j), 1, &ucData, I2CDEV);
			if (ucData < 0x10)
				HDMITX_DEBUG_PRINTF(("0%X ", (_u16) ucData));
			else
				HDMITX_DEBUG_PRINTF(("%X ", (_u16) ucData));
			if ((j == 3) || (j == 7) || (j == 11)) {
				HDMITX_DEBUG_PRINTF((" -"));
			}
		}
		HDMITX_DEBUG_PRINTF(("\n"));
		if ((i % 0x40) == 0x30) {
			HDMITX_DEBUG_PRINTF(("        -----------------------------------------------------\n"));
		}
	}

}

void DumpCatHDMITXReg(void)
{
	int i, j;
	_u8 ucData;

	HDMITX_DEBUG_PRINTF(("        "));
	for (j = 0; j < 16; j++) {

		if (j < 0x10)
			HDMITX_DEBUG_PRINTF(("0%X ", j));
		else
			HDMITX_DEBUG_PRINTF(("%X ", j));
		if ((j == 3) || (j == 7) || (j == 11)) {
			HDMITX_DEBUG_PRINTF(("  "));
		}
	}
	HDMITX_DEBUG_PRINTF(("\n        -----------------------------------------------------\n"));

	Switch_HDMITX_Bank(0);

	for (i = 0; i < 0x100; i += 16) {
		if (i < 0x10)
			HDMITX_DEBUG_PRINTF(("[ 0%X ]  ", i));
		else
			HDMITX_DEBUG_PRINTF(("[ %X ]  ", i));
		for (j = 0; j < 16; j++) {
			ucData = HDMITX_ReadI2C_Byte((_u8) ((i + j) & 0xFF));
			if (ucData < 0x10)
				HDMITX_DEBUG_PRINTF(("0%X ", (_u16) ucData));
			else
				HDMITX_DEBUG_PRINTF(("%X ", (_u16) ucData));
			if ((j == 3) || (j == 7) || (j == 11)) {
				HDMITX_DEBUG_PRINTF((" -"));
			}
		}
		HDMITX_DEBUG_PRINTF(("\n"));
		if ((i % 0x40) == 0x30) {
			HDMITX_DEBUG_PRINTF(("        -----------------------------------------------------\n"));
		}
	}
	Switch_HDMITX_Bank(1);
	for (i = 0x00; i < 0x100; i += 16) {
		if (i < 0x10)
			HDMITX_DEBUG_PRINTF(("[10%X ]  ", i));
		else
			HDMITX_DEBUG_PRINTF(("[1%X ]  ", i));
		for (j = 0; j < 16; j++) {
			ucData = HDMITX_ReadI2C_Byte((_u8) ((i + j) & 0xFF));
			if (ucData < 0x10)
				HDMITX_DEBUG_PRINTF(("0%X ", (_u16) ucData));
			else
				HDMITX_DEBUG_PRINTF(("%X ", (_u16) ucData));
			if ((j == 3) || (j == 7) || (j == 11)) {
				HDMITX_DEBUG_PRINTF((" -"));
			}
		}
		HDMITX_DEBUG_PRINTF(("\n"));
		if ((i % 0x40) == 0x30) {
			HDMITX_DEBUG_PRINTF(("        -----------------------------------------------------\n"));
		}

	}
	Switch_HDMITX_Bank(0);
}
#endif
static int it6263_loop_kthread(void *data)
{
	struct it6263_data *it6263data = get_it6263_data();	//dev_get_drvdata(it6263_dev);

	delay1ms(100);
	printk("it6263_loop_kthread CREATE !!! \n");

	InitIT626X_Instance();
	InitIT626X();
#ifdef SUPPORT_SPLASH
	FastOutput();
#endif
	HDMITX_ChangeDisplayOption(HDMI_1080p24, HDMI_RGB444);

	VideoModeDetect();

	while (1) {
		wait_event_interruptible_timeout(it6263data->it6263_wq, 0, 100 * HZ / 1000);	//100ms
		if (kthread_should_stop())
			break;
		mutex_lock(&it6263data->lock);
		HDMITX_DevLoopProc();
		mutex_unlock(&it6263data->lock);
	}

	return 0;
}

static int init_it6263_loop_kthread(void)
{
	struct it6263_data *it6263data = dev_get_drvdata(it6263_dev);
	printk("IT6263 -- %s ++\n", __FUNCTION__);
	it6263data->it6263_timer_task =
	    kthread_create(it6263_loop_kthread, NULL, "it6263_loop_kthread");
	wake_up_process(it6263data->it6263_timer_task);
	return 0;
}

ssize_t it6263_fop_read(struct file * file, char *buf, size_t count,
			loff_t * f_ops)
{
	return 0;
}

ssize_t it6263_fop_write(struct file * file, const char *buf, size_t count,
			 loff_t * f_ops)
{
	return 0;
}

ssize_t it6263_fop_open(struct inode * inode, struct file * file)
{
	return 0;
}

ssize_t it6263_fop_release(struct inode * inode, struct file * file)
{
	return 0;
}

long it6263_fop_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	printk("it6263_fop_ioctl - cmd=%u, arg=0x%08x\n", cmd, (int)arg);
	return 0;
}

static const struct file_operations it6263_file_ops = {
	.owner = THIS_MODULE,
	.open = it6263_fop_open,
	.read = it6263_fop_read,
	.write = it6263_fop_write,
	.unlocked_ioctl = it6263_fop_ioctl,
	.release = it6263_fop_release,
};

static int it6263_hdmi_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct it6263_data *it6263;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(" i2c_check_functionality()    FAIL!!!) \n");
		return -EIO;
	}

	it6263 = kzalloc(sizeof(struct it6263_data), GFP_KERNEL);
	if (!it6263) {
		printk("IT6263 -- failed to allocate driver data\n");
		return -ENOMEM;
	}
	memset(it6263, 0, sizeof(struct it6263_data));
	g_it6263_data = it6263;
	it6263_platform = client->dev.platform_data;
	it6263_platform->hdmi_client = client;
	it6263->it6263_timer_task = NULL;
	it6263->dev_inited = 0;
	mutex_init(&it6263->lock);
	i2c_set_clientdata(client, it6263);
	it6263_dev = &client->dev;
	init_waitqueue_head(&it6263->it6263_wq);

	printk("it6263 i2c probe!!! \n");
	return 0;
}

static int it6263_LVDS_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{

	it6263_platform->LVDS_client = client;

	return 0;
}

static int it6263_hdmi_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int it6263_LVDS_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id it6263_hdmi_i2c_id[] = {
	{"it6263_hdmi_i2c", 0},
	{}
};

static const struct i2c_device_id it6263_LVDS_i2c_id[] = {
	{"it6263_LVDS_i2c", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, it6263_hdmi_i2c_id);
MODULE_DEVICE_TABLE(i2c, it6263_LVDS_i2c_id);
static struct i2c_driver it6263_hdmi_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "it6263_hdmi_i2c",
		   },
	.id_table = it6263_hdmi_i2c_id,
	.probe = it6263_hdmi_i2c_probe,
	.remove = it6263_hdmi_i2c_remove,

};

static struct i2c_driver it6263_LVDS_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "it6263_LVDS_i2c",
		   },
	.id_table = it6263_LVDS_i2c_id,
	.probe = it6263_LVDS_i2c_probe,
	.remove = it6263_LVDS_i2c_remove,

};

//---------    I2C  Board information    -------------

#define IT6263_HDMI_ADDR 0x98
#define IT6263_LVDS_ADDR 0x66
static struct it6263_platform_data pdata_it6263;
static struct i2c_board_info __initdata board_i2c_it6263[] = {
	{
	 I2C_BOARD_INFO("it6263_hdmi_i2c", IT6263_HDMI_ADDR >> 1),
	 .platform_data = &pdata_it6263,
	 },
	{
	 I2C_BOARD_INFO("it6263_LVDS_i2c", IT6263_LVDS_ADDR >> 1),
	 .platform_data = &pdata_it6263,
	 },
};

//--------------------------------------------
static int __init it6263_init(void)
{
	int ret = 0, i = 0;
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	adapter = i2c_get_adapter(2);
	if (!adapter) {
		printk("%s:fail to get adapter!\n", __func__);
		return -1;
	}
	for (i = 0; i < 2; i++) {
		client = i2c_new_device(adapter, &board_i2c_it6263[i]);
		if (!client) {
			printk("i2c_new_device() Fail!! \n");
			return -ENXIO;
		}
	}
	ret = i2c_add_driver(&it6263_hdmi_i2c_driver);
	if (ret < 0)
		goto err_exit1;
	msleep(20);
	ret = i2c_add_driver(&it6263_LVDS_i2c_driver);
	if (ret < 0)
		goto err_exit2;

	printk("it6263_init done\n");

	init_it6263_loop_kthread();

	return ret;

err_exit1:
	i2c_del_driver(&it6263_hdmi_i2c_driver);
	printk("i2c_add_hdmi_driver fail\n");
err_exit2:
	i2c_del_driver(&it6263_LVDS_i2c_driver);
	printk("i2c_add_LVDS_driver fail\n");
	return ret;
}

static void __exit it6263_exit(void)
{
	struct it6263_data *it6263 = dev_get_drvdata(it6263_dev);
	if (it6263 && it6263->it6263_timer_task) {

		kthread_stop(it6263->it6263_timer_task);
		it6263->it6263_timer_task = NULL;
	}
	i2c_del_driver(&it6263_hdmi_i2c_driver);
	i2c_del_driver(&it6263_LVDS_i2c_driver);
	printk("it6263 i2c driver delete \n");
}

//------------------------------   I2C  Prototype ------------------------------//
module_init(it6263_init);
module_exit(it6263_exit);
