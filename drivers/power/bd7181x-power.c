/*
 * bd7181x-power.c
 * @file ROHM BD71815/BD71817 Charger driver
 *
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/mfd/bd7181x.h>
#include <linux/delay.h>

#define JITTER_DEFAULT		3000		/* hope 3s is enough */
#define JITTER_REPORT_CAP	10000		/* 10 seconds */
#define BD7181X_BATTERY_CAP	mAh_A10s(battery_capacity)
#define MAX_VOLTAGE		ocv_table[0]
#define MIN_VOLTAGE		3400000
#define THR_VOLTAGE		3500000
#define MAX_CURRENT		1500000		/* uA */
#define AC_NAME			"bd7181x_ac"
#define BAT_NAME		"bd7181x_bat"
#define BD7181X_BATTERY_FULL	100

#define BY_BAT_VOLT		0
#define BY_VBATLOAD_REG		1
#define INIT_COULOMB		BY_VBATLOAD_REG

#define USE_CALIB_CURRENT	1
#define CALIB_CURRENT_A2A3	0xCE9E

//VBAT Low voltage detection Threshold 
#define VBAT_LOW_TH		0x00D4 // 0x00D4*16mV = 212*0.016 = 3.392v 



#define A10s_mAh(s)		((s) * 1000 / 360)
#define mAh_A10s(m)		((m) * 360 / 1000)

#define MIN_FULL_CHG_TEMP	15	/* 1 degrees C unit */
#define MAX_FULL_CHG_TEMP	45	/* 1 degrees C unit */

static unsigned int battery_capacity;


static int ocv_table[] = {
	4200000,
	4176940,
	4126229,
	4082231,
	4041213,
	3997549,
	3968819,
	3938827,
	3911201,
	3877931,
	3839736,
	3817228,
	3801908,
	3790304,
	3781487,
	3775685,
	3767288,
	3750583,
	3731266,
	3703010,
	3686252,
	3614821
};	/* unit 1 micro V */

static int soc_table[] = {
	1000,
	1000,
	950,
	900,
	850,
	800,
	750,
	700,
	650,
	600,
	550,
	500,
	450,
	400,
	350,
	300,
	250,
	200,
	150,
	100,
	50,
	0
	/* unit 0.1% */
};


/** @brief power deivce */
struct bd7181x_power {
	struct device *dev;
	struct bd7181x *mfd;			/**< parent for access register */
	struct power_supply ac;			/**< alternating current power */
	struct power_supply bat;		/**< battery power */
	struct delayed_work bd_work;		/**< delayed work for timed work */

	int    reg_index;			/**< register address saved for sysfs */

	int    vbus_status;			/**< last vbus status */
	int    charge_status;			/**< last charge status */
	int    bat_status;			/**< last bat status */

	int	hw_ocv1;			/**< HW ocv1 */
	int	hw_ocv2;			/**< HW ocv2 */
	int	bat_online;			/**< battery connect */
	int	charger_online;			/**< charger connect */
	int	vcell;				/**< battery voltage */
	int	rpt_status;			/**< battery status report */
	int	prev_rpt_status;		/**< previous battery status report */
	int	bat_health;			/**< battery health */
	int	full_cap;			/**< battery capacity */
	int	curr;				/**< battery current from DS-ADC */
	int	curr_sar;			/**< battery current from VM_IBAT */
	int	temp;				/**< battery tempature */
	u32	coulomb_cnt;			/**< Coulomb Counter */
	int	state_machine;			/**< initial-procedure state machine */
	#if USE_CALIB_CURRENT
	volatile int calib_current;		/**< calibration current */
	#endif
	u32	soc;				/**< State Of Charge with by load */
	u32	soc_norm;			/**< State Of Charge without by load */
};

#if USE_CALIB_CURRENT
#define CALIB_NORM			0
#define CALIB_START			1
#define CALIB_GO			2
#endif

enum {
	STAT_POWER_ON,
	STAT_INITIALIZED,
};

/** @brief read a register group once
 *  @param mfd bd7181x device
 *  @param reg	 register address of lower register
 *  @return register value
 */
#ifdef __BD7181X_REGMAP_H__
static u16 bd7181x_reg_read16(struct bd7181x* mfd, int reg) {
	u16 v;

	v = (u16)bd7181x_reg_read(mfd, reg) << 8;
	v |= (u16)bd7181x_reg_read(mfd, reg + 1) << 0;
	return v;
}
#else
static u16 bd7181x_reg_read16(struct bd7181x* mfd, int reg) {
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return be16_to_cpu(u.long_type);
}
#endif

/** @brief write a register group once
 * @param mfd bd7181x device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd7181x_reg_write16(struct bd7181x *mfd, int reg, u16 val) {
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	u.long_type = cpu_to_be16(val);
	// printk("write16 0x%.4X 0x%.4X\n", val, u.long_type);
#ifdef __BD7181X_REGMAP_H__
	r = mfd->write(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return 0;	
}

/** @brief read quad register once
 *  @param mfd bd7181x device
 *  @param reg	 register address of lower register
 *  @return register value
 */
static int bd7181x_reg_read32(struct bd7181x *mfd, int reg) {
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

#ifdef __BD7181X_REGMAP_H__
	r = mfd->read(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return be32_to_cpu(u.long_type);
}

#if 0
/** @brief write quad register once
 * @param mfd bd7181x device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd7181x_reg_write32(struct bd7181x *mfd, int reg, unsigned val) {
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

	u.long_type = cpu_to_be32(val);
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return 0;
}
#endif

#if INIT_COULOMB == BY_VBATLOAD_REG
/** @brief get initial battery voltage and current
 * @param pwr power device
 * @return 0
 */
static int bd7181x_get_init_bat_stat(struct bd7181x_power *pwr) {
	struct bd7181x *mfd = pwr->mfd;
	int vcell;

	vcell = bd7181x_reg_read16(mfd, BD7181X_REG_VM_VBATLOAD_PRE_U) * 1000;
	dev_info(pwr->dev, "VM_VBATLOAD_PRE = %d\n", vcell);
	pwr->hw_ocv1 = vcell;

	vcell = bd7181x_reg_read16(mfd, BD7181X_REG_VM_VBATLOAD_PST_U) * 1000;
	dev_info(pwr->dev, "VM_VBATLOAD_PST = %d\n", vcell);
	pwr->hw_ocv2 = vcell;

	return 0;
}
#endif

/** @brief get battery average voltage and current
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @param curr  pointer to return back current in unit uA.
 * @return 0
 */
static int bd7181x_get_vbat_curr(struct bd7181x_power *pwr, int *vcell, int *curr) {
	struct bd7181x* mfd = pwr->mfd;
	int tmp_vcell, tmp_curr;

	tmp_vcell = 0;
	tmp_curr = 0;

	tmp_vcell = bd7181x_reg_read16(mfd, BD7181X_REG_VM_SMA_VBAT_U);
	tmp_curr = bd7181x_reg_read16(mfd, BD7181X_REG_VM_SMA_IBAT_U);
	if (bd7181x_reg_read(mfd, BD7181X_REG_CC_CURCD_U) & CURDIR_Discharging) {
		tmp_curr = -tmp_curr;
	}

	*vcell = tmp_vcell * 1000;
	*curr = tmp_curr * 1000;

	return 0;
}

/** @brief get battery current from DS-ADC
 * @param pwr power device
 * @return current in unit uA
 */
static int bd7181x_get_current_ds_adc(struct bd7181x_power *pwr) {
	int r;
	
	r = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_CC_CURCD_U);
	if (r < 0) {
		return 0;
	}
	if (r & 0x8000) {
		r = -(r & ~0x8000);
	}
	return r * 1000;
}

/** @brief get battery capacity
 * @param ocv open circuit voltage
 * @return capcity in unit 0.1 percent
 */
static int bd7181x_voltage_to_capacity(int ocv) {
	int i = 0;
	int soc;

	if (ocv > ocv_table[0]) {
		soc = soc_table[0];
	} else {
		i = 0;
		while (soc_table[i] != 0) {
			if ((ocv <= ocv_table[i]) && (ocv > ocv_table[i+1])) {
				soc = (soc_table[i] - soc_table[i+1]) * (ocv - ocv_table[i+1]) / (ocv_table[i] - ocv_table[i+1]);
				soc += soc_table[i+1];
				break;
			}
			i++;
		}
		if (soc_table[i] == 0)
			soc = soc_table[i];
	}
	return soc;
}

/** @brief get battery temperature
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd7181x_get_temp(struct bd7181x_power *pwr) {
	struct bd7181x* mfd = pwr->mfd;
	int t;

	t = 200 - (int)bd7181x_reg_read(mfd, BD7181X_REG_VM_BTMP);

	// battery temperature error
	t = (t > 200)? 200: t;
	
	return t;
}

/** @brief get battery charge status
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd7181x_charge_status(struct bd7181x_power *pwr)
{
	u8 state;
	int ret = 1;

	state = bd7181x_reg_read(pwr->mfd, BD7181X_REG_CHG_STATE);
	// dev_info(pwr->dev, "CHG_STATE %d\n", state);

	switch (state) {
	case 0x00:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x0E:
		pwr->rpt_status = POWER_SUPPLY_STATUS_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x0F:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_FULL;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x13:
	case 0x14:
	case 0x20:
	case 0x21:
	case 0x22:
	case 0x23:
	case 0x24:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case 0x30:
	case 0x31:
	case 0x32:
	case 0x40:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x7f:
	default:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_DEAD;
		break;	
	}

	if ((pwr->prev_rpt_status != POWER_SUPPLY_STATUS_FULL) && (pwr->rpt_status == POWER_SUPPLY_STATUS_FULL) && 
		(pwr->temp >= MIN_FULL_CHG_TEMP) && (pwr->temp <= MAX_FULL_CHG_TEMP)) {
		/* Stop Coulomb Counter */
		bd7181x_clear_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

		bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_3, ((pwr->full_cap + pwr->full_cap / 200) & 0x1FFFUL));

		pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
		dev_info(pwr->dev, "Reset Coulomb Counter at POWER_SUPPLY_STATUS_FULL\n");
		dev_info(pwr->dev, "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);
	}

	pwr->prev_rpt_status = pwr->rpt_status;

	return ret;
}

#if INIT_COULOMB == BY_BAT_VOLT
static int bd7181x_calib_voltage(struct bd7181x_power* pwr, int* ocv) {
	int r, curr, volt;

	bd7181x_get_vbat_curr(pwr, &volt, &curr);

	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_CHG_STATE);
	if (r >= 0 && curr > 0) {
		// voltage increment caused by battery inner resistor
		if (r == 3) volt -= 100 * 1000;
		else if (r == 2) volt -= 50 * 1000;
	}
	*ocv = volt;

	return 0;
}
#endif

/** @brief set initial coulomb counter value from battery voltage
 * @param pwr power device
 * @return 0
 */
static int calibration_coulomb_counter(struct bd7181x_power* pwr) {
	u32 bcap;
	int soc, ocv;

#if INIT_COULOMB == BY_VBATLOAD_REG
	/* Get init OCV by HW */
	bd7181x_get_init_bat_stat(pwr);

	ocv = (pwr->hw_ocv1 >= pwr->hw_ocv2)? pwr->hw_ocv1: pwr->hw_ocv2;
	dev_info(pwr->dev, "ocv %d\n", ocv);
#elif INIT_COULOMB == BY_BAT_VOLT
	bd7181x_calib_voltage(pwr, &ocv);
#endif

	/* Get init soc from ocv/soc table */
	soc = bd7181x_voltage_to_capacity(ocv);
	dev_info(pwr->dev, "soc %d[0.1%%]\n", soc);

	bcap = pwr->full_cap * soc / 1000;

	bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_3, ((bcap + bcap / 200) & 0x1FFFUL));

	pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
	dev_info(pwr->dev, "%s() CC_CCNTD = %d\n", __func__, pwr->coulomb_cnt);

	/* Start canceling offset of the DS ADC. This needs 1 second at least */
	bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCCALIB);

	return 0;
}

/** @brief get battery parameters, such as voltages, currents, temperatures.
 * @param pwr power device
 * @return 0
 */
static int bd7181x_get_vcell(struct bd7181x_power* pwr)
{
	int curr, curr_sar;

	/* Read detailed vcell and current */
	bd7181x_get_vbat_curr(pwr, &pwr->vcell, &curr_sar);
	// dev_info(pwr->dev, "VM_VBAT = %d\n", pwr->vcell);
	// dev_info(pwr->dev, "VM_IBAT = %d\n", curr_sar);

	curr = bd7181x_get_current_ds_adc(pwr);
	// dev_info(pwr->dev, "CC_CURCD = %d\n", curr);

	pwr->curr = curr;
	pwr->curr_sar = curr_sar;

	/* Get tempature */
	pwr->temp = bd7181x_get_temp(pwr);
	// dev_info(pwr->dev, "Temperature %d degrees C\n", pwr->temp);

	return 0;
}

/** @brief get coulomb counter values
 * @param pwr power device
 * @return 0
 */
static int bd7181x_coulomb_count(struct bd7181x_power* pwr) {
	if (pwr->state_machine == STAT_POWER_ON) {
		pwr->state_machine = STAT_INITIALIZED;
		/* Start Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);
	} else if (pwr->state_machine == STAT_INITIALIZED) {
		pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
		// dev_info(pwr->dev, "CC_CCNTD = %d\n", pwr->coulomb_cnt);
	}
	return 0;
}

/** @brief calculate SOC values
 * @param pwr power device
 * @return 0
 */
static int bd7181x_calc_soc_norm(struct bd7181x_power* pwr) {
	pwr->soc_norm = (pwr->coulomb_cnt >> 16) * 100 /  pwr->full_cap;
	if (pwr->soc_norm > 100) {
		pwr->soc_norm = 100;
		/* Stop Coulomb Counter */
		bd7181x_clear_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

		bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_3, ((pwr->full_cap + pwr->full_cap / 200) & 0x1FFFUL));

		pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
		dev_info(pwr->dev, "Limit Coulomb Counter\n");
		dev_info(pwr->dev, "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);
	}
	{
	static int i;
	if (i++ % 60 == 0)
	dev_info(pwr->dev, "%s() pwr->soc_norm = %d\n", __func__, pwr->soc_norm);
	}
	return 0;
}

/** @brief get OCV value by SOC
 * @param pwr power device
 * @return 0
 */
static int bd7181x_get_ocv(struct bd7181x_power* pwr, u32 dsoc) {
	int i = 0;
	int ocv = 0;

	if (dsoc > soc_table[0]) {
		ocv = MAX_VOLTAGE;
	}
	else if (dsoc == 0) {
		i = 0;
		while (i < 21) {
			if (dsoc == soc_table[i]) {
				ocv = ocv_table[i];
				break;
			}
			i++;
		}
		if (i == 21)
			ocv = ocv_table[21];
	}
	else {
		i = 0;
		while (i < 21) {
			if ((dsoc <= soc_table[i]) && (dsoc > soc_table[i+1])) {
				ocv = (ocv_table[i] - ocv_table[i+1]) * (dsoc - soc_table[i+1]) / (soc_table[i] - soc_table[i+1]) + ocv_table[i+1];
				break;
			}
			i++;
		}
		if (i == 21)
			ocv = ocv_table[21];
	}
	// dev_info(pwr->dev, "%s() ocv = %d\n", __func__, ocv);
	return ocv;
}

/** @brief calculate SOC value by load
 * @param pwr power device
 * @return OCV
 */
static int bd7181x_calc_soc(struct bd7181x_power* pwr) {
	int ocv_table_load[22];
	int lost_cap;

	pwr->soc = pwr->soc_norm;

	switch (pwr->rpt_status) { /* Adjust for 0% between THR_VOLTAGE and MIN_VOLTAGE */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->vcell <= THR_VOLTAGE) {
			int i;
			int ocv;
			u32 dsoc = (pwr->coulomb_cnt >> 16) * 1000 /  pwr->full_cap;
			// dev_info(pwr->dev, "%s() dsoc = %d\n", __func__, dsoc);
			ocv = bd7181x_get_ocv(pwr, dsoc);
			for (i = 1; i < 22; i++) {
				ocv_table_load[i] = ocv_table[i] - (ocv - pwr->vcell);
				if (ocv_table_load[i] <= MIN_VOLTAGE) {
					// dev_info(pwr->dev, "%s() ocv_table_load[%d] = %d\n",
					//		__func__, i, ocv_table_load[i]);
					break;
				}
			}
			if (i < 22) {
				int j;
				int dv = (ocv_table_load[i] - ocv_table_load[i-1]) / 5;
				int mod_coulomb_cnt, mod_full_cap;
				for (j = 1; j < 5; j++){
					if ((ocv_table_load[i] + dv * j) > MIN_VOLTAGE) {
						break;
					}
				}
				lost_cap = ((21 - i) * 5 + (j - 1)) * pwr->full_cap / 100;
				// dev_info(pwr->dev, "%s() lost_cap = %d\n", __func__, lost_cap);
				mod_coulomb_cnt = (pwr->coulomb_cnt >> 16) - lost_cap;
				mod_full_cap = pwr->full_cap - lost_cap;
				if ((mod_coulomb_cnt > 0) && (mod_full_cap > 0)) {
					pwr->soc = mod_coulomb_cnt * 100 / mod_full_cap;
				}
				else {
					pwr->soc = 0;
				}
				// dev_info(pwr->dev, "%s() pwr->soc(by load) = %d\n", __func__, pwr->soc);
			}
		}
		break;
	default:
		break;
	}

	switch (pwr->rpt_status) {/* Adjust for 0% and 100% */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->vcell <= MIN_VOLTAGE) {
			pwr->soc = 0;
		}
		else {
			if (pwr->soc == 0) {
				pwr->soc = 1;
			}
		}
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		if (pwr->soc == 100) {
			pwr->soc = 99;
		}
		break;
	default:
		break;
	}
	{
	static int i;
	if (i++ % 60 == 0) 
	dev_info(pwr->dev, "%s() pwr->soc = %d\n", __func__, pwr->soc);
	}
	return 0;
}

/** @brief get battery and DC online status
 * @param pwr power device
 * @return 0
 */
static int bd7181x_get_online(struct bd7181x_power* pwr) {
	int r;

#if 0
#define TS_THRESHOLD_VOLT	0xD9
	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_VM_VTH);
	pwr->bat_online = (r > TS_THRESHOLD_VOLT);
#endif
#if 0
	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_STAT);
	if (r >= 0 && (r & BAT_DET_DONE)) {
		pwr->bat_online = (r & BAT_DET) != 0;
	}
#endif
#if 1
#define BAT_OPEN	0x7
	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_TEMP);
	pwr->bat_online = (r != BAT_OPEN);
#endif	
	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_DCIN_STAT);
	if (r >= 0) {
		pwr->charger_online = (r & VBUS_DET) != 0;
	}

	return 0;
}

/** @brief init bd7181x sub module charger
 * @param pwr power device
 * @return 0
 */
static int bd7181x_init_hardware(struct bd7181x_power *pwr) {
	struct bd7181x *mfd = pwr->mfd;
	int r;

	r = bd7181x_reg_write(mfd, BD7181X_REG_DCIN_CLPS, 0x36);

#define TEST_SEQ_00		0x00
#define TEST_SEQ_01		0x76
#define TEST_SEQ_02		0x66
#define TEST_SEQ_03		0x56
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_01);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_02);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_03);

	r = bd7181x_reg_read(mfd, BD7181X_REG_VSYS_MAX);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);

#if 0
	for (i = 0; i < 300; i++) {
		r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_STAT);
		if (r >= 0 && (r & BAT_DET_DONE)) {
			break;
		}
		msleep(5);
	}
#endif
	if ((r & 0x01) == 0x00) {
	//if (r & BAT_DET) {
		/* Init HW, when the battery is inserted. */

		bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_01);
		bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_02);
		bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_03);
		bd7181x_reg_write(mfd, BD7181X_REG_VSYS_MAX, r | 0x01);

		bd7181x_reg_write16(mfd, 0xA2, CALIB_CURRENT_A2A3);
		bd7181x_reg_write(mfd, 0x03, 0x1F); //Masked 2.9v ULVO , added by John Zhang
		
		bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);

		/* Stop Coulomb Counter */
		bd7181x_clear_bits(mfd, BD7181X_REG_CC_CTRL, CCNTENB);

		/* Set Coulomb Counter Reset bit*/
		bd7181x_set_bits(mfd, BD7181X_REG_CC_CTRL, CCNTRST);

		/* Clear Coulomb Counter Reset bit*/
		bd7181x_clear_bits(mfd, BD7181X_REG_CC_CTRL, CCNTRST);

		/* Set default Battery Capacity */
		pwr->full_cap = BD7181X_BATTERY_CAP;

		/* Set initial Coulomb Counter by HW OCV */
		calibration_coulomb_counter(pwr);

		/* WDT_FST manual set */
		bd7181x_set_bits(mfd, BD7181X_REG_CHG_SET1, WDT_AUTO);

		/* Watch Dog Timer 480 minutes */
		bd7181x_reg_write(mfd, BD7181X_REG_CHG_WDT_FST, 0x38);

		/* VBAT Low voltage detection Setting, added by John Zhang*/
		bd7181x_reg_write16(mfd, BD7181X_REG_ALM_VBAT_TH_U, VBAT_LOW_TH); 

		pwr->state_machine = STAT_POWER_ON;
	} else {
		pwr->full_cap = BD7181X_BATTERY_CAP;	// bd7181x_reg_read16(pwr->mfd, BD7181X_REG_CC_BATCAP_U);
		pwr->state_machine = STAT_INITIALIZED;	// STAT_INITIALIZED
	}

	pwr->coulomb_cnt = bd7181x_reg_read32(mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
	bd7181x_calc_soc_norm(pwr);
	pwr->soc = pwr->soc_norm;
	dev_info(pwr->dev, "%s() CC_CCNTD = %d\n", __func__, pwr->coulomb_cnt);
	dev_info(pwr->dev, "%s() pwr->soc = %d\n", __func__, pwr->soc);

	pwr->curr = 0;
	pwr->curr_sar = 0;

	return 0;
}

/**@brief timed work function called by system
 *  read battery capacity,
 *  sense change of charge status, etc.
 * @param work work struct
 * @return  void
 */

static void bd_work_callback(struct work_struct *work)
{
	struct bd7181x_power *pwr;
	struct delayed_work *delayed_work;
	int status, changed = 0;
	static int cap_counter = 0;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd7181x_power, bd_work);

	status = bd7181x_reg_read(pwr->mfd, BD7181X_REG_DCIN_STAT);
	if (status != pwr->vbus_status) {
		//printk("DCIN_STAT CHANGED from 0x%X to 0x%X\n", pwr->vbus_status, status);
		pwr->vbus_status = status;
		changed = 1;
	}

	status = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_STAT);
	status &= ~BAT_DET_DONE;
	if (status != pwr->bat_status) {
		//printk("BAT_STAT CHANGED from 0x%X to 0x%X\n", pwr->bat_status, status);
		pwr->bat_status = status;
		changed = 1;
	}

	status = bd7181x_reg_read(pwr->mfd, BD7181X_REG_CHG_STATE);
	if (status != pwr->charge_status) {
		//printk("CHG_STATE CHANGED from 0x%X to 0x%X\n", pwr->charge_status, status);
		pwr->charge_status = status;
		//changed = 1;
	}

	bd7181x_get_vcell(pwr);
	bd7181x_coulomb_count(pwr);
	bd7181x_calc_soc_norm(pwr);
	bd7181x_calc_soc(pwr);
	bd7181x_get_online(pwr);
	bd7181x_charge_status(pwr);

	if (changed || cap_counter++ > JITTER_REPORT_CAP / JITTER_DEFAULT) {
		power_supply_changed(&pwr->ac);
		power_supply_changed(&pwr->bat);
		cap_counter = 0;
	}

	#if USE_CALIB_CURRENT
	if (pwr->calib_current == CALIB_NORM) {
	#endif
		schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(JITTER_DEFAULT));
	#if USE_CALIB_CURRENT
	} else if (pwr->calib_current == CALIB_START) {
		pwr->calib_current = CALIB_GO;
	}
	#endif
}

/**@brief bd7181x power interrupt
 * @param irq system irq
 * @param pwrsys bd7181x power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t bd7181x_power_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd7181x *mfd = dev_get_drvdata(dev->parent);
	// struct bd7181x_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	reg = bd7181x_reg_read(mfd, BD7181X_REG_INT_STAT_03);
	if (reg < 0)
		return IRQ_NONE;

	// printk("INT_STAT_03 = 0x%.2X\n", reg);

	r = bd7181x_reg_write(mfd, BD7181X_REG_INT_STAT_03, reg);
	if (r)
		return IRQ_NONE;

	if (reg & DCIN_MON_DET) {
		// printk("\n~~~DCIN removed\n");
	} else if (reg & DCIN_MON_RES) {
		// printk("\n~~~DCIN inserted\n");
	}

	return IRQ_HANDLED;
}


/**@brief bd7181x vbat low voltage detection interrupt
 * @param irq system irq
 * @param pwrsys bd7181x power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 * added by John Zhang at 2015-07-22
 */
static irqreturn_t bd7181x_vbat_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd7181x *mfd = dev_get_drvdata(dev->parent);
	// struct bd7181x_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	reg = bd7181x_reg_read(mfd, BD7181X_REG_INT_STAT_08);
	if (reg < 0)
		return IRQ_NONE;

	// printk("INT_STAT_08 = 0x%.2X\n", reg);

	r = bd7181x_reg_write(mfd, BD7181X_REG_INT_STAT_08, reg);
	if (r)
		return IRQ_NONE;

	if (reg & VBAT_MON_DET) {
		printk("\n~~~ VBAT LOW Detected ... \n");
		
	} else if (reg & VBAT_MON_RES) {
		printk("\n~~~ VBAT LOW Resumed ... \n");
	}

	return IRQ_HANDLED;
	
}



/** @brief get property of power supply ac
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */
static int bd7181x_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd7181x_power *pwr = dev_get_drvdata(psy->dev->parent);
	u32 vot;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->charger_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_VM_DCIN_U);
		val->intval = 5000 * vot;		// 5 milli volt steps
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief get property of power supply bat
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */

static int bd7181x_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd7181x_power *pwr = dev_get_drvdata(psy->dev->parent);
	// u32 cap, vot, r;
	// u8 ret;

	switch (psp) {
	/*
	case POWER_SUPPLY_PROP_STATUS:
		r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_CHG_STATE);
		// printk("CHG_STATE = 0x%.2X\n", r);
		switch(r) {
		case CHG_STATE_SUSPEND:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case CHG_STATE_TRICKLE_CHARGE:
		case CHG_STATE_PRE_CHARGE:
		case CHG_STATE_FAST_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case CHG_STATE_TOP_OFF:
		case CHG_STATE_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_STAT);
		if (ret & DBAT_DET)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else if (ret & VBAT_OV)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		cap = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_CC_BATCAP_U);
		// printk("CC_BATCAP = 0x%.4X\n", cap);
		val->intval = cap * 100 / 0x1FFF;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_VM_VBAT_U) * 1000;
		val->intval = vot;
		break;
	*/
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pwr->rpt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = pwr->bat_health;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (pwr->rpt_status == POWER_SUPPLY_STATUS_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pwr->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = pwr->soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		{
		u32 t;

		t = pwr->coulomb_cnt >> 16;
		t = A10s_mAh(t);
		if (t > battery_capacity) t = battery_capacity;
		val->intval = t * 1000;		/* uA to report */
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = BD7181X_BATTERY_FULL * battery_capacity * 10;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = pwr->curr_sar;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = pwr->curr;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = pwr->temp * 10; /* 0.1 degrees C unit */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = MAX_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = MIN_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = MAX_CURRENT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief ac properties */
static enum power_supply_property bd7181x_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/** @brief bat properies */
static enum power_supply_property bd7181x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

/** @brief directly set raw value to chip register, format: 'register value' */
static ssize_t bd7181x_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd7181x_power *pwr = container_of(psy, struct bd7181x_power, bat);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		pwr->reg_index = -1;
		return count;
	}

	if (ret == 1 && reg <= BD7181X_MAX_REGISTER) {
		pwr->reg_index = reg;
		return count;
	}
	if (reg > BD7181X_MAX_REGISTER || val > 255)
		return -EINVAL;

	ret = bd7181x_reg_write(pwr->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/** @brief print value of chip register, format: 'register=value' */
static ssize_t bd7181x_sysfs_print_reg(struct bd7181x_power *pwr,
				       u8 reg,
				       char *buf)
{
	int ret = bd7181x_reg_read(pwr->mfd, reg);

	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/** @brief show all raw values of chip register, format per line: 'register=value' */
static ssize_t bd7181x_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd7181x_power *pwr = container_of(psy, struct bd7181x_power, bat);
	ssize_t ret = 0;
	int i;

	if (pwr->reg_index >= 0) {
		ret += bd7181x_sysfs_print_reg(pwr, pwr->reg_index, buf + ret);
	} else {
		for (i = 0; i <= BD7181X_MAX_REGISTER; i++) {
			ret += bd7181x_sysfs_print_reg(pwr, i, buf + ret);
		}
	}
	return ret;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bd7181x_sysfs_show_registers, bd7181x_sysfs_set_registers);

#if USE_CALIB_CURRENT
static int first_offset(struct bd7181x_power *pwr)
{
	unsigned char ra2, ra3, ra6, ra7;
	unsigned char ra2_temp;
	struct bd7181x *mfd = pwr->mfd;

	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_01);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_02);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_03);


	ra2 = bd7181x_reg_read(mfd, 0xA2);	// I want to know initial A2 & A3.
	ra3 = bd7181x_reg_read(mfd, 0xA3);	// I want to know initial A2 & A3.
	ra6 = bd7181x_reg_read(mfd, 0xA6);
	ra7 = bd7181x_reg_read(mfd, 0xA7);

	bd7181x_reg_write(mfd, 0xA2, 0x00);
	bd7181x_reg_write(mfd, 0xA3, 0x00);

	dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2);
	dev_info(pwr->dev, "TEST[A3] = 0x%.2X\n", ra3);
	dev_info(pwr->dev, "TEST[A6] = 0x%.2X\n", ra6);
	dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);

	//-------------- First Step -------------------
	dev_info(pwr->dev, "Frist Step begginning \n");

	// delay some time , Make a state of IBAT=0mA
	// mdelay(1000 * 10);

	ra2_temp = ra2;

	if (ra7 != 0) {
		//if 0<0xA7<20 decrease the Test register 0xA2[7:3] until 0xA7 becomes 0x00.
		if ((ra7 > 0) && (ra7 < 20)) {
			do {
				ra2 = bd7181x_reg_read(mfd, 0xA2);
				ra2_temp = ra2 >> 3;
				ra2_temp -= 1;
				ra2_temp <<= 3;
				bd7181x_reg_write(mfd, 0xA2, ra2_temp);
				dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2_temp);

				ra7 = bd7181x_reg_read(mfd, 0xA7);
				dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);
				mdelay(1000);	// 1sec?
			} while (ra7);

			dev_info(pwr->dev, "A7 becomes 0 . \n");

		}		// end if((ra7 > 0)&&(ra7 < 20)) 
		else if ((ra7 > 0xDF) && (ra7 < 0xFF))
			//if DF<0xA7<FF increase the Test register 0xA2[7:3] until 0xA7 becomes 0x00.
		{
			do {
				ra2 = bd7181x_reg_read(mfd, 0xA2);
				ra2_temp = ra2 >> 3;
				ra2_temp += 1;
				ra2_temp <<= 3;

				bd7181x_reg_write(mfd, 0xA2, ra2_temp);
				dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2_temp);

				ra7 = bd7181x_reg_read(mfd, 0xA7);
				dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);
				mdelay(1000);	// 1sec?                           
			} while (ra7);

			dev_info(pwr->dev, "A7 becomes 0 . \n");
		}
	}

	// please use "ra2_temp" at step2.
	return ra2_temp;
}

static int second_step(struct bd7181x_power *pwr, u8 ra2_temp)
{
	u16 ra6, ra7;
	u8 aft_ra2, aft_ra3;
	u8 r79, r7a;
	unsigned int LNRDSA_FUSE;
	long ADC_SIGN;
	long DSADGAIN1_INI;
	struct bd7181x *mfd = pwr->mfd;

	//-------------- Second Step -------------------
	dev_info(pwr->dev, "Second Step begginning \n");

	// need to change boad setting ( input 1A tio 10mohm)
	// delay some time , Make a state of IBAT=1000mA
	// mdelay(1000 * 10);

// rough adjust
	dev_info(pwr->dev, "ra2_temp = 0x%.2X\n", ra2_temp);

	ra6 = bd7181x_reg_read(mfd, 0xA6);
	ra7 = bd7181x_reg_read(mfd, 0xA7);
	ra6 <<= 8;
	ra6 |= ra7;		// [0xA6 0xA7]
	dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);

	bd7181x_reg_write(mfd, 0xA2, ra2_temp);	// this value from step1
	bd7181x_reg_write(mfd, 0xA3, 0x00);

	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);

	r79 = bd7181x_reg_read(mfd, 0x79);
	r7a = bd7181x_reg_read(mfd, 0x7A);

	ADC_SIGN = r79 >> 7;
	ADC_SIGN = 1 - (2 * ADC_SIGN);
	DSADGAIN1_INI = r79 << 8;
	DSADGAIN1_INI = DSADGAIN1_INI + r7a;
	DSADGAIN1_INI = DSADGAIN1_INI & 0x7FFF;
	DSADGAIN1_INI = DSADGAIN1_INI * ADC_SIGN; //  unit 0.001

	// unit 0.000001
	DSADGAIN1_INI *= 1000;
	{
	if (DSADGAIN1_INI > 1000001) {
		DSADGAIN1_INI = 2048000000UL - (DSADGAIN1_INI - 1000000) * 8187;
	} else if (DSADGAIN1_INI < 999999) {
		DSADGAIN1_INI = -(DSADGAIN1_INI - 1000000) * 8187;
	} else {
		DSADGAIN1_INI = 0;
	}
	}

	LNRDSA_FUSE = (int) DSADGAIN1_INI / 1000000;

	dev_info(pwr->dev, "LNRDSA_FUSE = 0x%.8X\n", LNRDSA_FUSE);

	aft_ra2 = (LNRDSA_FUSE >> 8) & 255;
	aft_ra3 = (LNRDSA_FUSE) & 255;

	aft_ra2 = aft_ra2 + ra2_temp;

	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_01);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_02);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_03);

	bd7181x_reg_write(mfd, 0xA2, aft_ra2);
	bd7181x_reg_write(mfd, 0xA3, aft_ra3);

	return 0;
}

static int third_step(struct bd7181x_power *pwr, unsigned thr) {
	u16 ra2_a3, ra6, ra7;
	u8 ra2, ra3;
	u8 aft_ra2, aft_ra3;
	struct bd7181x *mfd = pwr->mfd;

// fine adjust
	ra2 = bd7181x_reg_read(mfd, 0xA2);	//
	ra3 = bd7181x_reg_read(mfd, 0xA3);	//

	ra6 = bd7181x_reg_read(mfd, 0xA6);
	ra7 = bd7181x_reg_read(mfd, 0xA7);
	ra6 <<= 8;
	ra6 |= ra7;		// [0xA6 0xA7]
	dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);


	if (ra6 > thr) {
		do {
			ra2_a3 = bd7181x_reg_read(mfd, 0xA2);
			ra2_a3 <<= 8;
			ra3 = bd7181x_reg_read(mfd, 0xA3);
			ra2_a3 |= ra3;
			//ra2_a3 >>= 3; // ? 0xA3[7:3] , or 0xA3[7:0]

			ra2_a3 -= 1;
			//ra2_a3 <<= 3;
			ra3 = ra2_a3;
			bd7181x_reg_write(mfd, 0xA3, ra3);

			ra2_a3 >>= 8;
			ra2 = ra2_a3;
			bd7181x_reg_write(mfd, 0xA2, ra2);

			dev_info(pwr->dev, "TEST[A2] = 0x%.2X , TEST[A3] = 0x%.2X \n", ra2, ra3);

			mdelay(1000);	// 1sec?

			ra6 = bd7181x_reg_read(mfd, 0xA6);
			ra7 = bd7181x_reg_read(mfd, 0xA7);
			ra6 <<= 8;
			ra6 |= ra7;	// [0xA6 0xA7]
			dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);
		} while (ra6 > thr);
	} else if (ra6 < thr) {
		do {
			ra2_a3 = bd7181x_reg_read(mfd, 0xA2);
			ra2_a3 <<= 8;
			ra3 = bd7181x_reg_read(mfd, 0xA3);
			ra2_a3 |= ra3;
			//ra2_a3 >>= 3; // ? 0xA3[7:3] , or 0xA3[7:0]

			ra2_a3 += 1;
			//ra2_a3 <<= 3;
			ra3 = ra2_a3;
			bd7181x_reg_write(mfd, 0xA3, ra3);

			ra2_a3 >>= 8;
			ra2 = ra2_a3;
			bd7181x_reg_write(mfd, 0xA2, ra2);

			dev_info(pwr->dev, "TEST[A2] = 0x%.2X , TEST[A3] = 0x%.2X \n", ra2, ra3);

			mdelay(1000);	// 1sec?

			ra6 = bd7181x_reg_read(mfd, 0xA6);
			ra7 = bd7181x_reg_read(mfd, 0xA7);
			ra6 <<= 8;
			ra6 |= ra7;	// [0xA6 0xA7]
			dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);

		} while (ra6 < thr);
	}

	dev_info(pwr->dev, "[0xA6 0xA7] becomes [0x%.4X] . \n", thr);
	dev_info(pwr->dev, " Calibation finished ... \n\n");

	aft_ra2 = bd7181x_reg_read(mfd, 0xA2);	// 
	aft_ra3 = bd7181x_reg_read(mfd, 0xA3);	// I want to know initial A2 & A3.

	dev_info(pwr->dev, "TEST[A2,A3] = 0x%.2X%.2X\n", aft_ra2, aft_ra3);

	// bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);

	return 0;
}

static ssize_t bd7181x_sysfs_set_calibrate(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd7181x_power *pwr = container_of(psy, struct bd7181x_power, bat);
	ssize_t ret = 0;
	unsigned int val, mA;
	static u8 rA2;

	ret = sscanf(buf, "%d %d", &val, &mA);
	if (ret < 1) {
		dev_info(pwr->dev, "error: write a integer string");
		return count;
	}

	if (val == 1) {
		pwr->calib_current = CALIB_START;
		while (pwr->calib_current != CALIB_GO) {
			msleep(500);
		}
		rA2 = first_offset(pwr);
	}
	if (val == 2) {
		second_step(pwr, rA2);
	}
	if (val == 3) {
		if (ret <= 1) {
			dev_info(pwr->dev, "error: Fine adjust need a mA argument!");
		} else {
		unsigned int ra6_thr;

		ra6_thr = mA * 0xFFFF / 20000;
		dev_info(pwr->dev, "Fine adjust at %d mA, ra6 threshold %d(0x%X)\n", mA, ra6_thr, ra6_thr);
		third_step(pwr, ra6_thr);
		}
	}
	if (val == 4) {
		bd7181x_reg_write(pwr->mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);
		pwr->calib_current = CALIB_NORM;
		schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(0));
	}

	return count;
}

static ssize_t bd7181x_sysfs_show_calibrate(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	// struct power_supply *psy = dev_get_drvdata(dev);
	// struct bd7181x_power *pwr = container_of(psy, struct bd7181x_power, bat);
	ssize_t ret = 0;

	ret = 0;
	ret += sprintf(buf + ret, "write string value\n"
		"\t1      0 mA for step one\n"
		"\t2      1000 mA for rough adjust\n"
		"\t3 <mA> for fine adjust\n"
		"\t4      exit current calibration\n");
	return ret;
}

static DEVICE_ATTR(calibrate, S_IWUSR | S_IRUGO,
		bd7181x_sysfs_show_calibrate, bd7181x_sysfs_set_calibrate);
#endif

static struct attribute *bd7181x_sysfs_attributes[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use pwr supply class props.
	 */
	&dev_attr_registers.attr,
	#if USE_CALIB_CURRENT
	&dev_attr_calibrate.attr,
	#endif
	NULL,
};

static const struct attribute_group bd7181x_sysfs_attr_group = {
	.attrs = bd7181x_sysfs_attributes,
};

/** @brief powers supplied by bd7181x_ac */
static char *bd7181x_ac_supplied_to[] = {
	BAT_NAME,
};

/** @brief probe pwr device 
 * @param pdev platform deivce of bd7181x_power
 * @retval 0 success
 * @retval negative fail
 */
static int __init bd7181x_power_probe(struct platform_device *pdev)
{
	struct bd7181x *bd7181x = dev_get_drvdata(pdev->dev.parent);
	struct bd7181x_power *pwr;
	int irq, ret;

	pwr = kzalloc(sizeof(*pwr), GFP_KERNEL);
	if (pwr == NULL)
		return -ENOMEM;

	pwr->dev = &pdev->dev;
	pwr->mfd = bd7181x;

	platform_set_drvdata(pdev, pwr);

	if (battery_capacity <= 0) {
		battery_capacity = 1720;
	}
	dev_err(pwr->dev, "battery_capacity = %d\n", battery_capacity);

	/* If the product often power up/down and the power down time is long, the Coulomb Counter may have a drift. */
	/* If so, it may be better accuracy to enable Coulomb Counter using following commented out code */
	/* for counting Coulomb when the product is power up(including sleep). */
	/* The condition  */
	/* (1) Product often power up and down, the power down time is long and there is no power consumed in power down time. */
	/* (2) Kernel must call this routin at power up time. */
	/* (3) Kernel must call this routin at charging time. */
	/* (4) Must use this code with "Stop Coulomb Counter" code in bd7181x_power_remove() function */
	/* Start Coulomb Counter */
	/* bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB); */

	bd7181x_init_hardware(pwr);

	pwr->bat.name = BAT_NAME;
	pwr->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	pwr->bat.properties = bd7181x_battery_props;
	pwr->bat.num_properties = ARRAY_SIZE(bd7181x_battery_props);
	pwr->bat.get_property = bd7181x_battery_get_property;

	ret = power_supply_register(&pdev->dev, &pwr->bat);
	if (ret) {
		dev_err(&pdev->dev, "failed to register usb: %d\n", ret);
		goto fail_register_bat;
	}

	pwr->ac.name = AC_NAME;
	pwr->ac.type = POWER_SUPPLY_TYPE_MAINS;
	pwr->ac.properties = bd7181x_charger_props;
	pwr->ac.supplied_to = bd7181x_ac_supplied_to;
	pwr->ac.num_supplicants = ARRAY_SIZE(bd7181x_ac_supplied_to);
	pwr->ac.num_properties = ARRAY_SIZE(bd7181x_charger_props);
	pwr->ac.get_property = bd7181x_charger_get_property;

	ret = power_supply_register(&pdev->dev, &pwr->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ac: %d\n", ret);
		goto fail_register_ac;
	}

	/*Add DC_IN Inserted and Remove ISR */
	irq  = platform_get_irq(pdev, 0); // get irq number 
#ifdef __BD7181X_REGMAP_H__
	irq += bd7181x->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd7181x_power_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}


	/*add VBAT Low Voltage detection, John Zhang*/
	irq  = platform_get_irq(pdev, 1);
#ifdef __BD7181X_REGMAP_H__
	irq += bd7181x->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd7181x_vbat_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}



	ret = sysfs_create_group(&pwr->bat.dev->kobj, &bd7181x_sysfs_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register sysfs interface\n");
	}

	pwr->reg_index = -1;

	INIT_DELAYED_WORK(&pwr->bd_work, bd_work_callback);

	/* Schedule timer to check current status */
	#if USE_CALIB_CURRENT
	pwr->calib_current = CALIB_NORM;
	#endif
	schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(0));

	return 0;

      //error_exit:
	power_supply_unregister(&pwr->ac);
      fail_register_ac:
	power_supply_unregister(&pwr->bat);
      fail_register_bat:
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);

	return ret;
}

/** @brief remove pwr device
 * @param pdev platform deivce of bd7181x_power
 * @return 0
 */

static int __exit bd7181x_power_remove(struct platform_device *pdev)
{
	struct bd7181x_power *pwr = platform_get_drvdata(pdev);

	/* If the product often power up/down and the power down time is long, the Coulomb Counter may have a drift. */
	/* If so, it may be better accuracy to disable Coulomb Counter using following commented out code */
	/* for stopping counting Coulomb when the product is power down(without sleep). */
	/* The condition  */
	/* (1) Product often power up and down, the power down time is long and there is no power consumed in power down time. */
	/* (2) Kernel must call this routin at power down time. */
	/* (3) Must use this code with "Start Coulomb Counter" code in bd7181x_power_probe() function */
	/* Stop Coulomb Counter */
	/* bd7181x_clear_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB); */

	sysfs_remove_group(&pwr->bat.dev->kobj, &bd7181x_sysfs_attr_group);

	cancel_delayed_work(&pwr->bd_work);

	power_supply_unregister(&pwr->bat);
	power_supply_unregister(&pwr->ac);
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);

	return 0;
}

static struct platform_driver bd7181x_power_driver = {
	.driver = {
		   .name = "bd7181x-power",
		   .owner = THIS_MODULE,
		   },
	.remove = __exit_p(bd7181x_power_remove),
};



/** @brief module initialize function */
static int __init bd7181x_power_init(void)
{
	return platform_driver_probe(&bd7181x_power_driver, bd7181x_power_probe);
}

module_init(bd7181x_power_init);

/** @brief module deinitialize function */
static void __exit bd7181x_power_exit(void)
{
	platform_driver_unregister(&bd7181x_power_driver);
}

module_exit(bd7181x_power_exit);

module_param(battery_capacity, uint, 0644);
MODULE_PARM_DESC(battery_capacity, "battery capacity, unit mAh");

MODULE_AUTHOR("Tony Luo <luofc@embest-tech.com>");
MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_DESCRIPTION("BD71815/BD71817 Battery Charger Power driver");
MODULE_LICENSE("GPL");
