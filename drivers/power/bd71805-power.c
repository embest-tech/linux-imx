/*
 * bd71805-pwr.c
 * @file ROHM BD71805MWV Charger driver
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
#include <linux/mfd/bd71805.h>

#define JITTER_DEFAULT		50		/* hope 50ms is enough */
#define JITTER_REPORT_CAP	10000		/* 30 seconds */
#define BD71805_BATTERY_CAP	(battery_capacity * 360 / 1000)	/* [mAh]/1000*360[A/10S] for New ALG */
#define MAX_VOLTAGE		4200000
#define MIN_VOLTAGE		2500000
#define MAX_CURRENT		1500
#define AC_NAME			"bd71805_ac"
#define BAT_NAME		"bd71805_bat"

static unsigned int battery_capacity;


static int ocv_table[] = {
	4300000,
	4200000,
	4128442,
	4064928,
	4007164,
	3950366,
	3900408,
	3850403,
	3806685,
	3765556,
	3712877,
	3678102,
	3655723,
	3638088,
	3622955,
	3608886,
	3590663,
	3564027,
	3522301,
	3473833,
	3443710,
	3330334
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
struct bd71805_power {
	struct device *dev;
	struct bd71805 *mfd;			/**< parent for access register */
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
	int	bat_health;			/**< battery health */
	int	full_cap;			/**< battery capacity */
	int	curr;				/**< battery current */
	int	curr_sar;			/**< battery current from DS-ADC */
	int	temp;				/**< battery tempature */
	u32	coulomb_cnt;			/**< Coulomb Counter */
	int	state_machine;			/**< initial-procedure state machine */
};

enum {
	STAT_POWER_ON,
	STAT_REBOOT,
	STAT_INITIALIZED,
};

/** @brief read a register group once
 *  @param mfd bd71805 device
 *  @param reg	 register address of lower register
 *  @return register value
 */
static u16 bd71805_reg_read16(struct bd71805* mfd, int reg) {
	u16 v;

	v = (u16)bd71805_reg_read(mfd, reg) << 8;
	v |= (u16)bd71805_reg_read(mfd, reg + 1) << 0;
	return v;
}

/** @brief write a register group once
 * @param mfd bd71805 device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd71805_reg_write16(struct bd71805 *mfd, int reg, u16 val) {
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	u.long_type = cpu_to_be16(val);
	// printk("write16 0x%.4X 0x%.4X\n", val, u.long_type);
	// r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
	r = mfd->write(mfd, reg, sizeof u.chars, u.chars);
	if (r) {
		return -1;
	}
	return 0;	
}

/** @brief read quad register once
 *  @param mfd bd71805 device
 *  @param reg	 register address of lower register
 *  @return register value
 */
static int bd71805_reg_read32(struct bd71805 *mfd, int reg) {
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

	// r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
	r = mfd->read(mfd, reg, sizeof u.chars, u.chars);
	if (r) {
		return -1;
	}
	return be32_to_cpu(u.long_type);
}

/** @brief write quad register once
 * @param mfd bd71805 device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd71805_reg_write32(struct bd71805 *mfd, int reg, unsigned val) {
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

	u.long_type = cpu_to_be32(val);
	// r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
	r = mfd->write(mfd, reg, sizeof u.chars, u.chars);
	if (r) {
		return -1;
	}
	return 0;
}

/** @brief get initial battery voltage and current
 * @param pwr power device
 * @return 0
 */
static int bd71805_get_init_bat_stat(struct bd71805_power *pwr) {
	struct bd71805 *mfd = pwr->mfd;
	int vcell;
	//int curr;

	vcell = bd71805_reg_read16(mfd, BD71805_REG_VM_VBATLOAD_PRE) * 1000;
	dev_info(pwr->dev, "VM_VBATLOAD_PRE = %d\n", vcell);
	pwr->hw_ocv1 = vcell;

	/* curr = bd71805_reg_read16(mfd, BD71805_REG_VM_IBATLOAD_PRE);
	if (bd71805_reg_read(mfd, BD71805_REG_CC_CURCD) & CURDIR_Discharging) {
		curr = -curr;
	}
	dev_info(pwr->dev, "VM_IBATLOAD_PRE = %d\n", curr);
	*/

	vcell = bd71805_reg_read16(mfd, BD71805_REG_VM_VBATLOAD_PST) * 1000;
	dev_info(pwr->dev, "VM_VBATLOAD_PST = %d\n", vcell);
	pwr->hw_ocv2 = vcell;

	/* curr = bd71805_reg_read16(mfd, BD71805_REG_VM_IBATLOAD_PST);
	if (bd71805_reg_read(mfd, BD71805_REG_CC_CURCD) & CURDIR_Discharging) {
		curr = -curr;
	}
	dev_info(pwr->dev, "VM_IBATLOAD_PST = %d\n", curr);
	*/

	if (pwr->hw_ocv1 == 0 && pwr->hw_ocv2 == 0) {
		pwr->hw_ocv1 = bd71805_reg_read16(mfd, BD71805_REG_VM_VBAT_U) * 1000;
	}

	return 0;
}

/** @brief get battery average voltage and current
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @param curr  pointer to return back current in unit uA.
 * @return 0
 */
static int bd71805_get_vbat_curr(struct bd71805_power *pwr, int *vcell, int *curr) {
	struct bd71805* mfd = pwr->mfd;
	int tmp_vcell, tmp_curr, tmp_curr2, i;

	tmp_vcell = 0;
	tmp_curr = 0;
	for (i = 0; i < 10; i++) {
		tmp_vcell += bd71805_reg_read16(mfd, BD71805_REG_VM_VBAT_U);
		tmp_curr2 = bd71805_reg_read16(mfd, BD71805_REG_VM_IBAT_U);
		if (bd71805_reg_read(mfd, BD71805_REG_CC_CURCD) & CURDIR_Discharging) {
			tmp_curr2 = -tmp_curr2;
		}
		tmp_curr += tmp_curr2;
	}
	tmp_vcell = tmp_vcell / 10;
	tmp_curr = tmp_curr / 10;
	*vcell = tmp_vcell * 1000;
	*curr = tmp_curr * 1000;

	return 0;
}

/** @brief get battery current from DS-ADC
 * @param pwr power device
 * @return current in unit uA
 */
static int bd71805_get_current_ds_adc(struct bd71805_power *pwr) {
	int r;
	
	r = 1000 * (bd71805_reg_read16(pwr->mfd, BD71805_REG_CC_CURCD) & ~0x8000);
	if (bd71805_reg_read(pwr->mfd, BD71805_REG_CC_CURCD) & CURDIR_Discharging) {
		r = -r;
	}
	return r;
}

/** @brief get battery capacity
 * @param ocv open circuit voltage
 * @return capcity in unit 0.1 percent
 */
static int bd71805_voltage_to_capacity(int ocv) {
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
static int bd71805_get_temp(struct bd71805_power *pwr) {
	struct bd71805* mfd = pwr->mfd;
	int t;

	t = bd71805_reg_read(mfd, BD71805_REG_VM_BTMP) - 55;

	// battery temperature error
	t = (t > 150)? 25: t;
	
	return t;
}

/** @brief get battery charge status
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd71805_charge_status(struct bd71805_power *pwr)
{
	u8 state;
	int ret = 1;

	state = bd71805_reg_read(pwr->mfd, BD71805_REG_CHG_STATE);
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

	return ret;
}

/** @brief set initial coulomb counter value from battery voltage
 * @param pwr power device
 * @return 0
 */
static int calibration_coulomb_counter(struct bd71805_power* pwr) {
	u32 bcap;
	int soc, ocv;

	/* Get init OCV by HW */
	bd71805_get_init_bat_stat(pwr);

	pwr->state_machine = STAT_REBOOT;

	ocv = (pwr->hw_ocv1 >= pwr->hw_ocv2)? pwr->hw_ocv1: pwr->hw_ocv2;
	dev_info(pwr->dev, "ocv %d\n", ocv);

	/* Get init soc from ocv/soc table */
	soc = bd71805_voltage_to_capacity(ocv);
	dev_info(pwr->dev, "soc %d\n", soc);

	bcap = pwr->full_cap * soc / 1000;

	bd71805_reg_write16(pwr->mfd, BD71805_REG_CC_CCNTD_3, ((bcap + bcap / 200) & 0x1FFFUL));

	pwr->coulomb_cnt = bd71805_reg_read32(pwr->mfd, BD71805_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
	dev_info(pwr->dev, "CC_CCNTD = %d\n", pwr->coulomb_cnt);

	return 0;
}

/** @brief get battery parameters, such as voltages, currents, temperatures.
 * @param pwr power device
 * @return 0
 */
static int bd71805_get_vcell(struct bd71805_power* pwr)
{
	int curr, curr_sar;

	/* Read detailed vcell and current */
	bd71805_get_vbat_curr(pwr, &pwr->vcell, &curr_sar);
	// dev_info(pwr->dev, "VM_VBAT = %d\n", pwr->vcell / 1000);
	// dev_info(pwr->dev, "VM_IBAT = %d\n", curr_sar / 1000);

	curr = bd71805_get_current_ds_adc(pwr);
	// dev_info(pwr->dev, "CC_CURCD = %d\n", curr / 1000);

	pwr->curr = curr;
	pwr->curr_sar = curr_sar;

	/* Get tempature */
	pwr->temp = bd71805_get_temp(pwr);
	// dev_info(pwr->dev, "Temperature %d degrees C\n", pwr->temp);

	if (!bd71805_charge_status(pwr) && (pwr->state_machine == STAT_POWER_ON)) {
		// 
	}
	return 0;
}

/** @brief get coulomb counter values
 * @param pwr power device
 * @return 0
 */
static int bd71805_coulomb_count(struct bd71805_power* pwr) {
	if (pwr->state_machine == STAT_REBOOT) {
		pwr->state_machine = STAT_INITIALIZED;
		bd71805_set_bits(pwr->mfd, BD71805_REG_CC_CCNTD_3, CCNTENB);
	} else if (pwr->state_machine == STAT_INITIALIZED) {
		pwr->coulomb_cnt = bd71805_reg_read32(pwr->mfd, BD71805_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
		// dev_info(pwr->dev, "CC_CCNTD = %d\n", pwr->coulomb_cnt);
	}
	return 0;
}

/** @brief get battery and DC online status
 * @param pwr power device
 * @return 0
 */
static int bd71805_get_online(struct bd71805_power* pwr) {
	int r;

#define TS_THRESHOLD_VOLT	0xD9
	r = bd71805_reg_read(pwr->mfd, BD71805_REG_VM_VTH);
	pwr->bat_online = (r > TS_THRESHOLD_VOLT);
	
	r = bd71805_reg_read(pwr->mfd, BD71805_REG_VBUS_STAT);
	pwr->charger_online = (r & VBUS_DET) != 0;
	return 0;
}

/** @brief init bd71805 sub module charger
 * @param pwr power device
 * @return 0
 */
static int bd71805_init_hardware(struct bd71805_power *pwr) {
	struct bd71805 *mfd = pwr->mfd;
	//u32 coul;
	int r;

#define TEST_SEQ_00		0x00
#define TEST_SEQ_01		0x76
#define TEST_SEQ_02		0x66
#define TEST_SEQ_03		0x56
	bd71805_reg_write(mfd, BD71805_REG_TEST_MODE, TEST_SEQ_01);
	bd71805_reg_write(mfd, BD71805_REG_TEST_MODE, TEST_SEQ_02);
	bd71805_reg_write(mfd, BD71805_REG_TEST_MODE, TEST_SEQ_03);
	r = bd71805_reg_read(mfd, BD71805_REG_VSYS_MAX);
	bd71805_reg_write(mfd, BD71805_REG_TEST_MODE, TEST_SEQ_00);
	if ((r & 0x01) == 0x00) {
	//coul = bd71805_reg_read32(pwr->mfd, BD71805_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
	//if (coul == 0x0UL) {
		/* Init HW, when the battery is inserted. */
		bd71805_reg_write(mfd, BD71805_REG_TEST_MODE, TEST_SEQ_01);
		bd71805_reg_write(mfd, BD71805_REG_TEST_MODE, TEST_SEQ_02);
		bd71805_reg_write(mfd, BD71805_REG_TEST_MODE, TEST_SEQ_03);
		bd71805_reg_write(mfd, BD71805_REG_VSYS_MAX, r | 0x01);
		bd71805_reg_write(mfd, BD71805_REG_TEST_MODE, TEST_SEQ_00);

		/* Reset Coulomb Counter */
		bd71805_reg_write(mfd, BD71805_REG_CC_CCNTD_3, CCNTRST);

		/* Set default Coulomb Counter */
		bd71805_reg_write16(mfd, BD71805_REG_CC_CCNTD_3, BD71805_BATTERY_CAP & 0x1FFF);
	
		/* Set default Battery Capacity */
		pwr->full_cap = BD71805_BATTERY_CAP;

		/* WDT_FST manual set */
		bd71805_set_bits(mfd, BD71805_REG_CHG_SET1, WDT_AUTO);
		// Watch Dog Timer 480 minutes */
		bd71805_reg_write(mfd, BD71805_REG_CHG_WDT_FST, 0x38);
		
		pwr->state_machine = STAT_POWER_ON;
	} else {
		pwr->full_cap = BD71805_BATTERY_CAP;	// bd71805_reg_read16(pwr->mfd, BD71805_REG_CC_BATCAP_U);
		pwr->state_machine = STAT_POWER_ON;	// STAT_REBOOT
	}

	calibration_coulomb_counter(pwr);

	pwr->coulomb_cnt = bd71805_reg_read32(mfd, BD71805_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
	dev_info(pwr->dev, "CC_CCNTD = %d\n", pwr->coulomb_cnt);

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
	struct bd71805_power *pwr;
	struct delayed_work *delayed_work;
	int status, changed = 0;
	static int cap_counter = 0;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd71805_power, bd_work);

	status = bd71805_reg_read(pwr->mfd, BD71805_REG_VBUS_STAT);
	if (status != pwr->vbus_status) {
		//printk("VBUS_STAT CHANGED from 0x%X to 0x%X\n", pwr->vbus_status, status);
		pwr->vbus_status = status;
		changed = 1;
	}

	status = bd71805_reg_read(pwr->mfd, BD71805_REG_BAT_STAT);
	status &= ~BAT_DET_DONE;
	if (status != pwr->bat_status) {
		//printk("BAT_STAT CHANGED from 0x%X to 0x%X\n", pwr->bat_status, status);
		pwr->bat_status = status;
		changed = 1;
	}

	status = bd71805_reg_read(pwr->mfd, BD71805_REG_CHG_STATE);
	if (status != pwr->charge_status) {
		//printk("CHG_STATE CHANGED from 0x%X to 0x%X\n", pwr->charge_status, status);
		pwr->charge_status = status;
		//changed = 1;
	}

	bd71805_get_vcell(pwr);
	bd71805_coulomb_count(pwr);
	bd71805_get_online(pwr);
	bd71805_charge_status(pwr);

	if (changed || cap_counter++ > JITTER_REPORT_CAP / JITTER_DEFAULT) {
		power_supply_changed(&pwr->ac);
		power_supply_changed(&pwr->bat);
		cap_counter = 0;
	}

	schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(JITTER_DEFAULT));
}

/** @brief get property of power supply ac
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */
static int bd71805_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd71805_power *pwr = dev_get_drvdata(psy->dev->parent);
	u32 vot;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->charger_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd71805_reg_read16(pwr->mfd, BD71805_REG_VM_VBUS_U);
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

static int bd71805_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd71805_power *pwr = dev_get_drvdata(psy->dev->parent);
	// u32 cap, vot, r;
	// u8 ret;

	switch (psp) {
	/*
	case POWER_SUPPLY_PROP_STATUS:
		r = bd71805_reg_read(pwr->mfd, BD71805_REG_CHG_STATE);
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
		ret = bd71805_reg_read(pwr->mfd, BD71805_REG_BAT_STAT);
		if (ret & DBAT_DET)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else if (ret & VBAT_OV)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		cap = bd71805_reg_read16(pwr->mfd, BD71805_REG_CC_BATCAP_U);
		// printk("CC_BATCAP = 0x%.4X\n", cap);
		val->intval = cap * 100 / 0x1FFF;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd71805_reg_read16(pwr->mfd, BD71805_REG_VM_VBAT_U) * 1000;
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
		val->intval = (pwr->coulomb_cnt >> 16) * 100 /  pwr->full_cap;
		if (val->intval > 100) {
			val->intval = 100;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = pwr->coulomb_cnt;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = battery_capacity * 1000;
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
static enum power_supply_property bd71805_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/** @brief bat properies */
static enum power_supply_property bd71805_battery_props[] = {
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
static ssize_t bd71805_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71805_power *pwr = container_of(psy, struct bd71805_power, bat);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		pwr->reg_index = -1;
		return count;
	}

	if (ret == 1 && reg <= BD71805_MAX_REGISTER) {
		pwr->reg_index = reg;
		return count;
	}
	if (reg > BD71805_MAX_REGISTER || val > 255)
		return -EINVAL;

	ret = bd71805_reg_write(pwr->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/** @brief print value of chip register, format: 'register=value' */
static ssize_t bd71805_sysfs_print_reg(struct bd71805_power *pwr,
				       u8 reg,
				       char *buf)
{
	int ret = bd71805_reg_read(pwr->mfd, reg);

	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/** @brief show all raw values of chip register, format per line: 'register=value' */
static ssize_t bd71805_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd71805_power *pwr = container_of(psy, struct bd71805_power, bat);
	ssize_t ret = 0;
	int i;

	if (pwr->reg_index > 0) {
		ret += bd71805_sysfs_print_reg(pwr, pwr->reg_index, buf + ret);
	} else {
		for (i = 0; i <= BD71805_MAX_REGISTER; i++) {
			ret += bd71805_sysfs_print_reg(pwr, i, buf + ret);
		}
	}
	return ret;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bd71805_sysfs_show_registers, bd71805_sysfs_set_registers);

static struct attribute *bd71805_sysfs_attributes[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use pwr supply class props.
	 */
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bd71805_sysfs_attr_group = {
	.attrs = bd71805_sysfs_attributes,
};

/** @brief powers supplied by bd71805_ac */
static char *bd71805_ac_supplied_to[] = {
	BAT_NAME,
};

/** @brief probe pwr device 
 * @param pdev platform deivce of bd71805_power
 * @retval 0 success
 * @retval negative fail
 */
static int __init bd71805_power_probe(struct platform_device *pdev)
{
	struct bd71805 *bd71805 = dev_get_drvdata(pdev->dev.parent);
	struct bd71805_power *pwr;
	int ret;

	pwr = kzalloc(sizeof(*pwr), GFP_KERNEL);
	if (pwr == NULL)
		return -ENOMEM;

	pwr->dev = &pdev->dev;
	pwr->mfd = bd71805;

	platform_set_drvdata(pdev, pwr);

	if (battery_capacity <= 0) {
		battery_capacity = 1500;
	}

	bd71805_init_hardware(pwr);

	pwr->bat.name = BAT_NAME;
	pwr->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	pwr->bat.properties = bd71805_battery_props;
	pwr->bat.num_properties = ARRAY_SIZE(bd71805_battery_props);
	pwr->bat.get_property = bd71805_battery_get_property;

	ret = power_supply_register(&pdev->dev, &pwr->bat);
	if (ret) {
		dev_err(&pdev->dev, "failed to register usb: %d\n", ret);
		goto fail_register_bat;
	}

	pwr->ac.name = AC_NAME;
	pwr->ac.type = POWER_SUPPLY_TYPE_MAINS;
	pwr->ac.properties = bd71805_charger_props;
	pwr->ac.supplied_to = bd71805_ac_supplied_to;
	pwr->ac.num_supplicants = ARRAY_SIZE(bd71805_ac_supplied_to);
	pwr->ac.num_properties = ARRAY_SIZE(bd71805_charger_props);
	pwr->ac.get_property = bd71805_charger_get_property;

	ret = power_supply_register(&pdev->dev, &pwr->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ac: %d\n", ret);
		goto fail_register_ac;
	}

	ret = sysfs_create_group(&pwr->bat.dev->kobj, &bd71805_sysfs_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register sysfs interface\n");
	}

	pwr->reg_index = -1;

	INIT_DELAYED_WORK(&pwr->bd_work, bd_work_callback);

	/* Schedule timer to check current status */
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
 * @param pdev platform deivce of bd71805_power
 * @return 0
 */

static int __exit bd71805_power_remove(struct platform_device *pdev)
{
	struct bd71805_power *pwr = platform_get_drvdata(pdev);

	sysfs_remove_group(&pwr->bat.dev->kobj, &bd71805_sysfs_attr_group);

	cancel_delayed_work(&pwr->bd_work);

	power_supply_unregister(&pwr->bat);
	power_supply_unregister(&pwr->ac);
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);

	return 0;
}

static struct platform_driver bd71805_power_driver = {
	.driver = {
		   .name = "bd71805-power",
		   .owner = THIS_MODULE,
		   },
	.remove = __exit_p(bd71805_power_remove),
};



/** @brief module initialize function */
static int __init bd71805_power_init(void)
{
	return platform_driver_probe(&bd71805_power_driver, bd71805_power_probe);
}

module_init(bd71805_power_init);

/** @brief module deinitialize function */
static void __exit bd71805_power_exit(void)
{
	platform_driver_unregister(&bd71805_power_driver);
}

module_exit(bd71805_power_exit);

module_param(battery_capacity, uint, 0644);
MODULE_PARM_DESC(battery_capacity, "battery capacity, unit mAh");

MODULE_AUTHOR("Tony Luo <luofc@embest-tech.com>");
MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_DESCRIPTION("BD71805MWV Battery Charger Power driver");
MODULE_LICENSE("GPL");
