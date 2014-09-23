/**
 * @file bd71805.h  ROHM BD71805MWV header file
 *
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * @author yanglsh@embest-tech.com
 */

#ifndef __LINUX_MFD_BD71805_H
#define __LINUX_MFD_BD71805_H

#include <linux/regmap.h>

#define BD71805_BUCK1               0
#define BD71805_BUCK2               1
#define BD71805_BUCK3               2
#define BD71805_BUCK4               3
#define BD71805_LDO1                4
#define BD71805_LDO2                5
#define BD71805_LDO3                6
#define BD71805_VODVREF             7
#define BD71805_VOSNVS              8
#define BD71805_NUM_REGULATOR       9

#define BD71805_SUPPLY_STATE_ENABLED    0x1

#define BD71805_REG_DEVICE		0x00
#define BD71805_REG_PWRCTRL		0x01
#define BD71805_REG_BUCK1_ON		0x02
#define BD71805_REG_BUCK1_STBY		0x03
#define BD71805_REG_BUCK1_SLP      	0x04
#define BD71805_REG_BUCK1_MODE		0x05
#define BD71805_REG_BUCK1_CONF		0x06
#define BD71805_REG_BUCK2_ON		0x07
#define BD71805_REG_BUCK2_STBY		0x08
#define BD71805_REG_BUCK2_SLP		0x09
#define BD71805_REG_BUCK2_MODE		0x0A
#define BD71805_REG_BUCK2_CONF		0x0B
#define BD71805_REG_BUCK3_VOLT		0x0C
#define BD71805_REG_BUCK3_MODE		0x0D
#define BD71805_REG_BUCK4_VOLT		0x0E
#define BD71805_REG_BUCK4_MODE		0x0F
#define BD71805_REG_LDO1_CTRL		0x10
#define BD71805_REG_LDO2_CTRL		0x11
#define BD71805_REG_LDO1_VOLT		0x12
#define BD71805_REG_LDO2_VOLT		0x13
#define BD71805_REG_LDO3_VOLT		0x14
#define BD71805_REG_BUCK_PDDIS		0x15
#define BD71805_REG_LDO_PDDIS		0x16
#define BD71805_REG_CHG_STATE		0x34
#define BD71805_REG_BAT_STAT		0x36
#define BD71805_REG_VBUS_STAT		0x37
#define BD71805_REG_BAT_TEMP		0x3B
#define BD71805_REG_CHG_SET1		0x42
#define BD71805_REG_CHG_SET2		0x43
#define BD71805_REG_VM_CUR_U		0x56
#define BD71805_REG_VM_CUR_L		0x57
#define BD71805_REG_VM_VBAT_U		0x58
#define BD71805_REG_VM_VBAT_L		0x59
#define BD71805_REG_VM_BTMP		0x5A
#define BD71805_REG_VM_VBUS_U		0x5C
#define BD71805_REG_VM_VBUS_L		0x5D
#define BD71805_REG_VM_VSYS		0x5E
#define BD71805_REG_CC_BATCAP_U		0x6C
#define BD71805_REG_CC_BATCAP_L		0x6D
#define BD71805_REG_CC_STAT		0x74
#define BD71805_REG_CC_CCNTD_3		0x75
#define BD71805_REG_CC_CCNTD_2		0x76
#define BD71805_REG_CC_CCNTD_1		0x77
#define BD71805_REG_CC_CCNTD_0		0x78
#define BD71805_MAX_REGISTER		0xA0

/* FIXME */
#define LDO3_EN					0x01
#define LDO2_EN					0x02
#define LDO1_EN					0x04
#define VOLT_MASK				0x3F

#define BAT_DET_DONE				0x10
#define BAT_DET					0x20
#define BAT_DET_OFFSET				5
#define DBAT_DET				0x01
#define VBAT_OV					0x08

#define VBUS_DET				0x01

#define BUCK1_RAMPRATE_10MV_US			0x0
#define BUCK1_RAMPRATE_5MV_US			0x1
#define BUCK1_RAMPRATE_2P5MV_US			0x2
#define BUCK1_RAMPRATE_1P25MV_US		0x3

/** @brief charge state enumuration */
enum CHG_STATE {
	CHG_STATE_SUSPEND = 0x0,		/**< suspend state */
	CHG_STATE_TRICKLE_CHARGE,		/**< trickle charge state */
	CHG_STATE_PRE_CHARGE,			/**< precharge state */
	CHG_STATE_FAST_CHARGE,			/**< fast charge state */
	CHG_STATE_TOP_OFF,			/**< top off state */
	CHG_STATE_DONE,				/**< charge complete */
};

struct bd71805;

/**
 * @brief Board platform data may be used to initialize regulators.
 */

struct bd71805_board {
	struct regulator_init_data *bd71805_pmic_init_data[BD71805_NUM_REGULATOR];
	/**< regulator initialize data */
};

/**
 * @brief bd71805 sub-driver chip access routines
 */

struct bd71805 {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct mutex io_mutex;
	unsigned int id;

	/* Client devices */
	struct bd71805_pmic *pmic;	/**< client device regulator */
	struct bd71805_power *power;	/**< client device battery */

	struct bd71805_board *of_plat_data;
	/**< Device node parsed board data */
};

static inline int bd71805_chip_id(struct bd71805 *bd71805)
{
	return bd71805->id;
}


/**
 * @brief bd71805_reg_read
 * read single register's value of bd71805
 * @param bd71805 device to read
 * @param reg register address
 * @return register value if success
 *         error number if fail
 */
static inline int bd71805_reg_read(struct bd71805 *bd71805, u8 reg)
{
	int r, val;

	r = regmap_read(bd71805->regmap, reg, &val);
	if (r < 0) {
		return r;
	}
	return val;
}

/**
 * @brief bd71805_reg_write
 * write single register of bd71805
 * @param bd71805 device to write
 * @param reg register address
 * @param val value to write
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int bd71805_reg_write(struct bd71805 *bd71805, u8 reg,
		unsigned int val)
{
	return regmap_write(bd71805->regmap, reg, val);
}

/**
 * @brief bd71805_set_bits
 * set bits in one register of bd71805
 * @param bd71805 device to read
 * @param reg register address
 * @param mask mask bits
 * @retval 0 if success
 * @retval negative error number if fail
 */
static inline int bd71805_set_bits(struct bd71805 *bd71805, u8 reg,
		u8 mask)
{
	return regmap_update_bits(bd71805->regmap, reg, mask, mask);
}

/**
 * @brief bd71805_clear_bits
 * clear bits in one register of bd71805
 * @param bd71805 device to read
 * @param reg register address
 * @param mask mask bits
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int bd71805_clear_bits(struct bd71805 *bd71805, u8 reg,
		u8 mask)
{
	return regmap_update_bits(bd71805->regmap, reg, mask, 0);
}

/**
 * @brief bd71805_update_bits
 * update bits in one register of bd71805
 * @param bd71805 device to read
 * @param reg register address
 * @param mask mask bits
 * @param val value to update
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int bd71805_update_bits(struct bd71805 *bd71805, u8 reg,
					   u8 mask, u8 val)
{
	return regmap_update_bits(bd71805->regmap, reg, mask, val);
}

#endif /* __LINUX_MFD_BD71805_H */
