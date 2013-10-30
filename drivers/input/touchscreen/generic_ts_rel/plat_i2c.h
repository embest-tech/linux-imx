#ifndef PLAT_I2C_H
#define PLAT_I2C_H


// ****************************************************************************
// Includes
// ****************************************************************************
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

// ****************************************************************************
// Defines
// ****************************************************************************
/* I2C config */
#define PLATFORM_I2C_BUS			2	// I2C Bus
#define PLATFORM_I2C_ADDR			0x01
#define PLATFORM_I2C_SPEED			400000

/* rk30xx config */
#define PLATFORM_RK_PWR_PIN			0//
#define PLATFORM_RK_IRQ_PIN			RK30_PIN4_PC2//
#define PLATFORM_RK_RST_PIN			RK30_PIN4_PD0//
/* rk29xx config */
//#define PLATFORM_RK_PWR_PIN			0//
//#define PLATFORM_RK_RST_PIN			RK29_PIN6_PC3
//#define PLATFORM_RK_IRQ_PIN			RK29_PIN0_PA2
/* a1x config */
#define PLATFORM_AW_PIO_BASE_ADDR		(0x01C20800)
#define PLATFORM_AW_PIO_RANGE_SIZE		(0x400)

#define PLATFORM_AW_PIO_INT_STAT_OFFSET		(0x214)
#define PLATFORM_AW_PIO_INT_CTRL_OFFSET		(0x210)
#define PLATFORM_AW_PIO_INT_CFG0_OFFSET		(0x200)
#define PLATFORM_AW_PIO_INT_CFG1_OFFSET		(0x204)
#define PLATFORM_AW_PIO_INT_CFG2_OFFSET		(0x208)
#define PLATFORM_AW_PIO_INT_CFG3_OFFSET		(0x20C)

#define PLATFORM_AW_CTP_PARA_STR		"ctp_para"
#define PLATFORM_AW_PWR_PIN_STR		NULL
#define PLATFORM_AW_RST_PIN_STR		"ctp_reset"
#define PLATFORM_AW_IRQ_PIN_STR		"ctp_int_port"

// CONFIG_ARCH_SUN4I
#define PLATFORM_AW_CTP_IRQ_NUM			(21)	// EINT_21
// CONFIG_ARCH_SUN5I
#define PLATFORM_AW_CTP_IRQ_NUM			(9)	// EINT_9

#define PLATFORM_AW_IRQ_MOD_POS_EDG		(0x00)
#define PLATFORM_AW_IRQ_MOD_NEG_EDG		(0x01)
#define PLATFORM_AW_IRQ_MOD_HI_LVL		(0x02)
#define PLATFORM_AW_IRQ_MOD_LO_LVL		(0x03)
#define PLATFORM_AW_IRQ_MOD_DBL_EDG		(0x04)

#define PLATFORM_AW_CTP_IRQ_MOD			PLATFORM_AW_IRQ_MOD_NEG_EDG
/* aml config */
#define PLATFORM_AML_PWR_PIN			0//
#define PLATFORM_AML_RST_PIN			PAD_GPIOC_3
#define PLATFORM_AML_IRQ_PIN			PAD_GPIOA_16
#define PLATFORM_AML_IRQ_NUM			INT_GPIO_0

/* sabrelite config */
#define PLATFORM_SABRELITE_PWR_PIN                    0//
#define PLATFORM_SABRELITE_RST_PIN                    IMX_GPIO_NR(2, 0)
#define PLATFORM_SABRELITE_IRQ_PIN                    IMX_GPIO_NR(1, 9)

/* riot board config */
#define PLATFORM_RIOT_PWR_PIN                    0//
#define PLATFORM_RIOT_RST_PIN                    IMX_GPIO_NR(1, 29)
#define PLATFORM_RIOT_IRQ_PIN                    IMX_GPIO_NR(6, 14)

// ****************************************************************************
// Types
// ****************************************************************************
// Platform i2c data
struct struct_platform_i2c_var;
struct struct_platform_i2c_ops {
	int (*i2c_read)(struct i2c_client *client, unsigned short addr, char *buf, int len);
	int (*i2c_write)(struct i2c_client *client, unsigned short addr, char *buf, int len);
	void (*get_cfg)(struct struct_platform_i2c_var *platform_i2c_data);
	int (*set_dev)(struct struct_platform_i2c_var *platform_i2c_data);
	int (*get_rsrc)(struct struct_platform_i2c_var *platform_i2c_data);
	void (*put_rsrc)(struct struct_platform_i2c_var *platform_i2c_data);
	void (*hw_reset)(struct struct_platform_i2c_var *platform_i2c_data);
	int (*chk_irq)(struct struct_platform_i2c_var *platform_i2c_data);
};

struct struct_platform_i2c_var {
	/* Hardware IO settings */
	void				* __iomem gpio_addr;
	int 				pwr;
	int 				rst;
	int 				ss;
	int 				irq;

	/* Communication settings */
	int				i2c_bus;
	int				i2c_addr;
	struct i2c_client		*client;

	/* Platform ops */
	struct struct_platform_i2c_ops	ops;
};

struct struct_platform_i2c_var* platform_i2c_var_init(void);
void platform_i2c_var_exit(struct struct_platform_i2c_var *platform_i2c_data);

#endif
