#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

## ts config
obj-$(CONFIG_TOUCHSCREEN_GENERIC_TS)	+= generic_ts.o

## mmgr config
generic_ts-y	+= mmgr.o

## core config
generic_ts-y	+= touch.o

generic_ts-y	+= fsutils.o

## platform config
generic_ts-y	+= platform.o

## chip config
# debug version
#generic_ts-y	+= chip.o
# release version
generic_ts-y	+= lib/chip.x

# ct360
# debug version
#generic_ts-y	+= ct360.o
# release version
generic_ts-y	+= lib/ct360.x

# ct362/ct363/365
# debug version
#generic_ts-y	+= ct365.o
# release version
generic_ts-y	+= lib/ct365.x

## Enhance config
# debug version
#generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_ENHANCE)	+= enhance.o
# release version
generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_ENHANCE)	+= lib/enhance.x


## interface config
# I2C
generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_INTERFACE_I2C)	+= core_i2c.o
## platform config
# Abstract
generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_PLATFORM_ABS)	+= plat_i2c.o
# RockChip
generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_PLATFORM_RKXX)	+= plat_i2c_aio_rk.o
# AllWinner
generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_PLATFORM_A1X)	+= plat_i2c_aio_aw_a1x.o
# Amlogic
generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_PLATFORM_AML)	+= plat_i2c_aio_aml.o

## interface config
# USB
generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_INTERFACE_USB)	+= core_usb.o
## platform config
generic_ts-$(CONFIG_TOUCHSCREEN_GENERIC_TS_INTERFACE_USB)	+= plat_usb.o

## Misc

