#!/bin/sh

DESTDIR="/home/android/cpg"

export CROSS_COMPILE=/home/android/git/android-imx6/android-imx6-kk4.4.2-1.0.0/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-
export ARCH=arm

build_RIoTboard_linux() {
    if ! [ -f ".config" ]; then
        make  imx_v7_defconfig
        [ $? != 0 ] && exit 1
    fi

    make uImage LOADADDR=0x10008000 -j5
    [ $? != 0 ] && exit 1

    make imx6solo_RIoTboard.dtb
    [ $? != 0 ] && exit 1

    cp -f arch/arm/boot/uImage ${DESTDIR}/
    cp -f arch/arm/boot/dts/imx6solo_RIoTboard.dtb ${DESTDIR}/
}


build_SBC9000_linux() {
    if ! [ -f ".config" ]; then
        make imx_v7_sbc9000_defconfig
        [ $? != 0 ] && exit 1
    fi

    make imx6q-sbc9000.dtb uImage LOADADDR=0x10008000
    [ $? != 0 ] && exit 1

    cp -f arch/arm/boot/dts/imx6q-sbc9000.dtb ${DESTDIR}/imx6q-sbc9000.dtb
    cp -f arch/arm/boot/uImage ${DESTDIR}/uImage
}


# build_RIoTboard_linux
build_SBC9000_linux
