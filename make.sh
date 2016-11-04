
export PATH=/home/chengpg/bin/arm/fsl-linaro-toolchain/bin:$PATH 

export ARCH=arm
export CROSS_COMPILE=arm-linux-

if ! [ -f ".config" ]; then
    # make imx6_updater_defconfig
    make imx6_defconfig
    [ $? != 0 ] && exit 1
fi

make uImage -j5
[ $? != 0 ] && exit 1

cp -f arch/arm/boot/uImage ~/tftp/
