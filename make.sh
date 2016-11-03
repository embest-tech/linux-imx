export PATH=/home/chengpg/bin/arm/fsl-linaro-toolchain/bin:$PATH


export ARCH=arm

if ! [ -f ".config" ]; then
    make  imx_v7_defconfig
    [ $? != 0 ] && exit 1
fi

make uImage LOADADDR=0x10008000 -j5
[ $? != 0 ] && exit 1

make imx6solo_RIoTboard.dtb
[ $? != 0 ] && exit 1

cp -f arch/arm/boot/uImage ~/tftp/
cp -f arch/arm/boot/dts/imx6solo_RIoTboard.dtb ~/tftp/
