#!/bin/sh


#make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- menuconfig

make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- INSTALL_MOD_PATH=/Projects/LocusEnergy/at91sam/modules/ uImage 
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- INSTALL_MOD_PATH=/Projects/LocusEnergy/at91sam/modules/ modules modules_install
cp arch/arm/boot/uImage /tftpboot/uImage.bin

cd ../my_lcd && sh build.sh 
