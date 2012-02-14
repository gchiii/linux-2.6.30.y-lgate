#!/bin/sh


#make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- menuconfig
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- INSTALL_MOD_PATH=/Projects/LocusEnergy/at91sam/2.6.30.y-mti-modules uImage modules modules_install
#cp arch/arm/boot/uImage /tftpboot/uImage-${USER}.bin
cp arch/arm/boot/uImage /tftpboot/uImage.bin
#cd ../my_lcd-2.6.30.y && sh build.sh 
cd ../SierraWireless && sh build.sh 
