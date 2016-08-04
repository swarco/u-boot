#!/bin/sh

CPU=arm
CROSS_COMPILE="$CPU-linux-"
#BOARD=at91rm9200dk
BOARD=ccm2200
IMAGE_DIR=../../tftp_root/

#TOOLCHAIN=$PWD/../../toolchain-uboot-fsforth/
#TOOLCHAIN=$PWD/../../buildroot/buildroot-1.0-soft-float/build_${CPU}_nofpu/staging_dir/
TOOLCHAIN=$PWD/../../buildroot/buildroot-2.0-soft-float/build_${CPU}/staging_dir/usr

PATH=$PATH:$TOOLCHAIN/bin
export PATH

if [ "$1" == "makeall" ]; then
     export CROSS_COMPILE
#     ./MAKEALL ARM9 
     ./MAKEALL at91
     exit 0
fi


if ! grep $BOARD include/config.h >/dev/null; then
  make ${BOARD}_config CROSS_COMPILE=$CROSS_COMPILE
fi

if [ "$1" == "" ]; then
set all
fi

make CROSS_COMPILE=$CROSS_COMPILE ARCH=arm  $@ || exit $?

if [ \( "$1" != "clean" \) -a \( "$1" != "distclean" \) ]; then
  # build u-boot update scripts

  echo coping u-boot.bin in tftp download directory
  cp u-boot.bin $IMAGE_DIR/u-boot-ccm2200dev.bin

  test -d $IMAGE_DIR/u-boot-update || mkdir $IMAGE_DIR/u-boot-update
  cp u-boot.bin $IMAGE_DIR/u-boot-update/u-boot-ccm2200dev.bin

  MKIMAGE=tools/mkimage
  $MKIMAGE -A arm -O linux -T script -C none -a 0x20100000 -e 0x20100000 \
           -n "SWARCO CCM2200 Linux update"                              \
           -d board/swarco/ccm2200/scripts/linux-update.script           \
           $IMAGE_DIR/ccm2200.bin

  $MKIMAGE -A arm -O linux -T script -C none -a 0x20100000 -e 0x20100000 \
           -n "SWARCO CCM2200 U-Boot update"                             \
           -d board/swarco/ccm2200/scripts/u-boot-update.script          \
           $IMAGE_DIR/u-boot-update/ccm2200.bin

  # we generate a fallback script which only runs on u-boot v.1.2.0
  # in case a command (ccm2200 led) fails in the v2010 script

  $MKIMAGE -A arm -O linux -T script -C none -a 0x20100000 -e 0x20100000 \
           -n "CCM2200 U-Boot update v2010"                              \
           -d board/swarco/ccm2200/scripts/u-boot-update-v2010.script    \
           $IMAGE_DIR/u-boot-update/ub_v2010.bin

  $MKIMAGE -A arm -O linux -T script -C none -a 0x20100000 -e 0x20100000 \
           -n "CCM2200 U-Boot update v1.2"                               \
           -d board/swarco/ccm2200/scripts/u-boot-update-v1_2.script     \
           $IMAGE_DIR/u-boot-update/ub_v1_2.bin
fi


# Local Variables:
# mode: shell-script
# compile-command: "sh ./build-ccm2200.sh"
# End:
