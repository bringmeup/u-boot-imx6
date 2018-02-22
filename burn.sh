#!/bin/bash

SDCARD=$1

function umountall() {
	sudo umount "$1"p1
	sudo umount "$1"p2
	sudo umount "$1"p3
	sudo umount "$1"p4
}

#make sure it's safe to play with the disk
umountall $SDCARD

# prepare partitions -- make sure there is some space left for uboot
sudo sfdisk $SDCARD < sdcard.layout
sudo mkfs.vfat "$SDCARD"p1

#  +-----+------+--------+-----+----------------
#  | MBR |  ... | u-boot | ... | FAT partition ...
#  +-----+------+--------+-----+----------------
#  0     512    1024           20M

# copy uboot (skip mbr)
sudo dd if=u-boot.imx of=$SDCARD bs=512 seek=2 conv=fsync

# make sure it's safe to remove
sync
umountall $SDCARD
