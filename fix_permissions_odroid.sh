#!/bin/bash
lsusb
echo "Provide the vendor ID of the USB device:"
read idVendor
echo "Provide the product ID of the USB device"
read idProduct
string="# UVC cameras
SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", MODE=\"0666\"
ACTION==\"add\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", RUN+=\"/sbin/modprobe usbserial\", RUN+=\"/bin/sh -c 'echo $idVendor $idProduct > /sys/bus/usb-serial/drivers/generic/new_id'\"
"
echo "$string" >> /etc/udev/rules.d/99-uvc.rules

string2="usbserial vendor=0x$idVendor product=0x$idProduct"

echo "$string2" >> /etc/modules

# do not  display ext4 swap and mmc devices
blkid | grep -v -e 'ext4' -e 'swap' -e 'mmcblk'
echo "Provide UUID of the USB device"
read uuid
fstab_str="RUN+=\"/bin/mount -t vfat -o uid=0,gid=46,umask=007 /dev/disk/by-uuid/$uuid /media/camera\""
# copy cmd string to usbrules
echo "$fstab_str" >> /etc/udev/rules.d/80-usbdevice.rules

# create mount folder
mkdir /media/camera

echo "Unplug the USB device and reboot your system."
