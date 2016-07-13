#!/bin/bash
proc=$(uname -p)

if [ $proc == "x86_64" ]; then
  fstab_str="/dev/disk/by-id/usb-Cypress_FX3-0:0    /media/camera   auto    rw,user,noauto  0       0"
  echo "$fstab_str" >> /etc/fstab
elif [ $proc == "armv7l" ]; then
  str="RUN+=\"/bin/mount -t vfat -o uid=0,gid=46,umask=007 /dev/disk/by-id/usb-Cypress_FX3-0:0 /media/camera\""
  # copy cmd string to usbrules
  echo "$str" > /etc/udev/rules.d/99-uvc-usbdevice.rules
  # create mount folder
  mkdir -p /media/camera
fi

echo "Unplug the USB device and reboot your system."
