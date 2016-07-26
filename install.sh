#!/bin/bash

# set permissions on uvc device
idVendor=04b4
idProduct=00f8
string="# UVC cameras
SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", MODE=\"0666\"
"
echo "$string" >> /etc/udev/rules.d/99-uvc.rules


# set mount point of mass storage device 
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

# set serial device on modemmanager blacklist
string2="ACTION!=\"add|change|move\", GOTO=\"mm_usb_device_blacklist_end\"
SUBSYSTEM!=\"usb\", GOTO=\"mm_usb_device_blacklist_end\"
ENV{DEVTYPE}!=\"usb_device\",  GOTO=\"mm_usb_device_blacklist_end\"

# All devices from Cypress
ATTRS{idVendor}==\"04b4\", ENV{ID_MM_DEVICE_IGNORE}=\"1\"
"
echo "$string2" >> /etc/udev/rules.d/78-mm-sky-aware-blacklist.rules


echo "Unplug the USB device and plug it back in."

#reload rules
usermod -a -G dialout $USER
udevadm control --reload-rules

