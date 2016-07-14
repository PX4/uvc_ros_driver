#!/bin/bash

idVendor=04b4
idProduct=00f8
string="# UVC cameras
SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", MODE=\"0666\"
"
echo "$string" >> /etc/udev/rules.d/99-uvc.rules

echo "Unplug the USB device and plug it back in."

udevadm control --reload-rules

