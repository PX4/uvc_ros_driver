#!/bin/bash

lsusb
echo "Provide the vendor ID of the USB device:"
read idVendor
echo "Provide the product ID of the USB device"
read idProduct
string="# UVC cameras
SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", MODE=\"0666\"
"
echo "$string" >> /etc/udev/rules.d/99-uvc.rules

string2="usbserial vendor=0x$idVendor product=0x$idProduct"

echo "$string2" >> /etc/modules

echo "Unplug the USB device and plug it back in."
