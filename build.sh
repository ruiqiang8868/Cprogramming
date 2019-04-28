#!/usr/bin/bash

echo "files in this folder are:"

ls -al

gcc GNSS_USB_IF_Sampling.c -o guis -I/usr/local/include/libusb-1.0  -L/usr/local/lib -lusb-1.0

echo "build file done!"
