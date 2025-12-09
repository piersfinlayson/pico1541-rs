#!/bin/bash

# ATN low & Talk set, 0x40 talk | 0x08 device, 0x60 | 0x0f secondary address 15
echo "Put drive into talk mode"
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 bulk out 0x04 -d 0x09130200
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 bulk out 0x04 -d 0x486f
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 bulk in 0x83 -l 3

# Read 32 bytes
echo "Read up to 32 bytes from drive"
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 bulk out 0x04 -d 0x08102000
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 bulk in 0x83 -l 0x20

# ATN low, 0x5f, untalk
echo "Put drive into untalk mode"
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 bulk out 0x04 -d 0x09120100
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 bulk out 0x04 -d 0x5f
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 bulk in 0x83 -l 3
