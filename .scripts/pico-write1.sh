#!/bin/bash

# 0x09 is WRITE, 0x10 is protocol, 0x02 | (0x00 << 8) is amount of data to write
usbcmd/usbcmd.py -v 0x1209 -p 0xf541 bulk out 0x01 -d 0x09120100

# Now write 2 bytes (both 0x00)
usbcmd/usbcmd.py -v 0x1209 -p 0xf541 bulk out 0x01 -d 0xfc

# Read in 3 byte status response after READ
usbcmd/usbcmd.py -v 0x1209 -p 0xf541 bulk in 0x81 -l 3
