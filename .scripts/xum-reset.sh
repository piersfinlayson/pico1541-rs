#!/bin/bash

# -t 0x20 is 0x00 (OUT) | 0x20 (vendor specific) 
# -r 0x02 is the request ID for RESET
# -v is the value (ignored)
# -i is the itnerface
# -l 8 is the expected length of the response
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 control out -t 0x20 -r 0x02 -v 0 -i 0 -l 8
