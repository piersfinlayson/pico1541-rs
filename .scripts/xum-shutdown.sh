#!/bin/bash

# -t 0xa1 is 0x00 (OUT) | 0x20 (vendor specific) | 0x00 (recipient device)
# -r 0x01 is the request ID for INIT
# -v is the value (ignored)
# -i is the interface
usbcmd/usbcmd.py -v 0x16d0 -p 0x0504 control out -t 0x20 -r 0x03 -v 0 -i 0 
