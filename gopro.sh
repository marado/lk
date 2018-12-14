#!/bin/bash
for i in /local/mnt/workspace/c_vpasme/Madavi/GoPro_Patches/caf/bootable/bootloader/lk/*.patch;do patch -p1 < $i; done 