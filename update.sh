#!/usr/bin/bash

CADIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

cd "${CADIR}"

# check firmware integrity
if ! md5sum --status -c "release/crampon.md5" ; then
    echo "Firmware image failed md5 check"
    exit 1
fi

# put device into bootloader
for dev in /dev/serial/by-id/usb-Annex_Engineering_Crampon* /dev/serial/by-id/usb-Klipper_stm32l412xx* ; do
    if [ -e "$dev" ] ; then
        echo "Entering bootloader for $dev"
        stty -F "$dev" 1200
        sleep 0.8
    fi
done

# check for bootloader and launch updater
if lsusb -d 0483:df11 ; then
    if sudo dfu-util -d ,0483:df11 -a 0 -D "release/crampon.bin" --dfuse-address 0x08000000:leave ; then
        echo "Device update succeeded"
    else
        echo "Device update failed"
        exit 1
    fi
else
    echo "No device found in boot mode"
fi

echo "Checking for enumerated devices"
sleep 0.8
for dev in /dev/serial/by-id/usb-Annex_Engineering_Crampon* ; do
    if [ -e "$dev" ] ; then
        echo "Crampon device found at: $dev"
    fi
done
