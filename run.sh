#!/bin/bash
if [ -n "$1" ]; then
  PORTNUM=$1
else
  PORTNUM=0
fi

rm -R ./build
idf.py build

if [ $? -ne 0 ]
then
    exit
fi

python /home/user/esp/esp-mdf/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB`echo -e $PORTNUM` --baud 921600 erase_flash

python /home/user/esp/esp-mdf/esp-idf/components/esptool_py/esptool/esptool.py -p /dev/ttyUSB`echo -e $PORTNUM` -b 921600 --after hard_reset write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/rwble_bug.bin

python /home/user/esp/esp-mdf/esp-idf/tools/idf_monitor.py --port /dev/ttyUSB`echo -e $PORTNUM` --baud 115200 ./build/rwble_bug.elf
