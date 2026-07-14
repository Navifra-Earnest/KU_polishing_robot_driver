#!/bin/bash
#sudo /home/navifra/insmod_module_platform.sh
module=""
device=""
mode="664"
maxports="32"

echo  "Load can module and can protocol"
/sbin/modprobe can
/sbin/modprobe can-raw
/sbin/modprobe can-dev


echo "CAN Controller driver"
/sbin/insmod /home/abc/can-ahc0512.ko

/sbin/ip link set can0 up type can bitrate 500000 sample-point 0.6
/sbin/ip link set can1 up type can bitrate 250000 sample-point 0.6
/sbin/ip link set can0 txqueuelen 1000
/sbin/ip link set can1 txqueuelen 1000

echo "CAN Initialized"
