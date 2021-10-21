#!/bin/sh

while true;
do
    today=$(date "+%Y%m%d")
    datetime=$(date "+%Y-%m-%d %H:%M:%S")
    Date=`date`
    rssi=`iwpriv wlan0 stat | grep RSSI`
    w0tx=`iwpriv wlan0 stat | grep 'Last TX'`
    w0rx=`iwpriv wlan0 stat | grep 'Last RX'`
    i2cget1=`i2cget -y 0 0x20 0x02`
    Space=" "
    echo "Slave - "$datetime", "$rssi", w0: "$w0tx", "$w0rx", LNA: "$i2cget1 >> log_slave_${today}.txt 
    sleep 3;
done