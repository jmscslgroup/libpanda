#!/bin/bash
# Author: Matt Bunting

VIN=$(cat /etc/libpanda.d/vin)
VIN_CURRENT=$(cat /etc/hostname)

if [ "$VIN" == "$VIN_CURRENT" ];
then
	exit
fi

VIN_SIZE=${#VIN}
#echo "VIN_SIZE=${VIN_SIZE}"

if [ $VIN_SIZE -gt 0 ]; # Ensure it's not ablank file, otherwise everything will break
then
#    echo "VIN size is greater than 0"
#    echo "VIN=${VIN}"

	echo $VIN > /etc/hostname
	sed -i "/^127.0.1.1/ c\127.0.1.1  $VIN" /etc/hosts
	sed -i "/^ssid=/ c\ssid=$VIN" /etc/hostapd/hostapd.conf

	echo "vinToHostname: This is a new VIN!  Need to reboot to apply"
	reboot
fi


	

