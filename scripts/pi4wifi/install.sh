#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
echo "Installing crazywifi scripts"

cp interfaces.* /etc/network/
cp crazywifi.sh /usr/sbin/crazywifi

if [ ! -f /etc/network/interfaces.bak ]; then
	echo "Backing up /etc/network/interfaces to /etc/network/interfaces.bak"
	cp /etc/network/interfaces /etc/network/interfaces.bak
fi
# nothing to copy here

if [ ! -f /etc/hostapd/hostapd.conf.bak ]; then
	echo "Backing up /etc/hostapd/hostapd.conf to /etc/hostapd/hostapd.conf.bak"
	mv /etc/hostapd/hostapd.conf /etc/hostapd/hostapd.conf.bak
fi
cp hostapd.conf /etc/hostapd/hostapd.conf

if [ ! -f /etc/dhcpcd.conf.bak ]; then
	echo "Backing up /etc/dhcpcd.conf to /etc/dhcpcd.conf.bak"
	mv /etc/dhcpcd.conf /etc/dhcpcd.conf.bak
fi
cp dhcpcd.conf /etc/dhcpcd.conf

echo "Done."
echo "----------------------------"
