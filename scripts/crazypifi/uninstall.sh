#!/bin/bash
# Author: Matt Bunting

echo "------------------------------"
echo "Uninstalling crazywifi scripts"

cp interfaces.* /etc/network/
cp crazywifi.sh /usr/sbin/crazywifi

if [ -f /etc/network/interfaces.bak ]; then
	echo "Replacing /etc/network/interfaces with /etc/network/interfaces.bak"
	mv /etc/network/interfaces.bak /etc/network/interfaces
fi

if [ -f /etc/hostapd/hostapd.conf.bak ]; then
	echo "Replacing /etc/hostapd/hostapd.conf with /etc/hostapd/hostapd.conf.bak"
	mv /etc/hostapd/hostapd.conf.bak /etc/hostapd/hostapd.conf
fi

if [ -f /etc/dhcpcd.conf.bak ]; then
	echo "Replacing /etc/dhcpcd.conf with /etc/dhcpcd.conf.bak"
	mv /etc/dhcpcd.conf.bak /etc/dhcpcd.conf
fi

rm /etc/network/interfaces.client
rm /etc/network/interfaces.accesspoint
rm /usr/sbin/crazywifi

echo "Done."
echo "------------------------------"
