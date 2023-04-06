#!/bin/bash
# Author: Matt Bunting

if grep 'net.ipv4.ip_forward' /etc/sysctl.conf > /dev/null 2>&1;
then
	echo "Enabling forwarding in /etc/sysctl.conf"
	sed -i 's/.*net.ipv4.ip_forward.*/net.ipv4.ip_forward=1/' /etc/sysctl.conf
fi

if grep 'net.ipv4.ip_forward' /etc/sysctl.conf > /dev/null 2>&1;
then
	echo "Disabling forwarding in /etc/sysctl.conf"
	sed -i 's/.*net.ipv4.ip_forward.*/#net.ipv4.ip_forward=1/' /etc/sysctl.conf
fi

if systemctl is-active --quiet hostapd; then
echo "AP is active"
else
echo "AP is Inactive"
echo "Starting AP"

fi
