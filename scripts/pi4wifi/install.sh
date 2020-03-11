#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
echo "Installing crazywifi scripts"


#build dependencies:
declare -a depencencies=(hostapd isc-dhcp-server wireless-tools wpasupplicant ifupdown)
toInstall=()
echo "Dependencies:" ${depencencies[@]}
for dependency in "${depencencies[@]}"
do
	echo "Checking" $dependency
	if [ $(dpkg-query -W -f='${Status}' $dependency 2>/dev/null | grep -c "ok installed") -eq 0 ];
	then
		echo "not installed:" $dependency
		toInstall+=($dependency)
	fi
done
echo ${toInstall[@]}

if [ ${#toInstall[@]} -ne 0 ];
then
	apt-get update
	apt-get install -y ${toInstall[@]}
fi

# check for netplan, if exists then remove
if [ $(dpkg-query -W -f='${Status}' nplan 2>/dev/null | grep -c "ok installed") -eq 1 ];
then
	echo "Netplan installed, removing"
	apt-get purge -y nplan netplan.io
fi

#if [ $(dpkg-query -W -f='${Status}' nplan 2>/dev/null | grep -c "ok installed") ]; then
#	echo "netplan installed, removing and installing ifupdown"
#	apt-get install ifupdown
#fi

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

#if [ ! -f /etc/dhcpcd.conf.bak ]; then
#	echo "Backing up /etc/dhcpcd.conf to /etc/dhcpcd.conf.bak"
#	mv /etc/dhcpcd.conf /etc/dhcpcd.conf.bak
#fi
#cp dhcpcd.conf /etc/dhcpcd.conf

if [ ! -f /etc/dhcp/dhcpd.conf.bak ]; then
	echo "Backing up /etc/dhcp/dhcpd.conf to /etc/dhcp/dhcpd.conf.bak"
	cp /etc/dhcp/dhcpd.conf /etc/dhcp/dhcpd.conf.bak
fi
cp dhcpd.conf /etc/dhcp/dhcpd.conf



echo "Done."
echo "----------------------------"
