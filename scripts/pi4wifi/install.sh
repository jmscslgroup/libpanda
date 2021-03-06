#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
echo "Installing crazypifi scripts"


#build dependencies:
declare -a depencencies=(hostapd isc-dhcp-server wireless-tools wpasupplicant ifupdown iptables iw)
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

cp crazypifi.sh /usr/local/sbin/crazypifi
touch /etc/wpa_supplicant/wpa_supplicant.conf

if [ ! -f /etc/network/interfaces.bak ]; then
	echo "Backing up /etc/network/interfaces to /etc/network/interfaces.bak"
	cp /etc/network/interfaces /etc/network/interfaces.bak
fi
cp interfaces.* /etc/network/

if [ ! -f /etc/hostapd/hostapd.conf.bak ]; then
	echo "Backing up /etc/hostapd/hostapd.conf to /etc/hostapd/hostapd.conf.bak"
	mv /etc/hostapd/hostapd.conf /etc/hostapd/hostapd.conf.bak
fi
cp hostapd.conf /etc/hostapd/hostapd.conf

sed -i 's/#DAEMON_CONF=.*/DAEMON_CONF=\"\/etc\/hostapd\/hostapd.conf\"/' /etc/default/hostapd

#if [ ! -f /etc/dhcpcd.conf.bak ]; then
#	echo "Backing up /etc/dhcpcd.conf to /etc/dhcpcd.conf.bak"
#	mv /etc/dhcpcd.conf /etc/dhcpcd.conf.bak
#fi
#cp dhcpcd.conf /etc/dhcpcd.conf
sed -i 's/INTERFACESv4=""/INTERFACESv4="wlan0"/' /etc/default/isc-dhcp-server

if [ ! -f /etc/dhcp/dhcpd.conf.bak ]; then
	echo "Backing up /etc/dhcp/dhcpd.conf to /etc/dhcp/dhcpd.conf.bak"
	cp /etc/dhcp/dhcpd.conf /etc/dhcp/dhcpd.conf.bak
fi
cp dhcpd.conf /etc/dhcp/dhcpd.conf

if [ ! -f /etc/wpa_supplicant/wpa_supplicant.conf.bak ]; then
	echo "Backing up /etc/wpa_supplicant/wpa_supplicant.conf to /etc/wpa_supplicant/wpa_supplicant.conf.bak"
	cp /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.bak
fi
cp wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf


cp crazypifi.service.txt  /etc/systemd/system/crazypifi.service
chmod 655 /etc/systemd/system/crazypifi.service

systemctl daemon-reload

systemctl unmask hostapd
systemctl enable crazypifi.service

# On raspbian we have to not block wifi
systemctl stop wpa_supplicant
rm /var/run/wpa_supplicant/*
rfkill unblock wifi

# We will be managing the below services manually
systemctl disable hostapd
systemctl disable isc-dhcp-server
systemctl disable wpa_supplicant

# Setup Wifi AP:

name=""
while [ "${name}" != "n" ] && [ "${name}" != "y" ]
do
	read -n 1 -p " - Configure WiFi now? [y/n]:" name
	echo ""

	echo "you entered: ${name}"
done

if [ "${name}" = "y" ];
then
	read -p " - Enter SSID (WiFi name):" ssid

	read -sp " - Enter passphrase:" passphrase
	echo

	echo "Configuring WPA Supplicant..."

	wpa_passphrase ${ssid} ${passphrase} >> /etc/wpa_supplicant/wpa_supplicant.conf
fi

unset passphrase

# Setup status files:
if [ ! -d /etc/libpanda.d ]; then
	mkdir /etc/libpanda.d
fi

if [ ! -f /etc/libpanda.d/isaphost ]; then
	touch /etc/libpanda.d/isaphost
fi

if [ ! -f /etc/libpanda.d/isapclient ]; then
	touch /etc/libpanda.d/isapclient
fi

if [ ! -f /etc/libpanda.d/hasinternet ]; then
	touch /etc/libpanda.d/hasinternet
fi

if [ ! -f /etc/libpanda.d/hasapclients ]; then
	touch /etc/libpanda.d/hasapclients
fi

echo "Done."
echo "----------------------------"
