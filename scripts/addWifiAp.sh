#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
echo "WiFi Setup"

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

	sed -i "/#psk=/d" /etc/wpa_supplicant/wpa_supplicant.conf

	unset passphrase

	echo "Conifugarion set:"
	echo "- /etc/wpa_supplicant/wpa_supplicant.conf"
fi
echo "----------------------------"
