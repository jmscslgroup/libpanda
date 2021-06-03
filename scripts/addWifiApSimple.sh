#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
echo "WiFi Setup"

# Setup Wifi AP:

name=""


while getopts s:p: flag
do
    case "${flag}" in
        s) ssid=${OPTARG};;
        p) passphrase=${OPTARG};;
    esac
done

if [ -z "$ssid" ] || [ -z "$passphrase" ]; then
	echo "Usage: $1 -s <ssid> -p <passphrase>"
fi
	

if [ "${name}" = "y" ];
then
	#read -p " - Enter SSID (WiFi name):" ssid

	#read -sp " - Enter passphrase:" passphrase
	#echo

	echo "Configuring WPA Supplicant..."

	wpa_passphrase ${ssid} ${passphrase} >> /etc/wpa_supplicant/wpa_supplicant.conf

	sed -i "/#psk=/d" /etc/wpa_supplicant/wpa_supplicant.conf

	unset passphrase

	echo "Conifugarion set:"
	echo "- /etc/wpa_supplicant/wpa_supplicant.conf"
fi
echo "----------------------------"
