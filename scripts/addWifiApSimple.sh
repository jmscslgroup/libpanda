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
	echo "Usage: $0 -s <ssid> -p <passphrase>"
	exit 1
fi
	

#if [ "${name}" = "y" ];
#then
	#read -p " - Enter SSID (WiFi name):" ssid

	#read -sp " - Enter passphrase:" passphrase
	#echo

	echo "Configuring WPA Supplicant..."

#	wpa_passphrase ${ssid} ${passphrase} >> /etc/wpa_supplicant/wpa_supplicant.conf
	wpa_info=$(wpa_passphrase ${ssid} ${passphrase})
	
	if [ $? -ne 1 ];
	then
		echo "Saving"
		echo "${wpa_info}"
		echo "${wpa_info}" >> /etc/wpa_supplicant/wpa_supplicant.conf
		sed -i "/#psk=/d" /etc/wpa_supplicant/wpa_supplicant.conf
		
		echo "Confugaration saved to /etc/wpa_supplicant/wpa_supplicant.conf"
	else
		echo "Invalid - NOT setting wpa_supplicant.conf"
		echo "${wpa_info}"
	fi

	unset passphrase

	
#fi
echo "----------------------------"
