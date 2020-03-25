#!/bin/bash
# Author: Matt Bunting
# Inspired from: https://www.raspberrypi.org/forums/viewtopic.php?t=111529

echo "--------------------------------"
echo "Scanning for known WiFi networks"

setupDhcpServer ()
{
#	echo "Restarting dhcpcd"
#	service dhcpcd restart
	echo " - Restarting isc-dhcp-server"
	service isc-dhcp-server restart
}

echoGood ()
{
	echo -e " - \e[1m\e[32m$1\e[0m"
}

echoBad ()
{
	echo -e " - \e[1m\e[31m$1\e[0m"
}

connectToKnownWifi ()
{
	echo " - Configuring interfaces"
	cp /etc/network/interfaces.client /etc/network/interfaces
#	service dhcpcd stop
	echo " - Stopping DHCP server"
	service isc-dhcp-server stop

	echo " - Stopping Hostapd"
	service hostapd stop

	echo " - Restarting wlan0"
	ifdown wlan0
	ifup wlan0

	#ssids=$1

#	echo " - Connecting to SSID: " $1

#	echo " - Starting supplicant for WPA/WPA2"
	wpa_supplicant -B -i wlan0 -c /etc/wap_supplicant/wpa_supplicant.conf > /dev/null 2>&1
	echo " - Releasing DHCP lease"
	dhclient -r wlan0
	echo " - Obtaining IP from DHCP"
	if dhclient -1 wlan0;
	then
		echoGood "- Connected to WiFi: $1"
		return 1
	else
		echoBad "- Unaible to get IP"
		wpa_cli terminate
	fi

	return 0
}

setupAp ()
{
	echo " - Configuring interfaces"
	cp /etc/network/interfaces.accesspoint /etc/network/interfaces

	echo " - Restarting wlan0"
	ifdown wlan0
	ifup wlan0

	echo " - Restarting hostapd"
	service hostapd restart

	setupDhcpServer
}

enableDhcpServer ()
{
	return 1
}

haveInternet ()
{
	if ping -q -c 1 -W 1 8.8.8.8 > /dev/null; then
		return 1
	fi
	return 0
}

haveEthernetPhysicalConnection ()
{
	if ethtool eth0 | grep "Link detected: yes" > /dev/null; then
		return 1
	fi
	return 0
}

haveIpOnDevice ( )
{
	interface=$1
	ips=($(ifconfig ${interface} | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'))
	echo ${interface} "IP:" ${ips[@]}
	if [ ${#ips[@]} ]; then
		return 1
	fi
	return 0
}

haveWifiConnection ()
{
	currentSsid=$(iwconfig 2>&1 | grep -v "off/any" | grep -Po "(?<=ESSID:\")[a-zA-Z0-9\-_]*")
#	echo "currentSsid=" $currentSsid
	if [ -z "${currentSsid}" ]; then
#		echo "No wifi connection"
		return 0
	fi
#	echo "Yes wifi connection"
	return 1
}

cleanup ()
{
	echo "Received kill signal. Exiting..."
	kill -s SIGTERM $!
	echo "--------------------------------"
	exit 0
}

doStuff ()
{
	echo "Beginning network status check."

# If internet exists, things are probably in an acceptable state.
	echo "Checking Internet status..."
	haveInternet
	if [ $? -eq 1 ]; then
		echoGood "Connected"
		return 1
	else
		echoBad "Disconnected"
	fi

	wifiConnected=false
	ethernetConnected=false

# check for a physical connection on ethernet
	echo "Checking for eth0 link status"
	haveEthernetPhysicalConnection
	if [ $? -eq 1 ]; then
		echoGood "Connected"
		ethernetConnected=true

#		haveIpOnDevice eth0
#		if [ $? -eq 1 ]; then
#			echo " - I has ethernet IP!"
#			haveIpOnDevice wlan0
##			return 1
#		fi
	else
		echoBad "Disconnected"
#		setupDhcpServer
	fi

	if systemctl is-active --quiet hostapd; then
# If any wifi clients exist, don't change anything to cause a disconnection
		echo "Checking if wlan0 is busy with clients"
		wificlients=$(iw dev wlan0 station dump)
		if [ -z "$wificlients" ]; then
			echoGood "Free"
		else
			echoBad "Busy"
			return 1
		fi
	fi

# Check if wifi is currently a client
	echo "Checking if wifi is connected to AP"
	haveWifiConnection
	if [ $? -eq 1 ]; then
		echoGood "Connected to ${currentSsid}"
		wifiConnected=true
	else
		echoBad "Disconnected"

		echo "Searching for known APs"
#		Get known SSIDs:
		ssids=($(cat /etc/wpa_supplicant/wpa_supplicant.conf | grep -v "#" | grep -Po "(?<=ssid=\")[a-zA-Z0-9\-_]*"))
#		Scan for available SSIDs:
		ssidsavailable=($(iwlist wlan0 scan | grep -Po "(?<=ESSID:\")[a-zA-Z0-9\-_]*"))
		echo " - Available ESSIDs :" ${ssidsavailable[@]}

		for ssid in "${ssids[@]}"
		do
			if echo ${ssidsavailable[@]} | grep $ssid > /dev/null
			then
				echo "Connecting to $ssid"
				connectToKnownWifi $ssid
				if [ $? -eq 1 ]; then
					wifiConnected=true	# Success!
					break
				fi
				# If we are here, then try the next ssid
			fi
		done
	fi

	if [ "$wifiConnected" = true ]; then
		echo "No need to setup AP"
	else
		echo "Unable to connect to WiFi, checking if AP is active"
		if systemctl is-active --quiet hostapd; then
			echoGood "AP is active"
		else
			echoBad "AP inactive"
			echo "Starting AP"
			setupAp
		fi
	fi

	echo "Done."

haveIpOnDevice wlan0
haveIpOnDevice eth0

	return 1
}

trap cleanup SIGINT SIGTERM

while [ 1 ]
do
	echo "--------------------------------"
	doStuff
	sleep 10 &
	wait $!
done
