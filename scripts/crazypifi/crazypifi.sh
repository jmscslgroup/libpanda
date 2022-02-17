#!/bin/bash
# Author: Matt Bunting
# Inspired from: https://www.raspberrypi.org/forums/viewtopic.php?t=111529

LOG=/var/panda/crazpypifi.log

exec > >(tee -ia $LOG)
exec > >(tee -ia $LOG >&2)

echo "--------------------------------"
echo "Scanning for known WiFi networks"

echoGood ()
{
	echo -e "\e[1m\e[32m$1\e[0m"
}

echoBad ()
{
	echo -e "\e[1m\e[31m$1\e[0m"
}


setupDhcpServer ()
{
#	echo "Restarting dhcpcd"
#	service dhcpcd restart
	echo -n " - Restarting isc-dhcp-server..."
	if service isc-dhcp-server restart; then
		echoGood "Done."
	else
		echoBad "FAILED!"
	fi
}

connectToKnownWifi ()
{
	echo " - Configuring interfaces"
	cp /etc/network/interfaces.client /etc/network/interfaces
#	service dhcpcd stop

	echo -n " - Stopping DHCP server..."
	if systemctl stop isc-dhcp-server; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		return 0
	fi

	echo -n " - Stopping Hostapd..."
	if service hostapd stop; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		return 0
	fi

	echo -n " - Removing static IP..."
	if ifconfig wlan0 0.0.0.0; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		return 0
	fi


#	echo -n " - Restarting wpa_supplicant"
#	service wpa_supplicant restart

	echo -n " - Stopping wlan0..."
	if ifdown wlan0; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		return 0
	fi

	echo -n " - Starting wlan0..."
	if ifup wlan0; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		return 0
	fi

	#ssids=$1



#	echo -n " - Starting wpa_supplicant for WPA/WPA2..."
#	if wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf; then
#	if wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf; then
#		echoGood "Done."
#	else
#		echoBad "FAILED!"
#		#return 0
#	fi


	echo -n " - Selecting AP SSID: " $1 "..."
	if wpa_cli -i wlan0 select_network $1 > /dev/null; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		#return 0
	fi

	echo -n " - Finding wpa_cli nework number..."
	networkNumber=$(wpa_cli list_networks | grep -Po "[0-9]*(?=[\s]$1[\s].*)")

	if [ -z $networkNumber ]; then
		echoBad "Unable to find it!"
		return
	else
		echoGood "$networkNumber"
	fi

	echo -n " - Enabling network..."
	if wpa_cli -i wlan0 enable_network $networkNumber > /dev/null; then
		echoGood "Done"
	else
		echoBad "FAILED"
		return 0
	fi

	for (( attempt=1; attempt<=30; attempt++ )); do
 		echo " - - Waiting for connection to $1, attempt $attempt"
		sleep 1
		if wpa_cli -i wlan0 status | grep "wpa_state=COMPLETED"; then
			break
		fi
	done

	if [ "$attempt" -ge 31 ]; then
		echoBad "Unable to connect to $1"
		return 0
	fi

#	echo -n " - Releasing DHCP lease..."
#	if dhclient -r wlan0 -v; then
#		echoGood "Done."
#	else
#		echoBad "FAILED!"
#		#return 0
#	fi

	echo -n " - Obtaining IP from DHCP..."
	if dhclient wlan0 -v;
	then
		echoGood "Connected to WiFi: $1"
		return 1
	else
		echoBad "Unable to get IP"
#wpa_cli terminate
	fi

	return 0
}

setupAp ()
{
	echo -n " - Releasing DHCP lease..."
	if dhclient -r wlan0; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		#return 0
	fi


	echo -n " - Stopping wpa_supplicant..."
	if wpa_cli -i wlan0 disconnect > /dev/null; then
#	if service wpa_supplicant stop; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		#return 0
	fi

	echo -n " - Configuring interfaces..."
	if cp /etc/network/interfaces.accesspoint /etc/network/interfaces; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		#return 0
	fi

	echo -n " - Stopping wlan0..."
	if ifdown wlan0; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		#return 0
	fi

	echo -n " - Starting wlan0..."
	if ifup wlan0; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		#return 0
	fi

	echo -n " - Setting static IP..."
	if ifconfig wlan0 10.0.1.1/24; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		#return 0
	fi

	echo -n " - Restarting hostapd..."
	if service hostapd restart; then
		echoGood "Done."
	else
		echoBad "FAILED!"
		#return 0
	fi

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

havePhoneTether ()
{
	if ethtool eth1 | grep "Link detected: yes" > /dev/null 2>&1; then
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

bringupWlan ( )
{
	echo -n "Checking state of wlan0..."
	if ifconfig | grep "wlan0:" > /dev/null 2>&1; then
		echoGood "UP"
	else
		echoBad "DOWN"
	echo -n " - Attempting to bring up wlan0..."
		if ifup wlan0 > /dev/null 2>&1; then
			echoGood "Success"
		else
			echoBad "FAILED"
		fi
	fi
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
#	echo "Beginning network status check."

# If internet exists, things are probably in an acceptable state.
	echo -n "Checking Internet status..."
	haveInternet
	if [ $? -eq 1 ]; then
		echoGood "Connected"
		echo "1" > /etc/libpanda.d/hasinternet
		echo "0" > /etc/libpanda.d/isaphost
		echo "1" > /etc/libpanda.d/isapclient
#		return 1
	else
		echoBad "Disconnected"
		echo "0" > /etc/libpanda.d/hasinternet
	fi

	wifiConnected=false
	ethernetConnected=false
	phoneConnected=false

# check for a physical connection on ethernet
	echo -n "Checking for eth0 link status..."
	haveEthernetPhysicalConnection
	if [ $? -eq 1 ]; then
		echoGood "Connected"
		ethernetConnected=true
		echo "1" > /etc/libpanda.d/hasphonetether
		echo "0" > /etc/libpanda.d/isapclient
#		haveIpOnDevice eth0
#		if [ $? -eq 1 ]; then
#			echo " - I has ethernet IP!"
#			haveIpOnDevice wlan0
##			return 1
#		fi
	else
		echoBad "Disconnected"
		echo "0" > /etc/libpanda.d/hasphonetether
#		setupDhcpServer
	fi
	
	# check for a phone tether over USB
	echo -n "Checking eth1 phone tether status..."
	havePhoneTether
	if [ $? -eq 1 ]; then
		echoGood "Connected"
		phoneConnected=true
		# Ensure wifi is up to host AP
		bringupWlan
	else
		echoBad "Disconnected"
	fi

	# Only attempt to connect to ground station AP if phone no connected:
	if [ "$phoneConnected" = false ]; then
		
#		if systemctl is-active --quiet hostapd; then
		if iwconfig wlan0 | grep "Mode:Master"; then
# If any wifi clients exist, don't change anything to cause a disconnection
			echo -n "Checking if wlan0 is busy with clients..."
			wificlients=$(iw dev wlan0 station dump)
			if [ -z "$wificlients" ]; then
				echo "0" > /etc/libpanda.d/hasapclients
				echoGood "Free"
			else
				echoBad "Busy"
				echo "1" > /etc/libpanda.d/hasapclients
				return 1
			fi
		fi

# Check if wifi is currently a client
		echo -n "Checking if wifi is connected to AP..."
		haveWifiConnection
		if [ $? -eq 1 ]; then
			echoGood "Connected to ${currentSsid}"
			echo "1" > /etc/libpanda.d/isapclient
			wifiConnected=true
		else
			echoBad "Disconnected"
			echo "0" > /etc/libpanda.d/isapclient

#			echo -n "Checking state of wlan0..."
#			if ifconfig | grep "wlan0:" > /dev/null 2>&1; then
#				echoGood "UP"
#			else
#				echoBad "DOWN"
#				echo -n " - Attempting to bring up wlan0..."
#				if ifup wlan0 > /dev/null 2>&1; then
#					echoGood "Success"
#				else
#					echoBad "FAILED"
#				fi
#			fi
			bringupWlan

			echo "Searching for known APs..."
#			Get known SSIDs:
#			ssids=($(cat /etc/wpa_supplicant/wpa_supplicant.conf | grep -v "#" | grep -Po "(?<=ssid=\")[a-zA-Z0-9\-_]*"))
			ssids=($(wpa_cli list_networks | grep -Po "(?<=^[0-9][\s])[\s]*([a-zA-Z0-9\-_]*)(?=.*)"))
#			Scan for available SSIDs:
			ssidsavailable=($(iwlist wlan0 scan | grep -Po "(?<=ESSID:\")[a-zA-Z0-9\-_]*"))
			echo " - Available ESSIDs : ${ssidsavailable[@]}"

			for ssid in "${ssids[@]}"
			do
				echo -n "Checking for $ssid in [${ssidsavailable[@]}]..."
				if echo "${ssidsavailable[@]}" | grep "\b$ssid\b" > /dev/null; then
					echoGood "Found!"
					connectToKnownWifi $ssid
					if [ $? -eq 1 ]; then
						wifiConnected=true	# Success!
						echo "1" > /etc/libpanda.d/isapclient
						break
					fi
					# If we are here, then try the next ssid
				else
					echoBad "Does not Exist"
				fi
			done
		fi
	fi

	if [ "$wifiConnected" = true ]; then
		echo "No need to setup AP"
	else
		echo -n "Not Connecting to a WiFi groundstation AP, checking self-hosted AP state..."
		if systemctl is-active --quiet hostapd; then
			echo "1" > /etc/libpanda.d/isaphost
			echoGood "Active"
		else
			echoBad "Inactive"
			echo "Starting AP"
			setupAp
			echo "1" > /etc/libpanda.d/isaphost
		fi
	fi

	echo "Done."




	return 1
}

doEthernet ()
{
	# check for a physical connection on ethernet
	echo -n "Checking for eth0 link status..."
	haveEthernetPhysicalConnection
	if [ $? -eq 1 ]; then
		echoGood "Connected"
		ethernetConnected=true

		dhclient -1 eth0

#		haveIpOnDevice eth0

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





}

trap cleanup SIGINT SIGTERM

while [ 1 ]
do
	echo "--------------------------------"
	doStuff
	haveIpOnDevice wlan0
	haveIpOnDevice eth1
	doEthernet
	haveIpOnDevice eth0
	sleep 10 &
	wait $!
done
