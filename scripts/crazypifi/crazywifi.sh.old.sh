#!/bin/bash
# Author: Matt Bunting
# Inspired from: https://www.raspberrypi.org/forums/viewtopic.php?t=111529

echo "--------------------------------"
echo "Scanning for known WiFi networks"

setupDhcpServer ()
{
echo "Restarting dhcpcd"
service dhcpcd restart
}

getKnownSsids ()
{
echo "Getting list of prior configured SSIDs..."
ssids=($(cat /etc/wpa_supplicant/wpa_supplicant.conf | grep -v "#" | grep -Po "(?<=ssid=\")[a-zA-Z0-9\-_]*"))
}

getAvailableSsids ()
{
echo "Scanning for available SSIDs..."
ssidsavailable=($(iwlist wlan0 scan | grep -Po "(?<=ESSID:\")[a-zA-Z0-9\-_]*"))
}

connectToKnownWifi ()
{
cp /etc/network/interfaces.client /etc/network/interfaces
service dhcpcd stop
#service networking stop
#service networking start
ifdown wlan0
ifup wlan0

#ssids=$1

echo "Connecting to SSID: " $1
echo "Stopping Hostapd"
service hostapd stop
echo "Starting supplicant for WPA/WPA2"
wpa_supplicant -B -i wlan0 -c /etc/wap_supplicant/wpa_supplicant.conf > /dev/null 2>&1
echo "Obtaining IP from DHCP"
if dhclient -1 wlan0
then
echo "Connected to WiFi: " $1
wifiConnected=true
return 1
else
echo "DHCP server did not respond with an IP lease"
wifiConnected=false
wpa_cli terminate
fi

return 0
}

setupAp ()
{
echo "Starting AccessPoint"
#cp /etc/network/interfaces.accesspoint /etc/network/interfaces
cp /etc/network/interfaces.accesspoint /etc/network/interfaces
#service networking stop
#	service networking start
ifdown wlan0
ifup wlan0
#	echo "Restarting isc-dhcp-server"
#	service isc-dhcp-server restart
#	hostapd -dd /etc/hostapd/hostapd.conf
echo "Restarting hostapd"
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
echo "I has internet"
return 1
else
echo "I is alone"
fi
return 0
}

haveEthernetPhysicalConnection ()
{
if ethtool eth0 > /dev/null; then
echo "I has ethernet"
return 1
else
echo "I is physically disconnected"
fi
return 0
}

haveIpOnDevice ( )
{
interface=$1
ips=($(ifconfig ${interface} | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'))
echo "IP on" ${interface} ":" ${ips[@]}
if [ ${#ips[@]} ]; then
echo "I has ethernet IP"
return 1
else
echo "I have no address"
fi
return 0
}

haveWifiConnection ()
{
currentSsid=$(iwconfig 2>&1 | grep -v off/any | grep ESSID)
echo "currentSsid=" $currentSsid
if [ -z "${currentSsid}" ]; then
wifiConnected=false
echo "No wifi connection"
return 0
fi
wifiConnected=true
echo "Yes wifi connection"
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
if [ haveInternet ]; then
echo " - I gots the nets!  Nothing to do"
#		return 1
fi

wifiConnected=false
ethernetConnected=false
if [ haveEthernetPhysicalConnection ]; then
echo " - I gots the ethernets!"
ethernetConnected=true
haveIpOnDevice eth0
if [ $? ]; then
echo " - I has ethernet IP!"
haveIpOnDevice wlan0
#			return 1
fi
else
echo " - Ethernet not connected"
#		setupDhcpServer
fi

haveWifiConnection
echo "Connected to wifi:" ${wifiConnected}
if ${wifiConnected}; then
echo "Already connected to an AP"
else
echo "Attempting to connect to wifi"

getKnownSsids
echo " - Known ESSIDs :" ${ssids[@]}

getAvailableSsids
echo " - Available ESSIDs :" ${ssidsavailable[@]}

for ssid in "${ssids[@]}"
do
if echo ${ssidsavailable[@]} | grep $ssid > /dev/null
then
echo " - First known available:" $ssid
connectToKnownWifi $ssid
break
fi
done
fi

if $wifiConnected; then
echo "No need to setup AP"
else
echo "Unable to connect to WiFi, setting up hostapd"
if systemctl is-active --quiet service; then
echo "AP already running"
else
echo "AP NOT running, starting..."
setupAp
fi
fi

#connectToKnownWifi

echo "Done."
echo "--------------------------------"
return 1
}

trap cleanup SIGINT SIGTERM

while [ 1 ]
do
echo "Checking Status..."
doStuff
sleep 60 &
wait $!
done
