#!/bin/bash
# Author: Matt Bunting
# This pings Google's DNS server 8.8.8.8 to check for internet availability
#CyVerse.org does not seem to respond to pings

if ping -q -c 1 -W 1 8.8.8.8 > /dev/null; then
	#echo "Internet is online!"
	exit 0
fi

#echo "Internet not connected"
exit 1
