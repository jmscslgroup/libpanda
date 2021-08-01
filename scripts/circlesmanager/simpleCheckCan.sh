#!/bin/bash
# Author: Matt Bunting
# Checks if the "can" service is running

if systemctl status can > /dev/null; then
	#echo "Internet is online!"
	exit 0
fi

#echo "Internet not connected"
exit 1
