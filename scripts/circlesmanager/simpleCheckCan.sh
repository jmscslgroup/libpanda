#!/bin/bash
# Author: Matt Bunting
# Checks if the "can" service is running

systemctl status can > /dev/null
retVal=$?
if [ $retVal -ne 0 ]; then
	#echo "Internet is online!"
	exit 0
fi

#echo "Internet not connected"
exit 1
