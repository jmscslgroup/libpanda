#!/bin/bash

# format:
# rest.php?circles,VIN,GpsTime,SysTime,Latitude,Longitude,Altitude,Status
# example:
# curl -k https://engr-sprinkle01s.catnet.arizona.edu:8080/rest.php?circles,LT3MWRFVXLW056972,1615416836.2,1615416835.18839,-86.6058302,36.0252845,213.1,V


HEADER="https://ransom.isis.vanderbilt.edu:8080/GPS_REST_API/rest.php?circles"

VIN_FILE="/etc/libpanda.d/vin"
GPS_FILE="/etc/libpanda.d/latest_gps"

while true;
do
		sleep 5
	if [ ! -f ${VIN_FILE} ]; then
		echo "Error: ${VIN_FILE} does not exist"
		continue
	fi
	if [ ! -f ${GPS_FILE} ]; then
		echo "Error: ${GPS_FILE} does not exist"
		continue
	fi
	
	VIN=$(cat $VIN_FILE)
	GPS_STRING=$(cat $GPS_FILE)
	
	FULL_COMMAND="curl --connect-timeout 5 -k ${HEADER},${VIN},${GPS_STRING}"
	echo "Performing command: ${FULL_COMMAND}"
	eval ${FULL_COMMAND}
 
	
done
