#!/bin/bash

# Author: Jonathan Sprinkle, Jonathan Lee
# Based on example by Rahul Bhadani for the live viewer
# 
# format:
# 
# example:
# 

SP_BASE_URL="http://ransom.isis.vanderbilt.edu:80/inrix/api/target.php"
LN_CTRL_URL="http://ransom.isis.vanderbilt.edu:80/inrix/api/lane_assignments.php?vin="

SPEEDPLANNER_FILE="/etc/libpanda.d/speed_planner.json"
SPEEDPLANNER_FILE_TMP=$(mktemp "/tmp/speed_planner_file.XXXXXXXX")
LN_CTRL_FILE="/etc/libpanda.d/lane_control_allowable.json"
LN_CTRL_FILE_TMP=$(mktemp "/tmp/lane_control_allowable_file.XXXXXXXX")

VIN_FILE="/etc/libpanda.d/vin"
VIN=$(<${VIN_FILE})
while [[ $VIN = "circles" ]];
do
	sleep 1
	VIN=$(<${VIN_FILE})
done

while true;
do
	sleep 10
	
	# TODO: replace with real string once file is created/generated correctly
	PLANNER_STRING=$(echo "test_url" | head)
	
	FULL_COMMAND="curl --connect-timeout 10 -k ${SP_BASE_URL}"
	echo "Performing command: ${FULL_COMMAND}"
	$(${FULL_COMMAND} > ${SPEEDPLANNER_FILE_TMP})
	mv $SPEEDPLANNER_FILE_TMP $SPEEDPLANNER_FILE

	FULL_COMMAND="curl --connect-timeout 10 -k ${LN_CTRL_URL}${VIN}"
	echo "Performing command: ${FULL_COMMAND}"
	$(${FULL_COMMAND} > ${LN_CTRL_FILE_TMP})
	mv $LN_CTRL_FILE_TMP $LN_CTRL_FILE
	
	source /opt/ros/noetic/setup.bash

	# now that we have a new file, re-calculate our speed
	#/usr/local/sbin/publish-speed-plan.py
	
done
