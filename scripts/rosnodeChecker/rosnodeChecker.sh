#!/bin/bash
#Author: Matt Nice

source /opt/ros/noetic/setup.bash

while :
do
sleep 10

ROS_LIST=$(rosnode list)
ROS_PASS=$?

if [ $ROS_PASS -eq "1" ]
then
	echo "ROS down!" > /etc/libpanda.d/logMessage
	continue
fi

ROS_NODES=($(echo ${ROS_LIST} | tr -d '[],'))

subs_alive=0
vehicle_interface_alive=0
live_tracker_alive=0
nissan_target_to_buttons_alive=0

#CONTROLS_ALLOWED_MSG=$(rostopic echo /car/libpanda/controls_allowed -n 1 | grep data | sed 's/data: //g' | tr -d '\t\r\n ')
CONTROLS_ALLOWED_MSG=$(rosparam get /SP_CONTROL_ALLOWABLE | tr [:upper:] [:lower:])

for node in ${ROS_NODES[*]};
do
	if [ "$node" = "/subs" ]; then
		subs_alive=1
	fi
	if [ "$node" = "/vehicle_interface" ]; then
		vehicle_interface_alive=1
	fi
	if [ "$node" = "/live_tracker" ]; then
		live_tracker_alive=1
	fi
	if [ "$node" = "/nissan_target_speed_to_acc_buttons" ]; then
		nissan_target_to_buttons_alive=1
	fi

done

if [[ "$subs_alive" -eq "1" ]] && [[ "$vehicle_interface_alive" -eq "1" ]] && [[ "$live_tracker_alive" -eq "1" ]] && [[ "$nissan_target_to_buttons_alive" -eq "1" ]]; then
	if [[ "$CONTROLS_ALLOWED_MSG" = "true" ]]; then
		echo "nominal state - controls allowed!" > /etc/libpanda.d/logMessage
	else
		echo "nominal state - controls not allowed!" > /etc/libpanda.d/logMessage
	fi
	#echo "nominal state!" > /etc/libpanda.d/logMessage
else
	echo "rosnode(s) are down"
	echo "These rosnodes are down: " > /etc/libpanda.d/logMessage
	if [[ "$subs_alive" -eq "0" ]]; then
		sed -i.bck '$s/$/subs node is down. /' /etc/libpanda.d/logMessage
	fi
	if [[ "$vehicle_interface_alive" -eq "0" ]]; then
		sed -i.bck '$s/$/vehicle_inferface node is down. /' /etc/libpanda.d/logMessage
	fi
	if [[ "$live_tracker_alive" -eq "0" ]]; then
		sed -i.bck '$s/$/live_tracker node is down. /' /etc/libpanda.d/logMessage
	fi
	if [[ "$nissan_target_to_buttons_alive" -eq "0" ]]; then
		sed -i.bck '$s/$/nissan_target_speed_to_acc_buttons node is down. /' /etc/libpanda.d/logMessage
	fi
fi

done
