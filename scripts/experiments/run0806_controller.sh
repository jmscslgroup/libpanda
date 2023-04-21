#!/bin/bash

PI_USER=$(cat /etc/libpanda.d/libpanda_usr)

cd /home/$PI_USER/catkin_ws/
source devel/setup.bash

#roslaunch transfer_pkg rl0805_readonly.launch hwil:=true readonly:=false
roslaunch transfer_pkg rl0805_v3_readonly.launch hwil:=true readonly:=false
