#!/bin/bash

PI_USER=$(cat /etc/libpanda.d/libpanda_usr)

cd /home/$PI_USER/catkin_ws/
source devel/setup.bash
#roslaunch transfer_pkg micromodelv2_readonly.launch hwil:=true readonly:=false
# The anobve controller was changed last minute to the following:
roslaunch transfer_pkg rl0719_readonly.launch hwil:=true readonly:=false
