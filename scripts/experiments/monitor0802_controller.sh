#!/bin/bash

PI_USER=$(cat /etc/libpanda.d/libpanda_usr)

cd /home/$PI_USER/catkin_ws/
source devel/setup.bash
rostopic echo /cmd_accel
