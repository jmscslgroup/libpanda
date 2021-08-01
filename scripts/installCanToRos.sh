#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
echo "Installing can_to_ros"

source ~/.bashrc

cd ~
if [ ! -d catkin_ws/src ]; then
	mkdir -p catkin_ws/src
fi
cd catkin_ws/src

git clone https://github.com/jmscslgroup/can_to_ros.git

cd ~/catkin_ws
catkin_make

source devel/setup.sh
rosrun robot_upstart install can_to_ros/launch/vehicle_control.launch --user root



echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can

echo "----------------------------"
