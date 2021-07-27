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
git clone https://github.com/jmscslgroup/transfer_pkg.git
git clone https://github.com/jmscslgroup/followerstopperth
git clone https://github.com/jmscslgroup/margin
git clone https://github.com/jmscslgroup/ghostfollower
git clone https://github.com/jmscslgroup/ghostfollower_max
git clone https://github.com/jmscslgroup/micromodel
git clone https://github.com/jmscslgroup/micromodelv2
git clone https://github.com/jmscslgroup/followerstopperth4rl
git clone https://github.com/jmscslgroup/followerstoppermax.git
git clone https://github.com/jmscslgroup/velocity_controller
git clone https://github.com/jmscslgroup/ghost_mode
git clone https://github.com/jmscslgroup/velocityramp

# this requires credentials:
git clone https://github.com/CIRCLES-consortium/algos-stack
cd algos-stack
git checkout swil_rahul


cd ~/catkin_ws
catkin_make

source devel/setup.sh
rosrun robot_upstart install can_to_ros/launch/vehicle_control.launch --user root



echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can

echo "----------------------------"
