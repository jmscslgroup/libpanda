#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

echo "Installing/Updating Vandertest ROS packages"

source ~/.bashrc

cd ~
if [ ! -d catkin_ws/src ]; then
	mkdir -p catkin_ws/src
fi
cd catkin_ws/src

# The following are repositories under jmscslgroup:
declare -a repositories=(can_to_ros transfer_pkg followerstopperth margin ghostfollower ghostfollower_max micromodel micromodelv2 followerstopperth4rl followerstoppermax velocity_controller ghost_mode velocityramp setpointreader followerstoppermax4rl cbf setpointreader cbf_with_error)

for repository in "${repositories[@]}"
do
	echo "Checking" $repository
	if [ -d ${repository} ]; then
		cd ${repository}
		git pull
		cd ..
	else
		git clone "https://github.com/jmscslgroup/${repository}.git"
	fi
done

#git clone https://github.com/jmscslgroup/can_to_ros.git
#git clone https://github.com/jmscslgroup/transfer_pkg.git
#git clone https://github.com/jmscslgroup/followerstopperth
#git clone https://github.com/jmscslgroup/margin
#git clone https://github.com/jmscslgroup/ghostfollower
#git clone https://github.com/jmscslgroup/ghostfollower_max
#git clone https://github.com/jmscslgroup/micromodel
#git clone https://github.com/jmscslgroup/micromodelv2
#git clone https://github.com/jmscslgroup/followerstopperth4rl
#git clone https://github.com/jmscslgroup/followerstoppermax.git
#git clone https://github.com/jmscslgroup/velocity_controller
#git clone https://github.com/jmscslgroup/ghost_mode
#git clone https://github.com/jmscslgroup/velocityramp

## for testing on 8/2/2021 we will be changing branches for all-in-one startup script:
#cd can_to_ros
#git checkout experiments_monday
#cd ..


# Commented out by George Gunter
# # this requires credentials:
# if [ -d algos-stack ]; then
# 	cd algos-stack
# 	git pull
# 	cd ..
# else
# 	git clone https://github.com/CIRCLES-consortium/algos-stack
# fi
# cd algos-stack
# #git checkout swil_rahul
# git checkout setpoint_rahul


cd ~/catkin_ws
catkin_make

source devel/setup.sh
rosrun robot_upstart uninstall can
rosrun robot_upstart install can_to_ros/launch/vehicle_control_integrated_cbf_with_error.launch --user root



echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can

echo "----------------------------"
