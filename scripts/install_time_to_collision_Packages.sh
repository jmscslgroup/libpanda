#!/bin/bash
# Author: Matt Bunting, George Gunter, Matt Nice

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
declare -a repositories=(can_to_ros transfer_pkg followerstopperth margin ghostfollower ghostfollower_max micromodel micromodelv2 followerstopperth4rl followerstoppermax velocity_controller ghost_mode velocityramp setpointreader followerstoppermax4rl cbf delay_brake setpointreader)

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

cd ~/catkin_ws
catkin_make

source devel/setup.sh
rosrun robot_upstart uninstall can #Added by George Gunter and Matt Nice
rosrun robot_upstart install can_to_ros/launch/vehicle_control_integrated_time_to_collision.launch --user root



echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can

echo "----------------------------"