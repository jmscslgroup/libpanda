#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

echo "Installing/Updating Vandertest ROS packages"

source ~/.bashrc

sh uninstallRobotUpstart.sh

cd ~
if [ ! -d catkin_ws/src ]; then
	mkdir -p catkin_ws/src
fi
cd catkin_ws/src

# The following are repositories under jmscslgroup:
declare -a repositories=(can_to_ros  ghostfollower ghostfollower_max velocity_controller ghost_mode velocityramp setpointreader acc_settings)

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

# this requires credentials:
if [ -d algos-stack ]; then
	cd algos-stack
	git pull
	cd ..
else
	git clone https://github.com/CIRCLES-consortium/algos-stack
fi
cd algos-stack
#git checkout swil_rahul
#git checkout setpoint_rahul
git checkout rl-cruise


cd ~/catkin_ws
catkin_make

source devel/setup.sh
#rosrun robot_upstart uninstall can
rosrun robot_upstart install can_to_ros/launch/rl_cruise_noplanner_readonly.launch --user root



echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can

echo "----------------------------"
