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
declare -a repositories=(can_to_ros explicitaccel_microaccel)

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
#rosrun robot_upstart uninstall can
rosrun robot_upstart install can_to_ros/launch/explicitaccel_microaccel_readonly.launch --user root



echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can

echo "----------------------------"
