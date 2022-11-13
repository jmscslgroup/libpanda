#!/bin/bash
# Author: Matt Bunting

# Change params below, assuming you followed conventions, this should be
# all you need to change IF YOU ARE USING ONNX MODELS for your controller

PRETTY_STRING="RL Cruise Hybrid Planner"
ALGOS_STACK_BRANCH=rl_cruise_hybrid

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

echo "Installing/Updating ${PRETTY_STRING} packages"

source ~/.bashrc

sh uninstallRobotUpstart.sh

cd ~
if [ ! -d catkin_ws/src ]; then
	mkdir -p catkin_ws/src
fi
cd catkin_ws/src

# The following are repositories under jmscslgroup:
declare -a repositories=(can_to_ros live_tracker)

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
# checkout the repository branch we need
git checkout ${ALGOS_STACK_BRANCH}

cd ~/catkin_ws
catkin_make

echo "----------------------------"

