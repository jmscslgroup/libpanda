#!/bin/bash
# Author: Matt Bunting

# if you follow conventions correctly, you only need to change the below two parameters

LAUNCH_FILE=explicit_cruise_planner_readonly.launch

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

# uninstall the current installations
source ~/.bashrc
sh uninstallRobotUpstart.sh

echo "Installing/Updating Vandertest ROS packages"

cd ~
if [ ! -d catkin_ws/src ]; then
    mkdir -p catkin_ws/src
fi
cd catkin_ws/src

# The following are repositories under jmscslgroup:
declare -a repositories=(can_to_ros explicit_cruise_planner)

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
rosrun robot_upstart uninstall can
rosrun robot_upstart install can_to_ros/launch/${LAUNCH_FILE} --user root



echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can

echo "----------------------------"
