#!/bin/bash
# Author: Matt Bunting

#LAUNCH_FILE=cbftest.launch
LAUNCH_FILE=vehicle_control_integrated_cbf.launch
#LAUNCH_FILE=vehicle_control_integrated_cbf_with_error.launch

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

# uninstall the current installations
source ~/.bashrc

pushd ~/catkin_ws
source devel/setup.sh
#rosrun robot_upstart uninstall can
rosrun robot_upstart install can_to_ros/launch/${LAUNCH_FILE} --user root

echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can
popd

echo "----------------------------"

