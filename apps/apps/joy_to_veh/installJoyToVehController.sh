#!/bin/bash
# Author: Matt Nice

LAUNCH_FILE=joy_to_veh.launch
REPO=joy_to_veh

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

# uninstall the current installations
source ~/.bashrc

pushd ~/catkin_ws
source devel/setup.sh
rosrun robot_upstart install ${REPO}/launch/${LAUNCH_FILE} --user root

echo "Enabling joy_to_veh startup script"
sudo systemctl daemon-reload
sudo systemctl enable joy
popd

echo "----------------------------"
