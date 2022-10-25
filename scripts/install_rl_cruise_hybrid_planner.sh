#!/bin/bash
# Author: Matt Bunting

# if you follow conventions correctly, you only need to change the below two parameters

UPDATE_SCRIPT=update_rl_cruise_hybrid_planner.sh
LAUNCH_FILE=rl_cruise_hybrid_planner_readonly.launch

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

# uninstall the current installations
source ~/.bashrc
sh uninstallRobotUpstart.sh


# run the update script
./${UPDATE_SCRIPT}

pushd ~/catkin_ws
source devel/setup.sh
#rosrun robot_upstart uninstall can
rosrun robot_upstart install can_to_ros/launch/${LAUNCH_FILE} --user root

echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can
popd

echo "----------------------------"
