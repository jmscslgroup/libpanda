#!/bin/bash
# Author: Matt Bunting

>&2 echo "Running libpanda's update.sh"

if [[ $EUID == 0 ]];
  then >&2 echo "Do NOT run this script as root"
  exit
fi

source /opt/ros/noetic/setup.bash

cd /home/circles/libpanda # likely unnecessary
git pull

INSTALLED_LIBPANDA_GIT_VERSION=$(pandaversion)
CURRENT_LIBPANDA_GIT_VERSION=$(git rev-parse HEAD | tr -d "\n\r")

if [ "$INSTALLED_LIBPANDA_GIT_VERSION" = "$CURRENT_LIBPANDA_GIT_VERSION" ];
then
	>&2 echo "libpanda is already up to date!"
	exit 1
fi


>&2 echo "Stopping can_to_ros..."
sudo systemctl stop can

>&2 echo "Updating libpanda..."
./install.sh

cd scripts
>&2 echo "Executing installMVTRosPackages..."
./installMVTRosPackages.sh

>&2 echo "Executing install_rl_cruise_hybrid_planner..."
./install_rl_cruise_hybrid_planner.sh

>&2 echo "Starting can_to_ros..."
sudo systemctl start can

>&2 echo "libpanda update.sh done."
