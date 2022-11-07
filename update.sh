#!/bin/bash
# Author: Matt Bunting

echo "Running libpanda's update.sh"

if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

source /opt/ros/noetic/setup.bash

cd /home/circles/libpanda # likely unnecessary
git pull

echo "Stopping can_to_ros..."
sudo systemctl stop can

echo "Updating libpanda..."
./install.sh

cd scripts
echo "Executing installMVTRosPackages..."
./installMVTRosPackages.sh

echo "Executing install_rl_cruise_hybrid_planner..."
./install_rl_cruise_hybrid_planner.sh

echo "Starting can_to_ros..."
sudo systemctl start can

echo "libpanda update.sh done."
