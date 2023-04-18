#!/bin/bash

echo "=========================="
echo "Installing App cbf"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing scripts with harcoded directories..."

cd /home/circles/libpanda/scripts
#echo "Executing installMVTRosPackages..."
#./installMVTRosPackages.sh

echo "Uninstalling original can service if it exists..."
./uninstallRobotUpstart.sh

#echo "Executing installMVTController..."
#./installMVTController.sh
#./installMVTController.sh

