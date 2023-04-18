#!/bin/bash

echo "=========================="
echo "Installing App mvt"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing scripts with harcoded directories..."

$LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
cd $LIBPANDA_SRC/scripts
echo "Executing installMVTRosPackages..."
./installMVTRosPackages.sh

echo "Uninstalling original can service if it exists..."
./uninstallRobotUpstart.sh

echo "Executing installMVTController..."
./installMVTController.sh

