#!/bin/bash

echo "=========================="
echo "Installing App cbf"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing ROS packages for CBF..."

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

cd /etc/libpanda.d/apps/cbf
runuser -u $LIBPANDA_USER -c ./installRosPackagesForCBF.sh



echo "Uninstalling original can service if it exists..."
cd $LIBPANDA_SRC/scripts
./uninstallRobotUpstart.sh

echo "Installing CBF demo..."
cd /etc/libpanda.d/apps/cbf
runuser -u $LIBPANDA_USER -c ./installCBFController.sh



