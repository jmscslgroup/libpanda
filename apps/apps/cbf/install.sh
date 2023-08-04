#!/bin/bash

echo "=========================="
echo "Installing App cbf"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing ROS packages for CBF..."

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

source /home/$LIBPANDA_USER/.bashrc

runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/cbf/installRosPackagesForCbf.sh

#echo "Uninstalling original can service if it exists..."
#cd $LIBPANDA_SRC/scripts
#./uninstallRobotUpstart.sh

echo "Installing CBF demo..."
runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/cbf/installCbfController.sh
