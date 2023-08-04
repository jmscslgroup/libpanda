#!/bin/bash

echo "=========================="
echo "Installing App joy_to_veh"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing ROS packages for joy_to_veh"

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

source /home/$LIBPANDA_USER/.bashrc

apt-get install -y libdiagnostic-updater-dev

runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/joy_to_veh/installRosPackagesForJoyToVeh.sh

#echo "Uninstalling original can service if it exists..."
#cd $LIBPANDA_SRC/scripts
#./uninstallRobotUpstart.sh

echo "Installing joy_to_veh demo..."
runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/joy_to_veh/installJoyToVehController.sh
