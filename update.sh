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

INSTALLED_LIBPANDA_GIT_VERSION=$(cat /etc/libpanda.d/libpanda_version)
CURRENT_LIBPANDA_GIT_VERSION=$(git rev-parse HEAD | tr -d "\n\r")

if [ "$INSTALLED_LIBPANDA_GIT_VERSION" = "$CURRENT_LIBPANDA_GIT_VERSION" ];
then
	echo "Software stack is already up to date!"
	exit 1
fi

echo "Update needed, Git hash mismatch: $INSTALLED_LIBPANDA_GIT_VERSION != $CURRENT_LIBPANDA_GIT_VERSION"


#echo "Stopping can_to_ros..."
#sudo systemctl stop rosnodeChecker
#sudo systemctl stop can
#sudo sh -c "echo 'ROS has been stopped for update!' > /etc/libpanda.d/logMessage"
echo "Stopping app..."
sudo libpanda-app-manager -k
sudo sh -c "echo 'App has been stopped for update!' > /etc/libpanda.d/logMessage"

echo "Updating libpanda..."
./install.sh

#cd scripts
#echo "Executing installMVTRosPackages..."
#./installMVTRosPackages.sh
#
#echo "Uninstalling original can service if it exists..."
#./uninstallRobotUpstart.sh
#
#echo "Executing installMVTController..."
#./installMVTController.sh
echo "Updating Apps..."
cd apps
sudo ./update.sh
cd ..

# Do the following at the end of installs to ensure all other processes completed
echo "Saving current pandaversion to /etc/libpanda.d/libpanda_version"
sudo sh -c "pandaversion > /etc/libpanda.d/libpanda_version"

#echo "Starting can_to_ros..."
#sudo systemctl start can
#sudo systemctl start rosnodeChecker
echo "Starting app..."
sudo libpanda-app-manager -s

echo "libpanda update.sh done."
