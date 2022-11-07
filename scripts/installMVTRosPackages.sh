#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

echo "Installing/Updating Vandertest ROS packages"

source ~/.bashrc

cd ~
if [ ! -d catkin_ws/src ]; then
	mkdir -p catkin_ws/src
fi
cd catkin_ws/src

# The following are repositories under jmscslgroup:
declare -a repositories=("jmscslgroup,can_to_ros,63130161901da3e286c140ba6426630fcc82cd31" "jmscslgroup,setpointreader,bf9116d7e7a8284527e1d94537de1228c2b3abb8" "jmscslgroup,live_tracker,32dcb38decc03646e2dbeec9a3e3487411e0aa54" "CIRCLES-consortium,algos-stack,f95d26f78ffed06cd21fa73fcf414d5d5e5042a2")

for repositoryAndHash in "${repositories[@]}"
do
	IFS=","
	set -- $repositoryAndHash # convert the "tuple" into the param args $1 $2...
#    echo $1 and $2
    owner=$1
    repository=$2
    versionHash=$3
	echo "Checking ${owner}/${repository} with hash ${versionHash}"
	if [ -d ${repository} ]; then
		cd ${repository}
		GIT_VERSION=$(git rev-parse HEAD | tr -d "\n\r")
		if [ "$GIT_VERSION" != "$versionHash" ]; then
			echo " - Mismatch in hash, checking out specified commit..."
			git pull
			git checkout ${versionHash}
		else
			echo " - Already on the right commit!"
		fi
		cd ..
	else
		git clone "https://github.com/${owner}/${repository}.git"
		git checkout ${versionHash}
	fi
done


# Build:
cd ~/catkin_ws
catkin_make

# ROS upstart install:
source devel/setup.sh
rosrun robot_upstart install can_to_ros/launch/vehicle_interface.launch --user root

echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable can


# Hash saving:
echo "Saving hashes to /etc/libpanda.d/git_hashes/"
cd src
sudo mkdir -p /etc/libpanda.d/git_hashes
for repositoryAndHash in "${repositories[@]}"
do
	IFS=","
	set -- $repositoryAndHash # convert the "tuple" into the param args $1 $2...
#    echo $1 and $2
	owner=$1
    repository=$2
    versionHash=$3
	cd ${repository}
	GIT_VERSION=$(git rev-parse HEAD | tr -d "\n\r")
	sudo sh -c "echo -n ${GIT_VERSION} > /etc/libpanda.d/git_hashes/${repository}"
	cd ..
done

echo "----------------------------"
