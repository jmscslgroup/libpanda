#!/bin/bash
# Author: Matt Bunting, Matt Nice

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

echo "Installing/Updating CBF ROS packages"

source ~/.bashrc

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
ROS_PACKAGE_REPOSITORY_CSV=/etc/libpanda.d/apps/joy_to_veh/rosRepositories.csv

cat $ROS_PACKAGE_REPOSITORY_CSV | tr -d " \t\r" | awk -F',' '{print $2 ": " substr($3,1,7)}' | tr '\n' ',' > $LIBPANDA_SRC/scripts/rosRepoShort.txt

cd ~
if [ ! -d catkin_ws/src ]; then
    mkdir -p catkin_ws/src
    source /opt/ros/noetic/setup.bash   # will it always be noetic, in this location?
fi
cd catkin_ws/src

while IFS= read -r LINE
do
    #echo $LINE
    LINE=$(echo $LINE | tr -d [:space:])
    IFS=","
    set -- $LINE
    IFS=
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
        cd ${repository}
        git checkout ${versionHash}
        cd ..
    fi
done < $ROS_PACKAGE_REPOSITORY_CSV

#TODO add 'sudo apt-get install libdiagnostic-updater-dev'

# Build:
cd ~/catkin_ws
source devel/setup.sh
catkin_make

# ROS upstart install:
#rosrun robot_upstart install can_to_ros/launch/vehicle_interface.launch --user root

#echo "Enabling can_to_ros startup script"
#sudo systemctl daemon-reload
#sudo systemctl enable can


# Hash saving:
echo "Saving hashes to /etc/libpanda.d/git_hashes/"
cd src
sudo mkdir -p /etc/libpanda.d/git_hashes
while IFS= read -r LINE
do
    #echo $LINE
    LINE=$(echo $LINE | tr -d [:space:])
    IFS=","
    set -- $LINE
    IFS=
    owner=$1
    repository=$2
    versionHash=$3

    cd ${repository}
    GIT_VERSION=$(git rev-parse HEAD | tr -d "\n\r")
    sudo sh -c "echo -n ${GIT_VERSION} > /etc/libpanda.d/git_hashes/${repository}"
    cd ..

    echo "Saved hash for ${owner}/${repository}"
done < $ROS_PACKAGE_REPOSITORY_CSV

echo "----------------------------"
