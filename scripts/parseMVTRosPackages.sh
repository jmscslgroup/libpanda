#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

echo "Installing/Updating Vandertest ROS packages"

#source ~/.bashrc

ROS_PACKAGE_REPOSITORY_CSV=rosRepositoriesExperimental.csv


cat $ROS_PACKAGE_REPOSITORY_CSV | tr -d " \t\r" | awk -F',' '{print $2 ": " substr($3,1,7)}' | tr '\n' ',' > rosRepoShort.txt

#cd ~
#if [ ! -d catkin_ws/src ]; then
#	mkdir -p catkin_ws/src
#fi
#cd catkin_ws/src

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
			#git pull
			#git checkout ${versionHash}
		else
			echo " - Already on the right commit!"
		fi
		cd ..
	else
		#git clone "https://github.com/${owner}/${repository}.git"
		echo "cd ${repository}"
		#git checkout ${versionHash}
		echo "cd .."
	fi
done < $ROS_PACKAGE_REPOSITORY_CSV

