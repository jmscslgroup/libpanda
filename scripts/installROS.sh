#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
echo "Installing ROS Melodic"

# These steps are copied from here, folling Buster: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi


sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
#sudo apt-get upgrade

sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake python-empy python3-empy python-setuptools catkin python3-catkin python3-catkin-pkg python3-yaml

sudo rosdep init
rosdep update

mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

mkdir -p src

# If installing multiple times, we need to remove the .rosinstall file (or modify it)
if [ -f src/.rosinstall ]; then
	rm src/.rosinstall
fi

#rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
# The below is edited from above simply to include common_msgs
rosinstall_generator ros_comm common_msgs robot_upstart --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
wstool init src melodic-ros_comm-wet.rosinstall

cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster

# The below does not allow other packages tobe compiled against common_msgs, see above instead
# Bunting: I included the below block for also installing common ROS messages
# I also feel like this should be instlalled via apt, not sure if that's possible
#cd src
#git clone https://github.com/ros/common_msgs.git
#cd ..

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3

source /opt/ros/melodic/setup.bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

echo "----------------------------"
