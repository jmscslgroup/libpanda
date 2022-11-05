#!/bin/bash
# Author: Matt Bunting
#  This Noetic install version was created for the latest 64-bit raspbian image (debian-bullseye) since there are dependency issues with non-python3 modules.


echo "----------------------------"

if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

echo "Installing ROS Noetic"


# These steps are copied from here, folling Buster: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi

# Noetic install: the above link was modified with information frm the following: http://wiki.ros.org/Installation/Source

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get upgrade -y

#sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
#sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake python-empy python-setuptools catkin # python3-catkin python3-empy python3-catkin-pkg python3-yaml
sudo apt-get install -y python3-rosdep python3-rosinstall-generator python3-vcstool build-essential cmake python3-wstool python3-rosinstall
#extras for noetic:
sudo apt-get install -y python3-empy python3-catkin python3-catkin-pkg python3-yaml python3-rosdep

sudo rosdep init
rosdep update

mkdir -p ~/ros_install_ws
cd ~/ros_install_ws

mkdir -p src

# If installing multiple times, we need to remove the .rosinstall file (or modify it)
if [ -f src/.rosinstall ]; then
        rm src/.rosinstall
fi

#rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
# The below is edited from above simply to include common_msgs
#rosinstall_generator ros_comm common_msgs robot_upstart --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
rosinstall_generator ros_comm common_msgs robot_upstart --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall
#wstool init src melodic-ros_comm-wet.rosinstall
wstool init src noetic-ros_comm-wet.rosinstall

cd ~/ros_install_ws
#rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r #--os=debian:buster

# The below does not allow other packages to be compiled against common_msgs, see above instead
# Bunting: I included the below block for also installing common ROS messages
# I also feel like this should be installed via apt, not sure if that's possible
#cd src
#git clone https://github.com/ros/common_msgs.git
#cd ..

#sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j4 #-DPYTHON_EXECUTABLE=/usr/bin/python3
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3
# Noetic note: Setting python3 is needed for catkin_make_isolated to fix python3 dependency issues


#source /opt/ros/melodic/setup.bash
source /opt/ros/noetic/setup.bash
#echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

echo "----------------------------"
