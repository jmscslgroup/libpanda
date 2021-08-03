#!/bin/bash

cd /home/circles/catkin_ws/
source devel/setup.bash
roslaunch transfer_pkg micromodelv2_readonly.launch hwil:=true readonly:=false
