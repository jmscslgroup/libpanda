#!/bin/bash

cd /home/circles/catkin_ws/
source devel/setup.bash
rostopic echo /cmd_accel
