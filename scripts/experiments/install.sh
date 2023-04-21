#!/bin/bash
# Author: Matt Bunting

echo "=============================="
echo "Installing experiment scripts!"

PI_USER=$(cat /etc/libpanda.d/libpanda_usr)
HOME_DIR=/home/$PI_USER

cp monitor0802_controller.sh $HOME_DIR/monitor.sh
cp run0806_controller.sh $HOME_DIR/
cp run0806_controller_phase_2.sh $HOME_DIR/
