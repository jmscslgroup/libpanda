#!/bin/bash
# Author: Matt Bunting

sudo systemctl disable bluezone.service
sudo rm /etc/systemd/system/bluezone.service
sudo rm /usr/local/sbin/bluezone.sh
sudo systemctl daemon-reload

echo "=============================="
echo "Uninstall is complete"
echo "=============================="

