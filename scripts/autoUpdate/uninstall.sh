#!/bin/bash
# Author: Alex Richardson

sudo systemctl disable autoUpdate.service
sudo rm /etc/systemd/system/autoUpdate.service
sudo rm /usr/local/sbin/autoUpdate.sh
sudo systemctl daemon-reload

echo "=============================="
echo "Uninstall is complete"
echo "=============================="

