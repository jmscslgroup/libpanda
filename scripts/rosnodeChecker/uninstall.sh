#!/bin/bash
# Author: Matt Nice

sudo systemctl disable rosnodeChecker.service
sudo rm /etc/systemd/system/rosnodeChecker.service
sudo rm /usr/local/sbin/rosnodeChecker.sh
sudo systemctl daemon-reload

echo "=============================="
echo "Uninstall is complete"
echo "=============================="

