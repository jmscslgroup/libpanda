#!/bin/bash

echo "=========================="
echo "Installing App simpleSend"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing scripts..."
cp /etc/libpanda.d/apps/simpleSend/simplesend.service.txt /etc/systemd/system/simplesend.service
chmod 655 /etc/systemd/system/simplesend.service

# Enable the installed services on boot:
echo " - Enabling startup scripts..."
systemctl daemon-reload # if needed
systemctl enable simplesend

