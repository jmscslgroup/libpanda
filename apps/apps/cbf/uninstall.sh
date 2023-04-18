#!/bin/bash

echo "=========================="
echo "Removing App cbf"


# Disable the installed services:
echo " - Disabling startup scripts..."
systemctl disable can

# Here is where we remove scripts, services, etc.
echo " - Removing scripts..."
cd
if [ "x"`systemctl list-units | grep -c can.service` = "x1" ]; then
    echo "Uninstalling can.service"
    rosrun robot_upstart uninstall can
fi

systemctl daemon-reload # if needed

