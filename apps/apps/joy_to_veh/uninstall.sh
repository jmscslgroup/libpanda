#!/bin/bash

echo "=========================="
echo "Removing App joy_to_veh"


LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

# Disable the installed services:
echo " - Disabling startup scripts..."
systemctl disable joy


# Here is where we remove scripts, services, etc.
echo " - Removing scripts..."
cd
if [ "x"`systemctl list-units | grep -c joy.service` = "x1" ]; then
    echo "Uninstalling joy.service"

    source /home/$LIBPANDA_USER/catkin_ws/devel/setup.bash
    rosrun robot_upstart uninstall joy
fi

systemctl daemon-reload # if needed
