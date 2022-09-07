#!/bin/bash


# check to see if pandarecord.service is installed
if [ "x"`systemctl list-units | grep -c pandarecord.service` == "x1" ]; then
	echo "Uninstalling pandarecord.service"
	rosrun robot_upstart uninstall pandarecord 
fi

# check to see if can.service is installed
if [ "x"`systemctl list-units | grep -c can.service` == "x1" ]; then
	echo "Uninstalling can.service"
	rosrun robot_upstart uninstall can
fi

echo "Done uninstalling services"
