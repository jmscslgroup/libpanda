#!/bin/bash
# Author: Jonathan Sprinkle, based on example of Matt Bunting

echo "========================="
echo "Installing circlesplanner!  "

declare -a depencencies=( )
toInstall=()
echo "Dependencies:" ${depencencies[@]}
for dependency in "${depencencies[@]}"
do
	echo "Checking" $dependency
	if [ $(dpkg-query -W -f='${Status}' $dependency 2>/dev/null | grep -c "ok installed") -eq 0 ];
	then
		echo "not installed:" $dependency
		toInstall+=($dependency)
	fi
done
echo ${toInstall[@]}

if [ ${#toInstall[@]} -ne 0 ];
then
	apt-get update
	apt-get install -y ${toInstall[@]}
fi


cp speed-planner.sh /usr/local/sbin/speed-planner
chmod +x /usr/local/sbin/speed-planner

cp circlesplanner.service.txt  /etc/systemd/system/circlesplanner.service
chmod 655 /etc/systemd/system/circlesplanner.service

systemctl daemon-reload

systemctl enable circlesplanner.service
systemctl restart circlesplanner.service

echo "Done."
echo "========================="
