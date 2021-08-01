#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing gps-tracker!  "

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


cp gps-tracker.sh /usr/local/sbin/gps-tracker
chmod +x /usr/local/sbin/gps-tracker

cp gps-tracker.service.txt  /etc/systemd/system/gps-tracker.service
chmod 655 /etc/systemd/system/gps-tracker.service

systemctl daemon-reload

systemctl enable gps-tracker.service
systemctl restart gps-tracker.service

echo "Done."
echo "========================="
