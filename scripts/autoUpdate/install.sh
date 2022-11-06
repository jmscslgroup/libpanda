#!/bin/bash
# Author: Alex Richardson

echo "========================="
echo "Installing pi-status "

declare -a depencencies=(ifstat wget)
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

sudo cp autoUpdate.sh /usr/local/sbin/autoUpdate.sh

sudo cp autoUpdate.service.txt /etc/systemd/system/autoUpdate.service
sudo chmod 655 /etc/systemd/system/autoUpdate.service

systemctl daemon-reload

systemctl enable autoUpdate.service

echo "Done."
echo "========================="
