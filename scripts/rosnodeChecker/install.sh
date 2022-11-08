#!/bin/bash
# Author: Matt Nice

echo "========================="
echo "Installing rosnodeChecker"

declare -a depencencies=()
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

sudo cp rosnodeChecker.sh /usr/local/sbin/rosnodeChecker.sh

sudo cp rosnodeChecker.service.txt /etc/systemd/system/rosnodeChecker.service
sudo chmod 655 /etc/systemd/system/rosnodeChecker.service

systemctl daemon-reload

systemctl disable rosnodeChecker.service

echo "Done."
echo "========================="
