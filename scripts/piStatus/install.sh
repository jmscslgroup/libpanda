#!/bin/bash
# Author: Matt Nice

echo "========================="
echo "Installing pi-status "

declare -a depencencies=(ifstat)
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


cp getPiStatus.sh /usr/local/sbin/getPiStatus.sh

cp piStatus.service.txt  /etc/systemd/system/piStatus.service
chmod 655 /etc/systemd/system/piStatus.service

systemctl daemon-reload

systemctl enable piStatus.service

systemctl restart piStatus.service


# Setup status files:
# json file
if [ ! -f /etc/libpanda.d/piStatus.json ]; then
	touch /etc/libpanda.d/piStatus.json
fi

echo "Done."
echo "========================="
