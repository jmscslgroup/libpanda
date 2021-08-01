#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing circles-ui!       "

declare -a depencencies=(python3-pip)
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

pip3 install flexx


cp circles-ui.py /usr/local/sbin/circles-ui.py

cp circles-ui.service.txt  /etc/systemd/system/circles-ui.service
chmod 655 /etc/systemd/system/circles-ui.service

systemctl daemon-reload

systemctl enable circles-ui.service

systemctl restart circles-ui.service

echo "Done."
echo "========================="
