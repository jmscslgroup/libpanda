#!/bin/bash
# Author: Alex Richardson

echo "========================="
echo "Installing live-tracker!  "

declare -a dependencies_apt=(python3-pandas)
toInstall_apt=()
echo "Dependencies:" ${dependencies_apt[@]}
for dependency in "${dependencies_apt[@]}"
do
	echo "Checking" $dependency
	if [ $(dpkg-query -W -f='${Status}' $dependency 2>/dev/null | grep -c "ok installed") -eq 0 ];
	then
		echo "not installed:" $dependency
		toInstall_apt+=($dependency)
	fi
done
echo ${toInstall_apt[@]}

if [ ${#toInstall_apt[@]} -ne 0 ];
then
	apt-get update
	apt-get install -y ${toInstall_apt[@]}
fi

pip3 install strymread-lite==0.4.17.rc0
cp live-tracker.py /usr/local/sbin/live-tracker
chmod +x /usr/local/sbin/live-tracker
mkdir -p /var/panda/CyverseData/JmscslgroupData/live_tracker_temporary_files/
chown circles -R /var/panda/CyverseData/JmscslgroupData/live_tracker_temporary_files/

cp live-tracker.service.txt  /etc/systemd/system/live-tracker.service
chmod 655 /etc/systemd/system/live-tracker.service

systemctl daemon-reload

systemctl enable live-tracker.service
systemctl restart live-tracker.service

echo "Done."
echo "========================="
