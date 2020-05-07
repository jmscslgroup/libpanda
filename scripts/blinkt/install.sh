#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing blinkt!       "

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

pip3 install blinkt


cp blinktStatus.py /usr/local/sbin/blinktStatus.py

cp blinkt.service.txt  /etc/systemd/system/blinkt.service
chmod 655 /etc/systemd/system/blinkt.service

systemctl daemon-reload

systemctl enable blinkt.service

systemctl restart blinkt.service


# Setup status files:
if [ ! -d /etc/libpanda.d ]; then
	mkdir /etc/libpanda.d
fi

if [ ! -f /etc/libpanda.d/x725hasexternalpower ]; then
	touch /etc/libpanda.d/x725hasexternalpower
fi

if [ ! -f /etc/libpanda.d/x725batteryvoltage ]; then
	touch /etc/libpanda.d/x725batteryvoltage
fi

if [ ! -f /etc/libpanda.d/x725capacity ]; then
	touch /etc/libpanda.d/x725capacity
fi

if [ ! -f /etc/libpanda.d/x725batterycurrent ]; then
	touch /etc/libpanda.d/x725batterycurrent
fi

echo "Done."
echo "========================="
