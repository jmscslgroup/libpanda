#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing circlesmanager  "

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


cp circlesmanager.py /usr/local/sbin/circlesmanager.py
cp simplePing.sh /usr/local/sbin/simplePing

cp circlesmanager.service.txt  /etc/systemd/system/circlesmanager.service
chmod 655 /etc/systemd/system/circlesmanager.service

systemctl daemon-reload

systemctl enable circlesmanager.service

systemctl restart circlesmanager.service


# Setup status files:
if [ ! -d /etc/libpanda.d ]; then
	mkdir /etc/libpanda.d
fi

# x725
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

# crazypifi
if [ ! -f /etc/libpanda.d/isaphost ]; then
	touch /etc/libpanda.d/isaphost
fi

if [ ! -f /etc/libpanda.d/isapclient ]; then
	touch /etc/libpanda.d/isapclient
fi

if [ ! -f /etc/libpanda.d/hasinternet ]; then
	touch /etc/libpanda.d/hasinternet
fi

if [ ! -f /etc/libpanda.d/hasapclients ]; then
	touch /etc/libpanda.d/hasapclients
fi

echo "Done."
echo "========================="
