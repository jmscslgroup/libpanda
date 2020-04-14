#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing panda services"

if [ ! -d /etc/libpanda.d ]; 
	mkdir /etc/libpanda.d
fi

if [ ! -f /etc/libpanda.d/vin ]; then
	touch /etc/libpanda.d/vin
fi

cp pandarecord.sh /usr/local/sbin/pandarecord

cp pandasettime.service.txt  /etc/systemd/system/pandasettime.service
cp pandarecord.service.txt  /etc/systemd/system/pandarecord.service
chmod 655 /etc/systemd/system/pandasettime.service
chmod 655 /etc/systemd/system/pandarecord.service

systemctl daemon-reload

systemctl enable pandasettime.service
systemctl enable pandarecord.service

echo "Done."
echo "========================="
