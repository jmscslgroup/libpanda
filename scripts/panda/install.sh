#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing panda services"

echo "Configuring VIN file."

if [ ! -d /etc/libpanda.d ]; then
	mkdir /etc/libpanda.d
fi

if [ ! -f /etc/libpanda.d/vin ]; then
	touch /etc/libpanda.d/vin
fi

if [ ! -s /etc/libpanda.d/vin ]; then
	choice=""

	while [ "${choice}" != "n" ] && [ "${choice}" != "y" ]
	do
		read -n 1 -p " - VIN file is empty, enter VIN now? [y/n]:" choice
		echo ""

		echo "you entered: ${name}"
	done

	if [ "${choice}" = "y" ];
	then
		read -p " - Enter VIN:" vin

		echo "${vin}" > /etc/libpanda.d/vin
	fi
fi

echo "Configuring startup scripts"

cp pandarecord.sh /usr/local/sbin/pandarecord

cp pandasettime.service.txt  /etc/systemd/system/pandasettime.service
cp pandarecord.service.txt  /etc/systemd/system/pandarecord.service
chmod 655 /etc/systemd/system/pandasettime.service
chmod 655 /etc/systemd/system/pandarecord.service

systemctl daemon-reload

echo "Enabling startup scripts"

systemctl enable pandasettime.service
systemctl enable pandarecord.service

echo "Done."
echo "========================="
