#!/bin/bash

if [ ! -d /etc/libpanda.d ]; then
	mkdir /etc/libpanda.d
fi

cp scripts/vinToHostname.sh /usr/sbin/vinToHostname
if [[ ! -f "/etc/libpanda.d/vin" ]]; then
	echo "circles" > /etc/libpanda.d/vin
fi

./build.sh
cd scripts/blinkt
./install.sh
cd ../crazypifi
./install.sh
cd ../xUps
./install.sh
cd ../circles-ui
./install.sh
cd ../circlesmanager
./install.sh
cd ../..



systemctl enable ssh
systemctl start ssh

#cp scripts/addWifiAp.sh ../
#../setVin.sh
#../addWifiAp.sh
