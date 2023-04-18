#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing bluezone"

declare -a depencencies=(git python3-dbus)
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

if [ ! -d /var/panda/cputemp ];
then
    mkdir -p /var/panda
        
    CURRENT_DIR=`pwd`
    cd /var/panda/
#    git clone https://github.com/Douglas6/cputemp
    git clone https://github.com/jmscslgroup/cputemp   # moved to a fork to fix issues
    cd ${CURRENT_DIR}
else
    # update existing:
    CURRENT_DIR=`pwd`
    cd /var/panda/cputemp
    git pull
    cd ${CURRENT_DIR}
fi

# we need experimental mode for BLE GATT stuff
sed -i 's/.*ExecStart=\/usr\/libexec\/bluetooth\/bluetoothd.*/ExecStart=\/usr\/libexec\/bluetooth\/bluetoothd -E/' /etc/systemd/system/dbus-org.bluez.service

sed -i 's/.DiscoverableTimeout = .*/DiscoverableTimeout = 0/' /etc/bluetooth/main.conf
sed -i 's/.PairableTimeout = .*/PairableTimeout = 0/' /etc/bluetooth/main.conf

sudo cp bluezone.sh /usr/local/sbin/bluezone
sudo cp bluezone.py /usr/local/sbin/bluezone.py
sudo cp bzwifihelper.sh /usr/local/sbin/bzwifihelper

sudo cp bluezone.service.txt /etc/systemd/system/bluezone.service
sudo chmod 655 /etc/systemd/system/bluezone.service

systemctl daemon-reload

systemctl enable bluezone.service


grep -qxF "bluetoothctl discoverable on" /etc/rc.local || echo "bluetoothctl discoverable on" >> /etc/rc.local

echo "Done."
echo "========================="
