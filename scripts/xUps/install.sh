#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing x725monitor   "

declare -a depencencies=(python3-smbus)
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

#g++ x725power.cpp -o x725power
#mv x725power /usr/local/bin/x725power

echo "Enabling i2c-dev module"
grep -qxF 'i2c-dev' /etc/modules || echo 'i2c-dev' >> /etc/modules
echo "Enabling i2c-1"
grep -qxF 'dtparam=i2c1=on' /boot/config.txt || echo 'dtparam=i2c1=on' >> /boot/config.txt


if [ ! -d "build" ]; then
	mkdir build
fi
cd build
cmake ..
make install
cd ..


cp x725shutdown.sh /usr/local/sbin/x725shutdown
cp scriptToRunBeforeShutdown.sh /usr/local/sbin/scriptToRunBeforeShutdown

cp x725power.service.txt  /etc/systemd/system/x725power.service
cp x725button.service.txt  /etc/systemd/system/x725button.service
chmod 655 /etc/systemd/system/x725power.service
chmod 655 /etc/systemd/system/x725button.service

systemctl daemon-reload

systemctl enable x725power.service
systemctl enable x725button.service

systemctl restart x725button.service
systemctl restart x725power.service

# Setup status files:
if [ ! -d /etc/libpanda.d ]; then
	mkdir /etc/libpanda.d
fi

if [ ! -f /etc/libpanda.d/x725hasexternalpower ]; then
	touch /etc/libpanda.d/x725hasexternalpower
	echo "1" > /etc/libpanda.d/x725hasexternalpower
fi

if [ ! -f /etc/libpanda.d/x725batteryvoltage ]; then
	touch /etc/libpanda.d/x725batteryvoltage
	echo "1" > /etc/libpanda.d/x725batteryvoltage
fi

if [ ! -f /etc/libpanda.d/x725capacity ]; then
	touch /etc/libpanda.d/x725capacity
	echo "1" > /etc/libpanda.d/x725capacity
fi


if [ ! -f /etc/libpanda.d/x725batterycurrent ]; then
	touch /etc/libpanda.d/x725batterycurrent
	echo "1" > /etc/libpanda.d/x725batterycurrent
fi

echo "Done."
echo "========================="
