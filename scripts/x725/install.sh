#!/bin/bash
# Author: Matt Bunting

echo "========================="
echo "Installing x725monitor   "

declare -a depencencies=(python-smbus)
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

if [ ! -d "build" ]; then
	mkdir build
fi
cd build
cmake ..
make install
cd ..


#cp x725button.sh /usr/local/sbin/x725button
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

echo "Done."
echo "========================="
