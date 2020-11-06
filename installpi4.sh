#!/bin/bash
# Author: Matt Bunting

echo "--------------------------------------------------------"
echo "Installing libpandac:"

#build dependencies:
declare -a depencencies=(build-essential libncurses5-dev libusb-1.0-0-dev cmake)
toInstall=()
echo "Dependencies:" ${depencencies[@]}
for dependency in "${depencencies[@]}"
do
	echo "Checking for" $dependency
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

if [ ! -d "build" ];
then
	mkdir build
fi
cd build
cmake ..
make -j4
make install
cd ..

# Install panda services:
cd scripts/panda
./install.sh
cd ../..

# Install configuration files and welcome msg
cd scripts/
cp irsyncCyverse.sh ../irsyncCyverse.sh
cp setVin.sh ../setVin.sh
cp 99-libpanda.sh /etc/profile.d/99-libpanda.sh
cd ..

echo "Done."
echo "--------------------------------------------------------"
