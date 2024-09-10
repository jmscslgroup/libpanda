#!/bin/bash

if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

sudo ln -sf /usr/share/zoneinfo/UTC /etc/localtime

git submodule init
git submodule update

declare -a depencencies=( bmon scons usbmuxd pip3 )
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
	sudo apt-get update
	sudo apt-get install -y ${toInstall[@]}
fi

pip install cantools    # for can_to_ros codegen

echo "=================================="
echo "Installing libWiringPi for Digital Potentiometer"
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
git pull origin
./build
cd ..

echo "Done installing libWiringPi, result of \$ gpio readall
:"
gpio readall
echo "=================================="

echo "=================================="
echo "Installing Panda Firmware SDK"
cd panda/board
./get_sdk.sh
cd ..
scons -j4
cd ..
echo "Panda firmware version:"
cat panda/board/obj/version
echo ""
echo "Done installing Panda Firmware SDK"
echo "=================================="



if [ ! -d /etc/libpanda.d ]; then
	sudo mkdir /etc/libpanda.d
fi

SRC_DIR=`pwd`   # this install is run from this directory
sudo sh -c "echo $SRC_DIR > /etc/libpanda.d/libpanda_src_dir"
sudo sh -c "echo $USER > /etc/libpanda.d/libpanda_usr"

sudo cp resources/zone-testbed.json /etc/libpanda.d/zone-default.json
sudo cp resources/zone-test.json /etc/libpanda.d/

sudo cp scripts/vinToHostname.sh /usr/sbin/vinToHostname
sudo cp scripts/vinParser.py /usr/sbin/vinParser.py
sudo cp scripts/addWifiApSimple.sh /usr/sbin/addWifiApSimple
sudo cp scripts/installVandertestRosPackages.sh /usr/sbin/updateVandertest

if [[ ! -f "/etc/libpanda.d/vin" ]]; then
	sudo -s eval 'echo "circles" > /etc/libpanda.d/vin'
fi

./build.sh
cd $SRC_DIR/scripts/xUps && sudo ./install.sh
cd $SRC_DIR/scripts/circles-ui && sudo ./install.sh
cd $SRC_DIR/scripts/gps-tracker && sudo ./install.sh
#cd $SRC_DIR/scripts/experiments && ./install.sh
cd $SRC_DIR/scripts/circlesplanner && sudo ./install.sh
cd $SRC_DIR/scripts/piStatus && sudo ./install.sh
cd $SRC_DIR/scripts/autoUpdate && sudo ./install.sh
cd $SRC_DIR/scripts/rosnodeChecker && sudo ./install.sh
cd $SRC_DIR/scripts/bluezone && sudo ./install.sh
# not installing live tracker python version
# cd ../live-tracker-raspberry-pi
# sudo ./install.sh
# cd ../..
cd $SRC_DIR/apps && sudo ./install.sh

sudo systemctl enable ssh
sudo systemctl start ssh

# just ensure all the above installs before this due to its reboot functionality:
cd $SRC_DIR/scripts/circlesmanager && sudo ./install.sh


#cp scripts/addWifiAp.sh ../
#../setVin.sh
#../addWifiAp.sh






# Build the bagfile directory:
BAGFILE_DIR=/var/panda/CyverseData/JmscslgroupData/bagfiles

if [ ! -d ${BAGFILE_DIR} ]; then
	echo "Creating ${BAGFILE_DIR}..."
	sudo mkdir -p ${BAGFILE_DIR}
fi

# The following is authored by Jonathan Sprinkle:
# 1. setting the permissions of bagfiles folder to be in the dialout group (circles is a member of dialout):
cd /var/panda/CyverseData/JmscslgroupData/bagfiles
sudo chown -R :dialout .

# 2. Make the bagfiles folder and all contained folders writeable by group membership:
sudo chmod -R g+w .

# 3. Set the group id permissions to be carried forward for any folders created in bagfiles:

sudo chmod g+s .
# With these changes in place (which can be done during install for the libpanda stuff) we can have everything such that you do not need to be sudo su to write to the bagfiles.


MISMATCHFILES=libpanda-check-mismatched-files
cd ~
if [ -d ${MISMATCHFILES} ]; then
	echo "${MISMATCHFILES} already exists...executing git pull"
	cd ${MISMATCHFILES}
	git pull
	cd ~
else
	git clone https://github.com/graciegumm/libpanda-check-mismatched-files.git
fi
cd ~/${MISMATCHFILES}
sudo cp check_VIN_before_upload.sh /usr/local/sbin/check_VIN_before_upload
sudo chmod +x /usr/local/sbin/check_VIN_before_upload

sudo cp /home/circles/libpanda/scripts/local_data_size_maintenance.sh /usr/local/sbin/local_data_size_maintenance
sudo chmod +x /usr/local/sbin/local_data_size_maintenance

sudo cp /home/circles/libpanda/scripts/local_bagfile_maintenance.sh /usr/local/sbin/local_bagfile_maintenance
sudo chmod +x /usr/local/sbin/local_bagfile_maintenance


#if [ ! -d ~/irods-icommands-debs ]; then
#	tar xzf ~/libpanda/scripts/irods-icommands-debs.tgz -C ~/
#
#	cd ~/irods-icommands-debs
#	./install.sh
#	# the following needs to run AFTER installing irods commands
#	sudo sed -i 's/libssl1.0.0/libssl1.1/g' /var/lib/dpkg/status
#else
#	echo "irods icommands already installed."
#fi

# enable persisten journalctl logging:
sudo sed -i 's/#Storage.*/Storage=persistent/' /etc/systemd/journald.conf

# install GoCMD
if [ ! -d ~/go-cmd-binary ]; then
	mkdir -p ~/go-cmd-binary

	GOCMD_VER=$(curl -L -s https://raw.githubusercontent.com/cyverse/gocommands/main/VERSION.txt)
	curl -L -s https://github.com/cyverse/gocommands/releases/download/${GOCMD_VER}/gocmd-${GOCMD_VER}-linux-arm64.tar.gz | tar zxvf - -C ~/go-cmd-binary
	cd ~/go-cmd-binary
	sudo cp gocmd /usr/local/bin/

else
	echo "gocommands already installed."
fi


# Install ROS:
#cd ~/libpanda/scripts
#./installROS.sh

# Install can_to_ros:
#./installCanToRos.sh
#./installExperiments7-22.sh


# Add the vuDevices to the network:
if sudo grep -Fq "vuDevices" /etc/wpa_supplicant/wpa_supplicant.conf
then
	echo "WiFi AP SSID vuDevices network already configured"
else
	sudo addWifiApSimple -s vuDevices -p Acorn1873
fi

# This will invoke a reboot on first installation if hostname is not "circles"
systemctl is-active circlesmanager || sudo systemctl start circlesmanager
