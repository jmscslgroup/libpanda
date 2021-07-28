#!/bin/bash

if [ ! -d /etc/libpanda.d ]; then
	sudo mkdir /etc/libpanda.d
fi

tar xzf scripts/irods-icommands-debs.tgz -C ~/

sudo cp scripts/vinToHostname.sh /usr/sbin/vinToHostname
sudo cp scripts/addWifiApSimple.sh /usr/sbin/addWifiApSimple

if [[ ! -f "/etc/libpanda.d/vin" ]]; then
	sudo eval 'echo "circles" > /etc/libpanda.d/vin'
fi

sudo ./build.sh
cd scripts/blinkt
sudo ./install.sh
cd ../crazypifi
sudo ./install.sh
cd ../xUps
sudo ./install.sh
cd ../circles-ui
sudo ./install.sh
cd ../circlesmanager
sudo ./install.sh
cd ../..

sudo apt install bmon

sudo systemctl enable ssh
sudo systemctl start ssh


#cp scripts/addWifiAp.sh ../
#../setVin.sh
#../addWifiAp.sh

# Install ROS:
#cd scripts
#./installROS.sh

# Install can_to_ros:
#./installCanToRos.sh




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




cd ~/irods-icommands-debs
./install.sh
# the following needs to run AFTER installing irods commands
sudo sed -i 's/libssl1.0.0/libssl1.1/g' /var/lib/dpkg/status


# enable persisten journalctl logging:
sudo sed -i 's/#Storage=auto/Storage=Persistent/g' /etc/systemd/journald.conf 

cd ~

