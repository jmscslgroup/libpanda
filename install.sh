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


# Build the bagfile directory:
BAGFILE_DIR=/var/panda/CyverseData/JmscslgroupData/bagfiles

if [ ! -d ${BAGFILE_DIR} ]; then
	echo "Creating ${BAGFILE_DIR}..."
	mkdir -p ${BAGFILE_DIR}
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
