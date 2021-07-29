#!/bin/bash

if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

declare -a depencencies=( bmon )
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

if [ ! -d /etc/libpanda.d ]; then
	sudo mkdir /etc/libpanda.d
fi

sudo cp scripts/vinToHostname.sh /usr/sbin/vinToHostname
sudo cp scripts/addWifiApSimple.sh /usr/sbin/addWifiApSimple
sudo cp scripts/installVandertestRosPackages.sh /usr/sbin/updateVandertest

if [[ ! -f "/etc/libpanda.d/vin" ]]; then
	sudo -s eval 'echo "circles" > /etc/libpanda.d/vin'
fi

./build.sh
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

sudo systemctl enable ssh
sudo systemctl start ssh


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



if [ ! -d ~/irods-icommands-debs ]; then
	tar xzf ~/libpanda/scripts/irods-icommands-debs.tgz -C ~/

	cd ~/irods-icommands-debs
	./install.sh
	# the following needs to run AFTER installing irods commands
	sudo sed -i 's/libssl1.0.0/libssl1.1/g' /var/lib/dpkg/status
else
	echo "irods icommands already isntalled."
fi

# enable persisten journalctl logging:
sudo sed -i 's/#Storage.*/Storage=persistent/' /etc/systemd/journald.conf





# Install ROS:
#cd ~/libpanda/scripts
#./installROS.sh

# Install can_to_ros:
#./installCanToRos.sh
#./installExperiments7-22.sh
