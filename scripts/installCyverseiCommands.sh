#!/bin/bash
# Author: Matt Bunting
#  This is the set of irods comamnds


if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)

if [ ! -d ~/irods-icommands-debs ]; then
#	tar xzf ~/libpanda/scripts/irods-icommands-debs.tgz -C ~/
    tar xzf ${LIBPANDA_SRC}/scripts/irods-icommands-debs.tgz -C ~/
fi

if ! command -v ils &> /dev/null;
then
	cd ~/irods-icommands-debs
	./install.sh
	# the following needs to run AFTER installing irods commands
	sudo sed -i 's/libssl1.0.0/libssl1.1/g' /var/lib/dpkg/status
    echo "irods icommands have been installed."
else
	echo "irods icommands already installed."
fi
