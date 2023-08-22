#!/bin/bash
# Author: Matt Bunting

# installs iCommands and does some nasty stuff with dpkg and ssl

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

SRC_DIR=$(cat /etc/libpanda.d/libpanda_src_dir)
USER=$(cat /etc/libpanda.d/libpanda_usr)

#if [ ! -d ~/irods-icommands-debs ]; then
if [ ! -d ~/irods-icommands-debs ]; then
    echo "Installing irods icommands..."
    tar xzf $SRC_DIR/scripts/irods-icommands-debs.tgz -C /home/$USER/

    cd /home/$USER/irods-icommands-debs
    ./install.sh
    # the following needs to run AFTER installing irods commands
    sudo sed -i 's/libssl1.0.0/libssl1.1/g' /var/lib/dpkg/status
else
    echo "irods icommands already installed."
fi

echo "----------------------------"
