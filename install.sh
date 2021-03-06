#!/bin/bash

./build.sh
cd scripts/blinkt
./install.sh
cd ../circlesmanager
./install.sh
cd ../crazypifi
./install.sh
cd ../xUps
./install.sh
cd ../..

systemctl enable ssh
systemctl start ssh

cp scripts/addWifiAp.sh ../
../setVin.sh
../addWifiAp.sh
