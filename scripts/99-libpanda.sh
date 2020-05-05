#/bin/bash
echo ""
echo "This machine is running a version of libpanda last pulled on" 
echo $(cd /home/eternity/libpanda && git show | head | grep Date: && cd )
echo ""

if [ -s /etc/libpanda.d/vin ]; then
	echo "WARNING:"
	echo "The VIN entry in /etc/libpanda/vin is " $(cat /etc/libpanda/vin)
else
	echo "The VIN file is empty, run 'sudo ./setVin.sh' to set your VIN"
fi
echo ""
if [ ! -s /home/eternity/.irods/irods_environment.json ]; then
	echo "WARNING:"
	echo "Your irods is not set up, run 'iinit' and enter your credentials, along with "
	echo "  hostname: data.cyverse.org"
	echo "  port: 1247"
	echo "  zone: iplant"
	echo ""
	echo "If you get an error, still try 'ils' to see if anything comes up"
fi
echo ""
echo "Run ./irsyncCyverse.sh to sync data to Cyverse; edit it to change destination folder."
echo ""


