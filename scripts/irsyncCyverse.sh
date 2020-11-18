#/bin/bash
VIN=$(cat /etc/libpanda.d/vin)
LOCAL=/var/panda/CyverseData/JmscslgroupData/PandaData
REMOTE=/iplant/home/sprinkjm/${VIN}/libpanda

if [ -s ${VIN} ]
then

read -r -p "Would you like to push data to $REMOTE? [y/N] " response
case "$response" in
      [yY][eE][sS]|[yY])
	  irsync -r -v ${LOCAL} i:${REMOTE}
          ;;
      *)
          echo "Exiting" 
          ;;
  esac
else
	echo "You have not set your VIN...exiting."
fi
