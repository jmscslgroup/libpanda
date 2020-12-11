#/bin/bash
VINFILE=/etc/libpanda.d/vin
VIN=$(cat ${VINFILE})
LOCAL=/var/panda/CyverseData/JmscslgroupData/PandaData
REMOTE=/iplant/home/sprinkjm/private-circles/${VIN}/libpanda

if [ -s ${VINFILE} ]
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
