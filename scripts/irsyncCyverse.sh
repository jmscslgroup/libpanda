#/bin/bash
VINFILE=/etc/libpanda.d/vin
VIN=$(cat ${VINFILE})
LOCAL=/var/panda/CyverseData/JmscslgroupData/PandaData
REMOTE=/iplant/home/sprinkjm/private-circles/${VIN}/libpanda
response=

while getopts “hf” opt; do
  case $opt in
    h) echo "Uploads data to CyVerse."
	echo " -f : force upload without prompt" 
	;;
    f) response=y;;
    y) response=y;;
  esac
done

if [ -s ${VINFILE} ]
then

if [ -z "$response" ]; then
	read -r -p "Would you like to push data to $REMOTE? [y/N] " response
fi

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
