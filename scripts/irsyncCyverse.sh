#/bin/bash

/usr/local/sbin/check_VIN_before_upload

VINFILE=/etc/libpanda.d/vin
VIN=$(cat ${VINFILE})

# csv files:
LOCAL=/var/panda/CyverseData/JmscslgroupData/PandaData
REMOTE=/iplant/home/sprinkjm/private-circles/${VIN}/libpanda

# For rosbagfiles:
DIR_PATH_CYVERSE="/iplant/home/sprinkjm/private-circles/${VIN}/bagfiles/"
DIR_PATH_LOCAL="/var/panda/CyverseData/JmscslgroupData/bagfiles/"

response=

while getopts “hf” opt; do
  case $opt in
    h) echo "Uploads data to CyVerse."
	echo " -f : force upload without prompt"
    echo ""
    exit
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
      #csv files:
	  irsync -r -v ${LOCAL} i:${REMOTE}
	  
	  # rosbag files:
	  imkdir -pv ${DIR_PATH_CYVERSE}
	  irsync -r ${DIR_PATH_LOCAL} i:${DIR_PATH_CYVERSE}
          ;;
      *)
          echo "Exiting" 
          ;;
  esac
else
	echo "You have not set your VIN...exiting."
fi
