#/bin/bash

#/usr/local/sbin/check_VIN_before_upload

VINFILE=/etc/libpanda.d/vin
VIN=$(cat ${VINFILE})

# csv files:
LOCAL=/var/panda/CyverseData/JmscslgroupData/PandaData/
REMOTE=/iplant/home/sprinkjm/private-ndd/${VIN}/libpanda

# For rosbagfiles:
DIR_PATH_CYVERSE="/iplant/home/sprinkjm/private-ndd/${VIN}/bagfiles/"
DIR_PATH_LOCAL="/var/panda/CyverseData/JmscslgroupData/bagfiles/"

response=

while getopts “hfy” opt; do
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
	  gocmd sync --progress --no_hash --show_path ${LOCAL} i:${REMOTE}
	  
	  # rosbag files:
	  gocmd mkdir -p ${DIR_PATH_CYVERSE}
	  gocmd sync --progress --no_hash ${DIR_PATH_LOCAL} i:${DIR_PATH_CYVERSE}
          
#	  /usr/local/sbin/local_data_size_maintenance ##keeping data size in check
	;;
      *)
          echo "Exiting" 
          ;;
  esac
else
	echo "You have not set your VIN...exiting."
fi

/usr/local/sbin/local_data_size_maintenance
