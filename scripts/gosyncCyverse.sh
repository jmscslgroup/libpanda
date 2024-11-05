#/bin/bash

#/usr/local/sbin/check_VIN_before_upload

VINFILE=/etc/libpanda.d/vin
VIN=$(cat ${VINFILE})

CYVERSE_DEST_DIR_FILE=/etc/libpanda.d/cyverse_dest_dir
CYVERSE_DEST_DIR=$(cat ${CYVERSE_DEST_DIR_FILE})

LIBPANDA_USR=$(cat /etc/libpanda.d/libpanda_usr)
GO_CFG_DIR=/home/${LIBPANDA_USR}/.irods

# csv files:
LOCAL=/var/panda/CyverseData/JmscslgroupData/PandaData/
REMOTE=${CYVERSE_DEST_DIR}/${VIN}/libpanda

# For rosbagfiles:
DIR_PATH_CYVERSE="${CYVERSE_DEST_DIR}/${VIN}/"
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

if ! gocmd ls -c ${GO_CFG_DIR} > /dev/null 2>&1; then
	echo "Unable to conenct to CyVerse!  Invalid credentials?  try: gocmd init"
	exit 1
fi

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
#	  gocmd -c ~/go-cmd-binary/config.yaml mkdir -p ${DIR_PATH_CYVERSE}
#	  gocmd mkdir -p ${DIR_PATH_CYVERSE}
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
/usr/local/sbin/local_bagfile_maintenance
