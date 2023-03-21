#/bin/bash
# Author: Jonathan Sprinkle

#cd ~/VersionControl/strym/examples/ && source ~/VirtualEnv/stream/bin/activate && python strym_impl.py
FOLDER_PRE=/var/panda/CyverseData/JmscslgroupData/PandaData
VIN=""
# Note: does not remove spaces etc. should harden later TODO
if [ -f /etc/libpanda.d/vin ]; then
	VIN=_$(cat /etc/libpanda.d/vin)
fi
FOLDER_DATE=$(date +%Y_%m_%d)
FOLDER=${FOLDER_PRE}/${FOLDER_DATE}
FILENAME_PRE=$(date +%Y-%m-%d-%H-%M-%S)${VIN}
if [ ! -d ${FOLDER_PRE}/${FOLDER_DATE} ]; then
	echo "Creating ${FOLDER_PRE}/${FOLDER_DATE}..."
	mkdir -p ${FOLDER_PRE}/${FOLDER_DATE}
fi
#pandacord -g ${FOLDER}/${FILENAME_PRE}_GPS_Messages.csv -c ${FOLDER}/${FILENAME_PRE}_CAN_Messages.csv
pandazone -g ${FOLDER}/gps -c ${FOLDER}/can
#echo FOLDER_DATE=${FOLDER_DATE}
#echo FILENAME_PRE=${FILENAME_PRE}
