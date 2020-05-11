#/bin/bash
LOCAL=/var/panda/CyverseData/JmscslgroupData/PandaData
REMOTE=/iplant/home/rahulbhadani/JmscslgroupData/PandaData

read -r -p "Would you like to push data to $REMOTE? [y/N] " response
case "$response" in
      [yY][eE][sS]|[yY])
	  irsync -r -v ${LOCAL} i:${REMOTE}
          ;;
      *)
          echo "Exiting" 
          ;;
  esac

