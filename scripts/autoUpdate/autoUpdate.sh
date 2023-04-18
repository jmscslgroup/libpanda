#!/bin/bash
#Author: Alex Richardson

i=0
max_tries=12

while :
do
  #SSID=$(iwconfig wlan0 | grep -Eo "ESSID:\".*\"" | grep -Eo "\".*\"" | tail -c +2 | head -c -2)
  #echo $SSID
  #if [ "$SSID" = "vuDevices" ]
  #then
  #  echo "Connected to vuDevices!"
    wget -q --spider http://google.com
    if [ $? -eq 0 ]
    then
      echo "Connected to internet! Breaking!"
      break
    fi
  #fi
  i=$((i + 1))
  if [ $i -ge $max_tries ]
  then
    echo "1 minute timeout reached for connectivity."
    exit 1
  fi
  #echo "Not on vuDevices or internet. Waiting 5 seconds...."
  echo "Not connected to internet. Waiting 5 seconds...."
  sleep 5
done

echo "Updating..."
echo "Updating..." > /etc/libpanda.d/logMessage
#systemctl stop can
#su -c "cd /home/circles/libpanda && git pull && ./install.sh" circles
#systemctl start can
LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
PI_USER=$(cat /etc/libpanda.d/libpanda_usr)
#su -c "cd /home/circles/libpanda && ./update.sh" circles
su -c "cd $LIBPANDA_SRC && ./update.sh" $PI_USER
echo "Updating done!"
echo "Updating done!" > /etc/libpanda.d/logMessage

