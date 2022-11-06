#!/bin/bash
#Author: Alex Richardson

i=0
max_tries=12

while :
do
  SSID=$(iwconfig wlan0 | grep -Eo "ESSID:\".*\"" | grep -Eo "\".*\"" | tail -c +2 | head -c -2)
  echo $SSID
  if [ "$SSID" = "vuDevices" ]
  then
    echo "Connected to vuDevices!"
    wget -q --spider http://google.com
    if [ $? -eq 0 ]
    then
      echo "Connected to internet! Breaking!"
      break
    fi
  fi
  i = $((i + 1))
  if [ $i -ge $max_tries ]
  then
    echo "1 minute timeout reached for connectivity."
    exit 1
  fi
  echo "Not on vuDevices or internet. Waiting 5 seconds...."
  sleep 5
done

echo "Updating..."
systemctl stop can
su -c "cd /home/circles/libpanda && git pull && ./install.sh" circles
systemctl start can
echo "Updating done!"


