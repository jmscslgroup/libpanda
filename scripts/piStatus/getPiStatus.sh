#!/bin/sh
#Author: Matt Nice

while :
do
  SSID=$(iwconfig wlan0 | grep -Eo "ESSID:\".*\"" | grep -Eo "\".*\"" | tail -c +2 | head -c -2)
  #SSID=$(echo ${SSID:1:-1})
  UPLOAD_DOWNLOAD_STR=$(ifstat -i wlan0 1 1 | tail -n 1)
  DOWNLOAD_RATE_KB=$(echo $UPLOAD_DOWNLOAD_STR | awk -F'[^0-9\.]+' '{ print $2}')
  UPLOAD_RATE_KB=$(echo $UPLOAD_DOWNLOAD_STR | awk -F'[^0-9\.]+' '{ print $3}')
  WLAN0_IP=$(ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' )
  ETH0_IP=$(ifconfig eth0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' )
  WLAN0_MAC=$(ifconfig wlan0 | grep -Eo 'ether (addr:)?([0-9a-f]*\:){5}[0-9a-f]*' | grep -Eo '([0-9a-f]*\:){5}[0-9a-f]*')
  VIN=$(cat /etc/libpanda.d/vin)
  BATTERY_VOLTAGE=$(cat /etc/libpanda.d/x725batteryvoltage | tr -d '\r')
  if [ -z "$BATTERY_VOLTAGE" ]
  then
     BATTERY_VOLTAGE="0.0"
  fi
  EXTERNAL_POWER=$(echo $(cat /etc/libpanda.d/x725hasexternalpower)|tr -d '\r')
  UPDATE_TIME=$(date +%s)

  FREE_RAM=$(awk '/MemFree/ { printf "%.3fG", $2/1024/1024 }' /proc/meminfo)
  AVAILABLE_RAM=$(awk '/MemAvailable/ { printf "%.3fG", $2/1024/1024 }' /proc/meminfo)
  TOTAL_RAM=$(awk '/MemTotal/ { printf "%.3fG", $2/1024/1024 }' /proc/meminfo)

  TOTAL_MEMORY=$(lsblk -fmo NAME,FSUSE%,FSAVAIL,FSSIZE | grep 'mmcblk0p2' | awk '{printf "%.2fG", $4 }')
  TOTAL_MEMORY_AVAILABLE=$(lsblk -fmo NAME,FSUSE%,FSAVAIL,FSSIZE | grep 'mmcblk0p2' | awk '{printf "%.2fG", $3 }')
  TOTAL_MEMORY_USED_PERCENT=$(lsblk -fmo NAME,FSUSE%,FSAVAIL,FSSIZE | grep 'mmcblk0p2' | awk '{printf "%.1f", $2 }')

  CPU_USAGE_1MIN=$(top -bn1 | grep load | awk '{printf "%.2f", $(NF-2)}')
  CPU_USAGE_5MIN=$(top -bn1 | grep load | awk '{printf "%.2f", $(NF-1)}')
  CPU_USAGE_15MIN=$(top -bn1 | grep load | awk '{printf "%.2f", $(NF)}')

  JSON='{"wlan0_mac":"%s","update_time":"%d","ssid":"%s","wlan0_upload_rate_kb":"%.2f","wlan0_download_rate_kb":"%.2f","wlan0_ip":"%s","eth0_ip":"%s","vin":"%s","battery_voltage":"%.2f","external_power":"%d","free_ram":"%s","available_ram":"%s","total_ram":"%s","total_memory_available":"%s","total_memory":"%s","total_memory_used_percent":"%s","cpu_load_1min":"%.2f","cpu_load_5min":"%.2f","cpu_load_15min":"%.2f"}'
  JSON=$(printf "$JSON" "$WLAN0_MAC" "$UPDATE_TIME" "$SSID" "$UPLOAD_RATE_KB" "$DOWNLOAD_RATE_KB" "$WLAN0_IP" "$ETH0_IP" "$VIN" "$BATTERY_VOLTAGE" "$EXTERNAL_POWER" "$FREE_RAM" "$AVAILABLE_RAM" "$TOTAL_RAM" "$TOTAL_MEMORY_AVAILABLE" "$TOTAL_MEMORY" "$TOTAL_MEMORY_USED_PERCENT" "$CPU_USAGE_1MIN" "$CPU_USAGE_5MIN" "$CPU_USAGE_15MIN")
  ENDPOINT="http://ransom.isis.vanderbilt.edu/LIVE_VIEWER_SITE/piStatusRest.php?PASS=circles&DATA=%s"
  ENDPOINT=$(printf "$ENDPOINT" "$JSON")
  echo $JSON > /etc/libpanda.d/piStatus.json
  echo $ENDPOINT
  wget -O - $ENDPOINT

  sleep 10
done
