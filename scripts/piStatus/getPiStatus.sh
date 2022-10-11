#!/bin/sh
#Author: Matt Nice

while :
do
  WLAN0_IP=$(ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' )
  ETH0_IP=$(ifconfig eth0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' )
  WLAN0_MAC=$(ifconfig wlan0 | grep -Eo 'ether (addr:)?([0-9a-f]*\:){5}[0-9a-f]*' | grep -Eo '([0-9a-f]*\:){5}[0-9a-f]*')
  VIN=$(cat /etc/libpanda.d/vin)
  EXTERNAL_POWER=$(echo $(cat /etc/libpanda.d/x725hasexternalpower)|tr -d '\r')
  UPDATE_TIME=$(date +%s)
#  FREE_RAM=$(free -h | grep 'Mem' | awk '{print $7}')
#  TOTAL_MEMORY=$(awk '/VmallocTotal/ { printf "%.3f \n", $2/1024/1024 }' /proc/meminfo)
#  USED_MEMORY=$(awk '/VmallocUsed/ { printf "%.3f \n", $2/1024/1024 }' /proc/meminfo)
  FREE_RAM=$(awk '/MemFree/ { printf "%.3fG", $2/1024/1024 }' /proc/meminfo)
  AVAILABLE_RAM=$(awk '/MemAvailable/ { printf "%.3fG", $2/1024/1024 }' /proc/meminfo)
  TOTAL_RAM=$(awk '/MemTotal/ { printf "%.3fG", $2/1024/1024 }' /proc/meminfo)
  TOTAL_MEMORY=$(lsblk -fmo NAME,FSUSE%,FSAVAIL,FSSIZE | grep 'mmcblk0p2' | awk '{printf "%.2fG", $4 }')
  TOTAL_MEMORY_USED=$(lsblk -fmo NAME,FSUSE%,FSAVAIL,FSSIZE | grep 'mmcblk0p2' | awk '{printf "%.2fG", $3 }')
  TOTAL_MEMORY_USED_PERCENT=$(lsblk -fmo NAME,FSUSE%,FSAVAIL,FSSIZE | grep 'mmcblk0p2' | awk '{printf "%.1f", $2 }')


  JSON='{"wlan0_mac":"%s","update_time":"%d","wlan0_ip":"%s","eth0_ip":"%s","vin":"%s","external_power":"%d","free_ram":"%s","available_ram":"%s","total_ram":"%s","total_memory_used":"%s","total_memory":"%s","total_memory_used_percent":"%s"}'


  printf "$JSON" "$WLAN0_MAC" "$UPDATE_TIME" "$WLAN0_IP" "$ETH0_IP" "$VIN" "$EXTERNAL_POWER" "$FREE_RAM" "$AVAILABLE_RAM" "$TOTAL_RAM" "$TOTAL_MEMORY_USED" "$TOTAL_MEMORY" "$TOTAL_MEMORY_USED_PERCENT" > /etc/libpanda.d/piStatus.json

  sleep 10
done
