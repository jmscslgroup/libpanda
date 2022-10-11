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

  JSON='{"wlan0_mac":"%s","wlan0_ip":"%s","eth0_ip":"%s","vin":"%s","external_power":%d,"update_time":"%s"}'

  printf "$JSON" "$WLAN0_MAC" "$WLAN0_IP" "$ETH0_IP" "$VIN" "$EXTERNAL_POWER" "$UPDATE_TIME" > /etc/libpanda.d/piStatus.json

  sleep 10
done
