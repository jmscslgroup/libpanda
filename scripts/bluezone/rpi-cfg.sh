#!/bin/sh
INTERACTIVE=True

list_wlan_interfaces() {
  for dir in /sys/class/net/*/wireless; do
    if [ -d "$dir" ]; then
      IFACE="$(basename "$(dirname "$dir")")"
      if wpa_cli -i "$IFACE" status > /dev/null 2>&1; then
        echo "$IFACE"
      fi
    fi
  done
}

do_wifi_ssid_passphrase() {
  RET=0
  if [ "$INTERACTIVE" = True ] && [ -z "$(get_wifi_country)" ]; then
    do_wifi_country
  fi

  if systemctl -q is-active dhcpcd; then
    IFACE="$(list_wlan_interfaces | head -n 1)"

    if [ -z "$IFACE" ]; then
      if [ "$INTERACTIVE" = True ]; then
#        whiptail --msgbox "No wireless interface found" 20 60
        echo "No wireless interface found"
      fi
      return 1
    fi

    if ! wpa_cli -i "$IFACE" status > /dev/null 2>&1; then
      if [ "$INTERACTIVE" = True ]; then
#        whiptail --msgbox "Could not communicate with wpa_supplicant" 20 60
        echo "Could not communicate with wpa_supplicant"
      fi
      return 1
    fi
  elif ! systemctl -q is-active NetworkManager; then
    if [ "$INTERACTIVE" = True ]; then
#        whiptail --msgbox "No supported network connection manager found" 20 60
        echo "No supported network connection manager found"
      fi
      return 1
  fi

  SSID="$1"
  echo "SSID=$SSID"
  while [ -z "$SSID" ] && [ "$INTERACTIVE" = True ]; do
    
    if ! SSID=$(whiptail --inputbox "Please enter SSID" 20 60 3>&1 1>&2 2>&3); then
      return 0
    elif [ -z "$SSID" ]; then
#      whiptail --msgbox "SSID cannot be empty. Please try again." 20 60
      echo "SSID cannot be empty. Please try again."
    fi
  done

  PASSPHRASE="$2"
  while  [ -z "$PASSPHRASE" ] && [ "$INTERACTIVE" = True ]; do
    if ! PASSPHRASE=$(whiptail --passwordbox "Please enter passphrase. Leave it empty if none." 20 60 3>&1 1>&2 2>&3); then
      return 0
    else
      break
    fi
  done

  # Escape special characters for embedding in regex below
  ssid="$(echo "$SSID" \
   | sed 's;\\;\\\\;g' \
   | sed -e 's;\.;\\\.;g' \
         -e 's;\*;\\\*;g' \
         -e 's;\+;\\\+;g' \
         -e 's;\?;\\\?;g' \
         -e 's;\^;\\\^;g' \
         -e 's;\$;\\\$;g' \
         -e 's;\/;\\\/;g' \
         -e 's;\[;\\\[;g' \
         -e 's;\];\\\];g' \
         -e 's;{;\\{;g'   \
         -e 's;};\\};g'   \
         -e 's;(;\\(;g'   \
         -e 's;);\\);g'   \
         -e 's;";\\\\\";g')"

  HIDDEN=${3:-0}
  PLAIN=${4:-1}

  if systemctl -q is-active dhcpcd; then
    echo "systemctl -q is-active dhcpcd -> true"
    wpa_cli -i "$IFACE" list_networks \
     | tail -n +2 | cut -f -2 | grep -P "\t$ssid$" | cut -f1 \
     | while read -r ID; do
      wpa_cli -i "$IFACE" remove_network "$ID" > /dev/null 2>&1
    done

    ID="$(wpa_cli -i "$IFACE" add_network)"
    echo "ID=$ID"
    wpa_cli -i "$IFACE" set_network "$ID" ssid "\"$SSID\"" 2>&1 | grep -q "OK"
    RET=$((RET + $?))
    echo "RET=$RET"

    if [ -z "$PASSPHRASE" ]; then
      wpa_cli -i "$IFACE" set_network "$ID" key_mgmt NONE 2>&1 | grep -q "OK"
      RET=$((RET + $?))
      echo "RET=$RET"
    else
      if [ "$PLAIN" = 1 ]; then
        PASSPHRASE="\"$PASSPHRASE\""
      fi
      wpa_cli -i "$IFACE" set_network "$ID" psk "$PASSPHRASE" 2>&1 | grep -q "OK"
      RET=$((RET + $?))
    fi
    if [ "$HIDDEN" -ne 0 ]; then
      wpa_cli -i "$IFACE" set_network "$ID" scan_ssid 1 2>&1 | grep -q "OK"
      RET=$((RET + $?))
    fi
    if [ $RET -eq 0 ]; then
      echo "All good so far!  setting network with: wpa_cli -i $IFACE enable_network $ID"
      wpa_cli -i "$IFACE" enable_network "$ID" > /dev/null 2>&1
    else
      echo "Failed! removing network with: wpa_cli -i $IFACE remove_network $ID"
      wpa_cli -i "$IFACE" remove_network "$ID" > /dev/null 2>&1
      if [ "$INTERACTIVE" = True ]; then
#        whiptail --msgbox "Failed to set SSID or passphrase" 20 60
        echo "Failed to set SSID or passphrase"
      fi
    fi
    wpa_cli -i "$IFACE" save_config > /dev/null 2>&1
    echo "$IFACE_LIST" | while read -r IFACE; do
      wpa_cli -i "$IFACE" reconfigure > /dev/null 2>&1
    done
    
      
    if [ $RET -eq 0 ]; then
        echo "wpa_cli -i $IFACE select_network $ID"
        wpa_cli -i "$IFACE" select_network "$ID" 2>&1 | grep -q "OK"
        RET=$((RET + $?))
        echo "select_network RET=$RET"
    fi
  else
    echo "systemctl -q is-active dhcpcd -> false"
    if [ "$HIDDEN" -ne 0 ]; then
      nmcli device wifi connect "$SSID"  password "$PASSPHRASE" hidden true | grep -q "activated"
    else
      nmcli device wifi connect "$SSID"  password "$PASSPHRASE" | grep -q "activated"
    fi
    RET=$((RET + $?))
  fi
  

    
  echo "Final RET=$RET"
  return "$RET"
}

delete_network() {
    SSID="$1"
  if systemctl -q is-active dhcpcd; then
    IFACE="$(list_wlan_interfaces | head -n 1)"
    echo "Found interface $IFACE"
    
      # Escape special characters for embedding in regex below
  ssid="$(echo "$SSID" \
   | sed 's;\\;\\\\;g' \
   | sed -e 's;\.;\\\.;g' \
         -e 's;\*;\\\*;g' \
         -e 's;\+;\\\+;g' \
         -e 's;\?;\\\?;g' \
         -e 's;\^;\\\^;g' \
         -e 's;\$;\\\$;g' \
         -e 's;\/;\\\/;g' \
         -e 's;\[;\\\[;g' \
         -e 's;\];\\\];g' \
         -e 's;{;\\{;g'   \
         -e 's;};\\};g'   \
         -e 's;(;\\(;g'   \
         -e 's;);\\);g'   \
         -e 's;";\\\\\";g')"
         
    wpa_cli -i "$IFACE" list_networks \
     | tail -n +2 | cut -f -2 | grep -P "\t$ssid$" | cut -f1 \
     | while read -r ID; do
     echo "$ID"
      wpa_cli -i "$IFACE" remove_network "$ID" > /dev/null 2>&1
    done
    
    if [ -z "$IFACE" ]; then
      if [ "$INTERACTIVE" = True ]; then
#        whiptail --msgbox "No wireless interface found" 20 60
        echo "No wireless interface found"
      fi
      return 1
    fi
    
    if ! wpa_cli -i "$IFACE" status > /dev/null 2>&1; then
      if [ "$INTERACTIVE" = True ]; then
#        whiptail --msgbox "Could not communicate with wpa_supplicant" 20 60
        echo "Could not communicate with wpa_supplicant"
      fi
      return 1
    fi
    
    wpa_cli -i "$IFACE" save_config > /dev/null 2>&1
    echo "$IFACE_LIST" | while read -r IFACE; do
      wpa_cli -i "$IFACE" reconfigure > /dev/null 2>&1
    done
    
    return 0
  fi
  return 1
}

CMD=$1
SSID=$2
PSK=$3
#echo "Running: do_wifi_ssid_passphrase $1 $2"

if [ "$CMD" = "mk" ]; then
    do_wifi_ssid_passphrase "$SSID" "$PSK"
elif [ "$CMD" = "rm" ]; then
    delete_network "$SSID"
    return 0
else
    echo "Usage: $0 [mk|rm] <SSID> <PSK>"
    return
fi

#retval=$( do_wifi_ssid_passphrase "$1" "$2" )
retval=$?

echo "retval=$retval"

if [ $retval -eq 0 ]; then
     echo "Wifi configuration success, checking connection..."
#     while [ iwconfig | grep -q "$1" ]; do
    
     i=1
#     for i in 1 2 3 4 5 6 7 8 9 10
     while [ "$i" -lt 21 ];
     do
        iwconfig 2>&1 | grep -q "$SSID"
        if [ $? -eq 0 ]; then
            echo "Success!"
            wpa_cli -i wlan0 select_network any > /dev/null 2>&1
            wpa_cli -i wlan0 save_config > /dev/null 2>&1
            return 0
        fi
        sleep 1
        echo "Waiting... $i"
        i=$((i + 1))
     done
     echo "Unable to connect, wrong psk?"
                 
    wpa_cli -i wlan0 select_network any > /dev/null 2>&1
    wpa_cli -i wlan0 save_config > /dev/null 2>&1
     return 2
else
     echo "Wifi configuration failed"
                 
    wpa_cli -i wlan0 select_network any > /dev/null 2>&1
    wpa_cli -i wlan0 save_config > /dev/null 2>&1
     return 1
fi

return 0




