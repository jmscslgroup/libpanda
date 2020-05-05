#/bin/bash

if [ $EUID -ne 0 ]; then
    echo "$0 not running as root. Please call using sudo."
    exit 2
fi

echo "The current vin is " $(cat /etc/libpanda.d/vin)

read -r -p "Enter your vin, or press enter or ctl-C to quit: " VIN 

if [ "x"$VIN != "x" ]; then
  read -r -p "You have entered '$VIN' -- would you like to set this as your VIN? [y/N] " response
  case "$response" in
      [yY][eE][sS]|[yY])
          sudo echo $VIN > /etc/libpanda.d/vin
	  echo "The vin is updated to " $(cat /etc/libpanda.d/vin)
          ;;
      *)
          echo "Exiting with no changes"
          ;;
  esac
fi
