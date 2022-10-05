#/bin/bash
#Authors: Matt Bunting, Matt Nice
if [ $EUID -ne 0 ]; then
    echo "$0 not running as root. Please call using sudo."
    exit 2
fi

Help()
{
   # Display Help
   echo "Add description of the script functions here."
   echo
   echo "Syntax: scriptTemplate [-o|h]"
   echo "options:"
   echo "o     One line use. Use VIN as input, bash read inputs."
   echo "h     Print this Help."
   echo
}

Runit()
{
  # Run VIN Update
  sudo echo $Name > /etc/libpanda.d/vin
  sudo perl -pi -e 'chomp if eof' /etc/libpanda.d/vin
  sudo python3 /home/circles/libpanda/scripts/vinParser.py
  # sudo python3 ./vinParser.py
}
############################################################
############################################################
# Main program                                             #
############################################################
############################################################

# Set variables
# Name="goodbye"
############################################################
# Process the input options. Add options as needed.        #
############################################################
while getopts ":ho" option; do
   case $option in
      h) # display Help
         Help
         exit;;
      o) #one line Use
         Name=$2;
         echo "Setting VIN to $Name without asking to confirm."
         Runit
         exit;;
     \?) # Invalid option
         echo "Error: Invalid option"
         exit;;
   esac
done

# echo "hello $Name"

if test "$1" != "-o"; then
  # echo "hello $Name"

  echo "The current vin is " $(cat /etc/libpanda.d/vin)

  read -r -p "Enter your vin, or press enter or ctl-C to quit: " VIN

  if [ "x"$VIN != "x" ]; then
    read -r -p "You have entered '$VIN' -- would you like to set this as your VIN? [y/N] " response
    case "$response" in
        [yY][eE][sS]|[yY])
            sudo echo $VIN > /etc/libpanda.d/vin
            sudo perl -pi -e 'chomp if eof' /etc/libpanda.d/vin
            sudo python3 /home/circles/libpanda/scripts/vinParser.py
            # sudo python3 ./vinParser.py
  	  echo "The vin is updated to " $(cat /etc/libpanda.d/vin)
            ;;
        *)
            echo "Exiting with no changes"
            ;;
    esac
  fi
fi
