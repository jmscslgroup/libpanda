#!/bin/bash
# This is the script installed by the x725 installtion process
# This simply invokes a safe-shutdown signal by faking a button press

#BUTTON=18 # x725
BUTTON=13 # x728 first versions
BUTTONTWO=26 # x728 v2.0 and higher

echo "$BUTTON" > /sys/class/gpio/export;
echo "out" > /sys/class/gpio/gpio$BUTTON/direction
echo "1" > /sys/class/gpio/gpio$BUTTON/value

echo "$BUTTONTWO" > /sys/class/gpio/export;
echo "out" > /sys/class/gpio/gpio$BUTTONTWO/direction
echo "1" > /sys/class/gpio/gpio$BUTTONTWO/value

SLEEP=${1:-4}

re=^[0-9.]+
if ! [[ $SLEEP =~ $re ]] ; then
	echo "error: sleep time not a number" >&2; exit 1
fi

echo "X728 Shutting down..."
echo $SLEEP
/bin/sleep $SLEEP


#restore GPIO 18
echo "0" > /sys/class/gpio/gpio$BUTTON/value
echo "$BUTTON" > /sys/class/gpio/unexport;

echo "0" > /sys/class/gpio/gpio$BUTTONTWO/value
echo "$BUTTONTWO" > /sys/class/gpio/unexport;
