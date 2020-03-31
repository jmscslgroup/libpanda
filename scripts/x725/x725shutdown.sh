#!/bin/bash
# This is the script installed by the x725 installtion process
# This simply invokes a safe-shutdown signal by faking a button press

BUTTON=18

echo "$BUTTON" > /sys/class/gpio/export;
echo "out" > /sys/class/gpio/gpio$BUTTON/direction
echo "1" > /sys/class/gpio/gpio$BUTTON/value

SLEEP=${1:-4}

re=^[0-9.]+
if ! [[ $SLEEP =~ $re ]] ; then
	echo "error: sleep time not a number" >&2; exit 1
fi

echo "X750 Shutting down..."
echo $SLEEP
/bin/sleep $SLEEP


#restore GPIO 18
echo "0" > /sys/class/gpio/gpio$BUTTON/value

echo "$BUTTON" > /sys/class/gpio/unexport;
