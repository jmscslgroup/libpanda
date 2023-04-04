#!/bin/bash
#Author: Matt Bunting

#echo "Updating..."
#echo "Updating..." > /etc/libpanda.d/logMessage

export PYTHONPATH="/var/panda/cputemp:$PYTHONPATH"
bluezone.py

#echo "Updating done!"
#echo "Updating done!" > /etc/libpanda.d/logMessage

