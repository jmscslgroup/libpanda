#!/usr/bin/env python

import struct
import smbus
import sys
import time

from datetime import datetime

def readVoltage(bus):
	address = 0x36
	read = bus.read_word_data(address, 2)
	swapped = struct.unpack("<H", struct.pack(">H", read))[0]
	voltage = swapped * 1.25 /1000/16
	return voltage


def readCapacity(bus):
	address = 0x36
	read = bus.read_word_data(address, 4)
	swapped = struct.unpack("<H", struct.pack(">H", read))[0]
	capacity = swapped/256
	return capacity


bus = smbus.SMBus(1) # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

run = True

while run:
	print " "
	print "*********************"
	now = datetime.now()
	current_time = now.strftime("%H:%M:%S")
	print "Current Time:%s" %current_time
	print "Voltage:%5.5fV" % readVoltage(bus)

	print "Battery:%5i%%" % readCapacity(bus)

	if readCapacity(bus) == 100:

		print "Battery FULL"
		run = False

	if readCapacity(bus) < 20:


		print "Battery LOW"

	if readCapacity(bus) == 0:
		print "DEAD!"
		run = False

		print "*********************"
		time.sleep(2)


