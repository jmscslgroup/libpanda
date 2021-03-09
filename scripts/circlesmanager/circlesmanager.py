import colorsys
import time
import math
import os

import logging

logging.basicConfig(level="INFO")

updateRate = 0.5	# Hz

fileInternet = "/etc/libpanda.d/hasinternet"
fileIsApClient = "/etc/libpanda.d/isapclient"
fileisApHost = "/etc/libpanda.d/isaphost"
fileHasApClients = "/etc/libpanda.d/hasapclients"

fileX725HasExtenalPower = "/etc/libpanda.d/x725hasexternalpower"
fileX725BatteryCurrent = "/etc/libpanda.d/x725batterycurrent"
fileX725BatteryVoltage = "/etc/libpanda.d/x725batteryvoltage"
fileX725Capacity = "/etc/libpanda.d/x725capacity"

def getFileContents( filename ):
	f = open(filename, "r")
	contents = f.read()
	f.close()
	#	print("From " + filename + " Read: " + contents )
	return contents

class CirclesManager:
	def __init__(self, powerOffTime=60):
		self.powerOffTimeoutInSeconds = powerOffTime
		self.powerDisconnectTime = 0

	def loop(self):
		logging.info("in loop()")

		try:
			hasInternet = int(getFileContents( fileInternet ))
			isApClient = int(getFileContents( fileIsApClient ))
			isApHost = int(getFileContents( fileisApHost ))
			hasApClients = int(getFileContents( fileHasApClients ))

			hasExternalPower = bool(int(getFileContents( fileX725HasExtenalPower )))
			batteryCurrent = float(getFileContents( fileX725BatteryCurrent ))
			batteryVoltage = float(getFileContents( fileX725BatteryVoltage ))
			capacity = float(getFileContents( fileX725Capacity ))
		except Exception as e:
			logging.info(e)
			return

		logging.info("Battery Voltage: " + str(batteryVoltage))
		logging.info("External Power:  " + str(hasExternalPower))

		if not hasExternalPower:
			self.powerDisconnectTime += 1.0/updateRate
			logging.info(" - Power disconnected, shutting down in " + str(self.powerOffTimeoutInSeconds - self.powerDisconnectTime))
			if self.powerDisconnectTime >= self.powerOffTimeoutInSeconds:
				logging.info(" - - Power disconnect timeout!  Running final scripts...")
				os.system("echo \"- - - This is where we run a script\"")
				logging.info(" - - Shutting system down...")
				f = open(fileX725HasExtenalPower, "w")
				f.write("1")
				f.close()
				os.system("x725shutdown")
		else:
			self.powerDisconnectTime = 0

if __name__ == "__main__":
	f = open(fileX725HasExtenalPower, "w")
	f.write("1")
	f.close()

	myCirclesManager = CirclesManager(60*10);

	while True:
		time.sleep(1.0/updateRate)
		myCirclesManager.loop()

