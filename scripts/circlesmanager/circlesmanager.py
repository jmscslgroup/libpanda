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

fileVin = "/etc/libpanda.d/vin"

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
		self.powerDisconnectTimeHysteresisWaitTime = 60

	def loop(self):
		#logging.info("in loop()")

		hasExternalPower = True
		try:
			#hasInternet = int(getFileContents( fileInternet ))
			isApClient = int(getFileContents( fileIsApClient ))
			isApHost = int(getFileContents( fileisApHost ))
			hasApClients = int(getFileContents( fileHasApClients ))
		except Exception as e:
			logging.info(e)

		try:
			hasExternalPower = bool(int(getFileContents( fileX725HasExtenalPower )))
			batteryCurrent = float(getFileContents( fileX725BatteryCurrent ))
			batteryVoltage = float(getFileContents( fileX725BatteryVoltage ))
			capacity = float(getFileContents( fileX725Capacity ))
			
			logging.info("Battery Voltage: " + str(batteryVoltage))
			logging.info("External Power:  " + str(hasExternalPower))
		except Exception as e:
			logging.info(e)
			
		try:
			os.system("vinToHostname")
		except Exception as e:
			logging.info(e)
		

		

		if not hasExternalPower:
			self.powerDisconnectTime += 1.0/updateRate
			
			if self.powerDisconnectTime < self.powerDisconnectTimeHysteresisWaitTime:
				logging.info(" - Power disconnected, waiting for power reconnect -- " + str(self.powerDisconnectTimeHysteresisWaitTime - self.powerDisconnectTime))
				continue
				
			logging.info(" - Power reconnect timeout, shutting down in " + str(self.powerOffTimeoutInSeconds - self.powerDisconnectTime))
			
			# Check for internet connectivity.
			if os.system("simplePing") == 0:	# Error code 0 means success
				os.system("echo \"- - -  Internet was found!  Running irsync\"")
				#os.system("irsyncCyverse -f")
				pandarecordWasRunning = os.system("simpleCheckPandarecord")
				canWasRunning = os.system("simpleCheckCan")
				os.system("service pandarecord stop")
				os.system("service rosnodeChecker stop")
				os.system("service can stop")
				os.system("runuser -l circles -c 'irsyncCyverse -f'")
				try:	# This whole try statement is a hack fix for preventing shutdowns if power was reconnected during irsync
					hasExternalPower = bool(int(getFileContents( fileX725HasExtenalPower )))
					if not hasExternalPower:
						self.powerDisconnectTime = self.powerOffTimeoutInSeconds
					else:
						if pandarecordWasRunning:
							os.system("service pandarecord start")
						if canWasRunning:
							os.system("service can start")
				except Exception as e:
					logging.info(e)
			
			if self.powerDisconnectTime >= self.powerOffTimeoutInSeconds:
				logging.info(" - - Power disconnect timeout!  Running final scripts...")
				#os.system("echo \"- - - This is where we run a script\"")
				logging.info(" - - Shutting system down...")
				os.system("sudo sh -c 'echo \"Rsync finished, shutting system down...\" > /etc/libpanda.d/logMessage'")
				time.sleep(20)
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

