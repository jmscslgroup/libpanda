import colorsys
import time
import math

from blinkt import set_pixel, show

pi = 3.1415926535897


maxLevel = 0.250	# Max = 1.0, 0.0 = off
updateRate = 1.0	# Hz

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

while True:
	#hue = (time.time()*16.0 % 360)/360.0
	#for x in range(8):
		#lev = pow(math.cos((x-3.5)*2.0*pi/64.0 + math.sin(time.time()*0.4)*pi/4.0), 64.0)
		#sat = (lev*1.0)
		#r, g, b = [int(pow(c,2.2) * 255) for c in colorsys.hsv_to_rgb(hue, sat, lev)]
		#set_pixel(x, r, g, b)

	try:
		hasInternet = int(getFileContents( fileInternet ))
		isApClient = int(getFileContents( fileIsApClient ))
		isApHost = int(getFileContents( fileisApHost ))
		hasApClients = int(getFileContents( fileHasApClients ))

		hasExternalPower = int(getFileContents( fileX725HasExtenalPower ))
		batteryCurrent = float(getFileContents( fileX725BatteryCurrent ))
		batteryVoltage = float(getFileContents( fileX725BatteryVoltage ))
		capacity = float(getFileContents( fileX725Capacity ))
	except:
		continue
# LED 0
	hue = (time.time()*32.0 % 360)/360.0
	r, g, b = [int(pow(c,2.2) * 255) for c in colorsys.hsv_to_rgb(hue, 1, maxLevel)]
	set_pixel(0, r, g, b)

# LED 1
#	print("Has Internet: " + str(hasInternet) )
	#blue = has internet, red otherwise
	if hasInternet:
		set_pixel(1, 0, 0, int(255.0*maxLevel))
	else:
		set_pixel(1, int(255.0*maxLevel), 0, 0)

# LED 2
# blue for is host, green for is client, red for neither
	if isApHost:
		set_pixel(2, 0, 0, int(255.0*maxLevel))
	elif isApClient:
		set_pixel(2, 0, int(255.0*maxLevel), 0)
	else:
		set_pixel(2, int(255.0*maxLevel), 0, 0)

# LED 3:
# purple for having clients, off otherwise
#	print("Has Clients: " + str(hasApClients) )
	if hasApClients:
		set_pixel(3, 100, 0, 100)
	else:
		set_pixel(3, 0, 0, 0)

# LED 4:
	newCapacity = max(min(capacity, 100.0), 0.0)
	hue = newCapacity/100.0 * 120.0/360.0	# green = charged, red = discharged
	r, g, b = [int(pow(c,2.2) * 255) for c in colorsys.hsv_to_rgb(hue, 1.0, maxLevel)]
	set_pixel(4, r, g, b)

# LED 5:
	newVoltage = max(min(batteryVoltage, 4.2), 3.0)
	hue = (newVoltage-3.0)/(4.2 - 3.0) * 120.0/360.0	# green = charged, red = discharged
	r, g, b = [int(pow(c,2.2) * 255) for c in colorsys.hsv_to_rgb(hue, 1.0, maxLevel)]
	set_pixel(5, r, g, b)

# LED 6:
#	print("Battery current = " + str(batteryCurrent))
	batteryCurrent = max(min(batteryCurrent, 1.0), -1.0)
#	print("new Battery current = " + str(batteryCurrent))
	hue = (-(batteryCurrent - -1.0)/(1.0 - -1.0) * 120.0 + 360.0)/360.0	# green = charged, red = discharged
#	print(" - hue = " + str(hue))
	r, g, b = [int(pow(c,2.2) * 255) for c in colorsys.hsv_to_rgb(hue, 1.0, maxLevel)]
	set_pixel(6, r, g, b)

# LED 7:
	if hasExternalPower:
		set_pixel(7, 0, int(255.0*maxLevel), int(255.0*maxLevel))
	else:
		set_pixel(7, int(255.0*maxLevel), 0, 0)


	show()
	time.sleep(1.0/updateRate)
