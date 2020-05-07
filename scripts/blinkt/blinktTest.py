import colorsys
import time
import math

from blinkt import set_brightness, set_pixel, show

pi = 3.1415926535897

set_brightness(0.1)

while True:
	hue = (time.time()*16.0 % 360)/360.0
	for x in range(8):
		lev = pow(math.cos((x-3.5)*2.0*pi/64.0 + math.sin(time.time()*0.4)*pi/4.0), 64.0)
		sat = (lev*1.0)
		r, g, b = [int(pow(c,2.2) * 255) for c in colorsys.hsv_to_rgb(hue, sat, lev)]
		set_pixel(x, r, g, b)

	show()
	time.sleep(1.0/60.0)
