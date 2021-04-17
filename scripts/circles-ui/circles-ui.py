# doc-export: SineExample
"""
A sine, with sliders to manipulate phase and amplitude.
"""

from flexx import flx, event
import subprocess
import threading
import time
import asyncio
import logging

logging.basicConfig(level="INFO")

fileVin = "/etc/libpanda.d/vin"

fileX725BatteryVoltage = "/etc/libpanda.d/x725batteryvoltage"
fileX725Capacity = "/etc/libpanda.d/x725capacity"

filePandaRecording = "/etc/libpanda.d/pandaRecording"
filePandaGps = "/etc/libpanda.d/pandaHaveGPS"

def getFileContents( filename ):
	f = open(filename, "r")
	contents = f.read()
	f.close()
	#	print("From " + filename + " Read: " + contents )
	return contents
	
#class Store(flx.PyComponent):
#	timeString = flx.StringProp(settable=True)

class Relay(flx.Component):
#	def init(self):
#		super().init()
#		self.time = "Relay Default Time"
#		self.newTime = False
#
#	def setTime(self, time):
#		self.time = time
#		self.newTime = True
	vin = flx.StringProp("", settable=True)
	timeString = flx.StringProp("", settable=True)
	voltage = flx.FloatProp(0, settable=True)
	capacity = flx.FloatProp(0, settable=True)
	pandaRecording = flx.IntProp(0, settable=True)
	pandaGps = flx.IntProp(0, settable=True)

	@flx.reaction('vin')
	def on_vin(self, *events):
		for ev in events:
			self.updateVin(ev.new_value)
			
	@flx.reaction('timeString')
	def on_timeString(self, *events):
		for ev in events:
			self.updateTime(ev.new_value)
			
	@flx.reaction('voltage')
	def on_voltage(self, *events):
		for ev in events:
			self.updateVoltage(ev.new_value)
			
	@flx.reaction('capacity')
	def on_voltage(self, *events):
		for ev in events:
			self.updateCapacity(ev.new_value)
			
	@flx.reaction('pandaRecording')
	def on_pandaRecording(self, *events):
		for ev in events:
			self.updatePandaRecording(ev.new_value)
			
	@flx.reaction('pandaGps')
	def on_pandaGps(self, *events):
		for ev in events:
			self.updatePandaGps(ev.new_value)
			
	""" Global object to relay paint events to all participants.
	"""
	@flx.emitter
	def updateVin(self, value):
		return dict(value=value)
		
	@flx.emitter
	def updateTime(self, value):
		return dict(value=value)
		
	@flx.emitter
	def updateVoltage(self, value):
		return dict(value=value)
		
	@flx.emitter
	def updateCapacity(self, value):
		return dict(value=value)
		
	@flx.emitter
	def updatePandaRecording(self, value):
		return dict(value=value)
		
	@flx.emitter
	def updatePandaGps(self, value):
		return dict(value=value)
		
# Create global relay object, shared by all connections
relay = Relay()

class CirclesViewController(flx.PyWidget):
#	store = flx.ComponentProp()
	def init(self):
		super().init()
		
#		self._mutate_store(Store())
		self.circlesView = CirclesView(self)
		#self.circlesView.set_data()
	def test(self):
		print("Object method call test cuccess")
		
	
	@relay.reaction('updateVin')
	def _updateVin(self, *events):
		for ev in events:
			self.circlesView.updateVin(ev.value)
			
	@relay.reaction('updateTime')
	def _updateTime(self, *events):
		for ev in events:
			self.circlesView.updateTime(ev.value)
			
	@relay.reaction('updateVoltage')
	def _updateVoltage(self, *events):
		for ev in events:
			self.circlesView.updateVoltage(ev.value)
			
	@relay.reaction('updateCapacity')
	def _updateCapacity(self, *events):
		for ev in events:
			self.circlesView.updateCapacity(ev.value)
			
	@relay.reaction('updatePandaRecording')
	def _updatePandaRecording(self, *events):
		for ev in events:
			self.circlesView.updatePandaRecording(ev.value)
			
	@relay.reaction('updateGps')
	def _updateGps(self, *events):
		for ev in events:
			self.circlesView.updateGps(ev.value)

class CirclesView(flx.PyWidget):

	def init(self, model):
		super().init()
		self.model = model
	
		times = [i/20 for i in range(20)]
		with flx.VBox():
			with flx.HBox():
				with flx.VBox():
					flx.Label(text='Settings', style='text-align:center;font-weight: bold;')
					with flx.HBox(flex=0):
						with flx.VBox():
							flx.Label(style="text-align:right",text='VIN:')
							flx.Label(style="text-align:right",text='Controls Enabled:')
							flx.Label(style="text-align:right",text='Time:')
							flx.Label(style="text-align:right",text='Recording:')
							flx.Label(style="text-align:right",text='GPS Fix:')
							flx.Label(style="text-align:right",text='Battery Voltage:')
							flx.Label(style="text-align:right",text='Battery Capacity:')
						with flx.VBox():
							self.labelVin = flx.Label(style="text-align:left",text=relay.vin)
							self.labelControls = flx.Label(style="text-align:left",text='False')
							self.labelTime = flx.Label(style="text-align:left",text='Not Set')
							#self.labelTime = flx.Label(style="text-align:left",text=lambda: self.root.store.time)
							self.labelRecording = flx.Label(style="text-align:left",text=str(relay.pandaRecording))
							self.labelGps = flx.Label(style="text-align:left",text=str(relay.pandaGps))
							self.labelVoltage = flx.Label(style="text-align:left",text=str(relay.voltage))
							self.labelCapacity = flx.Label(style="text-align:left",text=str(relay.capacity))
					with flx.VBox():
						self.b1 = flx.Button(text='Start')
						self.b2 = flx.Button(text='Stop')
			#	self.
				with flx.VBox():
					flx.Label(text='Plot', style='text-align:center;font-weight: bold;')
					with flx.HBox():
						flx.Label(text='Time:')
						self.slider1 = flx.Slider(min=1, max=10, value=5, flex=1)
						flx.Label(text='Magnitude:')
						self.slider2 = flx.Slider(min=0, max=10, value=0, flex=1)
					self.plot = flx.PlotWidget(flex=1, xdata=times, xlabel='time',
                                       ylabel='Acc', title='Profile')
		#while True:
		#self.tick()
		#time.sleep(1)

	@flx.reaction
	def __update_amplitude(self, *events):
		#global Math
		freq, phase = self.slider1.value, self.slider2.value
		ydata = []
		for x in self.plot.xdata:
			ydata.append((freq*x*2+phase))
		self.plot.set_data(self.plot.xdata, ydata)
		
	#def updateWindow(self, text):
	#	self.labelTime.set_text( text )
			
	@flx.action
	def updateVin(self, value):
		self.labelVin.set_text( value )
		
	@flx.action
	def updateTime(self, time):
		self.labelTime.set_text( time )
		
	@flx.action
	def updateVoltage(self, value):
		self.labelVoltage.set_text( value )
		
	@flx.action
	def updateCapacity(self, value):
		self.labelCapacity.set_text( value )
		
	@flx.action
	def updatePandaRecording(self, value):
		self.labelRecording.set_text( value )
		
	@flx.action
	def updatePandaGps(self, value):
		self.labelGps.set_text( value )
		
	#def tick(self):
		#global window
		#t = time()
		#self.updateWindow("hello!")
		#if relay.newTime:
		#	relay.updateTime("ticked")
		#window.setTimeout(self.tick, 100)
		
#circlesUi = flx.App(CirclesViewController)
#class Updater(threading.Thread):
class Updater(threading.Thread):
	def __init__(self, loop):
		threading.Thread.__init__(self)
		self.timeString = ""
		self.keepRunning = True;
		self.loop=loop
		self.count = 0
	
	
	def update(self):
		systemTime = subprocess.run(["date"], stdout=subprocess.PIPE, text=True, input="")
		#systemTime.wait();
		#self.labelTime.set_text(systemTime.stdout)
		self.timeString = systemTime.stdout
		#print("New Time: " + self.timeString)
			
#		relay.updateTime(" updater string")
		#try:
		#relay.set_vinString("1234567890m ")
		relay.set_timeString(self.timeString)
		
		try:
			relay.set_vin( getFileContents( fileVin ) )
		except Exception as e:
			logging.info(e)
		try:
			relay.set_voltage( float(getFileContents( fileX725BatteryVoltage )) )
		except Exception as e:
			logging.info(e)
		try:
			relay.set_capacity( float(getFileContents( fileX725Capacity )) )
		except Exception as e:
			logging.info(e)
		try:
			relay.set_pandaRecording( int(getFileContents( filePandaRecording )) )
		except Exception as e:
			logging.info(e)
		try:
			relay.set_pandaGps( int(getFileContents( filePandaGps )) )
		except Exception as e:
			logging.info(e)
			
		self.count+=1
		
				
		
		#except Exception as e:
		#	print("error: " + str(e))
		#event.loop.add_reaction_event(CirclesViewController._updateTime, dict(time='WTF!'))
		#event.loop.call_soon(relay.updateTime, "ok....")
		#print("Done")
	
	def stop(self):
		self.keepRunning = False;
			
	def run(self):
		self.keepRunning = True
		self.update()
		
class Networking(flx.PyWidget):
	def init(self):
		with flx.VBox():
			flx.Label(style="text-align:center",text="Networking")
			with flx.HBox():
				with flx.VBox():
					flx.Label(style="text-align:right",text="AP Name:")
					flx.Label(style="text-align:right",text="Password:")
				with flx.VBox():
					self.fieldSsid = flx.LineEdit(style="background-color:#BBBBBB;text-align:center",placeholder_text="ssid")
					self.fieldPassword = flx.LineEdit(style="background-color:#BBBBBB;text-align:center",placeholder_text="psk")
			self.b1 = flx.Button(text='Submit')
			flx.Widget(flex=1)
	
class Circles(flx.PyWidget):
	def init(self):
		with flx.TabLayout():
			CirclesViewController(title='Status')
			Networking(title='Networking')
			
def start_flexx(loop):
	#circlesApp = flx.App(CirclesViewController, title='Circles')
	##circlesApp.test()
	#circlesApp.serve()
	
	flx.App(Circles, title='Circles').serve()
	
	flx.create_server(host='0.0.0.0', port=8080, loop=loop)
	
	
	flx.start()


		

if __name__ == '__main__':
	#m = flx.launch(CirclesUI)
	
	
	
	#flx.create_server(host='0.0.0.0',port=8080)
	#flx.run()
	
	#circlesUi = CirclesUI()
	#circlesUi  = flx.launch(CirclesViewController)
	
		
	loop = asyncio.new_event_loop()
	t = threading.Thread(target=start_flexx, args=(loop,))
	t.start()
	
	updater = Updater(loop)
	#updater.start()
	
	keepRunning = True
	while keepRunning:
		updater.update()
		time.sleep(1)
		
	updater.stop()
	updater.join()
