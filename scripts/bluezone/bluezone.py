#!/usr/bin/python3

"""Copyright (c) 2019, Douglas Otwell

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Modified by Matt Bunting
"""

import dbus
import json

from advertisement import Advertisement
from service import Application, Service, Characteristic, Descriptor
from gpiozero import CPUTemperature

GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
NOTIFY_TIMEOUT = 5000

class CirclesAdvertisement(Advertisement):
    def __init__(self, index):
#        Advertisement.__init__(self, index, "peripheral")
        Advertisement.__init__(self, index, "broadcast")
        self.add_local_name("CIRCLES")
        self.include_tx_power = True
 #       self.add_service_uuid("00000001-710e-4a5b-8d75-3e5b444bc3cf")
        self.add_service_uuid("00000001-3d3d-3d3d-3d3d-3d3d3d3d3d3d")
#        self.add_solicit_uuid("00000001-710e-4a5b-8d75-3e5b444bc3cf")

class CirclesService(Service):
    CIRCLES_SVC_UUID = "00000001-3d3d-3d3d-3d3d-3d3d3d3d3d3d"

    def __init__(self, index):
        self.farenheit = True
        self.circle = True
        self.command = " "

        Service.__init__(self, index, self.CIRCLES_SVC_UUID, True)
        self.add_characteristic(TempCharacteristic(self))
        self.add_characteristic(UnitCharacteristic(self))
        self.outputCharacteristic = CirclesOutputCharacteristic(self)
        self.add_characteristic(self.outputCharacteristic)
        self.add_characteristic(CirclesInputCharacteristic(self))
        self.add_characteristic(CirclesCommandCharacteristic(self))
        self.add_characteristic(CirclesConfigCharacteristic(self))
        self.statusCharacteristic = CirclesStatusCharacteristic(self)
        self.add_characteristic(self.statusCharacteristic)

    def is_farenheit(self):
        return self.farenheit

    def set_farenheit(self, farenheit):
        self.farenheit = farenheit
        
    def get_command(self):
        return self.command
    
    def set_command(self, command):
        if command == "R":
            print("Read command!")
            self.outputCharacteristic.send_zone_file()
            
        if command == "{example: 'data'}":
            print("Send command!")
            self.outputCharacteristic.StopNotify()
            
        self.command = command

    def set_status(self, s):
        print("Setting status to " + str(s))
        self.statusCharacteristic.send_status(s)
        

# this is structured to send large amounts of data to the paired device
class CirclesOutputCharacteristic(Characteristic):
    CIRCLES_OUTPUT_CHARACTERISTIC_UUID = "00000002-3d3d-3d3d-3d3d-3d3d3d3d3d3d"
    MAX_SEND = 500
    
    def __init__(self, service):
        self.notifying = False
        self.index = 0
#        self.count = 10

        Characteristic.__init__(
                self, self.CIRCLES_OUTPUT_CHARACTERISTIC_UUID,
                ["notify", "read"], service)
        self.add_descriptor(CirclesOutputDescriptor(self))
                
    def get_circles(self):
        value = []

        strtemp = "Hello Circles! - " + self.service.get_command()
        for c in strtemp:
            value.append(dbus.Byte(c.encode()))

        #return value
        return value
        
    def send(self, val):
        #alue = str(self.count)
        value = []
        for c in val:
            value.append(dbus.Byte(c.encode()))
        if self.index+self.MAX_SEND >= len(self.buffer):
            value.append(dbus.Byte(0))
        #print("in send()")
        #print("-- set_circles_callback(): Sending notification: " + str(val))
        self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])
    
    def set_circles_callback(self):
        print("in set_circles_callback....")
        if self.notifying:
#            self.count -= 1
#            value = str(self.count)
            #print("set_circles_callback(): Sending notification: " + str(value))
            #self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])
            #self.send(self.buffer[self.index])
            self.send(self.buffer[self.index:self.MAX_SEND+self.index])
            self.index += self.MAX_SEND
            #self.send(" 123")
            if self.index >= len(self.buffer):
                self.StopNotify()

        return self.notifying
        
    def send_zone_file(self):
        self.readZoneFile() # this sets the buffer
        self.StartNotify()
        
    def StartNotify(self):
        print("in StartNotify(self)")
        if self.notifying or len(self.buffer) <= 0:
            return

        #self.buffer = "Hello buffer!"
        #self.readZoneFile()
        self.index = 0
#        self.count = 5
        self.notifying = True
    
        print("in StartNotify(self): sendinf value...")
       # value = str(self.get_circles())
#        value = self.get_circles() + ": " + str(self.count)
        #print("Startin notificaiton: " + str(value))
        #self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])
        self.send(self.buffer[self.index:self.MAX_SEND+self.index])
        self.index += self.MAX_SEND
        #self.send(" 123")
        print("in StartNotify(self): starting callback...")
        self.add_timeout(50, self.set_circles_callback)

    def StopNotify(self):
        self.notifying = False

    def ReadValue(self, options):
#        self.count -= 1
        value = str(self.count) + ": " + self.get_circles()
#           value = self.get_circles()
#        if self.count == 0:
#            self.StopNotify()

        return value

    def readZoneFile(self):
        myJson = { 'type': 'zonefile' };
        file = open('/etc/libpanda.d/zone-processed.json')
#        self.buffer = file.read()
        contents = file.read()
        myJson['contents'] = contents
        file.close()
        myJson['length'] = len(contents)
        print("contents length: " + str(len(contents)))
        
        self.buffer = json.dumps(myJson)
        

class CirclesOutputDescriptor(Descriptor):
    CIRCLES_OUTPUT_DESCRIPTOR_UUID = "2910"
    CIRCLES_OUTPUT_DESCRIPTOR_VALUE = "CIRCLES Output descriptor"

    def __init__(self, characteristic):
        Descriptor.__init__(
                self, self.CIRCLES_OUTPUT_DESCRIPTOR_UUID,
                ["read"],
                characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.CIRCLES_OUTPUT_DESCRIPTOR_VALUE

        for c in desc:
            value.append(dbus.Byte(c.encode()))

        return value
        
        
        
class CirclesCommandCharacteristic(Characteristic):
    CIRCLES_COMMAND_CHARACTERISTIC_UUID = "00000003-3d3d-3d3d-3d3d-3d3d3d3d3d3d"

    def __init__(self, service):
        Characteristic.__init__(
                self, self.CIRCLES_COMMAND_CHARACTERISTIC_UUID,
                ["read", "write"], service)
        self.add_descriptor(CirclesCommandDescriptor(self))
        
    def WriteValue(self, value, options):
    # value is type dbus array.  string is located in first element of the array
        val = str(value[0]).upper()
        val = ''.join([str(v) for v in value])
        print("CirclesCommandCharacteristic.WriteValue(): received: " + str(value))
        print("CirclesCommandCharacteristic.WriteValue(): received: " + str(val))
#        print("CirclesCommandCharacteristic.WriteValue(): received: " + str(value.decode("utf-8") ))
        self.service.set_command(val)
#        if val == "C":
#            self.service.set_farenheit(False)
#        elif val == "F":
#            self.service.set_farenheit(True)

    def ReadValue(self, options):
        value = []
#        val = "R"
        val = self.service.get_command()
#        if self.service.is_farenheit(): val = "F"
#        else: val = "C"
        value.append(dbus.Byte(val.encode()))
        print("CirclesCommandCharacteristic.ReadValue(): " + str(value))

        return value
#    def WriteValue(self, value, options):
#        #val = str(value[0]).upper()
#        print("CirclesCommandCharacteristic.WriteValue(): received: " + str(value.decode("utf-8") ))
#       # self.service.set_command(val)
##        if val == "R":
##            self.service.set_farenheit(False)
##        elif val == "F":
##            self.service.set_farenheit(True)
#
#    def ReadValue(self, options):
#        value = []
#        val = "F" # self.service.get_command()
#
##        if self.service.is_farenheit(): val = "F"
##        else: val = "C"
#        value.append(dbus.Byte(val.encode()))
#        print("CirclesCommandCharacteristic.ReadValue(): " + str(value ))
#
#        return value

class CirclesCommandDescriptor(Descriptor):
    CIRCLES_COMMAND_DESCRIPTOR_UUID = "2904"
    CIRCLES_COMMAND_DESCRIPTOR_VALUE = "Circles Command"

    def __init__(self, characteristic):
        Descriptor.__init__(
                self, self.CIRCLES_COMMAND_DESCRIPTOR_UUID,
                ["read"],
                characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.CIRCLES_COMMAND_DESCRIPTOR_VALUE

        for c in desc:
            value.append(dbus.Byte(c.encode()))

        return value
        
        
class CirclesInputCharacteristic(Characteristic):
    CIRCLES_INPUT_CHARACTERISTIC_UUID = "00000005-3d3d-3d3d-3d3d-3d3d3d3d3d3d"
    
    def __init__(self, service):
        self.inputBuffer = ""
        
        Characteristic.__init__(
                self, self.CIRCLES_INPUT_CHARACTERISTIC_UUID,
                ["write"], service)
        self.add_descriptor(CirclesInputDescriptor(self))
        
    def WriteValue(self, value, options):
    # value is type dbus array.  string is located in first element of the array
        #val = str(value[0]).upper()
        val = ''.join([str(v) for v in value])
#        print("CirclesCnfigmandCharacteristic.WriteValue(): received: " + str(value))
        #print("CirclesInputCharacteristic.WriteValue(): received: " + val)
        
#        print("CirclesCommandCharacteristic.WriteValue(): received: " + str(value.decode("utf-8") ))
        #self.service.set_command(val)
        self.inputBuffer = self.inputBuffer + val
        
        
        print("len(value) = " + str(len(value)))
        if value[len(value)-1] == 0:
            print("Null terminated string!")
            self.complete(self.inputBuffer[0:len(self.inputBuffer)-1])
            self.inputBuffer = ""
        
        
    def complete(self, result):
        print("complete string: " + result)
        print("complete type: " + str(type(result)))
        inputJson = json.loads(result)
        print("json parsed... ")
        print("complete input type: " + inputJson["type"])
        print("complete input length: " + str(inputJson["length"]))
        print(" - checking length: " + str(len(inputJson["contents"])) )
        if len(inputJson["contents"]) == inputJson["length"]:
            self.service.set_status("OK")
        else:
            self.service.set_status("Length mismatch!")
    
class CirclesInputDescriptor(Descriptor):
    CIRCLES_INPUT_DESCRIPTOR_UUID = "2905"
    CIRCLES_INPUT_DESCRIPTOR_VALUE = "Circles Input"

    def __init__(self, characteristic):
        Descriptor.__init__(
                self, self.CIRCLES_INPUT_DESCRIPTOR_UUID,
                ["read"],
                characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.CIRCLES_INPUT_DESCRIPTOR_VALUE

        for c in desc:
            value.append(dbus.Byte(c.encode()))

        return value
        
        
class CirclesConfigCharacteristic(Characteristic):
    CIRCLES_CONFIG_CHARACTERISTIC_UUID = "00000004-3d3d-3d3d-3d3d-3d3d3d3d3d3d"

    def __init__(self, service):
        Characteristic.__init__(
                self, self.CIRCLES_CONFIG_CHARACTERISTIC_UUID,
                ["read", "write"], service)
        self.add_descriptor(CirclesConfigDescriptor(self))
        
    def WriteValue(self, value, options):
    # value is type dbus array.  string is located in first element of the array
        val = str(value[0]).upper()
        val = ''.join([str(v) for v in value])
#        print("CirclesCnfigmandCharacteristic.WriteValue(): received: " + str(value))
        print("CirclesConfigCharacteristic.WriteValue(): received: " + str(val))
#        print("CirclesCommandCharacteristic.WriteValue(): received: " + str(value.decode("utf-8") ))
        self.service.set_command(val)

    def ReadValue(self, options):
        value = []
#        val = "R"
#        val = self.service.get_command()
#        if self.service.is_farenheit(): val = "F"
#        else: val = "C"
        value.append(dbus.Byte(val.encode()))
        print("CirclesConfigCharacteristic.ReadValue(): " + str(value))

        return value

class CirclesConfigDescriptor(Descriptor):
    CIRCLES_CONFIG_DESCRIPTOR_UUID = "2904"
    CIRCLES_CONFIG_DESCRIPTOR_VALUE = "Circles Config"

    def __init__(self, characteristic):
        Descriptor.__init__(
                self, self.CIRCLES_CONFIG_DESCRIPTOR_UUID,
                ["read"],
                characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.CIRCLES_CONFIG_DESCRIPTOR_VALUE

        for c in desc:
            value.append(dbus.Byte(c.encode()))

        return value
        
class CirclesStatusCharacteristic(Characteristic):
    STATUS_CHARACTERISTIC_UUID = "00000006-3d3d-3d3d-3d3d-3d3d3d3d3d3d"

    def __init__(self, service):
        self.notifying = False

        Characteristic.__init__(
                self, self.STATUS_CHARACTERISTIC_UUID,
                ["notify", "read"], service)
        self.add_descriptor(CirclesStatusDescriptor(self))
        
    def send_status(self, s):
        value = []
        for c in s:
            value.append(dbus.Byte(c.encode()))
#        if self.index+self.MAX_SEND >= len(self.buffer):
#            value.append(dbus.Byte(0))
        self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])
        
#    def get_temperature(self):
#        value = []
#
#        cpu = CPUTemperature()
#        temp = cpu.temperature
#        if self.service.is_farenheit():
#            temp = (temp * 1.8) + 32
#            unit = "F"
#
#        strtemp = str(round(temp, 1)) + " " + unit
#        for c in strtemp:
#            value.append(dbus.Byte(c.encode()))
#
#        #return value
#        return value
#
#    def set_temperature_callback(self):
#        if self.notifying:
#            value = self.get_temperature()
#            #print("Sending notificaiton: " + str(value))
#            self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])
#
#        return self.notifying
#
#    def StartNotify(self):
#        if self.notifying:
#            return
#
#        self.notifying = True
#
#        value = self.get_temperature()
#        print("Startin notificaiton: " + str(value))
#        self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])
#        self.add_timeout(NOTIFY_TIMEOUT, self.set_temperature_callback)
#
#    def StopNotify(self):
#        self.notifying = False
#
#    def ReadValue(self, options):
#        value = self.get_temperature()
#
#        return value

class CirclesStatusDescriptor(Descriptor):
    STATUS_DESCRIPTOR_UUID = "2906"
    STATUS_DESCRIPTOR_VALUE = "Circles Status"

    def __init__(self, characteristic):
        Descriptor.__init__(
                self, self.STATUS_DESCRIPTOR_UUID,
                ["read"],
                characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.STATUS_DESCRIPTOR_VALUE

        for c in desc:
            value.append(dbus.Byte(c.encode()))

        return value
        

#class ThermometerService(Service):
#    THERMOMETER_SVC_UUID = "00000001-710e-4a5b-8d75-3e5b444bc3cf"
#
#    def __init__(self, index):
#        self.farenheit = True
#
#        Service.__init__(self, index, self.THERMOMETER_SVC_UUID, False)
#        self.add_characteristic(TempCharacteristic(self))
#        self.add_characteristic(UnitCharacteristic(self))
#
#    def is_farenheit(self):
#        return self.farenheit
#
#    def set_farenheit(self, farenheit):
#        self.farenheit = farenheit

class TempCharacteristic(Characteristic):
    TEMP_CHARACTERISTIC_UUID = "00000002-710e-4a5b-8d75-3e5b444bc3cf"

    def __init__(self, service):
        self.notifying = False

        Characteristic.__init__(
                self, self.TEMP_CHARACTERISTIC_UUID,
                ["notify", "read"], service)
        self.add_descriptor(TempDescriptor(self))

    def get_temperature(self):
        value = []
        unit = "C"

        cpu = CPUTemperature()
        temp = cpu.temperature
        if self.service.is_farenheit():
            temp = (temp * 1.8) + 32
            unit = "F"

        strtemp = str(round(temp, 1)) + " " + unit
        for c in strtemp:
            value.append(dbus.Byte(c.encode()))

        #return value
        return value

    def set_temperature_callback(self):
        if self.notifying:
            value = self.get_temperature()
            #print("Sending notificaiton: " + str(value))
            self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])

        return self.notifying

    def StartNotify(self):
        if self.notifying:
            return

        self.notifying = True

        value = self.get_temperature()
        print("Startin notificaiton: " + str(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])
        self.add_timeout(NOTIFY_TIMEOUT, self.set_temperature_callback)

    def StopNotify(self):
        self.notifying = False

    def ReadValue(self, options):
        value = self.get_temperature()

        return value

class TempDescriptor(Descriptor):
    TEMP_DESCRIPTOR_UUID = "2906"
    TEMP_DESCRIPTOR_VALUE = "CPU Temperature"

    def __init__(self, characteristic):
        Descriptor.__init__(
                self, self.TEMP_DESCRIPTOR_UUID,
                ["read"],
                characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.TEMP_DESCRIPTOR_VALUE

        for c in desc:
            value.append(dbus.Byte(c.encode()))

        return value

class UnitCharacteristic(Characteristic):
    UNIT_CHARACTERISTIC_UUID = "00000003-710e-4a5b-8d75-3e5b444bc3cf"

    def __init__(self, service):
        Characteristic.__init__(
                self, self.UNIT_CHARACTERISTIC_UUID,
                ["read", "write"], service)
        self.add_descriptor(UnitDescriptor(self))

    def WriteValue(self, value, options):
        val = str(value[0]).upper()
        print("received: " + str(value))
        if val == "C":
            self.service.set_farenheit(False)
        elif val == "F":
            self.service.set_farenheit(True)

    def ReadValue(self, options):
        value = []

        if self.service.is_farenheit(): val = "F"
        else: val = "C"
        value.append(dbus.Byte(val.encode()))
        print("read: " + str(value))

        return value

class UnitDescriptor(Descriptor):
    UNIT_DESCRIPTOR_UUID = "2903"
    UNIT_DESCRIPTOR_VALUE = "Temperature Units (F or C)"

    def __init__(self, characteristic):
        Descriptor.__init__(
                self, self.UNIT_DESCRIPTOR_UUID,
                ["read"],
                characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.UNIT_DESCRIPTOR_VALUE

        for c in desc:
            value.append(dbus.Byte(c.encode()))

        return value

app = Application()
app.add_service(CirclesService(0))
#app.add_service(ThermometerService(0))
app.register()

adv = CirclesAdvertisement(0)
adv.register()

try:
    app.run()
except KeyboardInterrupt:
    app.quit()
