Hardware tutorial                         {#mainpage}
============



# Table of Contents
* [About](#about)
* [Hardware](#hardware)
	* [List of Components](#hardware-list)
		* [Required](#hardware-req)
		* [Optional](#hardware-opt)
		* [Optional Power](#hardware-opt)
	* [Configuration](#hardware-config)
		* [Toyota RAV4](#hardware-rav4)
		* [Honda Pilot](#hardware-pilot)
* [Software](#software)
	* [Ubuntu 18.04 LTS](#software-OS)
	* [libpanda Setup](#libpanda-setup)
* [Software Guide](#software-guide)
	* [libpanda Services](#libpanda-services)
	* [libpanda Utilities](#libpanda-utilities)
		* [pandacord](#libpanda-pandacord)
		* [pandaSetSystemTime](#libpanda-pandaSetSystemTime)
		* [pandaCurses](#libpanda-pandaCurses)
* [Power Guide with x725](#power-guide)
* [Network Guide](#network-guide)
* [Data Visualization](#visualization)
* [Data Repository](#repository)


<a name="about"></a>
___
# About
Libpanda is a software library and set of utilities intended to interface with vehicles via comma.ai's Panda and optional Giraffe.  The source for the library is located here:

[https://github.com/jmscslgroup/libpanda](https://github.com/jmscslgroup/libpanda)

The majority of example source code to interface with the Panda is based on python and uses libusb.  We chose to write our own C++ version due to performance issues found with minimalist implementations in Python.  This effects both CPU usage nearing 92%, and missing nearly 50% of CAN data.  Libpanda is able to perform the same tasks as the Python version, with the addition of reading GPS data, at 35% usage, and with twice the throughput of data collection.  These tests were performed on a Raspberry Pi 4.  Here is a video demonstrating our findings:

\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/GavbXC_aOCY?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

> Note: After the video was recorded some small tuning resulted in CPU usage closer to 35% instead of 50%

<a name="hardware"></a>
___
# Hardware

<a name="hardware-list"></a>
___
## List of Components
The hardware choice is based around running linux with comm.ai hardware, leveraging prior efforts in CAN bus access.  

![Hardware Overview](/doc/images/overview.png "Overview")

<a name="hardware-req"></a>

Both the Black and Grey Panda is supported by libpanda.  It is recommended to use the Black Panda since it can do everything the Grey Panda can do, but also can connect tothe Car's OBD II port.  This means that the Black Panda is capable of reading 3 total CAN busses for information, wherase the Grey Panda can only listen to 2 CAN busses.  Comma.ai also sells a White Panda which features no GPS but WiFi instead. Libpanda will be ale to work with the White Panda over USB and will diasble GPS methods, however it is not listed as a supported device due to libpanda's lack of WiFi connection support.

#### Required for Black Panda:
* [comma.ai Car Harness](https://comma.ai/shop/products/panda-obd-ii-dongle)

Be sure to select the "Black Panda" option when ordering the Car Harness.  Also very importantly, read the [Hardware Configuration](#hardware-config) section to understand how **the Black Panda may brick your computer if not understood completely**.

#### Required for Grey Panda:
* [comma.ai Grey Panda](https://comma.ai/shop/products/panda-obd-ii-dongle)
* [comma.ai Giraffe](https://comma.ai/shop/products/giraffe)

	There are many flavors, be sure to select the Giraffe for your particular vehicle.

#### Recommended Computer Hardware:
Libpanda may be able to work on most linux machines, however the following hardware list is what is used for development and hould eliminate concerns of portability.

* [Raspberry Pi 4GB](https://www.amazon.com/Raspberry-Model-2019-Quad-Bluetooth/dp/B07TD42S27/ref=sxin_2_ac_d_pm?ac_md=4-0-VW5kZXIgJDUw-ac_d_pm&cv_ct_cx=raspberry+pi+4&keywords=raspberry+pi+4&pd_rd_i=B07TD42S27&pd_rd_r=897fc6f0-a9d6-430f-8811-c07c3c7b9e19&pd_rd_w=K0xrG&pd_rd_wg=WSCdw&pf_rd_p=0e223c60-bcf8-4663-98f3-da892fbd4372&pf_rd_r=AGXA47R72X2ZKA2F8X3Z&psc=1&qid=1583948920)
* [64GB SD card](https://www.amazon.com/Samsung-Class-Adapter-MB-MC64DA-AM/dp/B01273JZMG?tag=androidcentralb-20&ascsubtag=UUacUdUnU77910YYwYg)
*  [USB 2.0/3.0 to USB C adapter cable for power](https://www.amazon.com/Anker-Powerline-Pull-up-Resistor-Samsung/dp/B01A6F3WHG/ref=sr_1_5?keywords=usb+3+to+usb+C&qid=1583954394&s=electronics&sr=1-5)
* [Pimoroni Blinkt!](https://www.adafruit.com/product/3195)

	This is a nifty, inexpensive, and compact set of addressable LEDs that will be supported for displaying the state of various scripting components such as WiFi connectivity, data upload, and power states, but is not required for any functionality.
	


<a name="hardware-opt"></a>
#### Optional:

These items are for convenience and will not change the function of the baove hardware, but may be needed if you are new to using a Raspberry Pi.

* [SD Card reader for Ubuntu flashing](https://www.amazon.com/Vanja-Adapter-Portable-Memory-Reader/dp/B00W02VHM6/ref=sr_1_6?keywords=usb+sd+card&qid=1583949114&s=electronics&sr=1-6)
* [Micro HDMI adapter for Pi 4](https://www.amazon.com/GANA-Adapter-Female-Action-Supported/dp/B07K21HSQX/ref=sxin_2_ac_d_pm?ac_md=1-0-VW5kZXIgJDEw-ac_d_pm&cv_ct_cx=micro+hdmi&keywords=micro+hdmi&pd_rd_i=B07K21HSQX&pd_rd_r=b124f42c-a587-491e-9400-52aef81c3d88&pd_rd_w=mNPkI&pd_rd_wg=saxZx&pf_rd_p=0e223c60-bcf8-4663-98f3-da892fbd4372&pf_rd_r=J1YKVHPV73CBGDNP1MXZ&psc=1&qid=1583949163&s=electronics)
* [Ethernet Cable](https://www.amazon.com/AmazonBasics-RJ45-Cat-6-Ethernet-Patch-Cable-10-Feet-3-Meters/dp/B00N2VIALK/ref=sr_1_3?keywords=ethernet&qid=1583954176&sr=8-3)

<a name="hardware-opt-power"></a>
#### Optional power for data collection

* [x725 Raspebrry Pi UPS](https://www.amazon.com/Raspberry-Shutdown-Management-Expansion-Compatible/dp/B07Z3S42MK)
* [18650 Batteries with no built-in protection, qty 2 per x750](https://www.amazon.com/liogea-LG3400G-3400mAh-Rechargeable-Batteries/dp/B07YBTQSQL/ref=sr_1_2?keywords=18650b&qid=1585673515&s=electronics&sr=1-2)


The x725 and batteries are shown as optional since they are not required to record data, nor needed for future control of the vehicle.  They will allow for the automatic data-upload upon power shut-down from the vehicle by maintaining power to the Pi, which will be able to detect the new power state and invoke a CyVerse data synchronization before shutting itself down.


There are multiple ways to power the x725 board in a car that can make use of automatic data upload scripts.  The most important part for functionality as intended is that the power source turns off when the car is turned off.  

##### Fast Charge w/Debugging (most expensive)

For fastest charging at 3A, and for power debugging, the following list provides power information when in use.  This list requires a small amount of assembly.

* [DROK DC Buck Module with Display](https://www.amazon.com/gp/product/B07JZ2GQJF/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1)
* [Cigarette Lighter Plug with Cable](https://www.amazon.com/gp/product/B01HGO2OIS/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1)
* [2.5mm x 5.5mm Coaxial power plug](https://www.amazon.com/gp/product/B07VPHMLHC/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1)

> Note: The 2.5mm x 5.5mm uses 22 AWG wire shich may be too small for 3A charging, yet to be evaluated.

 ##### Fast Charge
 
For full 3A charging using USB with minimal install, you can use a a USB to 5.5mmx2.5mm barrel jack cable with higher current cigarette lighter USB source
* [USB to 5.5x2.5mm Adapter](https://www.amazon.com/TENINYU-Angled-Degree-5-5x2-5mm-Positive/dp/B07B11VK4S/ref=sr_1_2?keywords=usb+to+2.5mm+5.5+male&qid=1585767229&s=electronics&sr=1-2)
* [Cigarette Ligheter to USB adapter, 4.8A](https://www.amazon.com/Charger-RAVPower-Adapter-iSmart-Compatible/dp/B071FHZRQN/ref=sr_1_2?keywords=cigarrete+usb&qid=1585767324&s=electronics&sr=1-2)

##### Inexpensive (Slowest Charge)

Alternatively, if your vehicle has USB ports that turn off when the car is shut off, then you may use a USB mini cable with a maximum supported charge current of 2A.
 * [USB mini cable (3-pack)](https://www.amazon.com/Premium-Super-Durable-Android-Smartphones-Tablets/dp/B076WXJPQ1/ref=sr_1_4?keywords=usb+micro&qid=1585767182&sr=8-4)
 



<a name="hardware-config"></a>
___
## Configuration
The Panda serves as a bridge between the vehicle's CAN bus and USB.  The Grey Panda uses a standard OBD II connector whereas the Black Panda interfaces using special connectors. The OBD port is a standardized connector guaranteed to be on any car newer than 1996 for emmissions pruproses.  This means that the Grey/White Panda can connect to any car, however some caveats to this have been noted:

* On a 2009 Ford Hybrid Escape,  CAN data can be read but the Panda causes an electrical issue resulting in disabling the dash, locks, and creature comforts.
* On a 2003 Jeep Grand Cherokee, the car runs normally but no CAN data can be read.  Also, the Panda USB interface connection fails on car starting.

With this in mind, be cautious on the car that you wish to plug the Panda into, as the connector may be standardized but the pinout varies.  It is safest to only collect data on cars supported by comma.ai.

#### Grey Panda with Giraffe
The OBD II port is intended for emmissions and mechanic diagnostics.  In order to read lower-level data, the Giraffe exposes deeped CAN buses and adapts them to an OBD II port.  This enables reading of a vehicle's Radar data.  The Giraffe also feature DIP switches for setting different modes, including enabling writing to the CAN bus for vehicel control.  Currently we have the DIP switches are configures as follows:

![Giraffe DIP switch configuration: ON:(1,3,4) OFF:(2)](/doc/images/dip.jpg "DIP Switches")

#### Black Panda with Car Harness

The Black Panda does not feature any DIP switches, but the associated Car Harness, if matched to the vehicle, should be all that is needed.  There is a caveat to the installation of the black panda when not using comma.ai's EON.  First let's discuss the power methodology regarding the Black Panda/Car Harness/EON intended use case:

* The EON is a rooted phone, featuring a single USB connector that is intended for normal charging.  This charging is usually power provided by the host computer to the client USB device.
* The EON features USB OTG functionality, meaning that the phone can act a USB host.  The phone however still needs to charge.
* The Black Panda is a USB client device, but has been engineered to provide back-power to the EON to power the host by getting power through the Car Harness.
* This project, using libpanda, is intended to run on a Raspberry Pi which provides power to the client device.
* Thus there is a conflict where both the Raspberry Pi (or other computer) is trying to drive the USB power pins along with the Black Panda.

The above items were found out by assuming that the USB connectors followed standard protocols but this is **very importantly not the case**.  **Thus to make use of the Black Panda, modification or special parts must be ordered to not burn out any components**.

Fortunately htere are a few workarounds that will let you safely make use of the Black Panda.  

#### Purchasing an Adapter:
The least invasive method is to purchase a cable that has only the data lines connected.  These are however difficult to comeby.  One example of a cable is [this power-switchable cable](https://www.amazon.com/dp/B07T9BRNHW/ref=sspa_dk_detail_5?psc=1&pd_rd_i=B07T9BRNHW&pd_rd_w=L4Rlk&pf_rd_p=48d372c1-f7e1-4b8b-9d02-4bd86f5158c5&pd_rd_wg=TWwR4&pf_rd_r=46V14Z53DTJWNP6WXF3Q&pd_rd_r=5f3b21c8-2c73-4df3-a883-129fbd9c115a&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExTlZMTDJIU1JXVVY4JmVuY3J5cHRlZElkPUEwMDAxMDA3MVQ2QlFZN0lORTVWOCZlbmNyeXB0ZWRBZElkPUEwMzY4MzU0MTBNRDJQQkpWRkdVTSZ3aWRnZXROYW1lPXNwX2RldGFpbCZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=), however please keep in mind that you must ensure that the switch is never accidentally flipped.  Also, this means that a person new tot he Black Panda may be unaware of the cables importance, bypass the cable, and cause hardware damage.

#### Using Scotch Tape on the USB Cable:
A simple solution without the purchase of a separate adapter that can be easily done at home is to tape-off the +5V line in the USB cable.  In the following image, you can see the placement of the tape within the cable.  This end may be plugged into either the Black Panda or the Raspberry Pi.  This method is non-invasive and easily reversible.  To read more about the pinouts with USB type A connectors, refer to [this website](https://www.electroschematics.com/usb-how-things-work/?utm_referrer=https%3A%2F%2Fwww.google.com).  This method also has the same issue as using the adapter method above regarding having knowledge about the cable, but even moreso since the cable change with tape is subtle.  A person could potentially swap out cables without realizing the tape is on the old cable, causing hardware damage.

![Tape on the +5V pin on one end of a USB cable for the Black Panda](/doc/images/tapedUSB.png "USB taped off")

#### Black Panda Modification:
As with the scotch-tape method above, this method involves disconencting power but through a more invasive method.  Again, [this website](https://www.electroschematics.com/usb-how-things-work/?utm_referrer=https%3A%2F%2Fwww.google.com) should be read regarding pinouts to understand what this modification does.  This method involves the disassembly of the Black Panda to sever the +5V line directly on the USB connector.  Fortunately this can be done in a number of ways, using some snipping tool or a soldering iron, and disassembly is a breeze.  I personally prefer this method to the above two since this will ensure that the Black Panda will never be able to fry a computer.  Also depending on the method of modification, this may be reversible using a soldering iron.  The following image shows the steps to modify the Black Panda:

![Black Panda Power Modification](/doc/images/modification.png "Black Panda Power Modification")

1. Peel back the tape until the seam is exposed.
2. Slide the two halves apart as shown.  Be careful of the GPS antenna cable.
3. Use a USB cable or other blunt object to aid in sliding the PCB out of the casing.
4. Remove the USB cable after the board slides a bit
5. Slide the PCB out of the case
6. Locate the USB A +5V power pin.  You may use a tool to sever the connection and bend the pins out of the way.  In the photo, a tool is used to pry the pin out of the USB housing.
7. Furhter prying shown, witht he pin coming out of the connector.
8. The solder joint may break, but the pin is fully removed and the Black Panda, with any cable, is now incapable of breaking a computer.

Reassmbly may be done by following steps 1, 2, and 5 in reverse.


#### Vehicle Specific Information
<a name="hardware-rav4"></a>
### Installation in Toyota RAV4 2019

\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/N2x18QqpV5s?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

<a name="hardware-pilot"></a>
### Installation in Honda Pilot 2017

TBD


<a name="software"></a>
___
# Software

<a name="software-OS"></a>
___

![ ](/doc/images/ubuntu.jpg "Ubuntu")

## Ubuntu 18.04 LTS

Currently Ubuntu 18.04 LTS is the only supported OS though it is very possible that libpanda will work just fine on other *nix based platforms.  To install Ubuntu 18.04 LTS on the Raspberry Pi 4, you will need a method to write to micro SD cards (see [Optional Hardware](#hardware-optional)).  Follow the tutorial on Ubuntu's website for downloding and flashing based on your own OS:

* [Ubuntu 18.04 LTS Raspberry Pi 4 Installation Guide](https://ubuntu.com/download/raspberry-pi)

Once the OS is flashed, insert the SD card into the Pi 4.  Etherrnet is by default enabled with a DHCP client.  There are two ways to configure the Pi 4 from this point, either by plugging in a keyboard and monitor using the HDMI adapter from the [Optional Hardware](#hardware-optional), or through the preinstalled SSH server.  The default credentials for this image are:

* Username: ubuntu
* Password: ubuntu

The default username can be changed, but is out of scope for this tutorial.

### Setup via Monitor and Keyboard (Easy but clunky)

1. Hook up monitor and keyboard
2. Power the Pi via the USB C connector
3. On boot, enter the credentials
4. Follow the instructions to enter a new password

### Setup via Ethernet and SSH (Difficult but with minimal extra hardware)

1. Plug in an ethernet cable 
2. Power the Pi via the USB C connector
3. Wait a few minutes, then get the Pi's IP from your router
4. SSH into the pi.  You may use PuTTY on Windows, or on a *nix system (including macOS):
~~~
$ ssh ubuntu@<your Pi's IP>
~~~
5. Follow instructions to change password
6. SSH in again with new password

Here is an example of logging into a Pi after obtaining an IP address of 10.0.1.80:
\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/w7WJoFv7HlY?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

<a name="libpanda-setup"></a>
___
## libpanda Setup

Libpanda is a multithreaded C++ based library for custom Panda interface applications using the observer based software design pattern (i.e. callbacks).  Also featured in the software area set of premade data recording utilities, simple data visualizers, startup services, and network handling services.  The utilities save CSV formatted data that is supported by Rahul Bhadani's [Strym](https://github.com/jmscslgroup/strym).

Installation requires an internet connection.  Ethernet is the easiest form to provide internet.  Once logged into your Pi 4 (either SSH or keyboard/monitor), try pinging Google's DNS server to check for receivng bytes from 8.8.8.8:
~~~
$ ping 8.8.8.8
64 bytes from 8.8.8.8: icmp_seq=1 ttl=56 time=25.6 ms
64 bytes from 8.8.8.8: icmp_seq=2 ttl=56 time=21.0 ms
~~~

If successful, run the following to download and install libpanda.  This will take a few minutes to complete:
~~~
$ git clone https://github.com/jmscslgroup/libpanda.git
$ cd libpanda
$ sudo ./installpi4.sh 
~~~

\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/A3hyYB-Fy_U?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

> Note: The above video may show different results as development continues.




<a name="software-guide"></a>
___
# Software Guide

<a name="libpanda-services"></a>
___
## libpanda Services

### About

The [libpanda setup process](libpanda-setup) installs a set of startup scripts using linux's systemd.  These scripts 
* pandasettime - This opens the Panda, waits for a GPS fix, then sets the Pi 4's system clock based on GPS time
* pandarecord - This runs after pandasettime.  Once the system clock is set, pandarecord opens the panda for data recording.  This produces a set of CSV formatted files for both CAN and GPS data.  All data is currently recorded into timestamped files and folders, which may be viewable here:
~~~
$ cd /var/panda/CyverseData/JmscslgroupData/PandaData
~~~

> Note: We currently are aware of an issue where when the Pi 4 is soft rebooted (i.e. sudo reboot), the Panda device fails to be recognized.  In the case of a hard boot (i.e. reconnecting Pi 4 power), this problem does not occur.  In the following example, you can see how to check for Oanda USB connectivitiy by performing the command lsusb.  This is the result afeter a soft boot, then the Panda was reconnected, followed by andother lsusb to check for its presence.

![Using lsusb to check for the Panda device, device ID bbaa:ddcc](/doc/images/lsusb.jpg "lsusb")

### Checking Status

Systemd provides a set of utilities to work with services.  To check on the status of one of the above services:
~~~
$ service pandasettime status
$ service pandarecord status
~~~
To exit the status check, press 'q'.
\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/qbSig0QSgKk?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

A similar command for checking status is journalctl.  This may be run as follows:
~~~
$ journalctl -f -u pandasettime
~~~

Press ctrl-c to exit journalctl.


### Fixing Errors

These services should automatically start and take control of the Panda.  To run other utilities, you can manually stop a service, or start it up later.  Restarting a service effectively performs a stop, then a start.  Here are useful commands for this scenario:
~~~
$ sudo service pandasettime stop
$ sudo service pandasettime start
$ sudo service pandasettime restart
$ sudo service pandarecord stop
$ sudo service pandarecord start
$ sudo service pandarecord restart
~~~

This is an example of when the services show a fialed status, and how to reset them:
\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/mohtZnknESk?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

### Disabling Services

If you are in a development environment then it can be annoying to have these services automatically take control fo the Panda.  In this case you can disable and later re-enable services by performing the following:
~~~
$ sudo service pandasettime disable
$ sudo service pandarecord disable
$ sudo service pandasettime enable
$ sudo service pandarecord enable
~~~

<a name="libpanda-utilities"></a>
___
## libpanda Utilities

In order to run the following utilities, ensure that no services are currently running that use the Panda.  For example on a fresh boot with services still enabled, run the following:
~~~
$ sudo service pandasettime stop
$ sudo service pandarecord stop
~~~

The error you may receive if the Panda device is already in use will look similar to the following error:
![Panda device busy by another process resulting ins LIBUSB_ERROR_BUSY ](/doc/images/libusb-busy.jpg "busy")

<a name="libpanda-pandacord"></a>
### pandacord
This program is the same program invoked by the pandarecord systemd service.  In the case of the systemd service, directories and filenames are hardcoded to a pre-defined directory.  Running the service manually lets the user specify filenames.  The following records CSV formatted data to the specified filenames in the current working directory:
~~~
$ sudo pandacord -g gpsData.csv -c canData.csv
~~~

To finish recording press ctrl-c.  More options for file saving are possible by providing the "-h" argument.

~~~
$ sudo pandacord -h
~~~
![pandacord usage information ](/doc/images/pandacord.jpg "busy")

Here is a video showing a successful run of pandacord, however no CAN bus was physically conencted in this demonstration:
\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/DVjhZIUuJeI?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

<a name="libpanda-pandaSetSystemTime"></a>
### pandaSetSystemTime
As similar to pandacord, pandaSetSystemTime is the program invoked by the pandasettime systemd service.  This program opens the Panda, continuously reads the GPS and waits for a fix, compares the system clock against GPS time, then resets the system clock if beyond an epsilon of 0.2 seconds.

~~~
$ sudo pandaSetSystemTime
~~~
\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/L0e9RAOfZ9U?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

> Note: This video is showing an older method of invoking pandaSetSytemTime but provides the same output.

<a name="libpanda-pandaCurses"></a>
### pandaCurses
pandaCurses is an example program that uses ncurses for viewing data-in-real-time (DIRT).  Both GPS and CAN data are handled in different views.  For GPS, all reported statistics, as configured by the the ublox m8030 are displayed.  The CAN visualization is similar to eh *nix program "top", allowing for sorting of different metrics of live data.  This program is not currently installed and needs run in a different manner.  Data logging is also provided if the correct arguments are provided (see usage statement).

~~~
$ cd libpanda/build
$ ./pandaCurses -h # This displays the usage statement
$ sudo ./pandaCurses
~~~

When run, pressing the 'g' key siwtches between view modes for GPS and CAN data.

\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/qKrr3wVKoyA?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

> Note: In the above video example the argument "-f" was provided.  This fakes CAN data for testing purposes and should not be used.  This was used purely for demonstration.

##### pandaCurses GPS display
![pandaCurses GPS view](/doc/images/gpscurses.jpg "GPS View")

The GPS provides a simple display of the state reported by the GPS module after parsing NMEA messages.  This may be used to assess the quality of antenna placement based on the current set of viewable satellites.


##### pandaCurses CAN display
![pandaCurses CAN view](/doc/images/cancurses.jpg "CAN View")

This represents a similar structure to the program "top".  When in this view, the arrow keys can select a column and change the ordering (high-low or low-high) of the current column.

* MessageID represents the CAN message parsed ID. 
* Count represents the number of times a message with MessageID has been sent
* Rate is the current rate of messages sent with MessageID
* UniqueData represents the number of unique data sent with MessageID (see video below).  Pressing '-' resets the count to 0.

When a particular column is selected, the "Highlight Value:" fiel in the lower left lets the user type in a number for particualr data to be highlighted.  Once a number is typed, press the "enter" key to perform the highlight.  For MessageID, Count, and UniqueData, all current values matching "Highlight Value" will be immediately highlight, and persist even if the value changes.  For Rate, this highlights anything withing +/-1 from "Highlight Value".

UniqueData is intended to be used for reverse-engineering of vehicle ocmponents.  In the case of constantly changing data such as floating point values for engine status reports, UniqueData may not be an effective method.  For messages that involve reporting discrete states, UniqueData can easily highlight messages that changed absed on interacting with the vehicle.  The effective sequence of this interaction is as follows:

1. Reset UniqueData with '-'
2. Select the UniqueData column
3. Wait until an acceptable steady state is reached
4. Enter a value into the "Highlight Field", such as "1", then press enter to highlight data
5. Perform a simple interaction on the car, like pushing a single button, shifting gears, or pressing the brake
6. Observe values highlighted that no longer have UniqueData at 1, suggesting new data has been published as a result of the vehicle interraction.


\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/kJtX92a0EbM?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly


<a name="power-guide"></a>
___
# Power Guide with x725

> Note: Do not follow the installation instructions provided by the manufacturer.  We have provided an installation script that functions for our use case.

![x725 installed on the Raspberry Pi](/doc/images/x725.jpg "x725")

The x725 is a battery backup solution for the Raspberry Pi, using GPIO pins 4, 17, and 18 along with i2c.  The documentation of the particular hardware is a bit lacking, but the functionality is sufficient for data collection applications.  Some preliminary data was taken in regard to charge/discharge times under expected use:

* With the Panda plugged in while the OS is idle, power can be supplied for ~4 hours
* With the Panda plugged in actively recording data at ~30% CPU usage, charge time is ~2.5 hours

For use with the CIRCLES project, the installation script in libpanda->scripts->x725 sets up the necessary start scripts to automatically shut down the Pi in a safe manner.  The statemachine for the boot process is shown in the following state diagram. The x725 allows the state machine to detect a power outage, then invoke the necessary scripts for wifi connectivity and cyverse data uploading, then finally invoke a safe shutdown when all data has been uploaded.

![Statemachine of data recording, network connectivity, and data upload](/doc/images/statemachine.png "High-Level State Machine of libpanda, Wifi, and Power Management")

The x725 may operate in certain modes based on the installation of certain jumpers per the x725's user manual.  The scripts provided by the libpanda project are based on the jumper settings in the following image.  This ensures that the x725 will apply power to the Raspberry Pi when power is re-established, e.g. when starting a car.  This jumper is labelled on the x725 PCB as "Auto SD".  The "Auto On" feature appears to leave the Raspberry Pi powered after a shutdown event, which is why a jumper should not be installed in this location.  The "Lan PWR" jumper allows for power to be applied to the ethernet port for WOL funcitonality which is not needed for this porject.  Also, the x725 ships with a small USB cable for a built-in USB-Ethernet adapter.  Again, since WOL is not needed for the CIRCLES project, this cable does not need ot be installed.

![x725 jumper settings](/doc/images/x725jumper.png "x725 Jumper Settings for the scripts in libpanda")

To install the x725 software for the above state machine, navigate to the installation directory for the x725.  Then, invoke the installation script.

~~~
$ cd libpanda/scripts/x725
$ sudo ./install.sh
~~~

This installation process involves the compilation of two C programs, installs a script to invoke a system shutdown, and installs two systemd services.

* x725shutdown.sh is installed as /usr/local/sbin/x725shutdown.  When called, this operates the GPIO pins to invoke a shutdown.
* x725button.service is a systemd script that invokes the C program, x725button.  This listens to the GPIO for an invoked shutdown command, then calls a "poweroff".
*  x725power.service is a systemd script that invokes the C program, x725power.   This monitors the charging/discharging state. If power disconnect event occurs, user-defined script is invoked, followed by a call to "x725shutdown".

To manually invoke a system shutdown:

~~~
$ sudo x725shutdown
~~~

Invoking the shutdown works by signaling to the x725 that a shutdown is taking place, at which the x725 signals back to the Pi via GPIO that a shutdown should take place.  The x725button service listens for this and invokes the shutdown automatically.  To check on the systemd button status:

~~~
$ systemctl status x725button
~~~

The x725 script is automatically invoked based on the power conditions as measured by the x725power service.  These conditions include a power input interrupt, low battery voltage, or low battery capacity.  The overall flow of signals is therefore:

x725power -> x725shutdown -> x725button

Again, the status may be checked with systemctl:

~~~
$ systemctl status x725power
~~~

If you desire to not have the system automatically shutdown upon, such as for when you are debugging the car during starting and stopping the engine, then disable the x725power script:

~~~
$ sudo systemctl stop x725power
$ sudo systemctl disable x725power
~~~

To return to normal operation:

~~~
$ sudo systemctl restart x725power
$ sudo systemctl enable x725power
~~~


<a name="network-guide"></a>
___
# Network Guide

The directory libpanda/scripts/pi4wifi provides a set of installation scripts for the automatic connection of WiFi for future automatic data uploading.  When a known wifi Access Point (AP) is not available, then the scripts with generate an AP for easy connection with a phone or laptop for Raspberry Pi debugging.  A very high level view of these scripts in the form of a state machine is as follows:

![State machine of the wifi handler](/doc/images/wifistate.png "High-Level state machine of crazypifi")

The pi4wifi directory has an installation script for easy setup.  Simply run this script for installation of dependencies, along with the removal of confliciting packages.  The main script is called "crazypifi.sh" and is dependent on hostapd, isc-dhcp-server, wpa_supplicant, iw, and ifupdown.  On Ubuntu 18.04, netplan is the default network interface manager but needs to be removed for the use of ifupdown.  Please make a note of this if you need netplan for other applications.  Again, running the install.sh script will handle all of installation needed.

During installation you will be asked to enter a WiFi SSID and passphrase.  It is not checked upon entering, so if you have connection issues you may need to correct your passphrase later.  The passphrase and SSID are stored in a wpa_supplicant configuration file, specifically you may edit known wifi SSIDs and passphrases using nano as follows:

~~~
$ sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
~~~

The installation script also installs crazypifi.sh as a service, called crazypifi.  As with the prior service scripts covered above, you may use systemctl to check the status, stop/start/restart the service, or enable/disable on boot:

~~~
$ systemctl status crazypifi
$ sudo systemctl stop crazypifi
$ sudo systemctl start crazypifi
$ sudo systemctl restart crazypifi
$ sudo systemctl disable crazypifi
$ sudo systemctl enable crazypifi
~~~


<a name="visualization"></a>
___
# Data Visualization

Data visualization is currently handled by [Strym](https://github.com/jmscslgroup/strym).

A tutorial for Strym can be found [here](https://github.com/jmscslgroup/strym/blob/master/notebook/strymread_example.ipynb).

\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/w2p1uYmHBPA?rel=0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

<a name="repository"></a>
___
# Data Repository

Currently all data is intended to be uploaded to [CyVerse](https://cyverse.org).  In order to acess data we need to grant you access in an account-by-account basis.  Please request access from someone at the JMS CSL Group.  Currently data is manually uploaded to CyVerse directly from the Pi 4, but with manual commands at the end of each colleciton session.  Eventually we will provide methods to perform uploads automatically.  This is also why the power supply has yet to be determined for the Pi 4, since power needs to remain after the car is shut off to give time for the Pi to connect to wifi and transfer data.

To access Cyverse data after you have created an account and have been granted access:

1. Log into CyVerse, then open the discovery environment.
![CyVerse Discovery Environment](/doc/images/cyverse.jpg "Cyverse")
2. Click on the "Data" square in the top left in the environment.
3. Navigate to Shared With Me->rahulbhadani->Jmscslgroup->PandaData
![Cyverse Discovery Environment data navigation](/doc/images/cyversedata.jpg "Discovery Environemnt Data")
4. All data is timestamped in folders.  You can download data by clicking on the Download dorp down menu at the top.  Alternatively, the discovery environment lets you preview data once selected.

> Note: Data between different days may show to have inconsistent formatting due to the development process of data collection.  Also, some data may be available along with a video of the collection session that may be used for data evaluation.
