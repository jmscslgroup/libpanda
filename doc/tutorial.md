Hardware tutorial                         {#mainpage}
============

[Hardware](#hardware)

# Hardware<a name="hardware"></a>

## List of components
The hardware choice is based around running linux with comm.ai hardware, leveraging prior efforts in CAN bus access.  

![Hardware Overview](/doc/images/overview.png "Overview")

#### Required:
* [comma.ai Grey Panda](https://comma.ai/shop/products/panda-obd-ii-dongle)

	Currently only the Grey Panda is supported in libpandac.
	
* [comma.ai Giraffe](https://comma.ai/shop/products/giraffe)

	There are many flavors, be sure to select the Giraffe for your particular vehicle.
	
* [Raspberry Pi 4GB](https://www.amazon.com/Raspberry-Model-2019-Quad-Bluetooth/dp/B07TD42S27/ref=sxin_2_ac_d_pm?ac_md=4-0-VW5kZXIgJDUw-ac_d_pm&cv_ct_cx=raspberry+pi+4&keywords=raspberry+pi+4&pd_rd_i=B07TD42S27&pd_rd_r=897fc6f0-a9d6-430f-8811-c07c3c7b9e19&pd_rd_w=K0xrG&pd_rd_wg=WSCdw&pf_rd_p=0e223c60-bcf8-4663-98f3-da892fbd4372&pf_rd_r=AGXA47R72X2ZKA2F8X3Z&psc=1&qid=1583948920)
* [64GB SD card](https://www.amazon.com/Samsung-Class-Adapter-MB-MC64DA-AM/dp/B01273JZMG?tag=androidcentralb-20&ascsubtag=UUacUdUnU77910YYwYg)
*  [USB 2.0/3.0 to USB C adapter cable for power](https://www.amazon.com/Anker-Powerline-Pull-up-Resistor-Samsung/dp/B01A6F3WHG/ref=sr_1_5?keywords=usb+3+to+usb+C&qid=1583954394&s=electronics&sr=1-5)
* Power supply (TBD)

	We have yet to determine the best method of providing power to the pi 4.  They consume quite a bit, you may need a 2.0A USB outlet to prevent brownout reboots.

#### Optional:<a name="hardware-optional"></a>
* [SD Card reader for Ubuntu flashing](https://www.amazon.com/Vanja-Adapter-Portable-Memory-Reader/dp/B00W02VHM6/ref=sr_1_6?keywords=usb+sd+card&qid=1583949114&s=electronics&sr=1-6)
* [Micro HDMI adapter for pi 4](https://www.amazon.com/GANA-Adapter-Female-Action-Supported/dp/B07K21HSQX/ref=sxin_2_ac_d_pm?ac_md=1-0-VW5kZXIgJDEw-ac_d_pm&cv_ct_cx=micro+hdmi&keywords=micro+hdmi&pd_rd_i=B07K21HSQX&pd_rd_r=b124f42c-a587-491e-9400-52aef81c3d88&pd_rd_w=mNPkI&pd_rd_wg=saxZx&pf_rd_p=0e223c60-bcf8-4663-98f3-da892fbd4372&pf_rd_r=J1YKVHPV73CBGDNP1MXZ&psc=1&qid=1583949163&s=electronics)
* [Ethernet Cable](https://www.amazon.com/AmazonBasics-RJ45-Cat-6-Ethernet-Patch-Cable-10-Feet-3-Meters/dp/B00N2VIALK/ref=sr_1_3?keywords=ethernet&qid=1583954176&sr=8-3)


## Configuration
The Panda serves as a bridge between the vehicle's CAN bus and USB.  The Panda uses a standard OBD II connector. The OBD port is a standardized connector guaranteed to be on any car newer than 1996 for emmissions pruproses.  This means that the Panda can connect to any car, however some caveats to this have been noted:

* On a 2009 Ford Hybrid Escape,  CAN data can be read but the Panda causes an electrical issue resutling in disabling the dash, locks, and creature comforts.
* On a 2003 Jeep Grand Cherokee, the car runs normally but no CAN data can be read.  Also, the Panda USB interface connection fails on car starting.

With this in mind, be cautious on the car that you wish to plug the Panda into, as the connector may be standardized but the pinout varies.  It is safest to only collect data on cars supported by comma.ai.

The OBD II port is intended for emmissions and mechanic diagnostics.  In order to read lower-level data, the Giraffe exposes deeped CAN buses and adapts them to an OBD II port.  This enables reading of a vehicle's Radar data.  The Giraffe also feature DIP switches for setting different modes, including enabling writing to the CAN bus for vehicel control.  Currently we have the DIP switches are configures as follows:

![Giraffe DIP switch configuration: ON:(1,3,4) OFF:(2)](/doc/images/dip.jpg "DIP Switches")

### Installation in Toyota RAV4 2020

Video

### Installation in Honda Pilot 2017

TBD



# Software

## Ubuntu 18.04 LTS

Currently Ubuntu 18.04 LTS is the only supported OS though it is very possible that libpandac will work just fine on other *nix based platforms.  To install Ubuntu 18.04 LTS on the raspberry pi 4, you will need a method to write to micro SD cards (see [Optional Hardware](#hardware-optional)).  Follow the tutorial on Ubuntu's website for downloding and flashing based on your own OS:

* [Ubuntu 18.04 LTS Raspberry Pi 4 Installation Guide](https://ubuntu.com/download/raspberry-pi)

Once the OS is flashed, insert the SD card into the pi 4.  Etherrnet is by default enabled with a DHCP client.  There are two ways to configure the pi 4 from this point, either by plugging in a keyboard and monitor using the HDMI adapter from the [Optional Hardware](#hardware-optional), or through the preinstalled SSH server.  The default credentials for this image are:

* Username: ubuntu
* Password: ubuntu

The default username can be changed, but is out of scope for this tutorial.

### Setup via Monitor and Keyboard (Easy but clunky)

1. Hook up monitor and keyboard
2. Power the pi via the USB C connector
3. On boot, enter the credentials
4. Follow the instructions to enter a new password

### Setup via Ethernet and SSH (Difficult but with minimal extra hardware

1. Plug in an ethernet cable 
2. Power the pi via the USB C connector
3. Wait a few minutes, then get the pi's IP from your router
4. SSH into the pi.  You may use PuTTY on Windows, or on a *nix system (including macOS):
~~~
$ ssh ubuntu@<your pi's IP>
~~~
5. Follow instructions to change password
6. SSH in again with new password

Here is an example of logging into a pi after obtaining an IP address of 10.0.1.80:
\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/w7WJoFv7HlY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

## libpandac Setup

Libpandac is a multithreaded C++ based library for custom Panda interface applications using the observer based software design pattern (i.e. callbacks).  Also featured int he software area set of premade data recording utilities, simple data visualizers, startup services, and network handling services.  The utilities save CSV formatted data that is supported by Rahul Bhadani's [Strym](https://github.com/jmscslgroup/strym).

Installation requires an internet connection.  Ethernet is the easiest form to provide internet.  Once logged into your pi 4 (either SSH or keyboard/monitor), try pinging Google's DNS server to check for receivng bytes from 8.8.8.8:
~~~
$ ping 8.8.8.8
64 bytes from 8.8.8.8: icmp_seq=1 ttl=56 time=25.6 ms
64 bytes from 8.8.8.8: icmp_seq=2 ttl=56 time=21.0 ms
~~~

If successful, run the following to downlaod and install libpanda.  This will take a few minutes to complete:
~~~
$ git clone https://github.com/jmscslgroup/libpanda.git
$ cd libpanda
$ sudo ./installpi4.sh 
~~~

\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/A3hyYB-Fy_U" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

Note: The above video may show different results as development continues.


## libpandac Utilities




Example:

\htmlonly
<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/L0e9RAOfZ9U" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>
\endhtmlonly

