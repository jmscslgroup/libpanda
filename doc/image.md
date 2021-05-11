Raspberry Pi Image                         {#mainpage}
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
* [Power Guide with x728](#power-guide)
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
# Steps

<a name="Steps"></a>
___

The following steps are not the only way to run everything, but are the exact steps used to set up the distributed image with ROS/libpanda preinstalled.

## Raspbian Image

1. Download the [Raspberry Pi Image Flashing Tool](https://www.raspberrypi.org/software/)
2. Select the appropriate SD card
3. Select Operating System -> Raspberry Pi OS (Other) -> Raspberry Pi OS Lite (32-bit)
4. Write to SD card witht e "Write" button


## Raspbian Setup

1. Insert SD card into Raspberry Pi 4 and boot ( username: pi  password: raspberry )
2. Run the configuration utility
```bash
$ sudo raspi-config
```
3. Enter Localisation Options -> WLAN Country -> Select US
4. Enter Localisation Options -> Keyboard -> Generic 105-Key PC (intl.) -> Other -> English (US) -> English (US). Then press enter a few times until back at the main menu
5. Select Finish, and reboot

## Account Setup
These steps are not necessary but provide a way to let users know that this raspberry pi is built with the CIRCLES project in mind

1. Log in under the default account ( username: pi  password: raspberry )
2. Change the root password to "circles" then logout by running the following.
```bash
$ sudo passwd root
$ exit
```
3. Log out, then log back in under account root ( username: root  password: circles )
4. Run the following to change the "pi" account to "circles"
```bash
$ usermod -l circles pi
$ usermod -m -d /home/circles circles
$ exit
```
5. Log back in under the renamed circles account  ( username: circles  password: raspberry )
6. Change the password of the circles account to be "circles"
```bash
$ sudo passwd circles
```

## Installation of libpanda

Ensure the Pi is connected to internet, preferably with an ethernet cable.

Run the following commands:
```bash
$ sudo apt update && sudo apt install git -y
$ git clone httpd://github.com/jmscslgroup/libpanda.git
$ cd libpanda
$ sudo ./install.sh
```

## Installation of ROS

Ensure the Pi is connected to internet, preferably with an ethernet cable.

Run the following commands:
```bash
$ cd ~/libpanda/scripts
$ ./installROS.sh
```
