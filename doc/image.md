CIRCLES Raspberry Pi Images
============



# Table of Contents
* [About](#about)
* [Change Log](#changelog)
* [Usage and Expectation](#usage)
* [Setup Steps](#steps)
	* [Raspbian Image](#raspbian-image)
	* [Raspbian Setup](#raspbian-setup)
	* [Account Setup](#account-setup)
	* [Installation of libpanda](#installation-libpanda)
	* [Installing irods-icommands](#installing-irods)
	* [Image Resizing](#image-resize)


<a name="about"></a>
___
# About
The latest images can be downloaded from here:

[CIRCLES Raspberry Pi Images](https://drive.google.com/drive/folders/1elnHg37kVFJXYLIB2_yXWHSoSZLLiZKJ?usp=sharing)

An SD card image is distributed for purposes of consistently and deployment convenience.  The intend of this image is to perform vehicle data collection or ROS-based acceleration control commands.  To learn more about libpanda, see the following:

[https://github.com/jmscslgroup/libpanda](https://github.com/jmscslgroup/libpanda)

[https://github.com/jmscslgroup/can_to_ros](https://github.com/jmscslgroup/can_to_ros)

<a name="changelog"></a>
___
# Change Log

- [circles-raspbian-control-2021-07-12.img.tar.gz](https://drive.google.com/file/d/1Nnuk8sEyjMaRw7azkzGogAXUjbXUGxBF/view?usp=sharing)

	This is a 64-bit image that has ROS-based vehicle_control running on boot instead of the pandarecord service

- [circles-raspbian-64-2021-06-02.img.tar.gz](https://drive.google.com/file/d/1krjo8meTaXulDfu1BeywtvY-fKL2Fr3H/view?usp=sharing)

	Transition to 64-bit Rasbpian for easier irods support, but gives warnings about libssl1.0.0.  Also includes fixes to libpanda for recording radar data in record-only mode.

- [circles-raspbian-2021-05-21.img.tar.gz](https://drive.google.com/file/d/1u53-8XlpFQcdZwb8dSwQdVgkzwnplou2/view?usp=sharing)

	This introduces fixed to libpanda where cruise control would malfunction in data-recording scenarios.
	
- [circles-raspbian-2021-05-11.img.tar.gz](https://drive.google.com/file/d/1whDua53SyYLUnU6sM6c9JlXF1aUeAmPP/view?usp=sharing)

	This image is the initial upload, featuring ROS, Libpanda, and can_to_ros

<a name="usage"></a>
___
# Usage and Expectations

Download the image and flash it to your SD card using any steps you are comfortable with.  If you have never done this before, then you can try the [Raspberry Pi Image Flashing Tool](https://www.raspberrypi.org/software/) and select a custom image.

After downloading one of the provided images ensure that the image ends in either .tar.gz or .img: on some operating systems like macOS the .tar.gz will become automatically unzipped into a .tar but this will not flash correctly.  Instead of flashing .tar, extract (like any other archive) to ge tthe resulting .img file for proper flashing.

>Note: If you are flashing only one SD card then you can select the .img.gz file directly, however this may be a bit slower.  If you play on making many SD cards then extract the .img.gz to a pure .img to potentailly speedier flashing.


## First Boot
On first boot the image will automatically resize itself to fill the free space on the SD card.  This will cause a reboot.

When connected to a supported vehicle with panda hardware, the Pi will read the VIN and immediately perform another reboot to set the hostname and WiFi AP name.

## List of Running Packages
libpanda features a set of packages that run as systemd services.

- pandarecord: Runs a low-level CAN/GPS data recoder
- crazypifi: Manages network, automatically generating WiFi AP if no known network exists
- circlesmanager: Helps glue other services together
- circles-ui: Runs a webserver for displaying system state and for cahnging settings
- blinkt: Converts state produced by other scripts for the blinkt module to display
- x725power: Reads the current power state
- x725button: Reads the ups button for invoking software shutdowns

## Credentials and Naming

By default, before connecting the Pi to a car:
- Hostname: circles
- Username: circles
- Password: circles
- WiFi AP ssid: circles
- WiFi AP passkey: circles0   <- needs to be either 6 or 8 characters unfortunately

Once the Pi boots to a connected car and correctly reads the VIN, it will reboot with the new credentials:
- Hostname: The vehicle's VIN
- Username: circles
- Password: circles
- WiFi AP Name: The vehicle's VIN
- WiFi AP psk: circles0   

The above VIN-based naming conventions will persist and will only change if connected to a vehicle with a different VIN.

## SSH

An SSH server is enabled.  Access your pi with the following, assuming default hostname
```bash
$ ssh circles@circles.local
$ ssh circles@<IP>
$ ssh circles@<VIN>.local
```

## The Service pandarecord Is Enabled

Every time the Pi boots it will attempt to open the panda device and begin recording using low-level libpanda utilities.  It is not possible to run any other libpanda utilities when this service is running since the panda device will be claimed.  If you desire to run ROS-based control commands, you need to stop the service:

```bash
$ sudo systemctl stop pandarecord
```
Stopping the service will not prevent it from starting again on boot.  If going out for field experiments involving control or dedicating the Pi to something other than data collection, then I recommend disabling the service to stop it from running on boot.  Disabling does not stop the service in the current boot, so also stop the service if needed:
```bash
$ sudo systemctl disable pandarecord
$ sudo systemctl stop pandarecord
```

<!---
## irods Setup

irods is installed on the image, however you will need to add your CyVerse login credentials.  The following steps have been 

1. Log into your pi (see SSH above or connect monitor/keyboard)
2. run `iinit` and enter your user/pass as well as the necessary info for cyverse
```bash
$ iinit 
```
   The parameters you should use are from here: https://learning.cyverse.org/projects/data_store_guide/en/latest/step2.html
```
 hostname: data.cyverse.org
 port #: 1247
 username: [.cyverse username.]
 zone: iplant
 password: [.cyverse password.]
```
3. Try the put command on some test data to see if it works. The below one will copy any folders in your pi's recorded data onto cyverse in Rahul's shared folder.
```bash
$ iput -r -v /var/panda/CyverseData/JmscslgroupData/PandaData/* /iplant/home/rahulbhadani/JmscslgroupData/PandaData
```
-->


<a name="Steps"></a>
___
# Setup Steps

>Note: If you downloaded one of the supplied images then DO NOT follow these steps, they have already been done.

The steps listed here are what have been used to build the preconfigured image.  They do not need to be run when using the image.  The purpose of noting these steps is to inform others on exactly how it has been configured if there are issues or if others need to build a slightly modified image.  The following steps are not the only way to run everything, but are the exact steps used to set up this distributed image with ROS/libpanda preinstalled.

<a name="raspbian-image"></a>
___
## Raspbian Image

1. Download the [Raspberry Pi Image Flashing Tool](https://www.raspberrypi.org/software/)
2. Select the appropriate SD card
3. Select OS
	a. 32-bit: Select Operating System -> Raspberry Pi OS (Other) -> Raspberry Pi OS Lite (32-bit)
	b. 64-bit: Download the [64-bit Lite version](https://downloads.raspberrypi.org/raspios_lite_arm64/images/raspios_lite_arm64-2021-05-28/) the Select Operating System -> Use Custom
4. Write to SD card with the "Write" button


<a name="raspbian-setup"></a>
___
## Raspbian Setup

1. Insert SD card into Raspberry Pi 4 and boot ( username: pi  password: raspberry )
2. Run the configuration utility
```bash
$ sudo raspi-config
```
3. Enter Localisation Options -> WLAN Country -> Select US
4. Enter Localisation Options -> Keyboard -> Generic 105-Key PC (intl.) -> Other -> English (US) -> English (US). Then press enter a few times until back at the main menu
4. Enter Interface Options -> SSH -> Yes
5. Select Finish, and reboot


<a name="account-setup"></a>
___
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

<a name="installation-libpanda"></a>
___
## Installation of libpanda

Ensure the Pi is connected to internet, preferably with an ethernet cable.

Run the following commands:
```bash
$ sudo apt update && sudo apt upgrade && sudo apt install git -y
$ git clone https://github.com/jmscslgroup/libpanda.git
$ cd libpanda
$ sudo ./install.sh
```

Running the above will invoke an automatic reboot and rename the hostname from "raspberrypi" to "circles"

At this stage you will notice a trend that everything in this Pi is now "circles", including:
1. Hostname
1. Account
1. Password

The above ALSO now installs ROS and can_to_ros

<!--
<a name="installation-ros"></a>
___
## Installation of ROS

Log back into the pi ( username: circles  password: circles )

Run the following commands:
```bash
$ cd ~/libpanda/scripts
$ ./installROS.sh
```

Now go make some coffee, this will take a while (yup, I wrote down these notes, made coffee, and it is still going)

<a name="installation-can-to-ros"></a>
___
## Installation of can_to_ros

Run the following commands:
```bash
$ source ~/.bashrc
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/jmscslgroup/can_to_ros.git
$ cd ~/catkin_ws
$ catkin_make
```
-->


<a name="installing-irods"></a>
___
## Installing irods-icommands (for 64-bit only)

These steps follow the irods-icommands-rpi.md file found in /iplant/home/rahulbhadani/JmscslgroupData/PandaDevelopment/ on cyverse.org

0. Fetch the irods-icommands-debs.tgz from /iplant/home/rahulbhadani/JmscslgroupData/PandaDevelopment/ and send it to the pi.  On macOs, the following comamnd works:
```bash
$ scp ~/Downloads/irods-icommands-debs.tgz circles@circles.local:~/
```
1. Extract to your pi
```bash
$ tar xzf irods-icommands-debs.tgz
$ rm irods-icommands-debs.tgz
$ cd irods-icommands-debs
```
2. run the install script in there
```
$ ./install.sh
```
   This, when it completes, will install all the necessary prerequisites, and may require your sudo password.

>Note: At this stage commands like iinit have been omitted since they require cyverse login credentials



<a name="image-resize"></a>
___
## Image Resizing

Prior to distribution the image is resized the smallest size possible to easier uploading and downloading.
1. Safely poweroff the Pi
```bash
$ sudo poweroff
```
2. Remove SD card and insert it into another running Linux instance
3. On this Linux, download [PiShrink](https://github.com/Drewsif/PiShrink)
4. Make an image of the card and run PiShrink
```bash
$ sudo dd bs=4M if=/dev/sdb of=~/circles-raspbian-64-`date +%Y-%m-%d`.img
$ sudo ./pishrink.sh ~/circles-raspbian-64-`date +%Y-%m-%d`.img
$ tar -czvf ~/circles-raspbian-`date +%Y-%m-%d`.img.tar.gz ~/circles-raspbian-64-`date +%Y-%m-%d`.img
```
>Note: Make sure you use the right drive letter in the above command regarding /dev/sd*

