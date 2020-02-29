/*
 Author: Matt Bunting
 Copyright (c) 2020 Arizona Board of Regents
 All rights reserved.

 Permission is hereby granted, without written agreement and without
 license or royalty fees, to use, copy, modify, and distribute this
 software and its documentation for any purpose, provided that the
 above copyright notice and the following two paragraphs appear in
 all copies of this software.

 IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
 FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
 ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
 IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 SUCH DAMAGE.

 THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
 IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
 TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 */

#include <iostream>
#include <signal.h>
#include <unistd.h>	// usleep()
#include <time.h>

#include "panda.h"

// A ctrl-c handler for safe panda handler destruction
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}


class SimpleGpsObserver : public Panda::GpsListener {
private:
	int notificationCount = 0;
	void newDataNotification( Panda::GpsData* gpsData ) {
		notificationCount++;
		if(notificationCount > 10) {
			std::cerr << "*";
			notificationCount = 0;
		}
	}
};



int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;

	//Set up graceful exit
	signal(SIGINT, killPanda);

	SimpleGpsObserver mSimpleGpsObserver;

	// Construction:
	Panda::Usb mUsb;
	Panda::Gps mGps;
	mGps.setUsb(&mUsb);
	mUsb.addObserver(&mGps);

	mGps.addObserver(&mSimpleGpsObserver);

	std::cout << "Saving NMEA to nmea.txt" << std::endl;
	mGps.saveToFile("nmea.txt");

	// Initialization:
	mUsb.initialize();
	mGps.initialize();
	mUsb.startRecording();
	mGps.startParsing();

	std::cout << "Each \'.\' represents 100 NMEA messages received.  " << std::endl;
	std::cout << "Each \'*\' represents 10 notifications received.  " << std::endl;
	int lastNmeaMessageCount = 0;
	while (keepRunning == true) {
		if (mGps.getData().successfulParseCount-lastNmeaMessageCount > 100) {
			std::cerr << ".";
			lastNmeaMessageCount = mGps.getData().successfulParseCount;
		}
		usleep(10000);
	}
	std::cout << std::endl;

	std::cout << "Stopping GPS...";
	mGps.stopParsing();
	std::cout << "Done." << std::endl;
	std::cout << "Stopping USB...";
	mUsb.stopRecording();
	std::cout << "Done." << std::endl;

	return 0;
}
