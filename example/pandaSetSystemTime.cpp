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
#include <cmath>	// fabs()
#include <time.h>

#include "panda.h"

// A ctrl-c handler for safe panda handler destruction
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}

// For debugging time
std::string timevalToPrettyString(struct timeval& time) {
	char timeString[64];
	time_t time_tPrint = time.tv_sec;
	struct tm* tmPrint = localtime(&time_tPrint);
	int uSecondIndex = strftime(timeString, sizeof(timeString), "%d/%m/%Y %H:%M:%S", tmPrint);
	snprintf(timeString+uSecondIndex, sizeof(timeString)-uSecondIndex, ".%06d", (int)time.tv_usec);
	return timeString;
}

class SetSystemTimeObserver : public Panda::GpsListener {
public:
	SetSystemTimeObserver(double gmtOffsetInHours, double minimumAcceptableOffsetInSeconds) {
		timeHasBeenSet = false;
		gmtInSeconds = gmtOffsetInHours*60*60;
		epsilon = minimumAcceptableOffsetInSeconds;
	}
	// Check this before using system time
	bool hasTimeBeenSet() {
		return timeHasBeenSet;
	}
private:
	bool timeHasBeenSet;
	int gmtInSeconds;
	double epsilon;

	void newDataNotification( Panda::GpsData* gpsData ) {
		if (timeHasBeenSet || (gpsData->info.status != 'A')) {
			return;
		}

		// Current system time
		struct timeval sysTime;
		gettimeofday(&sysTime, NULL);

		// Current GPS local time based on GMT offset
		time_t gpsTime_t = mktime(&gpsData->time);
		struct timeval gpsTime;
		gpsTime.tv_sec = gpsTime_t + gmtInSeconds;
		gpsTime.tv_usec = (gpsData->timeMilliseconds)*1000;

		// This seems convoluted, but I think it will avoid floating point error math
		int differenceInSeconds      = gpsTime.tv_sec  - sysTime.tv_sec;
		int differenceInMicroseconds = gpsTime.tv_usec - sysTime.tv_usec;
		double totalDifference = (double)differenceInSeconds + ((double)differenceInMicroseconds)/1000000.0;

		std::cout << std::endl << "SetSystemTimeObserver.newDataNotification()" << std::endl;
		std::cout << " - GPS Time:" << timevalToPrettyString(gpsTime) << std::endl;
		std::cout << " - Sys Time:" << timevalToPrettyString(sysTime) << std::endl;
		std::cout << " - Time offset of " << totalDifference << " seconds, epsilon:" << epsilon << std::endl;

		if (fabs(totalDifference) > epsilon) {
			if (settimeofday(&gpsTime, NULL)) {
				std::cerr << "ERROR: Unable to set system time from GPS!" << std::endl;
			} else {
				std::cout << " |-- System time set to GPS time!" << std::endl;
			}
		} else {
			std::cout << " |-- No need to change system time" << std::endl;
		}

		timeHasBeenSet = true;
	}
};



int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;

	//Set up graceful exit
	signal(SIGINT, killPanda);

	int MSToffset = -7; 	// -7 hours.  DST not supported
	double epsilon = 0.2;	// If system time is off from GPS time by this amount, update time.
	SetSystemTimeObserver mSetSystemTimeObserver(MSToffset, epsilon);

	// Initialize Usb, this requires a conencted Panda
	Panda::Handler pandaHandler;
	pandaHandler.addGpsObserver(mSetSystemTimeObserver);

	//pandaHandler.getGps().saveToFile("nmeaDump.txt");

	pandaHandler.initialize();	// starts USB and GPS

	std::cout << "Waiting to acquire satellites..." << std::endl;
	std::cout << " - Each \'.\' represents 100 NMEA messages received:" << std::endl;
	int lastNmeaMessageCount = 0;
	while (keepRunning == true) {
		if (pandaHandler.getGps().getData().successfulParseCount-lastNmeaMessageCount > 100) {
			std::cerr << ".";
			lastNmeaMessageCount = pandaHandler.getGps().getData().successfulParseCount;
		}
		usleep(10000);
		if (mSetSystemTimeObserver.hasTimeBeenSet()) {	// Only run until time has been checked/
			keepRunning = false;
		}
	}
	std::cout << std::endl;
	pandaHandler.stop();

	std::cout << "\rDone." << std::endl;
	return 0;
}
