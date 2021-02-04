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

#include "panda/gpsTime.h"

#include <ctime>
#include <cmath>	//fabs
//#include <unistd.h>
#include <string.h>
#include <iomanip>	// put_time

//#define GPS_VERBOSE

using namespace Panda;

std::string timevalToPrettyString(struct timeval& time) {
	char timeString[64];
	time_t time_tPrint = time.tv_sec;
	struct tm* tmPrint = localtime(&time_tPrint);
	int uSecondIndex = strftime(timeString, sizeof(timeString), "%d/%m/%Y %H:%M:%S", tmPrint);
	snprintf(timeString+uSecondIndex, sizeof(timeString)-uSecondIndex, ".%06d", (int)time.tv_usec);
	return timeString;
}

SetSystemTimeObserver::SetSystemTimeObserver(double minimumAcceptableOffsetInSeconds) {
	timeHasBeenSet = false;
	epsilon = minimumAcceptableOffsetInSeconds;
}
// Check this before using system time
bool SetSystemTimeObserver::hasTimeBeenSet() {
	return timeHasBeenSet;
}

void SetSystemTimeObserver::newDataNotification( Panda::GpsData* gpsData ) {
	if (timeHasBeenSet || (gpsData->info.status != 'A')) {
		return;
	}
	
	// Current system time
	struct timeval sysTime;
	struct timezone sysZone;
	//gettimeofday(&sysTime, &sysZone);
	// per web, need NULL or behavior undefined for tz
	gettimeofday(&sysTime, NULL);
	
	// testing different time method
	std::time_t t = std::time(nullptr);
	std::cout << "UTC:   " << std::put_time(std::gmtime(&t), "%c %Z") << '\n';
	std::cout << "local: " << std::put_time(std::localtime(&t), "%c %Z") << '\n';
	std::time_t gmt = mktime(std::gmtime(&t));
	std::time_t localt = mktime(std::localtime(&t));
	// offset in seconds: will use this rather than taking the
	// tz value from gettimeofday which may not be consistent or supported
	// across different distros or with different packages installed
	time_t offset = gmt-localt;
	
	// Current GPS local time based on GMT offset
	time_t gpsTime_t = mktime(&gpsData->time);
	struct timeval gpsTime;
	
	std::cout << " sysTime=" << sysTime.tv_sec << std::endl;
	std::cout << " sysZone=" << sysZone.tz_minuteswest << std::endl;
	
	// fixing CH-61 by removing undefined behavior of tz (timezone)
	// the offset value is calculated by the minute subtraction from gmt
	gpsTime.tv_sec = gpsTime_t - offset;
	//gpsTime.tv_sec = gpsTime_t - sysZone.tz_minuteswest*60;
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


