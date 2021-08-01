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

#include "panda/gpsTracker.h"

#include <cstdio>
#include <unistd.h>

//#define GPS_VERBOSE

using namespace Panda;


GpsTracker::GpsTracker() {
}


void GpsTracker::newDataNotification( GpsData* gpsData ) {
	
	time_t gpsTime_t = mktime(&gpsData->time);
	
	// Get system time to check against GPS time skew
	struct timeval sysTime;
	gettimeofday(&sysTime, NULL);
	
	FILE* file = fopen( "/etc/libpanda.d/latest_gps", "w+");
//	fwrite( data, 1, strlen(data), file);
	
	// GpsTime,SysTime,Latitude,Longitude,Altitude,Status
	fprintf(file, "%d.%06d,%d.%06d,%0.7f,%0.7f,%0.1f,%c",
			(unsigned int)gpsTime_t,
			(gpsData->timeMilliseconds)*1000,
			(unsigned int)sysTime.tv_sec,
			(int)sysTime.tv_usec,
			gpsData->pose.latitude,
			gpsData->pose.longitude,
			gpsData->pose.altitude,
			gpsData->info.status
			);
	
	fclose(file);
}
