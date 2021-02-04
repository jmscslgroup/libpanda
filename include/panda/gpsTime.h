/*
 Author: Matt Bunting
 Copyright (c) 2021 Arizona Board of Regents
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

#ifndef PANDA_GPS_TIME_H
#define PANDA_GPS_TIME_H

#include "panda/gps.h"

namespace Panda {


// For debugging time
//std::string timevalToPrettyString(struct timeval& time);

class SetSystemTimeObserver : public Panda::GpsListener {
public:
	
	// Example usage:
	// double epsilon = 0.2;	// If system time is off from GPS time by this amount, update time.
	// SetSystemTimeObserver mSetSystemTimeObserver(epsilon);

	// // Initialize Usb, this requires a conencted Panda
	// Panda::Handler pandaHandler;
	// pandaHandler.addGpsObserver(mSetSystemTimeObserver);
	
	// Constructor with time to be set
	SetSystemTimeObserver(double minimumAcceptableOffsetInSeconds);
	
	// Check this before using system time
	bool hasTimeBeenSet();
	
private:
	bool timeHasBeenSet;
	double epsilon;

	void newDataNotification( Panda::GpsData* gpsData );
};

}

#endif
