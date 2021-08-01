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

#ifndef PANDA_GPS_TRACKER_H
#define PANDA_GPS_TRACKER_H

#include "panda/gps.h"

namespace Panda {


// This class save GPS data in a hardcoded format to the hardcoded directory of /etc/libpanda.d/latest_gps

class GpsTracker : public Panda::GpsListener {
public:
	
	// Example usage:
	// GpsTracker mGpsTracker();
	// // Initialize Usb, this requires a connected Panda
	// Panda::Handler pandaHandler;
	// pandaHandler.addGpsObserver(mGpsTracker);
	
	// Constructor with time to be set
	GpsTracker();
	
	
private:
	
	void newDataNotification( Panda::GpsData* gpsData );
};

}

#endif
