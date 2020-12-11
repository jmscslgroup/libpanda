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

#include "panda.h"

// A simple concrete instance of a CAN listener
class SimpleCanObserver : public Panda::CanListener {
private:
	void newDataNotification( Panda::CanFrame* canData ) {
		// Gets called for every incomiming can data with data:
		
		// canData->messageID;					// unsigned int
		// canData->dataLength;					// unsigned char
		// canData->busTime;						// unsinged int
		// canData->bus;							// unsigned char
		// canData->data[CAN_DATA_MAX_LENGTH];	// unsigned char[8]
		// canData->sysTime;						// struct timeval
	}
};

// A simple concrete instance of a GPS listener
/*
class SimpleGpsObserver : public Panda::GpsListener {
private:
	int notificationCount = 0;
	void newDataNotification( Panda::GpsData* gpsData ) {
		notificationCount++;
		if(notificationCount > 10) {
			std::cerr << "g";
			notificationCount = 0;
		}
	}
};
*/


using namespace std;
int main(int argc, char **argv) {

	SimpleCanObserver canObserver;
	//SimpleGpsObserver myGpsObserver;

	// Initialize Usb, this requires a connected Panda
	Panda::Handler pandaHandler;
	
	pandaHandler.addCanObserver(canObserver);
	//pandaHandler.addGpsObserver(myGpsObserver);

	// Let's roll
	pandaHandler.initialize();
	
	while(1);
	
	// Will never reach here
	pandaHandler.stop();

	return 0;
}
