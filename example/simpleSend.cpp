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
#include <unistd.h>
#include <cstring>
#include <cmath>

#include "panda.h"

#include "device.h"

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

class JoystickListener : public DeviceObserver {
private:
	void newDeviceEvent( const DeviceEvent* event ) {	// override from DeviceObserver
		if (event->type == TYPE_AXIS) {
			switch (event->id) {
				case AXIS_RX: axisRX = ((double)event->value)/32768.0; break;
				case 3: axisRY = ((double)event->value)/32768.0; break;
				case AXIS_LX: axisLX = ((double)event->value)/32768.0; break;
				case AXIS_LY: axisLY = ((double)event->value)/32768.0; break;
				default: break;
			}
			//std::cout << "Event id:" << (int)event->id << "\tvalue:" << event->value << std::endl;
		} else if (event->type == TYPE_BUTTON) {
			switch (event->id) {
				case BUTTON_TRIANGLE: buttonTriangle = event->value; break;
				case BUTTON_SQUARE: buttonSquare = event->value; break;
				case BUTTON_CIRCLE: buttonCircle = event->value; break;
				default: break;
			}
		}
	}
	
	double axisRX;
	double axisRY;
	double axisLX;
	double axisLY;
	bool buttonTriangle;
	bool buttonSquare;
	bool buttonCircle;
public:
	JoystickListener()
	:axisRX(0), axisRY(0), axisLX(0), axisLY(0), buttonTriangle(false), buttonSquare(false), buttonCircle(false) {
	}
	double getRX() { return axisRX; }
	double getRY() { return axisRY; }
	double getLX() { return axisLX; }
	double getLY() { return axisLY; }
	bool getTriangle() { return buttonTriangle; }
	bool getSquare() { return buttonSquare; }
	bool getCircle() { return buttonCircle; }
};

//using namespace std;
int main(int argc, char **argv) {
	
	JoystickListener mJoystickState;
	Joystick mJoystick;

	mJoystick.addObserver(&mJoystickState);
	mJoystick.open("/dev/input/js0");
	mJoystick.start();
	
	//SimpleCanObserver canObserver;
	//SimpleGpsObserver myGpsObserver;

	// Initialize Usb, this requires a connected Panda
	Panda::Handler pandaHandler;
	
	Panda::ToyotaHandler toyotaHandler(&pandaHandler);
	
	//pandaHandler.addCanObserver(canObserver);
	//pandaHandler.addGpsObserver(myGpsObserver);

	// Let's roll
	pandaHandler.initialize();
	
	toyotaHandler.start();
	
	while(1) {
		usleep(1000000.0/10.0);	// run at 10 Hz

		double acceleration = 0.0;
		double joystickValue = -mJoystickState.getLY();

		//const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s2
		//const int TOYOTA_MIN_ACCEL = -3000;       // -3.0 m/s2
		// The following limits can be achieved by setting the panda into "unsafe" mode:
		//const int TOYOTA_ISO_MAX_ACCEL = 2000;        // 2.0 m/s2
		//const int TOYOTA_ISO_MIN_ACCEL = -3500;       // -3.5 m/s2
		if (joystickValue > 0) {
			acceleration = 1.5*joystickValue;
		} else if (joystickValue < 0) {
			acceleration = 3.0*joystickValue;
		}

		
		toyotaHandler.setHudLdaAlert( mJoystickState.getTriangle() );
		toyotaHandler.setHudCruiseCancelRequest( mJoystickState.getSquare() );
		
		joystickValue = mJoystickState.getRX();
		//const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
		int steerTorque = 1500 * joystickValue;	// range: 1500
		
		if (!mJoystickState.getCircle()) {	// Holding circle tests the heartbeat (stopping it)
			toyotaHandler.setAcceleration(acceleration);
			toyotaHandler.setSteerTorque(steerTorque);
		}
		
		
		std::cout << "Joystick RX:" << mJoystickState.getRX() <<
					" RY:" << mJoystickState.getRY() <<
					" LX:" << mJoystickState.getLX() <<
					" LY:" << mJoystickState.getLY() <<
					" btnTri: " << mJoystickState.getTriangle() <<
					" btnSqu: " << mJoystickState.getSquare() <<
					" btnCir: " << mJoystickState.getCircle() << std::endl;
		
	}
	
	
	// Will never reach here
	toyotaHandler.stop();
	pandaHandler.stop();

	return 0;
}
