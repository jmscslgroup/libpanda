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

#include "panda/toyota.h"

#include "joystickState.h"

int main(int argc, char **argv) {
	
	// Build the joystick reader:
	JoystickState mJoystickState;
	Joystick mJoystick;

	mJoystick.addObserver(&mJoystickState);
	mJoystick.open("/dev/input/js0");
	mJoystick.start();


	// Initialize panda and toyota handlers
	Panda::Handler pandaHandler;
	Panda::ToyotaHandler toyotaHandler(&pandaHandler);
	
	// Let's roll
	pandaHandler.initialize();
	toyotaHandler.start();
	
	// These are for setting the HUD
	unsigned char hudLaneLeft = 2;
	unsigned char hudLaneRight = 2;
	
	while(1) {
		usleep(1000000.0/10.0);	// run at ~10 Hz

		// Setting HUD elements:
		toyotaHandler.setHudLdaAlert( mJoystickState.getTriangle() );
		toyotaHandler.setHudCruiseCancelRequest( mJoystickState.getSquare() );
		hudLaneLeft += mJoystickState.getButtonL1Rising();
		hudLaneLeft -= mJoystickState.getButtonL2Rising();
		hudLaneRight += mJoystickState.getButtonR1Rising();
		hudLaneRight -= mJoystickState.getButtonR2Rising();
		toyotaHandler.setHudLanes(hudLaneLeft, hudLaneRight);
		toyotaHandler.setHudTwoBeeps( mJoystickState.getX() );
		toyotaHandler.setHudBarrier( mJoystickState.getDY() > 0 );
		toyotaHandler.setHudRepeatedBeeps( mJoystickState.getSelect() );
		
		// Acceleration command building.  Units are m/s^2
		double acceleration = 0.0;
		double joystickValue = mJoystickState.getLY();
		//const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s2
		//const int TOYOTA_MIN_ACCEL = -3000;       // -3.0 m/s2
		// The following limits can be achieved by setting the panda into "unsafe" mode:
		//const int TOYOTA_ISO_MAX_ACCEL = 2000;        // 2.0 m/s2
		//const int TOYOTA_ISO_MIN_ACCEL = -3500;       // -3.5 m/s2
		if (joystickValue > 0) {
			acceleration = 1.5 * joystickValue;
		} else if (joystickValue < 0) {
			acceleration = 3.0 * joystickValue;
		}
		
		// Steering torque command.  Deosn't yet work, unknown units
		joystickValue = mJoystickState.getRX();
		//const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
		int steerTorque = 1500 * joystickValue;	// range: -1500:1500
		
		
		// Send the Steering and Scceleration commands:
		if (!mJoystickState.getCircle()) {	// Holding circle tests the heartbeat (stopping it)
			toyotaHandler.setAcceleration(acceleration);
			toyotaHandler.setSteerTorque(steerTorque);
		}
		
		// Debug Joystick:
//		std::cout << "Joystick L1:" << mJoystickState.getL1() <<
//		" L2:" << mJoystickState.getL2() <<
//		" L3:" << mJoystickState.getL3() <<
//		" R1:" << mJoystickState.getR1() <<
//		" R2:" << mJoystickState.getR2() <<
//		" R3:" << mJoystickState.getR3() <<
//		" X:" << mJoystickState.getX() <<
//		" Tri:" << mJoystickState.getTriangle() <<
//		" Squ:" << mJoystickState.getSquare() <<
//		" Cir:" << mJoystickState.getCircle() <<
//		" Pause:" << mJoystickState.getPause() <<
//		" Select:" << mJoystickState.getSelect() <<
//		"\tLX:" << mJoystickState.getLX() <<
//		"\tLY:" << mJoystickState.getLY() <<
//		"\tRX:" << mJoystickState.getRX() <<
//		"\tRY:" << mJoystickState.getRY() <<
//		"\tDX:" << mJoystickState.getDX() <<
//		"\tDY:" << mJoystickState.getDY() <<  std::endl;
		
	}
	
	
	// Will never reach here
	toyotaHandler.stop();
	pandaHandler.stop();

	return 0;
}
