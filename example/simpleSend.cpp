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
#include <unistd.h>
#include <cstring>
#include <cmath>

#include "panda/controller.h"
#include "panda/toyota.h"

#include "joystickState.h"


class ExampleControllerListener : public Panda::ControllerListener {
private:
	void newPandaHealthNotification(const PandaHealth& pandaHealth) {
		Panda::printPandaHealth(pandaHealth);
	};
	
	
	void newControlNotification(Panda::Controller* controller) {
		std::cout << "ExampleControllerListener::newControlNotification() controls_allowed: " << controller->getControlsAllowed() << std::endl;
	};
};



static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}

int main(int argc, char **argv) {
	
	// Build the joystick reader:
	JoystickState mJoystickState;
	Joystick mJoystick;

	mJoystick.addObserver(&mJoystickState);
	mJoystick.open("/dev/input/js0");
	mJoystick.start();

	//Set up graceful exit
	signal(SIGINT, killPanda);

	// Initialize panda and toyota handlers
	Panda::Handler pandaHandler;
//	Panda::ToyotaHandler toyotaHandler(&pandaHandler);
//	Panda::ToyotaHandler toyotaHandler();
	
	
	
//	pandaHandler.getCan().addObserver(&toyotaHandler);
	
	// Let's roll
	pandaHandler.initialize();
	
	// initialize gets the VIN, now we can build a controller:
	Panda::ControllerClient* pandaController = new Panda::ControllerClient(pandaHandler);
//	toyotaHandler.start();
	if(pandaController->getController() == NULL) {
		std::cerr << "ERROR: No VIN discovered, unable to make controller" << std::endl;
//		exit(EXIT_FAILURE);
		
		delete pandaController;
		
		pandaHandler.forceSetVin((const unsigned char*)"2T3Y1RFV8KC014025");
		pandaController = new Panda::ControllerClient(pandaHandler);
		if(pandaController->getController() == NULL) {
			std::cerr << "ERROR 2: No VIN discovered, unable to make test controller" << std::endl;
			exit(EXIT_FAILURE);
		}
		
	}
	
	ExampleControllerListener myExampleControllerListener;
	pandaController->getController()->addObserver(&myExampleControllerListener);
	pandaController->getController()->start();
	
	// These are for setting the HUD
	unsigned char hudLaneLeft = 2;
	unsigned char hudLaneRight = 2;
	
//	int printPandaHealthDecimator = 0;
    int steerTorqueToSend = 0;
	while(keepRunning) {
		usleep(1000000.0/100.0);	// run at ~50 Hz

//		if(printPandaHealthDecimator++ >= 10) {	// run at 1Hz
//			printPandaHealthDecimator = 0;
//			Panda::printPandaHealth(toyotaHandler.getPandaHealth());
////			std::cout << "Controls Allowed panda: " << (int)toyotaHandler.getPandaControlsAllowed() << std::endl;
////			std::cout << "Controls Allowed Libpanda: " << toyotaHandler.getControlsAllowed()() << std::endl;
////			std::cout << "Ignition On: " << toyotaHandler.getIgnitionOn() << std::endl;
//		}
		
		// Setting HUD elements:
		
		hudLaneLeft += mJoystickState.getButtonL1Rising();
		hudLaneLeft -= mJoystickState.getButtonL2Rising();
		hudLaneRight += mJoystickState.getButtonR1Rising();
		hudLaneRight -= mJoystickState.getButtonR2Rising();
		if (pandaHandler.getVehicleManufacturer() == Panda::VEHICLE_MANUFACTURE_TOYOTA) {
			Panda::ToyotaHandler* toyotaHandler = static_cast<Panda::ToyotaHandler*>(pandaController->getController());
			
			toyotaHandler->setHudLanes(hudLaneLeft, hudLaneRight);
			
			toyotaHandler->setHudLdaAlert( mJoystickState.getTriangle() );
			toyotaHandler->setHudTwoBeeps( mJoystickState.getX() );
			toyotaHandler->setHudRepeatedBeeps( mJoystickState.getSelect() );
			toyotaHandler->setHudBarrier( mJoystickState.getDY() > 0 );
			toyotaHandler->setHudMiniCar( mJoystickState.getDX() > 0 );
		}
		
		
		
		
		// This will cancel the cruise control, cruise must be rest by driver to allow further controls
//		toyotaHandler.setHudCruiseCancelRequest( mJoystickState.getSquare() );
		
		// Acceleration command building.  Units are m/s^2
		// The following are hard-coded limits in the Panda firmware:
		//const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s2
		//const int TOYOTA_MIN_ACCEL = -3000;       // -3.0 m/s2
		// The following limits can be achieved by setting the panda into "unsafe" mode:
		//const int TOYOTA_ISO_MAX_ACCEL = 2000;        // 2.0 m/s2
		//const int TOYOTA_ISO_MIN_ACCEL = -3500;       // -3.5 m/s2
		double acceleration = 0.0;
		double joystickValue = mJoystickState.getLY();	// getLY() returns a range of -1.0:1.0
        
//		if (joystickValue > 0) {
//			acceleration = 1.5 * joystickValue;
//		} else if (joystickValue < 0) {
//			acceleration = 3.0 * joystickValue;
//		}
        
        acceleration = 4.5*((joystickValue + 1.0)/2.0 ) - 3.0; // This function means joystick at rest = -0.75 m/s/s
		
		// Steering torque command.  Deosn't yet work, unknown units
		// The following is a hard-coded limit in the Panda firmware:
		//const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
		joystickValue = mJoystickState.getRX();	// getRX() returns a range of -1.0:1.0
		double steerTorque = 1500 * joystickValue;	// range: -1500:1500
		
        if(steerTorqueToSend < steerTorque) {
            double diff = steerTorque - steerTorqueToSend;
            steerTorqueToSend += diff > 5 ? 5 : diff;
        } else if (steerTorqueToSend > steerTorque) {
            double diff = steerTorque - steerTorqueToSend;
            steerTorqueToSend += diff < -5 ? -5 : diff;
        }
//        if(
//        steerTorqueToSend += (steerTorque - steerTorqueToSend)/200.0;
		
		// Send the Steering and Scceleration commands:
		// Holding circle tests the heartbeat (stopping it)
		// The heartbeat failing will also trigger some HUD elements like setHudRepeatedBeeps and setHudLdaAlert
		if ( !mJoystickState.getCircle() ) {
//			toyotaHandler.setAcceleration(acceleration);
////			toyotaHandler.setSteerTorque(steerTorque);
//			toyotaHandler.setSteerTorque(0);
            std::cout << "Sending steerTorque = " << steerTorqueToSend << std::endl;
			pandaController->getController()->setAcceleration(acceleration);
			pandaController->getController()->setSteerTorque(steerTorqueToSend);
//			pandaController->getController()->setSteerTorque(0);
		}
		
		// Debug Joystick:
//		mJoystickState.printState();
		
	}
	
	
	// Will never reach here
	std::cout << "Stopping pandacontroller..." << std::endl;
//	toyotaHandler.stop();
	pandaController->getController()->stop();
	std::cout << "Stopping pandaHandler..." << std::endl;
	pandaHandler.stop();

	std::cout << "simpleSend is Done." << std::endl;
//	return 0;
	exit(EXIT_SUCCESS);
	return 0;
}
