/*
 Author: Matt Bunting
 */

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

#include "panda/controller.h"
#include "panda/nissan.h"

#include "joystickState.h"


class ExampleControllerListener : public Panda::ControllerListener {
private:
	
	
	void newPandaHealthNotification(const PandaHealth& pandaHealth) {
//		Panda::printPandaHealth(pandaHealth);
		printf("Panda reports controls_allowed as: %d and safetyModel as %s\n", pandaHealth.controls_allowed, Panda::safetyModelToString(pandaHealth.safety_mode));
	};
	
	
	void newControlNotification(Panda::Controller* controller) {
		std::cout << "ExampleControllerListener::newControlNotification() controls_allowed: " << (int)controller->getControlsAllowed() << std::endl;
	};
	
public:
	
};


class ExampleCheckCanSendSuccess : public Panda::CanListener {
	void newDataNotification(Panda::CanFrame* frame) {
		if (frame->rejected) {
			std::cout << "Found a rejected attempt to send CAN message:" << std::endl;
		} else if (frame->returned) {
			std::cout << "Success in sending CAN message with ID :" << std::endl;
		}
	}
};



static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}

int main(int argc, char **argv) {
	
//	// Build the joystick reader:
//	JoystickState mJoystickState;
//	Joystick mJoystick;
//
//	mJoystick.addObserver(&mJoystickState);
//	mJoystick.open("/dev/input/js0");
//	mJoystick.start();

	//Set up graceful exit
	signal(SIGINT, killPanda);

	// Initialize panda and toyota handlers
	Panda::Handler pandaHandler;
	pandaHandler.getCan().saveToCsvFile("nissanTest.csv");
//	Panda::ToyotaHandler toyotaHandler(&pandaHandler);
//	Panda::ToyotaHandler toyotaHandler();
//	Panda::NissanController* controllerAsNissanController = NULL;
	Panda::NissanAccButtonController* controllerAsNissanController = NULL;
	
	
//	ExampleCheckCanSendSuccess myExampleCheckCanSendSuccess;
//	pandaHandler.getCan().addObserver(&myExampleCheckCanSendSuccess);
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
		
			std::cerr << "     : Force setting the VIN to JN8AT3CB9MW240939" << std::endl;
		pandaHandler.forceSetVin((const unsigned char*)"JN8AT3CB9MW240939");	// Hard coded VIN setting
		pandaController = new Panda::ControllerClient(pandaHandler);
		if(pandaController->getController() == NULL) {
			std::cerr << "ERROR 2: No VIN discovered, unable to make test controller" << std::endl;
			exit(EXIT_FAILURE);
		}
		
	}
	
	if (pandaHandler.getVehicleManufacturer() == Panda::VEHICLE_MANUFACTURE_NISSAN) {
		controllerAsNissanController = static_cast<Panda::NissanAccButtonController*>(pandaController->getController());
	} else {
		
		std::cerr << "This is " << std::endl;
		
		pandaController->getController()->stop();
		pandaHandler.stop();
	}
	
	ExampleControllerListener myExampleControllerListener;
//	myExampleControllerListener.controller = controllerAsNissanController;	// Hard coding
	pandaController->getController()->addObserver(&myExampleControllerListener);
	pandaController->getController()->start();
	
	
	// More hardcoding:
//	pandaHandler.getUsb().setSafetyMode(SAFETY_ALLOUTPUT, 0);
	
//	int printPandaHealthDecimator = 0;
	
	Panda::CanFrame frame;
	frame.data[11] = 0x00;
	frame.data[10] = 0x96;
	frame.data[9] = 0x0f;
	frame.data[8] = 0x26;
	frame.data[7] = 0x0d;
	frame.data[6] = 0x26;
	frame.data[5] = 0x60;
	frame.data[4] = 0xa1;
	frame.data[3] = 0x07;
	frame.data[2] = 0x2f;
	frame.data[1] = 0x01;
	frame.data[0] = 0x00;
	frame.bus = 0;
	frame.messageID = 303;
	frame.dataLength = 12;
	
//	pandaHandler.getUsb().setSafetyMode(SAFETY_ALLOUTPUT, 1);
	while(keepRunning) {
		usleep(1000000.0/10.0);	// run at ~10 Hz

		// For testing these will do nothing other than clear heartbeats
			pandaController->getController()->setAcceleration(0);
			pandaController->getController()->setSteerTorque(0);
		
		

		
		// Fake a message:
//		pandaController->getController()->newDataNotification(&frame);
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
