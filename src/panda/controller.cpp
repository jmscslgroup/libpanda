/*
 Author: Matt Bunting
 */

#include <unistd.h>
#include <chrono>

#include "panda/controller.h"
#include "panda/toyota.h"
#include "panda/nissan.h"



using namespace Panda;

Controller::Controller( ) {
	pandaHandler = NULL;
	
	setIntervalActionRate(200);
	decimatorControlsAllowedCounter = 0;
	
	mHeartbeatHelper = new HeartbeatHelper(this);
	mHeartbeatHelper->start();
}

Controller::~Controller() {
	mHeartbeatHelper->stop();
	delete mHeartbeatHelper;
}

Controller*  Controller::create( Panda::Handler& handler ) {
	Controller* result = NULL;
	switch (handler.getVehicleManufacturer()) {
		case VEHICLE_MANUFACTURE_TOYOTA:
			result = new ToyotaHandler;
			break;
			
		case VEHICLE_MANUFACTURE_NISSAN:
			result = new NissanController;
			break;
			
		default:
			break;
	}
	if (result != NULL) {
		result->pandaHandler = &handler;
		result->pandaHandler->getCan().addObserver(result);
	}
	
	return result;
}

void Controller::sendHeartBeat() {
//	std::cout << "In Controller::sendHeartBeat()" << std::endl;
	pandaHandler->getUsb().sendHeartBeat();
	
	pandaHandler->getUsb().getHealth(&health);	// this should live in Panda::PandaHandler

	for (std::vector<ControllerListener*>::iterator it = controllerObservers.begin();
		 it != controllerObservers.end();
		 it++) {
		(*it)->newPandaHealthNotification(health);

	}
}

void Controller::addObserver( ControllerListener* observer ) {
	controllerObservers.push_back(observer);
}


void Controller::newDataNotification(CanFrame* canFrame) {
	controls_allowed_prior = controls_allowed;
	controls_allowed = checkControlsAllowed(canFrame);
	
	if (controls_allowed_prior != controls_allowed) {
		for (std::vector<ControllerListener*>::iterator it = controllerObservers.begin();
			 it != controllerObservers.end();
			 it++) {
			(*it)->newControlNotification(this);
		}
	}
}

bool Controller::getControlsAllowed() {
	return controls_allowed;
}


void Controller::entryAction() {
	std::cout << "In Controller::entryAction():" << std::endl;
	
	std::cout << " - Setting power save to POWER_SAVE_STATUS_DISABLED:" << std::endl;
	pandaHandler->getUsb().setPowerSaveEnable(POWER_SAVE_STATUS_DISABLED);
	
	std::cout << " - Setting Safety to SAFETY_TOYOTA" << std::endl;
	/* For the params setting see the code in comma.ai/panda/board/safety/safety_toyota.h:
	 // safety param flags
	 // first byte is for eps factor, second is for flags
	 const uint32_t TOYOTA_PARAM_OFFSET = 8U;
	 const uint32_t TOYOTA_EPS_FACTOR = (1U << TOYOTA_PARAM_OFFSET) - 1U;
	 const uint32_t TOYOTA_PARAM_ALT_BRAKE = 1U << TOYOTA_PARAM_OFFSET;
	 const uint32_t TOYOTA_PARAM_STOCK_LONGITUDINAL = 2U << TOYOTA_PARAM_OFFSET;
	 
	 // The value of 73 is based on car controller code in openpilot for Toyotas
	*/
	pandaHandler->getUsb().setSafetyMode(SAFETY_TOYOTA, 73);
	
}

void Controller::exitAction() {
	std::cout << "In Controller::exitAction():" << std::endl;
	
	std::cout << " - Setting Safety to SAFETY_NOOUTPUT:" << std::endl;
	pandaHandler->getUsb().setSafetyMode(SAFETY_NOOUTPUT, 0);
	
	std::cout << " - Setting power save to POWER_SAVE_STATUS_ENABLED:" << std::endl;
	pandaHandler->getUsb().setPowerSaveEnable(POWER_SAVE_STATUS_ENABLED);
}

void Controller::doAction() {
	
	// Sample start time
	auto start = std::chrono::high_resolution_clock::now();
	
	if (decimatorControlsAllowedCounter++ >= decimatorTotalControlsAllowed) {
		decimatorControlsAllowedCounter = 0;
		for (std::vector<ControllerListener*>::iterator it = controllerObservers.begin();
			 it != controllerObservers.end();
			 it++) {
//			(*it)->newPandaHealthNotification(health);
			(*it)->newControlNotification(this);
		}
	}
	
	// Let child handle CAN messaging
	this->intervalAction();
	
	// Now deterine the time to delay
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	int durationInMicroseconds = duration.count();
	
	int microsecondsToSleep = (1000000.0/intervalActionRate) - durationInMicroseconds;
	
	if (microsecondsToSleep < 0) {
		fprintf(stderr, "WARNING! Controller::doAction() execution time is too long.  Duration: %d us\n", durationInMicroseconds );
		microsecondsToSleep = 0;
	} else if ( microsecondsToSleep >= (1000000.0/intervalActionRate)) {
		microsecondsToSleep = 1000000.0/intervalActionRate;
	}
	
	usleep(microsecondsToSleep);	// run at 600 Hz
	
}

void Controller::setIntervalActionRate( double rate ) {
	this->intervalActionRate = rate;
	
	decimatorTotalControlsAllowed = intervalActionRate/CONTROLLER_RATE_CA_REPORT;
}

void Controller::sendCan(CanFrame& frame) {
	this->pandaHandler->getCan().sendMessage(frame);
}

Panda::Handler* Controller::getPandaHandler() {
	return pandaHandler;
}
