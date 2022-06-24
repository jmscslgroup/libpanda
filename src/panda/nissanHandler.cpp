/*
 Author: Matt Bunting
 */

#include "panda/nissan.h"

using namespace Panda;

NissanController::NissanController() {
	// Decimators:
	decimatorButton = 0;
	
	// Attributes:
	buttonValue = 0;
	
	// Other Initialization:
	this->setIntervalActionRate(NISSAN_COMMAND_THREAD_RATE);
}

void NissanController::intervalAction() {
	if (decimatorButton++ > NISSAN_DECIMATOR_BUTTON) {
		decimatorButton = 0;
		sendCruiseButtons();
	}
}

void NissanController::sendCruiseButtons() {
	
}

void NissanController::handleSetSteerTorque( int steerTorque ) {
	
}

void NissanController::handleSetAcceleration( double acceleration ) {
	
}

bool NissanController::checkControlsAllowed(Panda::CanFrame* frame) {
	return false;
}

void NissanController::sendButton( bool value ) {
	
}
