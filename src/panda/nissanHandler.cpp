/*
 Author: Matt Bunting
 */

#include "panda/nissan.h"

using namespace Panda;

NissanController::NissanController() {
	this->setIntervalActionRate(NISSAN_COMMAND_THREAD_RATE);
}

void NissanController::intervalAction() {
	
}

void NissanController::handleSetSteerTorque( int steerTorque ) {
	
}

void NissanController::handleSetAcceleration( double acceleration ) {
	
}

bool NissanController::checkControlsAllowed(Panda::CanFrame* frame) {
	return false;
}
