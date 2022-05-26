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

bool NissanController::checkControlsAllowed(Panda::CanFrame* frame) {
	return false;
}
