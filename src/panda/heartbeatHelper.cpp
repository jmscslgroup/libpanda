/*
 Author: Matt Bunting
 */

#include "panda/controller.h"

#include <unistd.h>

using namespace Panda;

HeartbeatHelper::HeartbeatHelper(Controller* handler) {
	this->mController = handler;
}

void HeartbeatHelper::doAction() {
	usleep(1000000.0/PANDA_RATE_HEARTBEAT);	// ~2 Hz
	
	if (mController != NULL) {
		mController->sendHeartBeat();
	}
}
