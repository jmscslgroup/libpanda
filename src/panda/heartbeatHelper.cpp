/*
 Author: Matt Bunting
 */

#include "panda.h"

#include <unistd.h>

using namespace Panda;

HeartbeatHelper::HeartbeatHelper(Handler* handler) {
	this->mHandler = handler;
}

void HeartbeatHelper::doAction() {
	usleep(1000000.0/PANDA_RATE_HEARTBEAT);	// ~2 Hz
	
	if (mHandler != NULL) {
//		printf("Sending heartbeat...\n");
//		mHandler->sendHeartBeat();
		
		mHandler->getUsb().sendHeartBeat();
		
		// TODO: notify observers:
		PandaHealth health;
		mHandler->getUsb().getHealth(&health);
		for (std::vector<HeartbeatHelperListener*>::iterator it = observers.begin();
			 it != observers.end();
			 it++) {
			(*it)->notificationHeartbeat(health);
		}
	}
}


void HeartbeatHelper::addObserver(HeartbeatHelperListener* observer) {
	observers.push_back(observer);
}
