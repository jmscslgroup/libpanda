/*
 Author: Matt Bunting
 */

#ifndef NISSAN_H
#define NISSAN_H

#include "panda/controller.h"

#define NISSAN_COMMAND_THREAD_RATE (200.0)	// Defines the rate of intervalAction

namespace Panda {

class NissanController : public Panda::Controller {
	friend class Panda::Controller;
	
private:
	
	// Overloaded from Panda::controller
	// This is called at regular fast intervals, where we decimate the interval and send CAN messages
	void intervalAction();
	
	// Overloaded from Panda::controller
	bool checkControlsAllowed(Panda::CanFrame* frame);
	

	void handleSetSteerTorque( int steerTorque );
	
	void handleSetAcceleration( double acceleration );
	
protected:
	NissanController();
};

}

#endif
