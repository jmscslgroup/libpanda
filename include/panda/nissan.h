/*
 Author: Matt Bunting
 */

#ifndef NISSAN_H
#define NISSAN_H

#include "panda/controller.h"

#define NISSAN_COMMAND_THREAD_RATE (200.0)	// Defines the rate of intervalAction

#define NISSAN_COMMAND_BUTTON_RATE (10.0)	// Defines the rate of sending a button

#define NISSAN_DECIMATOR_BUTTON (NISSAN_COMMAND_THREAD_RATE/NISSAN_COMMAND_BUTTON_RATE)

namespace Panda {

/*! CAN message building functions
	
 */
CanFrame buildCanNissanCruiseButtons(bool button);


/*! NissanController
	
 */
class NissanController : public Panda::Controller {
	friend class Panda::Controller;
	
private:
	// List of loop decimators:
	int decimatorButton;
	
	// Functional attributes:
	bool buttonValue;
	
	
	// Functions that handle sending CAN message
	void sendCruiseButtons();
	
	// Overloaded from Panda::controller
	// This is called at regular fast intervals, where we decimate the interval and send CAN messages
	void intervalAction();
	
	// Overloaded from Panda::controller
	bool checkControlsAllowed(Panda::CanFrame* frame);
	

	void handleSetSteerTorque( int steerTorque );
	
	void handleSetAcceleration( double acceleration );
	
protected:
	NissanController();
	
public:
	void sendButton( bool value );
};

}

#endif
