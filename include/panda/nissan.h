/*
 Author: Matt Bunting
 */

#ifndef NISSAN_H
#define NISSAN_H

#include "panda/controller.h"

#define NISSAN_COMMAND_THREAD_RATE (200.0)	// Defines the rate of intervalAction

#define NISSAN_COMMAND_BUTTON_RATE (10.0)	// Defines the rate of sending a button
#define NISSAN_COMMAND_303_RATE (50.0)	// Defines the rate of message 303

#define NISSAN_DECIMATOR_BUTTON (NISSAN_COMMAND_THREAD_RATE/NISSAN_COMMAND_BUTTON_RATE)
#define NISSAN_DECIMATOR_303 (NISSAN_COMMAND_THREAD_RATE/NISSAN_COMMAND_303_RATE)

namespace Panda {

/*! CAN message building functions
	
 */
CanFrame buildCanNissanCruiseButtons(bool button);


void replaceCanThreeOhThree( CanFrame* frame, int WHEEL_TORQUE, int speed );
CanFrame buildCanThreeOhThree( int WHEEL_TORQUE, int speed );
void nissanParseThreeOhThree( CanFrame& frame, int* WHEEL_TORQUE, int* speed  );


/*! NissanController
	
 */
class NissanController : public Panda::Controller {
	friend class Panda::Controller;
	
private:
	// List of loop decimators:
	int decimatorButton;
	int decimatorThreeOhThree;
	
	// Functional attributes:
	bool buttonValue;
	
	
	// Functions that handle sending CAN message
	void sendCruiseButtons();
	void sendThreeOhThree();
	
	// Overloaded from Panda::controller
	// This is called at regular fast intervals, where we decimate the interval and send CAN messages
	void intervalAction();
	
	// Overloaded from Panda::controller
	bool checkControlsAllowed(Panda::CanFrame* frame);
	void handleSetSteerTorque( int steerTorque );
	void handleSetAcceleration( double acceleration );
	void newCanNotification(CanFrame* canFrame);
	
	
	
protected:
	NissanController();
	
public:
	void sendButton( bool value );
};

}

#endif