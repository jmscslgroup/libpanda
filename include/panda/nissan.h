/*
 Author: Matt Bunting
 */

#ifndef NISSAN_H
#define NISSAN_H

#include "panda/controller.h"
#include "panda/buttonSequencer.h"

#define NISSAN_COMMAND_THREAD_RATE (200.0)	// Defines the rate of intervalAction

#define NISSAN_COMMAND_BUTTON_RATE (10.0)	// Defines the rate of sending a button
#define NISSAN_COMMAND_303_RATE (50.0)	// Defines the rate of message 303

#define NISSAN_DECIMATOR_BUTTON (NISSAN_COMMAND_THREAD_RATE/NISSAN_COMMAND_BUTTON_RATE)
#define NISSAN_DECIMATOR_303 (NISSAN_COMMAND_THREAD_RATE/NISSAN_COMMAND_303_RATE)

// Stuff for ACC button presses:
#define NISSAN_COMMAND_PEDAL_WAIT_TIME (1.0)	// Defines the rate of sending a button
#define NISSAN_DECIMATOR_PEDAL_WAIT (NISSAN_COMMAND_THREAD_RATE * NISSAN_COMMAND_PEDAL_WAIT_TIME)

namespace Panda {


/*! NissanAccButtonController
	This class is really event based, but using he thread may let us implement things like heartbeats
 */
class NissanAccButtonController : public Panda::Controller {
	friend class Panda::Controller;
	
private:
	// State definitions
	typedef enum {
		ACC_STATE_OFF,
		ACC_STATE_IDLE,	// ON
		ACC_STATE_SET_WAIT,	// wait for SET to be recognized to change vehicle's state
		ACC_STATE_CONTROLS_ALLOWED,	// armed and ON
		ACC_STATE_POWER_TOGGLE_NEEDED,
		//ACC_STATE_COME_HOME	// ON
	} AccCommandState;
	
	AccCommandState state;
	bool gasPressed;
//	bool brakePressed;
	unsigned char cruiseState;
	bool cruiseOn;
	
	bool pedalTimerExpired;
	
	// Gpio stuff operation
	MatthatAccButtonRelay relayHandler;
	MatthatBeep buzzerHandler;
	DigitalPotHandler potHandler;
	
	// List of loop decimators:
	int decimatorPedalWaitTimer;
	int decimatorSetWaitTimer;
	
	// Functional attributes:
	
	
	
	// Functions that handle sending Button Press message
//	void arm
	
	// Overloaded from Panda::controller
	// This is called at regular fast intervals, where we decimate the interval and send CAN messages
	void intervalAction();
	
	// Overloaded from Panda::controller
	bool checkControlsAllowed(Panda::CanFrame* frame);
	void handleSetSteerTorque( int steerTorque );
	void handleSetAcceleration( double acceleration );
	void newCanNotification(CanFrame* canFrame);
	
	void enterState( AccCommandState newState );
	void exitState( );
	
	
	void updateStateVariables( CanFrame* canFrame );
	
protected:
	NissanAccButtonController();
	
public:
	
	~NissanAccButtonController();
	
	bool sendButton( NissanButton button );
	
	static const char* stateToName( const AccCommandState& state);
	
};

/*! CAN message building functions
	
 */
CanFrame buildCanNissanCruiseButtons(bool button);


void replaceCanThreeOhThree( CanFrame* frame, int WHEEL_TORQUE, int speed );
CanFrame buildCanThreeOhThree( int WHEEL_TORQUE, int speed );
void nissanParseThreeOhThree( CanFrame& frame, int* WHEEL_TORQUE, int* speed  );

void nissanPedalThrottle( CanFrame& frame, int* ACCEL_PEDAL_POSITION  );
void nissanParseCruise( CanFrame& frame, unsigned char* CRUISE_STATE, bool* CRUISE_ENGAGED  );
void nissanParseWheelButtons( CanFrame& frame, unsigned char* ACC_BTNS  );

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
