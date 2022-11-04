/*
 Author: Matt Bunting
 */

#ifndef NISSAN_H
#define NISSAN_H

#include "panda/controller.h"
#include "panda/buttonSequencer.h"

#define NISSAN_COMMAND_THREAD_RATE (200.0)	// Defines the rate of intervalAction


// The following is deprecated stuff:
#define NISSAN_COMMAND_BUTTON_RATE (10.0)	// Defines the rate of sending a button
#define NISSAN_COMMAND_303_RATE (50.0)	// Defines the rate of message 303

#define NISSAN_DECIMATOR_BUTTON (NISSAN_COMMAND_THREAD_RATE/NISSAN_COMMAND_BUTTON_RATE)
#define NISSAN_DECIMATOR_303 (NISSAN_COMMAND_THREAD_RATE/NISSAN_COMMAND_303_RATE)

// Stuff for ACC button presses:
#define NISSAN_COMMAND_PEDAL_WAIT_TIME (1.0)	// The time in seconds to wait before pressing SET after pedals released
#define NISSAN_COMMAND_SET_WAIT_TIME (1.0)		// The time to wait in seconds after pressing SET to enter controls_allowed
#define NISSAN_TRANSITION_TO_BUTTON_TEST (30.0)		//
#define NISSAN_TRANSITION_EXIT_BUTTON_TEST (0.25)		//
#define NISSAN_DECIMATOR_PEDAL_WAIT (NISSAN_COMMAND_THREAD_RATE * NISSAN_COMMAND_PEDAL_WAIT_TIME)
#define NISSAN_DECIMATOR_TRANSITION_TO_BUTTON_TEST (NISSAN_COMMAND_THREAD_RATE * NISSAN_TRANSITION_TO_BUTTON_TEST)
#define NISSAN_DECIMATOR_TRANSITION_EXIT_BUTTON_TEST (NISSAN_COMMAND_THREAD_RATE * NISSAN_TRANSITION_EXIT_BUTTON_TEST)

// Stuff for a concurrent state machine for button chekcing:
#define NISSAN_CHECK_BUTTON_RESPONSE (2.0)		// The time in seconds to wait for a button response on the CAN bus
#define NISSAN_DECIMATOR_CHECK_BUTTON_RESPONSE (NISSAN_COMMAND_THREAD_RATE * NISSAN_CHECK_BUTTON_RESPONSE)


// Nassan' reported CRUISE_STATE values, from reverse-engineering:
//#define NISSAN_CRUISE_STATE_OFF (1)
//#define NISSAN_CRUISE_STATE_STANDSTILL (9)
//#define NISSAN_CRUISE_STATE_IDLE (13)
//#define NISSAN_CRUISE_STATE_ACTIVE (25)
//#define NISSAN_CRUISE_STATE_UNKNOWN_ACTIVE (29)
#define NISSAN_CRUISE_STATE_OFF (0)
#define NISSAN_CRUISE_STATE_STANDSTILL (4)
#define NISSAN_CRUISE_STATE_IDLE (6)
#define NISSAN_CRUISE_STATE_ACTIVE (12)
#define NISSAN_CRUISE_STATE_UNKNOWN_ACTIVE (14)	// Occurs from 12, when driver presses gas pedal

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
//		ACC_STATE_POWER_TOGGLE_NEEDED,
		//ACC_STATE_COME_HOME	// ON
		ACC_STATE_BUTTON_TEST
	} AccCommandState;
	
	AccCommandState state;
	bool gasPressed;
//	bool brakePressed;
	unsigned char cruiseState;
	bool cruiseOn;
	bool cruiseEngaged;
	NissanButton buttonState;
	NissanButton lastButtonSent;
	
	bool pedalTimerExpired;
	
	// Gpio stuff operation
	MatthatAccButtonRelay relayHandler;
	MatthatBeep buzzerHandler;
	DigitalPotHandler potHandler;
	
	// List of loop decimators:
	int decimatorPedalWaitTimer;
	int decimatorSetWaitTimer;
	int decimatorTranstionToButtonTest;
	int decimatorTranstionExitButtonTest;
	
	// Functional attributes:
	
	
	// a concurrent state machine for checking button responses:
	// State definitions
	typedef enum {
		CHECK_BUTTON_PASSED,
		CHECK_BUTTON_RUNNING,
		CHECK_BUTTON_FAILED
	} CheckButtonState;
	
	CheckButtonState stateCheckButton;
	int decimaterCheckButtonResponse;
	bool checkButtonFailed;
	
	
	// Functions for parsing CAN messages to udpate state variables:
	void parseCruiseState(CanFrame* canFrame, unsigned char* cruiseState, bool* cruiseEngaged);
	void parseGasPressed(CanFrame* canFrame, bool* gasPressed);
	void parseAccButtons(CanFrame* canFrame, NissanButton* buttonState);
	
	// Overloaded from Panda::controller
	// This is called at regular fast intervals, where we decimate the interval and send CAN messages
	void intervalAction();
	
	// Overloaded from Panda::controller
	bool checkControlsAllowed(Panda::CanFrame* frame);
	void handleSetSteerTorque( int steerTorque );
	void handleSetAcceleration( double acceleration );
	void newCanNotification(CanFrame* canFrame);
	
	
	// State handling:
	void transitionToState( AccCommandState newState );
	void enterState( AccCommandState newState );
	void exitState( AccCommandState priorState );
	void updateStateVariables( CanFrame* canFrame );
	
	// and for concurrent state machine:
	void intervalActionCheckButton();
	void transitionToState( CheckButtonState newState );
	void enterState( CheckButtonState oldState );
	void exitState( CheckButtonState newState );
	
	// Helper functions:
	void sendButtonPress( NissanButton button );
	
protected:
	NissanAccButtonController();
	
public:
	
	~NissanAccButtonController();
	
	bool sendButton( NissanButton button );
	
	
	bool isHardwareConnectionGood();
	
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
