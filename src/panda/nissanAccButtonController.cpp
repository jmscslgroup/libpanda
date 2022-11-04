/*
 Author: Matt Bunting
 
 Statecharts represent diagrams in Rev 5
 */

#include <cstring>

#include "panda/nissan.h"

#define FILE_ORANGE_WIRE_CONNECTED "/etc/libpanda.d/wireAccButtonConnected"

using namespace Panda;

// This helper was copied from pandacord:
void writeToFileThenClose(const char* filename, const char* data) {
	FILE* file = fopen( filename, "w+");
	fwrite( data, 1, strlen(data), file);
	fclose(file);
};

NissanAccButtonController::NissanAccButtonController() {
	writeToFileThenClose(FILE_ORANGE_WIRE_CONNECTED, "1\n");
	
	// State Variables:
	gasPressed = true;
//	brakePressed = false;
	cruiseState = NISSAN_CRUISE_STATE_OFF;
	cruiseOn = false;
	cruiseEngaged = false;
	
	// Other Initialization:
	this->setIntervalActionRate(NISSAN_COMMAND_THREAD_RATE);
	
	state = ACC_STATE_OFF;
	//transitionToState(ACC_STATE_OFF);
	
	potHandler.pressButton(NISSAN_BUTTON_OFF);	// Just incase
	lastButtonSent = NISSAN_BUTTON_OFF;
	
	stateCheckButton = CHECK_BUTTON_PASSED;
	checkButtonFailed = false;
}

NissanAccButtonController::~NissanAccButtonController() {
	relayHandler.disarm();
}

void NissanAccButtonController::sendButtonPress( NissanButton button ) {
	potHandler.pressButton(button);
	
	// Kickoff the button check timer, if not running:
	if (stateCheckButton != CHECK_BUTTON_RUNNING &&
		lastButtonSent != button) {
		transitionToState(CHECK_BUTTON_RUNNING);
	}
	lastButtonSent = button;
}

const char* NissanAccButtonController::stateToName( const AccCommandState& state) {
	switch (state) {
		case ACC_STATE_OFF: return "ACC_STATE_OFF";
		case ACC_STATE_IDLE: return "ACC_STATE_IDLE";
		case ACC_STATE_SET_WAIT: return "ACC_STATE_SET_WAIT";
		case ACC_STATE_CONTROLS_ALLOWED: return "ACC_STATE_CONTROLS_ALLOWED";
		case ACC_STATE_BUTTON_TEST: return "ACC_STATE_BUTTON_TEST";
//		case ACC_STATE_POWER_TOGGLE_NEEDED: return "ACC_STATE_POWER_TOGGLE_NEEDED";
	}
	return "";
}

void NissanAccButtonController::transitionToState( AccCommandState newState ) {
	std::cout << "NissanAccButtonController: Transition from " << stateToName(state) << " --> " << stateToName(newState) << std::endl;

	exitState(state);
	enterState(newState);
}

void NissanAccButtonController::enterState( AccCommandState newState ) {
//	exitState();
//	std::cout << "NissanAccButtonController: Entering state " << stateToName(newState) << std::endl;
	switch (newState) {
		case ACC_STATE_OFF:
			relayHandler.disarm();
			sendButtonPress(NISSAN_BUTTON_OFF);	// Just incase
			decimatorTranstionToButtonTest = 0;
			break;
			
		case ACC_STATE_IDLE:
			relayHandler.disarm();
			decimatorPedalWaitTimer = 0;
			break;
			
		case ACC_STATE_SET_WAIT:
			decimatorSetWaitTimer = 0;
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
//			buzzerHandler.singleBeep();
			relayHandler.arm();
			break;
			
//		case ACC_STATE_POWER_TOGGLE_NEEDED:
//			relayHandler.disarm();
//			break;
			
		case ACC_STATE_BUTTON_TEST:
			relayHandler.arm();
			sendButtonPress(NISSAN_BUTTON_CANCEL);
			decimatorTranstionExitButtonTest = 0;
			break;
	}
	
	this->state = newState;
}

void NissanAccButtonController::exitState(AccCommandState oldState) {
	switch (oldState) {
		case ACC_STATE_OFF:
			break;
			
		case ACC_STATE_IDLE:
			break;
			
		case ACC_STATE_SET_WAIT:
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
//			buzzerHandler.doubleBeep();
			break;
			
//		case ACC_STATE_POWER_TOGGLE_NEEDED:
//			break;
		case ACC_STATE_BUTTON_TEST:
			break;
	}
}

// This is basically a Statechart doAction:
void NissanAccButtonController::intervalAction() {
	
	// No matter the state, if cruise was turned off then enter that state:
	if (!cruiseOn &&
		(state != ACC_STATE_OFF) &&
		(state != ACC_STATE_BUTTON_TEST) ) {
		transitionToState(ACC_STATE_OFF);
	}
	
	switch (state) {
		case ACC_STATE_OFF:
			if (cruiseOn) {
				transitionToState(ACC_STATE_IDLE);
			}
			if( decimatorTranstionToButtonTest++ > NISSAN_DECIMATOR_TRANSITION_TO_BUTTON_TEST) {
				transitionToState(ACC_STATE_BUTTON_TEST);
			}
			break;
			
		case ACC_STATE_IDLE:
//			if ( gasPressed ) {	// brake press?
//				decimatorPedalWaitTimer = 0;
//			} else if( decimatorPedalWaitTimer++ > NISSAN_DECIMATOR_PEDAL_WAIT ) {
//				decimatorPedalWaitTimer = 0;
//				// Timer expired, enter new state:
//				transitionToState(ACC_STATE_SET_WAIT);
//			}
			if (cruiseEngaged == 1) {
				transitionToState(ACC_STATE_SET_WAIT);
			}
			break;
			
		case ACC_STATE_SET_WAIT:
			if( decimatorSetWaitTimer++ > NISSAN_DECIMATOR_PEDAL_WAIT ) {
				// Timer expired, enter new state:
				transitionToState(ACC_STATE_CONTROLS_ALLOWED);
			}
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
			if (cruiseEngaged == 0) {
				transitionToState(ACC_STATE_IDLE);
			}
			break;
			
//		case ACC_STATE_POWER_TOGGLE_NEEDED:
//			transitionToState(ACC_STATE_IDLE);
//			break;
		case ACC_STATE_BUTTON_TEST:
			if (cruiseOn) {
				transitionToState(ACC_STATE_IDLE);
			}
			if( decimatorTranstionExitButtonTest++ > NISSAN_DECIMATOR_TRANSITION_EXIT_BUTTON_TEST) {
				transitionToState(ACC_STATE_OFF);
			}
			break;
	}
	
	
	/*
	 This is a concurrent state machine purely for checking button activity on the 
	 */
	intervalActionCheckButton();
}

void NissanAccButtonController::parseCruiseState(CanFrame* canFrame, unsigned char* cruiseState, bool* cruiseEngaged) {
	if (canFrame->messageID == 308) {
		unsigned char CRUISE_STATE;
		bool CRUISE_ENGAGED;
		nissanParseCruise(*canFrame, &CRUISE_STATE, &CRUISE_ENGAGED);
		
		if (*cruiseState != CRUISE_STATE) {
			*cruiseState = CRUISE_STATE;
			
			printf("parseCruiseState(): CAN change detected: Message 308: cruiseState = %d\n", (int)*cruiseState);
		}
		
		if (*cruiseEngaged != CRUISE_ENGAGED) {
			*cruiseEngaged = CRUISE_ENGAGED;
			printf("parseCruiseState(): CAN change detected: Message 308: cruiseEngaged = %d\n", *cruiseEngaged);
		}
		
	}
}

void NissanAccButtonController::parseGasPressed(CanFrame* canFrame, bool* gasPressed) {
	if (canFrame->messageID == 140) {
		int ACCEL_PEDAL_POSITION;
		nissanPedalThrottle(*canFrame, &ACCEL_PEDAL_POSITION);
		if (*gasPressed != (ACCEL_PEDAL_POSITION != 0)) {
			*gasPressed = ACCEL_PEDAL_POSITION != 0;
			
//			printf("parseCruiseState(): Change detected: ACCEL_PEDAL_POSITION = %d\n", ACCEL_PEDAL_POSITION);
			printf("parseCruiseState(): CAN change detected: gasPressed = %d\n", *gasPressed);
		}
	}
}

//void parseBrakePressed(CanFrame* canFrame, bool* brakePressed) {
//	if (canFrame->messageID == 303) {	// fixme
//
//	}
//}

void NissanAccButtonController::parseAccButtons(CanFrame* canFrame, NissanButton* buttonState) {
	if (canFrame->messageID == 1119) {
		unsigned char ACC_BTNS = *buttonState;
		nissanParseWheelButtons( *canFrame, &ACC_BTNS  );
		if (*buttonState != ACC_BTNS) {
			*buttonState = (NissanButton)ACC_BTNS;
			
			if(stateCheckButton == CHECK_BUTTON_RUNNING) {
				transitionToState(CHECK_BUTTON_PASSED);
			}
//			printf("parseCruiseState(): Change detected: ACCEL_PEDAL_POSITION = %d\n", ACCEL_PEDAL_POSITION);
			printf("parseAccButtons(): CAN change detected: buttonState = %d:%s\n", (int)*buttonState, nissanButtonToStr( *buttonState ));
		}
	}
}

void NissanAccButtonController::updateStateVariables(CanFrame* canFrame) {
	parseCruiseState(canFrame, &cruiseState, &cruiseEngaged);
	cruiseOn = (cruiseState != NISSAN_CRUISE_STATE_OFF);
	parseGasPressed(canFrame, &gasPressed);
//	parseBrakePressed(canFrame, &brakePressed);
	parseAccButtons(canFrame, &buttonState);
}



void NissanAccButtonController::newCanNotification(CanFrame* canFrame) {
	updateStateVariables( canFrame);
	
//	// No matter the state, if cruise was turned off then enter that state:
//	if (!cruiseOn &&
//		state != ACC_STATE_OFF) {
//		transitionToState(ACC_STATE_OFF);
//	}
	
	switch (state) {
		case ACC_STATE_OFF:
//			if (cruiseOn) {
//				transitionToState(ACC_STATE_IDLE);
//			}
			break;
			
		case ACC_STATE_IDLE:
//			if (cruiseEngaged == 1) {
//				transitionToState(ACC_STATE_SET_WAIT);
//			}
			break;
			
		case ACC_STATE_SET_WAIT:
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
//			if (cruiseEngaged == 0) {
//				transitionToState(ACC_STATE_IDLE);
//			}
			break;
			
//		case ACC_STATE_POWER_TOGGLE_NEEDED:
//			break;
			
		case ACC_STATE_BUTTON_TEST:
			break;
	}
	
}

void NissanAccButtonController::handleSetSteerTorque( int steerTorque ) {
	// nothing to do
}

void NissanAccButtonController::handleSetAcceleration( double acceleration ) {
	// nothing to do
}

bool NissanAccButtonController::checkControlsAllowed(Panda::CanFrame* frame) {
	return state == ACC_STATE_CONTROLS_ALLOWED;
}

bool NissanAccButtonController::sendButton( NissanButton button ) {
	setAcceleration(0);	// HACK to clear heartbeat within button pressing scheme
	setSteerTorque(0);	// HACK to clear heartbeat within button pressing scheme
	if ((state == ACC_STATE_CONTROLS_ALLOWED) &&
		(cruiseEngaged == true) ) {	// This prevent race conditions
		//		printf("Sending a button! %d\n", (int)button);
		sendButtonPress(button);
		
		return 1;
	}
	return 0;
}


void NissanAccButtonController::intervalActionCheckButton() {
	switch (stateCheckButton) {
		case CHECK_BUTTON_PASSED:
			break;
			
		case CHECK_BUTTON_RUNNING:
			if(decimaterCheckButtonResponse++ > NISSAN_DECIMATOR_CHECK_BUTTON_RESPONSE) {
				transitionToState(CHECK_BUTTON_FAILED);
			}
			break;
			
		case CHECK_BUTTON_FAILED:
			break;
	}
}

void NissanAccButtonController::transitionToState( CheckButtonState newState ) {
	exitState(stateCheckButton);
	enterState(newState);
}



void NissanAccButtonController::enterState( CheckButtonState newState ) {
	switch (newState) {
		case CHECK_BUTTON_PASSED:
			if (checkButtonFailed) {
				writeToFileThenClose(FILE_ORANGE_WIRE_CONNECTED, "1\n");
				std::cerr << "NissanAccButtonController: \"Orange\" wire is good! Reconnected to ACC buttons." << std::endl;
			}
			checkButtonFailed = false;
			break;
			
		case CHECK_BUTTON_RUNNING:
			decimaterCheckButtonResponse = 0;
			break;
			
		case CHECK_BUTTON_FAILED:
			if (!checkButtonFailed) {
				writeToFileThenClose(FILE_ORANGE_WIRE_CONNECTED, "0\n");
			}
			checkButtonFailed = true;
			std::cerr << "ERROR! NissanAccButtonController: \"Orange\" wire disconnected from ACC buttons!" << std::endl;
			break;
	}
	
	stateCheckButton = newState;
}

void NissanAccButtonController::exitState( CheckButtonState priorState ) {
	switch (priorState) {
		case CHECK_BUTTON_PASSED:
			break;
			
		case CHECK_BUTTON_RUNNING:
			break;
			
		case CHECK_BUTTON_FAILED:
			break;
	}
}


bool NissanAccButtonController::isHardwareConnectionGood() {
	return !checkButtonFailed;
}
