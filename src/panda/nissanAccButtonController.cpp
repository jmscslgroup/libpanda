/*
 Author: Matt Bunting
 */

#include "panda/nissan.h"

using namespace Panda;

NissanAccButtonController::NissanAccButtonController() {
	
	// State Variables:
	gasPressed = true;
//	brakePressed = false;
	cruiseState = NISSAN_CRUISE_STATE_OFF;
	cruiseOn = false;
	cruiseEngaged = false;
	
	// Other Initialization:
	this->setIntervalActionRate(NISSAN_COMMAND_THREAD_RATE);
	
	state = ACC_STATE_OFF;
	//enterState(ACC_STATE_OFF);
}

NissanAccButtonController::~NissanAccButtonController() {
	relayHandler.disarm();
}

const char* NissanAccButtonController::stateToName( const AccCommandState& state) {
	switch (state) {
		case ACC_STATE_OFF: return "ACC_STATE_OFF";
		case ACC_STATE_IDLE: return "ACC_STATE_IDLE";
		case ACC_STATE_SET_WAIT: return "ACC_STATE_SET_WAIT";
		case ACC_STATE_CONTROLS_ALLOWED: return "ACC_STATE_CONTROLS_ALLOWED";
		case ACC_STATE_POWER_TOGGLE_NEEDED: return "ACC_STATE_POWER_TOGGLE_NEEDED";
	}
	return "";
}

void NissanAccButtonController::enterState( AccCommandState newState ) {
	exitState();
	std::cout << "NissanAccButtonController: Entering state " << stateToName(newState) << std::endl;
	switch (newState) {
		case ACC_STATE_OFF:
			break;
			
		case ACC_STATE_IDLE:
			decimatorPedalWaitTimer = 0;
			break;
			
		case ACC_STATE_SET_WAIT:
			buzzerHandler.singleBeep();
			decimatorSetWaitTimer = 0;
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
			break;
			
		case ACC_STATE_POWER_TOGGLE_NEEDED:
			potHandler.pressButton(NISSAN_BUTTON_OFF);	// Just incase
			relayHandler.disarm();
			buzzerHandler.doubleBeep();
			break;
	}
	
	state = newState;
}

void NissanAccButtonController::exitState() {
	switch (state) {
		case ACC_STATE_OFF:
			break;
			
		case ACC_STATE_IDLE:
			break;
			
		case ACC_STATE_SET_WAIT:
			relayHandler.arm();
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
			break;
			
		case ACC_STATE_POWER_TOGGLE_NEEDED:
			break;
	}
}

void NissanAccButtonController::intervalAction() {
	switch (state) {
		case ACC_STATE_OFF:
			break;
			
		case ACC_STATE_IDLE:
//			if ( gasPressed ) {	// brake press?
//				decimatorPedalWaitTimer = 0;
//			} else if( decimatorPedalWaitTimer++ > NISSAN_DECIMATOR_PEDAL_WAIT ) {
//				decimatorPedalWaitTimer = 0;
//				// Timer expired, enter new state:
//				enterState(ACC_STATE_SET_WAIT);
//			}
			break;
			
		case ACC_STATE_SET_WAIT:
			if( decimatorSetWaitTimer++ > NISSAN_DECIMATOR_PEDAL_WAIT ) {
				// Timer expired, enter new state:
				enterState(ACC_STATE_CONTROLS_ALLOWED);
			}
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
			break;
			
		case ACC_STATE_POWER_TOGGLE_NEEDED:
			enterState(ACC_STATE_IDLE);
			break;
	}
}

void parseCruiseState(CanFrame* canFrame, unsigned char* cruiseState, bool* cruiseEngaged) {
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
			printf("parseCruiseState(): CAN change detected: Message 308: cruiseEngaged = %d (Currently unused)\n", *cruiseEngaged);
		}
		
	}
}

void parseGasPressed(CanFrame* canFrame, bool* gasPressed) {
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

void parseAccButtons(CanFrame* canFrame, NissanButton* buttonState) {
	if (canFrame->messageID == 1119) {
		unsigned char ACC_BTNS = *buttonState;
		nissanParseWheelButtons( *canFrame, &ACC_BTNS  );
		if (*buttonState != ACC_BTNS) {
			*buttonState = (NissanButton)ACC_BTNS;
			
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
	
	// No matter the state, if cruise was turned off then enter that state:
	if (!cruiseOn &&
		state != ACC_STATE_OFF) {
		enterState(ACC_STATE_OFF);
	}
	
	switch (state) {
		case ACC_STATE_OFF:
			if (cruiseOn) {
				enterState(ACC_STATE_IDLE);
			}
			break;
			
		case ACC_STATE_IDLE:
//			if (buttonState == NISSAN_BUTTON_SET ||
//				buttonState == NISSAN_BUTTON_RES) {
//				enterState(ACC_STATE_SET_WAIT);
//			}
			if (cruiseEngaged == 1) {
				enterState(ACC_STATE_SET_WAIT);
			}
			break;
			
		case ACC_STATE_SET_WAIT:
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
//			if ( cruiseState == NISSAN_CRUISE_STATE_IDLE ) {	// BRAKE WAS PRESSED
//				printf("NissanAccButtonController: Entered ACC IDLE while controls_allowed, perhaps brake was pressed?\n");
//				enterState(ACC_STATE_POWER_TOGGLE_NEEDED);
//			}
//			if ( gasPressed ) {
//				enterState(ACC_STATE_SET_WAIT);
//			}
			if (cruiseEngaged == 0) {
				enterState(ACC_STATE_POWER_TOGGLE_NEEDED);
			}
			break;
			
		case ACC_STATE_POWER_TOGGLE_NEEDED:
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
	if (state == ACC_STATE_CONTROLS_ALLOWED) {
		printf("Sending a button! %d\n", (int)button);
		potHandler.pressButton(button);
		return 1;
	}
	return 0;
}
