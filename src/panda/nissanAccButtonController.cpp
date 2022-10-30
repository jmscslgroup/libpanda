/*
 Author: Matt Bunting
 */

#include "panda/nissan.h"

using namespace Panda;

NissanAccButtonController::NissanAccButtonController() {
	
	// Decimators:
//	decimatorButton = 0;
//	decimatorThreeOhThree = 0;
	gasPressed = true;
//	brakePressed = false;
	cruiseState = 1;
	cruiseOn = false;
	// Attributes:
//	buttonValue = 0;
	
	// Other Initialization:
	this->setIntervalActionRate(NISSAN_COMMAND_THREAD_RATE);
	
//	relayHandler.open(GPIO_PIN_RELAY_POTENTIOMETER);
//	relayHandler.setDirection(GpioHandler::GPIO_DIRECTION_OUT);
//	relayHandler.setGpioOutput(GPIO_PIN_RELAY_POTENTIOMETER_DISARMED);
//
//	buzzerHandler.open(GPIO_PIN_BUZZER);
//	buzzerHandler.setDirection(GpioHandler::GPIO_DIRECTION_OUT);
//	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_OFF);
	
	state = ACC_STATE_OFF;
	enterState(ACC_STATE_OFF);
}

NissanAccButtonController::~NissanAccButtonController() {
//	relayHandler.setGpioOutput(GPIO_PIN_RELAY_POTENTIOMETER_DISARMED);
//	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_OFF);
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
			pedalTimerExpired = false;
			break;
			
		case ACC_STATE_SET_WAIT:
			decimatorSetWaitTimer = 0;
			relayHandler.arm();
			buzzerHandler.singleBeep();
			potHandler.pressButton(NISSAN_BUTTON_SET);
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
			break;
			
		case ACC_STATE_POWER_TOGGLE_NEEDED:
			break;
	}
	
	state = newState;
}

void NissanAccButtonController::exitState() {
	std::cout << "NissanAccButtonController: Exiting state " << stateToName(state) << std::endl;
	switch (state) {
		case ACC_STATE_OFF:
			break;
			
		case ACC_STATE_IDLE:
			break;
			
		case ACC_STATE_SET_WAIT:
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
			relayHandler.disarm();
			buzzerHandler.doubleBeep();
			break;
			
		case ACC_STATE_POWER_TOGGLE_NEEDED:
			break;
	}
}

void NissanAccButtonController::intervalAction() {
	if( decimatorPedalWaitTimer++ > NISSAN_DECIMATOR_PEDAL_WAIT ) {
		decimatorPedalWaitTimer = 0;
		// Timer expired, enter new state:
		if(state == ACC_STATE_IDLE) {
			enterState(ACC_STATE_SET_WAIT);
		}
	}

	if( decimatorSetWaitTimer++ > NISSAN_DECIMATOR_PEDAL_WAIT ) {
		if(state == ACC_STATE_SET_WAIT) {
			enterState(ACC_STATE_CONTROLS_ALLOWED);
		}
	}
}

void parseCruiseState(CanFrame* canFrame, unsigned char* cruiseState) {
	if (canFrame->messageID == 308) {	// fixme
		unsigned char CRUISE_STATE;
		bool CRUISE_ENGAGED;
		nissanParseCruise(*canFrame, &CRUISE_STATE, &CRUISE_ENGAGED);
		
		if (*cruiseState != CRUISE_STATE) {
			
			printf("parseCruiseState(): Change detected: Message 308: CRUISE_STATE = %d, CRUISE_ENGAGED = %d\n", (int)CRUISE_STATE, CRUISE_ENGAGED);
		}
		
		*cruiseState = CRUISE_STATE;
	}
}

void parseGasPressed(CanFrame* canFrame, bool* gasPressed) {
	if (canFrame->messageID == 140) {	// fixme
		int ACCEL_PEDAL_POSITION;
		nissanPedalThrottle(*canFrame, &ACCEL_PEDAL_POSITION);
		if (*gasPressed != (ACCEL_PEDAL_POSITION != 0)) {
			
			printf("parseCruiseState(): Change detected: ACCEL_PEDAL_POSITION = %d\n", ACCEL_PEDAL_POSITION);
		}
		*gasPressed = ACCEL_PEDAL_POSITION != 0;
	}
}

//void parseBrakePressed(CanFrame* canFrame, bool* brakePressed) {
//	if (canFrame->messageID == 303) {	// fixme
//
//	}
//}
#define NISSAN_CRUISE_STATE_OFF (1)
#define NISSAN_CRUISE_STATE_IDLE (13)
#define NISSAN_CRUISE_STATE_ACTIVE (25)

void NissanAccButtonController::updateStateVariables(CanFrame* canFrame) {
	parseCruiseState(canFrame, &cruiseState);
	cruiseOn = (cruiseState != NISSAN_CRUISE_STATE_OFF);
	parseGasPressed(canFrame, &gasPressed);
//	parseBrakePressed(canFrame, &brakePressed);
	
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
			if ( gasPressed ) {	// brake press?
				decimatorPedalWaitTimer = 0;
			}
			break;
			
		case ACC_STATE_CONTROLS_ALLOWED:
			if ( cruiseState == NISSAN_CRUISE_STATE_IDLE ) {	// BRAKE WAS PRESSED
				printf("NissanAccButtonController: Entered ACC IDLE while controls_allowed, perhaps brake was pressed?\n");
				enterState(ACC_STATE_POWER_TOGGLE_NEEDED);
			}
			if ( gasPressed ) {
				enterState(ACC_STATE_IDLE);
			}
			break;
			
		case ACC_STATE_POWER_TOGGLE_NEEDED:
			break;
	}
	
//
//	if (canFrame->messageID == 303) {
//
//		// 1119: ACC_BTNS
//		//BO_ 1119 WHEEL_BUTTONS: 20 XXX
//		// SG_ SIGNAL1 : 48|3@0+ (1,0) [0|1] "" XXX
//		// SG_ LKAS_LDW_ON : 55|1@0+ (1,0) [0|1] "" XXX
//		// SG_ ACC_BTNS : 61|3@0+ (1,0) [0|1] "" XXX
//
//		/*
//		BO_ 308 CRUISE: 64 XXX
//		 SG_ CRUISE_TORQUE_STATE : 57|4@0+ (1,0) [0|16] "" XXX
//		 SG_ WHEEL_TORQUE_CMD : 68|12@0+ (1,0) [0|4000] "" XXX
//		 SG_ STEER_ANGLE_BUT_NOT : 151|16@0+ (1,0) [0|40000] "" XXX
//		 SG_ BRAKE_TORQUE_ACTIVE : 160|1@0+ (1,0) [0|1] "" XXX
//		 SG_ BRAKE_TORQUE_CMD : 170|10@0+ (1,0) [0|1000] "" XXX
//		 SG_ CRUISE_STATE : 306|5@0+ (1,0) [0|32] "" XXX
//		 SG_ CRUISE_ENGAGED : 321|1@0+ (1,0) [0|1] "" XXX
//		 */
//
//		/*
//		 BO_ 548 BRAKE: 7 XXX
//		  SG_ BRAKE_STATE : 7|5@0+ (1,0) [0|1024] "" XXX
//		  SG_ BRAKE_PEDAL : 2|10@0+ (1,0) [0|1024] "" XXX
//		 */
//
//
//		int torque, speed;
////		nissanParseThreeOhThree(*canFrame, &torque, &speed);
//
//		if (canFrame->bus == 2 &&
//			canFrame->dataLength == 12) {
//
////			std::cout << "Got a valid 303 on bus 0, length 12, with torque " << torque << " speed " << speed << " <- replacing with 0s for bus 2" << std::endl;
////			//		std::cout << "Got a valid 303, length 12, on bus 0 with torque " << torque << " speed " << speed << std::endl;
////
////			CanFrame frameCopy = *canFrame;
////
////			// Replacing the torque and speed data with 0:
////			replaceCanThreeOhThree(&frameCopy, 0, 0);
////			frameCopy.bus = 0;
////
////			nissanParseThreeOhThree(frameCopy, &torque, &speed);
//////			std::cout << " - Created message 303 for bus 2, length 12, with torque " << torque << " speed " << speed << std::endl;
////
////			//sendCan(frameCopy);
////
////		} else if (canFrame->rejected ) {
////			std::cout << " |- The attempt to send message 303 was REJECTED bus result: " << (int)canFrame->bus << std::endl;
////
////		} else if (canFrame->returned) {
////			std::cout << " |- The attempt to send message 303 was SUCCESS bus result: " << (int)canFrame->bus << " length:" << (int) canFrame->dataLength << std::endl;
////		} else {
////			std::cout << "Got a valid 303 on WRONG bus of " << (int)canFrame->bus << " length:" << (int) canFrame->dataLength << " with torque " << torque << " speed " << speed << std::endl;
////
//		}
//	}
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
