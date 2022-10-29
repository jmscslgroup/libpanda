/*
 Author: Matt Bunting
 */

#include "panda/nissan.h"

using namespace Panda;

NissanAccButtonController::NissanAccButtonController() {
	state = ACC_STATE_IDLE;
	// Decimators:
//	decimatorButton = 0;
//	decimatorThreeOhThree = 0;
	
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
}

NissanAccButtonController::~NissanAccButtonController() {
//	relayHandler.setGpioOutput(GPIO_PIN_RELAY_POTENTIOMETER_DISARMED);
//	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_OFF);
	relayHandler.disarm();
}

void NissanAccButtonController::intervalAction() {
//	if (decimator++ > NISSAN_DECIMATOR_BUTTON) {
//		decimatorButton = 0;
//		sendCruiseButtons();
//	}
//
//
//	if (decimatorThreeOhThree++ > NISSAN_DECIMATOR_303) {
//		decimatorThreeOhThree = 0;
////		sendThreeOhThree();
//	}
	
	if( decimatorWaitTimer++ > 100 ) {
		decimatorWaitTimer = 0;
	}
}


void NissanAccButtonController::newCanNotification(CanFrame* canFrame) {
	if (canFrame->messageID == 303) {
		
		// 1119: ACC_BTNS
		//BO_ 1119 WHEEL_BUTTONS: 20 XXX
		// SG_ SIGNAL1 : 48|3@0+ (1,0) [0|1] "" XXX
		// SG_ LKAS_LDW_ON : 55|1@0+ (1,0) [0|1] "" XXX
		// SG_ ACC_BTNS : 61|3@0+ (1,0) [0|1] "" XXX
		
		/*
		BO_ 308 CRUISE: 64 XXX
		 SG_ CRUISE_TORQUE_STATE : 57|4@0+ (1,0) [0|16] "" XXX
		 SG_ WHEEL_TORQUE_CMD : 68|12@0+ (1,0) [0|4000] "" XXX
		 SG_ STEER_ANGLE_BUT_NOT : 151|16@0+ (1,0) [0|40000] "" XXX
		 SG_ BRAKE_TORQUE_ACTIVE : 160|1@0+ (1,0) [0|1] "" XXX
		 SG_ BRAKE_TORQUE_CMD : 170|10@0+ (1,0) [0|1000] "" XXX
		 SG_ CRUISE_STATE : 306|5@0+ (1,0) [0|32] "" XXX
		 SG_ CRUISE_ENGAGED : 321|1@0+ (1,0) [0|1] "" XXX
		 */
		
		/*
		 BO_ 548 BRAKE: 7 XXX
		  SG_ BRAKE_STATE : 7|5@0+ (1,0) [0|1024] "" XXX
		  SG_ BRAKE_PEDAL : 2|10@0+ (1,0) [0|1024] "" XXX
		 */
		
		
		int torque, speed;
//		nissanParseThreeOhThree(*canFrame, &torque, &speed);
		
		if (canFrame->bus == 2 &&
			canFrame->dataLength == 12) {
			
//			std::cout << "Got a valid 303 on bus 0, length 12, with torque " << torque << " speed " << speed << " <- replacing with 0s for bus 2" << std::endl;
//			//		std::cout << "Got a valid 303, length 12, on bus 0 with torque " << torque << " speed " << speed << std::endl;
//
//			CanFrame frameCopy = *canFrame;
//
//			// Replacing the torque and speed data with 0:
//			replaceCanThreeOhThree(&frameCopy, 0, 0);
//			frameCopy.bus = 0;
//
//			nissanParseThreeOhThree(frameCopy, &torque, &speed);
////			std::cout << " - Created message 303 for bus 2, length 12, with torque " << torque << " speed " << speed << std::endl;
//
//			//sendCan(frameCopy);
//
//		} else if (canFrame->rejected ) {
//			std::cout << " |- The attempt to send message 303 was REJECTED bus result: " << (int)canFrame->bus << std::endl;
//
//		} else if (canFrame->returned) {
//			std::cout << " |- The attempt to send message 303 was SUCCESS bus result: " << (int)canFrame->bus << " length:" << (int) canFrame->dataLength << std::endl;
//		} else {
//			std::cout << "Got a valid 303 on WRONG bus of " << (int)canFrame->bus << " length:" << (int) canFrame->dataLength << " with torque " << torque << " speed " << speed << std::endl;
//
		}
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

void NissanAccButtonController::sendButton( bool value ) {
	
}
