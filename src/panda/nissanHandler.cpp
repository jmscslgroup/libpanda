/*
 Author: Matt Bunting
 */

#include "panda/nissan.h"

using namespace Panda;

NissanController::NissanController() {
	// Decimators:
	decimatorButton = 0;
	decimatorThreeOhThree = 0;
	
	// Attributes:
	buttonValue = 0;
	
	// Other Initialization:
	this->setIntervalActionRate(NISSAN_COMMAND_THREAD_RATE);
}

void NissanController::intervalAction() {
	if (decimatorButton++ > NISSAN_DECIMATOR_BUTTON) {
		decimatorButton = 0;
		sendCruiseButtons();
	}
	
	
	if (decimatorThreeOhThree++ > NISSAN_DECIMATOR_303) {
		decimatorThreeOhThree = 0;
//		sendThreeOhThree();
	}
}

void NissanController::sendCruiseButtons() {
	
}

void NissanController::sendThreeOhThree() {
	
}

void NissanController::newCanNotification(CanFrame* canFrame) {
	if (canFrame->messageID == 303) {
		
		
		int torque, speed;
		nissanParseThreeOhThree(*canFrame, &torque, &speed);
		
		if (canFrame->bus == 2 &&
			canFrame->dataLength == 12) {
			
			std::cout << "Got a valid 303 on bus 0, length 12, with torque " << torque << " speed " << speed << " <- replacing with 0s for bus 2" << std::endl;
			//		std::cout << "Got a valid 303, length 12, on bus 0 with torque " << torque << " speed " << speed << std::endl;
			
			CanFrame frameCopy = *canFrame;
			
			// Replacing the torque and speed data with 0:
			replaceCanThreeOhThree(&frameCopy, 0, 0);
			frameCopy.bus = 0;
			
			nissanParseThreeOhThree(frameCopy, &torque, &speed);
//			std::cout << " - Created message 303 for bus 2, length 12, with torque " << torque << " speed " << speed << std::endl;
			
			sendCan(frameCopy);
			
		} else if (canFrame->rejected ) {
			std::cout << " |- The attempt to send message 303 was REJECTED bus result: " << (int)canFrame->bus << std::endl;
			
		} else if (canFrame->returned) {
			std::cout << " |- The attempt to send message 303 was SUCCESS bus result: " << (int)canFrame->bus << " length:" << (int) canFrame->dataLength << std::endl;
		} else {
			std::cout << "Got a valid 303 on WRONG bus of " << (int)canFrame->bus << " length:" << (int) canFrame->dataLength << " with torque " << torque << " speed " << speed << std::endl;
			
		}
	}
}

void NissanController::handleSetSteerTorque( int steerTorque ) {
	
}

void NissanController::handleSetAcceleration( double acceleration ) {
	
}

bool NissanController::checkControlsAllowed(Panda::CanFrame* frame) {
//	return false;
	return true;	// HACK
}

void NissanController::sendButton( bool value ) {
	
}
