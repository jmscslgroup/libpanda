/*
 Author: Matt Bunting
 Copyright (c) 2020 Arizona Board of Regents
 All rights reserved.

 Permission is hereby granted, without written agreement and without
 license or royalty fees, to use, copy, modify, and distribute this
 software and its documentation for any purpose, provided that the
 above copyright notice and the following two paragraphs appear in
 all copies of this software.

 IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
 FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
 ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
 IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 SUCH DAMAGE.

 THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
 IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
 TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 */

#include "panda/toyota.h"
#include <unistd.h>
//#include <chrono>

using namespace Panda;

//ToyotaHandler::ToyotaHandler(Panda::Handler* handler) {
ToyotaHandler::ToyotaHandler() {
	// TODO: should reorganize this, or have some assertion that handler is valid.
//	this->pandaHandler = handler;
	
//	controls_allowed_prior = false;
//	controls_allowed = false;
	controls_allowed_delay_counter = 0;
	
	decimatorHeartbeat = 0;
//	decimatorControlsAllowed = 0;
	decimatorLka = 0;
	decimatorTrackB = 0;
	decimatorSteer = 0;
	decimatorAcc = 0;
	
	counterSteer = 0;
	counterTrackB = 0;
	
	// Default HUD:
	hudLdaAlert = false;
	hudBarrier = false;
	hudLeftLane = 2;
	hudRightLane = 2;
	hudTwoBeeps = 0;
	
	// Default Steer:
	steerTorqueControl = 0.0;
	steerRequest = false;
	steerLkaState = 0;
	
	// Default Acceleration:
	accelerationControl = 0.0;
	permitBraking = false;
	releaseStandstill = false;
	miniCar = false;
	cancelRequest = false;
	
	// CAN states:
	gas_released = false;
	brake_pressed = false;
	
	// Start the heartbeats to wait for a command
//	heartBeatSteer = TOYOTA_COMMAND_THREAD_RATE;
//	heartBeatAcceleration = TOYOTA_COMMAND_THREAD_RATE;
	
	this->setIntervalActionRate(TOYOTA_COMMAND_THREAD_RATE);
	
//	if (pandaHandler != NULL) {
//		pandaHandler->getCan().addObserver(this);
//	}
	
//	mHeartbeatHelper = new HeartbeatHelper(this);
//	mHeartbeatHelper->start();
}

//ToyotaHandler::~ToyotaHandler() {
////	mHeartbeatHelper->stop();
////	delete mHeartbeatHelper;
//}

//void ToyotaHandler::newDataNotification(CanFrame* canFrame) {
bool ToyotaHandler::checkControlsAllowed(CanFrame* canFrame) {
	if (canFrame->messageID == 466) {
		//printf("Got message 466:\n");
		gas_released = ((*(unsigned long*)canFrame->data) >> (4+1-1)) & 0x01;
		//printf(" - 466: GAS_RELEASED: %d\n", gas_released);
		cruise_state = ((*(unsigned long*)canFrame->data) >> (55+1-4)) & 0x0F;
		// cruise_active: if cuirse controller is on in general
		// cruise state: nonAdaptive if value is 1, 2, 3, 4, 5, 6
//		car_cruise_ready_for_commands = (cruise_state & 0x08) >> 3;
		
//		if (!gas_released && controls_allowed) {
		if (!gas_released && getControlsAllowed()) {
			setHudCruiseCancelRequest( true );
		}
//		controls_allowed = cruise_state >= 8 && gas_released;

//		printf(" - 466: CRUISE_STATE: %d\n", cruise_state);
	}
//	if (canFrame->messageID == 166) {	// brake
////		printf("Got message 166:\n");
//		int BRAKE_AMOUNT = ((*(unsigned long*)canFrame->data) >> (7+1-8)) & 0xFF;
////		printf(" - 166: BRAKE_AMOUNT: %d\n", BRAKE_AMOUNT);
//	}
	if (canFrame->messageID == 550) {	// brake
//		printf("Got message 166:\n");
		brake_pressed = ((*(unsigned long*)canFrame->data) >> (37+1-1)) & 0x01;
//		printf(" - 550: BRAKE_PRESSED: %d\n", brake_pressed);
//		if (brake_pressed && controls_allowed) {
////			if(controls_allowed) {
////				setHudCruiseCancelRequest( true );
////			}
//			controls_allowed = false;
//		}
	}
//	if (canFrame->messageID == 560) {	// brake
////		printf("Got message 166:\n");
//		bool BRAKE_PRESSED = ((*(unsigned long*)canFrame->data) >> (26+1-1)) & 0x01;
////		printf(" - 560: BRAKE_PRESSED: %d\n", BRAKE_PRESSED);
//	}
//	if (canFrame->messageID == 921) {
////		printf("Got message 921:\n");
//		int cruise_control_state = ((*(unsigned long*)canFrame->data) >> (11+1-4)) & 0x0F;
////		printf(" - 921: CRUISE_CONTROL_STATE: %d\n", cruise_control_state);
//	}
	
//	controls_allowed_prior = controls_allowed;
//	controls_allowed = (cruise_state >= 8) && gas_released && !brake_pressed;
	
	return (cruise_state >= 8) && gas_released && !brake_pressed;
	
//	if (controls_allowed_prior != controls_allowed) {
//		for (std::vector<ToyotaListener*>::iterator it = toyotaObservers.begin();
//			 it != toyotaObservers.end();
//			 it++) {
//			(*it)->newControlNotification(this);
//		}
//	}
//
////	fprintf(stderr, "cruise_state, gas_released, !brake_pressed = %d, %d, %d\n", cruise_state,  gas_released, !brake_pressed);
}

//void ToyotaHandler::entryAction() {
//	std::cout << "In ToyotaHandler::entryAction():" << std::endl;
//
//	std::cout << " - Setting power save to POWER_SAVE_STATUS_DISABLED:" << std::endl;
//	pandaHandler->getUsb().setPowerSaveEnable(POWER_SAVE_STATUS_DISABLED);
//
//	std::cout << " - Setting Safety to SAFETY_TOYOTA" << std::endl;
//	/* For the params setting see the code in comma.ai/panda/board/safety/safety_toyota.h:
//	 // safety param flags
//	 // first byte is for eps factor, second is for flags
//	 const uint32_t TOYOTA_PARAM_OFFSET = 8U;
//	 const uint32_t TOYOTA_EPS_FACTOR = (1U << TOYOTA_PARAM_OFFSET) - 1U;
//	 const uint32_t TOYOTA_PARAM_ALT_BRAKE = 1U << TOYOTA_PARAM_OFFSET;
//	 const uint32_t TOYOTA_PARAM_STOCK_LONGITUDINAL = 2U << TOYOTA_PARAM_OFFSET;
//
//	 // The value of 73 is based on car controller code in openpilot for Toyotas
//	*/
//	pandaHandler->getUsb().setSafetyMode(SAFETY_TOYOTA, 73);
//
//}

//void ToyotaHandler::exitAction() {
//	std::cout << "In ToyotaHandler::exitAction():" << std::endl;
//
//	std::cout << " - Setting Safety to SAFETY_NOOUTPUT:" << std::endl;
//	pandaHandler->getUsb().setSafetyMode(SAFETY_NOOUTPUT, 0);
//
//	std::cout << " - Setting power save to POWER_SAVE_STATUS_ENABLED:" << std::endl;
//	pandaHandler->getUsb().setPowerSaveEnable(POWER_SAVE_STATUS_ENABLED);
//
//}

//void ToyotaHandler::doAction() {
void ToyotaHandler::intervalAction() {

	if (hudTwoBeeps) {
		decimatorLka += 20;	//  increase rate to ensure HUD responds
	}
	if (decimatorLka++ >= TOYOTA_DECIMATOR_MAX_LKA) {	// 1Hz
		decimatorLka = 0;
		sendLka();
	}
	
	if (decimatorTrackB++ >= TOYOTA_DECIMATOR_MAX_TRACK_B) {	// 40Hz
		decimatorTrackB = 0;
		sendTrackB();
	}

	if(decimatorSteer++ >= TOYOTA_DECIMATOR_MAX_STEER) { // 100Hz
		decimatorSteer = 0;
		sendSteer();
	}
	
	if (decimatorAcc++ >= TOYOTA_DECIMATOR_MAX_ACC) {	// 30 Hz
		decimatorAcc = 0;
		sendAcc();
	}

}


void ToyotaHandler::sendLka() {
//	Panda::CanFrame frame = buildLdaAlert(hudLdaAlert || !heartbeatSteeringPass(), hudLeftLane, hudRightLane, hudBarrier);
	bool hudRepeatedBeepsToSend = hudRepeatedBeeps;
	
	if ((!heartbeatSteeringPass() || !heartbeatAccelerationPass()) &&
//		health.controls_allowed ) {
		getControlsAllowed() ) {
		hudRepeatedBeepsToSend = true;
	}
	
	bool hudLkaAlertToSend = hudLdaAlert;
//	if (health.controls_allowed && !heartbeatSteeringPass()) {
	if (getControlsAllowed() && !heartbeatSteeringPass()) {
		hudLkaAlertToSend = true;
	}
	
	Panda::CanFrame frame = buildLkasHud(hudLkaAlertToSend, hudLeftLane, hudRightLane, hudBarrier, hudTwoBeeps, hudRepeatedBeepsToSend);
//	printf("LKAS_HUD: "); printFrame(frame);
//	pandaHandler->getCan().sendMessage(frame);
	sendCan(frame);
}

void ToyotaHandler::sendTrackB() {
	counterTrackB += 2;	// I notice in recorded data that his increments by 2
	counterTrackB |= 0x80;	// also the MSB is always 1.  perhaps the DBC is wrong?
	Panda::CanFrame frame = buildTRACK_B_1(counterTrackB);
	
	//printf("TRACK_B_1: "); printFrame(frame);
//	pandaHandler->getCan().sendMessage(frame);
	sendCan(frame);
}

void ToyotaHandler::sendSteer() {
	int steerTorqueControlToSend;
	// Only send torque if both 1. controls are allowed and 2. the heartbeat passes.
	// If either of the above two fail, then send 0 torque
//	if (health.controls_allowed && heartbeatSteeringPass()) {
	if (getControlsAllowed() && heartbeatSteeringPass()) {
		steerRequest = true;
		steerLkaState = 0; // Real LKA steering control data shows this to be 0
		steerTorqueControlToSend = steerTorqueControl;	// Send the user steer torque
	} else {
		steerRequest = false;
		steerTorqueControlToSend = 0;	// This may not be needed, but is following the same convention as ACC control
	}
	
	Panda::CanFrame frame = buildSteeringLKA( counterSteer++, steerTorqueControlToSend, steerRequest, steerLkaState );
	//printf("STEERING_LKA at %d: ", (int) steer_torque); printFrame(frame);
//	pandaHandler->getCan().sendMessage(frame);
	sendCan(frame);
}

void ToyotaHandler::sendAcc() {
	double accelerationControlToSend;
	// Only send acceleration if both 1. controls are allowed and 2. the heartbeat passes.
	// If either of the above two fail, then send 0 acceleration
//	if (health.controls_allowed && heartbeatAccelerationPass() && this->gas_released && !this->brake_pressed) {
	if (getControlsAllowed() && heartbeatAccelerationPass() ) {//}&& car_cruise_ready_for_commands) {
		permitBraking = true;
		releaseStandstill = true;
		if (controls_allowed_delay_counter < TOYOTA_RATE_ACC) {	// Delay 1 second until commands are sent
			controls_allowed_delay_counter++;
			accelerationControlToSend = 0.0;
		} else {
			accelerationControlToSend = accelerationControl;	// Apply the user-set acceleration
		}
	} else {
		permitBraking = false;
		releaseStandstill = false;
		accelerationControlToSend = 0.0;	// If we send non-0 in this state, cruise control will fault
		controls_allowed_delay_counter = 0;
	}
	
//	// The following will send a cancel request if controls are allowed and the heartbeat fails, but
//	// I think I prefer the version without this logic.  Leaving here for future discussion
	bool cancelRequestToSend = cancelRequest;
//	if(!heartbeatAccelerationPass() && health.controls_allowed) {
//		cancelRequestToSend = true;
//	}
//	cancelRequestSpamCount = 0;
	if (cancelRequest ) {
		if (cancelRequestSpamCount < (TOYOTA_RATE_ACC * 3)) {	// 3 second spam
			cancelRequestSpamCount++;
		} else {
			cancelRequest = false;
			cancelRequestSpamCount = 0;
		}
	}
		
	Panda::CanFrame frame = buildACC_CONTROL( accelerationControlToSend, permitBraking, releaseStandstill, miniCar, cancelRequestToSend );
//	fprintf(stderr, "ACC_CONTROL at %02f, ACC_Input at %02f\n", accelerationControlToSend, accelerationControl); //printFrame(frame);
//	pandaHandler->getCan().sendMessage(frame);
	sendCan(frame);
}

void ToyotaHandler::triggerInstantLkaSend() {
	decimatorLka = TOYOTA_DECIMATOR_MAX_LKA;	// Trigger an instant send by maxing the decimator
}


void ToyotaHandler::setHudLdaAlert( bool enable ) {
	if( this->hudLdaAlert != enable ) {
		this->hudLdaAlert = enable;
		triggerInstantLkaSend();
	}
}

void ToyotaHandler::setHudBarrier( bool enable ) {
	if( this->hudBarrier != enable) {
		this->hudBarrier = enable;
		triggerInstantLkaSend();
	}
}

void ToyotaHandler::setHudLanes( unsigned char laneLeft, unsigned char laneRight ) {
	if( this->hudLeftLane != laneLeft ||
	  this->hudRightLane != laneRight ) {
		this->hudLeftLane = laneLeft;
		this->hudRightLane = laneRight;
		triggerInstantLkaSend();
	}
}

void ToyotaHandler::setHudCruiseCancelRequest( bool enable ) {
	if (enable) {
		this->cancelRequestSpamCount = 0;
		this->cancelRequest = true;
	}
}

void ToyotaHandler::setHudTwoBeeps( bool enable ) {
	this->hudTwoBeeps = enable;
}

void ToyotaHandler::setHudRepeatedBeeps( bool enable ) {
	if (this->hudRepeatedBeeps != enable) {
		this->hudRepeatedBeeps = enable;
		triggerInstantLkaSend();
	}
}

void ToyotaHandler::setHudMiniCar( bool enable ) {
	this->miniCar = enable;
}

/* The comma.ai panda code has the following limits for steeerTorque:
const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
 */
void ToyotaHandler::handleSetSteerTorque( int steerTorque ) {	// units are unknown, valid range is -1500:1500
	this->steerTorqueControl = steerTorque;
//	heartBeatSteer = 0;
}

/* The comma.ai panda code has the following limits for acceleration:
const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s2
const int TOYOTA_MIN_ACCEL = -3000;       // -3.0 m/s2
 The following limits can be achieved by setting the panda into "unsafe" mode:
const int TOYOTA_ISO_MAX_ACCEL = 2000;        // 2.0 m/s2
const int TOYOTA_ISO_MIN_ACCEL = -3500;       // -3.5 m/s2

 DBC range states: -20:20
 */
void ToyotaHandler::handleSetAcceleration( double acceleration ) {	// units are m/s2, valid range is -3.0:1.5
	this->accelerationControl = acceleration;
//	heartBeatAcceleration = 0;
}

//bool ToyotaHandler::getIgnitionOn() {
//	return health.ignition_line;
//}

//bool ToyotaHandler::getControlsAllowed() {
//	return health.controls_allowed;
////	return controls_allowed;
//}

//bool ToyotaHandler::getPandaControlsAllowed() {
//	return controls_allowed;
//}

//bool ToyotaHandler::getCarCruiseReadyForCommands() {
//	return car_cruise_ready_for_commands;
//}

unsigned char ToyotaHandler::getCarCruiseState() {
	return cruise_state;
}

//const PandaHealth& ToyotaHandler::getPandaHealth() const {
//	return health;
//}

//void ToyotaHandler::addObserver( ToyotaListener* observer ) {
//	toyotaObservers.push_back(observer);
//}
