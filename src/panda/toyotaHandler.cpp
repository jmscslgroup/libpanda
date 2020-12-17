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

using namespace Panda;

ToyotaHandler::ToyotaHandler(Panda::Handler* handler) {
	this->pandaHandler = handler;
	
	decimatorHeartbeat = 0;
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
	
	// Start the heartbeats to wait for a command
	heartBeatSteer = TOYOTA_COMMAND_THREAD_RATE;
	heartBeatAcceleration = TOYOTA_COMMAND_THREAD_RATE;
}

void ToyotaHandler::entryAction() {
	pandaHandler->getUsb().setPowerSaveEnable(POWER_SAVE_STATUS_DISABLED);
}

void ToyotaHandler::doAction() {
	usleep(1000000.0/TOYOTA_COMMAND_THREAD_RATE);	// run at 1000 Hz
	
	// We cannot have integer rollovers here at anytime, so only increase if not already failed
	if(heartbeatAccelerationPass()) {
		heartBeatSteer++;
	}
	if (heartbeatAccelerationPass()) {
		heartBeatAcceleration++;
	}

	if (decimatorHeartbeat++ >= TOYOTA_DECIMATOR_MAX_HEARTBEAT) {	// 1Hz
		decimatorHeartbeat = 0;
		sendHeartBeat();
	}

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

bool ToyotaHandler::heartbeatSteeringPass() {
	return heartBeatSteer < TOYOTA_COMMAND_THREAD_RATE;	// Should fail at 1 second
}
bool ToyotaHandler::heartbeatAccelerationPass() {
	return heartBeatAcceleration < TOYOTA_COMMAND_THREAD_RATE;	// Should fail at 1 second
}

void ToyotaHandler::sendHeartBeat() {
	pandaHandler->getUsb().sendHeartBeat();
	
	// Libpanda should probably automatically do the following:
	pandaHandler->getUsb().getHealth(&health);	// this shoudl live in Panda::PandaHandler
	Panda::printPandaHealth(health);
}

void ToyotaHandler::sendLka() {
//	Panda::CanFrame frame = buildLdaAlert(hudLdaAlert || !heartbeatSteeringPass(), hudLeftLane, hudRightLane, hudBarrier);
	Panda::CanFrame frame = buildLkasHud(hudLdaAlert || !heartbeatSteeringPass(), hudLeftLane, hudRightLane, hudBarrier, hudTwoBeeps, hudRepeatedBeeps);
//	printf("LKAS_HUD: "); printFrame(frame);
	pandaHandler->getCan().sendMessage(frame);
}

void ToyotaHandler::sendTrackB() {
	counterTrackB += 2;	// I notice in recorded data that his incrmeents by 2
	counterTrackB |= 0x80;	// also the MSB is always 1.  perhaps the DBC is wrong?
	Panda::CanFrame frame = buildTRACK_B_1(counterTrackB);
	
	//printf("TRACK_B_1: "); printFrame(frame);
	pandaHandler->getCan().sendMessage(frame);
}

void ToyotaHandler::sendSteer() {
	int steerTorqueControlToSend;
	if (health.controls_allowed && heartbeatSteeringPass()) {
		steerRequest = true;
		steerLkaState = 0; // Real LKA steering control data shows this to be 0
		steerTorqueControlToSend = steerTorqueControl;	// Send the user steer torque
	} else {
		steerRequest = false;
		steerTorqueControlToSend = 0;	// This may not be needed, but is following the same convention as ACC control
	}
	
	Panda::CanFrame frame = buildSteeringLKA( counterSteer++, steerTorqueControlToSend, steerRequest, steerLkaState );
	//printf("STEERING_LKA at %d: ", (int) steer_torque); printFrame(frame);
	pandaHandler->getCan().sendMessage(frame);
}

void ToyotaHandler::sendAcc() {
	double accelerationControlToSend;
	if (health.controls_allowed && heartbeatAccelerationPass()) {
	
		permitBraking = true;
		releaseStandstill = true;
		miniCar = true;
		accelerationControlToSend = accelerationControl;	// Apply the user-set acceleration
		//		}
	} else {
		permitBraking = false;
		releaseStandstill = false;
		miniCar = false;
		accelerationControlToSend = 0.0;	// If we send non-0 in this state, curise controll will fault
	}

	Panda::CanFrame frame = buildACC_CONTROL( accelerationControlToSend, permitBraking, releaseStandstill, miniCar, cancelRequest );
	//printf("ACC_CONTROL at %02f: ", acc); printFrame(frame);
	pandaHandler->getCan().sendMessage(frame);
}


void ToyotaHandler::setHudLdaAlert( bool ldaAlert ) {
	if( this->hudLdaAlert != ldaAlert ) {
		this->hudLdaAlert = ldaAlert;
		decimatorLka = TOYOTA_DECIMATOR_MAX_LKA;	// Trigger an instant send
	}
}

void ToyotaHandler::setHudBarrier( bool barrier ) {
	if( this->hudBarrier != barrier) {
		this->hudBarrier = barrier;
		decimatorLka = TOYOTA_DECIMATOR_MAX_LKA;	// Trigger an instant send
	}
}

void ToyotaHandler::setHudLanes( unsigned char laneLeft, unsigned char laneRight ) {
	if( this->hudLeftLane != laneLeft ||
	  this->hudRightLane != laneRight ) {
		this->hudLeftLane = laneLeft;
		this->hudRightLane = laneRight;
		decimatorLka = TOYOTA_DECIMATOR_MAX_LKA;	// Trigger an instant send
	}
}

void ToyotaHandler::setHudCruiseCancelRequest( bool cancelRequest ) {
	this->cancelRequest = cancelRequest;
}

void ToyotaHandler::setHudTwoBeeps( bool twoBeeps ) {
	this->hudTwoBeeps = twoBeeps;
	
//	Panda::CanFrame frame = buildLkasHud(hudLdaAlert || !heartbeatSteeringPass(), hudLeftLane, hudRightLane, hudBarrier, hudTwoBeeps);
//	//printf("LKAS_HUD: "); printFrame(frame);
//	pandaHandler->getCan().sendMessage(frame);
}

void ToyotaHandler::setHudRepeatedBeeps( bool repeatedBeeps ) {
	if (this->hudRepeatedBeeps != repeatedBeeps) {
		this->hudRepeatedBeeps = repeatedBeeps;
		decimatorLka = TOYOTA_DECIMATOR_MAX_LKA;	// Trigger an instant send
	}
}

/* The comma.ai panda code has the following limits for steeerTorque:
const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
 */
void ToyotaHandler::setSteerTorque( int steerTorque ) {	// units are unknown, valid range is -1500:1500
	this->steerTorqueControl = steerTorque;
	heartBeatSteer = 0;
}

/* The comma.ai panda code has the following limits for acceleration:
const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s2
const int TOYOTA_MIN_ACCEL = -3000;       // -3.0 m/s2
 The following limits can be achieved by setting the panda into "unsafe" mode:
const int TOYOTA_ISO_MAX_ACCEL = 2000;        // 2.0 m/s2
const int TOYOTA_ISO_MIN_ACCEL = -3500;       // -3.5 m/s2

 DBC range states: -20:20
 */
void ToyotaHandler::setAcceleration( double acceleration ) {	// units are m/s2, valid range is -3.0:1.5
	this->accelerationControl = acceleration;
	heartBeatAcceleration = 0;
}
