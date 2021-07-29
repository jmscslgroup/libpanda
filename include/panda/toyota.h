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

#ifndef TOYOTA_H
#define TOYOTA_H

#include <vector>
#include "mogi/thread.h"

#include "panda.h"


#define TIME_HEARTBEAT_FAIL_STEERING (1.0)		// In seconds, time until a heartbeat fails from not receiving a new steering command
#define TIME_HEARTBEAT_FAIL_ACCELERATION (1.0)	// In seconds, time until a heartbeat fails from not receiving a new acceleration command

#define TOYOTA_COMMAND_THREAD_RATE (600.0)	// Defines the rate of the thread, not for any particular command to be sent.
#define TOYOTA_RATE_HEARTBEAT (1.0)			// This is for the panda in general, not Toyota specific
#define TOYOTA_RATE_LKA (1.0)				// Rate of the LKAS_HUD command
#define TOYOTA_RATE_TRACK_B (40.0)			// Rate of the TRACK_B_1 command
#define TOYOTA_RATE_STEER (100.0)			// Rate of the STEERING_LKA command
#define TOYOTA_RATE_ACC (30.0)				// Rate of the ACC_CONTROL command

#define TOYOTA_DECIMATOR_MAX_HEARTBEAT (TOYOTA_COMMAND_THREAD_RATE/TOYOTA_RATE_HEARTBEAT)
#define TOYOTA_DECIMATOR_MAX_LKA (TOYOTA_COMMAND_THREAD_RATE/TOYOTA_RATE_LKA)
#define TOYOTA_DECIMATOR_MAX_TRACK_B (TOYOTA_COMMAND_THREAD_RATE/TOYOTA_RATE_TRACK_B)
#define TOYOTA_DECIMATOR_MAX_STEER (TOYOTA_COMMAND_THREAD_RATE/TOYOTA_RATE_STEER)
#define TOYOTA_DECIMATOR_MAX_ACC (TOYOTA_COMMAND_THREAD_RATE/TOYOTA_RATE_ACC)

namespace Panda {

/*!
 \brief Constructs the LKAS_HUS command that works on a Toyota RAV4.
 \param lkaAlert Will invoke a "Please grab steering wheel" notification on the car HUD.
 \param leftlane Can be 0-3, and will show different left lane visuals on the HUD.
 \param rightlane Can be 0-3, and will show different right lane visuals on the HUD.
 \param barrier Will display a barrier on the right and left lanes.
 \param twoBeeps Will cause a single two-beep audible alert in the car.  If called too frequently then it may not trigger.
 \param repeatedBeeps Will cause a continuous audible beeping alert.
 \return A constructed LKA_HUD CanFrame
 */
CanFrame buildLkasHud(bool lkaAlert, unsigned char leftLane, unsigned char rightLane, bool barrier, bool twoBeeps, bool repeatedBeeps);

/*!
 \brief Constructs the STEERING_LKA command, used for sending steering torque.
 \param count This needs to increase by 1 on each send, probably for error checking.
 \param steerTorque In unknown units, but is valid from -1500:1500.
 \param steerRequest Should be set to 1 when a steering torque is sent.  This is shown by CAN reading of the built-in LKA.
 \param lkaState Unknown, labeled in the DBC.  Stays 0 based on CAN data.
 \return A constructed STEERING_LKA CanFrame.
 */
CanFrame buildSteeringLKA( unsigned char count, int16_t steerTorque, bool steerRequest, unsigned char lkaState );

/*!
 \brief Constructs the ACC_CONTROL command, used for sending cruise cntrol accelerations.
 \param permitBraking Unsure of purpose outside of name.  Should be 1 when sending control commands, perhaps.
 \param releaseStandstill Unsure of purpose outside of name.  Should be 1 when sending control commands when car has been commanded to stop and is desired to let the car continue to be controlle dout of a stop, perhaps.
 \param miniCar Will display the "Mini Car" on the HUD, but only works when the cruise control is activated and operating.
 \param cancelRequest Will cancel the cruise controller, informing the driver to regain control of the vehicle.
 \return A constructed ACC_CONTROL CanFrame.
 */
CanFrame buildACC_CONTROL(double acc, bool permitBraking, bool releaseStandstill, bool miniCar, bool cancelRequest);

/*!
 \brief Constructs the TRACK_B_1 command, needed to fake adaptive cruise controller operation.
 Note that there could be more things to fake in this command, but for use of ACC_CONTROL this command is intercepted to prevent run-time faults of the ACC.
 \param count  Needs to be increment on each call.  It may be only 6 bits long even though the DBC states 8-bits.  See ToyotaHandler for how this is handeled.
 \return A constructed TRACK_B_1 CanFrame.
 */
CanFrame buildTRACK_B_1(unsigned char count);

/*!
 \brief Unused by ToyotaHandler.  Also untested.
 */
CanFrame buildPCM_CRUISE_2(unsigned char SET_SPEED);

/*!
 \brief Unused by ToyotaHandler.  Also untested.
 */
CanFrame buildDSU_CRUISE(unsigned char SET_SPEED);

/*!
 \brief Computes particular checksums within the CAN message, and is not the CRC for the CAN frame itself
 \param frame The frame for the checksum to be computer
 \return The resulting checksum for the CAN frame.
 */
uint8_t toyotaChecksum(Panda::CanFrame& frame);

/*!
 \brief This funciton is helpful for debugging but should either not live in toyota.h or shoul dbe removed:
 \param frame The frame to be printed to the console
 */
void printFrame( Panda::CanFrame frame );

class ToyotaHandler;

/*!
 @class ToyotaListener
 \brief An abstract class for listening to notifications from a ToyotaHandler
 \par
 This reports the Panda::Health messages immediately, along with others at a future poiint
 */
class ToyotaListener {
private:
	friend class ToyotaHandler;
	
protected:
	/*!
	 \brief Called on a new panda health request
	 Overload this to get instant updates from the panda health update on each poll
	 \param pandaHealth The most recent PandaHealth.
	 */
	virtual void newPandaHealthNotification(const PandaHealth& pandaHealth) = 0;
	
	
   virtual void newControlNotification(ToyotaHandler* toyotaHandler) = 0;
	
public:
	
	virtual ~ToyotaListener() { };
};


/*!
 @class ToyotaHandler
 \brief A threaded interface class that handles sending contorl commands to a Panda via a Panda::Handler
 \par
 This is the intended methodology for sending control commands for a toyota with TSS2.0.  Tested on a RAV4 2019.
 */
class ToyotaHandler : public Mogi::Thread, public Panda::CanListener {
private:
	
	// Overloaded from Mogi::Thread.
	// This will enable the required power save mode for vehicle control.
	void entryAction();
	void exitAction();
	
	// Overloaded from Mogi::Thread.
	// This handles the constant updates.
	void doAction();
	
	// All of the following are called from doAction()
	void sendHeartBeat();
	void sendLka();
	void sendTrackB();
	void sendSteer();
	void sendAcc();
	
	// This will max the LKA decimator to trigger an instant send.
	void triggerInstantLkaSend();

	// Helper functions to check whether the hearbeats currently pass.
	// These will pass if a command was sent within the corresponding defined times:
	// TIME_HEARTBEAT_FAIL_STEERING
	// TIME_HEARTBEAT_FAIL_ACCELERATION
	bool heartbeatSteeringPass();
	bool heartbeatAccelerationPass();
	
	Panda::Handler* pandaHandler;
	PandaHealth health;
	bool controls_allowed_prior;
	bool controls_allowed;	// This is based on a rising edge of the panda health
	
	int decimatorHeartbeat;
	int decimatorLka;
	int decimatorTrackB;
	int decimatorSteer;
	int decimatorAcc;
	
	// For building proper CanFrames for steering and track B:
	unsigned char counterSteer;
	unsigned char counterTrackB;
	
	// HUD:
	bool hudLdaAlert;
	bool hudBarrier;//!barrier;
	unsigned char hudLeftLane;
	unsigned char hudRightLane;
	bool hudTwoBeeps;
	bool hudRepeatedBeeps;
	
	// Steering Control:
	double steerTorqueControl;
	bool steerRequest;
	unsigned char steerLkaState;
	
	// Acceleration Control:
	double accelerationControl;
	bool permitBraking;
	bool releaseStandstill;
	bool miniCar;
	bool cancelRequest;
	
	// Safety management, only send control commands if they are continuously updated
	int heartBeatSteer;
	int heartBeatAcceleration;
	
	// These are read directly from teh CAN bus:
	bool gas_released; // message ID 466
	bool brake_pressed; // message ID 550
//	bool car_cruise_ready_for_commands; // message ID 466
	unsigned char cruise_state;
	
	// For event notification observers:
	std::vector<ToyotaListener*> toyotaObservers;
	
	// listend to CAN information for state handling:
	void newDataNotification(CanFrame* canFrame);
	
public:
	
	/*!
	 \brief Construction must be done with a Panda::Handler
	 \param handler The active interface for the Panda.  The handler should be initialed with Panda::Handler::initialize() before Toyota::Handler::start() is called.
	 */
	ToyotaHandler(Panda::Handler* handler);
	
	/*!
	 \brief Tells the driver to grab the steering wheel
	 \param enable Whether the "Grab Steering Wheel" alert should be displayed
	 */
	void setHudLdaAlert( bool enable );
	
	/*!
	 \brief Shows lane barriers on the HUD
	 \param enable Whether the barriers should be displayed
	 */
	void setHudBarrier( bool enable );
	
	/*!
	 \brief Shows different lanes for the right and left side.  Valid values 0-3:
	 \par
	  0: Off
	  1: White
	  2: Hollow white
	  3: Blinking orange
	 \param laneLeft The setting for the left lane
	 \param rightLeft The setting for the right lane
	 */
	void setHudLanes( unsigned char laneLeft, unsigned char laneRight );
	
	/*!
	 \brief Will cause a double-beep alert.  Appears to be inconsistent in operation
	 \param enable Whether the Two Beeps should be played
	 */
	void setHudTwoBeeps( bool enable );
	
	/*!
	 \brief Will cause continuous beeping.
	 \par
	 \param enable Whether the repeated beeping should be played
	 */
	void setHudRepeatedBeeps( bool enable );
	
	/*!
	 \brief Displays the "Mini Car" on the HUD.
	 \par
	 Cruise control must be both on and active from "SET" being pushed for this to appear.
	 If ACC is SET and not RES, (i.e. starting cruise from a standstill) then MiniCar will blink.  Otherwise MiniCar is solid.
	 \param enable Whether the Mini Car should be displayed in the HUD.
	 */
	void setHudMiniCar( bool enable );
	
	/*!
	 \brief Invokes a cruise control cancel request, for the driver to take control over vehicle.
	 This will disable cruise control.  Commands can only be sent to the car again if the driver re-activates cruise control.
	 \param enable Whether to invoke the Cancel Request
	 */
	void setHudCruiseCancelRequest( bool enable );
	
	/*!
	 \brief Sends a steering torque to the steering wheel (non-working).
	 \par
	 The comma.ai panda code has the following limits for steerTorque:
	 \par
	 const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
	 \param steerTorque The steering torque to be sent.  Valid range is -1500:1500
	 */
	void setSteerTorque( int steerTorque );
	
	/*!
	 \brief Sends acceleration to the cruise controller, in units of m/s^2
	 \par
	 The comma.ai panda code has the following limits for acceleration:
	 \par
	 const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s^2
	 \par
	 const int TOYOTA_MIN_ACCEL = -3000;        // -3.0 m/s^2
	 \par
	  The following limits can be achieved by setting the panda into "unsafe" mode:
	 \par
	 const int TOYOTA_ISO_MAX_ACCEL = 2000;        // 2.0 m/s^2
	 \par
	 const int TOYOTA_ISO_MIN_ACCEL = -3500;       // -3.5 m/s^2
	 \par
	 The DBC file however has a different supported range of -20:20
	 \param acceleration The acceleration to be sent. Units are m/s^2, valid range is always -3.0:1.5
	 */
	void setAcceleration( double acceleration );
	
	/*!
	 \brief Returns the Panda report for whether the ignition is on (line).
	 
	 \par
	 Note: The Panda also reports "ignition_can but appears to always be off in the RAV4.
	 This only gets updated at 1Hz from TOYOTA_RATE_HEARTBEAT.
	 \return Whether the ignition is turned on or off.
	 */
	bool getIgnitionOn();
	
	/*!
	 \brief Returns the Panda health report for whether controls are allowed.
	 Controls are allowed when the cruise control is turned on and active from a SET press.
	 This will get disabled by many events.  Consider the following sequences with the corresponding return result of this function:
	 \par
	 Example sequence 1:
	 \par
	 0. Be Parked: 0
	 1. Before Cruise control enabled: 0
	 2. Cruise control button pushed, enabling cruise: 0
	 3. SET pushed while still in Park: 0
	 4. SET pushed after shifting to Drive with brake held: 1
	 5. Brake released: 1
	 6. Brake pushed again: 0
	 7. controlsAreAllowed will only return 0 unless cruise is cancelled or reset at this stage
	
	 \par
	 Example sequence 2:
	 \par
	 0. Be Driving before Cruise control enabled: 0
	 1. Cruise control button pushed, enabling cruise: 0
	 2. SET pushed while moving in drive: 1
	 3. Brake pushed OR Throttle pushed: 0
	 4. SET pushed again: 0
	 5. Cancel pushed, then SET pushed: 1
	 
	 \par
	 \note
	 Note: When the brake is pushed to cancel, the HUD "SET" and radar will also disappear.
			However when the throttle is pressed, "SET" does not disappear even though controls are no longer allowed
	
	 \par
	 This only gets updated at 1Hz from TOYOTA_RATE_HEARTBEAT
	 \return Whether the Panda will allow controls to be sent to the vehicle.
	 */
	bool getPandaControlsAllowed();
	bool getControlsAllowed();
//	bool getCarCruiseReadyForCommands();
	unsigned char getCarCruiseState();	// from message 466
	
	/*!
	 \brief Returns the full Panda Health state.  controls_allowed and ignition_line can be read form this
	 \return The last read state of the panda's state
	 */
	const PandaHealth& getPandaHealth() const;
	
	/*!
	 \brief Adds an observer to event notifications
	 \param observer The observer for notification subscription
	 */
	void addObserver( ToyotaListener* observer );
};

}

#endif
