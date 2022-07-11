/*
 Author: Matt Bunting
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include "mogi/thread.h"

#include "panda.h"

//#define PANDA_RATE_HEARTBEAT (2.0)	// Hz
#define TIME_HEARTBEAT_FAIL_STEERING (1.0)		// In seconds, time until a heartbeat fails from not receiving a new steering command
#define TIME_HEARTBEAT_FAIL_ACCELERATION (1.0)	// In seconds, time until a heartbeat fails from not receiving a new acceleration command

#define CONTROLLER_RATE_CA_REPORT (1.0)	// This is for notifications of controls_allowed

//#define CONTROLLER_DECIMATOR_MAX_CA_REPORT (CONTROLLER_COMMAND_THREAD_RATE/CONTROLLER_RATE_CA_REPORT)

namespace Panda {


class ControllerClient;
class ControllerListener;
//class HeartbeatHelper;

class Controller : public Mogi::Thread, public Panda::CanListener, public Panda::HeartbeatHelperListener {
	friend class ControllerClient;
//	friend class HeartbeatHelper;
	friend class ControllerListener;
	
private:
//	HeartbeatHelper* mHeartbeatHelper;
	Panda::Handler* pandaHandler;
	
	PandaHealth health;
	std::vector<ControllerListener*> controllerObservers;
	
	bool controls_allowed_prior;
	bool controls_allowed;
	
	double intervalActionRate;
	int decimatorTotalControlsAllowed;
	int decimatorControlsAllowedCounter;
	
	int heartBeatSteer;
	int heartBeatAcceleration;
	
	// Overloaded from Mogi::Thread.
	// This will enable the required power save mode for vehicle control.
	void entryAction();
	void exitAction();
	
	// Overloaded from Mogi::Thread.
	// This handles the constant updates.
	void doAction();
	
	// Overload from Panda::HeartbeatHelperListener
	void notificationHeartbeat(const PandaHealth& health);
	
	
protected:
	Controller();
	
	static Controller* create( Panda::Handler& handler );
	
//	void sendHeartBeat();
	
	bool heartbeatSteeringPass();
	bool heartbeatAccelerationPass();
	
	// This will get called at regular intervals
	// Overlaod this, then decimate for CAN message generation
	virtual void intervalAction() {};
	
	void setIntervalActionRate( double rate );
	
	// For use within intervalAction()
	void sendCan(CanFrame& frame);
	
	
	virtual void handleSetSteerTorque( int steerTorque ) = 0;
	
	virtual void handleSetAcceleration( double acceleration ) = 0;
	
	virtual void newCanNotification(CanFrame* canFrame) {};
	
public:
	virtual ~Controller() = 0;
	
	void addObserver( ControllerListener* observer );
	
	/*! \brief Check the latest CAN message and return whether controls should be engaged.
	 Example, if a message states the brake is pressed, then return false.
	 This may require multiple messages, so you'll need to store the last state
	 of all important messages since they will arrive asynchronously
	 \param frame The The most recent CAN frame
	 \return Whether controls should be allowed
	 */
	virtual bool checkControlsAllowed(Panda::CanFrame* frame) = 0;
	
	bool getControlsAllowed();
	
	
	/*!
	 \brief Sends a steering torque to the steering wheel, if supported.
	 \param steerTorque The steering torque to be sent.
	 */
	void setSteerTorque( int steerTorque );
	
	/*!
	 \brief Sends acceleration to the cruise controller, in units of m/s^2 (if supported)
	 \param acceleration The acceleration to be sent. Units are m/s^2.
	 */
	void setAcceleration( double acceleration );
	
	
	Panda::Handler* getPandaHandler();
	
	
	// HACK HACK HACK
	// Overload from Panda::CanListener
	void newDataNotification(CanFrame* canFrame);
};
//
///*!
// @class PandaHeartbeatHelper
// \brief A helper thread class for clearing
// \par
// Invokes messages in ToyotaHandler at an asynchronous rate
// */
//class HeartbeatHelper : public Mogi::Thread {
//private:
////	Controller* mController;
//	Handler* mHandler;
//	
//	void doAction();
//	
//public:
////	HeartbeatHelper(Controller* handler);
//	HeartbeatHelper(Handler* handler);
//
//	
//};





class ControllerClient {
private:
	Controller* vehicleController;
	
public:
	ControllerClient(Panda::Handler& handler) {
		//		vehicleController = Controller::create(handler.getVehicleMake());
		vehicleController = Controller::create(handler);
	};
	
	~ControllerClient() {
		if(vehicleController != NULL) {
			delete vehicleController;
			vehicleController = NULL;
		}
	}
	Controller* getController() {
		return vehicleController;
	}
	
};




/*!
 @class ControllerListener
 \brief An abstract class for listening to notifications from a vehicle Controller
 \par
 This reports the Panda::Health messages immediately, along with others at a future poiint
 */
class ControllerListener {
private:
	friend class Controller;
	
protected:
	/*!
	 \brief Called on a new panda health request
	 Overload this to get instant updates from the panda health update on each poll
	 \param pandaHealth The most recent PandaHealth.
	 */
	virtual void newPandaHealthNotification(const PandaHealth& pandaHealth) {};
	
	
	virtual void newControlNotification(Controller* controller) {};
	
public:
	
	virtual ~ControllerListener() = 0;
};

}

#endif
