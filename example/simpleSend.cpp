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

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

#include "panda/controller.h"
#include "panda/toyota.h"

#include "joystickState.h"

using namespace Panda;

class ExampleControllerListener : public Panda::ControllerListener {
private:
    void newPandaHealthNotification(const PandaHealth& pandaHealth) {
        Panda::printPandaHealth(pandaHealth);
    };
    
    
    void newControlNotification(Panda::Controller* controller) {
        std::cout << "ExampleControllerListener::newControlNotification() controls_allowed: " << controller->getControlsAllowed() << std::endl;
    };
};

class ExampleSteeringLimitListener: public Panda::SteeringLimiterListener {
private:
    void steeringLimitNotification( Panda::STEERING_STATE value) {
        std::cout << "New steering limit notification: " << (int)value << std::endl;
    }
};

static volatile bool keepRunning = true;
void killPanda(int killSignal) {
    std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
    keepRunning = false;
}



double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

//// the following functions are ported from selfdrive/car/__init__.py
//double apply_dist_to_meas_limits(double val, double val_last, double val_meas, int STEER_DELTA_UP, int STEER_DELTA_DOWN, int STEER_ERROR_MAX, int STEER_MAX) {
//  // limits due to comparison of commanded val VS measured val (torque/angle/curvature)
//    double max_lim = fmin(fmax(val_meas + STEER_ERROR_MAX, STEER_ERROR_MAX), STEER_MAX);
//    double min_lim = fmax(fmin(val_meas - STEER_ERROR_MAX, -STEER_ERROR_MAX), -STEER_MAX);
//
//    val = clip(val, min_lim, max_lim);
//
//  // slow rate if val increases in magnitude
//    if(val_last > 0) {
//        val = clip(val,
//                   fmax(val_last - STEER_DELTA_DOWN, -STEER_DELTA_UP),
//                   val_last + STEER_DELTA_UP);
//    } else {
//        val = clip(val,
//                   val_last - STEER_DELTA_UP,
//                   fmin(val_last + STEER_DELTA_DOWN, STEER_DELTA_UP));
//    }
//    return val;
//}
//int apply_meas_steer_torque_limits(double apply_torque, double apply_torque_last, double motor_torque, int STEER_DELTA_UP, int STEER_DELTA_DOWN,
//                                   int STEER_ERROR_MAX, int STEER_MAX) {
//    return (int)round(apply_dist_to_meas_limits(apply_torque, apply_torque_last, motor_torque, STEER_DELTA_UP-1, STEER_DELTA_DOWN-1, STEER_ERROR_MAX, STEER_MAX));
//}


// copied from panda firwmare safety_toyota.h:
// global torque limit
const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever

// rate based torque limit + stay within actually applied
// packet is sent at 100hz, so this limit is 1500/sec
const int TOYOTA_MAX_RATE_UP = 15;        // ramp up slow
const int TOYOTA_MAX_RATE_DOWN = 25;      // ramp down fast
const int TOYOTA_MAX_TORQUE_ERROR = 350;  // max torque cmd in excess of torque motor

// real time torque limit to prevent controls spamming
// the real time limit is 1800/sec, a 20% buffer
const int TOYOTA_MAX_RT_DELTA = 450;      // max delta torque allowed for real time checks
const uint32_t TOYOTA_RT_INTERVAL = 250000;    // 250ms between real time checks

#define MIN(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
(_a < _b) ? _a : _b; })
    
#define MAX(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
(_a > _b) ? _a : _b; })

// convert a trimmed integer to signed 32 bit int
int to_signed(int d, int bits) {
    int d_signed = d;
    if (d >= (1 << MAX((bits - 1), 0))) {
        d_signed = d - (1 << MAX(bits, 0));
    }
    return d_signed;
}


//// sample struct that keeps 6 samples in memory
//struct sample_t {
//    int values[6];
//    int min;
//    int max;
//} sample_t_default = {.values = {0}, .min = 0, .max = 0};

// given a new sample, update the sample_t struct
void update_sample(struct sample_t *sample, int sample_new) {
    int sample_size = sizeof(sample->values) / sizeof(sample->values[0]);
    for (int i = sample_size - 1; i > 0; i--) {
        sample->values[i] = sample->values[i-1];
    }
    sample->values[0] = sample_new;
    
    // get the minimum and maximum measured samples
    sample->min = sample->values[0];
    sample->max = sample->values[0];
    for (int i = 1; i < sample_size; i++) {
        if (sample->values[i] < sample->min) {
            sample->min = sample->values[i];
        }
        if (sample->values[i] > sample->max) {
            sample->max = sample->values[i];
        }
    }
    
}

//class ToyotaSteeringTorqueLimiter2 : public Panda::CanListener {
//private:
//    struct sample_t torque_meas;       // last 6 motor torques produced by the eps
//
//    double last_steer;
//
//    // safety param flags
//    // first byte is for eps factor, second is for flags
//    const uint32_t TOYOTA_PARAM_OFFSET = 8U;
//    const uint32_t TOYOTA_EPS_FACTOR = (1U << TOYOTA_PARAM_OFFSET) - 1U;
//
//    int toyota_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file
//
//    void newDataNotification(  Panda::CanFrame* frame ) {
//        if(frame->messageID == 0x260 &&
//           frame->bus == 0 &&
//           frame->dataLength == 8) {    // also maybe good idea to check checksum
//            handleEpsFrame(*frame);
//        }
//    }
//
//    void handleEpsFrame(Panda::CanFrame& frame) {
//        //        int torque_meas_new = (GET_BYTE(to_push, 5) << 8) | GET_BYTE(to_push, 6);
//        int torque_meas_new = (frame.data[5] << 8) | frame.data[6];
//        torque_meas_new = to_signed(torque_meas_new, 16);
//
//        // scale by dbc_factor
//        torque_meas_new = (torque_meas_new * toyota_dbc_eps_torque_factor) / 100;
//
//        // update array of sample
//        update_sample(&torque_meas, torque_meas_new);
//
//        // increase torque_meas by 1 to be conservative on rounding
//        torque_meas.min--;
//        torque_meas.max++;
//    }
//
//public:
//    ToyotaSteeringTorqueLimiter2()
//    : last_steer(0) {
//
//    }
//
//    void initWithParam(uint16_t param) {
//        toyota_dbc_eps_torque_factor = param & TOYOTA_EPS_FACTOR;
//    }
//
//    int measuredTorque() {
//        return torque_meas.values[0];
//    }
//
//    // This runs at 100Hz per toyota's carcntroller.py
//    int doIt(double input) {
//        int result =  apply_meas_steer_torque_limits(input, last_steer, torque_meas.values[0], TOYOTA_MAX_RATE_UP, TOYOTA_MAX_RATE_DOWN, TOYOTA_MAX_TORQUE_ERROR, TOYOTA_MAX_TORQUE);
//        last_steer = result;
//        return result;
//    }
//
//};

// simulates the panda firmware to detect when and why violations occur:
class ToyotaSteeringViolationSimulator : public Panda::CanListener {
    
    uint32_t microsecond_timer_get() {
        // TODO:
        static uint32_t time = 0;
        time += 10000; // this assumes 100Hz
        return time;
    }
    
    // for safety modes with torque steering control
    int desired_torque_last = 0;       // last desired steer torque
    int rt_torque_last = 0;            // last desired torque for real time check
    struct sample_t torque_meas;       // last 6 motor torques produced by the eps
    struct sample_t torque_driver;     // last 6 driver torques measured
    uint32_t ts_last = 0;
    
    
    // safety param flags
    // first byte is for eps factor, second is for flags
    const uint32_t TOYOTA_PARAM_OFFSET = 8U;
    const uint32_t TOYOTA_EPS_FACTOR = (1U << TOYOTA_PARAM_OFFSET) - 1U;
    
    int toyota_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file
    


    
    bool max_limit_check(int val, const int MAX_VAL, const int MIN_VAL) {
        return (val > MAX_VAL) || (val < MIN_VAL);
    }
    
    // check that commanded value isn't too far from measured
    bool dist_to_meas_check(int val, int val_last, struct sample_t *val_meas,
                            const int MAX_RATE_UP, const int MAX_RATE_DOWN, const int MAX_ERROR) {
        
        // *** val rate limit check ***
        int highest_allowed_rl = MAX(val_last, 0) + MAX_RATE_UP;
        int lowest_allowed_rl = MIN(val_last, 0) - MAX_RATE_UP;
        
        // if we've exceeded the meas val, we must start moving toward 0
        int highest_allowed = MIN(highest_allowed_rl, MAX(val_last - MAX_RATE_DOWN, MAX(val_meas->max, 0) + MAX_ERROR));
        int lowest_allowed = MAX(lowest_allowed_rl, MIN(val_last + MAX_RATE_DOWN, MIN(val_meas->min, 0) - MAX_ERROR));
        
        if(val < lowest_allowed) {
            printf("dist_to_meas_check) Failed check: (val < lowest_allowed) (%d < %d)\n", val, lowest_allowed);
        }
        if(val > highest_allowed) {
            printf("dist_to_meas_check) Failed check: (val > highest_allowed) (%d > %d)\n", val, highest_allowed);
        }
        
        // check for violation
        return (val < lowest_allowed) || (val > highest_allowed);
    }
    
    
    
    // real time check, mainly used for steer torque rate limiter
    bool rt_rate_limit_check(int val, int val_last, const int MAX_RT_DELTA) {
        
        // *** torque real time rate limit check ***
        int highest_val = MAX(val_last, 0) + MAX_RT_DELTA;
        int lowest_val = MIN(val_last, 0) - MAX_RT_DELTA;
        
        // check for violation
        return (val < lowest_val) || (val > highest_val);
    }
    
    // compute the time elapsed (in microseconds) from 2 counter samples
    // case where ts < ts_last is ok: overflow is properly re-casted into uint32_t
    uint32_t get_ts_elapsed(uint32_t ts, uint32_t ts_last) {
        return ts - ts_last;
    }
    
    
    
    
    
    void newDataNotification(  Panda::CanFrame* frame ) {
        if(frame->messageID == 0x260 &&
           frame->bus == 0 &&
           frame->dataLength == 8) {    // also maybe good idea to check checksum
            handleEpsFrame(*frame);
        }
    }
    
public:
    
    ToyotaSteeringViolationSimulator() {
        initWithParam(73); // this is the hardcoded default when setting Toyota safety mode for control
    }
    
    void initWithParam(uint16_t param) {
        toyota_dbc_eps_torque_factor = param & TOYOTA_EPS_FACTOR;
    }
    
    void handleEpsFrame(Panda::CanFrame& frame) {
        //        int torque_meas_new = (GET_BYTE(to_push, 5) << 8) | GET_BYTE(to_push, 6);
        int torque_meas_new = (frame.data[5] << 8) | frame.data[6];
        torque_meas_new = to_signed(torque_meas_new, 16);
        
        // scale by dbc_factor
        torque_meas_new = (torque_meas_new * toyota_dbc_eps_torque_factor) / 100;
        
        // update array of sample
        update_sample(&torque_meas, torque_meas_new);
        
        // increase torque_meas by 1 to be conservative on rounding
        torque_meas.min--;
        torque_meas.max++;
    }
    
    void process(int desired_torque, bool controls_allowed) {
//        int desired_torque = (STEERING_LKA.data[1] << 8) | STEERING_LKA.data[2];
        bool violation = false;
        
        uint32_t ts = microsecond_timer_get();
        
        if (controls_allowed) {
            
            // *** global torque limit check ***
            if(max_limit_check(desired_torque, TOYOTA_MAX_TORQUE, -TOYOTA_MAX_TORQUE)) {
                violation |= 1;
                std::cerr << "ToyotaSteeringViolationSimulator::max_limit_check failed!" << std::endl;
            }
            
            // *** torque rate limit check ***
            if(dist_to_meas_check(desired_torque, desired_torque_last,
                                  &torque_meas, TOYOTA_MAX_RATE_UP, TOYOTA_MAX_RATE_DOWN, TOYOTA_MAX_TORQUE_ERROR)) {
                violation |= 1;
                std::cerr << "ToyotaSteeringViolationSimulator::dist_to_meas_check failed!" << std::endl;
            }
            
            // used next time
            desired_torque_last = desired_torque;
            
            // *** torque real time rate limit check ***
            if(rt_rate_limit_check(desired_torque, rt_torque_last, TOYOTA_MAX_RT_DELTA)) {
                violation |= 1;
                std::cerr << "ToyotaSteeringViolationSimulator::rt_rate_limit_check failed!" << std::endl;
            }
            
            // every RT_INTERVAL set the new limits
            uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
            if (ts_elapsed > TOYOTA_RT_INTERVAL) {
                rt_torque_last = desired_torque;
                ts_last = ts;
            }
        }
        
        // no torque if controls is not allowed
        if (!controls_allowed && (desired_torque != 0)) {
            violation = 1;
        }
        
        // reset to 0 if either controls is not allowed or there's a violation
        if (violation || !controls_allowed) {
            desired_torque_last = 0;
            rt_torque_last = 0;
            ts_last = ts;
        }
    }
};


int main(int argc, char **argv) {
    
    // Build the joystick reader:
    JoystickState mJoystickState;
    Joystick mJoystick;
    
    mJoystick.addObserver(&mJoystickState);
    mJoystick.open("/dev/input/js0");
    mJoystick.start();
    
    //Set up graceful exit
    signal(SIGINT, killPanda);
    
    // Initialize panda and toyota handlers
    Panda::Handler pandaHandler;
    //	Panda::ToyotaHandler toyotaHandler(&pandaHandler);
    //	Panda::ToyotaHandler toyotaHandler();
    
    
    
    //	pandaHandler.getCan().addObserver(&toyotaHandler);
    
    // Let's roll
    pandaHandler.initialize();
    
    // initialize gets the VIN, now we can build a controller:
    Panda::ControllerClient* pandaController = new Panda::ControllerClient(pandaHandler);
    //	toyotaHandler.start();
    if(pandaController->getController() == NULL) {
        std::cerr << "ERROR: No VIN discovered, unable to make controller" << std::endl;
        //		exit(EXIT_FAILURE);
        
        delete pandaController;
        
        pandaHandler.forceSetVin((const unsigned char*)"2T3Y1RFV8KC014025");
        pandaController = new Panda::ControllerClient(pandaHandler);
        if(pandaController->getController() == NULL) {
            std::cerr << "ERROR 2: No VIN discovered, unable to make test controller" << std::endl;
            exit(EXIT_FAILURE);
        }
        
    }
    
    ExampleControllerListener myExampleControllerListener;
    pandaController->getController()->addObserver(&myExampleControllerListener);
    
    ExampleSteeringLimitListener myExampleSteeringLimitListener;    // This is only added if toyota is confrimed (below, next if statemenr)
    
    
    ToyotaSteeringTorqueLimiter mToyotaSteeringTorqueLimiter2;
    ToyotaSteeringViolationSimulator mToyotaSteeringViolationSimulator;
    if (pandaHandler.getVehicleManufacturer() == Panda::VEHICLE_MANUFACTURE_TOYOTA) {
        mToyotaSteeringTorqueLimiter2.initWithParam(73);
        mToyotaSteeringViolationSimulator.initWithParam(73);
        pandaHandler.addCanObserver(mToyotaSteeringTorqueLimiter2);
        pandaHandler.addCanObserver(mToyotaSteeringViolationSimulator);
        
        // This is nasty:
        Panda::ToyotaHandler* toyotaHandler = static_cast<Panda::ToyotaHandler*>(pandaController->getController());
        pandaHandler.addCanObserver(toyotaHandler->mToyotaSteeringTorqueLimiter);
        
        toyotaHandler->addSteeringTorqueLimiterListener(&myExampleSteeringLimitListener);
        
    }
    
    pandaController->getController()->start();
    
    // These are for setting the HUD
    unsigned char hudLaneLeft = 2;
    unsigned char hudLaneRight = 2;
    
    //	int printPandaHealthDecimator = 0;
    int steerTorqueToSend = 0;
    while(keepRunning) {
        usleep(1000000.0/100.0);	// run at ~100 Hz
        
        //		if(printPandaHealthDecimator++ >= 10) {	// run at 1Hz
        //			printPandaHealthDecimator = 0;
        //			Panda::printPandaHealth(toyotaHandler.getPandaHealth());
        ////			std::cout << "Controls Allowed panda: " << (int)toyotaHandler.getPandaControlsAllowed() << std::endl;
        ////			std::cout << "Controls Allowed Libpanda: " << toyotaHandler.getControlsAllowed()() << std::endl;
        ////			std::cout << "Ignition On: " << toyotaHandler.getIgnitionOn() << std::endl;
        //		}
        
        // Setting HUD elements:
        
        hudLaneLeft += mJoystickState.getButtonL1Rising();
        hudLaneLeft -= mJoystickState.getButtonL2Rising();
        hudLaneRight += mJoystickState.getButtonR1Rising();
        hudLaneRight -= mJoystickState.getButtonR2Rising();
        if (pandaHandler.getVehicleManufacturer() == Panda::VEHICLE_MANUFACTURE_TOYOTA) {
            Panda::ToyotaHandler* toyotaHandler = static_cast<Panda::ToyotaHandler*>(pandaController->getController());
            
            toyotaHandler->setHudLanes(hudLaneLeft, hudLaneRight);
            
            toyotaHandler->setHudLdaAlert( mJoystickState.getTriangle() );
            toyotaHandler->setHudTwoBeeps( mJoystickState.getX() );
            toyotaHandler->setHudRepeatedBeeps( mJoystickState.getSelect() );
            toyotaHandler->setHudBarrier( mJoystickState.getDY() > 0 );
            toyotaHandler->setHudMiniCar( mJoystickState.getDX() > 0 );
        }
        
        
        
        
        // This will cancel the cruise control, cruise must be rest by driver to allow further controls
        //		toyotaHandler.setHudCruiseCancelRequest( mJoystickState.getSquare() );
        
        // Acceleration command building.  Units are m/s^2
        // The following are hard-coded limits in the Panda firmware:
        //const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s2
        //const int TOYOTA_MIN_ACCEL = -3000;       // -3.0 m/s2
        // The following limits can be achieved by setting the panda into "unsafe" mode:
        //const int TOYOTA_ISO_MAX_ACCEL = 2000;        // 2.0 m/s2
        //const int TOYOTA_ISO_MIN_ACCEL = -3500;       // -3.5 m/s2
        double acceleration = 0.0;
        double joystickValue = mJoystickState.getLY();	// getLY() returns a range of -1.0:1.0
        
        //		if (joystickValue > 0) {
        //			acceleration = 1.5 * joystickValue;
        //		} else if (joystickValue < 0) {
        //			acceleration = 3.0 * joystickValue;
        //		}
        
        acceleration = 4.5*((joystickValue + 1.0)/2.0 ) - 3.0 - 1.0; // This function means joystick at rest = -0.75 m/s/s
        
        // Steering torque command.  Deosn't yet work, unknown units
        // The following is a hard-coded limit in the Panda firmware:
        //const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
        joystickValue = mJoystickState.getRX();	// getRX() returns a range of -1.0:1.0
        double steerTorque = -1500 * joystickValue;	// range: -1500:1500
        
//        if(steerTorqueToSend < steerTorque) {
//            double diff = steerTorque - steerTorqueToSend;
//            steerTorqueToSend += diff > 5 ? 5 : diff;
//        } else if (steerTorqueToSend > steerTorque) {
//            double diff = steerTorque - steerTorqueToSend;
//            steerTorqueToSend += diff < -5 ? -5 : diff;
//        }
        //        if(
        //        steerTorqueToSend += (steerTorque - steerTorqueToSend)/200.0;
        
        
        
        // Send the Steering and Scceleration commands:
        // Holding circle tests the heartbeat (stopping it)
        // The heartbeat failing will also trigger some HUD elements like setHudRepeatedBeeps and setHudLdaAlert
        if ( !mJoystickState.getCircle() ) {
            //			toyotaHandler.setAcceleration(acceleration);
            ////			toyotaHandler.setSteerTorque(steerTorque);
            //			toyotaHandler.setSteerTorque(0);
            steerTorqueToSend = mToyotaSteeringTorqueLimiter2.doIt(steerTorque);
//            std::cout << "Want/Sent/Measured: "  << steerTorque << "\t" << steerTorqueToSend << "\t" << mToyotaSteeringTorqueLimiter2.measuredTorque() << std::endl;
            
            
            mToyotaSteeringViolationSimulator.process(steerTorqueToSend, pandaController->getController()->getControlsAllowed());
            
            
            
            pandaController->getController()->setAcceleration(acceleration);
            pandaController->getController()->setSteerTorque(steerTorqueToSend);
            //			pandaController->getController()->setSteerTorque(0);
//            std::cout << std::endl;
        }
        
        // Debug Joystick:
//        		mJoystickState.printState();
        
    }
    
    
    // Will never reach here
    std::cout << "Stopping pandacontroller..." << std::endl;
    //	toyotaHandler.stop();
    pandaController->getController()->stop();
    std::cout << "Stopping pandaHandler..." << std::endl;
    pandaHandler.stop();
    
    std::cout << "simpleSend is Done." << std::endl;
    //	return 0;
    exit(EXIT_SUCCESS);
    return 0;
}
