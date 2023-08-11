/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt University
 All rights reserved.

 */

#include "panda/toyota.h"
#include "panda/toyota-panda-steering-definitions.h"

#include <unistd.h>
//#include <chrono>
#include <cmath>


using namespace Panda;

double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

// the following functions are ported from selfdrive/car/__init__.py
double apply_dist_to_meas_limits(double val, double val_last, double val_meas, int STEER_DELTA_UP, int STEER_DELTA_DOWN, int STEER_ERROR_MAX, int STEER_MAX) {
  // limits due to comparison of commanded val VS measured val (torque/angle/curvature)
    double max_lim = fmin(fmax(val_meas + STEER_ERROR_MAX, STEER_ERROR_MAX), STEER_MAX);
    double min_lim = fmax(fmin(val_meas - STEER_ERROR_MAX, -STEER_ERROR_MAX), -STEER_MAX);

    val = clip(val, min_lim, max_lim);

  // slow rate if val increases in magnitude
    if(val_last > 0) {
        val = clip(val,
                   fmax(val_last - STEER_DELTA_DOWN, -STEER_DELTA_UP),
                   val_last + STEER_DELTA_UP);
    } else {
        val = clip(val,
                   val_last - STEER_DELTA_UP,
                   fmin(val_last + STEER_DELTA_DOWN, STEER_DELTA_UP));
    }
    return val;
}
int apply_meas_steer_torque_limits(double apply_torque, double apply_torque_last, double motor_torque, int STEER_DELTA_UP, int STEER_DELTA_DOWN,
                                   int STEER_ERROR_MAX, int STEER_MAX) {
    return (int)round(apply_dist_to_meas_limits(apply_torque, apply_torque_last, motor_torque, STEER_DELTA_UP, STEER_DELTA_DOWN, STEER_ERROR_MAX, STEER_MAX));
}


//// copied from panda firwmare safety_toyota.h:
//// global torque limit
//const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
//
//// rate based torque limit + stay within actually applied
//// packet is sent at 100hz, so this limit is 1500/sec
//const int TOYOTA_MAX_RATE_UP = 10;        // ramp up slow
//const int TOYOTA_MAX_RATE_DOWN = 20;      // ramp down fast
//const int TOYOTA_MAX_TORQUE_ERROR = 350;  // max torque cmd in excess of torque motor
//
//// real time torque limit to prevent controls spamming
//// the real time limit is 1800/sec, a 20% buffer
//const int TOYOTA_MAX_RT_DELTA = 450;      // max delta torque allowed for real time checks
//const uint32_t TOYOTA_RT_INTERVAL = 250000;    // 250ms between real time checks

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

//class ToyotaSteeringTorqueLimiter : public Panda::CanListener {
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
    
    void ToyotaSteeringTorqueLimiter::newDataNotification(  Panda::CanFrame* frame ) {
        if(frame->messageID == 0x260 &&
           frame->bus == 0 &&
           frame->dataLength == 8) {    // also maybe good idea to check checksum
            handleEpsFrame(*frame);
        }
    }
    
    void ToyotaSteeringTorqueLimiter::handleEpsFrame(Panda::CanFrame& frame) {
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
    
//public:
ToyotaSteeringTorqueLimiter::ToyotaSteeringTorqueLimiter()
    : last_steer(0) {
        initWithParam(73); // HACK
    }
    
    void ToyotaSteeringTorqueLimiter::initWithParam(uint16_t param) {
        toyota_dbc_eps_torque_factor = param & TOYOTA_EPS_FACTOR;
    }
    
    int ToyotaSteeringTorqueLimiter::measuredTorque() {
        return torque_meas.values[0];
    }
    
    // This runs at 100Hz per toyota's carcntroller.py
    int ToyotaSteeringTorqueLimiter::doIt(double input) {
        int result =  apply_meas_steer_torque_limits(input, last_steer, torque_meas.values[0], TOYOTA_MAX_RATE_UP, TOYOTA_MAX_RATE_DOWN, TOYOTA_MAX_TORQUE_ERROR, TOYOTA_MAX_TORQUE);
        last_steer = result;
        if(result != input) {
            for(std::vector<SteeringLimiterListener*>::iterator it = limitObservers.begin(); it != limitObservers.end(); it++) {
                (*it)->setLimitNotification( STEERING_STATE_OK );
            }
        } else {
            for(std::vector<SteeringLimiterListener*>::iterator it = limitObservers.begin(); it != limitObservers.end(); it++) {
            (*it)->setLimitNotification( STEERING_STATE_LIMITED );
        }
            
        }
        return result;
    }
    
void ToyotaSteeringTorqueLimiter::addObserver( SteeringLimiterListener* observer ) {
    limitObservers.push_back(observer);
}
