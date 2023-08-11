/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt University

 */

#ifndef TOYOTA_PANDA_STEERING_DEFINITIONS_H
#define TOYOTA_PANDA_STEERING_DEFINITIONS_H

// copied from panda firwmare safety_toyota.h:
// global torque limit
const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever

// rate based torque limit + stay within actually applied
// packet is sent at 100hz, so this limit is 1500/sec
const int TOYOTA_MAX_RATE_UP = 10;        // ramp up slow
const int TOYOTA_MAX_RATE_DOWN = 20;      // ramp down fast
const int TOYOTA_MAX_TORQUE_ERROR = 350;  // max torque cmd in excess of torque motor

// real time torque limit to prevent controls spamming
// the real time limit is 1800/sec, a 20% buffer
const int TOYOTA_MAX_RT_DELTA = 450;      // max delta torque allowed for real time checks
const uint32_t TOYOTA_RT_INTERVAL = 250000;    // 250ms between real time checks




#endif
