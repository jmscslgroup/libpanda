/*
 Author: Matt Bunting
 */

#include "panda/toyota.h"

using namespace Panda;

SteeringLimiterListener::~SteeringLimiterListener() {
	
}

void SteeringLimiterListener::setLimitNotification( STEERING_STATE value ) {
    if(value != limitError) {
        steeringLimitNotification( value );
        limitError = value;
    }
}
