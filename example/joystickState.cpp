/******************************************************************************
 *                                                                            *
 *             Copyright (C) 2020 Mogi, LLC - All Rights Reserved             *
 *                            Author: Matt Bunting                            *
 *                                                                            *
 *   Proprietary and confidential.                                            *
 *                                                                            *
 *   Unauthorized copying of this file via any medium is strictly prohibited  *
 *   without the explicit permission of Mogi, LLC.                            *
 *                                                                            *
 *   See license in root directory for terms.                                 *
 *   http://www.binpress.com/license/view/l/0088eb4b29b2fcff36e42134b0949f93  *
 *                                                                            *
 *****************************************************************************/

#include "joystickState.h"

#include <cstring>	// for memcpy
#include <iostream>

JoystickState::JoystickState() {
	memset(&valuesRising, 0, sizeof(valuesRising));
	memset(&valuesCurrent, 0, sizeof(valuesCurrent));
}

void JoystickState::newDeviceEvent( const DeviceEvent* event ) {	// override from DeviceObserver
	if (event->type == TYPE_AXIS) {
		switch (event->id) {
			case AXIS_RX: valuesCurrent.axisRX = ((double)event->value)/32768.0; break;
			case AXIS_RY: valuesCurrent.axisRY = -((double)event->value)/32768.0; break;
			case AXIS_LX: valuesCurrent.axisLX = ((double)event->value)/32768.0; break;
			case AXIS_LY: valuesCurrent.axisLY = -((double)event->value)/32768.0; break;
			case AXIS_DX: valuesCurrent.axisDX = ((double)event->value)/32768.0; break;
			case AXIS_DY: valuesCurrent.axisDY = -((double)event->value)/32768.0; break;
			default:
				std::cout << "Unrecognized Axis id: " << (int)event->id << "value:" << event->value << std::endl;
				break;
		}
		//std::cout << "Event id:" << (int)event->id << "\tvalue:" << event->value << std::endl;
	} else if (event->type == TYPE_BUTTON) {
		switch (event->id) {
			case BUTTON_TRIANGLE:
				valuesCurrent.buttonTriangle = event->value;
				if(event->value)
					valuesRising.buttonTriangle = 1;
				break;
			case BUTTON_SQUARE:
				valuesCurrent.buttonSquare = event->value;
				if(event->value)
					valuesRising.buttonSquare = 1;
				break;
			case BUTTON_CIRCLE:
				valuesCurrent.buttonCircle = event->value;
				if(event->value)
					valuesRising.buttonCircle = 1;
				break;
			case BUTTON_X:
				valuesCurrent.buttonX = event->value;
				if(event->value)
					valuesRising.buttonX = 1;
				break;
			case BUTTON_L1:
				valuesCurrent.buttonL1 = event->value;
				if(event->value)
					valuesRising.buttonL1 = 1;
				break;
			case BUTTON_L2:
				valuesCurrent.buttonL2 = event->value;
				if(event->value)
					valuesRising.buttonL2 = 1;
				break;
			case BUTTON_L3:
				valuesCurrent.buttonL3 = event->value;
				if(event->value)
					valuesRising.buttonL3 = 1;
				break;
			case BUTTON_R1:
				valuesCurrent.buttonR1 = event->value;
				if(event->value)
					valuesRising.buttonR1 = 1;
				break;
			case BUTTON_R2:
				valuesCurrent.buttonR2 = event->value;
				if(event->value)
					valuesRising.buttonR2 = 1;
				break;
			case BUTTON_R3:
				valuesCurrent.buttonR3 = event->value;
				if(event->value)
					valuesRising.buttonR3 = 1;
				break;
			case BUTTON_OPTIONS:
				valuesCurrent.buttonPause = event->value;
				if(event->value)
					valuesRising.buttonPause = 1;
				break;
			case BUTTON_SHARE:
				valuesCurrent.buttonSelect = event->value;
				if(event->value)
					valuesRising.buttonSelect = 1;
				break;
			default:
				std::cout << "Unrecognized button id: " << (int)event->id << "value:" << event->value << std::endl;
				break;
		}
	}
}

bool JoystickState::getTriangleRising() {
	if (valuesRising.buttonTriangle) {
		valuesRising.buttonTriangle = false;
		return true;
	}
	return false;
}
bool JoystickState::getSquareRising() {
	if (valuesRising.buttonSquare) {
		valuesRising.buttonSquare = false;
		return true;
	}
	return false;
}
bool JoystickState::getCircleRising() {
	if (valuesRising.buttonCircle) {
		valuesRising.buttonCircle = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonXRising() {
	if (valuesRising.buttonX) {
		valuesRising.buttonX = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonL1Rising() {
	if (valuesRising.buttonL1) {
		valuesRising.buttonL1 = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonL2Rising() {
	if (valuesRising.buttonL2) {
		valuesRising.buttonL2 = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonL3Rising() {
	if (valuesRising.buttonL3) {
		valuesRising.buttonL3 = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonR1Rising() {
	if (valuesRising.buttonR1) {
		valuesRising.buttonR1 = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonR2Rising() {
	if (valuesRising.buttonR2) {
		valuesRising.buttonR2 = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonR3Rising() {
	if (valuesRising.buttonR3) {
		valuesRising.buttonR3 = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonPauseRising() {
	if (valuesRising.buttonPause) {
		valuesRising.buttonPause = false;
		return true;
	}
	return false;
}
bool JoystickState::getButtonSelectRising() {
	if (valuesRising.buttonSelect) {
		valuesRising.buttonSelect = false;
		return true;
	}
	return false;
}


void JoystickState::printState() {
	std::cout <<
	"Joystick L1:" << getL1() <<
	" L2:" << getL2() <<
	" L3:" << getL3() <<
	" R1:" << getR1() <<
	" R2:" << getR2() <<
	" R3:" << getR3() <<
	" X:" << getX() <<
	" Tri:" << getTriangle() <<
	" Squ:" << getSquare() <<
	" Cir:" << getCircle() <<
	" Pause:" << getPause() <<
	" Select:" << getSelect() <<
	"\tLX:" << getLX() <<
	"\tLY:" << getLY() <<
	"\tRX:" << getRX() <<
	"\tRY:" << getRY() <<
	"\tDX:" << getDX() <<
	"\tDY:" << getDY() <<
	std::endl;
}
