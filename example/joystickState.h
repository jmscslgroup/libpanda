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
#ifndef JOYSTICK_STATE_H
#define JOYSTICK_STATE_H

#include "device.h"

typedef struct JoystickValues {
	
	double axisRX;
	double axisRY;
	double axisLX;
	double axisLY;
	double axisDX;
	double axisDY;
	bool buttonTriangle;
	bool buttonSquare;
	bool buttonCircle;
	bool buttonX;
	bool buttonL1;
	bool buttonL2;
	bool buttonL3;
	bool buttonR1;
	bool buttonR2;
	bool buttonR3;
	bool buttonPause;
	bool buttonSelect;
} JoystickValues;

// Classes
class JoystickState : public DeviceObserver {
private:
	void newDeviceEvent( const DeviceEvent* event );	// override from DeviceObserver
	
JoystickValues valuesRising;
JoystickValues valuesCurrent;

public:
	JoystickState();

	// Get Joystick axis:
	double getRX() { return valuesCurrent.axisRX; }
	double getRY() { return valuesCurrent.axisRY; }
	double getLX() { return valuesCurrent.axisLX; }
	double getLY() { return valuesCurrent.axisLY; }
	double getDX() { return valuesCurrent.axisDX; }
	double getDY() { return valuesCurrent.axisDY; }
	
	// Get current button states:
	bool getTriangle() { return valuesCurrent.buttonTriangle; }
	bool getSquare() { return valuesCurrent.buttonSquare; }
	bool getCircle() { return valuesCurrent.buttonCircle; }
	bool getX() { return valuesCurrent.buttonX; };
	bool getL1() { return valuesCurrent.buttonL1; };
	bool getL2() { return valuesCurrent.buttonL2; };
	bool getL3() { return valuesCurrent.buttonL3; };
	bool getR1() { return valuesCurrent.buttonR1; };
	bool getR2() { return valuesCurrent.buttonR2; };
	bool getR3() { return valuesCurrent.buttonR3; };
	bool getPause() { return valuesCurrent.buttonPause; };
	bool getSelect() { return valuesCurrent.buttonSelect; };
	
	// Get a rising edge if it ever occured in the past
	// Once this is called, the rising edge check is reset
	bool getTriangleRising();
	bool getSquareRising();
	bool getCircleRising();
	bool getButtonXRising();
	bool getButtonL1Rising();
	bool getButtonL2Rising();
	bool getButtonL3Rising();
	bool getButtonR1Rising();
	bool getButtonR2Rising();
	bool getButtonR3Rising();
	bool getButtonPauseRising();
	bool getButtonSelectRising();

};
#endif
