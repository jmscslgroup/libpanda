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
#ifndef DEVICETYPES_H
#define DEVICETYPES_H


// // This is for the PS4:
//typedef enum ButtonID {
//	BUTTON_X = 0,
//	BUTTON_CIRCLE = 1,
//	BUTTON_TRIANGLE = 2,
//	BUTTON_SQUARE = 3,
//	BUTTON_L1 = 4,
//	BUTTON_R1 = 5,
//	BUTTON_L2 = 6,
//	BUTTON_R2 = 7,
//	BUTTON_SHARE = 8,
//	BUTTON_OPTIONS = 9,
//	BUTTON_PS = 10,
//	BUTTON_L3 = 11,
//	BUTTON_R3 = 12,
//	BUTTON_TOUCHPAD = 200	// This is handled separately as a mouse event
//} ButtonID;
//
// typedef enum AxisID {
//	 AXIS_LX = 0,
//	 AXIS_LY = 1,
//	 AXIS_L2 = 2,
//	 AXIS_RX = 3,
//	 AXIS_RY = 4,
//	 AXIS_R2 = 5,
//	 AXIS_DX = 6,
//	 AXIS_DY = 7
// } AxisID;
 
/*
// This is for the PS5:
typedef enum ButtonID {
   BUTTON_X = 1,
   BUTTON_CIRCLE = 2,
   BUTTON_TRIANGLE = 3,
   BUTTON_SQUARE = 0,
   BUTTON_L1 = 4,
   BUTTON_R1 = 5,
   BUTTON_L2 = 6,
   BUTTON_R2 = 7,
   BUTTON_SHARE = 8,
   BUTTON_OPTIONS = 9,
   BUTTON_PS = 12,
   BUTTON_L3 = 10,
   BUTTON_R3 = 11,
   BUTTON_TOUCHPAD = 13	// on the PS5 this is handled here
} ButtonID;

typedef enum AxisID {
	AXIS_LX = 0,
	AXIS_LY = 1,
	AXIS_L2 = 3,
	AXIS_RX = 2,
	AXIS_RY = 5,
	AXIS_R2 = 4,
	AXIS_DX = 6,
	AXIS_DY = 7
} AxisID;
*/
// This is for the Logitech GamePad:
typedef enum ButtonID {
   BUTTON_X = 1,
   BUTTON_CIRCLE = 2,
   BUTTON_TRIANGLE = 3,
   BUTTON_SQUARE = 0,
   BUTTON_L1 = 4,
   BUTTON_R1 = 5,
   BUTTON_L2 = 6,
   BUTTON_R2 = 7,
   BUTTON_SHARE = 8,
   BUTTON_OPTIONS = 9,
   BUTTON_PS = 12,
   BUTTON_L3 = 10,
   BUTTON_R3 = 11,
   BUTTON_TOUCHPAD	// on the PS5 this is handled here
} ButtonID;

typedef enum AxisID {
	AXIS_LX = 0,
	AXIS_LY = 1,
//	AXIS_L2,
	AXIS_RX = 2,
	AXIS_RY = 3,
//	AXIS_R2,
	AXIS_DX = 4,
	AXIS_DY = 5
} AxisID;

typedef enum ButtonType {
	TYPE_BUTTON = 1,
	TYPE_AXIS = 2
} ButtonType;


// Structures

typedef struct DeviceEvent {
	unsigned int time;
	short int value;
	unsigned char type;
	unsigned char id;
	
	bool operator==(const DeviceEvent &other) const {
		return type == other.type && id == other.id;
	}

	bool operator<(const DeviceEvent &other) const {
		return type < other.type || (type == other.type && id < other.id);
	}
} DeviceEvent;


#endif
