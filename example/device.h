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
#ifndef DEVICE_H
#define DEVICE_H

#include <cstdio>		// for FILE
#include <mogi/thread.h>

#include "deviceTypes.h"


// Classes

class DeviceObserver {
public:
	virtual void newDeviceEvent( const DeviceEvent* event ) = 0;
};


// Handles the input device through file reading (from /dev/input/* )
class Device : public Mogi::Thread {
private:
	FILE* file = NULL;
	
	DeviceObserver* observer = NULL;
	
	int bytesPerFrame = 8;
	char* buffer = NULL;
	
	void doAction();	//override
	
protected:
	void init(const char* filename, int bytesPerFrame);
	
public:
	virtual ~Device() = 0;
	
	
	void addObserver(DeviceObserver* observer);
	
	virtual void interpret( char* buffer, DeviceEvent* event) = 0;
	
};


class Joystick : public Device {
private:
	void interpret( char* buffer, DeviceEvent* event);	//override
	
public:
	~Joystick() {};
	void open(const char* filename);
};


class Mouse : public Device {
private:
	void interpret( char* buffer, DeviceEvent* event);	//override
	
public:
	~Mouse() {};
	void open(const char* filename);
};

#endif
