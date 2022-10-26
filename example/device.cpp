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

#include "device.h"

#include <iostream>
#include <cstring>

void Device::init(const char* filename, int bytesPerFrame) {
	file = fopen( filename, "r" );

	if( file == NULL ) {
		std::cerr << "Couldn't open " << filename << std::endl;
	}
	
	this->bytesPerFrame = bytesPerFrame;
	this->buffer = new char[bytesPerFrame];
}

Device::~Device() {
	stop();
	WaitForInternalThreadToExit();
	fclose( file );
	
	if( buffer != NULL) {
		delete [] buffer;
	}
}

void Device::addObserver(DeviceObserver* observer) {
	this->observer = observer;
}

void Device::doAction() {
	//printf("Read: " );
	if (file == NULL) {
		stop();
		return;
	}
	
	for (int i = 0; i < bytesPerFrame; i++) {
		buffer[i] = fgetc( file );
		//printf("%d ", (int)buffer[i]);
	}
	//printf("\n");
	
	DeviceEvent event;
	
	interpret( buffer, &event );
	
	if (observer != NULL &&
		event.time != -1) {
		observer->newDeviceEvent( &event );
	}
}
