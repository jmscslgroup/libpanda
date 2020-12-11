
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

void Mouse::open(const char* filename) {
	init(filename, 3);
}

void Mouse::interpret( char* buffer, DeviceEvent* event) {
	if(	(buffer[0] == 8 || buffer[0] == 9) &&
		buffer[1] == 0 &&
		buffer[2] == 0 ) {
		event->time = 0;
		event->type = TYPE_BUTTON;
		event->id = BUTTON_TOUCHPAD;
		event->value = buffer[0] == 9 ? 1 : 0;
	} else {
		event->time = -1;
		event->type = -1;
	}
}
