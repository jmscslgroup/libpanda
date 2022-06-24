/*
 Author: Matt Bunting
 */

#include "panda/nissan.h"

#include <cstring>

using namespace Panda;



CanFrame Panda::buildCanNissanCruiseButtons(bool button) {	// in km/h
	CanFrame frame;
	frame.bus = 0;
	frame.busTime = 0;
	frame.dataLength = 7;		// TODO:
	frame.messageID = 869;		// TODO:
	
	memset(frame.data, 0, frame.dataLength);

	// TODO: build the message:
	
	
	/*
	 BO_ 869 DSU_CRUISE : 7 DSU
	 SG_ MAIN_ON : 0|1@0+ (1,0) [0|0] "" XXX
	 SG_ CANCEL_BTN : 1|1@0+ (1,0) [0|0] "" XX
	 SG_ SET_BTN : 2|1@0+ (1,0) [0|0] "" XXXX
	  SG_ RES_BTN : 3|1@0+ (1,0) [0|0] "" XXX
	  SG_ SET_SPEED : 15|8@0+ (1,0) [0|0] "km/h" XXX
	  SG_ CRUISE_REQUEST : 31|8@0+ (100,-12800) [0|0] "N" XXX
	  SG_ LEAD_DISTANCE : 39|8@0+ (1,0) [0|0] "m" XXX
	 */
	
//	bool MAIN_ON = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)MAIN_ON << (0+1-1));
//	bool CANCEL_BTN = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)CANCEL_BTN << (1+1-1));
//	bool SET_BTN = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)SET_BTN << (2+1-1));
//	bool RES_BTN = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)RES_BTN << (3+1-1));
//	//bool SET_SPEED = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)SET_SPEED << (15+1-8));
//	unsigned char CRUISE_REQUEST = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)CRUISE_REQUEST << (31+1-8));
//	unsigned char LEAD_DISTANCE = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)LEAD_DISTANCE << (39+1-8));

	
	// Checksum at the end:
	//*((uint64_t*)frame.data) |= ((uint64_t)checksum(frame) << (63+1-8));

	return frame;
}
