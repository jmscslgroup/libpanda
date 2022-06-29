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


/*
BO_ 740 STEERING_LKA: 5 XXX
 SG_ STEER_REQUEST : 0|1@0+ (1,0) [0|1] "" XXX
 SG_ COUNTER : 6|6@0+ (1,0) [0|63] "" XXX
 SG_ SET_ME_1 : 7|1@0+ (1,0) [0|1] "" XXX
 SG_ STEER_TORQUE_CMD : 15|16@0- (1,0) [0|65535] "" XXX
 SG_ LKA_STATE : 31|8@0+ (1,0) [0|255] "" XXX
 SG_ CHECKSUM : 39|8@0+ (1,0) [0|255] "" XXX
 */
//frame.bus = 0;
//frame.busTime = 0;
//frame.dataLength = 5;
//frame.messageID = 740;	// LKAS_HUD
//*((uint64_t*)frame.data) = 0;
//
////bool STEER_REQUEST = 0;
//*((uint64_t*)frame.data) = ((uint64_t)steerRequest << (0+1-1));
//// count
//*((uint64_t*)frame.data) |= ((uint64_t)(count & 0x3F) << (6+1-6));
//// set_me_1
//*((uint64_t*)frame.data) |= ((uint64_t)0x01 << (7+1-1));
//// Steer_torque
//*((uint64_t*)frame.data) |= ((uint64_t)((0xFF00 & steer_torque) >> 8) << (15+(16/2)+1-16));
//*((uint64_t*)frame.data) |= ((uint64_t)(0x00FF & steer_torque)  << (23+(16/2)+1-16));


void stuffCanData(char* payload, int location, int sizeInBits, uint64_t* data, uint64_t mask) {
	// TODO:
}

void Panda::replaceCanThreeOhThree( CanFrame* frame, int WHEEL_TORQUE, int speed ) {
	CanFrame newFrame = buildCanThreeOhThree( WHEEL_TORQUE, speed );
	
	frame->data[10] &= 0x03;	// speed
	frame->data[9] &= 0xC0;
	
	frame->data[8] &= 0x03;	// torque
	frame->data[7] &= 0xC0;
	
	for (int i = 0; i < frame->dataLength; i++) {
		frame->data[i] |= newFrame.data[i];
	}
}

CanFrame Panda::buildCanThreeOhThree( int WHEEL_TORQUE, int speed ) {
	CanFrame frame;
	frame.bus = 2;	// This is with a custom connector
	frame.busTime = 0;
	frame.dataLength = 12;
	frame.messageID = 303;
	
	memset(frame.data, 0, frame.dataLength);
	
//	1350645 	1656362686.4646 	1.0000 	303.0000 	00012f07a160260d260f9600 	12.0000
//	1350674 	1656362686.4747 	1.0000 	303.0000 	00012f079070260d2a0f9e00 	12.0000
//	1350697 	1656362686.4850 	1.0000 	303.0000 	00012f07ba80260d420fae00 	12.0000
//	1350726 	1656362686.4949 	1.0000 	303.0000 	00012f07c090260d360fb400 	12.0000
//
//	torque = [841, 842, 848, 845]
//	that's starting at bit 59
//
//	s = [997, 999, 1003, 1005]
//	that's starting at bit 75
//	the actual speed value is about x/20
	
	// old:
	//  BO_ 303 SPEED: 24 XXX
	//	 SG_ WHEEL_TORQUE: 60|12@0+ (1,0) [0|4000] “” XXX
	//	 SG_ SPEED : 77|12@0+ (0.05,0) [0|65535] “kph” XXX
	// new:
	//	BO_ 303 SPEED: 12 XXX
	//	 SG_ WHEEL_TORQUE: 60|12@0+ (1,0) [0|4000] “” XXX
	//	 SG_ SPEED : 76|12@0+ (0.05,0) [0|65535] “kph” XXX
	
//	*((uint64_t*)&frame.data[5]) |= ((uint64_t)WHEEL_TORQUE << (60+1-12 - 5*8));
//	*((uint64_t*)&frame.data[8]) |= ((uint64_t)speed << (76+1-12 - 8*8));
	
//	printf("WHEEL_TORQUE = 0x%03X\n", WHEEL_TORQUE);
	*((uint64_t*)&frame.data[5]) |= ((uint64_t)(0x003F & (WHEEL_TORQUE >> 6)) << (60+(16/2)+1 - 12 - 5*8 -1));	// odd masking since this lands on half a byte
	*((uint64_t*)&frame.data[5]) |= ((uint64_t)(0x003F & WHEEL_TORQUE) << (60+8+(16/2)+1 - 12 - 5*8 -1 + 2));

	
//	printf("SPEED = 0x%03X\n", speed);
	
	//	*((uint64_t*)&frame.data[8]) |= ((uint64_t)speed << (76+1-12 - 8*8));
	*((uint64_t*)&frame.data[8]) |= ((uint64_t)(0x003F & (speed >> 6)) << (76+(16/2)+1 - 12 - 8*8 -1));	// odd masking since this lands on half a byte
	*((uint64_t*)&frame.data[8]) |= ((uint64_t)(0x003F & speed) << (76+8+(16/2)+1 - 12 - 8*8 -1 + 2));
	
	return frame;
}


void Panda::nissanParseThreeOhThree( CanFrame& frame, int* WHEEL_TORQUE, int* speed  ) {
	*WHEEL_TORQUE = 0;
	*WHEEL_TORQUE |= (frame.data[10] >> 2) & 0x3F;
	*WHEEL_TORQUE |= ((int)frame.data[9] << 6) & 0x0FC0;	// speed
	
	*speed = 0;
	*speed |= (frame.data[8] >> 2) & 0x3F;
	*speed |= (((int)frame.data[7])  << 6) & 0x00FC0;	// speed
}
