/*
 Author: Matt Bunting
 Copyright (c) 2020 Arizona Board of Regents
 All rights reserved.

 Permission is hereby granted, without written agreement and without
 license or royalty fees, to use, copy, modify, and distribute this
 software and its documentation for any purpose, provided that the
 above copyright notice and the following two paragraphs appear in
 all copies of this software.

 IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
 FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
 ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
 IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 SUCH DAMAGE.

 THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
 IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
 TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 */

#include "panda/toyota.h"
#include <unistd.h>
#include <cstring> // memcpy


using namespace Panda;

CanFrame Panda::buildLkasHud(bool ldaAlert, unsigned char leftLane, unsigned char rightLane, bool barrier, bool twoBeeps, bool repeatedBeeps) {
	CanFrame frame;
	
	/* from https://github.com/commaai/opendbc/blob/master/toyota_rav4_2017_pt_generated.dbc
	 BO_ 1042 LKAS_HUD: 8 XXX
	 SG_ BARRIERS : 1|2@0+ (1,0) [0|3] "" XXX
SG_ RIGHT_LINE : 3|2@0+ (1,0) [0|3] "" XXX
SG_ LEFT_LINE : 5|2@0+ (1,0) [0|3] "" XXX
SG_ SET_ME_X01 : 7|2@0+ (1,0) [0|3] "" XXX
SG_ SET_ME_X01_2 : 11|2@0+ (1,0) [0|3] "" XXX
SG_ LDA_ALERT : 9|2@0+ (1,0) [0|3] "" XXX
SG_ TWO_BEEPS : 12|1@0+ (1,0) [0|1] "" XXX
SG_ ADJUSTING_CAMERA : 13|1@0+ (1,0) [0|1] "" XXX
SG_ LDA_MALFUNCTION : 15|1@0+ (1,0) [0|1] "" XXX
SG_ REPEATED_BEEPS : 32|1@0+ (1,0) [0|1] "" XXX
SG_ SET_ME_X0C : 23|8@0+ (1,0) [0|1] "" XXX
SG_ SET_ME_X2C : 47|8@0+ (1,0) [0|1] "" XXX
SG_ SET_ME_X38 : 55|8@0+ (1,0) [0|1] "" XXX
SG_ SET_ME_X02 : 63|8@0+ (1,0) [0|1] "" XXX
	 
	 BO_ 1042 DS12F03: 8 FCM
	  SG_ LKAINDI : 7|2@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LKAWLSL : 5|2@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LKAWLSR : 3|2@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LKAFCT : 15|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LKATUCT : 14|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LKACAMT : 13|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LDWEXIST : 10|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LKASPCND : 23|3@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LKAWTCS : 18|2@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LKASAUT : 16|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LDWBZ : 32|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LDAFCVB : 47|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ LDARDA : 46|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ SWSSENSD : 45|2@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ SWSSWD : 43|1@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ SWSRAD : 55|2@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ SWSFLD : 53|3@0+ (1,0) [0|0] "" Vector__XXX
	  SG_ SWSBUZ : 50|2@0+ (1,0) [0|0] "" Vector__XXX
	 */
	
	frame.bus = 0;
	frame.busTime = 0;
	frame.dataLength = 8;
	frame.messageID = 1042;	// LKAS_HUD
	*((uint64_t*)frame.data) = 0;
	
	bool BARRIERS = barrier;
	*((uint64_t*)frame.data) = ((uint64_t)BARRIERS << (1+1-2));
	char RIGHT_LANE = rightLane;
	*((uint64_t*)frame.data) |= ((uint64_t)(RIGHT_LANE & 0x03) << (3+1-2));
	char LEFT_LANE = leftLane;
	*((uint64_t*)frame.data) |= ((uint64_t)(LEFT_LANE & 0x03) << (5+1-2));
	
	//SET_ME_X01
	*((uint64_t*)frame.data) |= ((uint64_t)0x01 << (7+1-2));
	//SET_ME_X01_2
	*((uint64_t*)frame.data) |= ((uint64_t)0x01 << (11+1-2));
	
	bool LDA_ALERT = ldaAlert;
	*((uint64_t*)frame.data) |= ((uint64_t)LDA_ALERT << (9+1-2));
	bool TWO_BEEPS = twoBeeps;
	*((uint64_t*)frame.data) |= ((uint64_t)TWO_BEEPS << (12+1-1));
	bool ADJUSTING_CAMERA = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)ADJUSTING_CAMERA << (13+1-1));
	bool LDA_MALFUNCTION = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)LDA_MALFUNCTION << (15+1-1));
	bool REPEATED_BEEPS = repeatedBeeps;
	*((uint64_t*)frame.data) |= ((uint64_t)REPEATED_BEEPS << (32+1-1));
	
	//SET_ME_X0C
	*((uint64_t*)frame.data) |= ((uint64_t)0x0c << (23+1-8));
	//SET_ME_X2C
	*((uint64_t*)frame.data) |= ((uint64_t)0x2c << (47+1-8));
	//SET_ME_X38
	*((uint64_t*)frame.data) |= ((uint64_t)0x38 << (55+1-8));
	//SET_ME_X02
	*((uint64_t*)frame.data) |= ((uint64_t)0x02 << (63+1-8));
	
	
	
	return frame;
}

//CanFrame Panda::buildLdaAlert(bool LDA_ALERT, unsigned char leftLane, unsigned char rightLane, bool barrier) {
//	CanFrame frame = buildLkasHud();
//
//	*((uint64_t*)frame.data) |= ((uint64_t)LDA_ALERT << (9+1-2));
//
//	bool BARRIERS = barrier;
//	*((uint64_t*)frame.data) |= ((uint64_t)BARRIERS << (1+1-2));
//	char RIGHT_LANE = rightLane;
//	*((uint64_t*)frame.data) |= ((uint64_t)(RIGHT_LANE & 0x03) << (3+1-2));
//	char LEFT_LANE = leftLane;
//	*((uint64_t*)frame.data) |= ((uint64_t)(LEFT_LANE & 0x03) << (5+1-2));
//
//	return frame;
//}
//
//CanFrame Panda::buildTwoBeeps(bool enable) {
//	CanFrame frame = buildLkasHud();
//
//	*((uint64_t*)frame.data) |= ((uint64_t)enable << (12+1-1));
//
//	return frame;
//}

uint8_t Panda::toyotaChecksum(CanFrame& frame)
{
	unsigned char newdata[10];
	
	uint8_t sum = 0;
	sum += frame.messageID & 0xFF;
	sum += frame.messageID>>8;
	//sum += frame.bus;	// I have no idea if this is needed
	sum += frame.dataLength;
	
	for (int i = 0; i < frame.dataLength-1; i++) {
		sum += frame.data[i];
	}
	return sum;
}
CanFrame Panda::buildSteeringLTA( unsigned char count, int16_t steer, bool steerRequest ) {
    CanFrame frame;
    
    // https://github.com/commaai/opendbc/blob/master/toyota_nodsu_pt_generated.dbc
    //BO_ 401 STEERING_LTA: 8 XXX
    // SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX
    // SG_ SETME_X3 : 29|2@0+ (1,0) [0|3] "" XXX
    // SG_ PERCENTAGE : 39|8@0+ (1,0) [0|255] "" XXX
    // SG_ SETME_X64 : 47|8@0+ (1,0) [0|255] "" XXX
    // SG_ ANGLE : 55|8@0- (0.5,0) [0|255] "" XXX
    // SG_ STEER_ANGLE_CMD : 15|16@0- (0.0573,0) [-540|540] "" XXX
    // SG_ STEER_REQUEST_2 : 25|1@0+ (1,0) [0|1] "" XXX
    // SG_ LKA_ACTIVE : 26|1@0+ (1,0) [0|1] "" XXX
    // SG_ BIT : 30|1@0+ (1,0) [0|1] "" XXX
    // SG_ COUNTER : 6|6@0+ (1,0) [0|255] "" XXX
    // SG_ STEER_REQUEST : 0|1@0+ (1,0) [0|1] "" XXX
    // SG_ SETME_X1 : 7|1@0+ (1,0) [0|1] "" XXX
    
   // CM_ SG_ 401 PERCENTAGE "driver override percentage (0-100), very close to steeringPressed in OP";
   // CM_ SG_ 401 SETME_X64 "ramps to 0 smoothly then back on falling edge of STEER_REQUEST if BIT isn't 1";
   // CM_ SG_ 401 ANGLE "angle of car relative to lane center on LTA camera";
   // CM_ SG_ 401 STEER_ANGLE_CMD "desired angle, OEM steers up to 95 degrees, no angle limit but torque will bottom out";
   // CM_ SG_ 401 BIT "has correlation to STEER_REQUEST";
   // CM_ SG_ 401 STEER_REQUEST "enable bit for steering, 1 to steer, 0 to not";
   // CM_ SG_ 401 STEER_REQUEST_2 "enable bit for steering, 1 to steer, 0 to not";
   // CM_ SG_ 401 LKA_ACTIVE "1 when using LTA for LKA";
    
   frame.bus = 0;
   frame.busTime = 0;
   frame.dataLength = 8;
   frame.messageID = 401;    // STEERING_LTA
   *((uint64_t*)frame.data) = 0;
    
    
    char SETME_X3 = 0x3;
    *((uint64_t*)frame.data) |= ((uint64_t)(SETME_X3 & 0x03) << (29+1-2));
    char PERCENTAGE = 100;
    *((uint64_t*)frame.data) |= ((uint64_t)(PERCENTAGE & 0xFF) << (39+1-8));
    char SETME_X64 = 0x64;
    *((uint64_t*)frame.data) |= ((uint64_t)(SETME_X64 & 0xFF) << (47+1-8));
    char ANGLE = 0;
    *((uint64_t*)frame.data) |= ((uint64_t)(ANGLE & 0xFF) << (55+1-8));
    uint16_t STEER_ANGLE_CMD = steer;
    *((uint64_t*)frame.data) |= ((uint64_t)((0xFF00 & STEER_ANGLE_CMD) >> 8) << (15+(16/2)+1-16));
    *((uint64_t*)frame.data) |= ((uint64_t)(0x00FF & STEER_ANGLE_CMD)  << (23+(16/2)+1-16));
    bool STEER_REQUEST_2 = steerRequest;
    *((uint64_t*)frame.data) |= ((uint64_t)(STEER_REQUEST_2 & 0x01) << (25+1-1));
    bool LKA_ACTIVE = 0;    // ???? TODO HACK WIP HALP!
    *((uint64_t*)frame.data) |= ((uint64_t)(LKA_ACTIVE & 0x01) << (26+1-1));
    bool BIT = 0;
    *((uint64_t*)frame.data) |= ((uint64_t)(BIT & 0x01) << (30+1-1));
    unsigned char COUNT = count + 128;  // TODO: this doesn't make sense if it's only 6 bits long
    *((uint64_t*)frame.data) |= ((uint64_t)(COUNT & 0x3F) << (6+1-6));
    
    bool STEER_REQUEST = steerRequest;
    *((uint64_t*)frame.data) |= ((uint64_t)(STEER_REQUEST & 0x01) << (0+1-1));
    char SETME_X1 = 0x1;
    *((uint64_t*)frame.data) |= ((uint64_t)(SETME_X1 & 0x01) << (7+1-1));
    
    
    // Checksum at the end:
    *((uint64_t*)frame.data) |= ((uint64_t)toyotaChecksum(frame) << (63+1-8));
    
    return frame;
}
CanFrame Panda::buildSteeringLKA( unsigned char count, int16_t steer_torque, bool steerRequest, unsigned char lkaState ) {
	
	// max steer_torque: const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever
	CanFrame frame;
	/*
	BO_ 740 STEERING_LKA: 5 XXX
	 SG_ STEER_REQUEST : 0|1@0+ (1,0) [0|1] "" XXX
	 SG_ COUNTER : 6|6@0+ (1,0) [0|63] "" XXX
	 SG_ SET_ME_1 : 7|1@0+ (1,0) [0|1] "" XXX
	 SG_ STEER_TORQUE_CMD : 15|16@0- (1,0) [0|65535] "" XXX
	 SG_ LKA_STATE : 31|8@0+ (1,0) [0|255] "" XXX
	 SG_ CHECKSUM : 39|8@0+ (1,0) [0|255] "" XXX
	 */
	frame.bus = 0;
	frame.busTime = 0;
	frame.dataLength = 5;
	frame.messageID = 740;	// STEERING_LKA
	*((uint64_t*)frame.data) = 0;
	
	//bool STEER_REQUEST = 0;
	*((uint64_t*)frame.data) = ((uint64_t)steerRequest << (0+1-1));
	// count
	*((uint64_t*)frame.data) |= ((uint64_t)(count & 0x3F) << (6+1-6));
	// set_me_1
	*((uint64_t*)frame.data) |= ((uint64_t)0x01 << (7+1-1));
	// Steer_torque
	*((uint64_t*)frame.data) |= ((uint64_t)((0xFF00 & steer_torque) >> 8) << (15+(16/2)+1-16));
	*((uint64_t*)frame.data) |= ((uint64_t)(0x00FF & steer_torque)  << (23+(16/2)+1-16));
	
	//*((uint64_t*)frame.data) |= ((0xFFFF & (uint64_t)steer_torque) << (15+(16/2)+1-16));
	unsigned char LKA_STATE = lkaState;//0;
	*((uint64_t*)frame.data) |= ((uint64_t)LKA_STATE << (31+1-8));
	
	
	
	// Checksum at the end:
	*((uint64_t*)frame.data) |= ((uint64_t)toyotaChecksum(frame) << (39+1-8));
	
	return frame;
}

bool Panda::toyotaParseSteeringLKA(CanFrame* frame, unsigned char* count, int16_t* steerTorque, bool* steerRequest, unsigned char* lkaState ) {
    
    uint8_t checksum = toyotaChecksum(*frame);
    if(frame->dataLength != 5 || checksum != frame->data[4] || frame->messageID != 740) {
        printf("checksum = 0x%02X, toyotaChecksum() = 0x%02X\n", checksum, toyotaChecksum(*frame));
        return false;
    }
    
    *count = (*((uint64_t*)frame->data) >> (6+1-6)) & 0x3F;
    *steerRequest = (*((uint64_t*)frame->data) >> (0+1-1)) & 0x01;
    *lkaState = (*((uint64_t*)frame->data) >> (31+1-8)) & 0xFF;
    
    *steerTorque = (*((uint64_t*)frame->data)  >> (15+(16/2)+1-16 - 8)) & 0xFF00;
    *steerTorque |= (*((uint64_t*)frame->data)  >> (23+(16/2)+1-16)) & 0x00FF;
    
    
    return true;
}

CanFrame Panda::buildACC_CONTROL(double acc, bool permitBraking, bool releaseStandstill, bool miniCar, bool cancelRequest) {
	CanFrame frame;
	/*
	 BO_ 835 ACC_CONTROL: 8 DSU
	  SG_ ACCEL_CMD : 7|16@0- (0.001,0) [-20|20] "m/s2" HCU
	 SG_ SET_ME_X3 : 19|4@0+ (1,0) [0|15] "" XXX
	  SG_ DISTANCE : 20|1@0+ (1,0) [0|1] "" XXX
	  SG_ MINI_CAR : 21|1@0+ (1,0) [0|1] "" XXX
	 SG_ SET_ME_X01 : 23|2@0+ (1,0) [0|3] "" HCU
	 SG_ CANCEL_REQ : 24|1@0+ (1,0) [0|1] "" HCU
	  SG_ PERMIT_BRAKING : 30|1@0+ (1,0) [0|1] "" HCU
	  SG_ RELEASE_STANDSTILL : 31|1@0+ (1,0) [0|1] "" HCU
	  SG_ ACCEL_CMD_ALT : 47|8@0- (0.05,0) [0|0] "m/s^2" XXX
	 SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX
	 */
	frame.bus = 0;
	frame.busTime = 0;
	frame.dataLength = 8;
	frame.messageID = 835;	// LKAS_HUD
	*((uint64_t*)frame.data) = 0;
	
	//double acc = (192.0*256.0+254.0)*0.001;
	//acc = acc > 20 ? 20 : acc;
	//acc = acc < -20 ? -20 : acc;
	int16_t ACCEL_CMD = (acc*(1.0/0.001));
	*((uint64_t*)frame.data) = ((uint64_t)((0xFF00 & ACCEL_CMD) >> 8) << (7+(16/2)+1-16));
	*((uint64_t*)frame.data) |= ((uint64_t)(0x00FF & ACCEL_CMD)  << (15+(16/2)+1-16));
	// set_me_x3 LCDT2
	*((uint64_t*)frame.data) |= ((uint64_t)0x03 << (19+1-4));
	bool DISTANCE = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)DISTANCE << (20+1-1));
	bool MINI_CAR = miniCar;	// set to 1 from comma.ai joystick
	*((uint64_t*)frame.data) |= ((uint64_t)MINI_CAR << (21+1-1));
	// set_me_x01 ACCTYPE
	*((uint64_t*)frame.data) |= ((uint64_t)0x01 << (23+1-2));
	bool CANCEL_REQ = cancelRequest;
	*((uint64_t*)frame.data) |= ((uint64_t)CANCEL_REQ << (24+1-1));
	bool LCCWOK = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)LCCWOK << (25+1-1));
	bool LVSTP = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)LVSTP << (29+1-1));
	bool PERMIT_BRAKING = permitBraking;	// set to 1 from comma.ai joystick
	*((uint64_t*)frame.data) |= ((uint64_t)PERMIT_BRAKING << (30+1-1));
	bool RELEASE_STANDSTILL = releaseStandstill;	// set to 1 from comma.ai joystick
	*((uint64_t*)frame.data) |= ((uint64_t)RELEASE_STANDSTILL << (31+1-1));
	unsigned char CACC = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)CACC << (39+1-8));
	
	char ACCEL_CMD_ALT = (acc*(1.0/0.05)) * 0;
	*((uint64_t*)frame.data) |= ((uint64_t)ACCEL_CMD_ALT << (47+1-8));
	

	
	// Checksum at the end:
	*((uint64_t*)frame.data) |= ((uint64_t)toyotaChecksum(frame) << (63+1-8));
	
	return frame;
}

CanFrame Panda::buildPCM_CRUISE_2(unsigned char SET_SPEED) {	// in kph
	CanFrame frame;
	frame.bus = 0;
	frame.busTime = 0;
	frame.dataLength = 8;
	frame.messageID = 467;
	*((uint64_t*)frame.data) = 0;
	/*
BO_ 467 PCM_CRUISE_2: 8 XXX
 SG_ MAIN_ON : 15|1@0+ (1,0) [0|1] "" XXX
 SG_ LOW_SPEED_LOCKOUT : 14|2@0+ (1,0) [0|3] "kph" XXX
 SG_ SET_SPEED : 23|8@0+ (1,0) [0|255] "kph" XXX
 SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX
	 
	 
BO_ 467 ENG2F05: 8 CGW
  SG_ LCCW2 : 4|1@0+ (1,0) [0|0] "" DS1,FCM
  SG_ WSTL2 : 3|1@0+ (1,0) [0|0] "" DS1
  SG_ LCCHK : 2|3@0+ (1,0) [0|0] "" DS1
  SG_ XCCOK2 : 15|1@0+ (1,0) [0|0] "" DS1
  SG_ SLTACC : 14|2@0+ (1,0) [0|0] "" DS1
  SG_ LTME2 : 12|2@0+ (1,0) [0|0] "" DS1
  SG_ STPSWF2 : 10|1@0+ (1,0) [0|0] "" DS1
  SG_ CCSF : 9|1@0+ (1,0) [0|0] "" DS1
  SG_ CCSNG : 8|1@0+ (1,0) [0|0] "" DS1
  SG_ VM : 23|16@0+ (0.00390625,0) [0|0] "km/h" DS1
  SG_ INTG : 39|8@0- (0.04,0) [0|0] "m/s^2" DS1
  SG_ D2PRXMK : 47|1@0+ (1,0) [0|0] "" DS1
  SG_ SM1D3 : 63|8@0+ (1,0) [0|0] "" DS1,FCM
*/
	
	unsigned char LOW_SPEED_LOCKOUT = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)LOW_SPEED_LOCKOUT << (14+1-2));
	bool MAIN_ON = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)MAIN_ON << (15+1-1));
	//bool SET_SPEED = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)SET_SPEED << (23+1-8));
	
	// Checksum at the end:
	*((uint64_t*)frame.data) |= ((uint64_t)toyotaChecksum(frame) << (63+1-8));

	return frame;
}

CanFrame Panda::buildDSU_CRUISE(unsigned char SET_SPEED) {	// in km/h
	CanFrame frame;
	frame.bus = 0;
	frame.busTime = 0;
	frame.dataLength = 7;
	frame.messageID = 869;
	*((uint64_t*)frame.data) = 0;
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
	
	bool MAIN_ON = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)MAIN_ON << (0+1-1));
	bool CANCEL_BTN = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)CANCEL_BTN << (1+1-1));
	bool SET_BTN = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)SET_BTN << (2+1-1));
	bool RES_BTN = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)RES_BTN << (3+1-1));
	//bool SET_SPEED = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)SET_SPEED << (15+1-8));
	unsigned char CRUISE_REQUEST = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)CRUISE_REQUEST << (31+1-8));
	unsigned char LEAD_DISTANCE = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)LEAD_DISTANCE << (39+1-8));

	
	// Checksum at the end:
	//*((uint64_t*)frame.data) |= ((uint64_t)checksum(frame) << (63+1-8));

	return frame;
}

CanFrame Panda::buildTRACK_B_1(unsigned char count) {	// in km/h
	CanFrame frame;
	
   frame.bus = 0;
   frame.busTime = 0;
   frame.dataLength = 8;
   frame.messageID = 401;
   *((uint64_t*)frame.data) = 0;
	
	/*
	 BO_ 401 TRACK_B_1: 8 XXX
	  SG_ COUNTER : 7|8@0+ (1,0) [0|255] "" XXX
	  SG_ REL_ACCEL : 15|7@0- (1,0) [-64|63] "" XXX
	  SG_ SCORE : 23|8@0+ (1,0) [0|100] "" XXX
	  SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX
	 */
	
	
	// count
	*((uint64_t*)frame.data) |= ((uint64_t)count << (7+1-8));
	unsigned char REL_ACCEL = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)(REL_ACCEL & 0x7F) << (15+1-7));
	unsigned char SCORE = 0;
	*((uint64_t*)frame.data) |= ((uint64_t)SCORE << (23+1-8));
	
	// Checksum
	*((uint64_t*)frame.data) |= ((uint64_t)toyotaChecksum(frame) << (63+1-8));

	return frame;
}

void Panda::printFrame( CanFrame frame ) {
	printf("%d ", frame.messageID);
	printf("%d ", frame.bus);
	for(int i = 0; i < frame.dataLength; i++) {
		printf("%02X ", (int)frame.data[i]);
	}
	printf("\n");
}
