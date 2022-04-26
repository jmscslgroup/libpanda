///*
// Author: Matt Bunting
// Copyright (c) 2020 Arizona Board of Regents
// All rights reserved.
//
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that the
// above copyright notice and the following two paragraphs appear in
// all copies of this software.
//
// IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
// FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
// ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
// IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.
//
// THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
// IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
// TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
// */
//
//#include "panda/nissan.h"
//#include <unistd.h>
//#include <cstring> // memcpy
//
//
//// There exists no "Rogue" DPC file, however openpilot references the x-trail.  The following informaiton from:
//// https://github.com/commaai/opendbc/blob/master/nissan_x_trail_2017.dbc
//
//using namespace Panda;
//using namespace Nissan;
//
//CanFrame Panda::Nissan::buildLkasHud(bool hand_on_wheel_warning) {
//	CanFrame frame;
//
//	/*
//	 BO_ 689 PROPILOT_HUD: 8 XXX
//	  SG_ LARGE_WARNING_FLASHING : 9|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_RADAR_ERROR_FLASHING1 : 10|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_RADAR_ERROR_FLASHING2 : 11|1@0+ (1,0) [0|1] "" XXX
//	  SG_ RIGHT_LANE_YELLOW_FLASH : 12|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LEFT_LANE_YELLOW_FLASH : 13|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LEAD_CAR : 14|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LEAD_CAR_ERROR : 15|1@0+ (1,0) [0|1] "" XXX
//	  SG_ FRONT_RADAR_ERROR : 16|1@0+ (1,0) [0|1] "" XXX
//	  SG_ FRONT_RADAR_ERROR_FLASHING : 17|1@0+ (1,0) [0|1] "" XXX
//	  SG_ RIGHT_LANE_GREEN : 24|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LEFT_LANE_GREEN : 25|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_RADAR_ERROR_FLASHING3 : 27|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LKAS_ERROR_FLASHING : 29|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SAFETY_SHIELD_ACTIVE : 44|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LARGE_STEERING_WHEEL_ICON : 61|2@0+ (1,0) [0|3] "" XXX
//	  SG_ RIGHT_LANE_GREEN_FLASH : 62|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LEFT_LANE_GREEN_FLASH : 63|1@0+ (1,0) [0|1] "" XXX
//	  SG_ FOLLOW_DISTANCE : 3|2@0+ (1,0) [0|3] "" XXX
//	  SG_ AUDIBLE_TONE : 47|3@0+ (1,0) [0|8] "" XXX
//	  SG_ SPEED_SET_ICON : 7|2@0+ (1,0) [0|3] "" XXX
//	  SG_ SMALL_STEERING_WHEEL_ICON : 42|3@0+ (1,0) [0|7] "" XXX
//	  SG_ SET_SPEED : 39|8@0+ (1,0) [0|255] "" XXX
//	  SG_ unknown02 : 1|2@0+ (1,0) [0|3] "" XXX
//	  SG_ unknown05 : 5|2@0+ (1,0) [0|3] "" XXX
//	  SG_ unknown08 : 8|7@0+ (1,0) [0|63] "" XXX
//	  SG_ unknown26 : 26|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown28 : 28|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown31 : 31|2@0+ (1,0) [0|3] "" XXX
//	  SG_ unknown43 : 43|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown55 : 55|8@0+ (1,0) [0|63] "" XXX
//	  SG_ unknown59 : 59|4@0+ (1,0) [0|15] "" XXX
//	 */
//
//	frame.bus = 0;
//	frame.busTime = 0;
//	frame.dataLength = 8;
//	frame.messageID = 689;	// PROPILOT_HUD
//	*((uint64_t*)frame.data) = 0;
//
//	//SET_ME_X02
//	*((uint64_t*)frame.data) |= ((uint64_t)0x02 << (63+1-8));
//
//	return frame;
//}
//
//CanFrame Panda::Nissan::buildLkasHudWheel(bool hand_on_wheel_warning) {
//	CanFrame frame;
//
//	/* from https://github.com/commaai/opendbc/blob/master/nissan_x_trail_2017.dbc
//	 BO_ 1228 PROPILOT_HUD_INFO_MSG: 8 XXX
//	  SG_ NA_HIGH_ACCEL_TEMP : 0|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_RADAR_NA_HIGH_CABIN_TEMP : 8|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_RADAR_MALFUNCTION : 11|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LKAS_MALFUNCTION : 12|1@0+ (1,0) [0|1] "" XXX
//	  SG_ FRONT_RADAR_MALFUNCTION : 13|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_RADAR_NA_CLEAN_REAR_CAMERA : 14|1@0+ (1,0) [0|1] "" XXX
//	  SG_ NA_POOR_ROAD_CONDITIONS : 16|1@0+ (1,0) [0|1] "" XXX
//	  SG_ CURRENTLY_UNAVAILABLE : 17|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SAFETY_SHIELD_OFF : 18|1@0+ (1,0) [0|1] "" XXX
//	  SG_ FRONT_COLLISION_NA_FRONT_RADAR_OBSTRUCTION : 20|1@0+ (1,0) [0|1] "" XXX
//	  SG_ PEDAL_MISSAPPLICATION_SYSTEM_ACTIVATED : 24|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_IMPACT_NA_RADAR_OBSTRUCTION : 25|1@0+ (1,0) [0|1] "" XXX
//	  SG_ WARNING_DO_NOT_ENTER : 33|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_IMPACT_SYSTEM_OFF : 34|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_IMPACT_MALFUNCTION : 35|1@0+ (1,0) [0|1] "" XXX
//	  SG_ FRONT_COLLISION_MALFUNCTION : 36|1@0+ (1,0) [0|1] "" XXX
//	  SG_ SIDE_RADAR_MALFUNCTION2 : 37|1@0+ (1,0) [0|1] "" XXX
//	  SG_ LKAS_MALFUNCTION2 : 38|1@0+ (1,0) [0|1] "" XXX
//	  SG_ FRONT_RADAR_MALFUNCTION2 : 39|1@0+ (1,0) [0|1] "" XXX
//	  SG_ PROPILOT_NA_MSGS : 42|3@0+ (1,0) [0|7] "" XXX
//	  SG_ BOTTOM_MSG : 45|3@0+ (1,0) [0|7] "" XXX
//	  SG_ HANDS_ON_WHEEL_WARNING : 47|1@0+ (1,0) [0|1] "" XXX
//	  SG_ WARNING_STEP_ON_BRAKE_NOW : 51|1@0+ (1,0) [0|1] "" XXX
//	  SG_ PROPILOT_NA_FRONT_CAMERA_OBSTRUCTED : 52|1@0+ (1,0) [0|1] "" XXX
//	  SG_ PROPILOT_NA_HIGH_CABIN_TEMP : 53|1@0+ (1,0) [0|1] "" XXX
//	  SG_ WARNING_PROPILOT_MALFUNCTION : 54|1@0+ (1,0) [0|3] "" XXX
//	  SG_ ACC_UNAVAILABLE_HIGH_CABIN_TEMP : 62|1@0+ (1,0) [0|1] "" XXX
//	  SG_ ACC_NA_FRONT_CAMERA_IMPARED : 63|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown07 : 7|7@0+ (1,0) [0|127] "" XXX
//	  SG_ unknown10 : 10|2@0+ (1,0) [0|3] "" XXX
//	  SG_ unknown15 : 15|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown23 : 23|3@0+ (1,0) [0|7] "" XXX
//	  SG_ unknown19 : 19|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown31 : 31|6@0+ (1,0) [0|63] "" XXX
//	  SG_ unknown32 : 32|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown46 : 46|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown50 : 50|3@0+ (1,0) [0|7] "" XXX
//	  SG_ unknown55 : 55|1@0+ (1,0) [0|1] "" XXX
//	  SG_ unknown61 : 61|6@0+ (1,0) [0|63] "" XXX
//	 */
//
//	frame.bus = 0;
//	frame.busTime = 0;
//	frame.dataLength = 8;
//	frame.messageID = 1228;	// PROPILOT_HUD_INFO_MSG
//	*((uint64_t*)frame.data) = 0;
//
//	bool BARRIERS = barrier;
//	*((uint64_t*)frame.data) = ((uint64_t)BARRIERS << (1+1-2));
//	char RIGHT_LANE = rightLane;
//	*((uint64_t*)frame.data) |= ((uint64_t)(RIGHT_LANE & 0x03) << (3+1-2));
//	char LEFT_LANE = leftLane;
//	*((uint64_t*)frame.data) |= ((uint64_t)(LEFT_LANE & 0x03) << (5+1-2));
//
//	//SET_ME_X01
//	*((uint64_t*)frame.data) |= ((uint64_t)0x01 << (7+1-2));
//	//SET_ME_X01_2
//	*((uint64_t*)frame.data) |= ((uint64_t)0x01 << (11+1-2));
//
//	bool LDA_ALERT = ldaAlert;
//	*((uint64_t*)frame.data) |= ((uint64_t)LDA_ALERT << (9+1-2));
//	bool TWO_BEEPS = twoBeeps;
//	*((uint64_t*)frame.data) |= ((uint64_t)TWO_BEEPS << (12+1-1));
//	bool ADJUSTING_CAMERA = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)ADJUSTING_CAMERA << (13+1-1));
//	bool LDA_MALFUNCTION = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)LDA_MALFUNCTION << (15+1-1));
//	bool REPEATED_BEEPS = repeatedBeeps;
//	*((uint64_t*)frame.data) |= ((uint64_t)REPEATED_BEEPS << (32+1-1));
//
//	//SET_ME_X0C
//	*((uint64_t*)frame.data) |= ((uint64_t)0x0c << (23+1-8));
//	//SET_ME_X2C
//	*((uint64_t*)frame.data) |= ((uint64_t)0x2c << (47+1-8));
//	//SET_ME_X38
//	*((uint64_t*)frame.data) |= ((uint64_t)0x38 << (55+1-8));
//	//SET_ME_X02
//	*((uint64_t*)frame.data) |= ((uint64_t)0x02 << (63+1-8));
//
//
//
//	return frame;
//}
//
////CanFrame Panda::buildLdaAlert(bool LDA_ALERT, unsigned char leftLane, unsigned char rightLane, bool barrier) {
////	CanFrame frame = buildLkasHud();
////
////	*((uint64_t*)frame.data) |= ((uint64_t)LDA_ALERT << (9+1-2));
////
////	bool BARRIERS = barrier;
////	*((uint64_t*)frame.data) |= ((uint64_t)BARRIERS << (1+1-2));
////	char RIGHT_LANE = rightLane;
////	*((uint64_t*)frame.data) |= ((uint64_t)(RIGHT_LANE & 0x03) << (3+1-2));
////	char LEFT_LANE = leftLane;
////	*((uint64_t*)frame.data) |= ((uint64_t)(LEFT_LANE & 0x03) << (5+1-2));
////
////	return frame;
////}
////
////CanFrame Panda::buildTwoBeeps(bool enable) {
////	CanFrame frame = buildLkasHud();
////
////	*((uint64_t*)frame.data) |= ((uint64_t)enable << (12+1-1));
////
////	return frame;
////}
//
//uint8_t Panda::Nissan::checksum(CanFrame& frame)
//{
////	unsigned char newdata[10];
//
////	uint8_t sum = 0;
////	sum += frame.messageID & 0xFF;
////	sum += frame.messageID>>8;
////	//sum += frame.bus;	// I have no idea if this is needed
////	sum += frame.dataLength;
////
////	for (int i = 0; i < frame.dataLength; i++) {
////		sum += frame.data[i];
////	}
////	return sum;
//
//#define POLY (0x1d) // comma.ai has value 0x11d, but I assumt that 0x1** is implied
//	uint8_t crc = 0;
//
////	unsigned char newdata[10];
//
//	for (int i = 0; i < frame.dataLength; i++) {
//	  {
//		crc = crc ^ *frame.data[i]; // Apply Byte
//
//		for(int j=0; j<8; j++) // Eight rounds of 1-bit
//		{
//		  if (crc & 0x80)
//			crc = (crc << 1) ^ POLY;
//		  else
//			crc = (crc << 1); // Left Shifting
//		}
//	  }
//
//	  return(crc);
//}
//
//CanFrame Panda::buildSteeringLKA( unsigned char max_torque, bool lka_active, unsigned char counter, double desired_angle,  unsigned char counter ) {
//
//	CanFrame frame;
//	/*
//	BO_ 740 STEERING_LKA: 5 XXX
//	 SG_ STEER_REQUEST : 0|1@0+ (1,0) [0|1] "" XXX
//	 SG_ COUNTER : 6|6@0+ (1,0) [0|63] "" XXX
//	 SG_ SET_ME_1 : 7|1@0+ (1,0) [0|1] "" XXX
//	 SG_ STEER_TORQUE_CMD : 15|16@0- (1,0) [0|65535] "" XXX
//	 SG_ LKA_STATE : 31|8@0+ (1,0) [0|255] "" XXX
//	 SG_ CHECKSUM : 39|8@0+ (1,0) [0|255] "" XXX
//
//	 BO_ 361 LKAS: 8 XXX
//	  SG_ MAX_TORQUE : 39|8@0+ (0.01,0) [0|255] "Nm" XXX
//	  SG_ SET_0x80 : 47|8@0+ (1,0) [0|255] "" XXX
//	  SG_ LKA_ACTIVE : 52|1@0+ (1,0) [0|15] "" XXX
//	  SG_ SET_0x80_2 : 31|8@0+ (1,0) [0|255] "" XXX
//	  SG_ COUNTER : 51|4@0+ (1,0) [0|15] "" XXX
//	  SG_ DESIRED_ANGLE : 7|18@0+ (-0.01,1310) [0|255] "" XXX
//	  SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX
//	 */
//	frame.bus = 0;
//	frame.busTime = 0;
//	frame.dataLength = 8;
//	frame.messageID = 361;	// LKAS
//	*((uint64_t*)frame.data) = 0;
//
//	int location, length;
//
//	//unsigned char max_torque.  units of Nm
//	location = 39;
//	length = 8;
//	*((uint64_t*)frame.data) = ((uint64_t)max_torque << (location+1-length));
//	// SET_0x80
//	location = 47;
//	length = 8;
//	*((uint64_t*)frame.data) |= ((uint64_t)0x80 << (location+1-length));
//	// LKA_ACTIVE
//	location = 52;
//	length = 1;
//	*((uint64_t*)frame.data) |= ((uint64_t)lka_active << (location+1-length));
//	// SET_0x80_2
//	location = 31;
//	length = 8;
//	*((uint64_t*)frame.data) |= ((uint64_t)0x80 << (location+1-length));
//	// COUNTER
//	location = 51;
//	length = 4;
//	*((uint64_t*)frame.data) |= ((uint64_t)(counter & 0x0F) << (location+1-length));
////	*((uint64_t*)frame.data) |= ((uint64_t)((0xFF00 & steer_torque) >> 8) << (15+(16/2)+1-16));
////	*((uint64_t*)frame.data) |= ((uint64_t)(0x00FF & steer_torque)  << (23+(16/2)+1-16));
//
//	//*((uint64_t*)frame.data) |= ((0xFFFF & (uint64_t)steer_torque) << (15+(16/2)+1-16));
//	unsigned char LKA_STATE = lkaState;//0;
//	*((uint64_t*)frame.data) |= ((uint64_t)LKA_STATE << (31+1-8));
//
//
//
//	// Checksum at the end:
//	*((uint64_t*)frame.data) |= ((uint64_t)Nissan::checksum(frame) << (39+1-8));
//
//	return frame;
//}
//
//CanFrame Panda::buildACC_CONTROL(double acc, bool permitBraking, bool releaseStandstill, bool miniCar, bool cancelRequest) {
//	CanFrame frame;
//	/*
//	 BO_ 835 ACC_CONTROL: 8 DSU
//	  SG_ ACCEL_CMD : 7|16@0- (0.001,0) [-20|20] "m/s2" HCU
//	 SG_ SET_ME_X3 : 19|4@0+ (1,0) [0|15] "" XXX
//	  SG_ DISTANCE : 20|1@0+ (1,0) [0|1] "" XXX
//	  SG_ MINI_CAR : 21|1@0+ (1,0) [0|1] "" XXX
//	 SG_ SET_ME_X01 : 23|2@0+ (1,0) [0|3] "" HCU
//	 SG_ CANCEL_REQ : 24|1@0+ (1,0) [0|1] "" HCU
//	  SG_ PERMIT_BRAKING : 30|1@0+ (1,0) [0|1] "" HCU
//	  SG_ RELEASE_STANDSTILL : 31|1@0+ (1,0) [0|1] "" HCU
//	  SG_ ACCEL_CMD_ALT : 47|8@0- (0.05,0) [0|0] "m/s^2" XXX
//	 SG_ CHECKSUM : 63|8@0+ (1,0) [0|255] "" XXX
//	 */
//	frame.bus = 0;
//	frame.busTime = 0;
//	frame.dataLength = 8;
//	frame.messageID = 835;	// LKAS_HUD
//	*((uint64_t*)frame.data) = 0;
//
//	//double acc = (192.0*256.0+254.0)*0.001;
//	//acc = acc > 20 ? 20 : acc;
//	//acc = acc < -20 ? -20 : acc;
//	int16_t ACCEL_CMD = (acc*(1.0/0.001));
//	*((uint64_t*)frame.data) = ((uint64_t)((0xFF00 & ACCEL_CMD) >> 8) << (7+(16/2)+1-16));
//	*((uint64_t*)frame.data) |= ((uint64_t)(0x00FF & ACCEL_CMD)  << (15+(16/2)+1-16));
//	// set_me_x3 LCDT2
//	*((uint64_t*)frame.data) |= ((uint64_t)0x03 << (19+1-4));
//	bool DISTANCE = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)DISTANCE << (20+1-1));
//	bool MINI_CAR = miniCar;	// set to 1 from comma.ai joystick
//	*((uint64_t*)frame.data) |= ((uint64_t)MINI_CAR << (21+1-1));
//	// set_me_x01 ACCTYPE
//	*((uint64_t*)frame.data) |= ((uint64_t)0x01 << (23+1-2));
//	bool CANCEL_REQ = cancelRequest;
//	*((uint64_t*)frame.data) |= ((uint64_t)CANCEL_REQ << (24+1-1));
//	bool LCCWOK = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)LCCWOK << (25+1-1));
//	bool LVSTP = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)LVSTP << (29+1-1));
//	bool PERMIT_BRAKING = permitBraking;	// set to 1 from comma.ai joystick
//	*((uint64_t*)frame.data) |= ((uint64_t)PERMIT_BRAKING << (30+1-1));
//	bool RELEASE_STANDSTILL = releaseStandstill;	// set to 1 from comma.ai joystick
//	*((uint64_t*)frame.data) |= ((uint64_t)RELEASE_STANDSTILL << (31+1-1));
//	unsigned char CACC = 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)CACC << (39+1-8));
//
//	char ACCEL_CMD_ALT = (acc*(1.0/0.05)) * 0;
//	*((uint64_t*)frame.data) |= ((uint64_t)ACCEL_CMD_ALT << (47+1-8));
//
//
//
//	// Checksum at the end:
//	*((uint64_t*)frame.data) |= ((uint64_t)toyotaChecksum(frame) << (63+1-8));
//
//	return frame;
//}
//
//	CanFrame Panda::buildAccCancelCommand(  ) {
//		CanFrame frame;
//
//		/*
//		 BO_ 523 CRUISE_THROTTLE: 6 XXX
//		  SG_ PROPILOT_BUTTON : 8|1@0+ (1,0) [0|1] "" XXX
//		  SG_ CANCEL_BUTTON : 9|1@0+ (1,0) [0|1] "" XXX
//		  SG_ GAS_PEDAL_INVERTED : 37|10@0+ (1,0) [0|1023] "" XXX
//		  SG_ SET_BUTTON : 11|1@0+ (1,0) [0|1] "" XXX
//		  SG_ RES_BUTTON : 12|1@0+ (1,0) [0|1] "" XXX
//		  SG_ FOLLOW_DISTANCE_BUTTON : 10|1@0+ (1,0) [0|1] "" XXX
//		  SG_ NO_BUTTON_PRESSED : 13|1@0+ (1,0) [0|1] "" XXX
//		  SG_ GAS_PEDAL : 31|10@0+ (1,0) [0|255] "" XXX
//		  SG_ USER_BRAKE_PRESSED : 21|1@0+ (1,0) [0|1] "" XXX
//		  SG_ USER_BRAKE_PRESSED_INVERTED : 22|1@0+ (1,0) [0|3] "" XXX
//		  SG_ NEW_SIGNAL_2 : 23|1@0+ (1,0) [0|1] "" XXX
//		  SG_ GAS_PRESSED_INVERTED : 20|1@0+ (1,0) [0|255] "" XXX
//		  SG_ COUNTER : 17|2@0+ (1,0) [0|3] "" XXX
//		  SG_ unsure1 : 7|10@0+ (1,0) [0|1023] "" XXX
//		  SG_ unsure2 : 43|4@0+ (1,0) [0|1] "" XXX
//		  SG_ unsure3 : 19|2@0+ (1,0) [0|3] "" XXX
//		 */
//
//	   frame.bus = 0;
//	   frame.busTime = 0;
//	   frame.dataLength = 6;
//	   frame.messageID = 523;
//	   *((uint64_t*)frame.data) = 0;
//
//		return frame;
//	}
//
//CanFrame Panda::buildAccCancelCommand(  ) {
//	CanFrame frame;
//
//	/*
//	 */
//
//   frame.bus = 0;
//   frame.busTime = 0;
//   frame.dataLength = 8;
//   frame.messageID = ;
//   *((uint64_t*)frame.data) = 0;
//
//	return frame;
//}
//
//void Panda::printFrame( CanFrame frame ) {
//	printf("%d ", frame.messageID);
//	printf("%d ", frame.bus);
//	for(int i = 0; i < frame.dataLength; i++) {
//		printf("%02X ", (int)frame.data[i]);
//	}
//	printf("\n");
//}
