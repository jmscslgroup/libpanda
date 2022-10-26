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

#ifndef PANDA_CAN_DATA_H
#define PANDA_CAN_DATA_H

//#define CAN_DATA_MAX_LENGTH (8)
#define CAN_DATA_MAX_LENGTH (64) // For CAN FD

typedef struct {
  unsigned char reserved : 1;
  unsigned char bus : 3;
  unsigned char data_len_code : 4; // 1
  unsigned char rejected : 1;
  unsigned char returned : 1;
  unsigned char extended : 1;
  unsigned int addr : 29;	// 2, 3, 4, 5
  unsigned char data[CAN_DATA_MAX_LENGTH];	// 69
} __attribute__((packed, aligned(4))) CANPacket_t;

namespace Panda {

	/*!
	 \struct CanFrame
	 \brief A CAN bus packet data.
	 */
	
	typedef struct _CanFrame {
		/*! \brief CAN message ID
		 */
		unsigned int messageID = 0;
		/*! \brief Length of the data, should not exceed CAN_DATA_MAX_LENGTH
		 */
		unsigned char dataLength = 0;
		/*! \brief CAN message bus time
		 */
		unsigned int busTime = 0;
		/*! \brief CAN message bus ID
		 */
		unsigned char bus = 0;
		/*! \brief The data for the CAN message
		 */
		unsigned char data[CAN_DATA_MAX_LENGTH];
		/*! \brief This means a message with these same attributes was successfully
		 */
		unsigned char returned;
		/*! \brief Rejected occurs when the safety_tx_hook() does not allow the message
		 In the case of OBD PID communication, this will be set based on safety_elm327.h if:
		 (len != 8) || (addr != 0x18DB33F1) && ((addr & 0x1FFF00FF) != 0x18DA00F1) &&
			 ((addr & 0x1FFFFF00) != 0x700)
		 */
		unsigned char rejected;
		/*! \brief The system time upon rec
		 */
		struct timeval sysTime;
	} CanFrame;

}

#endif
