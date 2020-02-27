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

#define CAN_MAX_LENGTH (8)

namespace Panda {

	/*!
	 \struct CanFrame
	 \brief A CAN bus packet data.
	 */
	
	typedef struct _CanFrame {
		/*! \brief CAN message ID
		 */
		unsigned int messageID;
		/*! \brief Length of the data, should not exceed CAN_MAX_LENGTH (8)
		 */
		unsigned char dataLength;
		/*! \brief CAN message bus time
		 */
		unsigned int busTime;
		/*! \brief CAN message bus ID
		 */
		unsigned char bus;
		/*! \brief The data for the CAN message
		 */
		unsigned char data[CAN_MAX_LENGTH];
	} CanFrame;

}

#endif
