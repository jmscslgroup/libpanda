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

#ifndef OBD_PID_H
#define OBD_PID_H

#include <queue>

#include "can.h"

namespace Panda {


// TODO: Codify some PID codes from here as enums/defines
// https://en.wikipedia.org/wiki/OBD-II_PIDs#Service_09


//enum CanFrameTypes {
//	CAN_FRAME_SINGLE = 0x00,
//	CAN_FRAME_FIRST = 0x01,
//	CAN_FRAME_CONSECUTIVE = 0x02,
//	CAN_FRAME_FLOW_CONTROL = 0x03
//};


//class Handler;
class ObdPidRequest : public CanListener, public Mogi::Thread {
	
public:
	unsigned char* data;
	int dataLength; // this should always be 17
	
	ObdPidRequest(Can& handler);
	~ObdPidRequest();
	
	void request( unsigned char mode, unsigned char pid, bool extendedAddr );
	
	bool complete();
	
private:
	unsigned char* tempData;
	int tempLength;
	bool extended;
	
	long long assignedId;	// for extended we oly need 32, but I set this to -1 for encoding the state of handshaking
	
	bool busy;
	
	unsigned char mode;
	unsigned char pid;
	Can *canHandler;
	
	std::queue<CanFrame> frameQueue;
	
	void sendFlowControl();
	
	// Override from Mogi::Thread
	void doAction();
	
	// Override from CanListener:
	void newDataNotification( CanFrame* canFrame );
	
};


}


#endif
