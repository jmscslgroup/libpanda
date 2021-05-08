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

#include "panda/vin.h"
#include "panda/obd-pid-definitions.h"

using namespace Panda;

VinReader::VinReader()
:vinHasBeenRead(false), mPandaHandler(NULL), file(NULL) {
	addToBlacklistBus(0);
	addToBlacklistBus(2);
}

// Check this before using system time
bool VinReader::vinRead() {
	return vinHasBeenRead;
}

const char* VinReader::vinResult() {
	return vin;
}

void VinReader::setPandaHandler(Panda::Handler& handler) {
	mPandaHandler = &handler;
	mPandaHandler->addCanObserver(*this);
}

void VinReader::requestVin() {
	Panda::CanFrame vinRequest;
	
	// From comma.ai selfdrive code, should be bus 1:
	vinRequest.bus = 1;
	// Following https://m0agx.eu/2017/12/27/reading-obd2-data-without-elm327-part-1-can/
	vinRequest.messageID = 0x7DF;
//		vinRequest.messageID = 0x18DB33F1;
	vinRequest.data[0] = 0x02;	// designates length, unsure if this is a sub-protocol of the data
	vinRequest.data[1] = 0x09;	// Mode
	vinRequest.data[2] = 0x02;	// VIN PID for the mode
	vinRequest.data[3] = 0x00;
	vinRequest.data[4] = 0x00;
	vinRequest.data[5] = 0x00;
	vinRequest.data[6] = 0x00;
	vinRequest.data[7] = 0x00;
	vinRequest.dataLength = 8;
	
	mPandaHandler->getCan().sendMessage(vinRequest);
}

/*
 Will save the VIN to a file when discovered:
 */
void VinReader::saveToFile(const char* filename) {
	file = fopen(filename, "w+");
}


void VinReader::sendFlowControl() {
	Panda::CanFrame vinRequest;
	// From comma.ai selfdrive code, should be bus 1:
	vinRequest.bus = 1;
	// Following https://happilyembedded.wordpress.com/2016/02/15/can-multiple-frame-transmission/
	vinRequest.messageID = assignedId;
	vinRequest.data[0] = 0x30;	// Flow Control packet and
	vinRequest.data[1] = 0x00;	// Block Size. 0 means that all may be sent without another flow control
	vinRequest.data[2] = 0x00;	// Separation Time Minimum
	vinRequest.data[3] = 0x00;
	vinRequest.data[4] = 0x00;
	vinRequest.data[5] = 0x00;
	vinRequest.data[6] = 0x00;
	vinRequest.data[7] = 0x00;
	vinRequest.dataLength = 8;
	
	mPandaHandler->getCan().sendMessage(vinRequest);
}

void VinReader::newDataNotification( Panda::CanFrame* canFrame ) {
	if (vinHasBeenRead) {	// Would be better to unregister from parent when VIN is discovered
		return;
	}
	if (canFrame->messageID >= 0x7E8 &&
		canFrame->messageID <= 0x7EF) {	// Should we also check the bus?
		/*
		std::cout << "Got a response!" << std::endl;
		
		printf(" - Message ID: %04x\n", canFrame->messageID);
		printf(" - BUS       : %d\n", canFrame->bus);
		printf(" - Data Received: ");
		
		for (int i = 0; i < canFrame->dataLength; i++) {
			printf("0x%02x ", canFrame->data[i]);
		}
		printf("\n");
		*/
		
		
		// TODO: Take the below logic and make a CAN multi-header handler
		int index;
		switch ((canFrame->data[0] & CAN_FRAME_TYPE_MASK) >> 4) {
			case CAN_FRAME_SINGLE:
				// Shouldn't reach here
				//std::cout << "This is a CAN_FRAME_SINGLE" << std::endl;
				break;
				
			case CAN_FRAME_FIRST:
				//std::cout << "This is a CAN_FRAME_FIRST" << std::endl;
				responseLength = (((int)canFrame->data[0] & 0x0F) << 8) | (int)canFrame->data[1];
				//printf(" - Length: %d\n", responseLength);
				
				// If this is a correct response, data 2-4 should be 0x49 0x02 0x01
				if (canFrame->data[2] != 0x49 ||
					canFrame->data[3] != 0x02 ||
					canFrame->data[4] != 0x01) {
					//printf("Warning: this response does not correspond to the VIN request\n");
					return;
				}
				
				assignedId = canFrame->messageID - 8;
				//printf(" - Assigned ID: 0x%04x\n", assignedId);
				
				//printf(" - Multi-packet data length: %d\n", canFrame->data[1]);
				vinLength = canFrame->data[1] - 3;	// Protocol eats 3 bytes at the start
				
				memcpy(vin, &canFrame->data[5], 3);  // first message has first 3 bytes
				sendFlowControl();
				break;
				
			case CAN_FRAME_CONSECUTIVE:
				//std::cout << "This is a CAN_FRAME_CONSECUTIVE" << std::endl;
				index = canFrame->data[0] & 0x0F;
				//printf(" - Frame index: %d\n", index);
				memcpy(&vin[3+7*(index-1)], &canFrame->data[1], 7);
				if ( (3+7*index) >= vinLength) {
					vinHasBeenRead = true;
					if (file) {
						fprintf(file, "%s\n",vin);
						fclose(file);
					}
				}
				break;
				
			case CAN_FRAME_FLOW_CONTROL:
				//std::cout << "This is a CAN_FRAME_FLOW_CONTROL" << std::endl;
				break;
			default:
				
				break;
		}
		
		//printf("\n");
	}
	
}
