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

#include "panda/obd-pid.h"
#include "panda/obd-pid-definitions.h"

#include <cstring>

using namespace Panda;

ObdPidRequest::ObdPidRequest(Can& handler)
:data(NULL), dataLength(0), assignedId(-1), busy(false) {
	// Only care about bus 1
	addToBlacklistBus(0);
	addToBlacklistBus(2);
	
	canHandler = &handler;
//	mPandaHandler->addCanObserver(*this);
	canHandler->addObserver(this);
}

ObdPidRequest::~ObdPidRequest() {
	canHandler->removeObserver(this);
	
	if (data) {
		free(data);
		data = NULL;
	}
}

void ObdPidRequest::request( unsigned char mode, unsigned char pid ) {
	busy = true;
	
	this->mode = mode;
	this->pid = pid;
	
	Panda::CanFrame vinRequest;
	// From comma.ai selfdrive code, should be bus 1:
	vinRequest.bus = 1;
	// Following https://m0agx.eu/2017/12/27/reading-obd2-data-without-elm327-part-1-can/
	vinRequest.messageID = 0x7DF;	// broadcast message
//		vinRequest.messageID = 0x18DB33F1;
	vinRequest.data[0] = 0x02;	// designates length, unsure if this is a sub-protocol of the data
	vinRequest.data[1] = mode;	// Mode
	vinRequest.data[2] = pid;	// PID for the mode
	vinRequest.data[3] = 0x00;
	vinRequest.data[4] = 0x00;
	vinRequest.data[5] = 0x00;
	vinRequest.data[6] = 0x00;
	vinRequest.data[7] = 0x00;
	vinRequest.dataLength = 8;
	
	start();
	canHandler->sendMessage(vinRequest);
}

bool ObdPidRequest::complete() {
	return !busy;
}

void ObdPidRequest::sendFlowControl() {
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
	
	canHandler->sendMessage(vinRequest);
}

void printFrameInformation(CanFrame& frame) {
	fprintf(stderr, " - Message ID: %04x\n", frame.messageID);
	fprintf(stderr, " - BUS       : %d\n", frame.bus);
	fprintf(stderr, " - Data Received: ");
	
	for (int i = 0; i < frame.dataLength; i++) {
		fprintf(stderr, "0x%02x ", frame.data[i]);
	}
	fprintf(stderr, "\n");
	
}

void ObdPidRequest::doAction() {
	CanFrame frame;
	while (!frameQueue.empty()) {
		// pop from queue
		lock();
		frame = frameQueue.front();
		frameQueue.pop();
		unlock();
		
		// debug
		/*
		std::cout << "Got a response!" << std::endl;
		printf(" - Message ID: %04x\n", frame.messageID);
		printf(" - BUS       : %d\n", frame.bus);
		printf(" - Data Received: ");
		
		for (int i = 0; i < frame.dataLength; i++) {
			printf("0x%02x ", frame.data[i]);
		}
		printf("\n");
		*/
		
		if (frame.dataLength == 0) {
			fprintf(stderr, "The frame length of this PID request is 0\n");
			printFrameInformation(frame);
			continue;
		}
		
		// then process
		// TODO: Take the below logic and make a CAN multi-header handler
		int index;
		switch ((frame.data[0] & CAN_FRAME_TYPE_MASK) >> 4) {
			case CAN_FRAME_SINGLE:
				// Shouldn't reach here
				//std::cout << "This is a CAN_FRAME_SINGLE" << std::endl;
				if (frame.dataLength < 2) {
					fprintf(stderr, "Error: CAN_FRAME_SINGLE data length is too small: frame.dataLength=%d < 2\n", frame.dataLength);
					printFrameInformation(frame);
					continue;
				}
				if (frame.data[1] != (0x40 | mode) || //0x49 ||
					frame.data[2] != pid ) {
					//printf("Warning: this response does not correspond to the PID request\n");
//					std::cerr << "The frame length of this PID request is 0" << std::endl;
					fprintf(stderr, "Warning: this response does not correspond to the PID request\n");
					printFrameInformation(frame);
					
					continue;
				}
				
				dataLength = ((int)frame.data[0] & 0x0F) - 2;
				//printf(" - Length: %d\n", dataLength);
					
				if (data != NULL) {
					free(data);
					data = NULL;
				}
				
				if ( (dataLength + 3) > frame.dataLength ) {
					fprintf(stderr, "Error: Data Length of OBD PID packet in CAN_FRAME_SINGLE protocol is larger than CanFrame\n");
					printFrameInformation(frame);
					continue;
				}

				data = (unsigned char*)malloc(sizeof(unsigned char) * dataLength);
				memcpy(data, &frame.data[3], dataLength);  // first message has first 3 bytes
				
				busy = false;
				
				stop();
				return;
				break;
				
			case CAN_FRAME_FIRST:
				//std::cout << "This is a CAN_FRAME_FIRST" << std::endl;
				
				if (frame.dataLength < 8) {
					fprintf(stderr, "Error: CAN_FRAME_FIRST data length is too small: frame.dataLength=%d < 8\n", frame.dataLength);
					printFrameInformation(frame);
					continue;
				}
				
				// If this is a correct response, data 2-4 should be 0x49 0x02 0x01
				if (frame.data[2] != (0x40 | mode) || //0x49 ||
					frame.data[3] != pid ||//0x02 ||
					frame.data[4] != 0x01) {
					fprintf(stderr, "Warning: this response does not correspond to the VIN request\n");
					printFrameInformation(frame);
					continue;
				}
				
				
				dataLength = ((((int)frame.data[0] & 0x0F) << 8) | (int)frame.data[1]) - 3;
				//printf(" - Length: %d\n", dataLength);
				
				assignedId = frame.messageID - 8;
				//printf(" - Assigned ID: 0x%04x\n", assignedId);
				
				//printf(" - Multi-packet data length: %d\n", frame.data[1]);
				//dataLength = frame.data[1] - 3;	// Protocol eats 3 bytes at the start
				//dataLength = dataLength - 3;
				if (data != NULL) {
					free(data);
					data = NULL;
				}
				
//				if ( (dataLength + 5) > frame.dataLength ) {
//					fprintf(stderr, "Error: Data Length of OBD PID packet in CAN_FRAME_FIRST protocol is larger than CanFrame\n");
//					printFrameInformation(frame);
//					continue;
//				}
				
				data = (unsigned char*)malloc(sizeof(unsigned char) * 3);//dataLength);
				memcpy(data, &frame.data[5], 3);  // first message has first 3 bytes
				
				sendFlowControl();
				break;
				
			case CAN_FRAME_CONSECUTIVE:
				//std::cout << "This is a CAN_FRAME_CONSECUTIVE" << std::endl;
				index = frame.data[0] & 0x0F;
				//printf(" - Frame index: %d\n", index);
				
				if ( (7 + 1) > frame.dataLength ) {
					fprintf(stderr, "Error: CAN_FRAME_CONSECUTIVE data length is too small: frame.dataLength=%d < 8\n", frame.dataLength);
					printFrameInformation(frame);
					continue;
				}
				
				memcpy(&data[3 + 7*(index-1)], &frame.data[1], 7);
				if ( (3 + 7*index) >= dataLength) {
//					vinHasBeenRead = true;
//					if (file) {
//						fprintf(file, "%s\n",vin);
//						fclose(file);
//					}
					busy = false;
					stop();
					return;
				}
				break;
				
			case CAN_FRAME_FLOW_CONTROL:
				//std::cout << "This is a CAN_FRAME_FLOW_CONTROL" << std::endl;
				break;
			default:
				
				break;
		}
	}
	
	pause();
}

void ObdPidRequest::newDataNotification( CanFrame* canFrame ) {
	if ( (assignedId == (canFrame->messageID - 8)) ||
		(assignedId == -1 &&
		canFrame->messageID >= 0x7E8 &&
		canFrame->messageID <= 0x7EF)) {
		// This is a message of interest
	
		lock();	// queue a frame
		frameQueue.push(*canFrame);
		unlock();
		
		resume();
	}
}
