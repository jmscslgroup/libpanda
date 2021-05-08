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

#include "panda/can.h"
#include "panda/obd-pid.h"
#include "panda/obd-pid-definitions.h"

#include <unistd.h>
#include <time.h>
#include <cstring> // memcpy

//#define CAN_VERBOSE

using namespace Panda;

Can::Can()
:canDump(NULL), csvDump(NULL) {
}

Can::~Can() {
	stop();
	WaitForInternalThreadToExit();
	if (canDump != NULL) {
		fclose(canDump);
	}
	if (csvDump != NULL) {
		fclose(csvDump);
	}
}

void Can::initialize() {
	std::cout << "Initializing CAN" <<std::endl;
	

	std::cout << " - Purging ring buffers" << std::endl;
	usbHandler->canPurge();
	
	std::cout << " - CAN Done." << std::endl;
}

void Can::saveToFile( const char* filename ) {
	canDump = fopen(filename, "w");
}

void Can::saveToCsvFile(const char* filename) {
	FILE* csvDumpTemp = fopen(filename, "w");
	fprintf(csvDumpTemp, "Time,Bus,MessageID,Message,MessageLength\n");
	this->csvDump = csvDumpTemp;
}

void Can::setUsb( Panda::Usb* usbHandler ) {
	this->usbHandler = usbHandler;
}



void Can::addObserver( CanListener* listener ) {
	lock();
	
	listeners.push_back(listener);
	
	unlock();
}


void Can::removeObserver( CanListener* listener ) {
	lock();
	
	std::vector<CanListener*>::iterator it = listeners.begin();
	for (; it != listeners.end(); it++) {
		if (*it == listener) {
			break;
		}
	}
	
	if (it != listeners.end()) {
		listeners.erase(it);
	}
	
	unlock();
}

void Can::startParsing() {
	if(usbHandler == NULL) {
		std::cerr << "ERROR: Can::startParsing(): No Usb Handler set for Panda::Can!" << std::endl;
		return;
	}

	start();
	
	
	
	// Read the VIN here:
	std::cout << " - Attempting to read the VIN:";
	ObdPidRequest vinRequest(*this);
	int vinAttempts = 0;
	while( vinAttempts++ < 10 ) {
		std::cerr << std::endl << " - - Attempt " << vinAttempts << "/10...";
		vinRequest.request(Panda::OBD_PID_SERVICE_VEHICLE_INFO, Panda::OBD_PID_VEHICLE_INFO_VIN);
		sleep(1);
		if (vinRequest.complete()) {
			break;
		}
	}
	if (vinRequest.complete()) {
		// We got it!
		printf("Success! Read: ");
		for (int i = 0; i < vinRequest.dataLength; i++) {
			printf("%c", vinRequest.data[i]);
		}
		printf("\n");
		// Save the VIN:
		FILE* file = fopen( "/etc/libpanda.d/vin", "w+");
		fwrite( vinRequest.data, 1, vinRequest.dataLength, file);
		fclose(file);
		
		// Notify a new vin has been read:
		file = fopen( "/etc/libpanda.d/newVin", "w+");
		fwrite( "1\n", 1, strlen("1\n"), file);
		fclose(file);
	} else {
		std::cerr << std::endl << " - - WARNING: Unable to read the VIN!" << std::endl;
	}
	
	std::cout << " - Setting Safety to Toyota:" << std::endl;
	usbHandler->setSafetyMode(SAFETY_TOYOTA);	// OBD II port
}

void Can::stopParsing() {
	stop();
	WaitForInternalThreadToExit();
}

void Can::sendMessage( CanFrame& frame ) {	// TODO lots of testing
	if (usbHandler == NULL) {
		std::cerr << "ERROR: Can::sendMessage(): No Usb Handler set for Panda::Can!" << std::endl;
		return;
	}

	unsigned char message[16];
	canFrameToBuffer(frame, message);

	// In comma.ai code they bulk send multiple messages with the following:
	// usb_bulk_write(3, (unsigned char*)send, msg_count*0x10, 5);
	// Panda::usb_bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout
	usbHandler->sendCanData(message, 16);
}

void Can::writeCsvToFile(CanFrame* frame, unsigned char* converted, int bufLength) {
	if (csvDump != NULL) {
		fprintf(csvDump, "%d.%06d,", (unsigned int)frame->sysTime.tv_sec, (int)frame->sysTime.tv_usec);
//		for (int i = 0; i < bufLength; i++) {
//			fprintf(csvDump, "%02x", converted[i]);
//		}

		fprintf(csvDump,"%d,%d,", (int)frame->bus, frame->messageID);

		for (int i =0; i < frame->dataLength; i++) {
			fprintf(csvDump, "%02x", frame->data[i]);
		}
		fprintf(csvDump, ",%d\n", frame->dataLength);

	}
}
void Can::writeRawToFile(char* buffer, size_t length) {
	if (canDump != NULL) {
		fwrite(buffer, sizeof(char), length, canDump);
	}
}

void Can::doAction() {
	if (usbHandler == NULL) {
		std::cerr << "ERROR: Can::doAction(): No Usb Handler set for Panda::Can!" << std::endl;
		usleep(1000);
		return;
	}

	if (!currentlyReceiving) {
		usleep(500);
	}


	pause();	// yes, before requesting data, otherwise synchronous USB deadlocks

	//if (!currentlyRequesting) {
	//	currentlyRequesting = true;
	usbHandler->requestCanData(); // only request data when we have nothing to process
	//}
	 
//	if (canFrames.size() == 0) {
//		usleep(1000);	// some breathing room maybe?
//		return;
//	}

	while (canFrames.size() > 0) {

		// Copy first packet:
		lock();
		CanFrame canFrame(canFrames.front()); // FIFO read
		canFrames.pop_front();                 // FIFO remove
		unlock();

		// Notify observers:
		lock();
		for (std::vector<CanListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
			(*it)->newDataNotificationProxy(&canFrame);
		}
		unlock();

		// Save data to log
		//writeCsvToFile(&canFrame);
	}

}

void Panda::canFrameToBuffer( CanFrame& frame, unsigned char* buffer) {	// TODO lots of testing
	uint32_t* message = (uint32_t*)buffer;

	// From comma.ai board.cc code.
	// This may also set bits for designating a write?
	if (frame.messageID > 0x0800) {
		message[0] = (frame.messageID << 3) | 5;
	} else {
		message[0] = (frame.messageID << 21) | 1;
	}

	message[1] = (frame.bus << 4) | frame.dataLength;
	//message[2] = frame.busTime << 16;	// Not needed
	memcpy(&message[2], frame.data, frame.dataLength);
}

CanFrame Panda::bufferToCanFrame(char* buffer, int bufferLength) {
	// The following translation is ported from Rahul's parse_can_buffer
	CanFrame canFrame;
	uint32_t repackedBuffer[4];		// we only expect to ever get 16 byte maximum, this looks at first 2 bytes
	memcpy(repackedBuffer, buffer, bufferLength);

#ifdef CAN_VERBOSE
	std::cout << " - In bufferToCanFrame()" << std::endl;
	std::cout << " |-- Repacked:";
	for (int i = 0; i < 4; i++) {
		printf("0x%08X ", repackedBuffer[i]); // uint8 is 2 nibbles, uint32=4*uint8 -> 8 nibbles
	}
	std::cout << std::endl;
#endif

	// From comma.ai board.cc code:
	if (repackedBuffer[0] & 0x04) {	// Check for extended frame
		canFrame.messageID = repackedBuffer[0] >> 3;
	} else {
		canFrame.messageID = repackedBuffer[0] >> 21;
		// Ok I'm confused, if we retrieve this then the frame formation is completly different according to this:
		// https://fabiobaltieri.com/2013/07/23/hacking-into-a-vehicle-can-bus-toyothack-and-socketcan/
		// the the following methods only work for extended remote frames.
		// My conclusion here is that twe are NOT forming a CAN packet, but rather enought for the Panda
		// or even a potential ELM327 chip to understand, therefore the above link is meaningless
	}

	canFrame.busTime = repackedBuffer[1] >> 16;
	canFrame.dataLength = repackedBuffer[1] & 0x0000000F;
	memcpy(canFrame.data, &buffer[8], canFrame.dataLength);
	canFrame.bus = (repackedBuffer[1] >> 4) & 0x00000FF;

#ifdef CAN_VERBOSE
	printf(" |-- Parsed Frame: %d,%d,", canFrame.messageID, canFrame.bus);
	for (int i = canFrame.dataLength-1; i >= 0; i--) {
		printf("%02x", canFrame.data[i]);
	}
	printf(",%d\n", canFrame.dataLength);
#endif

	return canFrame;
}

void Can::notificationCanRead(char* buffer, size_t bufferLength) {
	currentlyReceiving = bufferLength > 0;
	resume();	// Invoke another CAN request
	
	struct timeval sysTime;
	gettimeofday(&sysTime, NULL);

	writeRawToFile(buffer, bufferLength);
#ifdef CAN_VERBOSE
	std::cout << "Got a CAN::notificationCanRead()!" << std::endl;
	std::cout << " - Received " << bufferLength <<	" Bytes :";
//	for (int i = 0; i < bufferLength; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
	std::cout << std::endl;
#endif

	int lengthOfFrame, remainingBytes;
	lock();
	for (int index = 0; index < bufferLength; index += 16) {
		remainingBytes = bufferLength - index;
		lengthOfFrame = remainingBytes > 16 ? 16 : remainingBytes;
#ifdef CAN_VERBOSE
		std::cout << " - Gonna parse this: ";
		for (int i = 0; i < lengthOfFrame; i++) {
					printf("0x%02X ", buffer[index+i]);
				}
		std::cout << std::endl;
#endif
//		if (((unsigned char)buffer[index]) == 0xFF) {	// TODO test if this is needed, preliminary testing failure showed this but may be an unrelated problem now resolved
//			continue;
//		}
		CanFrame newFrame = bufferToCanFrame(&buffer[index], lengthOfFrame);
		newFrame.sysTime = sysTime;
		writeCsvToFile(&newFrame, (unsigned char*)&buffer[index], lengthOfFrame);
		canFrames.push_back(newFrame);	// FIFO insert
	}
	unlock();
}

void Panda::CanListener::addToBlacklistBus( const int& busToBlock ) {
	blacklistBus.push_back(busToBlock);
}

void Panda::CanListener::addToBlacklistMessageId( const int& idToBlock ) {
	blacklistId.push_back(idToBlock);
}

void Panda::CanListener::newDataNotificationProxy(CanFrame* canFrame) {
	for (std::vector<int>::iterator it=blacklistId.begin();
		 it != blacklistId.end();
		 it++) {
		int& Id = *it;
		if (canFrame->messageID == Id) {
			return;	// kill the notification
		}
	}
	for (std::vector<int>::iterator it=blacklistBus.begin();
		 it != blacklistBus.end();
		 it++) {
		int& bus = *it;
		if (canFrame->bus == bus) {
			return;	// kill the notification
		}
	}
	newDataNotification(canFrame);
}
