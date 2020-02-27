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
#include <unistd.h>
#include <time.h>
#include <cstring> // memcpy

#define CAN_VERBOSE

using namespace Panda;

Can::Can() {
}

Can::~Can() {
	stop();
	WaitForInternalThreadToExit();
	if (canDump.is_open()) {
		canDump.close();
	}
	if (canCsv.is_open()) {
		canCsv.close();
	}
}

void Can::initialize() {
	std::cout << "Initializing CAN" <<std::endl;

	std::cout << " - CAN Done." << std::endl;
}

void Can::saveToFile( const char* filename ) {
	canDump.open(filename);
}

void Can::saveToCsvFile(const char* filename) {
	canCsv.open(filename);
	canCsv << "Time,Bus,MessageID,Message,MessageLength\n";
}

void Can::setUsb( Panda::Usb* usbHandler ) {
	this->usbHandler = usbHandler;
}



void Can::addObserver( CanListener* listener ) {
	listeners.push_back(listener);
}

void Can::startParsing() {
	if(usbHandler == NULL) {
		std::cerr << "ERROR: Can::startParsing(): No Usb Handler set for Panda::Can!" << std::endl;
		return;
	}

	start();
}

void Can::stopParsing() {
	stop();
	WaitForInternalThreadToExit();
}

void Panda::canFrameToBuffer( CanFrame& frame, unsigned char* buffer) {	// TODO lots of testing
	uint32_t* message = (uint32_t*)buffer;

	if (frame.messageID > 0x0800) {
		message[0] = (frame.messageID << 3) | 5;
	} else {
		message[0] = (frame.messageID << 21) | 1;
	}

	message[1] = (frame.bus << 4) | frame.dataLength;
	//message[2] = frame.busTime << 16;	// Not needed
	memcpy(&message[2], frame.data, frame.dataLength);
}

void Can::sendMessage( CanFrame& frame ) {	// TODO lots of testing
	if (usbHandler == NULL) {
		std::cerr << "ERROR: Can::sendMessage(): No Usb Handler set for Panda::Can!" << std::endl;
		return;
	}

	unsigned char message[16];
	canFrameToBuffer(frame, message);

	usbHandler->sendCanData(message, 16);
}



void Can::doAction() {
	if (usbHandler == NULL) {
		std::cerr << "ERROR: Can::doAction(): No Usb Handler set for Panda::Can!" << std::endl;
		usleep(1000);
		return;
	}
	usbHandler->requestCanData();
	
	if (canFrames.size() == 0) {
		usleep(100);	// some breathing room maybe?
		return;
	}

	// Copy first packet:
	lock();
	CanFrame canFrame(canFrames.front()); // FIFO read
	canFrames.pop_front();                 // FIFO remove
	unlock();

	if (canCsv.is_open()) {
		time_t epochTime = time(NULL);
		canCsv	<< epochTime << ","
				// raw data
				<< canFrame.bus << ","
				<< canFrame.messageID << ","
				<< canFrame.data << ","
				<< canFrame.dataLength << std::endl;

	}

	// Now we have a packet to parse.

	// Once parsed, notify observers:
	for (std::vector<CanListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		(*it)->newDataNotification(&canFrame);
	}

}

CanFrame Panda::bufferToCanFrame(char* buffer, int bufferLength) {
	// The following translation is ported from Rahul's parse_can_buffer
	CanFrame canFrame;			// TODO packet parsing logic here
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

	if (repackedBuffer[0] & 0x04) {	// Check for extended frame
		canFrame.messageID = repackedBuffer[0] >> 3;
	} else {
		canFrame.messageID = repackedBuffer[0] >> 21;
		// Ok I'm confused, if we retrive this then the frame formation is completly different according to this:
		// https://fabiobaltieri.com/2013/07/23/hacking-into-a-vehicle-can-bus-toyothack-and-socketcan/
		// the the following methods only work for extended remote frames.
	}

	canFrame.busTime = repackedBuffer[1] >> 16;
	canFrame.dataLength = repackedBuffer[1] & 0x0000000F;
	memcpy(canFrame.data, &buffer[8], canFrame.dataLength);
	canFrame.bus = (repackedBuffer[1] >> 4) & 0x00000FF;

#ifdef CAN_VERBOSE
//	std::cout << " |-- Parsed: Message ID:" << canFrame.messageID << " Bus:" << canFrame.bus << " Bus Time:" << canFrame.busTime << " Data:";
//	for (int i = 0; i < canFrame.dataLength; i++) {
//		printf("0x%02X ", canFrame.data[i]);
//	}
//	std::cout << std::endl;
	printf(" |-- Parsed Frame: %d,%d,", canFrame.messageID, canFrame.bus);
	for (int i = canFrame.dataLength-1; i >= 0; i--) {
		printf("%02x", canFrame.data[i]);
	}
	printf(",%d\n", canFrame.dataLength);
#endif

	return canFrame;
}

void Can::notificationCanRead(char* buffer, size_t bufferLength) {
	if (canDump.is_open()) {
		canDump.write(buffer, bufferLength);
	}
#ifdef CAN_VERBOSE
	std::cout << "Got a CAN::notificationCanRead()!" << std::endl;
	std::cout << " - Received " << bufferLength <<	" Bytes :";
	for (int i = 0; i < bufferLength; i++) {
		printf("0x%02X ", buffer[i]);
	}
	std::cout << std::endl;
#endif

	int lengthOfFrame, remainingBytes;
	lock();
	for (int index = 0; index < bufferLength; index += 16) {
		remainingBytes = bufferLength - index;
		lengthOfFrame = remainingBytes > 16 ? 16 : remainingBytes;
		canFrames.push_back(bufferToCanFrame(&buffer[index], lengthOfFrame));	// FIFO insert
	}
	unlock();
}
