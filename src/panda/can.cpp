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


void printBuffer(char* buffer, int length) {
	std::cout << " - Buffer of length " << length <<	":";
	for (int i = 0; i < length; i++) {
		if (i%16 == 0) {
			printf("\n");
		}
		printf("0x%02X ", buffer[i]);
	}
	printf("\n");
}

Can::Can()
:canDump(NULL), csvDump(NULL), pandaCanVersion(2) {
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
	
	if (usbHandler == NULL) {
		std::cerr << "Error: Can::initialize(): usbHandler == NULL" << std::endl;
		return;
	}
	
	std::cout << " - Checking CAN Packet Version..." <<std::endl;
	unsigned char healthVersion;
	unsigned char canVersion;
	usbHandler->getHealthAndCanVersions(&healthVersion, &canVersion);
	std::cout << " --- Panda CAN Version: " << (int)canVersion << std::endl;
	this->pandaCanVersion = canVersion;
	
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
		//		std::cerr << "Removing CAN observer" << std::endl;
		listeners.erase(it);
	}
	
	unlock();
}

void Can::startParsing() {
	if(usbHandler == NULL) {
		std::cerr << "ERROR: Can::startParsing(): No Usb Handler set for Panda::Can!" << std::endl;
		std::cerr << "       Can::startParsing(): Parsing will run only for testing" << std::endl;
		//		return;
	}
	
	start();
	
	
	
	if(usbHandler != NULL) {
		
		// Read the VIN here:
		usleep(200000);
		std::cout << " - Attempting to read the VIN:" << std::endl;
		
		int vinAttempts = 0;
		bool extended = true;	// HACK
		while( vinAttempts++ < 10 ) {
			usbHandler->sendHeartBeat();
			
			ObdPidRequest vinRequest(*this);
			
			if (vinAttempts > 5 && extended == true) {	// HACK
				std::cout << "Extended Addressing OBD PID failed, attempting legacy addressed OBD PID reading" << std::endl;	// HACK
				extended = false;	// HACK
			}	// HACK
			
			std::cerr << " - - Attempt " << vinAttempts << "/10...";
			
			vinRequest.request(Panda::OBD_PID_SERVICE_VEHICLE_INFO, Panda::OBD_PID_VEHICLE_INFO_VIN, extended);
			int timeoutCount = 0;
			while (timeoutCount++ < 100 && !vinRequest.complete()) {
				usleep(10000);
			}
			if (vinRequest.complete()) {
				//			break;
				//		}
				//	}
				//	if (vinRequest.complete()) {
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
				break;
			} else {
				std::cerr << "Timeout" << std::endl;
			}
			
		}
		
		std::cout << " - Setting Safety to SAFETY_NOOUTPUT:" << std::endl;
		usbHandler->setSafetyMode(SAFETY_NOOUTPUT, 0);	// OBD II port
	}
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
	
	int messageSize = 16;
	if (pandaCanVersion == 2) {
		messageSize = sizeof(CANPacket_t);
	}
	unsigned char message[messageSize]; // RED: packets are now 72 in length I think
	int length = canFrameToBuffer(frame, message, pandaCanVersion); // RED: we need to keep track of message lengths
	
#ifdef CAN_VERBOSE
	printf("Can::sendMessage(): Converted frame: %d,%u,", (int)frame.bus, frame.messageID);
	
	for (int i =0; i < frame.dataLength; i++) {
		printf("%02x", frame.data[i]);
	}
	printf(",%d\n to:\n", frame.dataLength);
	
	printBuffer((char*)message, length);
		
#endif
	
	// In comma.ai code they bulk send multiple messages with the following:
	// usb_bulk_write(3, (unsigned char*)send, msg_count*0x10, 5);
	// Panda::usb_bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout
	
	if(usbHandler != NULL) {
		usbHandler->sendCanData(message, length);	// RED: New packet
	}
}

void Can::writeCsvToFile(CanFrame* frame, unsigned char* converted, int bufLength) {
	if (csvDump != NULL) {
		fprintf(csvDump, "%d.%06d,", (unsigned int)frame->sysTime.tv_sec, (int)frame->sysTime.tv_usec);
		//		for (int i = 0; i < bufLength; i++) {
		//			fprintf(csvDump, "%02x", converted[i]);
		//		}
		
		fprintf(csvDump,"%d,%u,", (int)frame->bus, frame->messageID);
		
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
//		std::cerr << "ERROR: Can::doAction(): No Usb Handler set for Panda::Can!" << std::endl;
		usleep(1000);
		//		return;
	}
	
	if (!currentlyReceiving) {
		usleep(500);
	}
	
	
	pause();	// yes, before requesting data, otherwise synchronous USB deadlocks
	
	//if (!currentlyRequesting) {
	//	currentlyRequesting = true;
	
	if (usbHandler != NULL) {
		usbHandler->requestCanData(); // only request data when we have nothing to process
	}
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

int Panda::canFrameToBuffer( CanFrame& frame, unsigned char* buffer, int pandaCanVersion) {	// TODO lots of testing
	int bufferLength = 0;
	
	if (pandaCanVersion == 2) {
		
		
		// best to assert that length is within dlcToLen[]
		unsigned char tempBuffer[256];
		int extended = frame.messageID >= 0x800;
		int dataLengthCode;
		if (frame.dataLength < 9) {
			dataLengthCode = frame.dataLength;
		} else {
			switch (frame.dataLength) {
				case 12:
					dataLengthCode = 9;
					break;
				case 16:
					dataLengthCode = 10;
					break;
				case 20:
					dataLengthCode = 11;
					break;
				case 24:
					dataLengthCode = 12;
					break;
				case 32:
					dataLengthCode = 13;
					break;
				case 48:
					dataLengthCode = 14;
					break;
				case 64:
					dataLengthCode = 15;
					break;
					
				default:
					std::cerr << "Error! Panda::canFrameToBuffer() invalid data length of " << (int)frame.dataLength << std::endl;
					return 0;
					break;
			}
		}
		unsigned int word_4b = (frame.messageID << 3) | (extended << 2);
		tempBuffer[0] = (dataLengthCode << 4) | (frame.bus << 1);
		tempBuffer[1] = word_4b & 0xFF;
		tempBuffer[2] = (word_4b >> 8) & 0xFF;
		tempBuffer[3] = (word_4b >> 16) & 0xFF;
		tempBuffer[4] = (word_4b >> 24) & 0xFF;
		memcpy(&tempBuffer[5], frame.data, frame.dataLength);
		
		int counter = 0;
		for(int i=0, j=0; j < frame.dataLength + 5; i += 64, j+=63) {
			buffer[i] = counter++;
			memcpy(&buffer[i+1], &tempBuffer[j], (63 > frame.dataLength + 5 - j) ? 63 : (frame.dataLength + 5 - j));
		}
		
		bufferLength = counter + frame.dataLength + 5;
	} else {
		uint32_t* message = (uint32_t*)buffer;
		
		if (frame.dataLength > 8) {
			std::cerr << "Error! Panda::canFrameToBuffer() invalid data length of " << frame.dataLength << " > 8 "<< std::endl;
			return 0;
		}
		
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
		bufferLength = 16;
	}
	
	return bufferLength;
}

CanFrame Panda::bufferToCanFrame(char* buffer, int bufferLength, int pandaCanVersion) {
	// The following translation is ported from Rahul's parse_can_buffer
	CanFrame canFrame;
	
	if (pandaCanVersion == 2) {
		
		
		// All of the above commented out for CAN packet version 2 parsing:
		
#define CANPACKET_HEAD_SIZE (0x5)
		static unsigned char dlcToLen[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64}; // dat alength code
//		int parsePosition = 0;
		unsigned char lookupIndex = buffer[0] >> 4;
		if (lookupIndex > sizeof(dlcToLen)/sizeof(dlcToLen[0])) {	// This will never fail
			std::cerr << "Error: Panda::bufferToCanFrame(): Received an Data Length Lookup: " << (int)lookupIndex << " > " << (int)(sizeof(dlcToLen)/sizeof(dlcToLen[0])) << std::endl;
			printBuffer(buffer, bufferLength);
			canFrame.messageID = -1;
			return canFrame;
		}
//		if (buffer[0] & 0x01 ) {
//			std::cerr << "Error: Panda::bufferToCanFrame(): Reserved bit expected to be 0: " << std::endl;
//			printBuffer(buffer, bufferLength);
//			canFrame.messageID = -1;
//			return canFrame;
//		}
		
		canFrame.dataLength = dlcToLen[lookupIndex];
		int packetLength = CANPACKET_HEAD_SIZE + canFrame.dataLength;
		//if (packetLength < bufferLength) {	// we will have flexible sized packets sooo... this function will need work
		char* header = &buffer[0];	// will be of length CANPACKET_HEAD_SIZE
		
		canFrame.bus = (header[0] >> 1) & 0x07;
		if (canFrame.bus > 3) {	// Pandas shouldn't have this...
			std::cerr << "Error: Panda::bufferToCanFrame(): Received an invalid bus of " << (int)canFrame.bus << std::endl;
			printBuffer(buffer, bufferLength);
			canFrame.messageID = -1;
			return canFrame;
		}
//		canFrame.messageID = ((header[4] << 24) | (header[3] << 16) | (header[2] << 8) | header[1]) >> 3;
		canFrame.messageID = (header[4] << 21) | (header[3] << 13) | (header[2] << 5) | (header[1] >> 3);
		
		canFrame.returned = (header[1] >> 1) & 0x01;
		canFrame.rejected = header[1] & 0x01;
		memcpy(canFrame.data, &buffer[0 + CANPACKET_HEAD_SIZE], canFrame.dataLength);
		
		if (canFrame.returned) {
			canFrame.bus += 128;
		}
		if (canFrame.rejected) {
			canFrame.bus += 192;
		}
		//}
#ifdef CAN_VERBOSE
		
		printf("Parsed message: %d,%u,", (int)canFrame.bus, canFrame.messageID);
		
		for (int i =0; i < canFrame.dataLength; i++) {
			printf("%02x", canFrame.data[i]);
		}
		printf(",%d\n", canFrame.dataLength);
		
#endif
	} else {	// Other panda CAN version
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
	}
	return canFrame;
}

void Can::notificationCanRead(char* buffer, size_t bufferLength) {
	currentlyReceiving = bufferLength > 0;
	resume();	// Invoke another CAN request
	
	struct timeval sysTime;
	gettimeofday(&sysTime, NULL);
	
	writeRawToFile(buffer, bufferLength);
#ifdef CAN_VERBOSE
	if (bufferLength > 0) {
		std::cout << "Got a CAN::notificationCanRead()!" << std::endl;
		std::cout << " - Received " << bufferLength <<	" Bytes :";
		for (int i = 0; i < bufferLength; i++) {
			if (i%16 == 0) {
				printf("\n");
			}
			printf("0x%02X ", buffer[i]);
		}
		std::cout << std::endl;
	}
#endif
	
	int lengthOfFrame, remainingBytes;
	lock();
	
	if (pandaCanVersion == 2) {
		
		
		// The following is for a newer CANPACKET_VERSION == 2
		//#define CANPACKET_HEAD_SIZE (0x5)
		static unsigned char dlcToLen[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64}; // data length code
		
		
		int counter = 0;
		char chunk[256];
		int chunkLength = 0;
		char* tail = chunk;
		int tailLength = 0;
		
		// First let's check the integrity of the buffer, then fail if it's broken
		for (int index = 0; index < bufferLength; index += 64) {
			if (counter++ != buffer[index]) {
				std::cerr << "Error: Can::notificationCanRead(): Malformed Packet Counter.  What was recieved:" << std::endl;
				printBuffer(buffer, bufferLength);
				unlock();
				return;
			}
		}
		
		for (int index = 0; index < bufferLength; index += 64) {
//			if (counter++ != buffer[index]) {
//				std::cerr << "Error: Can::notificationCanRead(): Lost Recieve Packet Counter.  What was recieved:" << std::endl;
//				printBuffer(buffer, bufferLength);
//				break;
//			}
			
			memcpy(chunk, tail, tailLength);
			memcpy(&chunk[tailLength], &buffer[index+1], 63 > bufferLength-(index+1) ? bufferLength - (index+1) : 63);
			chunkLength = tailLength + (63 > bufferLength-(index+1) ? bufferLength - (index+1) : 63);
			int parsePosition = 0;
#ifdef CAN_VERBOSE
			std::cout << "At index " << index << " chunk length: " << chunkLength << std::endl;
			std::cout << " - Current chunk: " << chunkLength <<	" Bytes :";
			for (int i = 0; i < chunkLength; i++) {
				if (i%16 == 0) {
					printf("\n");
				}
				printf("0x%02X ", chunk[i]);
			}
			std::cout << std::endl;
#endif
			
			while (parsePosition < chunkLength) {
				int dataLength = dlcToLen[chunk[parsePosition] >> 4];
				int packetLength = CANPACKET_HEAD_SIZE + dataLength;
				
#ifdef CAN_VERBOSE
				std::cout << "- Data length, packet length =  " << dataLength << ", " << packetLength << std::endl;
#endif
				
				if (packetLength <= chunkLength - parsePosition) {
#ifdef CAN_VERBOSE
					std::cout << " --- packet length within chunk!" << std::endl;
#endif
					
					CanFrame newFrame = bufferToCanFrame(&chunk[parsePosition], chunkLength-parsePosition, pandaCanVersion);
					if (newFrame.messageID == -1) {
						std::cerr << "Error: Can::notificationCanRead(): Parsing packet resulted in error, this is what was recieved:" << std::endl;
						printBuffer(buffer, bufferLength);
						unlock();
						return;
					}
					newFrame.sysTime = sysTime;
					writeCsvToFile(&newFrame, (unsigned char*)&buffer[index], lengthOfFrame);
					canFrames.push_back(newFrame);	// FIFO insert
					
					parsePosition += packetLength;
					tailLength = 0;
				} else {
					tail = &chunk[parsePosition];
					tailLength = chunkLength - parsePosition;
					
#ifdef CAN_VERBOSE
					std::cout << " --- packet length too long, adding tail of length " << tailLength << std::endl;
#endif
					break;
				}
			}
			
		}
	} else {
		
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
			CanFrame newFrame = bufferToCanFrame(&buffer[index], lengthOfFrame, pandaCanVersion);
			newFrame.sysTime = sysTime;
			writeCsvToFile(&newFrame, (unsigned char*)&buffer[index], lengthOfFrame);
			canFrames.push_back(newFrame);	// FIFO insert
		}
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
