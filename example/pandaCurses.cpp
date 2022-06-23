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

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <iomanip>
#include <fstream>
#include <string.h>

#include <getopt.h>

#include "usbStats.h"
#include "canFrameStats.h"
#include "cursesHandler.h"
#include "panda.h"

/*
 A simple concrete instance of a combination CAN/GPS listener UI updates
 */
class SimpleEverythingObserver : public Panda::CanListener, public Panda::GpsListener {
public:
	bool checkNewData() {
		if (freshData) {
			freshData = false;
			return true;
		}
		return false;
	}
private:
	bool freshData = false;
	void newDataNotification(Panda::CanFrame* canFrame) {	// any CAN parse updates
		freshData = true;
	}
	void newDataNotification(Panda::GpsData* gpsData) {	// any GPS updates
		freshData = true;
	}
};

/*
 Argument setup
 */
void printUsage(const char* binary) {
	std::cout << "Usage: " << binary << " -[v] [-u <usbmode>] [-g <gpsfile>] [-c <csvfile>]" << std::endl;
	//std::cout << "   -v            : Verbose mode" << std::endl;
	std::cout << "   -f            : Fake CAN data for debugging" << std::endl;
	std::cout << "   -u <usbmode>  : USB operating mode:" << std::endl;
	std::cout << "                     a: Asynchronous" << std::endl;
	std::cout << "                     s: Synchronous" << std::endl;
	std::cout << "                     i: Isochronous" << std::endl;
	std::cout << "   -g <gpsfile>  : Filename to output GPS CSV file" << std::endl;
	std::cout << "   -c <csvfile>  : Filename to output CAN in CSV" << std::endl;
	std::cout << "   -b <id1>[,<id2>,<id3>...]  : Blacklist CAN Message Ids, in decimal" << std::endl;
	std::cout << "   -w <id1>[,<id2>,<id3>...]  : Whitelist CAN Message Ids, in decimal. All other messages blocked" << std::endl;
}

int verboseFlag = false;


static struct option long_options[] =
{
	{"verbose", no_argument, &verboseFlag, 0},
	{"fakedata", no_argument, NULL, 0},
	{"usbmode", required_argument, NULL, 'u'},
	{"gpsfile", required_argument, NULL, 'g'},
	{"csvfile", required_argument, NULL, 'c'},
	{"blacklist", required_argument, NULL, 'b'},
	{"whitelist", required_argument, NULL, 'w'},
	{NULL, 0, NULL, 0}
};


/*
 Catch ctrl-c for cleaner exits
 */
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	keepRunning = false;
}
/*
 Blacklist Parsing
 */

std::vector<int> parseBlacklist(char* blacklistString) {
	std::vector<int> result;
	const char* delimiter = ",";
	char* token = strtok(blacklistString, delimiter);
	
	while (token != NULL) {
	
		
		result.push_back(atoi(token));
		
		token = strtok(NULL, delimiter);
	}
	return result;
}

bool validWhiteBlack(char* list) {
//	char* blacklistChar;
//	blacklistChar = list;
	if (*list == 0x00) {
		return false;
	}
	for (; *list != 0x00; list++) {
		if ((*list < '0' || *list > '9') && *list != ',') {
			return false;
		}
	}
	return true;
}

/*
 Main
 */
int main(int argc, char **argv) {
	// Argument parsing
	bool fakeData = false;
	Panda::UsbMode usbMode = Panda::MODE_ASYNCHRONOUS;
	const char* gpsFilename = NULL;
	const char* canFilename = NULL;
	int ch;
	
	char* blacklistString = NULL;
	char* whitelistString = NULL;
	// loop over all of the options
	while ((ch = getopt_long(argc, argv, "u:g:c:b:w:f", long_options, NULL)) != -1)
	{
		// check to see if a single character or long option came through
		switch (ch)
		{
			case 'u':
				switch (optarg[0]) {
					case 'a': usbMode = Panda::MODE_ASYNCHRONOUS; break;
					case 's': usbMode =  Panda::MODE_SYNCHRONOUS; break;
					case 'i': usbMode =  Panda::MODE_ISOCHRONOUS; break; };
				break;
			case 'g':
				gpsFilename = optarg;
				break;
			case 'c':
				canFilename = optarg;
				break;
			case 'b':
				blacklistString = optarg;
				// Check for a valid format:
				if (!validWhiteBlack(blacklistString)) {
					printUsage(argv[0]);
					exit(EXIT_FAILURE);
				}
				break;
			case 'w':
				whitelistString = optarg;
				// Check for a valid format:
				if (!validWhiteBlack(whitelistString)) {
					printUsage(argv[0]);
					exit(EXIT_FAILURE);
				}
				break;
			case 'f':
				fakeData = true;
				break;
			default:
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
				break;
		}
	}
	
	// Set up graceful exit
	signal(SIGINT, killPanda);
	
	// Panda handler construction and initialization:
	std::cout << "Starting " << argv[0] << std::endl;
	
	SimpleEverythingObserver myObserver;
	
	// Buils the main CAN listener for stats tracking:
	CanFrameStats mCanFrameStats;
	// Check blacklist:
	if (blacklistString != NULL) {
		std::vector<int> blacklistIds = parseBlacklist(blacklistString);
		std::cout << "Blacklist size: " << blacklistIds.size() << std::endl;
		for (std::vector<int>::iterator it = blacklistIds.begin(); it != blacklistIds.end(); it++) {
			std::cout << " - " << *it << std::endl;
			mCanFrameStats.addToBlacklistMessageId(*it);
		}
	}
	// Check whitelist:
	if (whitelistString != NULL) {
		std::vector<int> whitelistIds = parseBlacklist(whitelistString);
		std::cout << "Whitelist size: " << whitelistIds.size() << std::endl;
		for (std::vector<int>::iterator it = whitelistIds.begin(); it != whitelistIds.end(); it++) {
			std::cout << " - " << *it << std::endl;
			mCanFrameStats.addToWhitelistMessageId(*it);
		}
	}
	
	
	UsbStats mUsbStats;
	mCanFrameStats.start();
	
	// Initialize Usb, this requires a conencted Panda
	Panda::Handler pandaHandler;
	pandaHandler.addCanObserver(myObserver);
	pandaHandler.addGpsObserver(myObserver);
	
	pandaHandler.addCanObserver(mCanFrameStats);
	pandaHandler.getUsb().addObserver(&mUsbStats);
	
	pandaHandler.getUsb().setOperatingMode(usbMode);
	if (gpsFilename != NULL) {
		pandaHandler.getGps().saveToCsvFile(gpsFilename);
	}
	if (canFilename != NULL) {
		pandaHandler.getCan().saveToCsvFile(canFilename);
	}
	
	pandaHandler.initialize();
	
	
	pandaHandler.getUsb().setGmlanOrObdCanMode(0x00);	// 1 will set the Panda to CAN_MODE_OBD_CAN2, otherwise CAN_MODE_NORMAL
	
	// faking can data for debugging, if enabled
	Panda::CanFrame newFrame;
	int i = 0, j = 0, k = 0, l = 0, m = 0, n = 0, o = 0;
	if (fakeData) {
		newFrame.messageID = 384; // fake data stagnant
		newFrame.data[0] = 1;
		newFrame.dataLength = 4;
		mCanFrameStats.newDataNotification(&newFrame);
	}
	
	// If no new data shows up, we wil still update at 1Hz
	int loopCounter = 0;
	
	// Main UI loop
	CursesHandler* mCursesHandler = CursesHandler::getInstance();
	mCursesHandler->setupTerminal();
	mCursesHandler->updateScreen(pandaHandler,mCanFrameStats, mUsbStats); // Draw initial screen
	while (keepRunning == true) {
		// Update whenever there is new data.
		if (myObserver.checkNewData() || loopCounter++ >= 10) {
			mCursesHandler->updateScreen(pandaHandler, mCanFrameStats, mUsbStats);
			loopCounter = 0;
		}
		
		if(fakeData) {
			newFrame.messageID = 1100;		// fakes stats fast 10Hz
			newFrame.data[0] = 1;
			newFrame.dataLength = 1;
			mCanFrameStats.newDataNotification(&newFrame);
			
			if (k++ >= 2) {
				newFrame.messageID = 1050;	// fake stats slow 5Hz
				newFrame.data[0] = i++;
				newFrame.dataLength = 8;
				mCanFrameStats.newDataNotification(&newFrame);
				//mCanFrameStats.newDataNotification(&newFrame);
				k = 1;
			}
			
			if (j++ >= 4) {
				newFrame.messageID = 1025;	// fake stats slow 2.5Hz
				newFrame.data[0] = i++*16;
				mCanFrameStats.newDataNotification(&newFrame);
				//mCanFrameStats.newDataNotification(&newFrame);
				j = 1;
			}
			
			if (l++ >= 10) {
				newFrame.messageID = 1010;	// fake stats slow 1Hz
				newFrame.data[0] = i++;
				mCanFrameStats.newDataNotification(&newFrame);
				//mCanFrameStats.newDataNotification(&newFrame);
				l = 1;
			}
			
			
			
			newFrame.messageID = 0x801;	// fake at 10Hz
			newFrame.dataLength = CAN_DATA_MAX_LENGTH;
			memset(newFrame.data, 0, newFrame.dataLength);
			*((int*)&newFrame.data[32]) = m++;
			mCanFrameStats.newDataNotification(&newFrame);
			memset(newFrame.data, 0, newFrame.dataLength);
			
			if (n++ >= 10) {
				newFrame.messageID = 0x802;	// fake stats slow 1Hz
				newFrame.data[0] = o;
				newFrame.dataLength = Panda::dataLengthCodeToDataLength(o++);
				if (o >= 16) {
					o = 0;
				}
				mCanFrameStats.newDataNotification(&newFrame);
				//mCanFrameStats.newDataNotification(&newFrame);
				n = 1;
			}
		}
		
		usleep(1000000.0/10.0); // run at max ~10Hz
		
		if(mCursesHandler->getUserInput() == 0x1B) {	// Look for ESC button
			keepRunning = false;
		}
	}
	mCursesHandler->destroy();
	pandaHandler.stop();
	
	std::cout << "Done." << std::endl;
	return 0;
}
