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
	std::cout << "Usage: " << binary << " -[v] [-u <usbmode>] [-g <gpsfile>]" << std::endl;
	std::cout << "   -v            : Verbose mode" << std::endl;
	std::cout << "   -u <usbmode>  : USB operating mode:" << std::endl;
	std::cout << "                     a: Asynchronous" << std::endl;
	std::cout << "                     s: Synchronous" << std::endl;
	std::cout << "                     i: Isochronous" << std::endl;
	std::cout << "   -g <gpsfile>  : Filename to output GPS NMEA strings" << std::endl;
}

int verboseFlag = false;

static struct option long_options[] =
{
	{"verbose", no_argument, &verboseFlag, 0},
	{"usbmode", required_argument, NULL, 'u'},
	{"gpsfile", required_argument, NULL, 'g'},
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
 Main
 */
int main(int argc, char **argv) {
	// Argument parsing
	Panda::UsbMode usbMode = Panda::MODE_ASYNCHRONOUS;
	const char* gpsFilename = NULL;
	int ch;
	// loop over all of the options
	while ((ch = getopt_long(argc, argv, "u:g:", long_options, NULL)) != -1)
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

	// Initialize Usb, this requires a conencted Panda
	Panda::Handler pandaHandler;
	pandaHandler.addCanObserver(myObserver);
	pandaHandler.addGpsObserver(myObserver);

	pandaHandler.getUsb().setOperatingMode(usbMode);
	if (gpsFilename != NULL) {
		pandaHandler.getGps().saveToFile(gpsFilename);
	}

	pandaHandler.initialize();

	
	// Main UI loop
	CursesHandler* mCursesHandler = CursesHandler::getInstance();
	mCursesHandler->setupTerminal();
	mCursesHandler->updateScreen(pandaHandler); // Draw initial screen
	while (keepRunning == true) {
		// Update whenever there is new data.
		if (myObserver.checkNewData()) {
			mCursesHandler->updateScreen(pandaHandler);
		}

		usleep(1000000.0/30.0); // run at ~30Hz

		if(mCursesHandler->getUserInput() == 0x1B) {	// Look for ESC button
			keepRunning = false;
		}
	}
	mCursesHandler->destroy();
	pandaHandler.stop();

	std::cout << "Done." << std::endl;
	return 0;
}
