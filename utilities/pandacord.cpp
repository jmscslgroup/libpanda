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
#include <getopt.h>
#include <cstring>

#include "panda.h"

static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}

// A simple concrete instance of a CAN listener
class SimpleCanObserver : public Panda::CanListener {
public:
	int getNotificationCount() {
		return notificationCount;
	}

private:
	int notificationCount = 0;
	void newDataNotification( Panda::CanFrame* canData ) {
		notificationCount++;
		if(notificationCount > 1000) {
			std::cerr << "c";
			notificationCount = 0;
		}
	}
};

// A simple concrete instance of a GPS listener
class SimpleGpsObserver : public Panda::GpsListener {
private:
	int notificationCount = 0;
	void newDataNotification( Panda::GpsData* gpsData ) {
		notificationCount++;
		if(notificationCount > 10) {
			std::cerr << "g";
			notificationCount = 0;
		}
	}
};


/*
 Argument setup
 */
void printUsage(const char* binary) {
	std::cout << "Usage: " << binary << " -[v] [-u usbmode] [-g gpsfile] [-c csvfile] [-n nmeafile]  [-r canfile]" << std::endl;
	std::cout << "   -v          : Verbose mode" << std::endl;
	std::cout << "   -u usbmode  : USB operating mode:" << std::endl;
	std::cout << "                   a: Asynchronous" << std::endl;
	std::cout << "                   s: Synchronous" << std::endl;
	std::cout << "                   i: Isochronous (not supported)" << std::endl;
	std::cout << "   -g gpsfile  : Filename to output GPS data in CSV format" << std::endl;
	std::cout << "   -n nmeafile : Filename to output GPS NMEA strings" << std::endl;
	std::cout << "   -c csvfile  : Filename to output CSV format CAN data" << std::endl;
	std::cout << "   -r canfile  : Filename to save raw messages from Panda CAN reads" << std::endl;
}

int verboseFlag = false;

static struct option long_options[] =
{
	{"verbose",    no_argument, &verboseFlag, 0},
	{"usbmode",    required_argument, NULL, 'u'},
	{"gpsfile",    required_argument, NULL, 'g'},
	{"gpscsvfile",    required_argument, NULL, 'n'},
	{"cancsvfile", required_argument, NULL, 'c'},
	{"canrawfile", required_argument, NULL, 'r'},
	{NULL, 0, NULL, 0}
};

void writeToFileThenClose(const char* filename, const char* data) {
	FILE* file = fopen( filename, "w+");
	fwrite( data, 1, strlen(data), file);
	fclose(file);
};

using namespace std;
int main(int argc, char **argv) {
	// Argument parsing
	Panda::UsbMode usbMode = Panda::MODE_ASYNCHRONOUS;
	const char*    gpsFilename = NULL;
	const char*   nmeaFilename = NULL;
	const char* canCsvFilename = NULL;
	const char* canRawFilename = NULL;
	int ch;
	while ((ch = getopt_long(argc, argv, "u:g:c:r:n:", long_options, NULL)) != -1)
	{
		switch (ch)
		{
			case 'u':
				switch (optarg[0]) {
					case 'a': usbMode = Panda::MODE_ASYNCHRONOUS; break;
					case 's': usbMode =  Panda::MODE_SYNCHRONOUS; break;
					case 'i': usbMode =  Panda::MODE_ISOCHRONOUS; break; };
				break;
			case 'g':    gpsFilename = optarg; break;
			case 'n':   nmeaFilename = optarg; break;
			case 'c': canCsvFilename = optarg; break;
			case 'r': canRawFilename = optarg; break;
			default:
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
				break;
		}
	}

	const char filenamePandaStatus[] = "/etc/libpanda.d/pandaRecording";
	const char filenameGpsStatus[] = "/etc/libpanda.d/pandaHaveGPS";
	writeToFileThenClose(filenamePandaStatus, "-1\n");
	writeToFileThenClose(filenameGpsStatus, "-1\n");

	std::cout << "Starting " << argv[0] << std::endl;

	//Set up graceful exit
	signal(SIGINT, killPanda);

	SimpleCanObserver canObserver;
	SimpleGpsObserver myGpsObserver;

	double epsilon = 0.2;	// If system time is off from GPS time by this amount, update time.
	Panda::SetSystemTimeObserver mSetSystemTimeObserver(epsilon);

	// Initialize Usb, this requires a conencted Panda
	Panda::Handler pandaHandler;
	pandaHandler.getUsb().setOperatingMode(usbMode);
	pandaHandler.addCanObserver(canObserver);
	pandaHandler.addGpsObserver(myGpsObserver);
	pandaHandler.addGpsObserver(mSetSystemTimeObserver);

	// Let's roll
	pandaHandler.initialize();

	writeToFileThenClose(filenameGpsStatus, "0\n");	// state 0: on but time not set


	std::cout << "Waiting to acquire satellites to set system time..." << std::endl;
	std::cout << " - Each \'.\' represents 100 NMEA messages received:" << std::endl;
	int lastNmeaMessageCount = 0;
	while ( !mSetSystemTimeObserver.hasTimeBeenSet() &&
		   keepRunning == true ) {
		if (pandaHandler.getGps().getData().successfulParseCount-lastNmeaMessageCount > 100) {
			std::cerr << ".";
			lastNmeaMessageCount = pandaHandler.getGps().getData().successfulParseCount;
		}
		usleep(10000);
	}
	writeToFileThenClose(filenameGpsStatus, "1\n");	// GPS time sync done
	writeToFileThenClose(filenamePandaStatus, "0\n"); // Recording should now start


	if (gpsFilename != NULL) {
		pandaHandler.getGps().saveToCsvFile(gpsFilename);
	}
	if (nmeaFilename != NULL) {
		pandaHandler.getGps().saveToFile(nmeaFilename);
	}
	if (canCsvFilename != NULL) {
		pandaHandler.getCan().saveToCsvFile(canCsvFilename);
	}
	if (canRawFilename != NULL) {
		pandaHandler.getCan().saveToFile(canRawFilename);
	}

	std::cout << "Time is synced with GPS!" << std::endl;
	std::cout << std::endl << "Press ctrl-c to exit" << std::endl;
	std::cout << " - Each \'c\' represents 1000 CAN notifications received." << std::endl;
	std::cout << " - Each \'.\' represents 100 NMEA messages received." << std::endl;
	std::cout << " - Each \'g\' represents 10 GPS notifications received." << std::endl;

	int priorCanCount = 0;
	int gpsHeartbeat = 0;
	while (keepRunning == true) {
		// GPS status checking:
		if (pandaHandler.getGps().getData().successfulParseCount-lastNmeaMessageCount > 100) {
			std::cerr << ".";
			lastNmeaMessageCount = pandaHandler.getGps().getData().successfulParseCount;
			gpsHeartbeat = 0;
			writeToFileThenClose(filenameGpsStatus, "1\n");
		}
		gpsHeartbeat++;
		if(gpsHeartbeat > 100) {
			writeToFileThenClose(filenameGpsStatus, "0\n");
		}

		// CAN recording status checking:
		if (priorCanCount == canObserver.getNotificationCount()) {
			writeToFileThenClose(filenamePandaStatus, "0\n");
			priorCanCount = canObserver.getNotificationCount();
		} else {
			writeToFileThenClose(filenamePandaStatus, "1\n");
		}



		usleep(10000);
	}
	//pandaHandler.stop();
	pandaHandler.stop();

	std::cout << "\rDone." << std::endl;
	return 0;
}
