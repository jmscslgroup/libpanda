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
#include <cmath>

#include <string.h>
#include <time.h>

#include "panda.h"

//#include <time.h>


static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr <<  "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}

// a simple concrete instance of a usb listener for debugging
class SimpleCanObserver : public Panda::CanListener {
private:
	void newDataNotification( Panda::CanFrame* frame ) {
		printf("Can Success! Received frame %d,%d,", frame->messageID, frame->bus);
		for (int i = frame->dataLength-1; i >= 0; i--) {	// flipped direction
			printf("%02x", frame->data[i]);
		}
		printf(",%d\tBusTime: %d", frame->dataLength, frame->busTime);
		std::cout << std::endl;
	}
};

class SimpleGpsObserver : public Panda::GpsListener {

private:
	void newDataNotification( Panda::GpsData* gpsData) {
		//std::cout << std::endl << "Gps Notification Success! ";

		//		std::cout << std::endl << "\rGPS status:" << gpsData->info.status;
		////		std::cout << " - GPS Time:" << ctime(&(gpsData->time.time));
		////		std::cout << " - My  Time:" << ctime(&rawTime);
		//		std::cout << " - GPS Time:" << gpsData->time.month << "/" << gpsData->time.day << "/" << gpsData->time.year << " " << gpsData->time.hour << ":" << gpsData->time.minute << ":" << gpsData->time.second << std::endl;
		//		std::cout << " - Time:" << timeinfo->tm_mon+1 << "/" << timeinfo->tm_mday << "/" << timeinfo->tm_year+1900 << " " << timeinfo->tm_hour << ":" << timeinfo->tm_min << ":" << (double)tv.tv_sec + (double)tv.tv_usec/1000000.0 << std::endl;
		//		std::cout << " - Long:" << gpsData->pose.longitude << "\tLat:" << gpsData->pose.latitude << std::endl;

	}
};



using namespace std;
int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;

	//Set up graceful exit
	signal(SIGINT, killPanda);

	SimpleCanObserver canObserver;
	SimpleGpsObserver myGpsObserver;

	// Initialize Usb, this requires a conencted Panda
	Panda::Handler pandaHandler;
	pandaHandler.addCanObserver(canObserver);
	pandaHandler.addGpsObserver(myGpsObserver);

	pandaHandler.getGps().saveToFile("nmeaDump.txt");

	pandaHandler.initialize();

	while (keepRunning == true) {
////		std::cerr << "Total bytes Read: " << usb.getBytesReceived() << ", Total bytes Sent: " << usb.getBytesSent() << "\r";//std::endl;
//
//		if((lengthRead = usb.serialRead(inputdata)) > 0 ) {
//			inputdata[lengthRead] = 0;	// just for printing to the terminal
//			//std::cerr << "Read bytes:" << lengthRead << std::endl;
//			//std::cerr << inputdata;// << "|||" << std::endl << std::endl;
//			nmeaFile << inputdata;
//			CNMEAParserData::ERROR_E nErr;
//			if ((nErr = NMEAParser.ProcessNMEABuffer((char*)inputdata, lengthRead)) != CNMEAParserData::ERROR_OK) {
//				printf("NMEA Parser ProcessNMEABuffer Failed and returned error: %d\n", nErr);
//
//				break;//return -1;
//			}
//			//			parser.readBuffer(inputdata, lengthRead);
////			for (int i = 0; i < lengthRead; i++) {
////				parser.readByte(inputdata[i]);
////			}
//		}
//
//		usleep(50000);

		Panda::CanFrame garbage;
		char testData[] = "\x7a\x1d\x10\xfe\xcc\x03\xe1\xde";
		garbage.messageID = 384;
		garbage.bus = 1;
		garbage.dataLength = 8;
		for (int i = 0; i < garbage.dataLength; i++) {
			garbage.data[i] = testData[i];
		}
		pandaHandler.getCan().sendMessage(garbage);

		usleep(500000);
		//std::cerr << "."; //heartbeat
	}
	//pandaHandler.stop();
	pandaHandler.stop();

	std::cout << "\rDone." << std::endl;
	return 0;
}
