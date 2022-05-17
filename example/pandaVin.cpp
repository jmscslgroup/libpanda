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
#include <unistd.h>	// usleep()
#include <cmath>	// fabs()
#include <time.h>
#include <iomanip>
#include <ctime>
#include <cstring> // memcpy

#include "panda.h"
#include "panda/vin.h"

// A ctrl-c handler for safe panda handler destruction
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}


int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;
	

	//Set up graceful exit
	signal(SIGINT, killPanda);

	// Initialize Usb, this requires a conencted Panda
	Panda::Handler pandaHandler;
	/*
	Panda::VinReader mVinReader;
	mVinReader.setPandaHandler(pandaHandler);
	
	if (argc > 1) {
		std::cout << "Saving VIN to file:" << argv[1] << std::endl;
		mVinReader.saveToFile(argv[1]);
	}
	
	pandaHandler.initialize();	// starts USB and GPS

	usleep(500000);	// This gives time for USB/Panda to initialize
	
	std::cout << "Waiting to acquire VIN..." << std::endl;
	int i = 1;
	while (keepRunning == true) {
		std::cout << " - Attempt " << i++ << std::endl;
		mVinReader.requestVin();
		
		sleep(1);	// only once every 1 seconds
		
		if (mVinReader.vinRead()) {	// Only run until time has been checked/
			keepRunning = false;
		}
	}
	
	std::cout << "VIN for this vehicle: " << mVinReader.vinResult() << std::endl;
	*/
	pandaHandler.stop();

	std::cout << "\rDone." << std::endl;
	return 0;
}
