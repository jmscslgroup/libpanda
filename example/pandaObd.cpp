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
#include "panda/obd-pid.h"

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
	
	pandaHandler.initialize();	// starts USB and GPS

	usleep(500000);	// This gives time for USB/Panda to initialize
	
	std::cout << "Attempting to acquire VIN..." << std::endl;
	
	Panda::ObdPidRequest vinRequest(pandaHandler);
	
	int i = 2;
	while (keepRunning == true) {
		vinRequest.request(0x09, 0x02);
		
		sleep(1);	// only once every 1 seconds
		
		if (vinRequest.complete()) {	// Only run until time has been checked/
			keepRunning = false;
			break;
		}
		std::cout << " - Attempt " << i++ << std::endl;
	}
	
	if (vinRequest.complete()) {
		printf(" - Got a VIN: ");
		for (int i = 0; i < vinRequest.dataLength; i++) {
			printf("%c", vinRequest.data[i]);
		}
		printf("\n");
	}
	
	keepRunning = true;
	std::cout << "Attempting to acquire RPM..." << std::endl;
	Panda::ObdPidRequest rpmRequest(pandaHandler);
	i = 2;
	while (keepRunning == true) {
		rpmRequest.request(0x01, 0x0c);
		
		sleep(1);	// only once every 1 seconds
		
		if (rpmRequest.complete()) {	// Only run until time has been checked/
			keepRunning = false;
			break;
		}
		
		std::cout << " - Attempt " << i++ << std::endl;
	}
	if (rpmRequest.complete()) {
		printf(" - Got RPM: ");
//		for (int i = 0; i < rpmRequest.dataLength; i++) {
//			printf("0x%02x ", rpmRequest.data[i]);
//		}
//		printf("\n");
		double rpm = ((((int)rpmRequest.data[0]) << 8) + (int)rpmRequest.data[1])/4.0;
		printf("%02f\n", rpm);
	}
	
	keepRunning = true;
	std::cout << "Attempting to acquire MAF sensor..." << std::endl;
	Panda::ObdPidRequest fuelRequest(pandaHandler);
	i = 2;
	while (keepRunning == true) {
		fuelRequest.request(0x01, 0x10);
		
		sleep(1);	// only once every 1 seconds
		
		if (fuelRequest.complete()) {	// Only run until time has been checked/
			keepRunning = false;
			break;
		}
		std::cout << " - Attempt " << i++ << std::endl;
	}
	if (fuelRequest.complete()) {
		printf(" - Got MAF Sensor: ");
//		for (int i = 0; i < fuelRequest.dataLength; i++) {
//			printf("0x%02x ", fuelRequest.data[i]);
//		}
//		printf("\n");
		double fuelRate = ((((int)fuelRequest.data[0]) << 8) + (int)fuelRequest.data[1])/100.0;
		printf("%02f grams/sec\n", fuelRate);
	}
	
	
	keepRunning = true;
	std::cout << "Attempting to acquire Fuel..." << std::endl;
	//   Panda::ObdPidRequest fuelRequest(pandaHandler);
	i = 2;
	while (keepRunning == true) {
		fuelRequest.request(0x01, 0x9d);
		
		sleep(1);	// only once every 1 seconds
		
		if (fuelRequest.complete()) {	// Only run until time has been checked/
			keepRunning = false;
			break;
		}
		std::cout << " - Attempt " << i++ << std::endl;
	}
	if (fuelRequest.complete()) {
		printf(" - Got Fuel Rate: ");
		printf(" Raw: ");
		for (int i = 0; i < fuelRequest.dataLength; i++) {
			printf("0x%02x ", fuelRequest.data[i]);
		}
		printf("\n");
//		double fuelRate = ((((int)fuelRequest.data[0]) << 8) + (int)fuelRequest.data[1])/100.0;
//		printf("%02f grams/sec\n", fuelRate);
	}
//	
//	keepRunning = true;
//	std::cout << "Attempting to acquire Transmission Gear..." << std::endl;
//	//   Panda::ObdPidRequest fuelRequest(pandaHandler);
//	i = 2;
//	while (keepRunning == true) {
//		fuelRequest.request(0x01, 0xa2);
//		
//		sleep(1);	// only once every 1 seconds
//		
//		if (fuelRequest.complete()) {	// Only run until time has been checked/
//			keepRunning = false;
//			break;
//		}
//		std::cout << " - Attempt " << i++ << std::endl;
//	}
//	if (fuelRequest.complete()) {
//		printf(" - Got Transmission Gear: ");
//		printf(" Raw: ");
//		for (int i = 0; i < fuelRequest.dataLength; i++) {
//			printf("0x%02x ", fuelRequest.data[i]);
//		}
//		printf("\n");
////		double fuelRate = ((((int)fuelRequest.data[0]) << 8) + (int)fuelRequest.data[1])/100.0;
////		printf("%02f grams/sec\n", fuelRate);
//	}
	
	
	
	
	
	pandaHandler.stop();

	std::cout << "\rDone." << std::endl;
	return 0;
}
