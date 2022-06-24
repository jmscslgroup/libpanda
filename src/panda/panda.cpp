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

#include "panda.h"

#include "panda/obd-pid-definitions.h"

#include <unistd.h>
#include <iostream>
#include <cstring>

using namespace Panda;

void Panda::printVersions() {
	std::cout << "libpanda Git Version:" << LIBPANDA_VERSION << std::endl;
	std::cout << "   Panda Git Version:" << PANDA_VERSION << std::endl;
}

Handler::Handler()
:vModel(VEHICLE_MODEL_OTHER), vRegion(VEHICLE_REGION_OTHER), vMake(VEHICLE_MANUFACTURE_OTHER), vYear(0)
{
	mCan.setUsb(&mUsb);
	mGps.setUsb(&mUsb);
	
	mUsb.addObserver(&mCan);
	mUsb.addObserver(&mGps);
}

Handler::~Handler() {
	//stop();	// just to be safe
}

void Handler::initialize() {
	mUsb.initialize();
	mCan.initialize();
	mGps.initialize();

	mUsb.startRecording();
	mGps.startParsing();
	mCan.startParsing();
	
	requestVin();
	
	if (getVehicleManufacturer() == VEHICLE_MANUFACTURE_NISSAN &&
		mUsb.getHardware() == HARDWARE_RED_PANDA) {
		
		std::cout << "This is a RED Panda on a Nissan, adjusting CAN:" << std::endl;
		std::cout << "  |- Multiplexing CAN bus 1 from OBD to Harness CAN 1" << std::endl;
		mUsb.setGmlanOrObdCanMode(0x00);	// 1 will set the Panda to CAN_MODE_OBD_CAN2, otherwise CAN_MODE_NORMAL
		std::cout << "  |- Setting baudrate of bus 0 to CAN FD speeds" << std::endl;
		mUsb.setCanFdBaud(0, 20000);
		std::cout << "  |- Setting baudrate of bus 1 to CAN FD speeds" << std::endl;
		mUsb.setCanFdBaud(1, 20000);
		std::cout << "  |- Setting baudrate of bus 2 to CAN FD speeds" << std::endl;
		mUsb.setCanFdBaud(2, 20000);
	}
}

void Handler::requestVin() {
	mUsb.sendHeartBeat();
	// Read the VIN here:
	usleep(200000);
	std::cout << " - Attempting to read the VIN:" << std::endl;
	std::cout << " - Setting Safety to ELM327:" << std::endl;
	 mUsb.setSafetyMode(SAFETY_ELM327, 0);	// OBD II port
//	std::cout << "  |- Multiplexing CAN bus 1 to OBD connector" << std::endl;
	
	   mUsb.setGmlanOrObdCanMode(0x01);	// 1 will set the Panda to CAN_MODE_OBD_CAN2, otherwise CAN_MODE_NORMAL
	std::cout << " - Setting power mode to POWER_SAVE_STATUS_DISABLED:" << std::endl;
	mUsb.setPowerSaveEnable(POWER_SAVE_STATUS_DISABLED);
	usleep(100000);
	
	int vinAttempts = 0;
	bool extended = true;	// HACK
	while( vinAttempts++ < 10 ) {
		mUsb.sendHeartBeat();
		
		ObdPidRequest vinRequest(mCan);
		
		if (vinAttempts > 5 && extended == true) {	// HACK
			std::cout << "Extended Addressing OBD PID failed, attempting legacy addressed OBD PID reading" << std::endl;	// HACK
			extended = false;	// HACK
		}	// HACK
		
		std::cerr << " - - VIN Attempt " << vinAttempts << "/10..." << std::endl;
		
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
//			printf("Success! ");
			//				for (int i = 0; i < vinRequest.dataLength; i++) {
			//					printf("%c", vinRequest.data[i]);
			//				}
			//				printf("\n");
			// Save the VIN:
			FILE* file = fopen( "/etc/libpanda.d/vin", "w+");
			fwrite( vinRequest.data, 1, vinRequest.dataLength, file);
			fclose(file);
			
			// Notify a new vin has been read:
			file = fopen( "/etc/libpanda.d/newVin", "w+");
			fwrite( "1\n", 1, strlen("1\n"), file);
			fclose(file);
			
			forceSetVin(vinRequest.data);
			
			break;
		} else {
//			std::cerr << "Timeout" << std::endl;
		}
		
	}
	
	
	std::cout << " - Setting Safety to SAFETY_NOOUTPUT:" << std::endl;
	mUsb.setSafetyMode(SAFETY_NOOUTPUT, 0);	// OBD II port
	
}

void Handler::forceSetVin(const unsigned char* vin) {
	printVin(vin);
	
	vMake = vinToManufacturer(vin);
	vModel = vinToModel(vin);
	vYear = vinToYear(vin);
	vRegion = vinToRegion(vin);
}


void Handler::stop() {
	std::cout << "Stopping GPS...";
	mGps.stopParsing();
	std::cout << "Done." << std::endl;
	std::cout << "Stopping CAN...";
	mCan.stopParsing();
	std::cout << "Done." << std::endl;
	std::cout << "Stopping USB...";
	mUsb.stopRecording();
	std::cout << "Done." << std::endl;
}

void Handler::addGpsObserver(GpsListener& gps) {
	mGps.addObserver(&gps);
}

void Handler::addCanObserver(CanListener& can) {
	mCan.addObserver(&can);
}

void Handler::addUsbObserver(UsbListener& usb) {
	mUsb.addObserver(&usb);
}

Gps& Handler::getGps() {
	return mGps;
}

Can& Handler::getCan() {
	return mCan;
}

Usb& Handler::getUsb() {
	return mUsb;
}

VehicleManufacturer Handler::getVehicleManufacturer() {
	return vMake;
}
