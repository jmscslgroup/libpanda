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

#include "panda/vin.h"
#include "panda/obd-pid-definitions.h"

using namespace Panda;


int Panda::vinToYear( unsigned char* vin ) {
	unsigned char yearCode = vin[9];
	if (yearCode >= '1' && yearCode <= '9') {
		return 2000 + (int)(yearCode-'0');
	}
	if (yearCode > 'I') {	// 'I' is an invalid letter in a VIN, so it is skipped
		yearCode -= 1;
	}
	return 2010 + (int)(yearCode-'A');
}

VehicleManufacturer Panda::vinToManufacturer( unsigned char* vin ) {
	switch (vin[1]) {
		case 'T': return VEHICLE_MANUFACTURE_TOYOTA;
		case 'N': return VEHICLE_MANUFACTURE_NISSAN;
		case 'F': return VEHICLE_MANUFACTURE_HONDA;
		case 'G': return VEHICLE_MANUFACTURE_GM;
			
		default:
			break;
	}
	return VEHICLE_MANUFACTURE_OTHER;
}

const char* Panda::vehicleManufacturerToString( VehicleManufacturer vm ) {
	switch (vm) {
		case VEHICLE_MANUFACTURE_TOYOTA: return "Toyota";
		case VEHICLE_MANUFACTURE_HONDA: return "Honda";
		case VEHICLE_MANUFACTURE_NISSAN: return "Nissan";
		case VEHICLE_MANUFACTURE_GM: return "General Motors";
			
		case VEHICLE_MANUFACTURE_OTHER:
		default:
			break;
	}
	
	return "Unknown by libpanda";
}


void Panda::printVin( unsigned char* vin) {
	printf("Information about VIN: %s\n", vin);
	printf(" - This vehicle is made by :%s\n", vehicleManufacturerToString(vinToManufacturer(vin)));
	printf(" - This vehicle was made in:%d\n", vinToYear(vin));
}

