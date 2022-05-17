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

VehicleRegion Panda::vinToRegion( unsigned char* vin ) {
	switch (vin[0]) {
		case '1':
		case '4':
		case '5': return VEHICLE_REGION_USA;
		case '2': return VEHICLE_REGION_CANADA;
		case 'J': return VEHICLE_REGION_JAPAN;
			
		default:
			break;
	}
	return VEHICLE_REGION_OTHER;
}

const char* Panda::vehicleManufacturerToString( VehicleManufacturer vm ) {
	switch (vm) {
		case VEHICLE_MANUFACTURE_TOYOTA: return "Toyota";
		case VEHICLE_MANUFACTURE_HONDA: return "Honda";
		case VEHICLE_MANUFACTURE_NISSAN: return "Nissan";
		case VEHICLE_MANUFACTURE_GM: return "General Motors";
			
		case VEHICLE_MANUFACTURE_OTHER: break;
	}
	
	return "Unknown Manufacturer";
}

const char* Panda::vehicleModelToString( VehicleModel vm ) {
	switch (vm) {
		case VEHICLE_MODEL_HONDA_PILOT: return "Pilot";
		case VEHICLE_MODEL_NISSAN_ROGUE: return "Rogue";
		case VEHICLE_MODEL_TOYOTA_RAV4: return "RAV4";
		case VEHICLE_MODEL_GM_CADIALLC_XT5: return "Cadillac XT5";
		case VEHICLE_MODEL_OTHER: break;
	}
	return "Unknown Model";
}
const char* Panda::vehicleRegionToString( VehicleRegion vr ) {
	switch (vr) {
		case VEHICLE_REGION_USA: return "USA";
		case VEHICLE_REGION_CANADA: return "Canada";
		case VEHICLE_REGION_JAPAN: return "Japan";
		case VEHICLE_REGION_OTHER: break;
	}
	return "Unknown Region";
}

VehicleModel Panda::vinToModel( unsigned char* vin ) {
	VehicleManufacturer manufacturer = vinToManufacturer(vin);
	VehicleRegion region = vinToRegion(vin);
	
	// This is awful, major TODO: make this into a lookup table
	if (region == VEHICLE_REGION_USA || region == VEHICLE_REGION_CANADA) {
		switch (manufacturer) {
			case VEHICLE_MANUFACTURE_TOYOTA:
				switch (vin[2]) {
					case '3': return VEHICLE_MODEL_TOYOTA_RAV4;
					default: break;
				}
				break;
			case VEHICLE_MANUFACTURE_HONDA:
				switch (vin[2]) {
					case 'N': return VEHICLE_MODEL_HONDA_PILOT;
					default: break;
				}
				break;
			case VEHICLE_MANUFACTURE_GM:
				switch (vin[2]) {
					case 'Y': return VEHICLE_MODEL_GM_CADIALLC_XT5;
					default: break;
				}
				break;
				
				
			case VEHICLE_MANUFACTURE_OTHER:
			default:
				break;
		}
	} else if (region == VEHICLE_REGION_JAPAN) {
		switch (manufacturer) {
			case VEHICLE_MANUFACTURE_TOYOTA:
				switch (vin[2]) {
					case 'M': return VEHICLE_MODEL_TOYOTA_RAV4;
					default: break;
				}
			case VEHICLE_MANUFACTURE_NISSAN:
				switch (vin[2]) {
					case '8': return VEHICLE_MODEL_NISSAN_ROGUE;
					default: break;
				}
			default:
				break;
		}
		
	}
	return VEHICLE_MODEL_OTHER;
}

void Panda::printVin( unsigned char* vin) {
	printf("Information about VIN: %s\n", vin);
	printf(" - Manufacturer :%s\n", vehicleManufacturerToString(vinToManufacturer(vin)));
	printf(" - Year         :%d\n", vinToYear(vin));
	printf(" - Region       :%s\n", vehicleRegionToString(vinToRegion(vin)));
	printf(" - Model        :%s\n", vehicleModelToString(vinToModel(vin)));
	
}

