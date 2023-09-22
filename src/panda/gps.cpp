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

#include "panda/gps.h"
#include "panda/pandadefinitions.h"

#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <termios.h>

//#define GPS_VERBOSE

using namespace Panda;

Gps::Gps()
:csvDump(NULL) {
	fidGps = -1;
	gpsConfigState = GPS_CONFIG_IDLE;
	responseLength = 0;
}

Gps::~Gps() {
	stop();
	WaitForInternalThreadToExit();
	if (csvDump != NULL) {
		fclose(csvDump);
	}
	if (nmeaDump.is_open()) {
		nmeaDump.close();
	}
}

void Gps::saveToCsvFile(const char* filename) {
	FILE* csvDumpTemp = fopen(filename, "w");
	fprintf(csvDumpTemp, "Gpstime,Status,Long,Lat,Alt,HDOP,PDOP,VDOP,Systime\n");
	this->csvDump = csvDumpTemp;
	remove("/etc/libpanda.d/currentGps.csv");
	symlink(filename,"/etc/libpanda.d/currentGps.csv");
}
void Gps::writeCsvToFile(GpsData& state) {
	if (csvDump) {
		time_t gpsTime_t = mktime(&state.time);
//		struct timeval gpsTime;
//		gpsTime.tv_sec = gpsTime_t;
//		gpsTime.tv_usec = (state.timeMilliseconds)*1000;
		
		// Get system time to check against GPS time skew
		struct timeval sysTime;
		gettimeofday(&sysTime, NULL);

		fprintf(csvDump, "%d.%06d,%c,%0.7f,%0.7f,%0.1f,%0.2f,%0.2f,%0.2f,%d.%06d\r\n",
				(unsigned int)gpsTime_t,
				(state.timeMilliseconds)*1000,
				state.info.status,
				state.pose.longitude,
				state.pose.latitude,
				state.pose.altitude,
				state.quality.HDOP,
				state.quality.PDOP,
				state.quality.VDOP,
				(unsigned int)sysTime.tv_sec,
				(int)sysTime.tv_usec);
	}
}

const GpsData& Gps::getData() const {
	return state;
}

void Gps::setUsb( Panda::Usb* usbHandler ) {
	this->usbHandler = usbHandler;
}

void Gps::handleConfiguration() {
	if (ubxCommands.empty() && gpsConfigState == GPS_CONFIG_IDLE) {
		return;
	}
	
	switch (gpsConfigState) {
		case GPS_CONFIG_IDLE:
			if (ubxCommands.empty()) {
				return;
			}
			gpsConfigState = GPS_CONFIG_START;
			// No break, continue

		case GPS_CONFIG_START: // Grab from queue
			ubxCurrentCommand = ubxCommands.front();
			ubxCommands.pop();
			
			ubxSendAttempt = 1;
			configurationWaitCounter = 0;
			
			gpsConfigState = GPS_CONFIG_SEND;
			// no break, send immediately:
			
		case GPS_CONFIG_SEND: // sending
			
			ubxLastSentClass = ubxCurrentCommand[2];
			ubxLastSentId = ubxCurrentCommand[3];
			
			printf(" - Sending UBX-%s...\n", ubxClassIdToString(ubxLastSentClass, ubxLastSentId).c_str());
			gpsSend(ubxCurrentCommand.c_str(), getUbxLength(ubxCurrentCommand.c_str()));

			gpsConfigState = GPS_CONFIG_WAIT;
			break;
			
		case GPS_CONFIG_WAIT:	// waiting...
			configurationWaitCounter++;
			if (configurationWaitCounter % 40 == 0) {
				std::cout << " --- Gps UBX-" << ubxClassIdToString(ubxLastSentClass, ubxLastSentId) << ": Timed out... attempting again " << ubxSendAttempt++ << std::endl;
				gpsConfigState = GPS_CONFIG_SEND; // send again
			}
			if (configurationWaitCounter >= 400) {
				std::cout << " --- Gps UBX-" << ubxClassIdToString(ubxLastSentClass, ubxLastSentId) << ": Too many attempts" << std::endl;
				configurationWaitCounter = 0;
				gpsConfigState = GPS_CONFIG_FAIL;
			}
			break;
			
		case GPS_CONFIG_ACK: // next task
			std::cout << " --- Gps UBX-" << ubxClassIdToString(ubxLastSentClass, ubxLastSentId) << ": Acknowledged!" << std::endl;
			gpsConfigState = GPS_CONFIG_COMPLETE;
			break;
			
		case GPS_CONFIG_NACK:
			std::cout << " --- Gps UBX-" << ubxClassIdToString(ubxLastSentClass, ubxLastSentId) << ": GPS module rejected command" << std::endl;
			gpsConfigState = GPS_CONFIG_COMPLETE;
			break;
			
		case GPS_CONFIG_SUCCESS:
			std::cout << " --- Gps UBX-" << ubxClassIdToString(ubxLastSentClass, ubxLastSentId) << " Success!" << std::endl;
//			std::cout << " --- Gps UBX-" << ubxClassIdToString(ubxLastSentClass, ubxLastSentId) << " response: ";
//			for (int i = 0; i < responseLength; i++) {
//				printf("0x%02X ", responsePayload[i]);
//			}
//			printf("\n");
			gpsConfigState = GPS_CONFIG_COMPLETE;
			break;
			
		case GPS_CONFIG_FAIL:
			std::cout << " --- Gps UBX-" << ubxClassIdToString(ubxLastSentClass, ubxLastSentId) << ": Failed to configure" << std::endl;
			gpsConfigState = GPS_CONFIG_COMPLETE;
			break;
			
		case GPS_CONFIG_COMPLETE:
			if (ubxCommands.empty()) {
				gpsConfigState = GPS_CONFIG_IDLE;
			} else {
				gpsConfigState = GPS_CONFIG_START;
			}
			break;
	}
	
}

bool Gps::busyUbx() {
	return gpsConfigState != GPS_CONFIG_IDLE;
}

// Returns the result of the latest successul UBX read
int Gps::getUbxResponse(char* result) {
	memcpy(result, responsePayload, responseLength);
	return responseLength;
}

void Gps::notificationUbxMessage(char mClass, char mId, short length, unsigned char* payload) {
	
	if (mClass == UBX_CLASS_ACK ) {
		if (payload[0] == ubxLastSentClass && payload[1] == ubxLastSentId) {
			if (mId == UBX_ID_ACK_ACK ) {
				gpsConfigState = GPS_CONFIG_ACK;
				return;
			} else if (mId == UBX_ID_ACK_NACK ) {
				gpsConfigState = GPS_CONFIG_NACK;
				return;
			}
		}
	}
	if (mClass == ubxLastSentClass && mId == ubxLastSentId) {
		memcpy(responsePayload, payload, length);
		responseLength = length;
//		std::cout << " --- Gps UBX-" << ubxClassIdToString(ubxLastSentClass, ubxLastSentId) << " response: ";
//		   for (int i = 0; i < responseLength; i++) {
//			   printf("0x%02X ", responsePayload[i]);
//		   }
//		   printf("\n");
		
		if (mClass != UBX_CLASS_CFG) {	// CFG commands respond with an additional ACK, so wait
			gpsConfigState = GPS_CONFIG_SUCCESS;
		}
		return;
	}
	
	
	printf("WARNING: Gps::notificationUbxMessage() unhandled UBX response for Class/Id %s, Length %d, Payload: ", ubxClassIdToString( mClass, mId).c_str(), length);
	for (int i = 0; i < length; i++) {
		printf("0x%02X ", payload[i]);
	}
	
}

int Gps::parseUbx(unsigned char* buffer, int length) {
	
	if (length < 8) {
		std::cerr << "GPS: Incomplete incoming UBX transmission" << std::endl;
		return 0;
	}
	int ubxLength = (int)buffer[4] | ((int)buffer[5] << 8);
	if (ubxLength + 4 > length) {
		std::cerr << "GPS: Incomplete incoming UBX transmission (type 2)" << std::endl;
		return 0;
	}
//	printf("Length %d\n", ubxLength);
	unsigned char copyOfUbx[ubxLength + 8];
	memcpy(copyOfUbx, buffer, ubxLength + 6);
	Panda::setUbxChecksum((char*)copyOfUbx);
	
	if (copyOfUbx[ubxLength+6] != buffer[ubxLength+6] ||
		copyOfUbx[ubxLength+7] != buffer[ubxLength+7]) {
		std::cerr << "GPS: UBX protocol recieved invalid checksum:";
//		for ( int i = 0; i < ubxLength + 8; i++) {
//			printf("0x%02X ", copyOfUbx[i]);
//		}
//		printf("\nublox:");
		for ( int i = 0; i < ubxLength + 8; i++) {
			printf("0x%02X ", buffer[i]);
		}
		printf("\n");
		return 0;
	}
//	std::cout << "Check this out! --> ";
//	for ( int i = 0; i < ubxLength + 8; i++) {
//		printf("0x%02X ", copyOfUbx[i]);
//	}
//	printf("\n");
	notificationUbxMessage(copyOfUbx[2], copyOfUbx[3], ubxLength, &copyOfUbx[6]);
	

	
	
	return ubxLength + 8;
}


void Gps::processUart(char* buffer, int bufferLength) {
	for (int i = 0;  i < bufferLength-1; i++) {
		if (((unsigned char*)buffer)[i] == 0xB5 &&
			((unsigned char*)buffer)[i+1] == 0x62) {
//			printf("Found an SOB: ");
//			for (int j = i; j < bufferLength; j++) {
//				printf("0x%02X ", buffer[j]);
//			}
//			printf("\n");
			int parseLength = parseUbx((unsigned char*)&buffer[i], bufferLength-i);
			if (parseLength > 0) {
//				buffer += i+parseLength;
//				bufferLength -= i+parseLength;
//				i = 0;
//				printf("new buffer: ");
//				for (int j = 0; j < bufferLength; j++) {
//					printf("0x%02X ", buffer[j]);
//				}
				
				processUart(&buffer[i+parseLength], bufferLength-i-parseLength);
				return;
			}
		}
	}

	CNMEAParserData::ERROR_E nErr;
	if ((nErr = this->ProcessNMEABuffer((char*)buffer, bufferLength)) != CNMEAParserData::ERROR_OK) {
		std::cerr << "NMEA Parser Gps::ProcessNMEABuffer Failed and returned error: " << nErr << std::endl;
	}
	if (nmeaDump.is_open()) {
//		uartBuffer[lengthRead] = 0;	// just for printing to the terminal
		nmeaDump << std::string(buffer,bufferLength);
	}

	//usbHandler->requestUartData();	// buffer may still have more, make an immediate new request

	//std::cout << "done with notificationUartRead()" << std::endl;
}

void Gps::notificationUartRead(char* buffer, size_t bufferLength) {
	currentlyReceiving = bufferLength > 0;

	resume();	// will request more data
	
//	printf("Gps::notificationUartRead(): ");
//	for (int j = 0; j < bufferLength; j++) {
//			printf("0x%02X ", buffer[j]);
//	}
//	printf("\n");
	
	
	processUart(buffer, bufferLength);
	
	
}

void Gps::doAction() {
	if (usbHandler == NULL && !usingExternalGps) {
		std::cerr << "ERROR: Gps::doAction(): No Usb Handler or external USB GPS set for Panda::Gps!" << std::endl;
		usleep(1000);
		return;
	}

	if (!currentlyReceiving) {
		usleep(10000);	// thread does not need to be particularly fast
	}

	// Tell this thread to pause, wait for request to complete
	pause();	// yes, before requesting data, otherwise synchronous USB deadlocks

	if (usingExternalGps) {
		char buffer[2048];
		int length = read(fidGps, buffer, sizeof(buffer));
//		std::cout << "GPS Read length " << length << ": ";
//		for (int i = 0; i < length; i++) {
//			printf("%c", buffer[i]);
//		}
//	std::cout << std::endl;
		if (length > 0) {
			notificationUartRead(buffer, length);	// HACK
		}
	} else {
		usbHandler->requestUartData();
	}
	
	handleConfiguration();
//	std::cout << "Requeting new data" << std::endl;
}

bool Gps::isReady() {
	return state.info.status == 'A';
}

bool Gps::available() {
	if (usbHandler != NULL && usbHandler->hasGpsSupport()) {
		return true;
	}
	return usingExternalGps;
}

void Gps::addObserver( GpsListener* listener ) {
	listeners.push_back(listener);
}

void Gps::startParsing() {
	if(usbHandler == NULL && usingExternalGps == false) {
		std::cerr << "ERROR: Gps::startParsing(): No Usb Handler nor external USB GPS set for Panda::Gps!" << std::endl;
		return;
	}

	if ( !usingExternalGps && !usbHandler->hasGpsSupport() ) {
		std::cerr << "WARNING: Gps::startParsing(): No available GPS!" << std::endl;
		return;
	}

	start();
}

void Gps::stopParsing() {
	stop();
	if(fidGps != -1) {
		close(fidGps);
	}
	WaitForInternalThreadToExit();
}

void Gps::saveToFile( const char* filename ) {
	nmeaDump.open(filename);
}


// GPS overloads:
void Gps::OnError(CNMEAParserData::ERROR_E nError, char *pCmd) {
	printf("Error: Gps::OnError(): for Cmd: \"%s\", Number: %d\n", pCmd, nError);
}

CNMEAParserData::ERROR_E Gps::ProcessRxCommand(char *pCmd, char *pData) {
	bool newData = false;
	state.successfulParseCount++;

	// Call base class to process the command
	CNMEAParser::ProcessRxCommand(pCmd, pData);

#ifdef GPS_VERBOSE
	printf("GPS - Cmd: %s\tData: %s\n", pCmd, pData);
#endif

	// Check if this is the GPGGA command. If it is, then display some data
	if (strcmp(pCmd, "GNGGA") == 0 || strcmp(pCmd, "GPGGA") == 0) {
		CNMEAParserData::GGA_DATA_T ggaData;
		CNMEAParserData::ERROR_E error;
		if (strcmp(pCmd, "GNGGA") == 0) {
			error = GetGNGGA(ggaData);
		} else {
			error = GetGPGGA(ggaData);
		}
		if(error == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
			printf("\nCmd: %s Data: %s\n", pCmd, pData);
			printf("GNGGA Parsed!  Time, position, and fix related data of the receiver\n");
			printf("   Time:                %02d:%02d:%02d\n", ggaData.m_nHour, ggaData.m_nMinute, ggaData.m_nSecond);
			printf("   Latitude:            %f\n", ggaData.m_dLatitude);
			printf("   Longitude:           %f\n", ggaData.m_dLongitude);
			printf("   Altitude:            %.01fM\n", ggaData.m_dAltitudeMSL);
			printf("   GPS Quality:         %d\n", ggaData.m_nGPSQuality);
			printf("   Satellites in use:   %d\n", ggaData.m_nSatsInView);
			printf("   HDOP:                %.02f\n", ggaData.m_dHDOP);
			printf("   Differential ID:     %d\n", ggaData.m_nDifferentialID);
			printf("   Differential age:    %f\n", ggaData.m_dDifferentialAge);
			printf("   Geoidal Separation:  %f\n", ggaData.m_dGeoidalSep);
			printf("   Vertical Speed:      %.02f\n", ggaData.m_dVertSpeed);
#endif

			state.time.tm_hour = ggaData.m_nHour;				// Handled in two message types
			state.time.tm_min = ggaData.m_nMinute;			// Handled in two message types
			//state.time.second = ggaData.m_nSecond;			// Handled in two message types // Type mismatch

			state.pose.latitude = ggaData.m_dLatitude;		// Handled in two message types
			state.pose.longitude = ggaData.m_dLongitude;	// Handled in two message types
			state.pose.altitude = ggaData.m_dAltitudeMSL;	// Handled in two message types
			state.pose.geoidalSeparation = ggaData.m_dGeoidalSep;

			state.satelitesInUse = ggaData.m_nSatsInView; // This is titled "in view" but I think it means "in use"

			state.quality.indicator = ggaData.m_nGPSQuality;
			state.quality.HDOP = ggaData.m_dHDOP;			// Handled in two message types

			state.info.diffId = ggaData.m_nDifferentialID;
			state.info.diffAge = ggaData.m_dDifferentialAge;

			state.motion.verticalSpeed = ggaData.m_dVertSpeed;


			//newData = true; // This doesn't give us much new except for vertical speed, which is of rare interest.  also no date
		}
	} else if (strcmp(pCmd, "GLGSV") == 0 || strcmp(pCmd, "GPGSV") == 0) {
		CNMEAParserData::GSV_DATA_T gsvData;
		
		CNMEAParserData::ERROR_E error;
		std::map<int,GpsSatellite>* satellites;
		if (strcmp(pCmd, "GLGSV") == 0) {
			error = GetGLGSV(gsvData);
			satellites = &state.satellitesGlonass;
		} else {
			error = GetGPGSV(gsvData);
			satellites = &state.satellitesGps;
		}
		
		if(error == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
			//printf("\nCmd: %s\nData: %s\n", pCmd, pData);
			
			std::cout << "GLGSV Parsed! GLGSV is used for GLONASS satellites" << std::endl;
			std::cout << "   Sentence Number:     " << gsvData.nSentenceNumber << std::endl;
			std::cout << "   Number of Sentences: " << gsvData.nTotalNumberOfSentences << std::endl;
			std::cout << "   Satellites in view:  " << gsvData.nSatsInView << std::endl;
			for (int i = 0; i < gsvData.nSatsInView; i++) {
				std::cout << "   - Sat ID:" << gsvData.SatInfo[i].nPRN << " Azimuth:" << gsvData.SatInfo[i].dAzimuth << " Elevation:" << gsvData.SatInfo[i].dElevation << ", SNR: " << gsvData.SatInfo[i].nSNR  << std::endl;
			}
#endif
			

			for (std::map<int,GpsSatellite>::iterator it = satellites->begin(); it != satellites->end(); it++) {
				it->second.visible = false;	// Is this logical?
			}

			state.satelitesInView = gsvData.nSatsInView;
			for (int i = 0; i < 64; i++) {//gsvData.nSatsInView; i++) {
				if (gsvData.SatInfo[i].nPRN == 0) {
					continue;
				}
				(*satellites)[gsvData.SatInfo[i].nPRN].visible = gsvData.SatInfo[i].nSNR > 0;	// Is this logical?
				(*satellites)[gsvData.SatInfo[i].nPRN].azimuth = gsvData.SatInfo[i].dAzimuth;
				(*satellites)[gsvData.SatInfo[i].nPRN].elevation = gsvData.SatInfo[i].dElevation;
				(*satellites)[gsvData.SatInfo[i].nPRN].SNR = gsvData.SatInfo[i].nSNR;
				(*satellites)[gsvData.SatInfo[i].nPRN].ID = gsvData.SatInfo[i].nPRN;
			}

			//newData = true;	// Satellite updates aren't that important
		}
	} else if (strcmp(pCmd, "GNRMC") == 0 || strcmp(pCmd, "GPRMC") == 0) {
		CNMEAParserData::RMC_DATA_T rmcData;
		
		CNMEAParserData::ERROR_E error;
		if (strcmp(pCmd, "GNRMC") == 0) {
			error = GetGNRMC(rmcData);
		} else {
			error = GetGPRMC(rmcData);
		}
		
		if(error == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
			printf("\nCmd: %s\nData: %s\n", pCmd, pData);
			std::cout << "GNRMC Parsed! Time, date, position, course and speed data." << std::endl;
			std::cout << "   Status:              " << (char)rmcData.m_nStatus<< std::endl;
			std::cout << "   UTC:                 " << rmcData.m_timeGGA << std::endl;
			std::cout << "   Date:                " << rmcData.m_nMonth << "/" << rmcData.m_nDay << "/" << rmcData.m_nYear << std::endl;
			std::cout << "   Time:                " << rmcData.m_nHour << ":" << rmcData.m_nMinute << ":" << rmcData.m_dSecond << std::endl;
			//std::cout << "   Altitude:            " << rmcData.m_dAltitudeMSL<< std::endl;  not a part of RMC messages
			std::cout << "   Longitude:           " << rmcData.m_dLongitude<< std::endl;
			std::cout << "   Latitude:            " << rmcData.m_dLatitude<< std::endl;
			std::cout << "   Speed(knots):        " << rmcData.m_dSpeedKnots<< std::endl;
			std::cout << "   Course:              " << rmcData.m_dTrackAngle<< std::endl;
			std::cout << "   Magnetic Variation:  " << rmcData.m_dMagneticVariation<< std::endl;	// only in devices with compass, so maybe always 0 in Panda
#endif

			state.info.status = rmcData.m_nStatus;

			//state.time.time = rmcData.m_timeGGA;	// I don't think m_timeGGA is actually ever set.
			state.time.tm_year = rmcData.m_nYear-1900;
			state.time.tm_mon = rmcData.m_nMonth-1;
			state.time.tm_mday = rmcData.m_nDay;
			state.time.tm_hour = rmcData.m_nHour;
			state.time.tm_min = rmcData.m_nMinute;
			state.time.tm_sec = rmcData.m_dSecond;
			state.timeMilliseconds = rmcData.m_nMilliSecond;

//			state.pose.altitude = rmcData.m_dAltitudeMSL;	// always 0, not a part of RMC messages
			state.pose.longitude = rmcData.m_dLongitude;
			state.pose.latitude = rmcData.m_dLatitude;

			state.motion.speed = rmcData.m_dSpeedKnots;
			state.motion.course = rmcData.m_dTrackAngle;

			state.magneticVariation = rmcData.m_dMagneticVariation;

			newData = true;	// We get time with precision, pose, and speed.  Definitely notification worthy


			// Save to CSV file
			writeCsvToFile(state);
		}
	} else if (strcmp(pCmd, "GNZDA") == 0) {
		CNMEAParserData::ZDA_DATA_T zdaData;
		if(GetGNZDA(zdaData) == CNMEAParserData::ERROR_OK) {
//#ifdef GPS_VERBOSE
//			printf("\nCmd: %s\nData: %s\n", pCmd, pData);
			std::cout << "GNZDA Parsed! Time, date, GMT data." << std::endl;
			std::cout << "   Date:                " << zdaData.m_nMonth << "/" << zdaData.m_nDay << "/" << zdaData.m_nYear << std::endl;
			std::cout << "   Time:                " << zdaData.m_nHour << ":" << zdaData.m_nMinute << ":" << zdaData.m_dSecond << std::endl;
			std::cout << "   GMT hour offset:     " << zdaData.m_nLocalHour << std::endl;
			std::cout << "   GMT minute offset:   " << zdaData.m_nLocalMinute << std::endl;
//#endif

			state.time.tm_year = zdaData.m_nYear-1900;
			state.time.tm_mon = zdaData.m_nMonth-1;
			state.time.tm_mday = zdaData.m_nDay;
			state.time.tm_hour = zdaData.m_nHour;
			state.time.tm_min = zdaData.m_nMinute;
			state.time.tm_sec = zdaData.m_dSecond;
			state.timeMilliseconds = zdaData.m_nMilliSecond;

			//newData = true;	// Just time.  boring
		}
	} else if (strcmp(pCmd, "GNGSA") == 0 || strcmp(pCmd, "GPGSA") == 0) {
		CNMEAParserData::GSA_DATA_T gsaData;
		
		CNMEAParserData::ERROR_E error;
		if (strcmp(pCmd, "GNGSA") == 0) {
			error = GetGNGSA(gsaData);
		} else {
			error = GetGPGSA(gsaData);
		}
		
		if(error == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
			printf("\nCmd: %s Data: %s\n", pCmd, pData);
			std::cout << "GNGSA Parsed! Time, date, position, course and speed data." << std::endl;
			std::cout << "   HDOP:" << gsaData.dHDOP << ", PDOP:" << gsaData.dPDOP << ", VDOP:" << gsaData.dVDOP << std::endl;
			std::cout << "   Auto Mode:           " << (char)gsaData.nAutoMode << std::endl;
			std::cout << "   Fix Type:            " << gsaData.nMode << std::endl;
			std::cout << "   GGACount:            " << gsaData.uGGACount << std::endl;
			std::cout << "   Satellite IDs:       ";
			for (int i = 0; i < 64; i++) {
				std::cout << gsaData.pnPRN[i] << ",";
			}
			std::cout << std::endl;
#endif

			state.quality.HDOP = gsaData.dHDOP;
			state.quality.VDOP = gsaData.dVDOP;
			state.quality.PDOP = gsaData.dPDOP;
			// Note, satelite IDs should be handled by GLGSV
			state.info.automode = gsaData.nAutoMode;
			state.info.fixType = gsaData.nMode;
			state.info.GGAcount = gsaData.uGGACount;

			//newData = true;	// Do we really care about notification on this?
		}
	} else if (strcmp(pCmd, "GNVTG") == 0 || strcmp(pCmd, "GPVTG") == 0) {
		CNMEAParserData::VTG_DATA_T	vtgData;
		
		CNMEAParserData::ERROR_E error;
		if (strcmp(pCmd, "GNVTG") == 0) {
			error = GetGNVTG(vtgData);
		} else {
			error = GetGPVTG(vtgData);
		}
		
		if(error == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
			printf("\nCmd: %s Data: %s\n", pCmd, pData);
			std::cout << "GNVTG Parsed! Track made good and ground speed data." << std::endl;
			std::cout << "   Track True    :" << vtgData.m_trackTrue << std::endl;
			std::cout << "   Track Magnetic:" << vtgData.m_trackMag << std::endl;
			std::cout << "   Speed in Kn   :" << vtgData.m_speedKn << std::endl;
			std::cout << "   Speed in km/h :" << vtgData.m_speedKm << std::endl;
			std::cout << "   Mode          :" << (char)vtgData.m_posModeInd << std::endl;
			

#endif
//			state.motion.course
			// TODO: fill in the appropriate fields
		}
		
	} else if (strcmp(pCmd, "GNTXT") == 0 || strcmp(pCmd, "GPTXT") == 0) {
		CNMEAParserData::TXT_DATA_T	txtData;
		
		CNMEAParserData::ERROR_E error;
		if (strcmp(pCmd, "GNTXT") == 0) {
			error = GetGNTXT(txtData);
		} else {
			error = GetGPTXT(txtData);
		}
		
		if(error == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
			printf("\nCmd: %s Data: %s\n", pCmd, pData);
			std::cout << "GNTXT Parsed! Text transmission" << std::endl;
			std::cout << "   Total Messages in this xmit:" << txtData.m_numMsgs << std::endl;
			std::cout << "   This message number        :" << txtData.m_msgNum << std::endl;
			std::cout << "   Message type               :" << (int)txtData.m_txtIdentifier << std::endl;
			std::cout << "   Message                    :" << txtData.m_txt << std::endl;
#endif
			switch (txtData.m_txtIdentifier) {
				case CNMEAParserData::TXT_IDENTIFIER_ERROR: std::cout << "\u001b[31mGPS ERROR ";
					break;
					
				case CNMEAParserData::TXT_IDENTIFIER_NOTICE: std::cout << "\u001b[34mGPS Notice ";
					break;
					
				case CNMEAParserData::TXT_IDENTIFIER_WARNING: std::cout << "\u001b[33mGPS Warning ";
					break;
					
				case CNMEAParserData::TXT_IDENTIFIER_USER: std::cout << "\u001b[32mGPS User ";
					break;
			}
			std::cout << "[" << txtData.m_msgNum << "/" << txtData.m_numMsgs << "]:";
			std::cout << "\u001b[0m" << txtData.m_txt << std::endl;
			
			// TODO: fill in the appropriate fields
		}
		
    } else if (strcmp(pCmd, "GNGLL") == 0 || strcmp(pCmd, "GPGLL") == 0 ) {
        CNMEAParserData::GLL_DATA_T	gllData;
        
        CNMEAParserData::ERROR_E error;
        if (strcmp(pCmd, "GNGLL") == 0) {
            error = GetGNGLL(gllData);
        } else {
            error = GetGPGLL(gllData);
        }
        
        if(error == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
            printf("\nCmd: %s Data: %s\n", pCmd, pData);
            std::cout << "GNGLL Parsed! Text transmission" << std::endl;
            printf("   Latitude:            %f\n", gllData.m_dLatitude);
            printf("   Longitude:           %f\n", gllData.m_dLongitude);
            printf("   Time:                %02d:%02d:%02d.%03d\n", gllData.m_nHour, gllData.m_nMinute, gllData.m_nSecond, gllData.m_nMilliSecond);
            printf("   Status:              %c\n", (char)gllData.m_nStatus);
            printf("   Mode:                %c\n", (char)gllData.m_posModeInd);
            
#endif
            
            // TODO: fill in the appropriate fields
        }
    } else if (strcmp(pCmd, "GNGST") == 0 ) {
            CNMEAParserData::GST_DATA_T    gstData;
            
            CNMEAParserData::ERROR_E error;
            if (strcmp(pCmd, "GNGST") == 0) {
                error = GetGNGST(gstData);
            }
            
            if(error == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
                printf("\nCmd: %s Data: %s\n", pCmd, pData);
                std::cout << "GNGST Parsed! Text transmission" << std::endl;
                printf("   Latitude:            %f\n", gstData.m_nRMS);
                printf("   Longitude:           %f\n", gstData.m_dLongitude);
                //                printf("   Time:                %02d:%02d:%02d.%03d\n", gllData.m_nHour, gllData.m_nMinute, gllData.m_nSecond, gllData.m_nMilliSecond);   // TODO
                printf("   Status:              %c\n", (char)gllData.m_nStatus);
                printf("   Mode:                %c\n", (char)gllData.m_posModeInd);
                
#endif
                
                // TODO: fill in the appropriate fields
                state.quality.RMS = gstData.m_nRMS;
                
                state.quality.ErrorEllipseMajor = gstData.m_nErrorEllipseMajor;
                state.quality.ErrorEllipseMinor = gstData.m_nErrorEllipseMinor;
                state.quality.ErrorEllipseOrientation = gstData.m_nErrorEllipseOrientation;
                state.quality.LatitudeSigmaError = gstData.m_nLatitudeSigmaError;
                state.quality.LongitudeSigmaError = gstData.m_nLongitudeSigmaError;
                state.quality.AltitudeSigmaError = gstData.m_nAltitudeSigmaError;
            }
        
		
	} else {
//#ifdef GPS_VERBOSE
		printf("\nCmd: %s\nData: %s\n", pCmd, pData);
		std::cout << " - This command is not handled by the parser!" << std::endl;
//#endif
		// No new data in this parse session;
	}

	if (newData) {
		for (std::vector<GpsListener*>:: iterator it = listeners.begin(); it != listeners.end(); it++) {
			GpsListener* listener = *it;
			listener->newDataNotification(&this->state);
		}
	}

	return CNMEAParserData::ERROR_OK;
}


void Gps::gpsSend(const char* data, int length) {
	if (usingExternalGps) {
		write(fidGps, data, length);
	} else if (usbHandler != NULL && usbHandler->hasGpsSupport()){
		usbHandler->uartWrite(data, length);
	}
}

//  An nmea string starting with '$', is checkum delimited with '*', and has placeholder for 2 bytes for the checksum.
//  May want the carriage return/newline as well.
//  i.e: packet[] = "$<Cmd>,<Data1>,..,<DataN>*<2 Byte Checksum>\r\n"
void setNmeaChecksum(char* packet) {
	unsigned char checksum = 0;
	int i;
	for ( i = 1; packet[i] != '*'; i++) {
		checksum ^= packet[i];
	}
	unsigned char checksumNibbleHigh = (checksum >> 4);
	unsigned char checksumNibbleLow = checksum & 0x0F;

	packet[i+1] = checksumNibbleHigh < 0x0A ? checksumNibbleHigh+'0' : checksumNibbleHigh+'A';
	packet[i+2] = checksumNibbleLow < 0x0A ? checksumNibbleLow+'0' : checksumNibbleLow+'A';
};

// Generates and places a UBX checksum based on the provided packet length
// For example,l a packet with length 6 (12 bytes) could be formatted with checksum holders denoted as 0xFF:
// packet[] = "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\xFF\xFF";
#define UBX_LENGTH_INDEX (4)
void Panda::setUbxChecksum(char* packet) {
	unsigned int checksumA = 0, checksumB = 0;
	int endIndexOfPacket = 6 + ((int)packet[UBX_LENGTH_INDEX]) + (((int)packet[UBX_LENGTH_INDEX+1]) << 8);
	for (int i = 2; i < endIndexOfPacket; i++) {
		checksumA += ((unsigned char*)packet)[i];
		checksumB += checksumA;
	}
	packet[endIndexOfPacket] = checksumA & 0xFF;
	packet[endIndexOfPacket+1] = checksumB & 0xFF;
}

int Panda::makeUbx(char* dst, char mClass, char mId, unsigned short payloadLength, char* payload) {
	dst[0] = 0xB5;
	dst[1] = 0x62;
	dst[2] = mClass;
	dst[3] = mId;
//	*(unsigned short*)&dst[4] = payloadLength;
	dst[4] = payloadLength & 0xFF;
	dst[5] = (payloadLength >> 8) && 0xFF;
	memcpy(&dst[6], payload, payloadLength);
	setUbxChecksum(dst);
	return payloadLength+8;
}

unsigned short Panda::getUbxLength(const char* ubx) {
	return (unsigned short)ubx[4] + (((unsigned short)ubx[5]) << 8 ) + 8;
}

void Gps::initialize() {
	std::cerr << "Initializing GPS" << std::endl;

	if (usbHandler == NULL) {
		std::cerr << "ERROR: Usb::initialize(): No Usb Handler set for Panda::Usb!" << std::endl;
		return;
	}

	if ( !usbHandler->hasGpsSupport() ) {
		std::cerr << "WARNING: Gps::initialize(): Panda does not support GPS!" << std::endl;
		std::cerr << "       : Gps::initialize(): Attempting to open /dev/ttyACM0 instead" << std::endl;
		
		struct termios newtio;
		if ((fidGps = open("/dev/ttyACM0", O_RDWR | O_NOCTTY )) < 0) {
			// std::cerr << "device open error: " << field << std::endl;
			
			std::cerr << "       : Gps::initialize(): /dev/ttyACM0  Does not exist" << std::endl;
			return;
		}
		
		usingExternalGps = true;
		
		char ubxCfgRst[256], ubxCfgRstPayload[4];
		ubxCfgRstPayload[0] = 0x00;	// hot start
		ubxCfgRstPayload[1] = 0x00; // hot start
		ubxCfgRstPayload[2] = 0x01; // Hardware Reset 0x00, Controlled Software Reset 0x01
		ubxCfgRstPayload[3] = 0x00;	// reserved1

		makeUbx(ubxCfgRst, UBX_CLASS_CFG, UBX_ID_CFG_RST, 4, ubxCfgRstPayload);
		gpsSend(ubxCfgRst, getUbxLength(ubxCfgRst));
		usleep(1000);
		close(fidGps);
		
		int reOpenCount = 0;
		while (reOpenCount++ < 10) {
			usleep(500000);
			if ((fidGps = open("/dev/ttyACM0", O_RDWR | O_NOCTTY )) < 0) {
				// std::cerr << "device open error: " << field << std::endl;

				std::cerr << "       : Gps::initialize(): Waiting for /dev/ttyACM0" << std::endl;
				continue;
			}
			std::cerr << "       : Gps::initialize(): Success!" << std::endl;
			break;
		}
		
		/*---------- Setting the Attributes of the serial port using termios structure --------- */

		   struct termios SerialPortSettings;  /* Create the structure                          */

		   tcgetattr(fidGps, &SerialPortSettings); /* Get the current attributes of the Serial port */

		   /* Setting the Baud rate */
		   cfsetispeed(&SerialPortSettings,B19200); /* Set Read  Speed as 19200                       */
		   cfsetospeed(&SerialPortSettings,B19200); /* Set Write Speed as 19200                       */
	
		   /* 8N1 Mode */
		   SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
		   SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
		   SerialPortSettings.c_cflag &= ~CSIZE;    /* Clears the mask for setting the data size             */
		   SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

		   SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
		   SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


		   SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
		   SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
		SerialPortSettings.c_lflag = 0;

		   SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

		   /* Setting Time outs */
		   SerialPortSettings.c_cc[VMIN] = 13; /* Read at least 10 characters */
		   SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


		   if((tcsetattr(fidGps,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
			   printf("\n  ERROR ! in Setting attributes");
		
		
//		char cfgrate[] = "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\xFF\xFF";
//		setUbxChecksum(cfgrate);
////		usbHandler->uartWrite(cfgrate, sizeof(cfgrate));
//		write(fidGps, cfgrate, sizeof(cfgrate));
//		usleep(100000);
//		write(fidGps, "\xB5\x62\x06\x24\x24\x00\x05\x00\x04\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x5A\x63", 44);
//		usleep(100000);
//		write(fidGps, "\xB5\x62\x06\x1E\x14\x00\x00\x00\x00\x00\x01\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3C\x37", 28);
		
//		return;
		std::cerr << " - Upping baudrate" << std::endl;
		// set baudrate:
		char nmeaString[100];
		sprintf(nmeaString, "$PUBX,41,1,0007,0003,%d,0*FF\r\n", GPS_BAUD);
		setNmeaChecksum(nmeaString);
//		usbHandler->uartWrite(nmeaString, strlen(nmeaString));
		gpsSend(nmeaString, strlen(nmeaString));
		
		
		
		
	} else {
		std::cerr << " - Resetting" << std::endl;
		
		usbHandler->setEspPower(0);
		usleep(100000);
		
		usbHandler->uartPurge();
		
		usbHandler->setUartBaud(UART_DEVICE_GPS, INIT_GPS_BAUD);
		
		usbHandler->setEspPower(1);
		usleep(500000);
		
		//	std::cerr << " - Enabling ZDA messages" << std::endl;
		char nmeaString[100];
		//
		//	// Bunting: the following was added by me after failing to accomplish the same using UBX commands further down.
		//	sprintf(nmeaString, "$PUBX,40,ZDA,0,1,0,0,0,0*FF\r\n");
		//	setNmeaChecksum(nmeaString);
		//	usbHandler->uartWrite(nmeaString, strlen(nmeaString));
		
		std::cerr << " - Upping baudrate" << std::endl;
		// set baudrate:
		sprintf(nmeaString, "$PUBX,41,1,0007,0003,%d,0*FF\r\n", GPS_BAUD);
		setNmeaChecksum(nmeaString);
//		usbHandler->uartWrite(nmeaString, strlen(nmeaString));
		gpsSend(nmeaString, strlen(nmeaString));
		
		usleep(100000);
		//	usbHandler->uartPurge();
		
		std::cerr << " - Reconnecting with new baudrate" << std::endl;
		usbHandler->setUartBaud(UART_DEVICE_GPS, GPS_BAUD);
		usleep(100000);
	}
	
	std::cerr << " - Sending config" << std::endl;

////	 Bunting: the following was added by me after failing to accomplish the same using UBX commands further down.
//	char nmeaString[] =  "$PUBX,40,ZDA,0,1,0,0,0,0*FF\r\n";
//	setNmeaChecksum(nmeaString);
//	usbHandler->uartWrite(nmeaString, strlen(nmeaString));
//	usleep(100000);
	
	// The following is copied from panda python code.  These are hardcoded UBX messages that they claim are generated from test/ubloxd.py
	//  The structure according to the u-blox-M8 manual, under Interface description->UBX protocl is summarized:
	// <0xB5> <0x62> <1B message Class> <1B ID> <2B length> <payload> <2B UBX checksum>
	//  - all message classes ar 0x06: CFG, Configuration input messages
	// Message class/ID pairs are then as follows according to the manual:
	// 0x06 0x00: CFG-PRT:   port configuration for IO, UART, SPI, USB.  We are using UART in the Panda
	// 0x06 0x3E: CFG-GNSS:  GNSS system configuration
	// 0x06 0x08: CFG-RATE:  Navigation/Measurement Rate Settings
	// 0x06 0x1E: CFG-ODO:   Odometer, Low-speed COG Engine...
	// 0x06 0x24: CFG-NAV5:  Navigation Engine Settings
	// 0x06 0x23: CFG-NAVX5:  Navigation Engine Expert Settings
	// 0x06 0x01: CFG-MSG:   Poll a message configuration/ Set MEssage rates
	// The GPS responds with protocol version 20.3, which is covered by the manual

	// CFG-PRT: length is 20 (0x14) implying a Uart configuration.
	//	serialWrite("\xB5\x62\x06\x00\x14\x00\x03\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x01\x00\x00\x00\x00\x00\x1E\x7F", 28);
	// 0x06 0x3E: CFG-GNSS:  hmm this payload is 0 bytes but there is no example of this in the found manual.  minimum should be a length of 4
	//	serialWrite("\xB5\x62\x06\x3E\x00\x00\x44\xD2", 8);
	// CFG-PRT: length is 20 (0x14) implying a Uart configuration.  again it's working, so I'll skip
	// all of these 0s suggest port with ID 0 will be disabled
	//	serialWrite("\xB5\x62\x06\x00\x14\x00\x00\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x19\x35", 28);
	// CFG-PRT: length is 20 (0x14) implying a Uart configuration.  again it's working, so I'll skip
	//  Byte 1: ID: 1
	//  Byte 2: reserved
	//  Bytes 2-3:  txReady is set to all 0;
	//  Bytes 4-7:  mode is set to 0x000008C0, meaning an 8N1 uart config
	//  Bytes 8-11: baud: is the baudrate in bytes 8,9,10,11, set as 0x00070800 = 406800bps
	//  Bytes 12-13: inProtoMask: set for UBX input
	//  Bytes 14-15: outProtoMask: set for UBX output
	//  Bytes 16-17: flags: timeout not enabled
	//  Bytes 18-19: reserved
	//	serialWrite("\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xC0\x08\x00\x00\x00\x08\x07\x00\x01\x00\x01\x00\x00\x00\x00\x00\xF4\x80", 28);
	//	// The immediate above converted to my format:
	//	char cfgprt1[] = "\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xC0\x08\x00\x00\x00\x08\x07\x00\x01\x00\x01\x00\x00\x00\x00\x00\xF4\x80";
	//	setUbxChecksum(cfgprt1);
	//	serialWrite(cfgprt1, sizeof(cfgprt1));
	//	// Again the imeediate above, but set to NMEA output instead which should be correct
	//	char cfgprt2[] = "\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xC0\x08\x00\x00\x00\x08\x07\x00\x01\x00\x02\x00\x00\x00\x00\x00\xF4\x80";
	//	setUbxChecksum(cfgprt2);
	//	uartWrite(cfgprt2, sizeof(cfgprt2));

	// CFG-PRT: length is 20 (0x14) implying a Uart configuration.  again it's working, so I'll skip
	// all of these 0s suggest port with ID 4 will be disabled
	//	uartWrite("\xB5\x62\x06\x00\x14\x00\x04\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x1D\x85", 28);
	// CFG-PRT: length is 0, which is not shown in the manual.  it's either supposed to be 1 or 20 bytes
	//	uartWrite("\xB5\x62\x06\x00\x00\x00\x06\x18", 8);
	// CFG-PRT: length is 1, which polls the I/O port configuration, but where does it go?  Port ID of this read is 0x01.
	//	uartWrite("\xB5\x62\x06\x00\x01\x00\x01\x08\x22", 9);
	// CFG-PRT: length is 1, which polls the I/O port configuration, but where does it go?  Port ID of this read is 0x02.
	//	uartWrite("\xB5\x62\x06\x00\x01\x00\x02\x09\x23", 9);
	// CFG-PRT: length is 1, which polls the I/O port configuration, but where does it go?  Port ID of this read is 0x03.
	//	uartWrite("\xB5\x62\x06\x00\x01\x00\x03\x0A\x24", 9);



	// CFG-RATE: length is 6. Sets the rate that the reciever send back messages.
	// Bytes 0-1, measRate, unit ms: 0x0064 = 100, meaning 100ms, or 10Hz read rate. "The elapsed time between GNSS measurements"
	// Bytes 2-3, navRate, unit cycles:0x0001 = 1.  This could be ignored based on firmware. "The ratio between the number of measurements and the number of navigation solutions, e.g. 5 means five measurements for every navigation solution. Maximum value is 127. (This parameter is ignored and the navRate is fixed to 1 inprotocol versions less than 18)"
	// Bytes 4-5, timeRef: 0x0000.  Set for UTC. "The time system to which measurements are aligned:
	//													0: UTC time
	//													1: GPS time
	//													2: GLONASS time (not supported in protocol versions less than 18)
	//													3: BeiDou time (not supported in protocol versions less than 18)
	//													4: Galileo time (not supported in protocol versions less than 18)"

	//	char cfgrateorig[] = "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x00\x00\x79\x10";
	//	char ck_a=0, ck_b=0;
	//	for (int i = 2; i < 12; i++) {
	//		ck_a += cfgrateorig[i];
	//		ck_b += ck_a;
	//	}
	//	std::cout << "-------- ck_a = " << (int)ck_a << "  ck_b = " << (int)ck_b << std::endl;
/*
	//	uartWrite("\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x00\x00\x79\x10", 14);
	//char cfgrate[] = "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x00\x00\xFF\xFF";
	// tyring GPS time instead of UTC time:
	std::cout << " - Upping report rate to 10Hz..." << std::endl;
	char cfgrate[] = "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\xFF\xFF";
	setUbxChecksum(cfgrate);
//	usbHandler->uartWrite(cfgrate, sizeof(cfgrate));
	gpsSend(cfgrate, sizeof(cfgrate));

	usleep(100000);
	// CFG-NAV5: length is 36. lots of GPS configured things in this one.  Seems well formed
	// Everything is 0 (default?) except:
	// Byte 0-1: mask: 0x05 bits 0 and 2 high.  bit 0: apply dynamic model settings.  bit 2: Apply fix mode settings
	// Byte 2: dynModel: automotive "Dynamic platform model"
	// Byte 3:   fixmode: Auto 2d/3d "Position Fixing Mode"
//	usbHandler->uartWrite("\xB5\x62\x06\x24\x24\x00\x05\x00\x04\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x5A\x63", 44);
	std::cout << " - Setting to dyanmic model..." << std::endl;
	char cfgnav[] = "\xB5\x62\x06\x24\x24\x00\x05\x00\x04\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x5A\x63";
	setUbxChecksum(cfgnav);
	gpsSend(cfgnav, sizeof(cfgnav));
	usleep(100000);
	// CFG-ODO: length is 20.  This sets up stuff involving a low-speed COG engine (?) filter.
	//    for filtering course and speed?
	//		Byte 4: Set of useODO (opposed to useCOG), I think meaning course wont be computed for speeds less than 8m/s
	//		Byte 5: set to be in automotive mode
	std::cout << " - Setting to automotive mode..." << std::endl;
	char cfgOdo[] = "\xB5\x62\x06\x1E\x14\x00\x00\x00\x00\x00\x01\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3C\x37";
	setUbxChecksum(cfgOdo);
	gpsSend(cfgOdo, sizeof(cfgOdo));
	usleep(100000);
	// CFG-NAV5: length is 0.  Again I don't see this in the manual
	//	uartWrite("\xB5\x62\x06\x24\x00\x00\x2A\x84", 8);
	// CFG-NAV5X: length is 0.  Again I don't see this in the manual
	//	uartWrite("\xB5\x62\x06\x23\x00\x00\x29\x81", 8);
	// CFG-ODO: length is 0.  Again I don't see this in the manual
	//	uartWrite("\xB5\x62\x06\x1E\x00\x00\x24\x72", 8);
	// CFG-MSG: length is 3.
	//   These are well formed, but the data goes outside of the manual set of message IDs.
	//	uartWrite("\xB5\x62\x06\x01\x03\x00\x01\x07\x01\x13\x51", 11);
	//	uartWrite("\xB5\x62\x06\x01\x03\x00\x02\x15\x01\x22\x70", 11);
	//	uartWrite("\xB5\x62\x06\x01\x03\x00\x02\x13\x01\x20\x6C", 11);
	//	uartWrite("\xB5\x62\x06\x01\x03\x00\x0A\x09\x01\x1E\x70", 11);



	/// Beyon Panda configuration:
	// I want to enable ZDA for local timezone
	// We use the CFG-MSG command for this: 0x06 0x01
	// ZDA messages have class ID 0xF0 0x08
	// enabling... in the 6-byte bitmask is unclear.
	//    and example has a a single 0x01, as follows:
	// char cfgMsgZda[] = "0xB5\x62\x06\x01\x08\x00\xF0\x08\x00\x01\x00\x00\x00\x01\xFF\xFF";
	// setUbxChecksum(cfgMsgZda);
	// uartWrite(cfgMsgZda, sizeof(cfgMsgZda));
	//
	//	// the 3-byte version:
	//	char cfgMsgZda[] = "0xB5\x62\x06\x01\x03\x00\xF0\x08\x01\xFF\xFF";
	//	setUbxChecksum(cfgMsgZda);
	//	uartWrite(cfgMsgZda, sizeof(cfgMsgZda));
	//	//  well both of the above failed.  I switched to PUBX command at the very top of this function and it enables ZDA


	
	//	char cfgrate[] = "\xB5\x62\x06\x00\x01\x00\x01\xFF\xFF";
	
	char ubx[256];
	char payload[256];
	payload[0] = 0x01;
	makeUbx(ubx, 0x06, 0x00, 1, payload);
	ubxCommands.push(std::string( "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\xFF\xFF", getUbxLength("\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\xFF\xFF")));
	ubxCommands.push(std::string( ubx, getUbxLength(ubx)));
	payload[0] = 0;
	makeUbx(ubx, 0x06, 0x00, 1, payload);
	ubxCommands.push(std::string( ubx, getUbxLength(ubx)));
	
	
	*/
	
	
	char cfgratePayload[] = "\x64\x00\x01\x00\x01\x00";
	sendUbxCommand(UBX_CLASS_CFG, UBX_ID_CFG_RATE, 6, cfgratePayload);
	
	sendUbxCommand(UBX_CLASS_CFG, UBX_ID_CFG_RATE, 0, NULL); // Poll the same message
	
	char cfgNavPayload[] = "\x05\x00\x04\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";
	sendUbxCommand(UBX_CLASS_CFG, UBX_ID_CFG_NAV5, 0x24, cfgNavPayload);
	
	
	char cfgOdoPayload[] = "\x00\x00\x00\x00\x01\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";
	sendUbxCommand(UBX_CLASS_CFG, UBX_ID_CFG_ODO, 0x14, cfgOdoPayload);
    
    char cfgMsgGstReadPayload[] = "\xf0\x07";
    sendUbxCommand(UBX_CLASS_CFG, UBX_ID_CFG_MSG, 2, cfgMsgGstReadPayload);
    
    char cfgMsgGstPayload[] = "\xf0\x07\x0a"; //0xF0 0x07
    sendUbxCommand(UBX_CLASS_CFG, UBX_ID_CFG_MSG, 3, cfgMsgGstPayload);
    
	
	std::cerr << " - GPS Done." << std::endl;
}


void Gps::sendUbxCommand(char mClass, char mId, unsigned short payloadLength, char* payload) {
	char ubx[1024];
	makeUbx(ubx, mClass, mId, payloadLength, payload);
	ubxCommands.push(std::string(ubx, getUbxLength(ubx)));
	if (gpsConfigState == GPS_CONFIG_IDLE) {
		gpsConfigState = GPS_CONFIG_START;
	}
	
}

std::string Panda::ubxClassIdToString( char mClass, char mId ) {
	switch (mClass) {
		case UBX_CLASS_CFG:
			switch (mId) {
				case UBX_ID_CFG_PRT:  return "CFG-PRT";
				case UBX_ID_CFG_ODO:  return "CFG-ODO";
                case UBX_ID_CFG_MSG:  return "CFG-MSG";
				case UBX_ID_CFG_RST:  return "CFG-RST";
				case UBX_ID_CFG_RATE: return "CFG-RATE";
				case UBX_ID_CFG_NAV5: return "CFG-NAV5";
					
				default:
					break;
			}
			break;
			
		case UBX_CLASS_ACK:
			switch (mId) {
				case UBX_ID_ACK_ACK:  return "ACK-ACK";
				case UBX_ID_ACK_NACK: return "ACK-NACK";
					break;
					
				default:
					break;
			}
			break;
			
		case UBX_CLASS_NAV:
			switch (mId) {
				default:
					break;
			}
			break;
			
		case UBX_CLASS_MON:
			switch (mId) {
				case UBX_ID_MON_VER:  return "MON-VER";
					
				default:
					break;
			}
			break;
			
		default:
			break;
	}
	char result[256];
	sprintf(result, "0x%02X 0x%02X", mClass, mId);
	
	return result;
}
