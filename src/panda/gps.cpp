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

//#define GPS_VERBOSE

using namespace Panda;

Gps::Gps()
:csvDump(NULL) {
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

void Gps::notificationUartRead(char* buffer, size_t bufferLength) {
	currentlyReceiving = bufferLength > 0;

	resume();	// will request more data

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

void Gps::doAction() {
	if (usbHandler == NULL) {
		std::cerr << "ERROR: Gps::doAction(): No Usb Handler set for Panda::Gps!" << std::endl;
		usleep(1000);
		return;
	}

	if (!currentlyReceiving) {
		usleep(10000);	// thread does not need to be particularly fast
	}

	// Tell this thread to pause, wait for request to complete
	pause();	// yes, before requesting data, otherwise synchronous USB deadlocks

	usbHandler->requestUartData();
//	std::cout << "Requeting new data" << std::endl;
}

bool Gps::isReady() {
	return state.info.status == 'A';
}

void Gps::addObserver( GpsListener* listener ) {
	listeners.push_back(listener);
}

void Gps::startParsing() {
	if(usbHandler == NULL) {
		std::cerr << "ERROR: Gps::startParsing(): No Usb Handler set for Panda::Gps!" << std::endl;
		return;
	}

	if ( !usbHandler->hasGpsSupport() ) {
		std::cerr << "WARNING: Gps::startParsing(): Usb Device does not support GPS!" << std::endl;
		return;
	}

	start();
}

void Gps::stopParsing() {
	stop();
	WaitForInternalThreadToExit();
}

void Gps::saveToFile( const char* filename ) {
	nmeaDump.open(filename);
}


// GPS overloads:
void Gps::OnError(CNMEAParserData::ERROR_E nError, char *pCmd) {
	printf("ERROR for Cmd: %s, Number: %d\n", pCmd, nError);
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
	if (strcmp(pCmd, "GNGGA") == 0) {
		CNMEAParserData::GGA_DATA_T ggaData;
		if(GetGNGGA(ggaData) == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
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
	} else if (strcmp(pCmd, "GLGSV") == 0) {
		CNMEAParserData::GSV_DATA_T gsvData;
		if(GetGLGSV(gsvData) == CNMEAParserData::ERROR_OK) {
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

			for (std::map<int,GpsSatellite>::iterator it = state.satellites.begin(); it != state.satellites.end(); it++) {
				it->second.visible = false;	// Is this logical?
			}

			state.satelitesInView = gsvData.nSatsInView;
			for (int i = 0; i < 64; i++) {//gsvData.nSatsInView; i++) {
				if (gsvData.SatInfo[i].nPRN == 0) {
					continue;
				}
				state.satellites[gsvData.SatInfo[i].nPRN].visible = gsvData.SatInfo[i].nSNR > 0;	// Is this logical?
				state.satellites[gsvData.SatInfo[i].nPRN].azimuth = gsvData.SatInfo[i].dAzimuth;
				state.satellites[gsvData.SatInfo[i].nPRN].elevation = gsvData.SatInfo[i].dElevation;
				state.satellites[gsvData.SatInfo[i].nPRN].SNR = gsvData.SatInfo[i].nSNR;
				state.satellites[gsvData.SatInfo[i].nPRN].ID = gsvData.SatInfo[i].nPRN;
			}

			//newData = true;	// Satellite updates aren't that important
		}
	} else if (strcmp(pCmd, "GNRMC") == 0) {
		CNMEAParserData::RMC_DATA_T rmcData;
		if(GetGNRMC(rmcData) == CNMEAParserData::ERROR_OK) {
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
#ifdef GPS_VERBOSE
//			printf("\nCmd: %s\nData: %s\n", pCmd, pData);
			std::cout << "GNZDA Parsed! Time, date, GMT data." << std::endl;
			std::cout << "   Date:                " << zdaData.m_nMonth << "/" << zdaData.m_nDay << "/" << zdaData.m_nYear << std::endl;
			std::cout << "   Time:                " << zdaData.m_nHour << ":" << zdaData.m_nMinute << ":" << zdaData.m_dSecond << std::endl;
			std::cout << "   GMT hour offset:     " << zdaData.m_nLocalHour << std::endl;
			std::cout << "   GMT minute offset:   " << zdaData.m_nLocalMinute << std::endl;
#endif

			state.time.tm_year = zdaData.m_nYear-1900;
			state.time.tm_mon = zdaData.m_nMonth-1;
			state.time.tm_mday = zdaData.m_nDay;
			state.time.tm_hour = zdaData.m_nHour;
			state.time.tm_min = zdaData.m_nMinute;
			state.time.tm_sec = zdaData.m_dSecond;
			state.timeMilliseconds = zdaData.m_nMilliSecond;

			//newData = true;	// Just time.  boring
		}
	} else if (strcmp(pCmd, "GNGSA") == 0) {
		CNMEAParserData::GSA_DATA_T gsaData;
		if(GetGNGSA(gsaData) == CNMEAParserData::ERROR_OK) {
#ifdef GPS_VERBOSE
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
	} else {
#ifdef GPS_VERBOSE
//		printf("\nCmd: %s\nData: %s\n", pCmd, pData);
		std::cout << " - This command is not handled by the parser!" << std::endl;
#endif
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
void setUbxChecksum(char* packet) {
	unsigned char checksumA = 0, checksumB = 0;
	int endIndexOfPacket = 6 + ((int)packet[UBX_LENGTH_INDEX]) + (((int)packet[UBX_LENGTH_INDEX+1]) << 8);
	for (int i = 2; i < endIndexOfPacket; i++) {
		checksumA += packet[i];
		checksumB += checksumA;
	}
	packet[endIndexOfPacket] = checksumA;
	packet[endIndexOfPacket+1] = checksumB;
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
		return;
	}

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
	usbHandler->uartWrite(nmeaString, strlen(nmeaString));

	usleep(100000);
//	usbHandler->uartPurge();

	std::cerr << " - Reconnecting with new baudrate" << std::endl;
	usbHandler->setUartBaud(UART_DEVICE_GPS, GPS_BAUD);
	usleep(100000);

	
	std::cerr << " - Sending config" << std::endl;

	// The following is copied from panda python code.  These are hardcoded UBX messages that they calim are generated from test/ubloxd.py
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

	//	uartWrite("\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x00\x00\x79\x10", 14);
	//char cfgrate[] = "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x00\x00\xFF\xFF";
	// tyring GPS time instead of UTC time:
	char cfgrate[] = "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\xFF\xFF";
	setUbxChecksum(cfgrate);
	usbHandler->uartWrite(cfgrate, sizeof(cfgrate));



	// CFG-NAV5: length is 36. lots of GPS configured things in this one.  Seems well formed
	// Everything is 0 (default?) except:
	// Byte 0-1: mask: 0x05 bits 0 and 2 high.  bit 0: apply dynamic model settings.  bit 2: Apply fix mode settings
	// Byte 2: dynModel: automotive "Dynamic platform model"
	// Byte 3:   fixmode: Auto 2d/3d "Position Fixing Mode"
	usbHandler->uartWrite("\xB5\x62\x06\x24\x24\x00\x05\x00\x04\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x5A\x63", 44);
	// CFG-ODO: length is 20.  This sets up stuff involving a low-speed COG engine (?) filter.
	//    for filtering course and speed?
	//		Byte 4: Set of useODO (opposed to useCOG), I think meaning course wont be computed for speeds less than 8m/s
	//		Byte 5: set to be in automotive mode
	usbHandler->uartWrite("\xB5\x62\x06\x1E\x14\x00\x00\x00\x00\x00\x01\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3C\x37", 28);
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
	//	char cfgMsgZda[] = "0xB5\x62\x06\x01\x08\x00\xF0\x08\x00\x01\x00\x00\x00\x01\xFF\xFF";
	//	setUbxChecksum(cfgMsgZda);
	//	uartWrite(cfgMsgZda, sizeof(cfgMsgZda));
	//
	//	// the 3-byte version:
	//	char cfgMsgZda[] = "0xB5\x62\x06\x01\x03\x00\xF0\x08\x01\xFF\xFF";
	//	setUbxChecksum(cfgMsgZda);
	//	uartWrite(cfgMsgZda, sizeof(cfgMsgZda));
	//	//  well both of the above failed.  I switched to PUBX command at the very top of this function and it enables ZDA



	std::cerr << " - GPS Done." << std::endl;
}
