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

#ifndef PANDA_GPS_H
#define PANDA_GPS_H

#include <vector>
#include <map>
#include <fstream>
#include <queue>
//#include <array>

#include "NMEAParser.h"
#include "mogi/thread.h"

#include "panda/gps.h"
#include "panda/usb.h"
#include "panda/gpsdata.h"

namespace Panda {

#define UBX_NAV_COV_POS_NN (16)
#define UBX_NAV_COV_POS_NE (20)
#define UBX_NAV_COV_POS_ND (24)
#define UBX_NAV_COV_POS_EE (28)
#define UBX_NAV_COV_POS_ED (32)
#define UBX_NAV_COV_POS_DD (36)

enum GPS_CONFIG_STATE {
	GPS_CONFIG_IDLE,
	GPS_CONFIG_START,
	GPS_CONFIG_SEND,
	GPS_CONFIG_WAIT,
//    GPS_CONFIG_BAD_CHECKSUM,
	GPS_CONFIG_ACK,
	GPS_CONFIG_NACK,
	GPS_CONFIG_SUCCESS,
	GPS_CONFIG_FAIL,
	GPS_CONFIG_COMPLETE
};

enum UBX_CLASS {
	UBX_CLASS_NAV = 0x01,
	UBX_CLASS_ACK = 0x05,
	UBX_CLASS_CFG = 0x06,
	UBX_CLASS_MON = 0x0A
};

enum UBX_ID_ACK {
	UBX_ID_ACK_ACK  = 0x01,
	UBX_ID_ACK_NACK = 0x00
};

enum UBX_ID_CFG {
	UBX_ID_CFG_PRT  = 0x00,
    UBX_ID_CFG_MSG  = 0x01,
	UBX_ID_CFG_RST  = 0x04,
	UBX_ID_CFG_RATE = 0x08,
	UBX_ID_CFG_ODO  = 0x1e,
	UBX_ID_CFG_NAV5 = 0x24
};

enum UBX_ID_MON {
	UBX_ID_MON_VER  = 0x04
};

enum UBX_ID_NAV {
    UBX_ID_NAV_COV  = 0x36
};

std::string ubxClassIdToString( char mClass, char mId);
	
	void setUbxChecksum(char* packet);
	int makeUbx(char* dst, char mClass, char mId, unsigned short payloadLength, char* payload);
    float parseFloat(char* payloadLocation);
	unsigned short getUbxLength(const char* ubx);

	class Gps;

	/*!
	 @class GpsListener
	 \brief An abstract class for new data notifications for new GPS data.

	 This is the intended methodology for reading GPS data.
	 */
	class GpsListener {
	private:
		friend class Gps;
	protected:
		/*!
		 \brief Called on a successful RMC message parse
		 Overload this to get instant updates form freshly parsed NMEA strings. Be efficient in this
		 method!  This will block further GPS reads and parsings.
		 @param gpsData The up-to-date GPS data.
		 */
		virtual void newDataNotification(GpsData* gpsData) = 0;
        
        
        /*!
         \brief Called on a successful VTG message parse
         Overload this to get instant updates form freshly parsed NMEA strings. Be efficient in this
         method!  This will block further GPS reads and parsings.
         @param gpsData The up-to-date GPS data.
         */
        virtual void newHeadingNotification(GpsData* gpsData) {};

	public:
		virtual ~GpsListener() { };
		
	};

	/*! \class Gps
	 \brief A class that handles the GPS data

	 This class interfaces with a Panda::Usb class to handle the polling of the Grey Panda's
	 internal UART interface for the u-blox M8 GPS module communication.

	 */
	class Gps : protected Mogi::Thread, public UsbListener, public CNMEAParser {
	public:
		Gps();
		~Gps();

		void initialize();	// needs USB device

		/*! \brief Gets the latest GPS data state.
		 This does not invoke a parse, the parser automatically populates this data
		 \return The current GPS data.
		 */
		const GpsData& getData() const;

		/*! \brief Saves raw NMEA data to a file.
		 More correctly, this saves data read by the UART via the Panda USB interface to file.
		 \param filename The filename and path for data to be saved.
		 */
		void saveToFile(const char* filename);

		/*! \brief Saves parsed GPS data to a file in CSV format.
		 This is done after parsing raw NMEA strings.
		 \param filename The filename and path for data to be saved.
		 */
		void saveToCsvFile(const char* filename);

		/*! \brief Sets the USB handler for GPS UART communication
		 \param usbHandler The Panda USB handler.
		 */
		void setUsb( Panda::Usb* usbHandler );

		/*! \brief Adds and Observer for updates on successful parsed NMEA strings
		 \param listener The observer for fresh GPS data.
		 */
		void addObserver( GpsListener* listener );

		/*! \brief Checks if GPS data is valid, from a GLL status parse.
		 \return true is ready, flase if not ready.
		 */
		bool isReady();
		
		/*! \brief Checks if a gps device is available, whether in panda or through external USB
		 \return true if one is conencted, flase otherwise.
		 */
		bool available();

		/*! \brief Starts a reading and parsing thread.
		 A valid and initialized USB handler is needed to be set for success.
		 */
		void startParsing();

		/*! \brief Stops the reading and parsing thread.
		 Called after startParsing() when done with GPS data
		 */
		void stopParsing();
		
		// Send a string to the GPS module's raw serial port
		void gpsSend(const char* data, int length);
		
		// Sends a command and checks that it's received
		void sendUbxCommand(char mClass, char mId, unsigned short payloadLength, char* payload);
		
		// Returns true is UBX commands are eing processed
		bool busyUbx();
		
		// Returns the result of the latest successul UBX read
		int getUbxResponse(char* result);
		
	private:
		GpsData state;
		int fidGps; // This is for external GPS devices
		bool usingExternalGps = false;

		bool currentlyReceiving = false;

		std::ofstream nmeaDump;
		FILE* csvDump;

		void writeCsvToFile(GpsData& state);

		std::vector<GpsListener*> listeners;
		Usb* usbHandler = NULL;

		// Overload from Mogi::Thread
		void doAction();

		// Overload frum UsbListener
		void notificationUartRead(char* buffer, size_t bufferLength);
		
		void processUart(char* buffer, int bufferLength);

		// Called when a UBX is received
		int parseUbx(unsigned char* buffer, int length);
		void notificationUbxMessage(char mClass, char mId, short length, unsigned char* payload);
		
		char ubxLastSentClass;
		char ubxLastSentId;
		int responseLength;
		char responsePayload[256];	// this could all be a struct...
		
		GPS_CONFIG_STATE gpsConfigState;
		int configurationWaitCounter = 0;
		int ubxSendAttempt = 0;
		void handleConfiguration();
		
		std::queue<std::string> ubxCommands;
		std::string ubxCurrentCommand;
		
		// NMEAParser overload:
		virtual void OnError(CNMEAParserData::ERROR_E nError, char *pCmd);
		// NMEAPArser overload:
		virtual CNMEAParserData::ERROR_E ProcessRxCommand(char *pCmd, char *pData);
		
		
	};

}

#endif
