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

#ifndef PANDA_CAN_H
#define PANDA_CAN_H

#include <fstream>
#include <list>

#include "mogi/thread.h"

#include "panda/usb.h"
#include "panda/candata.h"

namespace Panda {

	/*!
	 \brief Converts a buffer from Panda to CanFrame data
	 \param buffer The buffer from the Panda read.
	 \param bufferLength The number of bytes in the buffer.  Should be 8-16
	 \return A constructed CanFrame, populated witht eh bufffer data
	 */
	CanFrame bufferToCanFrame( char* buffer, int bufferLength);

	/*!
	 \brief Converts a CanFrame into a buffer for sending to the Panda
	 \param frame The CAN frame of data to be sent
	 \param buffer The array of data to be sent.  Should be 16 bytes in length.
	 */
	void canFrameToBuffer( CanFrame& frame, unsigned char* buffer);

	class Can;

	/*!
	 @class CanListener
	 \brief An abstract class for new data notifications for new CAN data.

	 This is the intended methodology for reading CAN data.
	 */
	class CanListener {
	private:
		friend class Can;
	protected:
		/*!
		 \brief Called on a successful RMC message parse
		 Overload this to get instant updates form freshly parsed CAN frames. Be efficient in this
		 method!  This will block further CAN reads and parsings.
		 \param canFrame The most recent CAN frame.
		 */
		virtual void newDataNotification(CanFrame* canFrame) = 0;

	public:
		virtual ~CanListener() { };
		
	};

	/*! \class Can
	 \brief A class that handles the CAN data

	 This class interfaces with a Panda::Usb class to handle the polling of the vehicle
	 CAN bus

	 */
	class Can : protected Mogi::Thread, public UsbListener {
	public:
		Can();
		~Can();

		void initialize();	// needs USB device

		/*! \brief Saves raw CANBus data to a file.
		 More correctly, this saves data read by the Panda USB interface to file.
		 \param filename The filename and path for data to be saved.
		 */
		void saveToFile(const char* filename);

		/*! \brief Saves CANBus data to a file in CSV format.
		 This is after parsing raw data into CanFrames, in a more readable format.
		 \param filename The filename and path for data to be saved.
		 */
		void saveToCsvFile(const char* filename);

		/*! \brief Sets the USB handler for CAN UART communication
		 \param usbHandler The Panda USB handler.
		 */
		void setUsb( Panda::Usb* usbHandler );

		/*! \brief Adds and Observer for updates on successful parsed CAN packets
		 \param listener The observer for fresh CAN data.
		 */
		void addObserver( CanListener* listener );

		/*! \brief Starts a reading and parsing thread.
		 A valid and initialized USB handler is needed to be set for success.
		 */
		void startParsing();

		/*! \brief Stops the reading and parsing thread.
		 Called after startParsing() when done with CAN data
		 */
		void stopParsing();

		/*! \brief Sends a CAN frame to the bus
		 \param frame The frame, with data, to be sent
		 */
		void sendMessage( CanFrame& frame );

	private:
//		bool currentlyRequesting = false;
		std::list<CanFrame> canFrames;

		std::vector<CanListener*> listeners;
		Usb* usbHandler = NULL;

		FILE* canDump;
		FILE* csvDump;

		void writeCsvToFile(CanFrame* frame, unsigned char* buffer, int bufLength);
		void writeRawToFile(char* buffer, size_t length);

		// Overload frum UsbListener
		void notificationCanRead(char* buffer, size_t bufferLength);

		// Overload from Mogi::Thread
		void doAction();
	};

}

#endif
