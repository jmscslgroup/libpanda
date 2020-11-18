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

#ifndef PANDA_USB_H
#define PANDA_USB_H

#include <iostream>
//#include <libusb.h>	// for macOS catalina
#include <libusb-1.0/libusb.h>
#include <vector>

#include "mogi/thread.h"

#define TIMEOUT (0)	// For libusb, in ms
#define NUM_CAN_PACKETS_PER_TRANSFER (4)	// I've had luck with a value of 4, but failure at 3 or less
#define BYTES_PER_CAN_PACKET (16)
#define BYTES_PER_UART_TRANSFER (64)

namespace Panda {
	class Usb;

	typedef enum {
		MODE_SYNCHRONOUS,
		MODE_ASYNCHRONOUS,
		MODE_ISOCHRONOUS
	} UsbMode;

	class UsbListener {
	private:
		friend class Usb;
	protected:
		// Overload the following for new incoming data
		// You MUST be efficient in this call, or you will hold up the notification thread.
		// Suggestion is to copy data, then return.  Spin up a new thread if you must
		//virtual void newDataNotification(char* buffer, size_t bufferLength) = 0;

		virtual void notificationUartRead(char* buffer, size_t bufferLength) {};
		virtual void notificationUartWrite() {};

		virtual void notificationCanRead(char* buffer, size_t bufferLength) {};
		virtual void notificationCanWrite() {};

	public:
		virtual ~UsbListener() = 0;

	};

	class Usb : protected Mogi::Thread {
	public:
		Usb(UsbMode mode = MODE_ASYNCHRONOUS);
		~Usb();

		// initialize the USB panda interface
		void initialize();

		// This uses an observer/listener pattern, listeners are notified when new USB info comes in.
		//  - Note, make sure Usb::stopRecording() is called BEFORE listener destruction, otherwise a
		//	  segfault will most likely happen
		// This is the ONLY way to read data fromt he USB device in this class
		void addObserver( UsbListener* listener );

		
		// Send USB data
		// Returns 0 on success, otherwise failure
		int sendCanData( unsigned char* buffer, int length);
		// Invokes a read request for CAN data:
		void requestCanData();
		void requestUartData();

		// uart write and read, for GPS
		bool hasGpsSupport();
		void uartWrite(const char* buffer, int length);
//		int uartRead(unsigned char* buffer);
		void uartPurge();
		void canPurge();
		void setUartBaud(int uart, int baud);
		void setUartParity(int uart, int parity);

		// Panda device handling:
		void setEspPower(bool enable);
		void espReset(int uart, int bootmode=0);
		void getPandaSerial(unsigned char* serial16);
		void setCanLoopback( int enable );
		void getFirmware(unsigned char* firmware128 );
		unsigned char getHardware();
		struct tm getRtc();
		void setSafetyMode(uint16_t mode);

		// The standard getters/setters:
		void setOperatingMode(UsbMode mode);
		const char* getModeAsString() const;
		std::string getUsbSerialNumber() { return serialNumber; };

		// Begin the USB data transfering, for reading:
		void startRecording();
		void stopRecording();

	private:
		bool hasGps;
		UsbMode mode;

		std::vector<UsbListener*> listeners;
		char serialNumber[200];	// Stores the Panda's usb serial number
		char pandaSerial[16];  // Panda's serial number

		// libusb variables:
		//struct libusb_device *device;
		struct libusb_device_handle *handler;
		//struct libusb_transfer *transfer;
		//struct libusb_transfer *outgoingTransfer;
//		struct libusb_transfer *uartTransfer;

		unsigned char bufferSynchronousCan[BYTES_PER_CAN_PACKET*NUM_CAN_PACKETS_PER_TRANSFER];	// I think Rahul wanted this at 16, which seems a bit small...
		unsigned char bufferSynchronousUart[BYTES_PER_UART_TRANSFER];
		unsigned int bufferLengthSynchronousUart;

		//void asyncControlTransferUart();
		void sendModedUart();

		// Opens the panda, a USB device with manufacture string "comma.ai"
		int openDeviceByManufacturer(libusb_device **devs, const char* manufacturerName);

		// Functions called by callbacks for completed transfer processing
		void processNewCanRead(char* buffer, size_t length);
		void processNewCanSend(size_t length);
		void processNewUartRead(char* buffer, size_t length);
		void processNewUartSend(size_t length);


		// callback function for libusb when thread is running
		static void transferCallbackReadCan(struct libusb_transfer *transfer);
		static void transferCallbackSendCan(struct libusb_transfer *transfer);
		static void transferCallbackReadUart(struct libusb_transfer *transfer);
		static void transferCallbackSendUart(struct libusb_transfer *transfer);

		// prints libusb status errors from most libsusb functions
		static void printError(int status);
		static void printErrorTransfer(libusb_transfer_status status);

		// Override from Mogi::Thread
		void entryAction();
		void doAction();


		// Simple handling layers:
		void sendPandaHardwareSimple(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index);
		void readPandaHardwareSimple(uint8_t requestType, uint8_t request, unsigned char* buffer, uint16_t length);

	};

}

#endif
