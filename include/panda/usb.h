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

#include <string>

#include "mogi/thread.h"
#include "pandadefinitions.h"

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

	const char* safetyModelToString( int safetyModel );
	const char* carHarnessStatusToString( int car_harness_status);
	const char* faultStatusToString( int fault_status);
	std::string faultsToString( int faults );
	const char* usbPowerModeToString( int usb_power_mode );
	const char* hardwareTypeToString( int hardwareType );
	void printPandaHealth( const PandaHealth& health );

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

		// uart write and read, for GPS
		bool hasGpsSupport();
		void uartWrite(const char* buffer, int length);
//		int uartRead(unsigned char* buffer);
		

		// Panda USB control requests.  These involve comma.ai/panda/board/usb_comms.h data transfers
		PandaRtcTimestamp getRtc(); 				// Neither the Red nor Black Panda have an RTC battery
		void setRtcYear( int n );
		void setRtcMonth( int n );
		void setRtcDay( int n );
		void setRtcWeekday( int n );
		void setRtcHour( int n );
		void setRtcMinute( int n );
		void setRtcSeconds( int n);
		
		void setIrPower(bool enable);                   // Unused on Red and Black
		void setFanPower(int power);                    // Unused on Red and Black
		uint16_t getFanRpm();                           // Unused on Red and Black
		void setPhonePower(bool enable);                // Unused on Red and Black
		void getCanDebug();                             // I think this invokes debug info through a separate UART/debug console
		
		unsigned char getHardware();                    // Whether this is  aRed, Black, White, Grey, Uno, Dos or Pedal
		void getPandaSerial(unsigned char* serial16);   // This is different than the USB serial
		void enterBootloaderMode( unsigned char mode ); // Making this for the sake of completeness, but will go unimplemented. 0 for bootloader (debug firmware build only), 1 for softloader
		
		void getHealth( PandaHealth* health );          // Reports a bunch of things, see printPandaHealth()
		void getFirmware(unsigned char* firmware128 );  // This is also called a signature, generated on compile time
		
		void getGitVersion( unsigned char* version );   // This responds with the firmware's git version
		void systemReset();                             // Calls same function after flashing so this must be a true system reset
		
		void setEspPower(bool enable);                  // This is for GPS power, unsused on Red
		void espReset(int uart, int bootmode=0);        // GPS reset (with optional boot mode)
		void setGmlanOrObdCanMode(unsigned char gmlan); // "set GMLAN (white/grey) or OBD CAN (black) multiplexing mode"
		
		/*! \brief Sets the low-level CAN message blocking and state handling inside the panda itself
		 \param mode The particular safety mode for vehicle model.  See macros SAFETY_* for this field
		 \param params This value is sent to the the particular safety mode's init function, to configure state handling if needed
		 */
		void setSafetyMode(uint16_t mode, uint16_t params);
		void getHealthAndCanVersions( unsigned char* healthVersion, unsigned char* canVersion); // Determines struct types
		
		void setCanBitrate(  int bus, int baud );       // Maybe not for CAN FD
		void setAlternativeExperience( int alternativeExperience );  // no idea
		
		void requestUartData(); // This really shouldn't be a control transfer, but I've thrown it into the asynch mix anyway
		void setUartBaud(int uart, int baud);           //  This fits higher bauds by dividing baud request by 300 to fit within uint16_t
		void setUartParity(int uart, int parity);       // GPS module is by defualt 8N1, so always set to 0
		void setUartBaudSmall(int uart, int baud);	    // Only usefulr for bauds that fit in uint16_t, use other
		
		void setCanLoopback( int enable );              // This is for debugging purposes
		
		void setUsbPowerMode( unsigned char mode );     // For working with EON charging or self-power
		void setPowerSaveEnable( bool enable );         // Places
		
		void setKLlineWakupPulse( unsigned char KL );   // Specific for some vehicle makes
		
		void canPurge();                                // Clear ring buffers
		void uartPurge();                               // Clear ring buffers
		
		void sendHeartBeat();                           // Call more than 0.5 Hz to keep talking to panda
		
		void setKLlineFiveBaudInit( int config, int baud );
		void setClockSourceMode( int mode );	// non-functional in Pandas (see Uno and Dos)
		void setSiren( bool enable );
		void setGreenLed( bool enable );
		void disableHeartbeat(); // only functional in debug firmware builds
		
		void setCanFdBaud( int bus, int baud ); // unsure on units
		void getCanFdEnabled( int bus, bool* fdEnabled, bool* brsEnabled ); // byte 0 is if FD enabled, byte 1 is if BRS enabled
		void enterDeepSleep(); // Unsure how to wake up

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

		// Opens the panda, a USB device with manufacture string "circles"
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
//		void sendPandaHardwareSimple(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index);
//		void readPandaHardwareSimple(uint8_t requestType, uint8_t request, unsigned char* buffer, uint16_t length);
		void sendPandaHardwareSimple(uint8_t request, uint16_t value, uint16_t index);
		void readPandaHardwareSimple(uint8_t request, unsigned char* buffer, uint16_t length);

	};

}

#endif
