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

#ifndef PANDA_HANDLER_H
#define PANDA_HANDLER_H

#include "panda_version_i.h"

#include "panda/usb.h"
#include "panda/can.h"
#include "panda/gps.h"
#include "panda/gpsTime.h"
#include "panda/gpsTracker.h"
#include "panda/obd-pid.h"
#include "panda/vin.h"

/**
 * @namespace Panda
 * \brief Handles USB communication and parsing with a comma.ai Panda
 */
namespace Panda {
	/*! \brief Prints the current git version of libpanda and supported git version of panda firmware
	*/
	void printVersions();

	/*! \class Handler
	 \brief A class that handles the Usb, GPS, and CAN data

	 This class deals with initializing objects for USB device handling, GPS parsing, and
	 CAN parsing.  The implementation is multithreaded.  The observer software pattern is
	 used for each contained object (GPS, USB, CAN) as the intended method to retrieve
	 Data-In-Real-Time (DIRT).
	 
	 A description of the Panda LEDs from https://github.com/commaai/openpilot/wiki/FAQ#what-do-the-led-colors-mean
	 * White: CAN send enabled
	 * Red: This is your panda's heartbeat(power). It fades in and out
	 * Green: Bad firmware or firmware flashing (only green, fast)
	 * Blue (static): CAN detected
	 * Blue (fades in and out): power saving

	 */
	class Handler {
	public:
		Handler();
		~Handler();

		/*! \brief Start up USB, GPS, and decoding threads
		 */
		void initialize();

		/*! \brief Stop USB, GPS, and decoding threads
		 This should be called after initialize() when done with data collection.
		 */
		void stop();

		/*! \brief Add listeners to CAN packet updates
		 \param can A concrete object that subscribes to CAN packet updates.
		 */
		void addCanObserver( CanListener& can );

		/*! \brief Add listeners to GPS data updates
		 \param gps A concrete object that subscribes to GPS data updates.
		 */
		void addGpsObserver( GpsListener& gps );

		/*! \brief Add listeners to USB data updates
		 \param usb A concrete object that subscribes to USB data updates.
		 */
		void addUsbObserver( UsbListener& usb );

		/*! \brief Returns the in-use CAN parser.
		 \return The CAN parser.
		 */
		Can& getCan();

		/*! \brief Returns the in-use GPS parser.
		 \return The GPS parser.
		 */
		Gps& getGps();

		/*! \brief Returns the in-use USB handler.
		 \return The USB Handler.
		 */
		Usb& getUsb();
		
		/*! \brief Returns the vehicle's manufacturer.  If it has not been set, failed to read, or
		 \return The vehicle manufacturer after VIN is set
		 */
		VehicleManufacturer getVehicleManufacturer();
		
		/*! \brief Force sets the Handler's VIN.  This should not be called manually if a known vehicle type is connected
		 \param vin The VIN
		 */
		void forceSetVin(const unsigned char* vin);

	private:
		Usb mUsb;
		Can mCan;
		Gps mGps;
		
		// This will be parsed from reading the VIN, if supported by libapnda
		VehicleModel vModel;
		VehicleRegion vRegion;
		VehicleManufacturer	vMake;
		int vYear;
		
		void requestVin();
	};
}

#endif
