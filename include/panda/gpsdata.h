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

#ifndef PANDA_GPS_DATA_H
#define PANDA_GPS_DATA_H

#include <map>
#include <time.h>

namespace Panda {

	/*!
	 \struct GpsPosition
	 \brief Holds pose information from RMC/GGA NMEA strings
	 */
	typedef struct _GpsPosition {
		/*! \brief Latitude from GGA or RMC NMEA strings.
		 */
		double latitude = 0;
		/*! \brief Longitude from GGA or RMC NMEA strings.
		 */
		double longitude = 0;
		/*! \brief Altitude from GGA NMEA strings.
		 */
		double altitude = 0;
		/*! \brief Geoidal Seperation from GGA or RMC NMEA strings.
		 */
		double geoidalSeparation = 0;
	} GpsPosition;

	/*!
	 \struct GpsMotion
	 \brief Holds motion information from RMC NMEA strings.

	 verticalSpeed is calculated from the NMEA parser, and may not be accurate
	 */
	typedef struct _GpsMotion {
		/*! \brief Speed in Knots.
		 */
		double speed = 0;
		/*! \brief Vertical speed calculated from NMEAParser, not from an NMEA string.
		 */
		double verticalSpeed = 0;
		/*! \brief Direction of travel.
		 */
		double course = 0;
	} GpsMotion;

	/*!
	 \struct GpsMotion
	 \brief Holds quality information from GSA/GGA NMEA strings.
	 */
	typedef struct _GpsQuality {
		/*! \brief Quality indicator from GGA string
		 */
		int indicator = 0;
		/*! \brief Position Dilution of Precision (3D)
		 */
		double PDOP = 0;
		/*! \brief Horizontal Dilution of Precision
		 */
		double HDOP = 0;
		/*! \brief Vertical Dilution of Precision
		 */
		double VDOP = 0;
	} GpsQuality;

	/*!
	 \struct GpsSatellite
	 \brief Holds information of a single satellite from GSV NMEA strings.
	 */
	typedef struct _GpsSatellite {
		/*! \brief Satellits are not always visible
		 */
		bool visible = 0;
		/*! \brief Azimuth of the satellite relative to current position
		 */
		double azimuth = 0;
		/*! \brief Elevation of the satellite relative to current position
		 */
		double elevation = 0;
		/*! \brief Signal-to-noise ratio in C/NO of the satellite
		 */
		int SNR = 0;
		/*! \brief Satellite identifier, or PRN
		 */
		int ID = 0;
	} GpsSatellite;

	/*!
	 \struct GpsInfo
	 \brief Holds status information from various NMEA strings
	 */
	typedef struct _GpsInfo {
		/*! \brief Either 'V' for Inactive (no locaiton updates) or 'A' for Active (GPS has a fix)
		 */
		char status = 'V';
		//bool datavalid;
		/*! \brief Differential ID from GGA strings
		 */
		int diffId = 0;
		/*! \brief Differential Age from GGA strings
		 */
		double diffAge = 0;
		/*! \brief Fix type mode, either 'M' for Manual or 'A'(?) for Auto from GSA string
		 */
		char automode = 0;
		/*! \brief Fix type from GSA string: 1 for no fix, 2 for 2D fix, 3 for 3D fix
		 */
		int fixType = 0;
		/*! \brief Not a part NMEA strings, from NMEAParser.
		 */
		int GGAcount = 0;	// Not in NMEA
	} GpsInfo;


	/*!
	 \struct GpsData
	 \brief A consolidation of all parsed NMEA data.
	 */
	typedef struct _GpsData {
//		GpsTime time;
		/*! \brief The latest time from the GPS, eithe RMC or GGA strings
		 */
		struct tm time;
		/*! \brief GPSData.time cannot account for sub-second precision, this stores the milliseconds from an RMC string
		 */
		int timeMilliseconds = 0;
		/*! \brief The current position information of the GPS fix
		 */
		GpsPosition pose;
		/*! \brief The current motion of the fix
		 */
		GpsMotion motion;
		/*! \brief The current quality of the fix
		 */
		GpsQuality quality;
		/*! \brief The magnetic variation, if supported.  Unsure if this is porivded by the Ublox M8
		 */
		double magneticVariation = 0;
		/*! \brief Total satellites in use
		 */
		int satelitesInUse = 0;
		/*! \brief Total satellites in view
		 */
		int satelitesInView = 0;
		/*! \brief A mapping of satellite PRN(int) to satellite information(GpsSatellite)
		 */
		std::map<int,GpsSatellite> satellites;
		/*! \brief Basic settings and statuses of the GPS
		 */
		GpsInfo info;
		/*! \brief Total number of parsed NMEA strings
		 */
		int successfulParseCount = 0;

		_GpsData() {
			time.tm_min = 0;
			time.tm_sec = 0;
			time.tm_mon = 0;
			time.tm_year = 0;
			time.tm_hour = 0;
			time.tm_mday = 0;
		}
	} GpsData;

}

#endif
