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
		double latitude;
		/*! \brief Longitude from GGA or RMC NMEA strings.
		 */
		double longitude;
		/*! \brief Altitude from GGA NMEA strings.
		 */
		double altitude;
		/*! \brief Geoidal Seperation from GGA or RMC NMEA strings.
		 */
		double geoidalSeparation;
	} GpsPosition;

	/*!
	 \struct GpsMotion
	 \brief Holds motion information from RMC NMEA strings.

	 verticalSpeed is calculated from the NMEA parser, and may not be accurate
	 */
	typedef struct _GpsMotion {
		/*! \brief Speed in Knots.
		 */
		double speed;
		/*! \brief Vertical speed calculated from NMEAParser, not from an NMEA string.
		 */
		double verticalSpeed;
		/*! \brief Direction of travel.
		 */
		double course;
	} GpsMotion;

	/*!
	 \struct GpsMotion
	 \brief Holds quality information from GSA/GGA NMEA strings.
	 */
	typedef struct _GpsQuality {
		/*! \brief Quality indicator from GGA string
		 */
		int indicator;
		/*! \brief Position Dilution of Precision (3D)
		 */
		double PDOP;
		/*! \brief Horizontal Dilution of Precision
		 */
		double HDOP;
		/*! \brief Vertical Dilution of Precision
		 */
		double VDOP;
	} GpsQuality;

	/*!
	 \struct GpsSatellite
	 \brief Holds information of a single satellite from GSV NMEA strings.
	 */
	typedef struct _GpsSatellite {
		/*! \brief Satellits are not always visible
		 */
		bool visible;
		/*! \brief Azimuth of the satellite relative to current position
		 */
		double azimuth;
		/*! \brief Elevation of the satellite relative to current position
		 */
		double elevation;
		/*! \brief Signal-to-noise ratio in C/NO of the satellite
		 */
		int SNR;
		/*! \brief Satellite identifier, or PRN
		 */
		int ID;
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
		int diffId;	// Not in NMEA?
		/*! \brief Differential Age from GGA strings
		 */
		double diffAge;	// Not in NMEA?
		/*! \brief Fix type mode, either 'M' for Manual or 'A'(?) for Auto from GSA string
		 */
		char automode;
		/*! \brief Fix type from GSA string: 1 for no fix, 2 for 2D fix, 3 for 3D fix
		 */
		int fixType;
		/*! \brief Not a part NMEA strings, from NMEAParser.
		 */
		int GGAcount;	// Not in NMEA
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
		int timeMilliseconds;
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
		double magneticVariation;
		/*! \brief Total satellites in use
		 */
		int satelitesInUse;
		/*! \brief Total satellites in view
		 */
		int satelitesInView;
		/*! \brief A mapping of satellite PRN(int) to satellite information(GpsSatellite)
		 */
		std::map<int,GpsSatellite> satellites;
		/*! \brief Basic settings and statuses of the GPS
		 */
		GpsInfo info;
		/*! \brief Total number of parsed NMEA strings
		 */
		int successfulParseCount = 0;
	} GpsData;

}

#endif
