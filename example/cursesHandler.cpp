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

#include "cursesHandler.h"


#include <sys/select.h>
#include <cstdlib>
#include <unistd.h>
#include <cstdio>
#include <string.h>
#include <iostream>
#include <math.h>

#include "curses-gfx.h"
#include "curses-clock.h"

#define SIMPLE_TERM

#ifdef SIMPLE_TERM
//#undef ACS_LRCORNER
//#undef  ACS_LLCORNER
//#undef  ACS_URCORNER
//#undef  ACS_ULCORNER
#undef  ACS_HLINE
//#undef  ACS_VLINE
//#undef  ACS_TTEE
//#undef  ACS_BTEE
//#undef  ACS_RTEE
//#undef  ACS_LTEE
//#undef  ACS_PLUS

//#define  ACS_LRCORNER '`'
//#define  ACS_LLCORNER '`'
//#define  ACS_URCORNER '.'
//#define  ACS_ULCORNER '.'
#define  ACS_HLINE '-'
//#define  ACS_VLINE '|'
//#define  ACS_TTEE '.'
//#define  ACS_BTEE '\''
//#define  ACS_RTEE '|'
//#define  ACS_LTEE '|'
//#define  ACS_PLUS '+'

#endif

//static int mainRunning = FALSE;
CursesHandler* CursesHandler::mCursesHandler = NULL;

CursesHandler::CursesHandler()
{
	displayMode = GPS;
	canSortMode = SORT_ID;
	reverseSortEnable = false;
	for (int i = 0; i < sizeof(highlightField); i++) {
		highlightField[i] = 0;
	}
}

CursesHandler* CursesHandler::getInstance() {
	if (mCursesHandler == NULL) {
		mCursesHandler = new CursesHandler;
	}
	return mCursesHandler;
}

void CursesHandler::destroy() {
	if(mCursesHandler != NULL) {
		mCursesHandler->cleanupConsole();
		delete mCursesHandler;
		mCursesHandler = NULL;
	}
}

bool isLineChar(int row, int column) {
	chtype ch = mvinch(row, column);
	return (ch == ACS_LRCORNER) ||
	(ch == ACS_LLCORNER) ||
	(ch == ACS_URCORNER) ||
	(ch == ACS_ULCORNER) ||
	(ch == ACS_HLINE) ||
	(ch == ACS_VLINE) ||
	(ch == ACS_TTEE) ||
	(ch == ACS_BTEE) ||
	(ch == ACS_RTEE) ||
	(ch == ACS_LTEE) ||
	(ch == ACS_PLUS);

}

void fillCorner(int row, int column) {
	int type = ((int)isLineChar(row-1, column)) |	// top
	((int)isLineChar(row+1, column) << 1) |		// bottom
	((int)isLineChar(row, column-1) << 2) |		// left
	((int)isLineChar(row, column+1) << 3);		// right
	switch (type) {
		case (1+4):
			mvaddch(row, column, ACS_LRCORNER);
			break;
		case (1+8):
			mvaddch(row, column, ACS_LLCORNER);
			break;
		case (2+4):
			mvaddch(row, column, ACS_URCORNER);
			break;
		case (2+8):
			mvaddch(row, column, ACS_ULCORNER);
			break;
		case (1+2+4):
			mvaddch(row, column, ACS_RTEE);
			break;
		case (1+2+8):
			mvaddch(row, column, ACS_LTEE);
			break;
		case (1+4+8):
			mvaddch(row, column, ACS_BTEE);
			break;
		case (2+4+8):
			mvaddch(row, column, ACS_TTEE);
			break;
		case (1+2+4+8):
			mvaddch(row, column, ACS_PLUS);
			break;

		default:
			return;
	}
}

void fillVTees(int row, int column, int endRow ) {
	for (int i = row+1; i <= endRow-1; i++) {
		bool pluscandidate = false;
		if (isLineChar(i, column-1)) {
			pluscandidate = true;
			mvaddch(i, column, ACS_RTEE);
			fillCorner(i, column-1);
		}
		if (isLineChar(i, column+1)) {
			if (pluscandidate) {
				mvaddch(i, column, ACS_PLUS);
			} else {
				mvaddch(i, column, ACS_LTEE);
			}
			fillCorner(i, column+1);
		}
	}
}
void fillHTees(int row, int column, int endColumn) {
	for (int i = column+1; i <= endColumn-1; i++) {
		bool pluscandidate = false;
		if (isLineChar(row-1, i)) {
			pluscandidate = true;
			mvaddch(row, i, ACS_BTEE);
			fillCorner(row-1, i);
		}
		if (isLineChar(row+1, i)) {
			if (pluscandidate) {
				mvaddch(row, i, ACS_PLUS);
			} else {
				mvaddch(row, i, ACS_TTEE);
			}
			fillCorner(row+1, i);
		}
	}
}



void miniWindow(int row, int endRow, int column, int endcolumn) {
	mvhline(   row, column+1, ACS_HLINE, endcolumn-column-1);
	mvhline(endRow, column+1, ACS_HLINE, endcolumn-column-1);

	mvvline(row+1, column, ACS_VLINE, endRow-row-1);
	mvvline(row+1, endcolumn, ACS_VLINE, endRow-row-1);

	fillVTees(row, column, endRow);
	fillVTees(row, endcolumn, endRow);
	fillHTees(row, column, endcolumn);
	fillHTees(endRow, column, endcolumn);

	fillCorner(row, column);
	fillCorner(endRow, column);
	fillCorner(row, endcolumn);
	fillCorner(endRow, endcolumn);
}

void printCentered(int y, int x, int width, const char* str) {
	mvprintw(y, x + (float)width/2.0 - (float)strlen(str)/2.0 + 1.0, str);
}

void CursesHandler::updateScreen( Panda::Handler& handler, CanFrameStats& canFrameStats, UsbStats& usbStats )
{


	//clear();
	erase();

	int menuX = 0;
	int menuY = 0;
	int menuWidth = 23;
	int menuHeight = 12;

	miniWindow(menuY, menuY+menuHeight, menuX, menuX+menuWidth);	// Panda menu
	printCentered(menuY+1, menuX, menuWidth, "Panda Menu");
	// Panda menu:
	mvprintw(menuY + 2, menuX + 1,"USB mode: %s", handler.getUsb().getModeAsString());

	printCentered(menuY+10, menuX, menuWidth, "\'g\' to toggle GPS/CAN");

	printCentered(menuY + 11, menuX, menuWidth, "\'ESC\' to Exit");

	switch (displayMode) {
		case GPS:
			drawGps(handler);
			break;


		case CAN:
			drawCan(canFrameStats, usbStats);
			break;
	}



//	mvprintw(rowSave++,0,"Console:");
//	move(rowSave++,0);

	refresh();
}


void CursesHandler::drawGps( Panda::Handler& handler ) {
	int row = 0;
	int col = 0;
	//mvprintw(row,31,"Verbose mode(press v for quiet)");

	//╬╝╚╔╦╩╠╣╗║
	//	mvprintw(row++,0,"+----------------------+--------------------+------------------------------------------------------+");
	//	mvprintw(row++,0,"|      Panda Menu      |        GPS         | Satellite                                            |");
	//	mvprintw(row++,0,"|    version:          |                    |    Map                                               |");
	//	mvprintw(row++,0,"|                      |                    |                                                      |");
	//	mvprintw(row++,0,"|                      |                    |                                                      |");
	//	mvprintw(row++,0,"|                      |                    |                                                      |");
	//	mvprintw(row++,0,"|                      |                    |                                                      |");
	//	mvprintw(row++,0,"|                      |                    |                                                      |");
	//	mvprintw(row++,0,"|                      |                    |                                                      |");
	//	mvprintw(row++,0,"|                      |                    |                                                      |");
	//	mvprintw(row++,0,"|                      |                    |                                                      |");
	//	mvprintw(row++,0,"|ESC or ctrl-c to Exit |                    |                                                      |");
	//	mvprintw(row++,0,"+----------------------+---+----------------+                                                      |");
	//	mvprintw(row++,0,"|                          |   GPS Stats    |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"|                          |                |                                                      |");
	//	mvprintw(row++,0,"+--------------------------+----------------+------------------------------------------------------+");
	//	int rowSave = row+1;

	//border(0, 0, 0, 0, 0, 0, 0, 0);
	
	
	double xyscale = 2.25; // since column space is stretched relative to row space

	int menuX = 0;
	int menuY = 0;
	int menuWidth = 23;
	int menuHeight = 12;

	int gpsX = menuX+menuWidth;
	int gpsY = menuY;
	int gpsWidth = 21;
	int gpsHeight = menuY+menuHeight;

	int satX = menuX;
	int satY = menuY+menuHeight;
	int satWidth = 27;
//	int satHeight = 17;
	int numSatellites = handler.getGps().getData().satellitesGps.size() + handler.getGps().getData().satellitesGlonass.size();
	int satHeight = numSatellites > 12 ? numSatellites + 4 : 16;

	int statX = satX + satWidth;
	int statY = satY;
	int statWidth = 17;
	int statHeight = satHeight;
	
	double alpha = 0.95;
    if(!isnan(handler.getGps().getData().motion.course)) {
        double gpsHeading = M_PI/180.0 * handler.getGps().getData().motion.course;
        //	static double gpsHeading;
        //	gpsHeading += 0.1;
        if (gpsHeading != 0) {	// HACK
            if (gpsHeading > M_PI*2.0) {
                gpsHeading -= M_PI*2.0;
            }
            if (heading - gpsHeading > M_PI) {
                heading -= M_PI*2.0;
            } else if (heading - gpsHeading < -M_PI) {
                heading += M_PI*2.0;
            }
            heading = alpha * heading + (1-alpha)*gpsHeading;
            if (heading > M_PI*2.0) {
                heading = heading - floor(heading/(M_PI*2.0))*M_PI*2.0; // float modulus
            } else if (heading < 0) {
                heading = heading + ceil(-heading/(M_PI*2.0))*M_PI*2.0;
            }
        }
        
        heading = gpsHeading;
    }

//	miniWindow(menuY, menuY+menuHeight, menuX, menuX+menuWidth);	// Panda menu
	miniWindow(gpsY, gpsY+gpsHeight, gpsX, gpsX+gpsWidth);	// GPS
	miniWindow(satY, satY+satHeight, satX, satX+satWidth); // Satellites
	miniWindow(statY, statY+statHeight, statX, statX+statWidth); // Stats

	int mapX = gpsX+gpsWidth+1;
	int mapY = gpsY;
	int mapHeight = 28;
	int mapWidth = mapHeight*xyscale-2;
	miniWindow( mapY, mapY+mapHeight, mapX, mapX+mapWidth);

	//miniWindow(0,29,0,99); // Full border
	
	
	// All the folling "+1", "+0.5", "/2.x" are herustics, a pain to tune
	int gpsClockX = mapX;
	int gpsClockY = mapY+mapHeight;
	int gpsClockWidth = mapWidth/2;
	int gpsClockHeight = (gpsClockWidth)/xyscale;
	miniWindow( gpsClockY, gpsClockY+gpsClockHeight + 1, gpsClockX, gpsClockX+gpsClockWidth);
	
	int sysClockX = gpsClockX+gpsClockWidth+1;
	int sysClockY = gpsClockY;
	int sysClockWidth = gpsClockWidth;
	int sysClockHeight = gpsClockHeight;
	miniWindow( sysClockY, sysClockY+sysClockHeight + 1, sysClockX, sysClockX+sysClockWidth);
	
	struct timeval time;
	gettimeofday(&time, NULL);
	Coordinates2D clockCenter;
	clockCenter.x = (double)sysClockX + (double)sysClockWidth/2.0 + 0.5;
	clockCenter.y = (double)sysClockY + (double)sysClockHeight/2.0 + 0.5;
	drawclock((double)sysClockWidth/2.2, (double)sysClockHeight/2.3, clockCenter, true, time);
	
	struct tm gpsTmCopy = handler.getGps().getData().time;
	time_t gpsTime_t = mktime(&gpsTmCopy);
	struct timeval gpsTime;
	gpsTime.tv_sec = gpsTime_t;
	gpsTime.tv_usec = (handler.getGps().getData().timeMilliseconds)*1000;
	clockCenter.x = (double)gpsClockX + (double)gpsClockWidth/2.0 + 0.5;
	clockCenter.y = (double)gpsClockY + (double)gpsClockHeight/2.0 + 0.5;
	drawclock((double)gpsClockWidth/2.2, (double)gpsClockHeight/2.3, clockCenter, true, gpsTime);
	

	attron(A_BOLD);
//	printCentered(menuY+1, menuX, menuWidth, "Panda Menu");
	printCentered(gpsY+1, gpsX, gpsWidth, "GPS");
	mvprintw(mapY+1, mapX+2, "Satellite");	// not centered
	mvprintw(mapY+2, mapX+5, "Map");

	printCentered(satY+1, satX, satWidth, "GPS Satellites");
	printCentered(statY+1, statX, statWidth, "GPS Stats");
	
	mvprintw(sysClockY+1, sysClockX+2, "System");
	mvprintw(gpsClockY+1, gpsClockX+2, "GPS");
	attroff(A_BOLD);

//	// Panda menu:
//	row = menuY + 4;
//	col = menuX + 1;
//	mvprintw(row,col,"USB mode: %s", handler.getUsb().getModeAsString());
//	mvprintw(menuY + 11, menuX + 1, "ESC or ctrl-c to Exit");


	// GPS printing

	col = gpsX + 1;
	row = gpsY + 2;
	mvprintw(row++,col,"Status :%12s",  handler.getGps().getData().info.status == 'A' ? "Active" : "Inactive");
	mvprintw(row++,col,"Time   : %02d:%02d:%02d.%.02d",
			 handler.getGps().getData().time.tm_hour,
			 handler.getGps().getData().time.tm_min,
			 handler.getGps().getData().time.tm_sec,
			 handler.getGps().getData().timeMilliseconds/10 );
	mvprintw(row++,col,"Date   :  %02d/%02d/%04d",
			 handler.getGps().getData().time.tm_mon+1,
			 handler.getGps().getData().time.tm_mday,
			 handler.getGps().getData().time.tm_year+1900 );

	mvprintw(row++,col,"Long   :%12.7f", handler.getGps().getData().pose.longitude );
	mvprintw(row++,col,"Lat    :%12.7f", handler.getGps().getData().pose.latitude );
	mvprintw(row++,col,"Alt    :%12.1f", handler.getGps().getData().pose.altitude );
	mvprintw(row++,col,"GeoSep :%12.7f", handler.getGps().getData().pose.geoidalSeparation );

	//row = 2;
	//col = 49;
	mvprintw(row++,col,"Speed  :%12.6f", handler.getGps().getData().motion.speed );
	mvprintw(row++,col,"Vert Sp:%12.6f", handler.getGps().getData().motion.verticalSpeed );
	mvprintw(row++,col,"Course :%12.6f", handler.getGps().getData().motion.course );

	// GPS other info:
	row = statY + 3;
	col = statX + 1;
	if (handler.getGps().getData().successfulParseCount >= 10000) {
		mvprintw(row++,col,"NMEA Cnt:%6.2fk", (double)handler.getGps().getData().successfulParseCount/1000.0);
	} else if (handler.getGps().getData().successfulParseCount >= 1000000) {
		mvprintw(row++,col,"NMEA Cnt:%6.3fM", (double)handler.getGps().getData().successfulParseCount/1000000.0);
	} else {
		mvprintw(row++,col,"NMEA Cnt:%7d", handler.getGps().getData().successfulParseCount);
	}
	mvprintw(row++,col,"Qual Ind:%7d", handler.getGps().getData().quality.indicator);
	mvprintw(row++,col,"AutoMode:%7c", handler.getGps().getData().info.automode);
	mvprintw(row++,col,"Fix Type:%7d", handler.getGps().getData().info.fixType);
	mvprintw(row++,col,"Diff ID :%7d", handler.getGps().getData().info.diffId);
	mvprintw(row++,col,"Diff Age:%7.2f", handler.getGps().getData().info.diffAge);
	//mvprintw(row++,col,"GGAcount:%7d", handler.getGps().getData().info.GGAcount);
	mvprintw(row++,col,"Magn Var:%7.2f", handler.getGps().getData().magneticVariation);
	mvprintw(row++,col,"HDOP    :%7.2f", handler.getGps().getData().quality.HDOP );
	mvprintw(row++,col,"PDOP    :%7.2f", handler.getGps().getData().quality.PDOP );
	mvprintw(row++,col,"VDOP    :%7.2f", handler.getGps().getData().quality.VDOP );



	mvprintw(satY + 2,satX + 4,"In view:%d In use:%d", handler.getGps().getData().satelitesInView, handler.getGps().getData().satelitesInUse);
	row = satY + 3;
	attron(A_BOLD);
	mvprintw(row,satX + 1,"ID");
	mvprintw(row,satX + 5,"Track");
	mvprintw(row,satX + 12,"Azim");
	mvprintw(row,satX + 18,"Elev");
	mvprintw(row++,satX + 24,"SNR");
	attroff(A_BOLD);
	const std::map<int,Panda::GpsSatellite>* satellites[2] = {&handler.getGps().getData().satellitesGps, &handler.getGps().getData().satellitesGlonass};
	for (int i = 0; i < 2; i++) {
		for (std::map<int,Panda::GpsSatellite>::const_iterator it = satellites[i]->begin(); it != satellites[i]->end(); it++) {
			if(!it->second.visible) {
				attron(A_DIM);
			} else {
				attron(A_BOLD);
			}
			mvprintw(row,satX + 1,"%02d%c", it->first, i == 0 ? 'P' : 'L');
			if(it->second.visible) {
				attroff(A_BOLD);
			}
			mvprintw(row,satX + 6,it->second.visible ? "yes" : " no");
			mvprintw(row,satX + 13,"%3d", (int)it->second.azimuth);
			mvprintw(row,satX + 19,"%2d", (int)it->second.elevation);
			int colorPair = (it->second.SNR / 17)+1;
			colorPair = colorPair > 4 ? 4 : colorPair;
			attron(COLOR_PAIR(colorPair));
			mvprintw(row++,satX + 25,"%02d", it->second.SNR);
			attroff(COLOR_PAIR(colorPair));
			if(!it->second.visible) {
				attroff(A_DIM);
			}
		}
	}


	// Satellite map just for kicks!
	double radius = 13; // row radius(height)
//	double xyscale = 2.2; // since column space is stretched relative to row space
	double centerx = mapX + 1 + 0.5 + radius*xyscale;
	double centery = mapY + 1 + 0.5 + radius;

	mvhline(centery,centerx-radius*xyscale, ACS_HLINE, 1+2*radius*xyscale);
	mvvline(centery-radius, centerx, ACS_VLINE, 1+2*radius);

	fillCorner(centery, centerx);
	
	Coordinates2D east, west;
	east.x = centerx + radius*sin(-heading + M_PI_2)*xyscale;
	east.y = centery - radius*cos(-heading + M_PI_2);
	west.x = centerx + radius*sin(-heading + M_PI + M_PI_2)*xyscale;
	west.y = centery - radius*cos(-heading + M_PI + M_PI_2);
	
	attron(COLOR_PAIR(2));
	ln2(east, west);
	attroff(COLOR_PAIR(2));
	
	Coordinates2D north, south;
	north.x = centerx + radius*sin(-heading)*xyscale;
	north.y = centery - radius*cos(-heading);
	south.x = centerx + radius*sin(-heading + M_PI)*xyscale;
	south.y = centery - radius*cos(-heading + M_PI);
	
	attron(COLOR_PAIR(3));
	ln2(north, south);
	attroff(COLOR_PAIR(3));
	
	

	// draw title:

	// draw a circle:
	for (int i = 0; i < 360; i+= 10) {
		double radian = 3.14159/180.0*(double)i - heading;
		double x = centerx + radius*sin(radian)*xyscale;
		double y = centery - radius*cos(radian);
//		mvprintw( y, x, "*" );
		drawDotFloat(x, y);
	}
	// print scale:
	for (int i = 0; i < 360; i+= 30) {
		double radian = 3.14159/180.0*(double)i - heading;
		int x = centerx + radius*sin(radian)*xyscale;
		int y = centery - radius*cos(radian);
		attron(A_BOLD);
		if (i == 0) {
			attron(COLOR_PAIR(1));
			mvprintw( y, x, "N" );
			attroff(COLOR_PAIR(1));
		} else if(i == 90) {
			attron(COLOR_PAIR(3));
			mvprintw( y, x, "E" );
			attroff(COLOR_PAIR(3));
		} else if(i == 180) {
			attron(COLOR_PAIR(2));
			mvprintw( y, x, "S" );
			attroff(COLOR_PAIR(2));
		} else if(i == 270) {
			attron(COLOR_PAIR(3));
			mvprintw( y, x, "W" );
			attroff(COLOR_PAIR(3));
		} else {
			attroff(A_BOLD);
			mvprintw( y, x-1, "%d", i );
		}
		attroff(A_BOLD);
	}
	// draw a nice origin
	mvaddch(centery, centerx, '+');
	// print satellite data:

	
	
	for (int i = 0; i < 2; i++) {
		for (std::map<int,Panda::GpsSatellite>::const_iterator it = satellites[i]->begin(); it != satellites[i]->end(); it++) {
			if (it->second.visible) {
				//				double satRadius = sin((90.0-(double)it->second.elevation)*3.14159/180.0) * (radius-1);
				double satRadius = (90.0-(double)it->second.elevation)/90.0 * (radius-1);
				double radian = 3.14159/180.0*(double)it->second.azimuth - heading;
				int x = centerx + satRadius*sin(radian)*xyscale;
				int y = centery - satRadius*cos(radian);
				
				int colorPair = (it->second.SNR / 17)+1;
				colorPair = colorPair > 4 ? 4 : colorPair;
				attron(COLOR_PAIR(colorPair+4));
				mvprintw( y, x, "%d", it->second.ID );
				attroff(COLOR_PAIR(colorPair+4));
			}
		}
	}
}

void CursesHandler::drawCan( CanFrameStats& canFrameStats, UsbStats& usbStats ) {
	if (resetUniqueData) {
		resetUniqueData = false;
		canFrameStats.resetUniqueCount();
	}

//	if (highlight >= 0) {
//		canFrameStats.highlightUniqueCount(highlight);
//		highlight = -1;
//	}
	if (highlightEnter == true) {
		highlightEnter = false;
		int valueToHighlight = 0;
		int powOf10 = 1;
		for (int i = highlightCurrentIndex-1; i >= 0; i--) {
			valueToHighlight += powOf10 * (int)(highlightField[i] - '0');
			powOf10 *= 10;
		}
		switch (canSortMode) {
			case SORT_ID:
				canFrameStats.highlightMessageId(valueToHighlight);
				break;
			case SORT_ID_COUNT:
				canFrameStats.highlightCount(valueToHighlight);
				break;
			case SORT_ID_RATE:
				canFrameStats.highlightRate(valueToHighlight);
				break;
			case SORT_UNIQUE_DATA_COUNT:
				canFrameStats.highlightUniqueCount(valueToHighlight);
				break;
			case SORT_ID_BUS:
				canFrameStats.highlightBus(valueToHighlight);
				break;
			default:
				break;
		}
		highlightCurrent = true;
	}

	int menuX = 0;
	int menuY = 0;
	int menuWidth = 23;
	int menuHeight = 12;
	//miniWindow(menuY, menuY+menuHeight, menuX, menuX+menuWidth);	// Panda menu

	printCentered(menuY+10, menuX, menuWidth, "\'g\' to toggle GPS/CAN");

	printCentered(menuY+4, menuX, menuWidth, "Sort by pressing key:");
	printCentered(menuY+5, menuX, menuWidth, "\'m\',\'c\',\'r\',\'u\'");
	printCentered(menuY+6, menuX, menuWidth, "Or use arrow keys    ");
	printCentered(menuY+7, menuX, menuWidth, "'x' Toggle hex/dec IDs");
	printCentered(menuY+8, menuX, menuWidth, "'-' Resets unique data");



	int canX = menuX+menuWidth;
	int canY = 0;
	int canWidth = 41 + CAN_DATA_MAX_LENGTH*2 + 1;
	int canHeight = 40;

	int uiX = menuX;
	int uiY = menuY + menuHeight;
	int uiWidth = menuWidth;
	int uiHeight = 10;

	int fieldY = uiY+3;
	int fieldX = uiX+6;
	miniWindow(uiY, uiY+uiHeight, uiX, uiX+uiWidth);
	printCentered(uiY+2, uiX, uiWidth, "Highlight Value:");
	if (highlightCurrent) {
		attron(A_STANDOUT);
	}
	mvaddch(fieldY, fieldX-1, '|');
	mvaddch(fieldY, fieldX+sizeof(highlightField), '|');
	mvprintw(fieldY, fieldX, highlightField);
	if (highlightCurrent) {
		attroff(A_STANDOUT);
	}

//	printCentered(uiY+2, uiX, uiWidth, "Highlight Field:");


	attron(A_BOLD);
	//	printCentered(menuY+1, menuX, menuWidth, "Panda Menu");
	printCentered(canY+1,canX, canWidth, "CAN top");
	attroff(A_BOLD);

	mvprintw(canY+2,canX+2, "Empty Uart recvs:%0.1f  Empty Can recvs:%0.1f", 100.0-usbStats.getUartSuccess(), 100.0-usbStats.getCanSuccess());



	const char msgId[] = "MessageID:";
	int msgIdCol = canX + 2;
	int msgIdLength = strlen(msgId);
	char formatStringMsgId[16];
	if (messageIdAsHex) {
		snprintf(formatStringMsgId, 16, "0x%%0%dX", msgIdLength-2);
	} else {
		snprintf(formatStringMsgId, 16, "%%%dd", msgIdLength);
	}
	
	const char msgBus[] = "Bus:";
	int msgBusCol = msgIdCol + msgIdLength + 3;
	int msgBusLength = strlen(msgBus);
	char formatStringMsgBus[16];
	snprintf(formatStringMsgBus, 16, "%%%dd", msgBusLength);

	const char msgCnt[] = "Count:";
	int msgCntCol = msgBusCol + msgBusLength + 3;
	int msgCntLength = strlen(msgCnt);
	char formatStringMsgCnt[16];
	snprintf(formatStringMsgCnt, 16, "%%%dd", msgCntLength);

	const char msgRate[] = "  Rate:";
	int msgRateCol = msgCntCol + msgCntLength + 2;
	int msgRateLength = strlen(msgRate);
	char formatStringMsgRate[16];
	snprintf(formatStringMsgRate, 16, "%%%d.2f", msgRateLength);

	const char uniCnt[] = "UniqueData:";
	int uniCntCol = msgRateCol + msgRateLength + 3;
	int uniCntLength = strlen(uniCnt);
	char formatStringUniCnt[16];
	snprintf(formatStringUniCnt, 16, "%%%dd", uniCntLength);
	
	
	const char latestCnt[] = "Latest Data:";
	int latestCntCol = uniCntCol + uniCntLength + 3;
	int latestCntLength = strlen(latestCnt) + 2 + (CAN_DATA_MAX_LENGTH-8)*2;
	char formatStringLatestCnt[CAN_DATA_MAX_LENGTH*2];	// this will need to be dynamic based on length of data
	snprintf(formatStringLatestCnt, CAN_DATA_MAX_LENGTH*2, "%%%dd", latestCntLength);

	int titleRow = canY+3;

	int reverseColumn;
	switch (canSortMode) {
		case SORT_ID:
			canFrameStats.sortById();
			attron(A_STANDOUT);
			mvprintw(titleRow, msgIdCol, msgId);
			attroff(A_STANDOUT);
			mvprintw(titleRow, msgBusCol, msgBus);
			mvprintw(titleRow, msgCntCol, msgCnt);
			mvprintw(titleRow, msgRateCol, msgRate);
			mvprintw(titleRow, uniCntCol, uniCnt);
			mvprintw(titleRow, latestCntCol, latestCnt);
			reverseColumn = msgIdCol + msgIdLength;
			break;
		case SORT_ID_BUS:
			canFrameStats.sortByBus();
			mvprintw(titleRow, msgIdCol, msgId);
			attron(A_STANDOUT);
			mvprintw(titleRow, msgBusCol, msgBus);
			attroff(A_STANDOUT);
			mvprintw(titleRow, msgCntCol, msgCnt);
			mvprintw(titleRow, msgRateCol, msgRate);
			mvprintw(titleRow, uniCntCol, uniCnt);
			mvprintw(titleRow, latestCntCol, latestCnt);
			reverseColumn = msgBusCol + msgBusLength;
			break;
		case SORT_ID_COUNT:
			canFrameStats.sortByIdCount();
			mvprintw(titleRow, msgIdCol, msgId);
			mvprintw(titleRow, msgBusCol, msgBus);
			attron(A_STANDOUT);
			mvprintw(titleRow, msgCntCol, msgCnt);
			attroff(A_STANDOUT);
			mvprintw(titleRow, msgRateCol, msgRate);
			mvprintw(titleRow, uniCntCol, uniCnt);
			mvprintw(titleRow, latestCntCol, latestCnt);
			reverseColumn = msgCntCol + msgCntLength;
			break;
		case SORT_ID_RATE:
			canFrameStats.sortByIdRate();
			mvprintw(titleRow, msgIdCol, msgId);
			mvprintw(titleRow, msgCntCol, msgCnt);
			mvprintw(titleRow, msgBusCol, msgBus);
			attron(A_STANDOUT);
			mvprintw(titleRow, msgRateCol, msgRate);
			attroff(A_STANDOUT);
			mvprintw(titleRow, uniCntCol, uniCnt);
			reverseColumn = msgRateCol + msgRateLength;
			mvprintw(titleRow, latestCntCol, latestCnt);
			break;
		case SORT_UNIQUE_DATA_COUNT:
			canFrameStats.sortByUniqueMessageCount();
			mvprintw(titleRow, msgIdCol, msgId);
			mvprintw(titleRow, msgCntCol, msgCnt);
			mvprintw(titleRow, msgRateCol, msgRate);
			mvprintw(titleRow, msgBusCol, msgBus);
			attron(A_STANDOUT);
			mvprintw(titleRow, uniCntCol, uniCnt);
			attroff(A_STANDOUT);
			mvprintw(titleRow, latestCntCol, latestCnt);
			reverseColumn = uniCntCol + uniCntLength;
			break;
		default:
			break;
	}


	int row = titleRow+1;
	if (reverseSortEnable) {
		attron(A_STANDOUT);
		mvaddch(titleRow, reverseColumn, ACS_UARROW);
		attroff(A_STANDOUT);

		std::vector<IdInfo*>::reverse_iterator it;
		std::vector<IdInfo*>::reverse_iterator end;
		it = canFrameStats.canStatsSorted.rbegin();
		end = canFrameStats.canStatsSorted.rend();
		for ( ; it != end; it++) {
			//unsigned int messageId = it->;
			IdInfo* idStats = *it;
			if (idStats->highlight) {
				attron(A_STANDOUT);
			}

			//mvprintw(row++, col, "%9d  %5d  %10d  ", idStats->ID, idStats->count, idStats->data.size());
			mvprintw(row, msgIdCol, formatStringMsgId, idStats->ID);
			mvprintw(row, msgBusCol, formatStringMsgBus, idStats->bus);
			mvprintw(row, msgCntCol, formatStringMsgCnt, idStats->count);
			mvprintw(row, msgRateCol, formatStringMsgRate, idStats->currentRate);
			mvprintw(row, uniCntCol, formatStringUniCnt, idStats->data.size());
			
			move(row, latestCntCol);
			for (int i = idStats->latest.dataLength-1; i >= 0; i--) {
				printw("%02X", idStats->latest.data[i]);
//				printw(char *fmt, ...)
			}
//			mvprintw(row, latestCntCol, formatStringLatestCnt, idStats->data.size());

			if (idStats->highlight) {
				attroff(A_STANDOUT);
			}

			row++;
		}
	} else {
		attron(A_STANDOUT);
		mvaddch(titleRow, reverseColumn, ACS_DARROW);
		attroff(A_STANDOUT);
		std::vector<IdInfo*>::iterator it;
		std::vector<IdInfo*>::iterator end;
		it = canFrameStats.canStatsSorted.begin();
		end = canFrameStats.canStatsSorted.end();
		for ( ; it != end; it++) {
			//unsigned int messageId = it->;
			IdInfo* idStats = *it;
			if (idStats->highlight) {
				attron(A_STANDOUT);
			}

			//mvprintw(row++, col, "%9d  %5d  %10d  ", idStats->ID, idStats->count, idStats->data.size());
			mvprintw(row, msgIdCol, formatStringMsgId, idStats->ID);
			mvprintw(row, msgBusCol, formatStringMsgBus, idStats->bus);
			mvprintw(row, msgCntCol, formatStringMsgCnt, idStats->count);
			mvprintw(row, msgRateCol, formatStringMsgRate, idStats->currentRate);
			mvprintw(row, uniCntCol, formatStringUniCnt, idStats->data.size());
			
			move(row, latestCntCol);
			for (int i = idStats->latest.dataLength-1; i >= 0; i--) {
				printw("%02X", idStats->latest.data[i]);
//				printw(char *fmt, ...)
			}
//			mvprintw(row, latestCntCol, formatStringLatestCnt, idStats->data.size());

			if (idStats->highlight) {
				attroff(A_STANDOUT);
			}

			row++;
		}
	}


	miniWindow(canY, canY+canHeight, canX, latestCntCol + latestCntLength + 2);


	move(fieldY, fieldX+highlightCurrentIndex);

}

char CursesHandler::getUserInput( )
{
	int result = getch();
	switch (result) {
		case 'g':
		case 'G':
			if (displayMode == CAN) {
				displayMode = GPS;
				curs_set(0);	// no cursor
			} else {
				displayMode = CAN;
				curs_set(1);	// yes cursor
			}
			break;


		case KEY_LEFT:
			if (canSortMode != SORT_ID) {
				canSortMode = (CanSortMode) (canSortMode-1);
			}
			highlightCurrent = false;
			break;
		case KEY_RIGHT:
			if (canSortMode != SORT_COUNT-1) {
				canSortMode = (CanSortMode) (canSortMode+1);
			}
			highlightCurrent = false;
			break;

		case KEY_UP:
			reverseSortEnable = 1;
			break;
		case KEY_DOWN:
			reverseSortEnable = 0;
			break;
		case 'm':
		case 'M':
			if (canSortMode == SORT_ID) {
				reverseSortEnable = !reverseSortEnable;
			}
			canSortMode = SORT_ID;
			break;

		case 'c':
		case 'C':
			if (canSortMode == SORT_ID_COUNT) {
				reverseSortEnable = !reverseSortEnable;
			}
			canSortMode = SORT_ID_COUNT;
			break;

		case 'r':
		case 'R':
			if (canSortMode == SORT_ID_RATE) {
				reverseSortEnable = !reverseSortEnable;
			}
			canSortMode = SORT_ID_RATE;
			break;

		case 'u':
		case 'U':
			if (canSortMode == SORT_UNIQUE_DATA_COUNT) {
				reverseSortEnable = !reverseSortEnable;
			}
			canSortMode = SORT_UNIQUE_DATA_COUNT;
			break;
			
		case 'b':
		case 'B':
			if (canSortMode == SORT_ID_BUS) {
				reverseSortEnable = !reverseSortEnable;
			}
			canSortMode = SORT_ID_BUS;
			break;
			
		case 'x':
		case 'X':
			messageIdAsHex = !messageIdAsHex;
			break;

		case '-':
		case '_':
			resetUniqueData = true;
			break;

		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
//			highlight = result - '0';
				if (highlightCurrentIndex < sizeof(highlightField)-1) {
					highlightField[highlightCurrentIndex++] = result;
				}
				highlightCurrent = false;
			break;

		case KEY_BACKSPACE:
			highlightCurrentIndex--;
			if (highlightCurrentIndex <= 0) {
				highlightCurrentIndex = 0;
			}
			highlightField[highlightCurrentIndex] = '\0';
			highlightCurrent = false;
			break;

		case KEY_ENTER:
		case '\r':
		case '\n':
			highlightEnter = true;
			highlightCurrent = true;
			break;

		default:
			break;
	}
	return result;
}


void CursesHandler::setupTerminal(  )
{
	// Start up Curses window
	initscr();
	cbreak();
	noecho();
	nodelay(stdscr, 1);	// Don't wait at the getch() function if the user hasn't hit a key
	keypad(stdscr, 1); // Allow Function key input and arrow key input

	start_color();
	init_pair(1, COLOR_RED, COLOR_BLACK);
	init_pair(2, COLOR_GREEN, COLOR_BLACK);
	init_pair(3, COLOR_CYAN, COLOR_BLACK);
	init_pair(4, COLOR_BLUE, COLOR_WHITE);

	init_pair(5, COLOR_BLACK, COLOR_RED );
	init_pair(6, COLOR_BLACK, COLOR_GREEN );
	init_pair(7, COLOR_BLACK, COLOR_CYAN );
	init_pair(8, COLOR_WHITE, COLOR_BLUE );

	curs_set(0);	// no cursor

	atexit(destroy);
}

void CursesHandler::cleanupConsole( )
{
	clear();
	endwin();

	std::cout << "Console has been cleaned!" << std::endl;
}
