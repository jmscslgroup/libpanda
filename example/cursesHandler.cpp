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

//static int mainRunning = FALSE;
CursesHandler* CursesHandler::mCursesHandler = NULL;

CursesHandler::CursesHandler()
{
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


void CursesHandler::updateScreen( Panda::Handler& handler )
{
	int row = 0;
	int col = 0;


	clear();

	//mvprintw(row,31,"Verbose mode(press v for quiet)");

	//╬╝╚╔╦╩╠╣╗║
	mvprintw(row++,0,"+-------------------------+-------------------------------------------------------+-----------------------------------------------------+");
	mvprintw(row++,0,"|       Panda Menu        |                          GPS                          | Satellite                                           |");
	mvprintw(row++,0,"|    version:             |                                                       |    Map                                              |");
	mvprintw(row++,0,"|                         |                                                       |                                                     |");
	mvprintw(row++,0,"|                         |                                                       |                                                     |");
	mvprintw(row++,0,"|                         |                                                       |                                                     |");
	mvprintw(row++,0,"|                         |                                                       |                                                     |");
	mvprintw(row++,0,"| ESC or ctrl-c to Exit   |                                                       |                                                     |");
	mvprintw(row++,0,"|                         |                                                       |                                                     |");
	mvprintw(row++,0,"+-------------------------+--------------------------+----------------------------+                                                     |");
	mvprintw(row++,0,"|                                                    |         GPS Stats          |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"|                                                    |                            |                                                     |");
	mvprintw(row++,0,"+----------------------------------------------------+----------------------------+-----------------------------------------------------+");
	int rowSave = row+1;

	// GPS printing
	col = 2;
	row = 4;
	mvprintw(row,col,"USB mode: %s", handler.getUsb().getModeAsString());

	// GPS printing
	col = 29;
	row = 3;
	mvprintw(row++,col,"Time     : %02d:%02d:%02d.%.02d",
			 handler.getGps().getData().time.tm_hour,
			 handler.getGps().getData().time.tm_min,
			 handler.getGps().getData().time.tm_sec,
			 handler.getGps().getData().timeMilliseconds/10 );
	mvprintw(row++,col,"Date     : %02d/%02d/%04d",
			 handler.getGps().getData().time.tm_mon+1,
			 handler.getGps().getData().time.tm_mday,
			 handler.getGps().getData().time.tm_year+1900 );

	row = 12;
	col = 56;
	mvprintw(row++,col,"NMEA Message Count :%d", handler.getGps().getData().successfulParseCount);

	if (handler.getGps().getData().info.status == 'A') {
		row = 2;
		col = 29;
		mvprintw(row++,col,"Status   : Active" );
	} else {
		row = 2;
		col = 29;
		mvprintw(row++,col,"Status   : Inactive" );
	}

	row = 5;
	mvprintw(row++,col,"Longitude: %0.7f", handler.getGps().getData().pose.longitude );
	mvprintw(row++,col,"Latitude : %0.7f", handler.getGps().getData().pose.latitude );
	mvprintw(row++,col,"Altitude : %0.1f", handler.getGps().getData().pose.altitude );
	mvprintw(row++,col,"GeodSep  : %0.7f", handler.getGps().getData().pose.geoidalSeparation );

	row = 2;
	col = 60;
	mvprintw(row++,col,"Speed     : %0.3f", handler.getGps().getData().motion.speed );
	mvprintw(row++,col,"Vert Speed: %0.3f", handler.getGps().getData().motion.verticalSpeed );
	mvprintw(row++,col,"Course    : %0.3f", handler.getGps().getData().motion.course );
	mvprintw(row++,col,"HDOP      : %0.2f", handler.getGps().getData().quality.HDOP );
	mvprintw(row++,col,"PDOP      : %0.2f", handler.getGps().getData().quality.PDOP );
	mvprintw(row++,col,"VDOP      : %0.2f", handler.getGps().getData().quality.VDOP );

	// GPS other info:
	row = 13;
	col = 56;
	mvprintw(row++,col,"Quality Indicator  :%d", handler.getGps().getData().quality.indicator);
	mvprintw(row++,col,"Auto Mode          :%c", handler.getGps().getData().info.automode);
	mvprintw(row++,col,"Fix Type           :%d", handler.getGps().getData().info.fixType);
	mvprintw(row++,col,"Differential ID    :%d", handler.getGps().getData().info.diffId);
	mvprintw(row++,col,"Differential ID    :%0.2f", handler.getGps().getData().info.diffAge);
	mvprintw(row++,col,"GGAcount           :%d", handler.getGps().getData().info.GGAcount);
	mvprintw(row++,col,"Magnetic Variation :%0.2f", handler.getGps().getData().magneticVariation);


	mvprintw(10,5,"GPS Satellites: In view:%d  In use:%d", handler.getGps().getData().satelitesInView, handler.getGps().getData().satelitesInUse);
	row = 12;
	col = 2;
	mvprintw(row,4,"ID");
	mvprintw(row,10,"Tracking");
	mvprintw(row,22,"Azimuth");
	mvprintw(row,34,"Elevation");
	mvprintw(row++,47,"SNR");
	for (std::map<int,Panda::GpsSatellite>::const_iterator it = handler.getGps().getData().satellites.begin(); it != handler.getGps().getData().satellites.end(); it++) {
		mvprintw(row,4,"%02d", it->first);
		mvprintw(row,12,it->second.visible ? "true" : "false");
		mvprintw(row,23,"%0.1f", it->second.azimuth);
		mvprintw(row,37,"%0.1f", it->second.elevation);
		int colorPair = (it->second.SNR / 17)+1;
		colorPair = colorPair > 4 ? 4 : colorPair;
		attron(COLOR_PAIR(colorPair));
		mvprintw(row++,48,"%02d", it->second.SNR);
		attroff(COLOR_PAIR(colorPair));

	}


	// Satellite map just for kicks!
	double centerx = 109.5;//109-13+45*2.0+0.5;//109.5;
	double centery = 14.5;//45+0.5;//13.5;
	double radius = 13;//45;//13;	// row radius
	double xyscale = 2.0; // since column space is stretched relative to row space
	for (int i = -radius*xyscale; i < radius*xyscale; i++) {
		mvprintw(centery,centerx+i,"-");
	}
	for (int i = -radius; i < radius; i++) {
		mvprintw(centery+i,centerx,"|");
	}
	mvprintw(centery,centerx,"+");


	// draw title:

	// draw a circle:
	for (int i = 0; i < 360; i+= 10) {
		double radian = 3.14159/180.0*(double)i;
		int x = centerx + radius*sin(radian)*xyscale;
		int y = centery - radius*cos(radian);
		mvprintw( y, x, "*" );
	}
	// print scale:
	for (int i = 0; i < 360; i+= 30) {
		double radian = 3.14159/180.0*(double)i;
		int x = centerx + radius*sin(radian)*xyscale;
		int y = centery - radius*cos(radian);
		if (i == 0) {
			mvprintw( y, x, "N" );
		} else if(i == 90) {
			mvprintw( y, x, "E" );
		} else if(i == 180) {
			mvprintw( y, x, "S" );
		} else if(i == 270) {
			mvprintw( y, x, "W" );
		} else {
			mvprintw( y, x-1, "%d", i );
		}
	}
	// print satellite data:


	for (std::map<int,Panda::GpsSatellite>::const_iterator it = handler.getGps().getData().satellites.begin(); it != handler.getGps().getData().satellites.end(); it++) {
		if (it->second.visible) {
			//				double satRadius = sin((90.0-(double)it->second.elevation)*3.14159/180.0) * (radius-1);
			double satRadius = (90.0-(double)it->second.elevation)/90.0 * (radius-1);
			double radian = 3.14159/180.0*(double)it->second.azimuth;
			int x = centerx + satRadius*sin(radian)*xyscale;
			int y = centery - satRadius*cos(radian);

			int colorPair = (it->second.SNR / 17)+1;
			colorPair = colorPair > 4 ? 4 : colorPair;
			attron(COLOR_PAIR(colorPair+4));
			mvprintw( y, x, "%d", it->second.ID );
			attroff(COLOR_PAIR(colorPair+4));
		}
	}





	mvprintw(rowSave++,0,"Console:");
	move(rowSave++,0);
	
	refresh();
}


char CursesHandler::getUserInput( )
{
	return getch();
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

	atexit(destroy);
}

void CursesHandler::cleanupConsole( )
{
	clear();
	endwin();

	std::cout << "Console has been cleaned!" << std::endl;
}
