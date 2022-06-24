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

#ifndef __PANDA_CURSES_H
#define __PANDA_CURSES_H

#include <ncurses.h>
#include <termios.h>

#include "panda.h"
#include "canFrameStats.h"
#include "usbStats.h"

class CursesHandler {
private:
	// Singleton:
	static CursesHandler* mCursesHandler;
	CursesHandler();
	~CursesHandler(){};

	//struct termios orig_termios;
	static int instanceCount;

	void cleanupConsole();

	typedef enum {
		CAN,
		GPS
	} DisplayMode;

	typedef enum _CanSortMode {
		SORT_ID = 0,
		SORT_ID_BUS = 1,
		SORT_ID_COUNT = 2,
		SORT_ID_RATE = 3,
		SORT_UNIQUE_DATA_COUNT = 4,
		SORT_COUNT = 5
	} CanSortMode;

	DisplayMode displayMode;
	CanSortMode canSortMode;
	volatile bool reverseSortEnable;
	
	bool messageIdAsHex = false;

	bool resetUniqueData;

	char highlightField[10];
	int highlightCurrentIndex = 0;
	bool highlightEnter = false;
	bool highlightCurrent = false;
	//int highlight = -1;
	
	double heading;	// This is heavily filtered

public:
	// Singleton:
	static CursesHandler* getInstance();

	static void destroy();

	void updateScreen( Panda::Handler& handler, CanFrameStats& canFrameStats, UsbStats& usbStats );

	void drawGps( Panda::Handler& handler );
	void drawCan( CanFrameStats& canFrameStats, UsbStats& usbStats );

	char getUserInput();

	void setupTerminal( );

};
#endif /* defined(__Project__danceCurses__) */
