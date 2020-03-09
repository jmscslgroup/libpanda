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

#ifndef __PANDA_USBSTATS_H
#define __PANDA_USBSTATS_H

#include "panda.h"
#include "mogi/thread.h"


class UsbStats : public Panda::UsbListener, public Mogi::Thread {
public:
	UsbStats();
	~UsbStats();

	float getUartSuccess();
	float getCanSuccess();

private:

	bool successulReceivesUart[1000];
	bool successulReceivesCan[1000];
	int runningIndexUart = 0;
	int runningIndexCan = 0;

	// overloads from Panda::UsbListener
	void notificationUartRead(char* buffer, size_t bufferLength);
	void notificationCanRead(char* buffer, size_t bufferLength);

	// overload from Mogi::Thread
	void doAction();

};
#endif
