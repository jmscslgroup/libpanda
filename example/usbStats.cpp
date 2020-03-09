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

#include "usbStats.h"

#include <unistd.h>

UsbStats::UsbStats() {
}
UsbStats::~UsbStats() {
}

float UsbStats::getUartSuccess() {
	int total = (runningIndexUart < 1000 ? runningIndexUart : 1000);
	int numberOfSuccess = 0;
	for (int i = 0; i < total; i++) {
		numberOfSuccess += (int)successulReceivesUart[i];
	}
	return 100.0*(float)numberOfSuccess/(float)total;
}
float UsbStats::getCanSuccess() {
	int total = (runningIndexCan < 1000 ? runningIndexCan : 1000);
	int numberOfSuccess = 0;
	for (int i = 0; i < total; i++) {
		numberOfSuccess += (int)successulReceivesCan[i];
	}
	return 100.0*(float)numberOfSuccess/(float)total;
}

void UsbStats::notificationUartRead(char* buffer, size_t bufferLength) {
	successulReceivesUart[runningIndexUart++ % 1000] = bufferLength > 0;
}

void UsbStats::notificationCanRead(char* buffer, size_t bufferLength) {
	successulReceivesCan[runningIndexCan++ % 1000] = bufferLength > 0;
}

void UsbStats::doAction() {
	pause();
	usleep(1000);
}

