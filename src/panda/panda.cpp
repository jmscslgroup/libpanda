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

#include "panda.h"

#include <iostream>

using namespace Panda;

Handler::Handler() {
	mCan.setUsb(&mUsb);
	mGps.setUsb(&mUsb);
	
	mUsb.addObserver(&mCan);
	mUsb.addObserver(&mGps);
}

Handler::~Handler() {
	//stop();	// just to be safe
}

void Handler::initialize() {
	mUsb.initialize();
	mCan.initialize();
	mGps.initialize();

	mUsb.startRecording();
	mGps.startParsing();
	mCan.startParsing();
}

void Handler::stop() {
	std::cout << "Stopping GPS...";
	mGps.stopParsing();
	std::cout << "Done." << std::endl;
	std::cout << "Stopping CAN...";
	mCan.stopParsing();
	std::cout << "Done." << std::endl;
	std::cout << "Stopping USB...";
	mUsb.stopRecording();
	std::cout << "Done." << std::endl;
}

void Handler::addGpsObserver(GpsListener& gps) {
	mGps.addObserver(&gps);
}

void Handler::addCanObserver(CanListener& can) {
	mCan.addObserver(&can);
}

void Handler::addUsbObserver(UsbListener& usb) {
	mUsb.addObserver(&usb);
}

Gps& Handler::getGps() {
	return mGps;
}

Can& Handler::getCan() {
	return mCan;
}

Usb& Handler::getUsb() {
	return mUsb;
}

