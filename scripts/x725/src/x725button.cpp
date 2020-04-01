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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "rpigpio.h"

int main(int argc, char **argv)
{
	fprintf(stderr,"Running %s\n", argv[0]);

	int g;

	// Set up gpi pointer for direct register access
	GpioHandler *gpio = setup_io();

	// Switch GPIO 7..11 to output mode

	/************************************************************************\
	 * You are about to change the GPIO settings of your computer.          *
	 * Mess this up and it will stop working!                               *
	 * It might be a good idea to 'sync' before running this program        *
	 * so at least you still have your code changes written to the SD-card! *
	 \************************************************************************/

	// Set GPIO pins 7-11 to output
	for (g=7; g<=11; g++)
	{
		setOutput( gpio, g );
	}

	setOutput( gpio, 17 ); // BOOT
	setGpio(gpio, 1<<17);



	setInput( gpio, 4);	// x725 power button
	bool onMode = true;
	bool currentlyTimingButton = false;
	double totalTime = 0;

	double buttonDelayMs = 0.2;

	while(1) {
		usleep(1000000.0 * buttonDelayMs);

		if (readButton(gpio, 4)) {
			if (currentlyTimingButton == false) {
				currentlyTimingButton = true;
				fprintf(stderr,"Button Pressed!\n");
				buttonDelayMs = 0.02;
			}
			totalTime += buttonDelayMs;
			if (totalTime >= 0.600) {
				fprintf(stderr,"Shutting system down now...");
				system("poweroff");
				exit(0);
			}
		} else {
			if (currentlyTimingButton == true) {
				fprintf(stderr,"Button was released!  Time pressed: %0.2f\n", totalTime);
				currentlyTimingButton = false;
				buttonDelayMs = 0.2;

				if ((totalTime >= 0.400) && (totalTime <= 0.600)) {
					fprintf(stderr,"System rebooting now...");
					system("reboot");
					exit(0);
				}

				totalTime = 0;
			}
		}

		// Status in HW LEDs:
		if (onMode) {
			if(g <= 11) {
				setGpio(gpio, 1<<g++);
			} else {
				g = 7;
				onMode = false;
			}
		} else {
			if(g <= 11) {
				clearGpio(gpio, 1<<g++);
			} else {
				g = 7;
				onMode = true;
			}
		}
	}

	return 0;

} // main
