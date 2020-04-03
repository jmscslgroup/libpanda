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

// for i2c:
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>


#define x725_ADDRESS (0x36)


int i2c_write_byte( int i2cFile, unsigned char cmd, unsigned char value) {
	unsigned char buffer[2];

	// Two bytes must be written, first the command (register), then the value
	buffer[0] = cmd;
	buffer[1] = value;

	int length = 2;
	if( write(i2cFile, buffer, length) != length ) {
		fprintf(stderr, "Failed to write to the i2c\n");
	}

	return 0;
}

int i2c_read_word( int i2cFile, unsigned char cmd) {
	unsigned char buffer[2];

	// First we need to write a byte for the register to be read
	buffer[0] = cmd;
	int length = 1;
	if (write(i2cFile, buffer, length) != length) {
		fprintf(stderr, "Failed to write to i2c\n");
	}

	// Then we can begin reading from that register onward
	length = 2;
	if (read(i2cFile, buffer, length) != length) {
		fprintf(stderr, "Failed to read from i2c\n");
	}

	int result = (((int)buffer[0])<<8) + (int)buffer[1];

	return result;
}



int main(int argc, char **argv) {
	fprintf(stderr, "Running %s\n", argv[0]);


	// This opens the raspberry pi device file:
	char i2cFilename[] = "/dev/i2c-1";
	int i2cFile;
	if( (i2cFile = open(i2cFilename, O_RDWR)) < 0) {
		printf("Unable to open the i2c file");
		return -1;
	}

	if( ioctl(i2cFile, I2C_SLAVE, x725_ADDRESS) < 0) {
		printf("Unable ot talk to x725 i2c address\n");
		return -1;
	}

	int result;


	// The following two functions ensure that i2c is connected, otherwise continue and wait.
	//  This assumes that 4 total byes are unchanging regardless of the x725 state.
	//  It is unsure what these registers are for.  primitive tests were unable to write
	//  new values for these registers, meaning that they may
	result = i2c_read_word(i2cFile, 8);
	if (result != 2) {
		printf("%s: x725 NOT Avaiable\n", argv[0]);
		return EXIT_FAILURE;
	}

	result = i2c_read_word(i2cFile, 12);
	if (result != 38656) {
		printf("%s: x725 NOT Avaiable\n", argv[0]);
		return EXIT_FAILURE;
	}

	printf("%s: x725 Avaiable!\n", argv[0]);
	return EXIT_SUCCESS;
}

