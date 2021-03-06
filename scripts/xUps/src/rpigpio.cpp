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

#include "rpigpio.h"


void setOutput(GpioHandler* gpio, int g) {
	INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
	OUT_GPIO(g);
}
void setInput(GpioHandler* gpio, int g) {
	INP_GPIO(g);
}

void setGpioPullup(GpioHandler* gpio, int g) {

}

void setGpio(GpioHandler* gpio, int g) {
	GPIO_SET = g;
}
void clearGpio(GpioHandler* gpio, int g) {
	GPIO_CLR = g;
}

bool readButton(GpioHandler* gpio, int g)
{
	if (GET_GPIO(g)) // !=0 <-> bit is 1 <- port is HIGH=3.3V
		return true;
	return false;
}


//
// Set up a memory regions to access GPIO
//
GpioHandler* setup_io()
{
	GpioHandler* gpio;
	int  mem_fd;
	void *gpio_map;

	/* open /dev/mem */
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
		printf("can't open /dev/mem \n");
		exit(-1);
	}

	/* mmap GPIO */
	gpio_map = mmap(
					NULL,             //Any adddress in our space will do
					BLOCK_SIZE,       //Map length
					PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
					MAP_SHARED,       //Shared with other processes
					mem_fd,           //File to map
					GPIO_BASE         //Offset to GPIO peripheral
					);

	close(mem_fd); //No need to keep mem_fd open after mmap

	if (gpio_map == MAP_FAILED) {
		printf("mmap error %ld\n", (long int)gpio_map);//errno also set!
		exit(-1);
	}

	// Always use volatile pointer!
	gpio = (GpioHandler *)gpio_map;

	return gpio;
} // setup_io
