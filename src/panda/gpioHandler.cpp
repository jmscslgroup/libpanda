/*
 Author: Matt Bunting
 */

#include "panda/buttonSequencer.h"

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace Panda;

GpioHandler::GpioHandler() {
	file = -1;
}

GpioHandler::~GpioHandler() {
	this->close();
}


bool GpioHandler::open( int pinNumber ) {
	pinStringLength = sprintf(pinString, "%d", pinNumber);
	sprintf(directionString, "/sys/class/gpio/gpio%d/direction", pinNumber);
	sprintf(valueString, "/sys/class/gpio/gpio%d/value", pinNumber);
	
	int fd = ::open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Error: GpioHandler::open(): Unable to open /sys/class/gpio/export\n");
		//	return 1;
	} else {
		
		if (write(fd, pinString, pinStringLength) != pinStringLength) {
			fprintf(stderr, "Error: GpioHandler::open(): Error writing \"%s\" to /sys/class/gpio/export\n", pinString);
			//::close(fd);
			//return 1;
		}
		::close(fd);
	}
	
	// default to input:
	setDirection( GPIO_DIRECTION_IN );
	
	file = ::open(valueString, O_WRONLY);
	if (file == -1) {
		fprintf(stderr, "Error: GpioHandler::open(): Unable to open %s\n", valueString);
		//		exit(1);
		return 1;
	}
	return 0;
}

bool GpioHandler::close( ) {
	if(file < 0) {
		//fprintf(stderr, "Warning: GpioHandler::close(): Gpio %s was never open!\n", pinString);
//		return 1;
	} else {
		::close(file);
		file = -1;
	}
	
	setDirection( GPIO_DIRECTION_IN );	// Juuust incase
	
	
	
	int fd = ::open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Error: GpioHandler::close(): Unable to open /sys/class/gpio/unexport\n");
		return 1;
	}
	
	if (write(fd, pinString, pinStringLength) != pinStringLength) {
		fprintf(stderr, "Error: GpioHandler::close(): Unable to open /sys/class/gpio/unexport\n");
//		::close(fd);
	} else {
		fprintf(stderr, "Info: GpioHandler::close(): \"%s\" written to /sys/class/gpio/unexport\n", pinString);
	}
	
	
	::close(fd);
	
	
	
	return 0;
}



void GpioHandler::setDirection( GpioHandler::GpioDirection enable ) {
	// Set the pin to be an output by writing "out" to /sys/class/gpio/gpio21/direction

	int fd = ::open(directionString, O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Error: GpioHandler::setOutput(): Unable to open %s", directionString);
		return;
	}
	
	if(enable == GPIO_DIRECTION_OUT){
		if (write(fd, "out", 3) != 3) {
			fprintf(stderr, "Error: GpioHandler::setOutput(): Unable to write \"out\" to %s\n", directionString);
		}
	} else {
		if (write(fd, "in", 2) != 2) {
			fprintf(stderr, "Error: GpioHandler::setOutput(): Unable to write \"in\" to %s\n", directionString);
		}
	}
	
	::close(fd);
}
//
//void GpioHandler::setInput( bool enable ) {
//	setOutput( false );
//}


void GpioHandler::setGpioOutput( bool value ) {
	if(file < 0) {
		fprintf(stderr, "Error: GpioHandler::setGpio(): Attempting to set GPIO value when not initialized!\n");
		return;
	}
	
	if (value != 0) {
		if (write(file, "1", 1) != 1) {
			fprintf(stderr, "Error: GpioHandler::setGpio(): Error writing \"1\" to %s\n", valueString);
		}
	} else {
		if (write(file, "0", 1) != 1) {
			fprintf(stderr, "Error: GpioHandler::setGpio(): Error writing \"0\" to %s\n", valueString);
		}
	}
}




bool GpioHandler::read() {
	// unimplemented
	fprintf(stderr, "Error: GpioHandler::read(): This function has not yet been implemented\n");
	return 0;
}
