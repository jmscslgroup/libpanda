/*
 Author: Matt Bunting
 */

#include <iostream>
#include <unistd.h>	// sleep()
#include <time.h>

#include "panda.h"


int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;

	Panda::MatthatReset mMatthatReset;
//
//	Panda::GpioHandler resetHandler;
//	resetHandler.open(GPIO_PIN_PANDA_RESET);
//	resetHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
//
//
//	Panda::GpioHandler bootHandler;
//	bootHandler.open(GPIO_PIN_PANDA_BOOT);
//	bootHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
//
	
	usleep(500000);
	mMatthatReset.doit();
	sleep(1);
	
	std::cout << "Done." << std::endl;

	return 0;
}
