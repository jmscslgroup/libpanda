/*
 Author: Matt Bunting
 */

#include <iostream>
#include <unistd.h>	// sleep()
#include <time.h>

#include "panda.h"


int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;

	
	Panda::GpioHandler resetHandler;
	resetHandler.open(GPIO_PIN_PANDA_RESET);
	resetHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	
	
	Panda::GpioHandler bootHandler;
	bootHandler.open(GPIO_PIN_PANDA_BOOT);
	bootHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	
	
	usleep(500000);
	std::cout << "Entering DFU mode:" << std::endl;
	std::cout << " - Pressing Reset and Boot" << std::endl;
	resetHandler.setGpioOutput(GPIO_PIN_PANDA_BOOT_PRESSED);
	bootHandler.setGpioOutput(GPIO_PIN_PANDA_RESET_PRESSED);
	sleep(1);
	
	
	std::cout << " - Releasing Reset with Boot still held" << std::endl;
//	resetHandler.setGpioOutput(GPIO_PIN_PANDA_BOOT_PRESSED);
	bootHandler.setGpioOutput(GPIO_PIN_PANDA_RESET_RELEASED);
	sleep(1);
	
	
	std::cout << "Entering application mode:" << std::endl;
	std::cout << " - Pressing Reset and releasing Boot" << std::endl;
	resetHandler.setGpioOutput(GPIO_PIN_PANDA_BOOT_RELEASED);
	bootHandler.setGpioOutput(GPIO_PIN_PANDA_RESET_PRESSED);
	sleep(1);
	
	
	std::cout << " - Releasing Reset with Boot still released" << std::endl;
//	resetHandler.setGpioOutput(GPIO_PIN_PANDA_BOOT_RELEASED);
	bootHandler.setGpioOutput(GPIO_PIN_PANDA_RESET_RELEASED);
	sleep(1);
	
	std::cout << "Done." << std::endl;

	return 0;
}
