/*
 Author: Matt Bunting
 */

#include <iostream>
#include <signal.h>
#include <unistd.h>	// usleep()
#include <time.h>

#include "panda.h"

// A ctrl-c handler for safe panda handler destruction
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}


int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;

	//Set up graceful exit
	signal(SIGINT, killPanda);
	
	Panda::GpioHandler mGpioHandler;
	mGpioHandler.open(GPIO_PIN_BUZZER);
	mGpioHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	

	int count = 0;
	std::cout << "Each \'.\' represents a 0.02s buzzer attempt." << std::endl;
	while (keepRunning == true ) {
		std::cerr << ".";
		mGpioHandler.setGpioOutput(1);
		usleep(20000);
		mGpioHandler.setGpioOutput(0);
		usleep(500000 - 20000);
		
		if(count++ >= 2) {
			keepRunning = false;
		}
	}
	
	
	std::cout << "Done." << std::endl;

	return 0;
}
