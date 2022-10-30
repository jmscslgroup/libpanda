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
	
//	Panda::GpioHandler mGpioHandler;
//	mGpioHandler.open(GPIO_PIN_BUZZER);
//	mGpioHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);

	Panda::MatthatBeep mMatthatBeep;

	int count = 0;
//	std::cout << "Each \'.\' represents a 0.02s buzzer attempt:" << std::endl;
	while (keepRunning == true ) {
//		std::cerr << ".";
		mMatthatBeep.singleBeep();
		std::cout << "Single Beep called" << std::endl;
		sleep(1);
		mMatthatBeep.doubleBeep();
		std::cout << "Double Beep called" << std::endl;
		sleep(1);
		mMatthatBeep.tripleBeep();
		std::cout << "Triple Beep called" << std::endl;
		sleep(1);
		
		if(count++ >= 0) {
			keepRunning = false;
		}
	}
	
	
	std::cout << std::endl << "Done." << std::endl;

	return 0;
}
