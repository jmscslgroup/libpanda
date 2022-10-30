/*
 Author: Matt Bunting
 */

#include <iostream>
#include <unistd.h>	// sleep()

#include "panda/matthat.h"

using namespace Panda;

MatthatReset::MatthatReset() {
	
	resetHandler.open(GPIO_PIN_PANDA_RESET);
	resetHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	
	bootHandler.open(GPIO_PIN_PANDA_BOOT);
	bootHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
}


void MatthatReset::doit() {
	
	resetHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	bootHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	
	std::cout << "MatthatReset: Entering DFU mode:" << std::endl;
	std::cout << "MatthatReset:  - Pressing Reset and Boot" << std::endl;
	resetHandler.setGpioOutput(GPIO_PIN_PANDA_RESET_PRESSED);
	bootHandler.setGpioOutput(GPIO_PIN_PANDA_BOOT_PRESSED);
	usleep(500000);
	
	
	std::cout << "MatthatReset:  - Releasing Reset with Boot still held" << std::endl;
	resetHandler.setGpioOutput(GPIO_PIN_PANDA_RESET_RELEASED);
	usleep(500000);
	
	
	std::cout << "MatthatReset: Entering application mode:" << std::endl;
	std::cout << "MatthatReset:  - Pressing Reset and releasing Boot" << std::endl;
	resetHandler.setGpioOutput(GPIO_PIN_PANDA_RESET_PRESSED);
	bootHandler.setGpioOutput(GPIO_PIN_PANDA_BOOT_RELEASED);
	usleep(500000);
	
	
	std::cout << "MatthatReset:  - Releasing Reset with Boot still released" << std::endl;
	resetHandler.setGpioOutput(GPIO_PIN_PANDA_RESET_RELEASED);
	std::cout << "MatthatReset:  - Waiting 1 second for Panda to start" << std::endl;
	sleep(1);
	
	
	resetHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_IN);
	bootHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_IN);
	
}



MatthatBeep::MatthatBeep() {
	buzzerHandler.open(GPIO_PIN_BUZZER);
	buzzerHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_OFF);
}

MatthatBeep::~MatthatBeep() {
	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_OFF);
//	buzzerHandler.close();
}

void MatthatBeep::entryAction() {
	for (int i = 0; i < beepCount; i++) {
		buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_ON);
		std::cout << "Beep!" << std::endl;
		usleep(onTime);	// 1/20 s
		buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_OFF);
		usleep(offTime);
	}
}

void MatthatBeep::singleBeep() {
//	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_ON);
//	usleep(50000);	// 1/20 s
//	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_OFF);
	beepCount = 1;
	onTime = 50000;
	offTime = 0;
	start();
}

void MatthatBeep::doubleBeep() {
	beepCount = 2;
	onTime = 50000;
	offTime = 100000;
	start();
}

void MatthatBeep::tripleBeep() {
	beepCount = 3;
	onTime = 60000;
	offTime = 200000;
	start();
}

void MatthatBeep::customBeep(int count, int onTimeMicroseconds, int offTimeMicroseconds) {
	beepCount = count;
	onTime = onTimeMicroseconds;
	offTime = offTimeMicroseconds;
	start();
}


MatthatAccButtonRelay::MatthatAccButtonRelay() {
	relayHandler.open(GPIO_PIN_RELAY_POTENTIOMETER);
	relayHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	disarm();
}

MatthatAccButtonRelay::~MatthatAccButtonRelay() {
	disarm();
//	relayHandler.close();
}

void MatthatAccButtonRelay::arm() {
	relayHandler.setGpioOutput(GPIO_PIN_RELAY_POTENTIOMETER_ARMED);
}

void MatthatAccButtonRelay::disarm() {
	relayHandler.setGpioOutput(GPIO_PIN_RELAY_POTENTIOMETER_DISARMED);
}


