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
}

void MatthatBeep::singleBeep() {
	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_ON);
	usleep(20000);	// 1/50 s
	buzzerHandler.setGpioOutput(GPIO_PIN_BUZZER_OFF);
}

void MatthatBeep::doubleBeep() {
	singleBeep();
	usleep(500000);
	singleBeep();
}

void MatthatBeep::tripleBeep() {
	doubleBeep();
	usleep(500000);
	singleBeep();
}


MatthatAccButtonRelay::MatthatAccButtonRelay() {
	relayHandler.open(GPIO_PIN_RELAY_POTENTIOMETER);
	relayHandler.setDirection(Panda::GpioHandler::GPIO_DIRECTION_OUT);
	disarm();
}

MatthatAccButtonRelay::~MatthatAccButtonRelay() {
	disarm();
}

void MatthatAccButtonRelay::arm() {
	relayHandler.setGpioOutput(GPIO_PIN_RELAY_POTENTIOMETER_ARMED);
}

void MatthatAccButtonRelay::disarm() {
	relayHandler.setGpioOutput(GPIO_PIN_RELAY_POTENTIOMETER_DISARMED);
}


