/*
 Author: Matt Bunting
 */

#include <iostream>
#include <unistd.h>


//#include <curses.h>

#include "panda.h"
#include "panda/buttonSequencer.h"

int main(int argc, char **argv) {
	
	Panda::ButtonSequence buttonSequencer;
	
	Panda::NissanButton buttons[] = {
		Panda::NISSAN_BUTTON_CRUISE_ON,
		Panda::NISSAN_BUTTON_CANCEL,
		Panda::NISSAN_BUTTON_DISTANCE,
		Panda::NISSAN_BUTTON_SET,
		Panda::NISSAN_BUTTON_RES
	};
	
	Panda::nissanButtonToStr(buttons[0]);
	
	Panda::NissanButton toSend;
	int i = 0;
	while(true) {
		
		
		if(argv[1][0] == 'r') {
			toSend = Panda::NISSAN_BUTTON_RES;
		} else if (argv[1][0] == 's') {
			toSend = Panda::NISSAN_BUTTON_SET;

		} else if (argv[1][0] == 'd') {
			toSend = Panda::NISSAN_BUTTON_DISTANCE;

		} else if (argv[1][0] == 'c') {
			toSend = Panda::NISSAN_BUTTON_CANCEL;
		} else if (argv[1][0] == 'o') {
			toSend = Panda::NISSAN_BUTTON_CRUISE_ON;
		} else {
			return 0;
		}
		buttonSequencer.reset();
	std::cout << "Sending " << Panda::nissanButtonToStr(toSend) << std::endl;
//		buttonSequencer.addButtonHold(buttons[i]);
	buttonSequencer.addButtonPress(toSend);

	buttonSequencer.send();
		return 0;

		
		
	buttonSequencer.reset();
//		buttons[i];
		std::cout << "Sending " << Panda::nissanButtonToStr(buttons[i]) << std::endl;
//		buttonSequencer.addButtonHold(buttons[i]);
		buttonSequencer.addButtonPress(buttons[i]);

		buttonSequencer.send();
		sleep(1);
		
		
		
//		sleep();
		
		
//		buttonSequencer.reset();
//		buttonSequencer.addButtonRelease();
//		buttonSequencer.send();
//		sleep(1);
		
		
		i++;
		if(i >= sizeof(buttons)/sizeof(buttons[0])) {
			i = 0;
		}
	}
	
	return 0;
}
