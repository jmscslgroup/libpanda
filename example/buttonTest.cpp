/*
 Author: Matt Bunting
 */

#include <iostream>
#include <unistd.h>

#include "panda.h"

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
	
	int i = 0;
	while(true) {
		buttonSequencer.reset();
		
		
//		buttons[i];
		std::cout << "Sending " << Panda::nissanButtonToStr(buttons[i]) << std::endl;
//		buttonSequencer.addButtonHold(buttons[i]);
		buttonSequencer.addButtonPress(buttons[i]);
		
		buttonSequencer.send();
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
