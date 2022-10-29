/*
 Author: Matt Bunting
 */

#ifndef MATTHAT_H
#define MATTHAT_H

#include "panda/buttonSequencer.h"

class MatthatReset {
private:
	Panda::GpioHandler resetHandler;
	Panda::GpioHandler bootHandler;
	
public:
	MatthatReset();
	
	void doit();
};

#endif
