/*
 Author: Matt Bunting
 */

#ifndef MATTHAT_H
#define MATTHAT_H

#include "panda/buttonSequencer.h"

namespace Panda {
class MatthatReset {
private:
	Panda::GpioHandler resetHandler;
	Panda::GpioHandler bootHandler;
	
public:
	MatthatReset();
	
	void doit();
};

class MatthatBeep {
private:
	Panda::GpioHandler buzzerHandler;
	
public:
	MatthatBeep();
	~MatthatBeep();
	
	void singleBeep();
	void doubleBeep();
	void tripleBeep();
};

class MatthatAccButtonRelay {
private:
	Panda::GpioHandler relayHandler;
	
public:
	MatthatAccButtonRelay();
	~MatthatAccButtonRelay();
	
	void arm();
	void disarm();
};

}

#endif
