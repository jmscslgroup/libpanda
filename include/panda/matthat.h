/*
 Author: Matt Bunting
 */

#ifndef MATTHAT_H
#define MATTHAT_H

#include "panda/buttonSequencer.h"
#include "mogi/thread.h"

namespace Panda {
class MatthatReset {
private:
	Panda::GpioHandler resetHandler;
	Panda::GpioHandler bootHandler;
	
public:
	MatthatReset();
	
	void doit();
};

class MatthatBeep : Mogi::Thread {
private:
	Panda::GpioHandler buzzerHandler;
	int beepCount;
	int onTime;
	int offTime;
	
	// Overlload from Mogi::Thread
	void entryAction();
	
public:
	MatthatBeep();
	~MatthatBeep();
	
	void singleBeep();
	void doubleBeep();
	void tripleBeep();
	
	void customBeep(int count, int onTimeMicroseconds, int offTimeMicroseconds);
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
