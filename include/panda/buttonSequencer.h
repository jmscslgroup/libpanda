/*
 Author: Matt Bunting
 */

#ifndef BUTTON_SEQUENCER_H
#define BUTTON_SEQUENCER_H

#include <vector>


#define RESISTANCE_POTENTIOMETER (10000)	

#define RESISTANCE_CRUISE_ON (0)	// 0V
#define RESISTANCE_CANCEL (315)		// 1.17V
#define RESISTANCE_DISTANCE (867)	// 2.11V
#define RESISTANCE_SET (1199.6)		// 2.89V
//#define RESISTANCE_RES (4700+5095)	// 3.56V
#define RESISTANCE_RES (4700)	// 3.56V

// RES: 3.63V from switch, 3.64V from circuit 	-> 4.77k ohm
// SET: 2.94V from switch, 2.90V from circuit 	-> 1.75k ohm

#define TIME_PER_BUTTON_PRESS ((unsigned int)(200000))	// in us
#define TIME_PER_BUTTON_RELEASE ((unsigned int)(200000))	// in us
#define TIME_PER_POTENTIOMETER_SET ((unsigned int)(100000))	// in us


namespace Panda {

enum NissanButton {
	NISSAN_BUTTON_CRUISE_ON,
	NISSAN_BUTTON_CANCEL,
	NISSAN_BUTTON_DISTANCE,
	NISSAN_BUTTON_SET,
	NISSAN_BUTTON_RES
};

enum ButtonEventType {
	BUTTON_TYPE_GPIO,
	BUTTON_TYPE_GPIO_D20,
	BUTTON_TYPE_POTENTIOMETER,
	BUTTON_TYPE_DELAY
};

typedef struct _ButtonEvent {
	ButtonEventType type;
	unsigned int data;
	unsigned int time;
} ButtonEvent;


const char* nissanButtonToStr(NissanButton button);

class ButtonSequence {
	
private:
	int file_i2c;
	int file_gpio;
	int file_gpio_d20;
	
	int tickTime;
	std::vector<ButtonEvent> events;
	
	void setGpio( bool value );
	void setGpioD20( bool value );
	void setDigialPotentiometer( unsigned int value );
	void applyEvent(ButtonEvent& event);
	
	
public:
	ButtonSequence();
	~ButtonSequence();
	
	// builds the sequence:
	void addButtonPress( NissanButton button );
	void addButtonHold( NissanButton button );
	void addButtonRelease();
	void addTimeDelay( unsigned int timeInMilliseconds );
	
	// Sending an resetting:
	void send();	// sends the built sequence, this is a blocking command
	void reset();
};

}

#endif
