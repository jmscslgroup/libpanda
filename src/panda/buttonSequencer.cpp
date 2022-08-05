/*
 Author: Matt Bunting
 */

#include "panda/buttonSequencer.h"

#include <unistd.h>
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <cstdio>

using namespace Panda;


const char* Panda::nissanButtonToStr(NissanButton button) {
	switch (button) {
		case NISSAN_BUTTON_CRUISE_ON:
			return "RESISTANCE_CRUISE_ON";
		case NISSAN_BUTTON_SET:
			return "RESISTANCE_SET";
		case NISSAN_BUTTON_RES:
			return "RESISTANCE_RES";
		case NISSAN_BUTTON_CANCEL:
			return "RESISTANCE_CANCEL";
		case NISSAN_BUTTON_DISTANCE:
			return "RESISTANCE_DISTANCE";
	}
	
	return NULL;
}

unsigned int nissanButtonToDigitalPot( NissanButton& button ) {
	switch (button) {
		case NISSAN_BUTTON_CRUISE_ON:
			return (127*RESISTANCE_CRUISE_ON)/RESISTANCE_POTENTIOMETER;
		case NISSAN_BUTTON_SET:
			return (127*RESISTANCE_SET)/RESISTANCE_POTENTIOMETER;
		case NISSAN_BUTTON_RES:
			return (127*RESISTANCE_RES)/RESISTANCE_POTENTIOMETER;
		case NISSAN_BUTTON_CANCEL:
			return (127*RESISTANCE_CANCEL)/RESISTANCE_POTENTIOMETER;
		case NISSAN_BUTTON_DISTANCE:
			return (127*RESISTANCE_DISTANCE)/RESISTANCE_POTENTIOMETER;
	}
	
	return 63;
}

void initializeRelayGpio() {
	int fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/export");
		//			exit(1);
	}
	
	if (write(fd, "21", 2) != 2) {
		perror("Error writing to /sys/class/gpio/export");
		//			exit(1);
	}
	
	close(fd);
	
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/export");
		//			exit(1);
	}
	
	if (write(fd, "20", 2) != 2) {
		perror("Error writing to /sys/class/gpio/export");
		//			exit(1);
	}
	
	close(fd);
	
	// Set the pin to be an output by writing "out" to /sys/class/gpio/gpio21/direction
	
	fd = open("/sys/class/gpio/gpio21/direction", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/gpio21/direction");
		//			exit(1);
	}
	
	if (write(fd, "out", 3) != 3) {
		perror("Error writing to /sys/class/gpio/gpio21/direction");
		//			exit(1);
	}
	
	close(fd);
	
	fd = open("/sys/class/gpio/gpio20/direction", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/gpio20/direction");
		//			exit(1);
	}
	
	if (write(fd, "out", 3) != 3) {
		perror("Error writing to /sys/class/gpio/gpio20/direction");
		//			exit(1);
	}
	
	close(fd);
}

void closeRelayGpio() {
	int fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/unexport");
		//		exit(1);
	}
	
	if (write(fd, "21", 2) != 2) {
		perror("Error writing to /sys/class/gpio/unexport");
		//		exit(1);
	}
	
	
	
	close(fd);
	
	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/unexport");
		//		exit(1);
	}
	
	if (write(fd, "20", 2) != 2) {
		perror("Error writing to /sys/class/gpio/unexport");
		//		exit(1);
	}
	
	
	
	close(fd);
}

ButtonSequence::ButtonSequence() {
	tickTime = 1000;
	
	if ((file_i2c = open("/dev/i2c-1", O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		//		return;
	}
	
	int addr = 0x28;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}
	
	initializeRelayGpio();
	
	file_gpio = open("/sys/class/gpio/gpio21/value", O_WRONLY);
	if (file_gpio == -1) {
		perror("Unable to open /sys/class/gpio/gpio21/value");
		//		exit(1);
	}
	
	file_gpio_d20 = open("/sys/class/gpio/gpio20/value", O_WRONLY);
	if (file_gpio_d20 == -1) {
		perror("Unable to open /sys/class/gpio/gpio20/value");
		//		exit(1);
	}
	
	setGpio(0);
	setGpioD20(0);
}


ButtonSequence::~ButtonSequence() {
	closeRelayGpio();
}

void ButtonSequence::addButtonPress( NissanButton button ) {
	unsigned int resistance = nissanButtonToDigitalPot(button);
	events.push_back({BUTTON_TYPE_POTENTIOMETER, resistance, TIME_PER_POTENTIOMETER_SET});
	
	if (button == NISSAN_BUTTON_RES) {
		events.push_back( { BUTTON_TYPE_GPIO, 1, TIME_PER_BUTTON_PRESS + TIME_PER_POTENTIOMETER_SET} );
	 //	addTimeDelay( 1000 );
			events.push_back( { BUTTON_TYPE_GPIO, 0, TIME_PER_BUTTON_RELEASE + TIME_PER_BUTTON_PRESS + TIME_PER_POTENTIOMETER_SET} );
	} else if (button == NISSAN_BUTTON_SET) {
		events.push_back( { BUTTON_TYPE_GPIO_D20, 1, TIME_PER_BUTTON_PRESS + TIME_PER_POTENTIOMETER_SET} );
	 //	addTimeDelay( 1000 );
			events.push_back( { BUTTON_TYPE_GPIO_D20, 0, TIME_PER_BUTTON_RELEASE + TIME_PER_BUTTON_PRESS + TIME_PER_POTENTIOMETER_SET} );
		
	} else {
		printf("Error! Button %s unsupported!\n", nissanButtonToStr(button));
	}
}

void ButtonSequence::addButtonHold( NissanButton button ) {
	unsigned int resistance = nissanButtonToDigitalPot(button);
	events.push_back({BUTTON_TYPE_POTENTIOMETER, resistance, TIME_PER_POTENTIOMETER_SET});
	events.push_back( { BUTTON_TYPE_GPIO, 1, 0} );
}

void ButtonSequence::addButtonRelease( ) {
	events.push_back( { BUTTON_TYPE_GPIO, 0, 0} );
}

void ButtonSequence::addTimeDelay( unsigned int timeInMilliseconds ) {
	events.push_back( {BUTTON_TYPE_DELAY, 0, timeInMilliseconds*1000} );
	
}

void ButtonSequence::setGpio( bool value ) {
	if (value != 0) {
		if (write(file_gpio, "1", 1) != 1) {
			perror("Error writing to /sys/class/gpio/gpio21/value");
			//			exit(1);
		}
	} else {
		if (write(file_gpio, "0", 1) != 1) {
			perror("Error writing to /sys/class/gpio/gpio21/value");
			//			exit(1);
		}
	}
	
}

void ButtonSequence::setGpioD20( bool value ) {
	if (value != 0) {
		if (write(file_gpio_d20, "1", 1) != 1) {
			perror("Error writing to /sys/class/gpio/gpio20/value");
			//			exit(1);
		}
	} else {
		if (write(file_gpio_d20, "0", 1) != 1) {
			perror("Error writing to /sys/class/gpio/gpio20/value");
			//			exit(1);
		}
	}
	
}

void ButtonSequence::setDigialPotentiometer( unsigned int value ) {
	unsigned char buffer[2];
	
	buffer[0] = 0x00;
	buffer[1] = (unsigned char)value;
	printf("Sending digital pot setting of %d\n", value);
	int length = 2;
	if (write(file_i2c, buffer, length) != length)
	{
		printf("Failed to set the digital potentimeter on the i2c bus.\n");
	}
	
}


void ButtonSequence::applyEvent(ButtonEvent& event) {
	switch (event.type) {
		case Panda::BUTTON_TYPE_GPIO:
			setGpio(event.data != 0);
			break;
			
		case Panda::BUTTON_TYPE_GPIO_D20:
			setGpioD20(event.data != 0);
			break;
			
		case Panda::BUTTON_TYPE_POTENTIOMETER:
			setDigialPotentiometer( event.data );
			break;
			
		case Panda::BUTTON_TYPE_DELAY:
			break;
	}
}

void ButtonSequence::send() {
	struct timeval startTime;
	gettimeofday(&startTime, NULL);
	
	struct timeval currentTime = startTime;
	long long runningTimeInMicroseconds = 0;
	
	bool done;
	
	for (std::vector<ButtonEvent>::iterator it = events.begin(); it != events.end(); it++) {
		ButtonEvent& event = (*it);
		
		done = false;
		
		while(!done) {
			// Send ASAP then check next event:
			if (event.time <= runningTimeInMicroseconds) {
				//				dualshock->applyEvent( &event );
				applyEvent(event);
				
				done = true;
				continue;
			}
			
			// Event not ready, wait then check new time:
			usleep(tickTime);
			
			gettimeofday(&currentTime, NULL);
			runningTimeInMicroseconds = (long long)(currentTime.tv_sec - startTime.tv_sec)*1000000 + (long long)(currentTime.tv_usec - startTime.tv_usec);
		}
		
	}
}


void ButtonSequence::reset() {
	events.clear();
}
