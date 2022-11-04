/*
 Author: Matt Bunting
 */

#include "panda/buttonSequencer.h"

#include <unistd.h>
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
//#include <linux/i2c-dev.h>

#include <wiringPiSPI.h>

#include <cstdio>
#include <iostream>

using namespace std;
using namespace Panda;

static const int CHANNEL = 0;

unsigned short digitalPotBufferToRegister(unsigned char *buffer) {
	unsigned short result = buffer[0];
	return (result << 8) | buffer[1];
}

void digitalPotResistanceToBuffer(double resistance, unsigned char *buffer) {
	resistance = resistance > (10000+DIGI_POT_RW) ? (10000+DIGI_POT_RW) : resistance;
	resistance = resistance < DIGI_POT_RW ? DIGI_POT_RW : resistance;
	unsigned short setting = (resistance - DIGI_POT_RW)/10000.0 * 256;
	buffer[0] = (buffer[0] & 0xFC) | (setting >> 8);
	buffer[1] = setting & 0xFF;
}


DigitalPotHandler::DigitalPotHandler() {
	int result;
	wiperState0 = -1;
	wiperState1 = -1;
	wipersEngaged = false;
	
	
	cout << "DigitalPotHandler::DigitalPotHandler(): Initializing " << endl;
	
	fd = wiringPiSPISetup(CHANNEL, 500000);
	
	buffer[0] = (DIGI_POT_REG_STATUS << DIGI_POT_ADDR_SHIFT) | DIGI_POT_READ;	// READ is a 16-bit (not 8-bit) command
	buffer[1] = 0x00;	// 2nd part of 16-bit command
	buffer[2] = 0x00;
	buffer[3] = 0x00;
	
//	printf("DigitalPotHandler::DigitalPotHandler(): Sending data: ");
//	for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
	
	result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
	
	
//	printf("DigitalPotHandler::DigitalPotHandler(): Result: %d, Read data: ", result );
//	for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
	
#define STATUS_D_CHECK (0x1F << 5)
	
	unsigned short digiResult = digitalPotBufferToRegister(buffer);
	
	if (digiResult & STATUS_D_CHECK != STATUS_D_CHECK) {
		printf("DigitalPotHandler::DigitalPotHandler(): Status read failure...\n");
	} else {
		printf("DigitalPotHandler::DigitalPotHandler(): Status check success!\n");
	}
	buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_READ;	// READ is a 16-bit (not 8-bit) command
	buffer[1] = 0x00;	// 2nd part of 16-bit command
	buffer[2] = 0x00;
	buffer[3] = 0x00;
	
//	printf("DigitalPotHandler::DigitalPotHandler(): TCON READ: Sending data: ");
//	for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
	
	
	result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
	
	
//	printf("DigitalPotHandler::DigitalPotHandler(): Result: %d, Read data: ", result );
//	for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
	
	
#define DIGI_POT_TCON_R1A (0x01 << 6)
#define DIGI_POT_TCON_R1W (0x01 << 5)
#define DIGI_POT_TCON_R0A (0x01 << 2)
#define DIGI_POT_TCON_R0W (0x01 << 1)
	
	// Disconnect wiperH (turn into a rheostat)
	buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
	buffer[0] |= 0x01;	//  reserved bit "forced to 1", may not be needed here
	buffer[1] = 0xFF & ~DIGI_POT_TCON_R1A & ~DIGI_POT_TCON_R0A & ~DIGI_POT_TCON_R0W & ~DIGI_POT_TCON_R1W;	// 2nd part of 16-bit command
	
	
	
	
	// Setup non-volatile resistance
	double resistance = RESISTANCE_CANCEL * 2.0;
//	printf("DigitalPotHandler::DigitalPotHandler(): Configuring NV wiper setting to CRUISE_CANCEL button:\n - Setting WIPER_0_NV to %f ohms\n", resistance);
	buffer[0] = (DIGI_POT_REG_WIPER_0_NV << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
	digitalPotResistanceToBuffer(resistance, buffer);
	result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
	
//	printf("DigitalPotHandler::DigitalPotHandler(): - Setting WIPER_1_NV to %f ohms\n", resistance);
	buffer[0] = (DIGI_POT_REG_WIPER_1_NV << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
	digitalPotResistanceToBuffer(resistance, buffer);
	result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
	
//	printf("DigitalPotHandler::DigitalPotHandler(): - Effective resistance = %f ohms\n", resistance/2.0);
	
	
	
//	printf("DigitalPotHandler::DigitalPotHandler(): TCON WRITE: Sending data: ");
//	for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
	
	
	result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
	
	
//	printf("DigitalPotHandler::DigitalPotHandler(): Result: %d, Read data: ", result );
//	for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
}


DigitalPotHandler::~DigitalPotHandler() {
	releaseButton();
}


double nissanButtonToDigitalPotResistance( NissanButton& button ) {
	switch (button) {
		case NISSAN_BUTTON_CRUISE_ON:
			return RESISTANCE_CRUISE_ON * 2.0;
		case NISSAN_BUTTON_SET:
			return RESISTANCE_SET * 2.0;
		case NISSAN_BUTTON_RES:
			return RESISTANCE_RES * 2.0;
		case NISSAN_BUTTON_CANCEL:
			return RESISTANCE_CANCEL * 2.0;
		case NISSAN_BUTTON_DISTANCE:
			return RESISTANCE_DISTANCE * 2.0;
		case NISSAN_BUTTON_OFF:
			return 10000.0;
	}
	
	return 10000;
}

void DigitalPotHandler::pressButton( NissanButton button ) {
	int result;
	if (button == NISSAN_BUTTON_OFF) {
		releaseButton();
		return;
	}
	double resistance = nissanButtonToDigitalPotResistance( button );
	
	if (resistance == wiperState0 &&
		resistance == wiperState1 ) {
		//printf("DigitalPotHandler::pressButton(): WIPER_0 and WIPER_1 already set to %f ohms\n", resistance);
		
	} else {
		
		if (wipersEngaged) {
			// ensure button is already released:
			releaseButton();
			usleep(TIME_PER_POTENTIOMETER_SET);
		}
		
		// Setup
		printf("DigitalPotHandler::pressButton(): Setting WIPER_0 and WIPER_1 to %f ohms, Effective resistance = %f ohms\n", resistance, resistance/2.0);
		buffer[0] = (DIGI_POT_REG_WIPER_0 << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
		digitalPotResistanceToBuffer(resistance, buffer);
		result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
		wiperState0 = resistance;
		
//		printf("DigitalPotHandler::pressButton(): Setting WIPER_1 to %f ohms\n", resistance);
		buffer[0] = (DIGI_POT_REG_WIPER_1 << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
		digitalPotResistanceToBuffer(resistance, buffer);
		result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
		wiperState1 = resistance;
		
//		printf("DigitalPotHandler::pressButton(): - Effective resistance = %f ohms\n", resistance/2.0);
		
		usleep(TIME_PER_POTENTIOMETER_SET);
		
	}
	// Button Press
	//	double dTime = TIME_PER_BUTTON_PRESS/1000000.0;
	//	if (argc > 2)
	//		dTime =(double) stoi(argv[2])/1000.0;
	if (!wipersEngaged) {
		
		
		printf("DigitalPotHandler::pressButton(): Applying resistance to physical output\n");
		buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
		buffer[0] |= 0x01;	//  reserved bit "forced to 1", may not be needed here
		buffer[1] = 0xFF & ~DIGI_POT_TCON_R1A & ~DIGI_POT_TCON_R0A;	// 2nd part of 16-bit command
		result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
		
		wipersEngaged = true;
	}
	
	//	usleep(dTime * 1000000);
}

void DigitalPotHandler::releaseButton() {
	int result;
	// Button Release
	//	dTime = TIME_PER_BUTTON_RELEASE/1000000.0;
	//	if (argc > 3)
	//		dTime =(double) stoi(argv[3])/1000.0;
	printf("DigitalPotHandler::releaseButton():  Disconnecting potentiometer\n");//, dTime);
	buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
	buffer[0] |= 0x01;	//  reserved bit "forced to 1", may not be needed here
	buffer[1] = 0xFF & ~DIGI_POT_TCON_R1A & ~DIGI_POT_TCON_R0A & ~DIGI_POT_TCON_R0W & ~DIGI_POT_TCON_R1W ;	// 2nd part of 16-bit command
	result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
	
	wipersEngaged = false;
	//	usleep(dTime * 1000000);
}
