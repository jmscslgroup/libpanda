#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>

#include "panda.h"


int main(int argc, char* argv[])
{
//   int fd, result;
//   unsigned char buffer[100];

	if (argc < 2) {
		printf("Please provide a command and optionally: press time (ms), release time (ms), and number of times\n");
		printf("Commands: r s d c o (RES, SET, Distance Lines, Cancel, On/Off\n");
		printf("Example: %s s 1000 20 5 # This presses SET 5 times, with presses for 1s and releases for 0.02s\n", argv[0]);
		return -1;
	}
	

   std::cout << "Initializing" << std::endl ;
	
	Panda::DigitalPotHandler mDigitalPotHandler;
	
	Panda::MatthatAccButtonRelay mMatthatAccButtonRelay;
	
	

//   // Configure the interface.
//   // CHANNEL insicates chip select,
//   // 500000 indicates bus speed.
//   fd = wiringPiSPISetup(CHANNEL, 500000);
//
//   cout << "Init result: " << fd << endl;

//   // clear display
// //  buffer[0] = 0x05;
// //  wiringPiSPIDataRW(CHANNEL, buffer, 1);
//
////   sleep(5);
//
//   // Do a one-hot bit selection for each field of the display
//   // It displays gibberish, but tells us that we're correctly addressing all
//   // of the segments.
////   for(int j = 0; j < 6; j++)
////   {
//      // the decimals, colon and apostrophe dots
//      buffer[0] = (DIGI_POT_REG_STATUS << DIGI_POT_ADDR_SHIFT) | DIGI_POT_READ;	// READ is a 16-bit (not 8-bit) command
//      buffer[1] = 0x00;	// 2nd part of 16-bit command
//      buffer[2] = 0x00;
//      buffer[3] = 0x00;
//
//      printf("Sending data: ");
//      for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
//
//
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
//
//
//      printf("Result: %d, Read data: ", result );
//      for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
//
//	#define STATUS_D_CHECK (0x1F << 5)
//
//	unsigned short digiResult = digitalPotBufferToRegister(buffer);
//
//	if (digiResult & STATUS_D_CHECK != STATUS_D_CHECK) {
//		printf("Status read failure...\n");
//	} else {
//		printf("Status read success!\n");
//	}
//
//
//
//
//      buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_READ;	// READ is a 16-bit (not 8-bit) command
//      buffer[1] = 0x00;	// 2nd part of 16-bit command
//      buffer[2] = 0x00;
//      buffer[3] = 0x00;
//
//      printf("TCON READ: Sending data: ");
//      for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
//
//
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
//
//
//      printf("Result: %d, Read data: ", result );
//      for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
//
//
//#define DIGI_POT_TCON_R1A (0x01 << 6)
//#define DIGI_POT_TCON_R1W (0x01 << 5)
//#define DIGI_POT_TCON_R0A (0x01 << 2)
//#define DIGI_POT_TCON_R0W (0x01 << 1)
//
//	// Disconnect wiperH (turn into a rheostat)
//      buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
//      buffer[0] |= 0x01;	//  reserved bit "forced to 1", may not be needed here
//      buffer[1] = 0xFF & ~DIGI_POT_TCON_R1A & ~DIGI_POT_TCON_R0A & ~DIGI_POT_TCON_R0W & ~DIGI_POT_TCON_R1W;	// 2nd part of 16-bit command
//
//
//
//
//	// Setup non-volatile resistance
//	double resistance = RESISTANCE_CANCEL * 2.0;
//	printf("Configuring NV wiper setting to CRUISE_CANCEL button:\n - Setting WIPER_0_NV to %f ohms\n", resistance);
//      buffer[0] = (DIGI_POT_REG_WIPER_0_NV << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
//	digitalPotResistanceToBuffer(resistance, buffer);
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
//
//	printf(" - Setting WIPER_1_NV to %f ohms\n", resistance);
//      buffer[0] = (DIGI_POT_REG_WIPER_1_NV << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
//	digitalPotResistanceToBuffer(resistance, buffer);
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
//
//	printf(" - Effective resistance = %f ohms\n", resistance/2.0);
//
//
//
//      printf("TCON WRITE: Sending data: ");
//      for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");
//
//
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
//
//
//      printf("Result: %d, Read data: ", result );
//      for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");


// Engage relay:
//	initializeRelayGpio();
//	setRelay( 1 );
	
	mMatthatAccButtonRelay.arm();

	// Let's try a resistance
//	printf("Sweeping RWB from 0 to 10k...\n");

	Panda::NissanButton button;
	
		if(argv[1][0] == 'r') {
//			resistance = RESISTANCE_RES*2.0;
			button = Panda::NISSAN_BUTTON_RES;
//			printf("Sending a RES command, ");
		} else if (argv[1][0] == 's') {
//			resistance = RESISTANCE_SET*2.0;
			button = Panda::NISSAN_BUTTON_SET;
		} else if (argv[1][0] == 'd') {
//			resistance = RESISTANCE_DISTANCE*2.0;
			button = Panda::NISSAN_BUTTON_DISTANCE;
		} else if (argv[1][0] == 'c') {
//			resistance = RESISTANCE_CANCEL*2.0;
			button = Panda::NISSAN_BUTTON_CANCEL;
		} else if (argv[1][0] == 'o') {
//			resistance = RESISTANCE_CRUISE_ON*2.0;
			button = Panda::NISSAN_BUTTON_CRUISE_ON;
		} else {
			//
			printf("Invalid command %s\n", argv[1]);
			return -1;
		}


	int N = 1;
	if(argc > 4) 
		N = std::stoi(argv[4]);

	printf("Presseing button %d times\n", N);
	for (int i = 0; i < N; i++) {

		

//	for (; resistance <= 10000; resistance += 100) {
//	usleep(100000);


//	// Setup
//	printf("Setting WIPER_0 to %f ohms\n", resistance);
//      buffer[0] = (DIGI_POT_REG_WIPER_0 << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
//	digitalPotResistanceToBuffer(resistance, buffer);
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
//
//	printf("Setting WIPER_1 to %f ohms\n", resistance);
//      buffer[0] = (DIGI_POT_REG_WIPER_1 << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
//	digitalPotResistanceToBuffer(resistance, buffer);
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
//
//	printf(" - Effective resistance = %f ohms\n", resistance/2.0);
//
//	usleep(TIME_PER_POTENTIOMETER_SET);


	// Button Press
	double dTime = TIME_PER_BUTTON_PRESS/1000000.0;
	if (argc > 2)
		dTime =(double) std::stoi(argv[2])/1000.0;
//	printf("Pressing button for %0.2f seconds\n", dTime);
//      buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
//      buffer[0] |= 0x01;	//  reserved bit "forced to 1", may not be needed here
//      buffer[1] = 0xFF & ~DIGI_POT_TCON_R1A & ~DIGI_POT_TCON_R0A;	// 2nd part of 16-bit command
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
		mDigitalPotHandler.pressButton(button);

	usleep(dTime * 1000000);


	// Button Release
	dTime = TIME_PER_BUTTON_RELEASE/1000000.0;
	if (argc > 3)
		dTime =(double) std::stoi(argv[3])/1000.0;
//	printf("Releasing button for %0.2f seconds\n", dTime);
//      buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
//      buffer[0] |= 0x01;	//  reserved bit "forced to 1", may not be needed here
//      buffer[1] = 0xFF & ~DIGI_POT_TCON_R1A & ~DIGI_POT_TCON_R0A & ~DIGI_POT_TCON_R0W & ~DIGI_POT_TCON_R1W ;	// 2nd part of 16-bit command
//      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
		mDigitalPotHandler.releaseButton();
	usleep(dTime * 1000000);
//	usleep(TIME_PER_BUTTON_RELEASE);


	}

	mMatthatAccButtonRelay.disarm();
}
