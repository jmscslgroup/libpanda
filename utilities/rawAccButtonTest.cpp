#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
using namespace std;


#define RESISTANCE_CRUISE_ON (0)	// 0V
#define RESISTANCE_CANCEL (315)		// 1.17V
#define RESISTANCE_DISTANCE (867)	// 2.11V
#define RESISTANCE_SET (1940)		// 2.89V
//#define RESISTANCE_RES (4700+5095)	// 3.56V
#define RESISTANCE_RES (4790)	// 3.56V

// RES: 3.63V from switch, 3.64V from circuit 	-> 4.77k ohm
// SET: 2.94V from switch, 2.90V from circuit 	-> 1.75k ohm

#define TIME_PER_BUTTON_PRESS ((unsigned int)(200000*25))	// in us
#define TIME_PER_BUTTON_RELEASE ((unsigned int)(200000))	// in us
#define TIME_PER_POTENTIOMETER_SET ((unsigned int)(10000))	// in us

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static const int CHANNEL = 0;

#define DIGI_POT_READ (0x3 << 2)
#define DIGI_POT_WRITE (0x0 << 2)
#define DIGI_POT_ADDR_SHIFT (4)

#define DIGI_POT_REG_WIPER_0 (0x00)
#define DIGI_POT_REG_WIPER_0_NV (0x02)
#define DIGI_POT_REG_WIPER_1 (0x01)
#define DIGI_POT_REG_WIPER_1_NV (0x03)
#define DIGI_POT_REG_STATUS (0x05)
#define DIGI_POT_REG_TCON (0x04)


unsigned short digitalPotBufferToRegister(unsigned char *buffer) {
	unsigned short result = buffer[0];
	return (result << 8) | buffer[1];
}

#define DIGI_POT_RW (75)
void digitalPotResistanceToBuffer(double resistance, unsigned char *buffer) {
	resistance = resistance > (10000+DIGI_POT_RW) ? (10000+DIGI_POT_RW) : resistance;
	resistance = resistance < DIGI_POT_RW ? DIGI_POT_RW : resistance;
	unsigned short setting = (resistance - DIGI_POT_RW)/10000.0 * 256;
	buffer[0] = (buffer[0] & 0xFC) | (setting >> 8);
	buffer[1] = setting & 0xFF;
}

// Definitions for relay operation:
#define DIGI_POT_RELAY_POWER_GPIO_PIN (16)

int file_gpio = -1;
void initializeRelayGpio() {
	int fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/export");
		//			exit(1);
	}
	
	char configstring[100];
	sprintf(configstring, "%d", DIGI_POT_RELAY_POWER_GPIO_PIN);
	
	if (write(fd, configstring, 2) != 2) {
		perror("Error writing to /sys/class/gpio/export");
		//			exit(1);
	}

	close(fd);

	sprintf(configstring, "/sys/class/gpio/gpio%d/direction", DIGI_POT_RELAY_POWER_GPIO_PIN);

	fd = open(configstring, O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Unable to open %s", configstring);
		//			exit(1);
	}

	if (write(fd, "out", 3) != 3) {
		perror("Error writing to /sys/class/gpio/gpio/16/direction");
		fprintf(stderr, "Error writing to %s", configstring);
		//			exit(1);
	}

	close(fd);

     file_gpio = open("/sys/class/gpio/gpio16/value", O_WRONLY);
	if (file_gpio == -1) {
		perror("Unable to open /sys/class/gpio/gpio16/value");
		//		exit(1);
	}
}

void closeRelayGpio() {
	char configstring[100];

	int fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/unexport");
		//		exit(1);
	}

	sprintf(configstring, "%d", DIGI_POT_RELAY_POWER_GPIO_PIN);
	if (write(fd, configstring, 2) != 2) {
		perror("Error writing to /sys/class/gpio/unexport");
		//		exit(1);
	}

	close(fd);
}

void setRelay( bool enable ) {
	if (enable != 0) {
		if (write(file_gpio, "1", 1) != 1) {
			perror("Error writing to /sys/class/gpio/gpio16/value");
			//			exit(1);
		}
	} else {
		if (write(file_gpio, "0", 1) != 1) {
			perror("Error writing to /sys/class/gpio/gpio16/value");
			//			exit(1);
		}
	}
}

int main(int argc, char* argv[])
{
   int fd, result;
   unsigned char buffer[100];

	if (argc < 2) {
		printf("Please provide a command and optionally: press time (ms), release time (ms), and number of times\n");
		printf("Commands: r s d c o (RES, SET, Distance Lines, Cancel, On/Off\n");
		printf("Example: %s s 1000 20 5 # This presses SET 5 times, with presses for 1s and releases for 0.02s\n", argv[0]);
		return -1;
	}

   cout << "Initializing" << endl ;

   // Configure the interface.
   // CHANNEL insicates chip select,
   // 500000 indicates bus speed.
   fd = wiringPiSPISetup(CHANNEL, 500000);

   cout << "Init result: " << fd << endl;

   // clear display
 //  buffer[0] = 0x05;
 //  wiringPiSPIDataRW(CHANNEL, buffer, 1);

//   sleep(5);

   // Do a one-hot bit selection for each field of the display
   // It displays gibberish, but tells us that we're correctly addressing all 
   // of the segments.
//   for(int j = 0; j < 6; j++)
//   {
      // the decimals, colon and apostrophe dots
      buffer[0] = (DIGI_POT_REG_STATUS << DIGI_POT_ADDR_SHIFT) | DIGI_POT_READ;	// READ is a 16-bit (not 8-bit) command
      buffer[1] = 0x00;	// 2nd part of 16-bit command
      buffer[2] = 0x00;
      buffer[3] = 0x00;

      printf("Sending data: ");
      for (int i = 0; i < 2; i++) {
		printf("0x%02X ", buffer[i]);
	}
	printf("\n");


      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);


      printf("Result: %d, Read data: ", result );
      for (int i = 0; i < 2; i++) {
		printf("0x%02X ", buffer[i]);
	}
	printf("\n");

	#define STATUS_D_CHECK (0x1F << 5)

	unsigned short digiResult = digitalPotBufferToRegister(buffer);

	if (digiResult & STATUS_D_CHECK != STATUS_D_CHECK) {
		printf("Status read failure...\n");
	} else {
		printf("Status read success!\n");
	}




      buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_READ;	// READ is a 16-bit (not 8-bit) command
      buffer[1] = 0x00;	// 2nd part of 16-bit command
      buffer[2] = 0x00;
      buffer[3] = 0x00;

      printf("TCON READ: Sending data: ");
      for (int i = 0; i < 2; i++) {
		printf("0x%02X ", buffer[i]);
	}
	printf("\n");


      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);


      printf("Result: %d, Read data: ", result );
      for (int i = 0; i < 2; i++) {
		printf("0x%02X ", buffer[i]);
	}
	printf("\n");


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
	printf("Configuring NV wiper setting to CRUISE_CANCEL button:\n - Setting WIPER_0_NV to %f ohms\n", resistance);
      buffer[0] = (DIGI_POT_REG_WIPER_0_NV << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
	digitalPotResistanceToBuffer(resistance, buffer);
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

	printf(" - Setting WIPER_1_NV to %f ohms\n", resistance);
      buffer[0] = (DIGI_POT_REG_WIPER_1_NV << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
	digitalPotResistanceToBuffer(resistance, buffer);
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

	printf(" - Effective resistance = %f ohms\n", resistance/2.0);



      printf("TCON WRITE: Sending data: ");
      for (int i = 0; i < 2; i++) {
		printf("0x%02X ", buffer[i]);
	}
	printf("\n");


      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);


      printf("Result: %d, Read data: ", result );
      for (int i = 0; i < 2; i++) {
		printf("0x%02X ", buffer[i]);
	}
	printf("\n");


// Engage relay:
	initializeRelayGpio();
	setRelay( 1 );

	// Let's try a resistance
//	printf("Sweeping RWB from 0 to 10k...\n");


		if(argv[1][0] == 'r') {
			resistance = RESISTANCE_RES*2.0;
			printf("Sending a RES command, ");
		} else if (argv[1][0] == 's') {
			resistance = RESISTANCE_SET*2.0;
		} else if (argv[1][0] == 'd') {
			resistance = RESISTANCE_DISTANCE*2.0;
		} else if (argv[1][0] == 'c') {
			resistance = RESISTANCE_CANCEL*2.0;
		} else if (argv[1][0] == 'o') {
			resistance = RESISTANCE_CRUISE_ON*2.0;
		} else {
			//
			printf("Invalid command %s\n", argv[1]);
			return -1;
		}


	int N = 1;
	if(argc > 4) 
		N = stoi(argv[4]);

	printf("Presseing button %d times\n", N);
	for (int i = 0; i < N; i++) {


//	for (; resistance <= 10000; resistance += 100) {
//	usleep(100000);


	// Setup
	printf("Setting WIPER_0 to %f ohms\n", resistance);
      buffer[0] = (DIGI_POT_REG_WIPER_0 << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
	digitalPotResistanceToBuffer(resistance, buffer);
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

	printf("Setting WIPER_1 to %f ohms\n", resistance);
      buffer[0] = (DIGI_POT_REG_WIPER_1 << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
	digitalPotResistanceToBuffer(resistance, buffer);
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

	printf(" - Effective resistance = %f ohms\n", resistance/2.0);

	usleep(TIME_PER_POTENTIOMETER_SET);


	// Button Press
	double dTime = TIME_PER_BUTTON_PRESS/1000000.0;
	if (argc > 2)
		dTime =(double) stoi(argv[2])/1000.0;
	printf("Pressing button for %0.2f seconds\n", dTime);
      buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
      buffer[0] |= 0x01;	//  reserved bit "forced to 1", may not be needed here
      buffer[1] = 0xFF & ~DIGI_POT_TCON_R1A & ~DIGI_POT_TCON_R0A;	// 2nd part of 16-bit command
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

	usleep(dTime * 1000000);


	// Button Release
	dTime = TIME_PER_BUTTON_RELEASE/1000000.0;
	if (argc > 3)
		dTime =(double) stoi(argv[3])/1000.0;
	printf("Releasing button for %0.2f seconds\n", dTime);
      buffer[0] = (DIGI_POT_REG_TCON << DIGI_POT_ADDR_SHIFT) | DIGI_POT_WRITE;	// READ is a 16-bit (not 8-bit) command
      buffer[0] |= 0x01;	//  reserved bit "forced to 1", may not be needed here
      buffer[1] = 0xFF & ~DIGI_POT_TCON_R1A & ~DIGI_POT_TCON_R0A & ~DIGI_POT_TCON_R0W & ~DIGI_POT_TCON_R1W ;	// 2nd part of 16-bit command
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

	usleep(dTime * 1000000);
//	usleep(TIME_PER_BUTTON_RELEASE);


	}

 //     printf("Result: %d, Read data: ", result );
   //   for (int i = 0; i < 2; i++) {
//		printf("0x%02X ", buffer[i]);
//	}
//	printf("\n");

//}


/*
      // The first character
      buffer[0] = 0x7b;
      buffer[1] = i;
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

      // The second character
      buffer[0] = 0x7c;
      buffer[1] = i;
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

      // The third character
      buffer[0] = 0x7d;
      buffer[1] = i;
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);

      // The last character
      buffer[0] = 0x7e;
      buffer[1] = i;
      result = wiringPiSPIDataRW(CHANNEL, buffer, 2);
*/
      // Pause so we can see them
//      sleep(1);
//   }


 // Close relay:
  setRelay( 0 );
  closeRelayGpio();
   // clear display again
   buffer[0] = 0x76;
   wiringPiSPIDataRW(CHANNEL, buffer, 1);

}
