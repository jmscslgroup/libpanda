#include <iostream>
#include <signal.h>
#include <unistd.h>	// usleep()
#include <time.h>

#include "panda.h"

// A ctrl-c handler for safe panda handler destruction
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
	std::cerr << std::endl << "Caught SIGINT: Terminating..." << std::endl;
	keepRunning = false;
}

class TestCanListener : public Panda::CanListener {
public:
	TestCanListener() {};
	
private:
	void newDataNotification(Panda::CanFrame* frame) {
		printf("New CAN read: %d.%06d,", (unsigned int)frame->sysTime.tv_sec, (int)frame->sysTime.tv_usec);

		printf("%d,%d,", (int)frame->bus, frame->messageID);

		for (int i =0; i < frame->dataLength; i++) {
			printf("%02x", frame->data[i]);
		}
		printf(",%d\n", frame->dataLength);
	}
};


int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;

	//Set up graceful exit
	signal(SIGINT, killPanda);

	Panda::Handler handler;
	TestCanListener mTestCanListener;
//	handler.getCan().addObserver(&mTestCanListener);
	handler.addCanObserver(mTestCanListener);
	
	handler.initialize();
	
	// Set RTC
	handler.getUsb().setRtcYear(2022);
	handler.getUsb().setRtcMonth(5);
	handler.getUsb().setRtcDay(10);
	handler.getUsb().setRtcWeekday(3);
	handler.getUsb().setRtcHour(3);
	handler.getUsb().setRtcMinute(19);
	handler.getUsb().setRtcSeconds(30);
	
	// Set CAN into loopback mode:
	handler.getUsb().setCanLoopback(true);
	
	
	bool greenLed = 0;
	
	int safetyModeIndex = 0;
	unsigned char safetyModes[] = {SAFETY_NOOUTPUT, SAFETY_SILENT, SAFETY_ELM327, SAFETY_ALLOUTPUT, SAFETY_TOYOTA, SAFETY_NISSAN, SAFETY_CIRCLES_NISSAN};
	while (keepRunning) {
		sleep(1);
		
		handler.getUsb().setSafetyMode(safetyModes[safetyModeIndex++], 0);
//		handler.getUsb().setSafetyMode(SAFETY_ALLOUTPUT, 0);
		if (safetyModeIndex >= sizeof(safetyModes)/sizeof(safetyModes[0])) {
			safetyModeIndex = 0;
		}
		
		// Check the RTC:
		PandaRtcTimestamp timestamp = handler.getUsb().getRtc();
		printf("Panda's RTC Time: %02d:%02d:%02d %d %2d/%02d/%04d\n", timestamp.hour, timestamp.minute, timestamp.second, timestamp.weekday, timestamp.month, timestamp.day, timestamp.year);
		
		// Get Fan Speed (pretty useless)
		std::cout << "Fan Rpm: " << handler.getUsb().getFanRpm() << std::endl;
		
		// Check hardware information:
		std::cout << "Hardware Information" << std::endl;
		std::cout << " - Type: " << Panda::hardwareTypeToString(handler.getUsb().getHardware()) << std::endl;
		unsigned char serial[16];
		
		// Panda Serial:
		handler.getUsb().getPandaSerial(serial);
		std::cout << " - Serial number: ";
		for (int i = 0; i < sizeof(serial); i++) {
			printf("%c", serial[i]);
		}
		std::cout << std::endl;
		
		// Get health packet:
		PandaHealth healthPacket;
		handler.getUsb().getHealth(&healthPacket);
		Panda::printPandaHealth(healthPacket);
		
		// Firmware or code signature:
		unsigned char firmware[128];
		handler.getUsb().getFirmware(firmware);
		std::cout << " - Firmware: ";
		for (int i = 0; i < sizeof(firmware); i++) {
			printf("%02X", firmware[i]);
		}
		std::cout << std::endl;
		
		// Code version based on git:
		std::cout << " - Git Version: ";
		unsigned char gitVersion[64];
		handler.getUsb().getGitVersion(gitVersion);
		std::cout << gitVersion << std::endl;
		
		// Vrsions of comma.ai Health structure and "CAN" something
		unsigned char healthVersion, canVersion;
		handler.getUsb().getHealthAndCanVersions(&healthVersion, &canVersion);
		std::cout << " - Health Version: " << (int)healthVersion << std::endl;
		std::cout << " - CAN Version: " << (int)canVersion << std::endl;
		
		// This is reported from USB descriptors, not from a USB control transfer:
		std::cout << " - USB Serial: " << handler.getUsb().getUsbSerialNumber() << std::endl;
		
		

		
//		int canBusses[] = {0, 1, 2, 0xFF};
		int canBusses[] = {0, 1, 2, 3};
		for (int i = 0; i < sizeof(canBusses)/sizeof(canBusses[0]); i++) {
			bool fdEnabled, brsEnabled;
			handler.getUsb().getCanFdEnabled(canBusses[i], &fdEnabled, &brsEnabled);
			std::cout << " CAN bus " << canBusses[i] << " - FD Enabled: " << fdEnabled << " BRS Enabled: " << brsEnabled << std::endl;
		}
		
		
		
		// Check if GPS supported:
		std::cout << "This device supports GPS: " << handler.getUsb().hasGpsSupport() << std::endl;
		
		
		// Send a heartbeat case why not.  Watch controls_allowed when in SAFETY_ALLOUTPUT
		handler.getUsb().sendHeartBeat();
		
		// Try faking some CAN messages with loopback enabled
//		handler.getUsb().setCanLoopback(true);
		Panda::CanFrame frame;
		frame.messageID = timestamp.second;
		frame.bus = 1;
		frame.dataLength = 8;
		frame.data[0] = timestamp.second;
		handler.getCan().sendMessage(frame);
		
		
		// Try green led:
		// Appears inconsistent because the loop that drives the LED in the panda runs at 1Hz.
		// Also, control_allowed will override this and force it to be on
		// control_allowed will be on with a heartbeat and when SAFETY_ALLOUTPUT
		greenLed = !greenLed;
		handler.getUsb().setGreenLed(greenLed);
		std::cout << "Green LED set to " << greenLed << std::endl;
		
		std::cout << std::endl;
		
		std::cout << "sizeof(CANPacket_t) = "  << (int)sizeof(CANPacket_t) << std::endl;
	}
	
	handler.stop();
	std::cout << "Done." << std::endl;

	return 0;
}
