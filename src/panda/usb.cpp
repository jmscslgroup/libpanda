/*
 Author: Matt Bunting
 Copyright (c) 2020 Arizona Board of Regents
 All rights reserved.

 Permission is hereby granted, without written agreement and without
 license or royalty fees, to use, copy, modify, and distribute this
 software and its documentation for any purpose, provided that the
 above copyright notice and the following two paragraphs appear in
 all copies of this software.

 IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
 FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
 ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
 IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 SUCH DAMAGE.

 THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
 IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
 TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 */

#include "panda/usb.h"
#include "panda/pandadefinitions.h"
#include "panda/matthat.h"	// For reset functionality if USB device not found

#include "panda_version_i.h"

#include <cstring>
#include <unistd.h>
#include <sstream>

using namespace Panda;

Usb::Usb(UsbMode mode)
:hasGps(false), mode(mode), handler(NULL) {

}

Usb::~Usb() {
	stopRecording();	// closes libusb

	libusb_exit(NULL);
}

const char* Usb::getModeAsString() const {
	switch (mode) {
		case MODE_SYNCHRONOUS:
			return "Synchronous";
		case MODE_ASYNCHRONOUS:
			return "Asynchronous";
		case MODE_ISOCHRONOUS:
			return "Isochronous";
	}
	return "UNKOWN mode";
}

void Usb::setOperatingMode(UsbMode mode) {
	this->mode = mode;
}

void Usb::initialize() {
	std::cerr << "Initializing Usb..." << std::endl;
	libusb_device **devices;

	//Returns:
	// - 0 on success, or a LIBUSB_ERROR code on failure
	if (libusb_init(NULL) != 0) {
		std::cerr << "FAILED to perform libusb_init()" << std::endl;
		exit(EXIT_FAILURE);
	}
	
	
	bool failureToOpen = false;
	for (int i = 0; i < 2; i++) {	// attempt to open device 2 times
		
		//Returns:
		// - the number of devices in the outputted list, or any libusb_error according to errors encountered by the backend.
		ssize_t numberOfDevices = libusb_get_device_list(NULL, &devices);
		if (numberOfDevices < 0){
			libusb_exit(NULL);
			std::cerr << "FAILED to perform libusb_get_device_list()" << std::endl;
			exit(EXIT_FAILURE);
		}
		
		// Returns:
		// - 0 if found, see get_panda_device() definition in this file
		if(openDeviceByManufacturer(devices, "circles") != 0 ) {
			std::cerr << "Failed to open a custom \"circles\" firmware-based panda device, trying a \"comma.ai\" device" << std::endl;
			if(openDeviceByManufacturer(devices, "comma.ai") != 0 ) {
				if (failureToOpen == true) {	// ALREADY FAILED ONCE, MUST NOT BE CONNECTED
					std::cerr << "FAILED to open \"comma.ai\", is it plugged in?" << std::endl;
					std::cerr << " - May need root privileges if failed to open" << std::endl;
					exit(EXIT_FAILURE);
				}
				failureToOpen = true;
				// Let's reset the
				std::cerr << "FAILED to open \"comma.ai\", performing a matthat board reset via GPIO..." << std::endl;
				MatthatReset reset;
				reset.doit();
				
			}
		}
	}
	
	//Returns:
	// - LIBUSB_SUCCESS on success
	// - LIBUSB_ERROR_NOT_SUPPORTED on platforms where the functionality is not available
	if (libusb_set_auto_detach_kernel_driver(handler, true) != LIBUSB_SUCCESS) {
		std::cerr << "FAILED to perform libusb_set_auto_detach_kernel_driver()" << std::endl;
		std::cerr << " - Does platform support this operation?" << std::endl;
		exit(EXIT_FAILURE);
	}

	//Returns
	// - 0 on success
	// - LIBUSB_ERROR_NOT_FOUND if the requested interface does not exist
	// - LIBUSB_ERROR_BUSY if another program or driver has claimed the interface
	// - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	// - a LIBUSB_ERROR code on other failure
	int status = libusb_claim_interface(handler, 0);
	if (status != 0) {
		std::cerr << "FAILED to perform libusb_claim_interface()" << std::endl;
		printError(status);
		exit(EXIT_FAILURE);
	}

	// Code version based on git:
	std::cout << " - Current Panda firmware git version: ";
	char gitVersion[64];
	getGitVersion((unsigned char*)gitVersion);
	std::cout << gitVersion << std::endl;
	
	bool firmwareMatch = false;
	char* token = strtok(gitVersion, "-");
	if (token != NULL) {
		token = strtok(NULL, "-");
		if (token != NULL) {
			if (strcmp(token, PANDA_VERSION) == 0) {
				firmwareMatch = true;
			}
		}
	}
	if (!firmwareMatch) {
		std::cout << " |-- " << "\u001b[1m\u001b[31m" << "Warning! This Panda firmware does not match the supported version: " << PANDA_VERSION
			   << "\u001b[0m" << std::endl;
	} else {
		std::cout << " |-- This Panda firmware is " << "\u001b[1m\u001b[32m" << "Supported" << "\u001b[0m" << std::endl;
	}
	
	
	
//	libusb_control_transfer(handler, 0xc0, 0xe5, 1, 0, NULL, 0, 0);
//	unsigned char firmware[128];
//	getFirmware(firmware);
//	std::cout << " - Firmware:";
//	for (int i = 0; i < 128; i++) {
//		printf("%02X", firmware[i]);
//	}
//	std::cout << std::endl;

	// Void function:
	libusb_free_device_list(devices, 1);
	
	
	PandaHealth healthPacket;
	getHealth(&healthPacket);
	
	std::cout << " - Current Panda Health:" << std::endl;
	printPandaHealth(healthPacket);
	

	
//	std::cout << " - Setting power mode to POWER_SAVE_STATUS_DISABLED:" << std::endl;
//	setPowerSaveEnable(POWER_SAVE_STATUS_DISABLED);
//	setPowerSaveEnable(POWER_SAVE_STATUS_ENABLED);

//	std::cout << " - Setting Safety to ELM327:" << std::endl;
////	setSafetyMode(SAFETY_TOYOTA);	// OBD II port
////	std::cout << " - Setting Safety elm327:" << std::endl;
//	setSafetyMode(SAFETY_ELM327, 0);	// OBD II port
//		std::cout << " - Setting Safety ALL_OUTPUT:" << std::endl;
 //	setSafetyMode(SAFETY_ALLOUTPUT);	// OBD II port
//
//	std::cout << " - Enabling CAN Loopback:" << std::endl;
//	setCanLoopback(true);
	std::cout << " - Disabling CAN Loopback:" << std::endl;
	setCanLoopback(false);


	std::cout << " - Reading USB hardware model:" << std::endl;
	PandaHardwareType hardwareType = getHardware();

	//printf("Got the following value in return: %d\n", (int)hardwareType);

	switch (hardwareType) {
		case HARDWARE_WHITE_PANDA:
			std::cout << " |-- This is a WHITE PANDA, no built-in GPS" << std::endl;
			break;
		case HARDWARE_GREY_PANDA:
			hasGps = true;
			std::cout << " |-- This is a GREY PANDA" << std::endl;
			break;
		case HARDWARE_BLACK_PANDA:
			hasGps = true;
			std::cout << " |-- This is a BLACK PANDA" << std::endl;
			break;
		case HARDWARE_UNO:
			hasGps = true;
			std::cout << " |-- This is an UNO" << std::endl;
			break;
		case HARDWARE_PEDAL:
			std::cout << " |-- This is an PEDAL, no built-in GPS" << std::endl;
			break;
		case HARDWARE_DOS:
			std::cout << " |-- This is a DOS" << std::endl;
			break;
		case HARDWARE_RED_PANDA:
			hasGps = false;
			std::cout << " |-- This is a RED PANDA, no built-in GPS" << std::endl;
			std::cout << " |-- CAN Bus FD information (only valid on Red Panda):" << std::endl;
			bool fdEnabled, brsEnabled;
			for (int i = 0; i < 3; i++) {
				getCanFdEnabled(i, &fdEnabled, &brsEnabled);
				std::cout << " |---- Bus " << i << " FD is " << (fdEnabled ? " ENABLED" : "DISABLED") << " BRS is " << (brsEnabled ? " ENABLED" : "DISABLED") << " - Setting Baud to 500kbs" << std::endl;
				setCanFdBaud(i, 500);
			}
//			std::cout << "  |- Setting baudrate of bus 0 to CAN FD speeds" << std::endl;
//			setCanFdBaud(0, 20000);
//			std::cout << "  |- Setting baudrate of bus 2 to CAN FD speeds" << std::endl;
//			setCanFdBaud(2, 20000);
			
			break;

		case HARDWARE_UNKNOWN:
			std::cout << " |-- This is UNKOWN HARDWARE" << std::endl;
	}
	
//	// Read the VIN here:
//	ObdPidRequest vinRequest(*this);
//	int vinAttempts = 0;
//	while(!vinRequest.complete() &&
//		  vinAttempts++ < 10 ) {
//		std::cout << " - Attempt " << vinAttempts << " reading VIN..." << std::endl;
//		vinRequest.request(Panda::OBD_PID_SERVICE_VEHICLE_INFO, Panda::OBD_PID_VEHICLE_INFO_VIN, false);
//		sleep(1);
//	}
//	if (vinRequest.complete()) {
//		// We got it!
//		printf(" - - Read VIN:");
//		for (int i = 0; i < vinRequest.dataLength; i++) {
//			printf("%c", vinRequest.data[i]);
//		}
//		printf("\n");
//		FILE* file = fopen( "/etc/libpanda.d/vin", "w+");
//		fwrite( vinRequest.data, 1, vinRequest.dataLength, file);
//		fclose(file);
//	} else {
//		std::cerr << "WARNING: Unable to read the VIN!" << std::endl;
//	}
//	
//	std::cout << " - Setting Safety to Toyota:" << std::endl;
//	setSafetyMode(SAFETY_TOYOTA);	// OBD II port

	std::cerr << " - USB Done." << std::endl;
}

int Usb::openDeviceByManufacturer(libusb_device **devices, const char* manufacturerName) {
	int i = 0;
	int status;
	struct libusb_device *device;

	std::cout << " - Checking USB device Manufacturer names.." << std::endl;
	while ((device = devices[i++]) != NULL) {
		
		struct libusb_device_descriptor desc;
		//Returns:
		// - 0 on success or a LIBUSB_ERROR code on failure
		if ((status = libusb_get_device_descriptor(device, &desc)) != 0) {
			std::cerr << "FAILED to perform libusb_get_device_descriptor()" << std::endl;
			printError(status);
			return -1;
		}

		char usbManufacturer[200];	// Bunting: I assume 200 is plenty big
		if((status = libusb_open(device, &handler)) == LIBUSB_SUCCESS) {
			libusb_get_string_descriptor_ascii(handler, desc.iManufacturer, (unsigned char*)usbManufacturer, 200);
			//std::cout << " |-- Found: " << usbManufacturer << std::endl;
			if(strcmp(usbManufacturer, manufacturerName) == 0) {	// Check if comma.ai device
				// populate serial number,device then exit.
				libusb_get_string_descriptor_ascii(handler, desc.iSerialNumber, (unsigned char*)serialNumber, 200);
				std::cout << " |-- Panda found! Device is open." << std::endl;
				return 0;
			}
			libusb_close(handler);
			//std::cout << "handler pointer:" << (long int)handler << std::endl;
			handler = NULL;	// We will set this manually for future checking if panda is not found
		} else {
			std::cerr << "FAILED to perform libusb_open()" << std::endl;
			printError(status);
		}

	}
	return -1; //failure, no panda device found
}

/*
 Libusb converted commands:
 These have been truncated from synchronous libusb methods to noe be asynchronous:
 */
// This is a truncation of a direct copy from libusb's sync.c libusb_control_transfer()
// Only the first half is performed, up to a transfer sumbission, thus letting the libusb_handle_event()
// in doAction() handle the transfers in an aynchronous fashion.
int asyncControlTransfer(libusb_device_handle *dev_handle,
						 uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
						 unsigned char *data, uint16_t wLength, unsigned int timeout, libusb_transfer_cb_fn callback, void *user_data) {
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	unsigned char *buffer;
	//int completed = 0;
	int status;

	if (transfer == NULL) {
		return LIBUSB_ERROR_NO_MEM;
	}

	buffer = (unsigned char*)malloc(LIBUSB_CONTROL_SETUP_SIZE + wLength);
	if (!buffer) {
		libusb_free_transfer(transfer);
		return LIBUSB_ERROR_NO_MEM;
	}

	libusb_fill_control_setup(buffer, bmRequestType, bRequest, wValue, wIndex, wLength);
	if ((bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT) {
		memcpy(buffer + LIBUSB_CONTROL_SETUP_SIZE, data, wLength);
	}
	libusb_fill_control_transfer(transfer, dev_handle, buffer,
								 callback, user_data, timeout);
	transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;
	status = libusb_submit_transfer(transfer);

	if (status < 0) {
		libusb_free_transfer(transfer);
		return status;
	}
	// <- Truncated here
	return LIBUSB_SUCCESS;
}

// Again, the below is a truncated version of libusb's do_sync_bulk_transfer() (i.e. libusb_bulk_transfer())
// This has also been modified to create a new separate buffer to prevent race conditions
int asyncBulkTransfer(struct libusb_device_handle *dev_handle,
					  unsigned char endpoint, unsigned char *buffer, int length, unsigned int timeout, unsigned char type, libusb_transfer_cb_fn callback, void *user_data)
{
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	unsigned char *newBuffer; // I added this
	//int completed = 0;
	int r;

	if (!transfer) {
		return LIBUSB_ERROR_NO_MEM;
	}

	newBuffer = (unsigned char*)malloc(length);	// also added
	if (!newBuffer) {		// also added
		libusb_free_transfer(transfer);
		return LIBUSB_ERROR_NO_MEM;
	}

	libusb_fill_bulk_transfer(transfer, dev_handle, endpoint, newBuffer, length,
							  callback, user_data, timeout);

	transfer->type = type;
	if ((endpoint & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT) {	// I added this too
		memcpy( newBuffer, buffer, length);
	}
	transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;	// also added this

	r = libusb_submit_transfer(transfer);
	if (r < 0) {
		libusb_free_transfer(transfer);
		return r;
	}
	// <- Truncated here
	return r;
}

int isocControlTransfer(libusb_device_handle *dev_handle,
						unsigned char endpoint,
						unsigned char *data,
						uint16_t wLength,
						unsigned int timeout,
						libusb_transfer_cb_fn callback,
						void *user_data) {
	// TODO: figure out how this shoul dbe set:
	int num_iso_packets = 2;
	struct libusb_transfer *transfer = libusb_alloc_transfer(num_iso_packets);
	unsigned char *buffer;
	//int completed = 0;
	int status;

	if (transfer == NULL) {
		return LIBUSB_ERROR_NO_MEM;
	}

	buffer = (unsigned char*)malloc(LIBUSB_CONTROL_SETUP_SIZE + wLength);	// not sure if padding needed
	if (!buffer) {
		libusb_free_transfer(transfer);
		return LIBUSB_ERROR_NO_MEM;
	}


	//	REQUEST_TYPE_IN,
	//	REQUEST_UART_READ,
	//	UART_DEVICE_GPS,
	libusb_fill_control_setup(buffer, REQUEST_TYPE_IN, REQUEST_UART_READ, UART_DEVICE_GPS, 0, wLength);


	//	if ((bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
	//		memcpy(buffer + LIBUSB_CONTROL_SETUP_SIZE, data, wLength);

	libusb_fill_iso_transfer(transfer, dev_handle, endpoint, buffer, wLength, num_iso_packets, callback, user_data, timeout);
	//libusb_fill_control_transfer(transfer, dev_handle, buffer,
	//							 callback, user_data, timeout)
	libusb_set_iso_packet_lengths(transfer, wLength/num_iso_packets);	// TODO: what hsould this length be?
	transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;	// should be OK in ISO style
	std::cout << "Submitting ISO request" << std::endl;
	status = libusb_submit_transfer(transfer);

	if (status < 0) {
		libusb_free_transfer(transfer);
		return status;
	}
	// <- Truncated here
	return status;
}


/*
 CAN data handling
 */
int Usb::sendCanData( unsigned char* buffer, int length) {
//	// "libusb_bulk_transfer(dev_handle, 3, (uint8_t*)send, msg_count*0x10, &sent, TIMEOUT)"
//	// comma.ai code uses the endpoint setting of 3
	int status, actualLength;
	switch (mode) {
		case MODE_SYNCHRONOUS:
			lock();
			if((status = libusb_bulk_transfer(handler,
											  ENDPOINT_CAN_OUT,
											  buffer,
											  length,
											  &actualLength,
											  TIMEOUT)) != LIBUSB_SUCCESS) {
				std::cerr << "ERROR: Usb::sendCanData()->libusb_bulk_transfer():" <<std::endl;
				printError(status);
				return -1;
			}
			// No callbacks are called here, so perform notifications manually:
			processNewCanSend(actualLength);
			unlock();
			break;

		case MODE_ASYNCHRONOUS:
			if((status = asyncBulkTransfer(handler,
										   ENDPOINT_CAN_OUT,
										   buffer,
										   length,
										   TIMEOUT,
										   LIBUSB_TRANSFER_TYPE_BULK,
										   Usb::transferCallbackSendCan,
										   (void*)this)) != LIBUSB_SUCCESS) {
				std::cerr << "ERROR: Usb::sendCanData()->asyncBulkTransfer():" <<std::endl;
				printError(status);
				if (status == LIBUSB_ERROR_NO_DEVICE) {
					std::cout << " - INFO: Usb::sendCanData(): Device may be unplugged" << std::endl;
					exit(EXIT_FAILURE);
				}
				return -1;
			}

			break;

		case MODE_ISOCHRONOUS:	// TODO:

			break;

	}
	return 0;
}

void Usb::requestCanData() {
	// Rahul's code had usb1.ENDPOINT_IN | self.ENDPOINT_READ
	// LIBUSB_ENDPOINT_IN is already defined.  not sure about the bitwise OR with Rahul's defined ENDPOINT_READ
	// the python code uses setBulk(), which is a simple wrapper for the following
	// Note: comma.ai uses an endpoint value of 0x81 for reads.  Thats the same as what's used here
	int status, actualLength;
	switch (mode) {
		case MODE_SYNCHRONOUS:
			lock();	// This is done not for libusb, but for bufferSynchronousCan

			if((status = libusb_bulk_transfer(handler,		// Returns LIBUS_ERROR_***
											  ENDPOINT_CAN_IN,
											  bufferSynchronousCan,
											  sizeof(bufferSynchronousCan),
											  &actualLength,
											  TIMEOUT)) != LIBUSB_SUCCESS) {
				std::cerr << "ERROR: Usb::requestCanData()->libusb_bulk_transfer():" << std::endl;
				printError(status);
				if(status == LIBUSB_ERROR_OVERFLOW) {
					std::cerr << " |-- Overflow: Received: " << actualLength << "Requested: " << (int)sizeof(bufferSynchronousCan) << std::endl;
				} else if (status == LIBUSB_ERROR_TIMEOUT) {
					// Still received data, according to boardd.cc code:
					processNewCanRead((char*)bufferSynchronousCan, actualLength);
				}
			} else {
				// No callbacks are called here, so perform notifications manually:
				processNewCanRead((char*)bufferSynchronousCan, actualLength);
			}
			unlock();

			break;

		case MODE_ASYNCHRONOUS:
			//std::cout << " - requesting bulk Asynchronous" << std::endl;
			if((status = asyncBulkTransfer(handler,		// Returns LIBUS_ERROR_***
										   ENDPOINT_CAN_IN,
										   NULL,
										   sizeof(bufferSynchronousCan),
										   TIMEOUT,
										   LIBUSB_TRANSFER_TYPE_BULK,
										   Usb::transferCallbackReadCan,
										   (void*)this)) != LIBUSB_SUCCESS) {
				std::cerr << "ERROR: Usb::requestCanData()->asyncBulkTransfer():" <<std::endl;
				printError(status);
				if( status == LIBUSB_ERROR_NO_DEVICE ) {
					std::cerr << "INFO: Usb::requestCanData(): Device likely disconnected, exiting..." <<std::endl;
				}
			}

			break;

		case MODE_ISOCHRONOUS:	// TODO

			break;

	}

}



/*
 Libusb Callbacks
 */
void Usb::transferCallbackSendCan(struct libusb_transfer *transfer) {
	Usb* This = (Usb*) transfer->user_data;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		std::cout << " - Incomplete: transferCallbackSendCan()" << std::endl;
		printErrorTransfer(transfer->status);
		if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
			std::cout << " - INFO: Usb::transferCallbackSendCan(): Device may be unplugged" << std::endl;
			exit(EXIT_FAILURE);
		}
		if (transfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
			This->processNewCanSend(0);
			libusb_free_transfer(transfer);
			return;
		}
	}
	This->processNewCanSend(transfer->actual_length);
	libusb_free_transfer(transfer);
}

void Usb::transferCallbackReadCan(struct libusb_transfer *transfer) {
	Usb* This = (Usb*) transfer->user_data;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		std::cout << " - Incomplete: transferCallbackReadCan()" << std::endl;
		printErrorTransfer(transfer->status);
		if (transfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
			This->processNewCanRead(NULL, 0);
			libusb_free_transfer(transfer);
			return;
		}
	}
	This->processNewCanRead((char*)transfer->buffer, transfer->actual_length);
	libusb_free_transfer(transfer);
}

void Usb::transferCallbackSendUart(struct libusb_transfer *transfer) {
	Usb* This = (Usb*) transfer->user_data;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		std::cout << " - Incomplete: transferCallbackSendUart()" << std::endl;
		printErrorTransfer(transfer->status);
		if (transfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
			This->processNewUartSend(0);
			libusb_free_transfer(transfer);
			return;
		}
	}
	This->processNewUartSend(transfer->actual_length);
	libusb_free_transfer(transfer);
}

void Usb::transferCallbackReadUart(struct libusb_transfer *transfer) {
	Usb* This = (Usb*) transfer->user_data;
	//std::cout << "In Usb::transferCallbackReadUart()" << std::endl;
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		std::cout << " - Incomplete: transferCallbackReadUart()" << std::endl;
		printErrorTransfer(transfer->status);
		if (transfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
			This->processNewUartRead(NULL, 0);
			libusb_free_transfer(transfer);
			return;
		}
	}
	if (transfer->type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
		std::cout << "ISOCHRONOUS UART READ! " << std::endl;
		for (int i = 0; i < transfer->num_iso_packets; i++) {
			if (transfer->iso_packet_desc[i].status != LIBUSB_TRANSFER_COMPLETED) {
				std::cout << " - Incomplete isochronous transfer: transferCallbackReadUart()" << std::endl;
				return;
			}
			printf("pack%u length:%u, actual_length:%u\n", i, transfer->iso_packet_desc[i].length, transfer->iso_packet_desc[i].actual_length);
		}

	} else {
		This->processNewUartRead((char*)libusb_control_transfer_get_data(transfer), transfer->actual_length);
	}

	libusb_free_transfer(transfer);
}

/*
 Observer Notifications
 */
void Usb::processNewCanRead(char* buffer, size_t length) {
	for (std::vector<UsbListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		UsbListener* listener = *it;
		listener->notificationCanRead(buffer, length);
	}
}

void Usb::processNewCanSend(size_t length) {
	for (std::vector<UsbListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		UsbListener* listener = *it;
		listener->notificationCanWrite();
	}
}

void Usb::processNewUartRead(char* buffer, size_t length) {
	for (std::vector<UsbListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		UsbListener* listener = *it;
		listener->notificationUartRead(buffer, length);
	}
}

void Usb::processNewUartSend(size_t length) {
	for (std::vector<UsbListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		UsbListener* listener = *it;
		listener->notificationUartWrite();
	}
}

void Usb::addObserver( UsbListener* listener ) {
	listeners.push_back(listener);
}



/*
 Thread handling
 */
void Usb::startRecording() {
	start();
};

void Usb::stopRecording() {
	stop();

	int status;
	if (handler != NULL) {
		std::cout << " - Releasing libusb interface..." << std::endl;
		if((status = libusb_release_interface(handler, 0) != LIBUSB_SUCCESS)) {
			std::cerr << "FAILED: Usb::end() libusb_release_interface() had an error:" << std::endl;
			printError(status);
		};
		
		std::cout << " - Closing libusb handler..." << std::endl;
		libusb_close(handler);	// causes libusb_handle_events to exit
		handler = NULL;
	}

	
	std::cout << " - Waiting for Panda::Usb to close..." << std::endl;
	WaitForInternalThreadToExit();
	std::cout << " - Done!" << std::endl;
}

// Thread entry:
void Usb::entryAction() {


}

// Thread action:
void Usb::doAction() {
	if (mode == MODE_SYNCHRONOUS) {
		// No need to perform any libusb_handle_events since sync functions do that already
		pause();
		return;
	}

	int status;
//  libusb API recommends doing something like the following, however doAction is a threaded,
//	effective infinite loop so it's not necessary.  Copied for convenience:
//	int completed = 0;
//	while (!completed) {
//		status = libusb_handle_events_completed(NULL, &completed);
//	}
//	status = libusb_handle_events(NULL);
	struct timeval timeout;
	timeout.tv_usec = 1000;
	timeout.tv_sec = 0;
	status = libusb_handle_events_timeout(NULL, &timeout);	// A timeout of 0 noticeably drives up CPU usage (from 90%->130%)

	if (status != LIBUSB_SUCCESS) {
		std::cerr << "ERROR: libusb_handle_events() != LIBUSB_SUCCESS, exiting" << std::endl;
		printError(status);
		// Exit cleanly: TODO
		//this->stopRecording();
	}
}

/*
 Libusb debug
 */
void Usb::printError(int status) {
	switch (status) {
		case LIBUSB_ERROR_IO:
			std::cerr << " - LIBUSB_ERROR_IO" << std::endl;
			break;
		case LIBUSB_ERROR_INVALID_PARAM:
			std::cerr << " - LIBUSB_INVALID_PARAM" << std::endl;
			break;
		case LIBUSB_ERROR_ACCESS:
			std::cerr << " - LIBUSB_ERROR_ACCESS" << std::endl;
			break;
		case LIBUSB_ERROR_NO_DEVICE:
			std::cerr << " - LIBUSB_ERROR_NO_DEVICE" << std::endl;
			break;
		case LIBUSB_ERROR_NOT_FOUND:
			std::cerr << " - LIBUSB_ERROR_NOT_FOUND" << std::endl;
			break;
		case LIBUSB_ERROR_BUSY:
			std::cerr << " - LIBUSB_ERROR_BUSY" << std::endl;
			break;
		case LIBUSB_ERROR_TIMEOUT:
			std::cerr << " - LIBUSB_ERROR_TIMEOUT" << std::endl;
			break;
		case LIBUSB_ERROR_OVERFLOW:
			std::cerr << " - LIBUSB_ERROR_OVERFLOW" << std::endl;
			break;
		case LIBUSB_ERROR_PIPE:
			std::cerr << " - LIBUSB_ERROR_PIPE" << std::endl;
			break;
		case LIBUSB_ERROR_INTERRUPTED:
			std::cerr << " - LIBUSB_ERROR_INTERRUPTED" << std::endl;
			break;
		case LIBUSB_ERROR_NO_MEM:
			std::cerr << " - LIBUSB_ERROR_NO_MEM" << std::endl;
			break;
		case LIBUSB_ERROR_NOT_SUPPORTED:
			std::cerr << " - LIBUSB_ERROR_NOT_SUPPORTED" << std::endl;
			break;
		case LIBUSB_ERROR_OTHER:
			std::cerr << " - LIBUSB_ERROR_OTHER" << std::endl;
			break;

		default:
			std::cerr << " - LIBUSB_ERROR, or some other libusb error code" << std::endl;
			break;
	}
}

void Usb::printErrorTransfer(libusb_transfer_status status) {
	switch (status) {
		case LIBUSB_TRANSFER_COMPLETED:
			std::cerr << " - LIBUSB_TRANSFER_COMPLETED" << std::endl;
			break;
		case LIBUSB_TRANSFER_ERROR:
			std::cerr << " - LIBUSB_TRANSFER_ERROR" << std::endl;
			break;
		case LIBUSB_TRANSFER_TIMED_OUT:
			std::cerr << " - LIBUSB_TRANSFER_TIMED_OUT" << std::endl;
			break;
		case LIBUSB_TRANSFER_CANCELLED:
			std::cerr << " - LIBUSB_TRANSFER_CANCELLED" << std::endl;
			break;
		case LIBUSB_TRANSFER_STALL:
			std::cerr << " - LIBUSB_TRANSFER_STALL" << std::endl;
			break;
		case LIBUSB_TRANSFER_NO_DEVICE:
			std::cerr << " - LIBUSB_TRANSFER_NO_DEVICE" << std::endl;
			break;
		case LIBUSB_TRANSFER_OVERFLOW:
			std::cerr << " - LIBUSB_TRANSFER_OVERFLOW" << std::endl;
			break;
	}
}


/*
 UART handling:
 */
void Usb::requestUartData() {
	//std::cout << "In requestUartData()" << std::endl;
	int status;
	switch (mode) {
		case MODE_SYNCHRONOUS:
			lock();	// This is not for libusb, but for bufferSynchronousUart
			bufferLengthSynchronousUart = libusb_control_transfer(handler,
																  REQUEST_TYPE_IN,
																  REQUEST_UART_READ,
																  UART_DEVICE_GPS,
																  0,	// no index needed for UART command
																  bufferSynchronousUart,
																  UART_BUFFER_READ_LENGTH,
																  TIMEOUT);
			// No callbacks are performed, so perform notification now:
			processNewUartRead((char*)bufferSynchronousUart, bufferLengthSynchronousUart);
			unlock();
			break;

		case MODE_ASYNCHRONOUS:
			if((status = asyncControlTransfer(handler,
											  REQUEST_TYPE_IN,
											  REQUEST_UART_READ,
											  UART_DEVICE_GPS,
											  0,	// no index needed for UART command
											  NULL,	// will allocate/free buffer automagically
											  UART_BUFFER_READ_LENGTH,
											  TIMEOUT,
											  &transferCallbackReadUart,
											  (void*)this) ) != LIBUSB_SUCCESS) {
				std::cerr << "Error in Usb::requestUartData() asyncControlTransfer()" << std::endl;
				printError(status);
			}
			break;
		case MODE_ISOCHRONOUS:
			//#define ENDPOINT_UART_IN (0x80)
			if((status = isocControlTransfer(handler,
											 0x82,//ENDPOINT_UART_IN,
											 NULL,
											 UART_BUFFER_READ_LENGTH,
											 0,	// no index needed for UART command
											 &transferCallbackReadUart,
											 (void*)this) != LIBUSB_SUCCESS)) {
				std::cerr << "Error in Usb::requestUartData() isocControlTransfer()" << std::endl;
				printError(status);
			}

			break;
	}
}

void Usb::uartWrite(const char* buffer, int length) {
	// In the Panda code, they send 32 byte chunks, and prepend the data with a 1.
	// they also make use of bulkWrite(), which does the following sanitizing before a bulkTransfer():
	// endpoint = (endpoint & ~ENDPOINT_DIR_MASK) | ENDPOINT_OUT
	// This is sent to the eventual libusb_bulk_transfer;

	char bufferPrepended[UART_BUFFER_WRITE_LENGTH+1] = "\x01";
	int sentLength = 0;
	int remainingLength = length;
	//int length = strlen(buffer);
	int lengthThisTransfer;
	int status;

	for (int i = 0; i < length; i += UART_BUFFER_WRITE_LENGTH) {
		lengthThisTransfer = UART_BUFFER_WRITE_LENGTH > remainingLength ? remainingLength : UART_BUFFER_WRITE_LENGTH;
		memcpy(&bufferPrepended[1], &buffer[i], lengthThisTransfer);

		switch (mode) {
			case MODE_SYNCHRONOUS:
				lock();
				if((status = libusb_bulk_transfer(handler,
												  ENDPOINT_UART_OUT,
												  (unsigned char*)bufferPrepended,
												  lengthThisTransfer + 1,
												  &sentLength,
												  TIMEOUT)) < 0) {
					std::cerr << "ERROR: uartWrite() libusb_bulk_transfer() reutrned status:" << std::endl;
					printError(status);
				}
				unlock();
				break;

			case MODE_ASYNCHRONOUS:
				if((status = asyncBulkTransfer(handler,
											   ENDPOINT_UART_OUT,
											   (unsigned char*)bufferPrepended,
											   lengthThisTransfer + 1,
											   TIMEOUT,
											   LIBUSB_TRANSFER_TYPE_BULK,
											   &transferCallbackSendUart,
											   (void*)this)) != LIBUSB_SUCCESS) {

				}
				sentLength = UART_BUFFER_WRITE_LENGTH+1;	// HACK: this is not read here, it is reported in the callback
				break;

			case MODE_ISOCHRONOUS:
				break;
		}

		remainingLength -= sentLength-1;	// HACK: this may not be effective in asynchronous mode
	}
}









PandaRtcTimestamp Usb::getRtc() {
	PandaRtcTimestamp rtcTime;
	readPandaHardwareSimple( REQUEST_RTC_FULL, (unsigned char*)&rtcTime, sizeof(rtcTime));
	return rtcTime;
}

void Usb::setRtcYear( int n ) {
	sendPandaHardwareSimple( REQUEST_RTC_YEAR, n, 0);
}
void Usb::setRtcMonth( int n ) {
	sendPandaHardwareSimple( REQUEST_RTC_MONTH, n, 0);
}
void Usb::setRtcDay( int n ) {
	sendPandaHardwareSimple( REQUEST_RTC_DAY, n, 0);
}
void Usb::setRtcWeekday( int n ) {
	sendPandaHardwareSimple( REQUEST_RTC_WEEKDAY, n, 0);
}
void Usb::setRtcHour( int n ) {
	sendPandaHardwareSimple( REQUEST_RTC_HOUR, n, 0);
}
void Usb::setRtcMinute( int n ) {
	sendPandaHardwareSimple( REQUEST_RTC_MINUTE, n, 0);
}
void Usb::setRtcSeconds( int n ) {
	sendPandaHardwareSimple( REQUEST_RTC_SECOND, n, 0);
}

void Usb::setIrPower( bool enable) {
	sendPandaHardwareSimple( REQUEST_IR_POWER, enable, 0);
}

void Usb::setFanPower(	int power ) {
	sendPandaHardwareSimple( REQUEST_SET_FAN_POWER, power, 0);
}

uint16_t Usb::getFanRpm() {
	int fanSpeed;
	readPandaHardwareSimple( REQUEST_FAN_SPEED, (unsigned char*)&fanSpeed, sizeof(fanSpeed));
	return fanSpeed;
}

void Usb::setPhonePower( bool enable ) {
	sendPandaHardwareSimple( REQUEST_SET_PHONE_POWER, enable, 0);
}

void Usb::getCanDebug() {
	sendPandaHardwareSimple( REQUEST_CAN_DEBUG, 0, 0);
}

PandaHardwareType Usb::getHardware() {
	unsigned char hardware;
	readPandaHardwareSimple( REQUEST_HARDWARE, &hardware, 1);
	return (PandaHardwareType)hardware;
}

void Usb::getPandaSerial( unsigned char* serial16 ) {
	readPandaHardwareSimple( REQUEST_SERIAL, serial16, 16);
}

void Usb::enterBootloaderMode( unsigned char mode ) {
	//sendPandaHardwareSimple( REQUEST_BOOTLOADER, mode, 0);
}

void Usb::getHealth( PandaHealth* health ) {
	readPandaHardwareSimple( REQUEST_CAN_HEALTH, (unsigned char*)health, sizeof(*health));
}

void Usb::getFirmware( unsigned char* firmware128 ) {
	readPandaHardwareSimple( REQUEST_FIRMWARE_LOW, firmware128, 64);
	readPandaHardwareSimple( REQUEST_FIRMWARE_HIGH, &firmware128[64], 64);
}

void Usb::getGitVersion( unsigned char* version ) {
	int length = readPandaHardwareSimple( REQUEST_GIT_VERSION, version, 64);
	if (length < 64) {
		version[length] = 0x00;	// NULL terminate
	}
}

void Usb::systemReset() {
	sendPandaHardwareSimple( REQUEST_RESET_ST, 0, 0);
}

void Usb::setEspPower( bool enable) {
	sendPandaHardwareSimple( REQUEST_ESP_POWER, enable, 0);
}

void Usb::espReset(int uart, int bootmode) {
	sendPandaHardwareSimple( REQUEST_ESP_RESET, bootmode, 0);
	usleep(200000);
}

void Usb::setGmlanOrObdCanMode(unsigned char gmlan) {
	sendPandaHardwareSimple( REQUEST_GMLAN_CAN_ENABLE, gmlan, 0);
}

void Usb::setSafetyMode(uint16_t mode, uint16_t params) {
	// comma.ai code has a discrepency in the two following calls:
//	sendPandaHardwareSimple(REQUEST_TYPE_WRITE, REQUEST_SAFETY_MODE, mode, 0);	// board.cc has this
//	sendPandaHardwareSimple( REQUEST_SAFETY_MODE, mode, 73);	// python Panda class has this
	sendPandaHardwareSimple( REQUEST_SAFETY_MODE, mode, params);	// python Panda class has this
	// the value 73 comes from openpilot/selfdrive/car/toyota/interface.py in the RAV4 section, ret.safetyParam
}

void Usb::getHealthAndCanVersions( unsigned char* healthVersion, unsigned char* canVersion) {
	unsigned char response[2];
	readPandaHardwareSimple( REQUEST_CAN_HEALTH_VERSIONS, response, 2);
	*healthVersion = response[0];
	*canVersion = response[1];
}

void Usb::setCanBitrate( int bus, int baud) {
	sendPandaHardwareSimple( REQUEST_SET_CAN_BAUD, bus, baud);
}

void Usb::setAlternativeExperience( int alternativeExperience) {
	sendPandaHardwareSimple( REQUEST_SET_ALT_EXP, alternativeExperience, 0);
}

// request uart data is handled elsewhere in this file

void Usb::setUartBaud(int uart, int baud) {
	setUartParity(uart, 0);
	sendPandaHardwareSimple( REQUEST_UART_BAUD_EXT, uart, baud/300);
}

void Usb::setUartParity(int uart, int parity) {
	sendPandaHardwareSimple( REQUEST_UART_PARITY, uart, parity);
}

void Usb::setUartBaudSmall( int uart, int baud ) {	// untested
	setUartParity(uart, 0);
	sendPandaHardwareSimple( REQUEST_UART_BAUD, uart, baud); // untested
}

void Usb::setCanLoopback( int enable ) {
	sendPandaHardwareSimple( REQUEST_CAN_LOOPBACK, enable, 0);
}

void Usb::setUsbPowerMode( unsigned char mode) {
	sendPandaHardwareSimple( REQUEST_USB_POWER_MODE, mode, 0);
}

void Usb::setPowerSaveEnable( bool enable ) {
	sendPandaHardwareSimple( REQUEST_USB_POWER_SAVE, enable, 0);
}

void Usb::setKLlineWakupPulse( unsigned char KL ) {
	sendPandaHardwareSimple( REQUEST_K_L_PULSE, KL, 0);
}

void Usb::canPurge() {
	sendPandaHardwareSimple( REQUEST_CAN_RING_CLEAR, CAN_DEVICE_RX, 0);
	sendPandaHardwareSimple( REQUEST_CAN_RING_CLEAR, CAN_DEVICE_TX_1, 0);
	sendPandaHardwareSimple( REQUEST_CAN_RING_CLEAR, CAN_DEVICE_TX_2, 0);
	sendPandaHardwareSimple( REQUEST_CAN_RING_CLEAR, CAN_DEVICE_TX_3, 0);
	sendPandaHardwareSimple( REQUEST_CAN_RING_CLEAR, CAN_DEVICE_TXGMLAN, 0);
}

void Usb::uartPurge() {
	//	comma.ai code calls this "clearing the ring buffer."  It could do more than just clear this UART's ring buffer.
	sendPandaHardwareSimple( REQUEST_UART_RING_CLEAR, UART_DEVICE_GPS, 0);	// TODO test this
}

void Usb::sendHeartBeat() {
	//	the panda has a heartbeat counter, calling this should reset it
	// The panda has a timer running at 8Hz.  A loop counter decimates this to 1Hz.
	// In the 1 Hz loop, the heartbeat counter will increment.
	// If the hearbeat is greater than: (check_started() ? EON_HEARTBEAT_IGNITION_CNT_ON : EON_HEARTBEAT_IGNITION_CNT_OFF)
	// or: (check_started() ? 5U : 2U)
	// then the panda will enter a SAFETY_SILENT mode
//	sendPandaHardwareSimple( REQUEST_HEARTBEAT, 0, 0);	// TODO test this
    sendPandaHardwareSimple( REQUEST_HEARTBEAT, 1, 0);    // TODO test this
}

void Usb::setKLlineFiveBaudInit( int config, int baud ) {
	sendPandaHardwareSimple( REQUEST_KL_LINE_BAUD, config, baud);
}

void Usb::setClockSourceMode( int mode ) {
	sendPandaHardwareSimple( REQUEST_CLOCK_SOURCE, mode, 0);
}

void Usb::setSiren(bool enable) {
	sendPandaHardwareSimple( REQUEST_SIREN, enable, 0);
}

void Usb::setGreenLed(bool enable) {
	sendPandaHardwareSimple( REQUEST_GREEN_LED, enable, 0);
}

void Usb::disableHeartbeat() {
	sendPandaHardwareSimple( REQUEST_DISABLE_HEARTBEAT, 0, 0);
}

void Usb::setCanFdBaud( int bus, int baud ) {
	sendPandaHardwareSimple( REQUEST_CAN_FD_BAUD, bus, baud);
}

void Usb::getCanFdEnabled( int bus, bool* fdEnabled, bool* brsEnabled ) {
	unsigned char response[2];
	readPandaHardwareSimple( REQUEST_CAN_FD_ENABLED, response, 2, bus);
	*fdEnabled = response[0];
	*brsEnabled = response[1];
}

void Usb::enterDeepSleep() {
	sendPandaHardwareSimple( REQUEST_DEEP_SLEEP, 0, 0);
	
}



//void Usb::sendPandaHardwareSimple(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index) {
//int status = libusb_control_transfer(handler, requestType, request, value, index, NULL, 0, TIMEOUT);
void Usb::sendPandaHardwareSimple(uint8_t request, uint16_t value, uint16_t index) {
	int status = libusb_control_transfer(handler, REQUEST_TYPE_OUT, request, value, index, NULL, 0, TIMEOUT);
	if (status < 0) {
		std::cerr << " FAILED: sendPandaHardwareSimple() libusb_control_transfer error" << std::endl;
		printError(status);
	}
}

//void Usb::readPandaHardwareSimple(uint8_t requestType, uint8_t request, unsigned char* buffer, uint16_t length) {
//	int status = libusb_control_transfer(handler, requestType, request, 0, 0, buffer, length, TIMEOUT);
int Usb::readPandaHardwareSimple(uint8_t request, unsigned char* buffer, uint16_t length, uint16_t value, uint16_t index) {
	int status = libusb_control_transfer(handler, REQUEST_TYPE_IN, request, value, index, buffer, length, TIMEOUT);
	if (status < 0) {
		std::cerr << " FAILED: readPandaHardwareSimple() libusb_control_transfer error" << std::endl;
		printError(status);
	}
	return status;
}


bool Usb::hasGpsSupport() {
	return hasGps;
}

const char* Panda::safetyModelToString( int safetyModel ) {
	switch (safetyModel) {
//			// Before comma.ai changed values:
//		case SAFETY_NOOUTPUT: return "NOOUTPUT";
//		case SAFETY_HONDA: return "HONDA";
//		case SAFETY_TOYOTA: return "TOYOTA";
//		case SAFETY_ELM327: return "ELM327";
//		case SAFETY_GM: return "GM";
//		case SAFETY_HONDA_BOSCH: return "HONDA_BOSCH";
//		case SAFETY_FORD: return "FORD";
//		case SAFETY_CADILLAC: return "CADILLAC";
//		case SAFETY_HYUNDAI: return "HYUNDAI";
//		case SAFETY_CHRYSLER: return "CHRYSLER";
//		case SAFETY_TESLA: return "TESLA";
//		case SAFETY_SUBARU: return "SUBARU";
//		case SAFETY_GM_PASSIVE: return "GM_PASSIVE";
//		case SAFETY_MAZDA: return "MAZDA";
//		case SAFETY_TOYOTA_IPAS: return "TOYOTA_IPAS";
//		case SAFETY_ALLOUTPUT: return "ALLOUTPUT";
//		case SAFETY_GM_ASCM: return "GM_ASCM";
			
			// Sart of Panda Red support:
		case SAFETY_SILENT: return "SILENT";
		case SAFETY_HONDA_NIDEC: return "HONDA_NIDEC";
		case SAFETY_TOYOTA: return "TOYOTA";
		case SAFETY_ELM327: return "ELM327";
		case SAFETY_GM: return "GM";
		case SAFETY_HONDA_BOSCH_GIRAFFE: return "HONDA_BOSCH_GIRAFFE";
		case SAFETY_FORD: return "FORD";
		case SAFETY_HYUNDAI: return "HYUNDAI";
		case SAFETY_CHRYSLER: return "CHRYSLER";
		case SAFETY_TESLA: return "TESLA";
		case SAFETY_SUBARU: return "SUBARU";
		case SAFETY_MAZDA: return "MAZDA";
		case SAFETY_NISSAN: return "NISSAN";
		case SAFETY_VOLKSWAGEN_MQB: return "VOLKSWAGEN_MQB";
		case SAFETY_ALLOUTPUT: return "ALLOUTPUT";
		case SAFETY_GM_ASCM: return "GM_ASCM";
		case SAFETY_NOOUTPUT: return "NOOUTPUT";
		case SAFETY_HONDA_BOSCH: return "HONDA_BOSCH";
		case SAFETY_VOLKSWAGEN_PQ: return "VOLKSWAGEN_PQ";
		case SAFETY_SUBARU_LEGACY: return "SUBARU_LEGACY";
		case SAFETY_HYUNDAI_LEGACY: return "HYUNDAI_LEGACY";
		case SAFETY_HYUNDAI_COMMUNITY: return "HYUNDAI_COMMUNITY";
		case SAFETY_STELLANTIS: return "STELLANTIS";
		case SAFETY_FAW: return "FAW";
		case SAFETY_BODY: return "BODY";
		case SAFETY_CIRCLES_NISSAN: return "CIRCLES_NISSAN";
		default:
			break;
	}
	return "Unrecognized :(";
}

const char* Panda::carHarnessStatusToString( int car_harness_status) {
	switch (car_harness_status) {
		case HARNESS_STATUS_NC: return "HARNESS_STATUS_NC";
		case HARNESS_STATUS_NORMAL: return "HARNESS_STATUS_NORMAL";
		case HARNESS_STATUS_FLIPPED: return "HARNESS_STATUS_FLIPPED";
			
		default:
			break;
	}
	return "Unrecognized :(";

}

const char* Panda::faultStatusToString( int fault_status ) {
	switch (fault_status) {
		case FAULT_STATUS_NONE: return "FAULT_STATUS_NONE";
		case FAULT_STATUS_TEMPORARY: return "FAULT_STATUS_TEMPORARY";
		case FAULT_STATUS_PERMANENT: return "FAULT_STATUS_PERMANENT";
			
		default:
			break;
	}
	return "Unrecognized :(";
	
}

std::string Panda::faultsToString( int faults ) {
	std::stringstream result;
	if ( faults & FAULT_RELAY_MALFUNCTION) result << "FAULT_RELAY_MALFUNCTION, ";
	if ( faults & FAULT_UNUSED_INTERRUPT_HANDLED) result << "FAULT_UNUSED_INTERRUPT_HANDLED, ";
	if ( faults & FAULT_INTERRUPT_RATE_CAN_1) result << "FAULT_INTERRUPT_RATE_CAN_1, ";
	if ( faults & FAULT_INTERRUPT_RATE_CAN_2) result << "FAULT_INTERRUPT_RATE_CAN_2, ";
	if ( faults & FAULT_INTERRUPT_RATE_CAN_3) result << "FAULT_INTERRUPT_RATE_CAN_3, ";
	if ( faults & FAULT_INTERRUPT_RATE_TACH) result << "FAULT_INTERRUPT_RATE_TACH, ";
	if ( faults & FAULT_INTERRUPT_RATE_GMLAN) result << "FAULT_INTERRUPT_RATE_GMLAN, ";
	if ( faults & FAULT_INTERRUPT_RATE_INTERRUPTS) result << "FAULT_INTERRUPT_RATE_INTERRUPTS, ";
	if ( faults & FAULT_INTERRUPT_RATE_SPI_DMA) result << "FAULT_INTERRUPT_RATE_SPI_DMA, ";
	if ( faults & FAULT_INTERRUPT_RATE_SPI_CS) result << "FAULT_INTERRUPT_RATE_SPI_CS, ";
	if ( faults & FAULT_INTERRUPT_RATE_UART_1) result << "FAULT_INTERRUPT_RATE_UART_1, ";
	if ( faults & FAULT_INTERRUPT_RATE_UART_2) result << "FAULT_INTERRUPT_RATE_UART_2, ";
	if ( faults & FAULT_INTERRUPT_RATE_UART_3) result << "FAULT_INTERRUPT_RATE_UART_3, ";
	if ( faults & FAULT_INTERRUPT_RATE_UART_5) result << "FAULT_INTERRUPT_RATE_UART_5, ";
	if ( faults & FAULT_INTERRUPT_RATE_UART_DMA) result << "FAULT_INTERRUPT_RATE_UART_DMA, ";
	if ( faults & FAULT_INTERRUPT_RATE_USB) result << "FAULT_INTERRUPT_RATE_USB, ";
	if ( faults & FAULT_INTERRUPT_RATE_TIM1) result << "FAULT_INTERRUPT_RATE_TIM1, ";
	if ( faults & FAULT_INTERRUPT_RATE_TIM3) result << "FAULT_INTERRUPT_RATE_TIM3, ";
	if ( faults & FAULT_REGISTER_DIVERGENT) result << "FAULT_REGISTER_DIVERGENT, ";
	if ( faults & FAULT_INTERRUPT_RATE_KLINE_INIT) result << "FAULT_INTERRUPT_RATE_KLINE_INIT, ";
	if ( faults & FAULT_INTERRUPT_RATE_CLOCK_SOURCE) result << "FAULT_INTERRUPT_RATE_CLOCK_SOURCE, ";
//	if ( faults & FAULT_INTERRUPT_RATE_TIM9) result << "FAULT_INTERRUPT_RATE_TIM9";	// deprecated from older panda firmware
	if ( faults & FAULT_INTERRUPT_RATE_TICK) result << "FAULT_INTERRUPT_RATE_TICK, ";	// Added for recent firmware
	if ( faults & FAULT_INTERRUPT_RATE_EXTI) result << "FAULT_INTERRUPT_RATE_EXTI, ";	// Added for recent firmware
	

	return result.str();
}

const char* Panda::hardwareTypeToString( PandaHardwareType hardwareType ) {
	switch (hardwareType) {
		case HARDWARE_WHITE_PANDA: return "White Panda";
		case HARDWARE_GREY_PANDA: return "Grey Panda";
		case HARDWARE_BLACK_PANDA: return "Black Panda";
		case HARDWARE_UNO: return "Uno";
		case HARDWARE_PEDAL: return "Pedal";
		case HARDWARE_DOS: return "Dos";
		case HARDWARE_RED_PANDA: return "Red Panda";
		case HARDWARE_UNKNOWN:
			break;
	}
	return "Unknown Hardware";
}

const char* Panda::usbPowerModeToString( int mode ) {
	switch (mode) {
		case USB_POWER_NONE: return "USB_POWER_NONE";
		case USB_POWER_CLIENT: return "USB_POWER_CLIENT";
		case USB_POWER_CDP: return "USB_POWER_CDP";
		case USB_POWER_DCP: return "USB_POWER_DDP";
			
		default:
			break;
	}
	return "Unrecognized :(";
	
}

void Panda::printPandaHealth( const PandaHealth& health ) {
	std::cout << "--------Health---------" << std::endl;
	std::cout << "uptime                :" << health.uptime << std::endl;
	std::cout << "voltage               :" << (double)health.voltage/1000.0 << std::endl;
	std::cout << "current               :" << health.current << std::endl;
	std::cout << "can_rx_errors         :" << health.can_rx_errs << std::endl;
	std::cout << "can_send_errors       :" << health.can_send_errs << std::endl;
	std::cout << "can_fwd_errors        :" << health.can_fwd_errs << std::endl;
	std::cout << "gmlan_send_errs       :" << health.gmlan_send_errs << std::endl;
	std::cout << "faults                :" << Panda::faultsToString(health.faults) << std::endl;
	std::cout << "ignition_line         :" << (int)health.ignition_line << std::endl;
	std::cout << "ignition_can          :" << (int)health.ignition_can << std::endl;
	std::cout << "controls_allowed      :" << (int)health.controls_allowed << std::endl;
	std::cout << "gas_int_detected      :" << (int)health.gas_interceptor_detected << std::endl;
	std::cout << "car_harness_status    :" << Panda::carHarnessStatusToString(health.car_harness_status) << std::endl;
	std::cout << "usb_power_mode        :" << Panda::usbPowerModeToString(health.usb_power_mode) << std::endl;
	std::cout << "Safety Model          :" << Panda::safetyModelToString(health.safety_mode) << std::endl;
	std::cout << "Safety Param          :" << (int)health.safety_param << std::endl; // In health packet version 7
	std::cout << "fault_status          :" << Panda::faultStatusToString(health.fault_status) << std::endl;
	std::cout << "power_save_enabled    :" << (int)health.power_save_enabled << std::endl;
	std::cout << "Heartbeat Lost        :" << (int)health.heartbeat_lost << std::endl; // In health packet version 7
	std::cout << "Alternative Experience:" << (int)health.alternative_experience << std::endl; // In health packet version 7
	std::cout << "Blocked Message Count :" << (int)health.blocked_msg_cnt << std::endl; // In health packet version 7
	std::cout << "Interrup Load         :" << (float)health.interrupt_load << std::endl; // In health packet version 7
	std::cout << "-------------------" << std::endl;
}
