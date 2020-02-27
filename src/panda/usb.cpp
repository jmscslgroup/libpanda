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
#include <cstring>
#include <unistd.h>

#define TIMEOUT (0)

using namespace Panda;

Usb::Usb(UsbMode mode)
:mode(mode), handler(NULL) {

}

Usb::~Usb() {
//	int status;

	stop();
//	if((status = libusb_release_interface(handler, 0) != LIBUSB_SUCCESS)) {
//		std::cerr << "FAILED: Usb::end() libusb_release_interface() had an error:" << std::endl;
//		printError(status);
//	};
//	if (handler != NULL) {
//		libusb_close(handler);
//	}
	WaitForInternalThreadToExit();

//	if (transfer != NULL) {
//		libusb_cancel_transfer(transfer);	// incase it is busy
//		libusb_free_transfer(transfer);
//	}
//	if (outgoingTransfer != NULL) {
//		libusb_cancel_transfer(outgoingTransfer);	// incase it is busy
//		libusb_free_transfer(outgoingTransfer);
//	}

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
	if(openDeviceByManufacturer(devices, "comma.ai") != 0 ) {
		std::cerr << "FAILED to open \"comma.ai\", is it plugged in?" << std::endl;
		std::cerr << " - May need root privileges if failed to open" << std::endl;
		exit(EXIT_FAILURE);
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

	std::cout << " - Setting Safety:" << std::endl;
	setSafetyMode(SAFETY_TOYOTA);	// OBD II port
//
	std::cout << " - Enabling CAN Loopback:" << std::endl;
	setCanLoopback(true);

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
		if(libusb_open(device, &handler) == 0) {
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
 Libusb converted commands: truncated synchronous to become asynchronous:
 */

// This is a truncation of a direct copy from libusb's sync.c libusb_control_transfer()
// Only the first half is performed, up to a transfer sumbission, thus letting the libusb_handle_event()
// handle the transfers in an aynchronous fashion.
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

// Again, the below is a truncated version of libusb's do_sync_bulk_transfer
int asyncBulkTransfer(struct libusb_device_handle *dev_handle,
					  unsigned char endpoint, unsigned char *buffer, int length, unsigned int timeout, unsigned char type, libusb_transfer_cb_fn callback, void *user_data)
{
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	//int completed = 0;
	int r;

	if (!transfer) {
		return LIBUSB_ERROR_NO_MEM;
	}

	libusb_fill_bulk_transfer(transfer, dev_handle, endpoint, buffer, length,
							  callback, user_data, timeout);
	transfer->type = type;

	r = libusb_submit_transfer(transfer);
	if (r < 0) {
		libusb_free_transfer(transfer);
		return r;
	}
	// <- Truncated here
	return LIBUSB_SUCCESS;
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
	return LIBUSB_SUCCESS;
}


/*
 CAN data handling
 */


int Usb::sendCanData( unsigned char* buffer, int length) {
//	int status;
//	struct libusb_transfer* outgoingTransfer = libusb_alloc_transfer(0);
//	if (outgoingTransfer == NULL) {
//		std::cerr << "ERROR: libusb_alloc_transfer() for outgoingTransfer returned NULL" << std::endl;
//	}
//	//lock();	// thread safety
//	// "libusb_bulk_transfer(dev_handle, 3, (uint8_t*)send, msg_count*0x10, &sent, TIMEOUT)"
//	// comma.ai code uses the endpoint setting of 3
//	libusb_fill_bulk_transfer(	outgoingTransfer,
//							  	handler,
//							  	ENDPOINT_CAN_OUT,// LIBUSB_ENDPOINT_OUT | 0x02, // ENDPOINT_WRITE maybe?
//							  	buffer,
//								length,
//							  	Usb::transferCallbackSendCan, (void*)this,
//							  	0);
//
//	status = libusb_submit_transfer(outgoingTransfer);
//	//unlock();	// thread safety
//
//	if(status != LIBUSB_SUCCESS) {
//		std::cerr << "ERROR: libusb_submit_transfer(outgoingTransfer) unsuccessful" << std::endl;
//		printError(status);
//		// Fail gracfully somehow?
//		//exit(EXIT_FAILURE);
//
//		return -1;
//	}
//
//	//resume();	// should handle events
//	return 0;
	int status, actualLength;
	switch (mode) {
		case MODE_SYNCHRONOUS:
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
			break;

		case MODE_ASYNCHRONOUS:
			if((status = asyncBulkTransfer(handler,
										   ENDPOINT_CAN_OUT,
										   buffer,
										   length,
										   0,
										   LIBUSB_TRANSFER_TYPE_BULK,
										   Usb::transferCallbackSendCan,
										   (void*)this)) != LIBUSB_SUCCESS) {
				std::cerr << "ERROR: Usb::sendCanData()->asyncBulkTransfer():" <<std::endl;
				printError(status);
				return -1;
			}

			break;

		case MODE_ISOCHRONOUS:	// TODO:

			break;

	}
	return 0;
}

void Usb::requestCanData() {
//	struct libusb_transfer* transfer = libusb_alloc_transfer(0); // num_iso_packets set to 0 for bulk per libusb1 API
//	if (transfer == NULL) {
//		std::cerr << "ERROR: libusb_alloc_transfer() for transfer returned NULL" << std::endl;
//	}
//	//	lock();	// thread safety
//	// I am uncomfortable with the following
//	// Rahul's code had usb1.ENDPOINT_IN | self.ENDPOINT_READ
//	// LIBUSB_ENDPOINT_IN is already defined.  not sure about the bitwise OR with Rahul's defined ENDPOINT_READ
//	// the python code uses setBulk(), which is a simple wrapper for the following
//	// My goal is to port Rahul's code first, then figure these issues out later
//	// Note: comma.ai uses an endpoint value of 0x81 for reads.  Thats the same as what's used here
//	libusb_fill_bulk_transfer(transfer,
//							  handler,
//							  ENDPOINT_CAN_IN,
//							  bufferSynchronousCan,
//							  sizeof(bufferSynchronousCan),
//							  Usb::transferCallbackReadCan, (void*)this,
//							  0);
//
//
//	int status = libusb_submit_transfer(transfer);
//	//	unlock();	// thread safety
//
//	if(status != LIBUSB_SUCCESS) {
//		std::cerr << "ERROR: libusb_submit_transfer() unsuccessful" << std::endl;
//		printError(status);
//		//	this->stopRecording();
//	}
	int status, actualLength;
	switch (mode) {
		case MODE_SYNCHRONOUS:
			if((status = libusb_bulk_transfer(handler,
											  ENDPOINT_CAN_IN,
											  bufferSynchronousCan,
											  sizeof(bufferSynchronousCan),
											  &actualLength,
											  TIMEOUT)) != LIBUSB_SUCCESS) {
				std::cerr << "ERROR: Usb::requestCanData()->libusb_bulk_transfer():" << std::endl;
				printError(status);
				return;
			}
			// No callbacks are called here, so perform notifications manually:
			if (actualLength > 0) {
				processNewCanRead((char*)bufferSynchronousCan, actualLength);
			}
			break;

		case MODE_ASYNCHRONOUS:
			if((status = asyncBulkTransfer(handler,
										   ENDPOINT_CAN_IN,
										   bufferSynchronousCan,
										   sizeof(bufferSynchronousCan),
										   0,
										   LIBUSB_TRANSFER_TYPE_BULK,
										   Usb::transferCallbackReadCan,
										   (void*)this)) != LIBUSB_SUCCESS) {
				std::cerr << "ERROR: Usb::requestCanData()->asyncBulkTransfer():" <<std::endl;
				printError(status);
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
		// be patient
		return;
	}

	This->processNewCanSend(transfer->actual_length);

	libusb_free_transfer(transfer);
}

void Usb::transferCallbackReadCan(struct libusb_transfer *transfer) {
	Usb* This = (Usb*) transfer->user_data;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		// be patient
		return;
	}
	if (transfer->actual_length > 0) {
		This->processNewCanRead((char*)transfer->buffer, transfer->actual_length);
	}

	libusb_free_transfer(transfer);
}

void Usb::transferCallbackSendUart(struct libusb_transfer *transfer) {
	Usb* This = (Usb*) transfer->user_data;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		return;
	}

	This->processNewUartSend(transfer->actual_length);

	libusb_free_transfer(transfer);
}

void Usb::transferCallbackReadUart(struct libusb_transfer *transfer) {
	Usb* This = (Usb*) transfer->user_data;
	//std::cout << "In Usb::transferCallbackReadUart()" << std::endl;
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		std::cout << " - Incomplete: transferCallbackReadUart()" << std::endl;
		return;
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

		//std::cout << " - Actual Length: " << transfer->actual_length << std::endl;
		if (transfer->actual_length > 0) {
//			This->processNewUartRead((char*)transfer->buffer, transfer->actual_length);
			This->processNewUartRead((char*)libusb_control_transfer_get_data(transfer), transfer->actual_length);
		}
	}

	libusb_free_transfer(transfer);
}

/*
 Observer Notifications
 */
void Usb::processNewCanRead(char* buffer, size_t length) {
	bytesReceivedCan += length;
	std::cout << "Read CAN data! Bytes:" << length << std::endl;
	for (std::vector<UsbListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		UsbListener* listener = *it;
		listener->notificationCanRead(buffer, length);
	}
}

void Usb::processNewCanSend(size_t length) {
	bytesSentCan += length;
	std::cout << "Sent CAN Data! Bytes:" << length << std::endl;
	for (std::vector<UsbListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		UsbListener* listener = *it;
		listener->notificationCanWrite();
	}
}

void Usb::processNewUartRead(char* buffer, size_t length) {
	bytesReceivedUart += length;
	for (std::vector<UsbListener*>::iterator it = listeners.begin(); it != listeners.end(); it++) {
		UsbListener* listener = *it;
		listener->notificationUartRead(buffer, length);
	}
}

void Usb::processNewUartSend(size_t length) {
	bytesSentUart += length;
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
	if((status = libusb_release_interface(handler, 0) != LIBUSB_SUCCESS)) {
		std::cerr << "FAILED: Usb::end() libusb_release_interface() had an error:" << std::endl;
		printError(status);
	};

	if (handler != NULL) {
		libusb_close(handler);
		handler = NULL;
	}
	WaitForInternalThreadToExit();
	//WaitForInternalThreadToExit();	// This cannot happen, since libusb_handle_events() needs an event or will block forever
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
	int completed = 0;
	status = libusb_handle_events_completed(NULL, &completed);

	if (status != LIBUSB_SUCCESS) {
		std::cerr << "ERROR: libusb_handle_events() != LIBUSB_SUCCESS, exiting" << std::endl;
		printError(status);
		// Exit cleanly:
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


/*
 UART handling:
 */
void Usb::requestUartData() {
	int status;
	switch (mode) {
		case MODE_SYNCHRONOUS:
			bufferLengthSynchronousUart = libusb_control_transfer(handler,
																  REQUEST_TYPE_IN,
																  REQUEST_UART_READ,
																  UART_DEVICE_GPS,
																  0,
																  bufferSynchronousUart,
																  UART_BUFFER_READ_LENGTH,
																  TIMEOUT);
			// No callbacks are performed, so perform notification now:
			processNewUartRead((char*)bufferSynchronousUart, bufferLengthSynchronousUart);
			break;

		case MODE_ASYNCHRONOUS:
			if((status = asyncControlTransfer(handler,
											  REQUEST_TYPE_IN,
											  REQUEST_UART_READ,
											  UART_DEVICE_GPS,
											  0,	// no index needed for UARt command
											  NULL,	// will allocate/free buffer automatically
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
											 0,
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
				if((status = libusb_bulk_transfer(handler,
												  ENDPOINT_UART_OUT,
												  (unsigned char*)bufferPrepended,
												  lengthThisTransfer + 1,
												  &sentLength,
												  0)) < 0) {
					std::cerr << "ERROR: uartWrite() libusb_bulk_transfer() reutrned status:" << std::endl;
					printError(status);
				}
				break;

			case MODE_ASYNCHRONOUS:
				if((status = asyncBulkTransfer(handler,
											   ENDPOINT_UART_OUT,
											   (unsigned char*)bufferPrepended,
											   lengthThisTransfer+1,
											   0,
											   LIBUSB_TRANSFER_TYPE_BULK,
											   &transferCallbackSendUart,
											   (void*)this)) != LIBUSB_SUCCESS) {

				}
				sentLength = UART_BUFFER_WRITE_LENGTH+1;	// HACK: this is not read here, it is reported int eh callback
				break;

			case MODE_ISOCHRONOUS:
				break;
		}

		remainingLength -= sentLength-1;	// HACK: this may not be effective in asynchronous mode
	}
}

void Usb::uartPurge() {
//	unsigned char dummybuffer[10000];
//	uartRead(dummybuffer);
	sendPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_UART_RING_CLEAR, UART_DEVICE_GPS, 0);	// TODO test this
}

//int Usb::uartRead(unsigned char* buffer) {
////	def serial_read(self, port_number):
////	ret = []
////	while 1:
////		lret = bytes(self._handle.controlRead(Panda.REQUEST_IN, 0xe0, port_number, 0, 0x40))
////		if len(lret) == 0:
////			break
////		ret.append(lret)
////	return b''.join(ret)
//	int runningLength = 0, lastReadlength = 0;
//
////	lock();	// thread safety
//	while ((lastReadlength = libusb_control_transfer(handler, REQUEST_TYPE_IN, REQUEST_UART_READ, 1, 0, buffer+runningLength, 0x40, 0)) > 0) {
//		runningLength += lastReadlength;
//	}
////	unlock();	// thread safety
//
//	return runningLength;
//}


void Usb::sendPandaHardwareSimple(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index) {
	int status = libusb_control_transfer(handler, requestType, request, value, index, NULL, 0, 0);
	if (status < 0) {
		std::cerr << " FAILED: sendPandaHardwareSimple() libusb_control_tranfer error" << std::endl;
		printError(status);
	}
}


void Usb::readPandaHardwareSimple(uint8_t requestType, uint8_t request, unsigned char* buffer, uint16_t length) {
	int status = libusb_control_transfer(handler, requestType, request, 0, 0, buffer, length, 0);
	if (status < 0) {
		std::cerr << " FAILED: readPandaHardwareSimple() libusb_control_tranfer error" << std::endl;
		printError(status);
	}
}

void Usb::setUartBaud(int uart, int baud) {
	setUartParity(uart, 0);
	sendPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_UART_BAUD, uart, baud/300);
}

void Usb::setUartParity(int uart, int parity) {
	sendPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_UART_PARITY, uart, parity);
}

/*
 Device handling:
 */
void Usb::setEspPower( bool enable) {
	sendPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_ESP_POWER, enable, 0);
}

void Usb::espReset(int uart, int bootmode) {
	sendPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_ESP_RESET, bootmode, 0);
	usleep(200000);
}

void Usb::getPandaSerial( unsigned char* serial16 ) {
//	libusb_control_transfer(handler, REQUEST_TYPE_OUT, REQUEST_SERIAL, 0, 0, firmware16, 64, 0);
	readPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_SERIAL, serial16, 16);
}


void Usb::setCanLoopback( int enable ) {
	sendPandaHardwareSimple(REQUEST_TYPE_OUT,REQUEST_CAN_LOOPBACK, enable, 0);
}

void Usb::getFirmware( unsigned char* firmware128 ) {
	readPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_FIRMWARE_LOW, firmware128, 64);
	readPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_FIRMWARE_HIGH, &firmware128[64], 64);
//	libusb_control_transfer(handler, REQUEST_TYPE_OUT, REQUEST_FIRMWARE_LOW, 0, 0, firmware128, 64, 0);
//	libusb_control_transfer(handler, REQUEST_TYPE_OUT, REQUEST_FIRMWARE_HIGH, 0, 0, firmware128 + 64, 64, 0);
}

unsigned char Usb::getHardware() {
	unsigned char hardware;
	readPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_HARDWARE, &hardware, 1);
	return hardware;
}

struct tm Usb::getRtc() {
	struct tm rtcTime;
	readPandaHardwareSimple(REQUEST_TYPE_OUT, REQUEST_RTC, (unsigned char*)&rtcTime, sizeof(rtcTime));
	return rtcTime;
}

void Usb::setSafetyMode(uint16_t mode) {
	sendPandaHardwareSimple(REQUEST_TYPE_WRITE, REQUEST_SAFETY_MODE, mode, 0);
}
