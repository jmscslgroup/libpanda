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

#include <iostream>
#include <cstring> // memset

#ifdef __APPLE__
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include "mogi/thread.h"

class UbloxInterface : public Mogi::Thread {
private:
	void doAction() {	// overload of Mogi::Thread
		
	};
	
public:
	
	enum UBLOX_INTERFACE_METHOD {
		UBLOX_INTERFACE_COM,
		UBLOX_INTERFACE_LIBUSB,
		UBLOX_INTERFACE_PANDA
	};
	
	UbloxInterface() {};
	
};

int main(int argc, char **argv) {
	std::cout << "Starting " << argv[0] << std::endl;
	
	libusb_context *context = NULL;
	struct libusb_device_handle *handler = NULL;
	int status;
	//	bool usingKernel = false;
	
	int numberOfInterfaces = 2;
	
	libusb_init(NULL);
	
	std::cerr << "Initializing Usb..." << std::endl;
	if (libusb_init(&context) != 0) {
		std::cerr << "FAILED to perform libusb_init()" << std::endl;
		exit(EXIT_FAILURE);
	}
	
#define USB_VENDOR_UBLOX (0x1546)
#define USB_PRODUCT_UBLOX_7 (0x01a7)
#define USB_PRODUCT_UBLOX_8 (0x01a8)
	
	if ((handler = libusb_open_device_with_vid_pid(context, USB_VENDOR_UBLOX, USB_PRODUCT_UBLOX_8)) != NULL) {
		std::cout << "Opened a U-Blox 8!" << std::endl;
	} else if ((handler = libusb_open_device_with_vid_pid(context, USB_VENDOR_UBLOX, USB_PRODUCT_UBLOX_7)) != NULL) {
		std::cout << "Opened a U-Blox 7!" << std::endl;
	} else {
		std::cout << "Failed to find a U-Blox GPS device..." << std::endl;
		libusb_exit(context);
		exit(EXIT_FAILURE);
	}
	
	libusb_exit(NULL);
	
	// get number of interfcaes here:
	libusb_device* device = libusb_get_device(handler);
	struct libusb_config_descriptor *configDescriptor;
	if((status = libusb_get_config_descriptor(device, 0, &configDescriptor)) == LIBUSB_SUCCESS) {
//		printf("bNumInterfaces: %d\n", configDescriptor->bNumInterfaces);
		numberOfInterfaces = configDescriptor->bNumInterfaces;
		libusb_free_config_descriptor(configDescriptor);
	}
	
	
	for (int i = 0; i < numberOfInterfaces; i++) {
		
		
		status = libusb_kernel_driver_active(handler, i);
		switch (status) {
			case 0:
				std::cout << " - No kernel active!" << std::endl;
				//			usingKernel = false;
				break;
			case 1:
				std::cout << " - Kernel active..." << std::endl;
				//			usingKernel = true;
				break;
			case LIBUSB_ERROR_NO_DEVICE:
				std::cout << " - libusb_kernel_driver_active() returned LIBUSB_ERROR_NO_DEVICE" << std::endl;
				break;
			case LIBUSB_ERROR_NOT_SUPPORTED:
				std::cout << " - libusb_kernel_driver_active() returned LIBUSB_ERROR_NOT_SUPPORTED" << std::endl;
				break;
				
			default:
				std::cout << " - libusb_kernel_driver_active() returned " << status << std::endl;
				break;
		}
		
		if ((status = libusb_set_auto_detach_kernel_driver(handler, true)) != LIBUSB_SUCCESS) {
			std::cout << "Warning! Unable to perform libusb_set_auto_detach_kernel_driver()" << std::endl;
		}
		
		if ((status = libusb_claim_interface(handler, i)) != LIBUSB_SUCCESS ) {
			std::cerr << "FAILED to perform libusb_claim_interface()" << std::endl;
			libusb_close(handler);
			libusb_exit(context);
			exit(EXIT_FAILURE);
		}
	}
	
	
	unsigned char buffer[4096];
	memset(buffer, 0, sizeof(buffer));
	int bufferIndex = 0;
	int length;
	for (int i = 0; i < 100; i++) {
		
		
		if((status = libusb_bulk_transfer(handler, 0x82, &buffer[bufferIndex], 64, &length, 10)) == LIBUSB_SUCCESS) {
//			printf("Incoming...  Read: %d\n", length);
//			printf("             Read: %s\n", buffer);
			bufferIndex += length;
		} else if (status == LIBUSB_ERROR_TIMEOUT && bufferIndex > 0) {
			printf("======= Success!  Read: %s\n", buffer);
			// process buffer
			bufferIndex = 0;
		} else {
//			printf("libusb error: %d\n", status);
			
		}
	}
	
	
	for (int i = 0; i < numberOfInterfaces; i++) {
		if ((status = libusb_release_interface(handler, i)) != LIBUSB_SUCCESS) {
			std::cerr << "FAILED to perform libusb_release_interface()" << std::endl;
		}
		
		//	if(usingKernel && (status = libusb_attach_kernel_driver(handler, interfacenumber)) == LIBUSB_SUCCESS) {
		//		std::cerr << "Kernel module re-attached!" << std::endl;
		//	}
	}
	
	
	libusb_close(handler);
	std::cout << "Device closed!" << std::endl;
	libusb_exit(context);
	
	std::cout << "Done." << std::endl;
	return 0;
}
