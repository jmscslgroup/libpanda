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

#ifndef PANDA_DEFINITIONS_H
#define PANDA_DEFINITIONS_H

#define UART_DEVICE_GPS (1)
#define UART_BUFFER_READ_LENGTH (64)
#define UART_BUFFER_WRITE_LENGTH (0x20)
#define INIT_GPS_BAUD  9600
#define	GPS_BAUD  460800


// These are from the Panda python init:
// Request Types:
#define REQUEST_TYPE_IN (LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
//#define REQUEST_OUT (LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define REQUEST_TYPE_OUT 0xc0
#define REQUEST_TYPE_WRITE 0x40	//	Not sure of the difference when this is called vs REQUEST_TYPE_OUT



#define REQUEST_RTC              0xa0 // Reading panda's built-in RTC
#define REQUEST_FAN_SPEED        0xb2 // for fan RPM.  Not the EON, not sure what this is for
// The following definitions are interpreted from openpilot/panda/board/main.c:246 usb_cb_control_msg(
// Requests involving REQUEST_TYPE_OUT:
#define REQUEST_CAN_DEBUG        0xc0 // CAN debug information
#define REQUEST_HARDWARE         0xc1 // determine Panda model (GPS)
#define REQUEST_SERIAL           0xd0
#define REQUEST_BOOTLOADER       0xd1 // For flashing
#define REQUEST_CAN_HEALTH       0xd2 // Can get ignition info here
#define REQUEST_FIRMWARE_LOW     0xd3 // not in main.c
#define REQUEST_FIRMWARE_HIGH    0xd4 // not in main.c
#define REQUEST_VERSION          0xd6 //
#define REQUEST_RESET_ST         0xd8 //
#define REQUEST_ESP_POWER        0xd9 // For setting GPS power
#define REQUEST_ESP_RESET        0xda // Can put ESP into optional bootmode
#define REQUEST_GMLAN_CAN_ENABLE 0xdb // Set GMLAN (white/grey) or OBD CAN (black) multiplexing mode
#define REQUEST_SAFETY_MODE      0xdc // Maybe enabling writing to CAN bus? See below SAFETY_xxxxx definitions
#define REQUEST_CAN_FORWARDING   0xdd // eEnable CAN forwarding,wValue = Can Bus Num to forward from, wIndex = Can Bus Num to forward to
#define REQUEST_CAN_BAUD         0xde // CAN bitrate
#define REQUEST_LONG_CONTROLS    0xdf // Enable long controls, whatever that is
#define REQUEST_UART_READ        0xe0 // GPS reading
#define REQUEST_UART_BAUD_SMALL  0xe1 // Not needed, for simple buad rates.  Use REQUEST_UART_BAUD
#define REQUEST_UART_PARITY      0xe2 // For GPS, default to 9600 8N1
#define REQUEST_UART_BAUD        0xe4 // For GPS, default to 9600 8N1
#define REQUEST_CAN_LOOPBACK     0xe5 // For testing CAN
#define REQUEST_USB_POWER_MODE   0xe6 // This deals with charging mode
#define REQUEST_USB_POWER_SAVE   0xe7 // Power save state enable?
#define REQUEST_K_LONE_PULSE     0xf0 // Pulses UART2, needed for Acura
#define REQUEST_CAN_RING_CLEAR   0xf1 // Clear CAN ring buffer
#define REQUEST_UART_RING_CLEAR  0xf2 // Clear UART ring buffer
#define REQUEST_HEARTBEAT        0xf3 //

// Requests but for REQUEST_TYPE_WRITE
#define REQUEST_RTC_YEAR         0xa1 // For setting RTC
#define REQUEST_RTC_MONTH        0xa2
#define REQUEST_RTC_MDAY         0xa3
#define REQUEST_RTC_WDAY         0xa4 // Not needed, use REQUEST_RTC_MDAY
#define REQUEST_RTC_HOUR         0xa5
#define REQUEST_RTC_MINUTE       0xa6
#define REQUEST_RTC_SECONDS      0xa7

// Endpoints:
#define ENDPOINT_CAN_IN 0x81
#define ENDPOINT_CAN_OUT 0x03
#define ENDPOINT_UART_OUT 0x02
#define ENDPOINT_UART_IN 0x86	// NOT DONE IN PANDA: uhhh maybe this value? only for isoc

// From openpilot/panda/board/safety:20
#define SAFETY_NOOUTPUT 0U
#define SAFETY_HONDA 1U
#define SAFETY_TOYOTA 2U
#define SAFETY_ELM327 3U
#define SAFETY_GM 4U
#define SAFETY_HONDA_BOSCH 5U
#define SAFETY_FORD 6U
#define SAFETY_CADILLAC 7U
#define SAFETY_HYUNDAI 8U
#define SAFETY_CHRYSLER 9U
#define SAFETY_TESLA 10U
#define SAFETY_SUBARU 11U
#define SAFETY_GM_PASSIVE 12U
#define SAFETY_MAZDA 13U
#define SAFETY_TOYOTA_IPAS 16U
#define SAFETY_ALLOUTPUT 17U
#define SAFETY_GM_ASCM 18U



#endif
