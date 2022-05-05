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

#define CAN_DEVICE_RX (0xFFFF)	// for clearing CAN ring buffer
#define CAN_DEVICE_TX_1 (0x0000)	// for clearing CAN ring buffer
#define CAN_DEVICE_TX_2 (0x0001)	// for clearing CAN ring buffer
#define CAN_DEVICE_TX_3 (0x0002)	// for clearing CAN ring buffer
#define CAN_DEVICE_TXGMLAN (0x0003)	// for clearing CAN ring buffer
#define UART_DEVICE_GPS (1)	// there are other UARTS, unsure if used

#define UART_BUFFER_READ_LENGTH (64)
#define UART_BUFFER_WRITE_LENGTH (0x20)

#define INIT_GPS_BAUD  9600
#define	GPS_BAUD  460800

// These are from the Panda python init:
// Request Types:
// The difference between out and in is determined by LIBUSB_ENDPOINT_DIR_MASK (0x80)
// The address is determeined by LIBUSB_ENDPOINT_ADDRESS_MASK (0x0F)
// i.e. from libusb:
/** Bits 0:4 determine recipient, see libusb_request_recipient.
 * Bits 5:6 determine type, see libusb_request_type.
 * Bit 7 determines data transfer direction, see libusb_endpoint_direction.
 */
#define REQUEST_TYPE_IN (LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE) // = 0xC0
//#define REQUEST_OUT (LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
//#define REQUEST_TYPE_OUT 0xc0	// NOT EQUAL TO (LIBUSB_ENDPOINT_OUT | TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
// Not sure about this but it's what boardd.cc does, same as REQUEST_TYPE_IN
#define REQUEST_TYPE_OUT (LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE) // = 0x40 <= ptyhon Panda class
//#define REQUEST_TYPE_WRITE 0x40 //	Not sure of the difference why this is called vs REQUEST_TYPE_OUT

// Endpoints:
// The difference between out and in is determined by LIBUSB_ENDPOINT_DIR_MASK (0x80)
// The address is determined by LIBUSB_ENDPOINT_ADDRESS_MASK (0x0F)
#define ENDPOINT_CAN_OUT  0x03	// = LIBUSB_ENDPOINT_OUT | 0x03 - Panda firmware: USBx_OUTEP(3)
#define ENDPOINT_CAN_IN   0x81	// = LIBUSB_ENDPOINT_IN | 0x01 - Panda firmware: USBx_INEP(1)
#define ENDPOINT_UART_OUT 0x02	// = LIBUSB_ENDPOINT_OUT | 0x02 - Panda firmware: USBx_OUTEP(2)
#define ENDPOINT_UART_IN  // doesn't exist in Panda firmware as far as I can tell


#define REQUEST_RTC              0xa0 // Reading panda's built-in RTC
#define REQUEST_FAN_SPEED        0xb2 // for fan RPM.  Not the EON, not sure what this is for
// The following definitions are interpreted from openpilot/panda/board/main.c:246 usb_cb_control_msg(
// Requests involving REQUEST_TYPE_OUT:
#define REQUEST_CAN_DEBUG        0xc0 // CAN debug information
#define REQUEST_HARDWARE         0xc1 // determine Panda model (unknown/black/grey/white/uno/dos/red)
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
// The following are new messages, discovered when needed to support red pandas:
#define REQUEST_KL_LINE_BAUD     0xf4 // k-line/l-line 5 baud initialization
#define REQUEST_CLOCK_SOURCE     0xf5 // Clock source mode
#define REQUEST_SIREN            0xf5 // Enable Siren
#define REQUEST_GREEN_LED        0xf7 // Set green LED
#define REQUEST_DISABLE_HEARTBEAT 0xf8 // disables heartbeat, only when firmware is compiled with debug flag
#define REQUEST_CAN_FD_BAUD      0xf9 // Set the baudrate of CAN FD
#define REQUEST_CAN_FD_ENABLED   0xfa // Checks is CAN FD and BRS are enabled

// Requests but for REQUEST_TYPE_WRITE
#define REQUEST_RTC_YEAR         0xa1 // For setting RTC
#define REQUEST_RTC_MONTH        0xa2
#define REQUEST_RTC_MDAY         0xa3
#define REQUEST_RTC_WDAY         0xa4 // Not needed, use REQUEST_RTC_MDAY
#define REQUEST_RTC_HOUR         0xa5
#define REQUEST_RTC_MINUTE       0xa6
#define REQUEST_RTC_SECONDS      0xa7

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


// copied from harness.h from comma.ai panda firmware
#define HARNESS_STATUS_NC 0U
#define HARNESS_STATUS_NORMAL 1U
#define HARNESS_STATUS_FLIPPED 2U

// copied from faults.h from comma.ai panda firmware
#define FAULT_STATUS_NONE 0U
#define FAULT_STATUS_TEMPORARY 1U
#define FAULT_STATUS_PERMANENT 2U

#define FAULT_RELAY_MALFUNCTION             (1U << 0)
#define FAULT_UNUSED_INTERRUPT_HANDLED      (1U << 1)
#define FAULT_INTERRUPT_RATE_CAN_1          (1U << 2)
#define FAULT_INTERRUPT_RATE_CAN_2          (1U << 3)
#define FAULT_INTERRUPT_RATE_CAN_3          (1U << 4)
#define FAULT_INTERRUPT_RATE_TACH           (1U << 5)
#define FAULT_INTERRUPT_RATE_GMLAN          (1U << 6)
#define FAULT_INTERRUPT_RATE_INTERRUPTS     (1U << 7)
#define FAULT_INTERRUPT_RATE_SPI_DMA        (1U << 8)
#define FAULT_INTERRUPT_RATE_SPI_CS         (1U << 9)
#define FAULT_INTERRUPT_RATE_UART_1         (1U << 10)
#define FAULT_INTERRUPT_RATE_UART_2         (1U << 11)
#define FAULT_INTERRUPT_RATE_UART_3         (1U << 12)
#define FAULT_INTERRUPT_RATE_UART_5         (1U << 13)
#define FAULT_INTERRUPT_RATE_UART_DMA       (1U << 14)
#define FAULT_INTERRUPT_RATE_USB            (1U << 15)
#define FAULT_INTERRUPT_RATE_TIM1           (1U << 16)
#define FAULT_INTERRUPT_RATE_TIM3           (1U << 17)
#define FAULT_REGISTER_DIVERGENT            (1U << 18)
#define FAULT_INTERRUPT_RATE_KLINE_INIT     (1U << 19)
#define FAULT_INTERRUPT_RATE_CLOCK_SOURCE   (1U << 20)
#define FAULT_INTERRUPT_RATE_TIM9           (1U << 21)

// copied from board_declarations.h from comma.ai panda firmware
// USB power modes (from cereal.log.health)
#define USB_POWER_NONE 0U
#define USB_POWER_CLIENT 1U
#define USB_POWER_CDP 2U
#define USB_POWER_DCP 3U

// copied from power_saving.h from comma.ai panda firmware
#define POWER_SAVE_STATUS_DISABLED 0
#define POWER_SAVE_STATUS_ENABLED 1


// This order was copied from cereal based nt he boardd.cc code.  The order however seems wrong.
// This is known to work, based on tests, for the white, grey and black pandas
enum {
	HARDWARE_UNKNOWN=0,	// I don't know what this is
	HARDWARE_WHITE_PANDA=1, // based on experimentation
	HARDWARE_GREY_PANDA=2, // based on experimentation
	HARDWARE_BLACK_PANDA=3, // based on experimentation
	HARDWARE_PEDAL=4,	// I don't know what this is
	HARDWARE_UNO=5,	// I don't know what this is
	HARDWARE_DOS=6,	// I don't know what this is
	HARDWARE_RED_PANDA=7,
};

// copied from panda/board/main.c
//typedef struct __attribute__((packed)) _PandaHealth {
typedef struct __attribute__((packed)) _PandaHealth {
  uint32_t uptime;
  uint32_t voltage;
  uint32_t current;
  uint32_t can_rx_errs;
  uint32_t can_send_errs;
  uint32_t can_fwd_errs;
  uint32_t gmlan_send_errs;
  uint32_t faults;
  uint8_t ignition_line;
  uint8_t ignition_can;
  uint8_t controls_allowed;
  uint8_t gas_interceptor_detected;
  uint8_t car_harness_status;
  uint8_t usb_power_mode;
  uint8_t safety_model;
  uint8_t fault_status;
  uint8_t power_save_enabled;
} PandaHealth;


#endif
