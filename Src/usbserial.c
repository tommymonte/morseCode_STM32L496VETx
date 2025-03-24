/*
 * usbserial.c
 *
 *  Created on: Jun 5, 2019
 *      Author: max
 */

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/unistd.h>
#include <stdlib.h>
#include <stdarg.h>
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "stream_buffer.h"
#include "io.h"
#include "usbd_def.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbserial.h"

// Task handle of USB task, defined in main.c

extern osThreadId_t usbTxTaskHandle;

// Task flags

#define USB_TXDONE_FLAG 1

// USB serial management

#define USBSERIAL_RXBUFSIZE 2048
#define USBSERIAL_TXBUFSIZE 2048

static uint8_t usbserialRxBuffer[USBSERIAL_RXBUFSIZE], usbserialTxBuffer[USBSERIAL_TXBUFSIZE];
static StaticStreamBuffer_t usbserialRxBufferStruct, usbserialTxBufferStruct;
static StreamBufferHandle_t usbserialRxHandle, usbserialTxHandle;

extern USBD_HandleTypeDef hUsbDeviceFS;

void usbserialInit( void ) {
	// Create Stream Buffer's
	usbserialTxHandle = xStreamBufferCreateStatic( USBSERIAL_TXBUFSIZE, 1, usbserialTxBuffer, &usbserialTxBufferStruct );
	usbserialRxHandle = xStreamBufferCreateStatic( USBSERIAL_RXBUFSIZE, 1, usbserialRxBuffer, &usbserialRxBufferStruct );
	// Init USB-CDC subsystem
	MX_USB_DEVICE_Init();
	osDelay( 2000 );
}

// Rx side
// This function is called ONLY by the USB CDC driver, when a data packet is received
// If buffer space is exhausted, then set usbRxStopped flag, save usb handle for later usa
// and return -1

static volatile int usbRxStopped;
static USBD_HandleTypeDef *usbHandle;

int usbserialQueue( USBD_HandleTypeDef *handle, uint8_t *dataPtr, uint32_t len ) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	int retval;
	xStreamBufferSendFromISR( usbserialRxHandle, dataPtr, len, &xHigherPriorityTaskWoken );
	if( xStreamBufferSpacesAvailable( usbserialRxHandle ) >= USB_FS_MAX_PACKET_SIZE ) {
		retval = 0;
	} else {
		usbRxStopped = 1;
		usbHandle = handle;
		retval = 1;
	}
	return retval;
}

// Extract a byte from the Rx buffer. If the usb rx is stopped, and enough space is available in the buffer
// then resume usb rx process

unsigned char usbserialGetByte() {
	uint8_t data;
	xStreamBufferReceive( usbserialRxHandle, &data, 1, portMAX_DELAY );
	if( usbRxStopped && xStreamBufferSpacesAvailable( usbserialRxHandle ) >= USB_FS_MAX_PACKET_SIZE ) {
		USBD_CDC_ReceivePacket( usbHandle );
	}
    return data;
}

// Tx side
// The Tx queue is read by a task (usbTxTask) constantly waiting for data to transmit
// Any other task can insert data in the queue

// Send data to USB port - very slow!!!
void usbserialPutByte( uint8_t data ) {
	xStreamBufferSend( usbserialTxHandle, &data, 1, portMAX_DELAY );
}

// Copy data to usb serial Tx buffer, and block task if no space is available
// Please notice that total amount of data must be less than Tx buffer size (see above)
void usbserialWriteData( uint8_t *data, unsigned int length ) {
	xStreamBufferSend( usbserialTxHandle, data, length, portMAX_DELAY );
}

void usbserialPrint( char *msg ) {
	usbserialWriteData( ( uint8_t * )msg, strlen( msg ) );
}

void usbserialWriteChar( char ch ) {
	usbserialPutByte( ( uint8_t )ch );
}

static char nibbleToHex( unsigned int value ) {
	char ch = ( value & 0xf ) + '0';
	if( ch > '9' ) {
		ch = ( value & 0xf ) + 'A' - 10;
	}
	return ch;
}

void usbserialWriteHexNibble( unsigned int value ) {
	usbserialPutByte( nibbleToHex( value ) );
}

void usbserialWriteHexByte( unsigned int value ) {
	usbserialPutByte( nibbleToHex( ( value & 0xf0 ) >> 4 ) );
	usbserialPutByte( nibbleToHex( value & 0x0f ) );
}

void usbserialWriteHexWord16( unsigned int value ) {
	usbserialPutByte( nibbleToHex( ( value & 0xf000 ) >> 12 ) );
	usbserialPutByte( nibbleToHex( ( value & 0x0f00 ) >> 8 ) );
	usbserialPutByte( nibbleToHex( ( value & 0x00f0 ) >> 4 ) );
	usbserialPutByte( nibbleToHex( value & 0x000f ) );
}

void usbserialWriteHexWord32( unsigned int value ) {
	usbserialPutByte( nibbleToHex( ( value & 0xf0000000 ) >> 28 ) );
	usbserialPutByte( nibbleToHex( ( value & 0x0f000000 ) >> 24 ) );
	usbserialPutByte( nibbleToHex( ( value & 0x00f00000 ) >> 20 ) );
	usbserialPutByte( nibbleToHex( ( value & 0x000f0000 ) >> 16 ) );
	usbserialPutByte( nibbleToHex( ( value & 0x0000f000 ) >> 12 ) );
	usbserialPutByte( nibbleToHex( ( value & 0x00000f00 ) >> 8 ) );
	usbserialPutByte( nibbleToHex( ( value & 0x000000f0 ) >> 4 ) );
	usbserialPutByte( nibbleToHex( value & 0x0000000f ) );
}

// Convenience functions for hexadecimal conversions

unsigned int usbserialReadHexNibble() {
	char ch = toupper( usbserialReadChar() );
	int result = 0;
	if( ch >= '0' ) {
		result = ch - '0';
		if( result > 9 ) {
			result = ch - 'A' + 10;
		}
		if( result > 15 ) { // Error
			result = 0;
		}
	}
	return result;
}

unsigned int usbserialReadHexByte() {
	return ( usbserialReadHexNibble() << 4 ) + usbserialReadHexNibble();
}

unsigned int usbserialReadHexWord16() {
	return
        ( usbserialReadHexNibble() << 12 ) + ( usbserialReadHexNibble() << 8 ) +
		( usbserialReadHexNibble() << 4 ) + usbserialReadHexNibble();
}

unsigned int usbserialReadHexWord32() {
	return
        ( usbserialReadHexNibble() << 28 ) + ( usbserialReadHexNibble() << 24 ) +
        ( usbserialReadHexNibble() << 20 ) + ( usbserialReadHexNibble() << 16 ) +
        ( usbserialReadHexNibble() << 12 ) + ( usbserialReadHexNibble() << 8 ) +
		( usbserialReadHexNibble() << 4 ) + usbserialReadHexNibble();
}

static int echoOn;

void usbserialEchoOn() {
	echoOn = 1;
}

void usbserialEchoOff() {
	echoOn = 0;
}

char usbserialReadChar() {
	char ch = usbserialGetByte();
	if( echoOn ) {
		usbserialPutByte( ch );
	}
	return ch;
}

// Return number of bytes available
int usbserialDataAvailable() {
	return xStreamBufferBytesAvailable( usbserialRxHandle );
}

void usbserialPrintf( const char *fmt, ... ) {
	char buffer[128];
	va_list args;
	va_start( args, fmt );
	vsnprintf( buffer, sizeof( buffer ), fmt, args );
	va_end( args );
	usbserialPrint( buffer );
}

// Flag to assess USB initialization ended
static int readyFlag = 0;

// Return USB initialization status
int usbserialIsReady() {
	return readyFlag;
}

// Wakeup function for Tx task
// Called by CDC_TransmitCplt_FS in usbd_cdc_if.c

void usbserialTxWakeup() {
	osThreadFlagsSet( usbTxTaskHandle, USB_TXDONE_FLAG );
}

// USB Tx task
// Just loop on the Tx queue, waiting for data to send
// Limit data sent to maximum USB packet size (64 bytes for USB full speed)
// If USB Tx is busy, then sleep up to flag assertion (no loop wait)

void StartUsbTxTask( void *argument ) {
	usbserialInit();
	readyFlag = 1;
	while( 1 ) {
		uint8_t data[64];
		int len = xStreamBufferReceive( usbserialTxHandle, data, sizeof( data ), 1 );
		if( len > 0 ) {
			// Schedule USB packet transmission, but wait for Tx end
			// to not modify buffer meanwhile
			while( CDC_Transmit_FS( data, len ) != USBD_OK ) {
				osThreadFlagsWait(
					USB_TXDONE_FLAG, osFlagsWaitAny, osWaitForever
				);
			}
		}
	}
}
