/**
  ******************************************************************************
  * @file           : usbserial.h
  * @brief          : Header for usbserial.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBSERIAL_H__
#define __USBSERIAL_H__

#include "usbd_def.h"

//
// PRIVATE FUNCTIONS: DO NOT USE IN USER CODE!!!
//

// Initialize usbserial subsystem, called by USB management task
void usbserialInit( void );

// Queue data to Rx buffer, called by USB CDC interrupt handler
int usbserialQueue( USBD_HandleTypeDef *handle, uint8_t *dataPtr, uint32_t len );

// Copy data to usbserial Tx buffer, and block task if no space is available
void usbserialWriteData( uint8_t *data, unsigned int length );

// Wakeup USB management task to trigger transmission of dfferent data
void usbserialTxWakeup();

//
// PUBLIC FUNCTIONS, FOR USER CODE USAGE
//

// Print a null terminated string to USB virtual serial port
void usbserialPrint( char *msg );

// Print a single char to USB virtual serial port
void usbserialWriteChar( char ch );

// Convenience functions to convert binary values to ASCII hexadecimal representation
// and to send them to USB virtual serial port
void usbserialWriteHexNibble( unsigned int value );
void usbserialWriteHexByte( unsigned int value );
void usbserialWriteHexWord16( unsigned int value );
void usbserialWriteHexWord32( unsigned int value );

// Read a character from USB virtual serial port, and return it
char usbserialReadChar( void );

// Change echo settings while reading chars
// If echo is on, then automatic echo characters are sent while reading
void usbserialEchoOn( void );
void usbserialEchoOff( void );

// Return number of available bytes in the input buffer (not blocking)
int usbserialDataAvailable();

// Convenience functions to read characters from the USB virtual serial port, and
// to convert ASCII hexadecimal representation to binary values
unsigned int usbserialReadHexNibble();
unsigned int usbserialReadHexByte();
unsigned int usbserialReadHexWord16();
unsigned int usbserialReadHexWord32();

// Standard printing and formatting function, with output to the USB virtual serial port
// NOTE: if used, code size and speed can be impaired, as standard printf complexity is introduced
void usbserialPrintf( const char *fmt, ... );

// Check if th USB virtual serial port is ready
// No operation must be performed if the result is false
int usbserialIsReady();

#endif /* __CONSOLE_H__ */

/************************ (C) COPYRIGHT Massimo Ruo Roch *****END OF FILE****/
