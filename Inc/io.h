/*
 * io.h
 *
 *  Created on: Jan 18, 2020
 *      Author: max
 */

#ifndef IO_H_
#define IO_H_

#include <main.h>

#define ON 	1
#define OFF 0

//
// LEDs management
//

// Set status of LEDs. status parameter must be ON or OFF (defined above)
void led0( int status );
void led1( int status );
void led2( int status );
void led3( int status );

//
// IO bus management
//

// Get status of the 32 bit IO bus as a 32 bit unsigned binary value
uint32_t getIoState( void );

//
// Switches management
//

// Macro to get status of a single switch. Index must be from 0 to 7
#define GET_SWITCH( index )	( LL_GPIO_IsInputPinSet( SW ## index ## _GPIO_Port, SW ## index ## _Pin ) )

// Get status of the 8 switches as an 8 bit unsigned binary value
uint8_t getSwitches( void );

// Convenience functions to get the value of a single switch
int getSwitch0( void );
int getSwitch1( void );
int getSwitch2( void );
int getSwitch3( void );
int getSwitch4( void );
int getSwitch5( void );
int getSwitch6( void );
int getSwitch7( void );

//
// LCD display management
//

// Load the specified ASCII character in the required position (0-3)
// Only few characters are implemented, mainly hex digits, blank, 'L' and '-'
void lcdWriteDigit( char ch, int position );

// Update the LCD display, forcing immediate display of internal buffer
void lcdUpdateDisplay( void );

// Convenience function to write hexadecimal digits on the LCD display
void lcdWriteHexDigit( unsigned int value, int position );

#endif /* IO_H_ */
