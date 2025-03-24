/*
 * io.c
 *
 *  Created on: Jan 18, 2020
 *      Author: max
 */

#include "io.h"
#include "cmsis_os.h"
#include "main.h"

//
// LEDs management
//

void led0( int status ) {
	if( status == ON ) {
		HAL_GPIO_WritePin( LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET );
	} else if( status == OFF ) {
		HAL_GPIO_WritePin( LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET );
	}
}

void led1( int status ) {
	if( status == ON ) {
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET );
	} else if( status == OFF ) {
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET );
	}
}

void led2( int status ) {
	if( status == ON ) {
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET );
	} else if( status == OFF ) {
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET );
	}
}

void led3( int status ) {
	if( status == ON ) {
		HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET );
	} else if( status == OFF ) {
		HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET );
	}
}

//
// Switches management
//

uint8_t getSwitches() {
	return
		GET_SWITCH( 0 ) |
		( GET_SWITCH( 1 ) << 1 ) |
		( GET_SWITCH( 2 ) << 2 ) |
		( GET_SWITCH( 3 ) << 3 ) |
		( GET_SWITCH( 4 ) << 4 ) |
		( GET_SWITCH( 5 ) << 5 ) |
		( GET_SWITCH( 6 ) << 6 ) |
		( GET_SWITCH( 7 ) << 7 );
}

int getSwitch0() {
	return LL_GPIO_IsInputPinSet( SW0_GPIO_Port, SW0_Pin );
}

int getSwitch1() {
	return LL_GPIO_IsInputPinSet( SW1_GPIO_Port, SW1_Pin );
}

int getSwitch2() {
	return LL_GPIO_IsInputPinSet( SW2_GPIO_Port, SW2_Pin );
}

int getSwitch3() {
	return LL_GPIO_IsInputPinSet( SW3_GPIO_Port, SW3_Pin );
}

int getSwitch4() {
	return LL_GPIO_IsInputPinSet( SW4_GPIO_Port, SW4_Pin );
}

int getSwitch5() {
	return LL_GPIO_IsInputPinSet( SW5_GPIO_Port, SW5_Pin );
}

int getSwitch6() {
	return LL_GPIO_IsInputPinSet( SW6_GPIO_Port, SW6_Pin );
}

int getSwitch7() {
	return LL_GPIO_IsInputPinSet( SW7_GPIO_Port, SW7_Pin );
}

//
// IO bus management
//

uint32_t getIoState() {
	return LL_GPIO_ReadInputPort( IO0_GPIO_Port ) | ( LL_GPIO_ReadInputPort( IO16_GPIO_Port ) << 16 );
}

//
// LCD management
//

extern LCD_HandleTypeDef hlcd;

// LCD display has segments of each digit spread along the 4 commons
// It means changing a single digit requires writing 4 different 2 bits values
// Segment map is:
//
//		 Bit1 Bit0          +-A-+
// COM0: DP   SEGD			F   B
// COM1: SEGC SEGE          +-G-+
// COM2: SEGB SEGG          E   C
// COM3: SEGA SEGF          +-D-+
//
// Different digits are located in frame buffer, shifting the two bit pattern
// Digits order is: 0 1 2 3

static const uint8_t lcdPatternEmpty[] = { 0, 0, 0, 0 };
static const uint8_t lcdPatternMinus[] = { 0, 0, 1, 0 };
static const uint8_t lcdPattern0[] = { 1, 3, 2, 3 };
static const uint8_t lcdPattern1[] = { 0, 2, 2, 0 };
static const uint8_t lcdPattern2[] = { 1, 1, 3, 2 };
static const uint8_t lcdPattern3[] = { 1, 2, 3, 2 };
static const uint8_t lcdPattern4[] = { 0, 2, 3, 1 };
static const uint8_t lcdPattern5[] = { 1, 2, 1, 3 };
static const uint8_t lcdPattern6[] = { 1, 3, 1, 3 };
static const uint8_t lcdPattern7[] = { 0, 2, 2, 2 };
static const uint8_t lcdPattern8[] = { 1, 3, 3, 3 };
static const uint8_t lcdPattern9[] = { 1, 2, 3, 3 };
static const uint8_t lcdPatternA[] = { 0, 3, 3, 3 };
static const uint8_t lcdPatternB[] = { 1, 3, 1, 1 };
static const uint8_t lcdPatternC[] = { 1, 1, 0, 3 };
static const uint8_t lcdPatternD[] = { 1, 3, 3, 0 };
static const uint8_t lcdPatternE[] = { 1, 1, 1, 3 };
static const uint8_t lcdPatternF[] = { 0, 1, 1, 3 };
static const uint8_t lcdPatternL[] = { 1, 1, 0, 1 };

// Write a character in the specified position of the display

#define BITS_RELOCATE( x ) ( ( ( ( x ) & 0x0f ) << 12 ) | ( ( ( x ) & 0xf0 ) << 20 ) )

void lcdWriteDigit( char ch, int position ) {
	const uint8_t *pattern;
	switch( ch ) {
	case '0':
		pattern = lcdPattern0;
		break;
	case '1':
		pattern = lcdPattern1;
		break;
	case '2':
		pattern = lcdPattern2;
		break;
	case '3':
		pattern = lcdPattern3;
		break;
	case '4':
		pattern = lcdPattern4;
		break;
	case '5':
		pattern = lcdPattern5;
		break;
	case '6':
		pattern = lcdPattern6;
		break;
	case '7':
		pattern = lcdPattern7;
		break;
	case '8':
		pattern = lcdPattern8;
		break;
	case '9':
		pattern = lcdPattern9;
		break;
	case 'A':
		pattern = lcdPatternA;
		break;
	case 'B':
		pattern = lcdPatternB;
		break;
	case 'C':
		pattern = lcdPatternC;
		break;
	case 'D':
		pattern = lcdPatternD;
		break;
	case 'E':
		pattern = lcdPatternE;
		break;
	case 'F':
		pattern = lcdPatternF;
		break;
	case ' ':
		pattern = lcdPatternEmpty;
		break;
	case '-':
		pattern = lcdPatternMinus;
		break;
	case 'L':
		pattern = lcdPatternL;
		break;
	default:
		pattern = lcdPatternEmpty;
		break;
	}
	// Compute shift and mask
	int shift = 2 * position; // 2 bits for each digit
	uint32_t mask = ~( 3 << shift );
	// Correct mask according to non contiguous segment assignment
	// (0-3 -> 12-15, 4-7 -> 24-27)
	mask = BITS_RELOCATE( mask );
	// Write frame buffer
	HAL_LCD_Write( &hlcd, LCD_RAM_REGISTER0, mask, BITS_RELOCATE( pattern[0] << shift ) );
	HAL_LCD_Write( &hlcd, LCD_RAM_REGISTER2, mask, BITS_RELOCATE( pattern[1] << shift ) );
	HAL_LCD_Write( &hlcd, LCD_RAM_REGISTER4, mask, BITS_RELOCATE( pattern[2] << shift ) );
	HAL_LCD_Write( &hlcd, LCD_RAM_REGISTER6, mask, BITS_RELOCATE( pattern[3] << shift ) );
}

void lcdUpdateDisplay( void ) {
	HAL_LCD_UpdateDisplayRequest( &hlcd );
}

void lcdWriteHexDigit( unsigned int value, int position ) {
	lcdWriteDigit( value < 10 ? value + '0' : value + 'A' - 10, position );
}

