/*
 * application.c
 *
 *  Created on: Oct 20, 2020
 *      Author: max
*/

#include "main.h"
#include "cmsis_os2.h"
#include "io.h"
#include "usbserial.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

extern COMP_HandleTypeDef hcomp1;

extern DAC_HandleTypeDef hdac1;

extern LCD_HandleTypeDef hlcd;

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;


__weak void StartDefaultTask( void *argument ) {
	while( 1 ) {



}
}

















