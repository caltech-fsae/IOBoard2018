/*
 * io.h
 *
 *  Created on: Feb 24, 2018
 *      Author: Adi
 */

#include "stm32f4xx_hal.h"

#ifndef IO_H_
#define IO_H_

#define SENSE_PINS_GROUP	GPIOA
#define	APPS1_PIN			GPIO_PIN_1
#define	APPS2_PIN			GPIO_PIN_2
#define	BSE1_PIN			GPIO_PIN_3
#define	BSE2_PIN			GPIO_PIN_4
#define ISENSE_PIN			GPIO_PIN_5

#define FLT_PIN_GROUP		GPIOB
#define	FLT_PIN				GPIO_PIN_8

#define BSPD_PIN_GROUP		GPIOB
#define BSPD_PIN			GPIO_PIN_7

#define	LED_PINS_GROUP		GPIOE
#define	BSPD_LED_PIN		GPIO_PIN_0
#define	APPS_LED_PIN		GPIO_PIN_1
#define	BSE_LED_PIN			GPIO_PIN_2
#define	BPPC_LED_PIN		GPIO_PIN_3
#define	FLT_LED_PIN			GPIO_PIN_4
#define	FLTNR_LED_PIN		GPIO_PIN_5

#define HIGH				GPIO_PIN_SET
#define LOW					GPIO_PIN_RESET

#endif /* IO_H_ */

/* Function Prototypes */
void adcInit();

//void readApps(uint16_t* apps1, uint16_t* apps2);
void readBse(uint32_t* bse1, uint32_t* bse2);
//void readCurrSensor(uint32_t* currSensor);

void clearFaultLEDs();
void displayFaultLEDs();
void assertFLT();
void resetFLT();
int appsMSMStatus();
int bseMSMStatus();
int bspdStatus();
int bppcStatus();
int fltStatus();
int fltnrStatus();
uint16_t ADC1_read();

