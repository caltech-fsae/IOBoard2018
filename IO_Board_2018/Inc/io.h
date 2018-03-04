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

#define FLT_PINS_GROUP		GPIOB
#define	FLT_PIN				GPIO_PIN_5
#define	FLT_NR_PIN			GPIO_PIN_6
#define MCU_FLT_PIN			GPIO_PIN_8

#define BSPD_PIN_GROUP		GPIOB
#define BSPD_PIN			GPIO_PIN_7

#define	LED_PINS_GROUP		GPIOE
#define	BSPD_LED_PIN		GPIO_PIN_0
#define	APPS_LED_PIN		GPIO_PIN_1
#define	BSE_LED_PIN			GPIO_PIN_2
#define	BPPC_LED_PIN		GPIO_PIN_3
#define	FLT_LED_PIN			GPIO_PIN_4
#define	FLTNR_LED_PIN		GPIO_PIN_5

#define HI                  GPIO_PIN_SET
#define LO                  GPIO_PIN_RESET

#define APPS_VAL_THRESH         1000
#define BSE_VAL_THRESH          1000
#define CURRSENSE_VAL_THRESH    1000

#define BPPC_QTR_THROTTLE       300         // 25% of pedal travel reading
#define BPPC_BRK_THRESH         100         // Is braking

#define APPS_DIFF_THRESH        0.1
#define BSE_DIFF_THRESH         0.1

#endif /* IO_H_ */

/* Function Prototypes */
void readApps(uint16_t* apps1, uint16_t* apps2, ADC_HandleTypeDef hadc3);
void readBse(uint16_t* bse1, uint16_t* bse2, ADC_HandleTypeDef hadc1);
void readCurr(uint16_t* currSensor, ADC_HandleTypeDef hadc2);
void filterApps(uint16_t* prevApps, uint16_t* apps);
void filterBse(uint16_t* prevBse, uint16_t* bse);
void filterCurr(uint16_t* prevCurr, uint16_t* curr);

int bspdStatus();
int fltStatus();
int fltNrStatus();
int appsStatus(uint16_t* apps1, uint16_t* apps2);
int bseStatus(uint16_t* bse1, uint16_t* bse2);
int bppcStatus(uint16_t* apps1, uint16_t* apps2, uint16_t* bse1, uint16_t* bse2);

void clearFaultLEDs();
void displayFaultLEDs();
void assertMcuFlt();
void resetMcuFlt();
