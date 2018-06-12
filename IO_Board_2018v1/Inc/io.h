#ifndef IO_H_
#define IO_H_

/*
 * io.h
 *
 *  Created on: Feb 24, 2018
 *      Author: Adi
 */

#include "stm32f4xx_hal.h"
#include "mycan.h"
#include "adc.h"
#include "can.h"
#include "identifiers.h"

/* Hardware Definitions */
/* Pin Values */
#define	PIN_APPS1			GPIO_PIN_1
#define	PIN_APPS2			GPIO_PIN_2
#define	PIN_BSE1			GPIO_PIN_3
#define	PIN_BSE2			GPIO_PIN_4
#define PIN_ISENSE			GPIO_PIN_5
#define	PIN_BSPD_LED		GPIO_PIN_0
#define	PIN_APPS_LED		GPIO_PIN_1
#define	PIN_BSE_LED			GPIO_PIN_2
#define	PIN_BPPC_LED		GPIO_PIN_3
#define	PIN_FLT_R_LED		GPIO_PIN_4
#define	PIN_FLT_NR_LED		GPIO_PIN_5
#define	PIN_FLT_R			GPIO_PIN_5
#define	PIN_FLT_NR			GPIO_PIN_6
#define PIN_BSPD			GPIO_PIN_6
#define PIN_MCU_FLT			GPIO_PIN_7
#define PIN_MCU_FLT_NR		GPIO_PIN_8
/* Port Values */
#define	GROUP_APPS1			GPIOA
#define	GROUP_APPS2			GPIOA
#define	GROUP_BSE1			GPIOA
#define	GROUP_BSE2			GPIOA
#define GROUP_ISENSE		GPIOA
#define	GROUP_BSPD_LED		GPIOE
#define	GROUP_APPS_LED		GPIOE
#define	GROUP_BSE_LED		GPIOE
#define	GROUP_BPPC_LED		GPIOE
#define	GROUP_FLT_R_LED		GPIOE
#define	GROUP_FLT_NR_LED	GPIOE
#define	GROUP_FLT_R			GPIOB
#define	GROUP_FLT_NR		GPIOB
#define GROUP_BSPD			GPIOA
#define GROUP_MCU_FLT		GPIOA
#define GROUP_MCU_FLT_NR	GPIOA
#define HI                  GPIO_PIN_SET
#define LO                  GPIO_PIN_RESET

/* Sensor Thresholds and Offsets */
#define APPS_DIFF_THRESH        				1500
#define BSE_DIFF_THRESH         				1500

#define APPS_OFFSET               				1000
#define BSE_OFFSET                				1000

#define THROTTLE_THRESH							1800	// Out of 4096
#define BRAKE_THRESH							2000	// Out of 4096

/* Time Constants */
#define STARTUP_GRACE_PERIOD					1000
#define IGNORE_FLT_NR_GRACE_PERIOD				1000

/* Function Prototypes */
void readRawApps(ADC_HandleTypeDef hadc3);
void readRawBse(ADC_HandleTypeDef hadc1);
void readRawCurr(ADC_HandleTypeDef hadc2);

void updateThrottle();
void updateBrake();
void updateCurrent();

void updateSensors();
void updateLEDs();
void clearLEDs();
void clearFaults();
void clearStatuses();
void clearFlags();
void updateFaults();
void updateStatuses();
void assertFaults();

uint16_t getFltR();
uint16_t getFltNR();
uint16_t getIsThrottle();
uint16_t getIsBrake();
uint16_t getAppsMismatch();
uint16_t getBseMismatch();
uint16_t getBspdFault();
uint16_t getBppcFault();

void can_sendBrake();
void can_sendThrottle();
void can_sendFaultStatus();

void readCANMessages();
void sendCANStatuses();
void checkCANMessages();
void sendHeartbeat();

void init();
void mainLoop();

#endif /* IO_H_ */
