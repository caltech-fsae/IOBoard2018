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

#define APPS_VAL_THRESH         	1000
#define BSE_VAL_THRESH          	1000
#define CURRSENSE_VAL_THRESH    	1000

#define BPPC_QTR_THROTTLE       	1000         // 25% of pedal travel reading
#define BPPC_BRK_THRESH         	1000         // Is braking
#define BPPC_STOP_THRESH        	1000           //5% of APPS travel reading

#define APPS_DIFF_THRESH        	3000			// As a fraction of 1
#define BSE_DIFF_THRESH         	3000			// As a fraction of 1

#define APPS_OFFSET               1000
#define BSE_OFFSET                1000

#define THROTTLE_THRESH			1500			// Out of 4096
#define BRAKE_THRESH				1500			// Out of 4096

#define APPS_AVG_SAMPLE_SIZE 	100 // Sample sizes for running averages
#define BSE_AVG_SAMPLE_SIZE 		100
#define CURR_AVG_SAMPLE_SIZE 	100

/* Function Prototypes */
void readApps(ADC_HandleTypeDef hadc3);
void readBse(ADC_HandleTypeDef hadc1);
void readCurr(ADC_HandleTypeDef hadc2);

void gen_avg_bse();
void gen_avg_apps();
void gen_avg_curr();

void filterApps();
void filterBse();
void filterCurr();

void scaleThrottle();
void scaleBrake();
void scaleCurrent();

void init_sensors();

void updateLEDs();
void clearFaults();
void updateFaults();
void assertFaults();

uint16_t getFltR();
uint16_t getFltNR();
uint16_t getIsThrottle();
uint16_t getIsBrake();
uint16_t getAppsMismatch();
uint16_t getBseMismatch();
uint16_t getPotato();
uint16_t getBppcFault();

void can_sendBrake();
void can_sendThrottle();
void can_sendFaultStatus();

void sendCANStatuses();
void checkCANMessages();
void sendHeartbeat();
void mainLoop();

void init();

#endif /* IO_H_ */
