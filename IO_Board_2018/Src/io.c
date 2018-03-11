/*
 * io.c
 *
 *  Created on: Feb 24, 2018
 *      Author: Adi
 */


#include "io.h"
#include "adc.h"
#include <stdlib.h>

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

// Global Variables
struct Sensors {
    // Raw values from ADCs
    uint16_t apps1, apps2, bse1, bse2, currSensor;
    uint16_t prevApps1, prevApps2, prevBse1, prevBse2, prevCurr;

    // Filtered / scaled values
    uint16_t throttle, brake, current;
} sensors;

struct Status {
    uint16_t isThrottle;
    uint16_t isBrake;

    uint16_t flt_apps_mismatch;
    uint16_t flt_bse_mismatch;

    uint16_t internal_flt_r;
    uint16_t internal_flt_nr;

    uint16_t flt_bspd;        // Strictly a software indication of BSPD fault
    uint16_t flt_bppc;
} status;

void mainLoop(){
	clearFaults();

	readApps(hadc3);
	readBse(hadc1);
	readCurr(hadc2);

	filterApps();
	filterBse();
	filterCurr();

	scaleThrottle();
	scaleBrake();
	scaleCurrent();

	updateFaults();
	updateLEDs();
	assertFaults();

	can_sendBrake();
	can_sendThrottle();
}

// #--------------------------# READ ADC FUNCTIONS #---------------------------#

void readApps(ADC_HandleTypeDef hadc3) {
    HAL_ADC_Start(&hadc3);

    // Poll the entire ADC3 group for conversion
    if (HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY) == HAL_OK) {
        // Read APPS1
        sensors.apps1 = HAL_ADC_GetValue(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
        // Read APPS2
        sensors.apps2 = HAL_ADC_GetValue(&hadc3);
    }

    HAL_ADC_Stop(&hadc3);
}


void readBse(ADC_HandleTypeDef hadc1) {
    HAL_ADC_Start(&hadc1);

    // Poll the entire ADC1 group for conversion
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        // Read BSE1
        sensors.bse1 = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        // Read BSE2
        sensors.bse2 = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
}


void readCurr(ADC_HandleTypeDef hadc2) {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK)
        sensors.currSensor = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
}


// #-------------------------# PROCESSING FUNCTIONS #--------------------------#


// Filter both APPS readings
void filterApps() {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
	uint16_t newApps1 = sensors.apps1;
	if (abs(sensors.apps1 - sensors.prevApps1) > APPS_VAL_THRESH)
		sensors.apps1 = sensors.prevApps1;
	// Update old value for next iteration
	sensors.prevApps1 = newApps1;

	uint16_t newApps2 = sensors.apps2;
		if (abs(sensors.apps2 - sensors.prevApps2) > APPS_VAL_THRESH)
			sensors.apps2 = sensors.prevApps2;
		// Update old value for next iteration
		sensors.prevApps2 = newApps2;
}


// Filter a single BSE reading
void filterBse() {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
	uint16_t newBse1 = sensors.bse1;
	if (abs(sensors.bse1 - sensors.prevBse1) > BSE_VAL_THRESH)
        sensors.bse1 = sensors.prevBse1;
    // Update old value for next iteration
    sensors.prevBse1 = newBse1;

    uint16_t newBse2 = sensors.bse2;
	if (abs(sensors.bse2 - sensors.prevBse2) > BSE_VAL_THRESH)
		sensors.bse2 = sensors.prevBse2;
	// Update old value for next iteration
	sensors.prevBse2 = newBse2;
}


// Filters the current sensor reading
void filterCurr() {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
    uint16_t newCurr = sensors.currSensor;
	if (abs(sensors.currSensor - sensors.prevCurr) > CURRSENSE_VAL_THRESH)
    	sensors.currSensor = sensors.prevCurr;
    // Update old value for next iteration
    sensors.prevCurr = newCurr;
}

void scaleThrottle() {
    sensors.throttle = (sensors.apps1 + sensors.apps2) / 2;
}

void scaleBrake() {
    sensors.brake = (sensors.bse1 + sensors.bse2) / 2;
}

void scaleCurrent() {
    sensors.current = sensors.currSensor;
}


void updateLEDs() {
	HAL_GPIO_WritePin(GROUP_BSE_LED, PIN_BSE_LED, status.flt_bse_mismatch);
	HAL_GPIO_WritePin(GROUP_APPS_LED, PIN_APPS_LED, status.flt_apps_mismatch);
	HAL_GPIO_WritePin(GROUP_BSPD_LED, PIN_BSPD_LED, status.flt_bspd);
	HAL_GPIO_WritePin(GROUP_BPPC_LED, PIN_BPPC_LED, status.flt_bppc);
	HAL_GPIO_WritePin(GROUP_FLT_R_LED, PIN_FLT_R_LED, status.internal_flt_r);
	HAL_GPIO_WritePin(GROUP_FLT_NR_LED, PIN_FLT_NR_LED, status.internal_flt_nr);
}


void clearFaults() {
    status.isThrottle = 0;
    status.flt_apps_mismatch = 0;

    status.isBrake = 0;
    status.flt_bse_mismatch = 0;

    status.flt_bspd = 0;
    status.flt_bppc = 0;

    status.internal_flt_r = 0;
    status.internal_flt_nr = 0;
}



void updateFaults() {
    status.isThrottle = getIsThrottle();
    status.isBrake = getIsBrake();
    status.flt_apps_mismatch = getAppsMismatch();
    status.flt_bse_mismatch = getBseMismatch();
    status.flt_bspd = getPotato();       // BSPD fault
    status.flt_bppc = getBppcFault();

    // Set main fault flags (ACTIVE HIGH !!!)
    status.internal_flt_r  = status.flt_apps_mismatch ||
                    status.flt_bse_mismatch || status.flt_bppc;
    status.internal_flt_nr = status.flt_bspd;
}



void assertFaults() {
    // Pull FLT_R line low if there is a flt_r
    // (inversion provided by hardware)
    HAL_GPIO_WritePin(GROUP_MCU_FLT, PIN_MCU_FLT,
            status.internal_flt_r);
}



// STRUCT NAMES:
// `sensors` - all sensor readings and processed values
// `status` - all status bits and faults
uint16_t getFltR() {
    return HAL_GPIO_ReadPin(GROUP_FLT_R, PIN_FLT_R);
}

uint16_t getFltNR() {
    return HAL_GPIO_ReadPin(GROUP_FLT_NR, PIN_FLT_NR);
}

uint16_t getIsThrottle() {
    return (sensors.throttle > THROTTLE_THRESH) ? 1 : 0;
}

uint16_t getIsBrake() {
    return (sensors.brake > BRAKE_THRESH) ? 1 : 0;
}

uint16_t getAppsMismatch() {
    return (( abs(sensors.apps1 - sensors.apps2) / sensors.apps1)
               > APPS_DIFF_THRESH) ||
           (( abs(sensors.apps1 - sensors.apps2) / sensors.apps2)
               > APPS_DIFF_THRESH);
}

uint16_t getBseMismatch() {
    return (( abs(sensors.bse1 - sensors.bse2) / sensors.bse1)
              > BSE_DIFF_THRESH) ||
           (( abs(sensors.bse1 - sensors.bse2) / sensors.bse2)
              > BSE_DIFF_THRESH);
}

uint16_t getPotato() {
    return HAL_GPIO_ReadPin(GROUP_BSPD, PIN_BSPD) == LO;
}

uint16_t getBppcFault() {
    return ( (sensors.apps1 > BPPC_QTR_THROTTLE) ||
             (sensors.apps2 > BPPC_QTR_THROTTLE) ) &&
           ( (sensors.bse1 > BPPC_BRK_THRESH) ||
             (sensors.bse2 > BPPC_BRK_THRESH) );
}


// #-----------------------------# CAN FUNCTIONS #-----------------------------#



// ##### MAIN LOOP FUNCTIONS #####

void can_sendThrottle() {
    can_msg_t msg;
    CAN_short_msg(&msg, create_ID(BID_IO, MID_THROTTLE), sensors.throttle);
	CAN_queue_transmit(&msg);
}

void can_sendBrake() {
    can_msg_t msg;
    CAN_short_msg(&msg, create_ID(BID_IO, MID_BRAKE), sensors.brake);
	CAN_queue_transmit(&msg);
}



// ##### 2nd SUBROUTINE FUNCTIONS #####

// Accelerator and brake messages (2 separate) - in mainloop


// Sends isThrottle, isBrake, and mismatch faults
void can_sendPedalStatus() {

    // MESSAGE BUS:
    //      [1] - mismatch fault (active high)
    //      [0] - pedal status (active high pressed)

    // (*) Send throttle status and mismatch fault
    uint16_t msg = ( (status.flt_apps_mismatch << 1)
				   | (status.isThrottle) );
    can_msg_t can_throttle_msg;
	CAN_short_msg(&can_throttle_msg, create_ID(BID_IO, MID_THROTTLE), msg);
	CAN_queue_transmit(&can_throttle_msg);

    // (*) Send brake status and mismatch fault
    msg          = ( (status.flt_bse_mismatch << 1)
				   | (status.isBrake) );
    can_msg_t can_brake_status_msg;
	CAN_short_msg(&can_brake_status_msg, create_ID(BID_IO,
            MID_BRAKE), msg);
	CAN_queue_transmit(&can_brake_status_msg);
}


// Sends everything else
void can_sendFaultStatus() {

    // MESSAGE BUS:
    //      [1] - BPPC fault (active high)
    //      [0] - BSPD fault (active high)

    uint16_t msg = ( (status.flt_bppc << 1)
				   | (status.flt_bspd) );
    can_msg_t can_faults_msg;
    CAN_short_msg(&can_faults_msg, create_ID(BID_IO, MID_BPPC_BSPD),
            msg);
	CAN_queue_transmit(&can_faults_msg);
}

void sendCANStatuses(){
	can_sendPedalStatus();
	can_sendFaultStatus();
}

void sendHeartbeat() {
    can_msg_t msg;
	CAN_short_msg(&msg, create_ID(BID_IO, MID_HEARTBEAT), 0);
	CAN_queue_transmit(&msg);
}
