/*
 * io.c
 *
 *  Created on: Feb 24, 2018
 *      Author: Adi
 */


#include "io.h"
#include <stdlib.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

// Global Variables
struct Sensors {
    // Raw values from ADCs
    uint16_t apps1, apps2, bse1, bse2, currSense;
    uint16_t scaled_apps1, scaled_apps2, scaled_bse1, scaled_bse2;
    uint16_t apps_scale, brake_scale;
    uint16_t prevApps1, prevApps2, prevBse1, prevBse2, prevCurr;
    float avg_apps1, avg_apps2, avg_bse1, avg_bse2, avg_curr;
    int ind_apps1, ind_apps2, ind_bse1, ind_bse2, ind_curr;
    uint16_t data_apps1[APPS_AVG_SAMPLE_SIZE], data_apps2[APPS_AVG_SAMPLE_SIZE],
		data_bse1[BSE_AVG_SAMPLE_SIZE], data_bse2[BSE_AVG_SAMPLE_SIZE],
		data_curr[CURR_AVG_SAMPLE_SIZE];

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

void init(){
  init_sensors();
  clearFaults();
}

void mainLoop(){
	readApps(hadc3);
	readBse(hadc1);
	readCurr(hadc2);

	gen_avg_bse();
	gen_avg_apps();
	gen_avg_curr();

  scaleThrottle();
	scaleBrake();
	scaleCurrent();

	filterApps();
	filterBse();
	filterCurr();

	updateFaults();
	updateLEDs();
	assertFaults();

	can_sendBrake();
	can_sendThrottle();
}

// #--------------------------# READ ADC FUNCTIONS #---------------------------#

void readApps(ADC_HandleTypeDef hadc) {
	// Run first conversin on the APPS ADC group
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
        // Read APPS1
        sensors.apps1 = HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);

    // Run a second conversino on the same ADC (second channel)
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
    	// Read APPS2
        sensors.apps2 = HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);
}


void readBse(ADC_HandleTypeDef hadc) {
	// Run first conversion on the BSE ADC group
	HAL_ADC_Start(&hadc);
	if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
		// Read BSE1
		sensors.bse1 = HAL_ADC_GetValue(&hadc);
	}
	HAL_ADC_Stop(&hadc);

	// Run a second conversion on the same ADC (second channel)
	HAL_ADC_Start(&hadc);
	if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
		// Read BSE2
		sensors.bse2 = HAL_ADC_GetValue(&hadc);
	}
	HAL_ADC_Stop(&hadc);
}


void readCurr(ADC_HandleTypeDef hadc) {
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 100000) == HAL_OK) {
        sensors.currSense = HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);
}


// #-------------------------# PROCESSING FUNCTIONS #--------------------------#

//Modify running average, for filtering purposes
void gen_avg_apps() {
  sensors.avg_apps1 += (float) sensors.apps1 / (float) APPS_AVG_SAMPLE_SIZE;
  sensors.avg_apps1 -= (float) sensors.data_apps1[sensors.ind_apps1] / (float) APPS_AVG_SAMPLE_SIZE;
  sensors.data_apps1[sensors.ind_apps1] = sensors.apps1;
  sensors.ind_apps1 += 1;
  sensors.ind_apps1 %= APPS_AVG_SAMPLE_SIZE;

  sensors.avg_apps2 += (float) sensors.apps2 / (float) APPS_AVG_SAMPLE_SIZE;
  sensors.avg_apps2 -= (float) sensors.data_apps2[sensors.ind_apps2] / (float) APPS_AVG_SAMPLE_SIZE;
  sensors.data_apps2[sensors.ind_apps2] = sensors.apps2;
  sensors.ind_apps2 += 1;
  sensors.ind_apps2 %= APPS_AVG_SAMPLE_SIZE;
}

void gen_avg_bse() {
  sensors.avg_bse1 += (float) sensors.bse1 / (float) BSE_AVG_SAMPLE_SIZE;
  sensors.avg_bse1 -= (float) sensors.data_bse1[sensors.ind_bse1] / (float) BSE_AVG_SAMPLE_SIZE;
  sensors.data_bse1[sensors.ind_bse1] = sensors.bse1;
  sensors.ind_bse1 += 1;
  sensors.ind_bse1 %= BSE_AVG_SAMPLE_SIZE;

  sensors.avg_bse2 += (float) sensors.bse2 / (float) BSE_AVG_SAMPLE_SIZE;
  sensors.avg_bse2 -= (float) sensors.data_bse2[sensors.ind_bse2] / (float) BSE_AVG_SAMPLE_SIZE;
  sensors.data_bse2[sensors.ind_bse2] = sensors.bse2;
  sensors.ind_bse2 += 1;
  sensors.ind_bse2 %= BSE_AVG_SAMPLE_SIZE;
}

void gen_avg_curr() {
  sensors.avg_curr += (float) sensors.currSense / (float) CURR_AVG_SAMPLE_SIZE;
  sensors.avg_curr -= (float) sensors.data_curr[sensors.ind_curr] / (float) CURR_AVG_SAMPLE_SIZE;
  sensors.data_curr[sensors.ind_curr] = sensors.currSense;
  sensors.ind_curr += 1;
  sensors.ind_curr %= CURR_AVG_SAMPLE_SIZE;
}

// Filter both APPS readings
void filterApps() {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
    if (abs(sensors.avg_apps1 - sensors.scaled_apps1) > APPS_VAL_THRESH) {
        sensors.apps1 = sensors.prevApps1;
      } else {
        sensors.prevApps1 = sensors.apps1;
      }

      if (abs(sensors.avg_apps2 - sensors.scaled_apps2) > APPS_VAL_THRESH) {
        sensors.scaled_apps2 = sensors.prevApps2;
      } else {
        sensors.prevApps2 = sensors.scaled_apps2;
      }
      sensors.throttle = (sensors.scaled_apps1 + sensors.scaled_apps2) / 2;
}

// Filter a single BSE reading
void filterBse() {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
    if (abs(sensors.avg_bse1 - sensors.scaled_bse1) > BSE_VAL_THRESH) {
        sensors.scaled_bse1 = sensors.prevBse1;
      } else {
        sensors.prevBse1 = sensors.scaled_bse1;
      }

      if (abs(sensors.avg_bse2 - sensors.scaled_bse2) > BSE_VAL_THRESH) {
        sensors.scaled_bse2 = sensors.prevBse2;
      } else {
        sensors.prevBse2 = sensors.scaled_bse2;
      }
      sensors.brake = (sensors.scaled_bse1 + sensors.scaled_bse2) / 2;

}

/**
 * DEPRECATED
 */
// Filters the current sensor reading
void filterCurr() {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
    if (abs(sensors.avg_curr - sensors.currSense) > CURRSENSE_VAL_THRESH) {
      sensors.currSense = sensors.prevCurr;
    } else {
      sensors.prevCurr = sensors.currSense;
    }
}

uint16_t scaleApps(uint16_t val) {
	return (val - APPS_MIN) * sensors.apps_scale;
}

uint16_t scaleBse(uint16_t val) {
	return (val - BSE_MIN) * sensors.brake_scale;
}

/*
void scaleValue(bool isThrottle, uint16_t val) {
    if (isThrottle) {
      return (val - APPS_MIN) * sensors.apps_scale;
    }
    else {
      return (val - BSE_MIN) * sensors.brake_scale;
    }
}
*/

void scaleThrottle() {
    sensors.apps2 -= APPS_OFFSET;
    sensors.scaled_apps1 = scaleApps(sensors.apps1);
    sensors.scaled_apps2 = scaleApps(sensors.apps2);
}

void scaleBrake() {
    sensors.bse2 -= BSE_OFFSET;
    sensors.scaled_bse1 = scaleBse(sensors.bse1);
    sensors.scaled_bse2 = scaleBse(sensors.bse2);
}

void scaleCurrent() {
    sensors.current = sensors.currSense;
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

void init_sensors() {
  sensors.ind_apps1 = sensors.ind_apps2 = sensors.ind_bse1 = sensors.ind_bse2 = sensors.ind_curr = 0;
  sensors.avg_bse1 = sensors.avg_bse2 = sensors.avg_apps1 = sensors.avg_apps2 = sensors.avg_curr = 0.0;
  sensors.prevCurr = sensors.prevBse1 = sensors.prevBse2 = sensors.prevApps1 = sensors.prevApps2 = 0;
  sensors.apps_scale = 4096 / (APPS_MAX - APPS_MIN);
  sensors.brake_scale = 4096 / (BSE_MAX - BSE_MIN);
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
    //HAL_GPIO_WritePin(GROUP_MCU_FLT, PIN_MCU_FLT, status.internal_flt_r);
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
    return (abs(sensors.apps1 - sensors.apps2) > APPS_DIFF_THRESH);
}

uint16_t getBseMismatch() {
    return (abs(sensors.bse1 - sensors.bse2) > BSE_DIFF_THRESH);
}

uint16_t getPotato() {
    return HAL_GPIO_ReadPin(GROUP_BSPD, PIN_BSPD) == LO;
}

uint16_t getBppcFault() {
    if (status.flt_bppc) {
      if (sensors.throttle < BPPC_STOP_THRESH) {
        return 0;
      } else {
        return 1;
      }
    }
    return ( (sensors.apps1 > BPPC_QTR_THROTTLE) ||
             (sensors.apps2 > BPPC_QTR_THROTTLE + 1000) ) &&
           ( (sensors.bse1 > BPPC_BRK_THRESH) ||
             (sensors.bse2 > BPPC_BRK_THRESH + 1000) );
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
	CAN_short_msg(&can_throttle_msg, create_ID(BID_IO, MID_THROTTLE_STATUS), msg);
	CAN_queue_transmit(&can_throttle_msg);

    // (*) Send brake status and mismatch fault
    msg          = ( (status.flt_bse_mismatch << 1)
				   | (status.isBrake) );
    can_msg_t can_brake_status_msg;
	CAN_short_msg(&can_brake_status_msg, create_ID(BID_IO, MID_BRAKE_STATUS), msg);
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
	//TODO(@bgberr): uncomment
	can_sendFaultStatus();
}

void sendHeartbeat() {
    can_msg_t msg;
	CAN_short_msg(&msg, create_ID(BID_IO, MID_HEARTBEAT), 0);
	CAN_queue_transmit(&msg);
}
