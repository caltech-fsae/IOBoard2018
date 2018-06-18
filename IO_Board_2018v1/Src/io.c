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
extern uint16_t adcValues[5];

int can_nr = 0;
int flt_delta = 0;
int just_fault = 0;
// Global Variables
int ignore_nr_start_time;

struct Sensors {
    // Raw values from ADCs
    int apps1, apps2, bse1, bse2, rawCurrent;
    // Filtered and scaled values
    int throttle, brake, current;
} sensors;

struct Status {
    uint16_t isThrottle;
    uint16_t isBrake;
} status;

struct Faults {
    uint16_t flt_apps_mismatch;
    uint16_t flt_bse_mismatch;
    uint16_t internal_flt_r;
    uint16_t internal_flt_nr;
    uint16_t flt_bspd;        // Strictly a software indication of BSPD fault
    uint16_t flt_bppc;
} faults;

struct Flags {
	uint16_t rst_all_flts;
	uint16_t flt_hit;
} flags;

// #--------------------------# MAIN FUNCTIONS #---------------------------#

void init(){
  HAL_Delay(STARTUP_GRACE_PERIOD);
  clearFaults();
  faults.flt_bspd = 0;
  clearStatuses();
  clearFlags();
  clearLEDs();
  while(getBspdFault()){}
}

void mainLoop(){
	readRawApps(hadc3);
	readRawBse(hadc1);
	readRawCurr(hadc2);

	updateSensors();
	updateFaults();
	updateStatuses();
	updateLEDs();

	assertFaults();

	can_sendBrake();
	can_sendThrottle();
	if(flt_delta) {
		can_msg_t msg;
		CAN_short_msg(&msg, create_ID(BID_IO, MID_FLT_CLEAR), 0);
		CAN_queue_transmit(&msg);
	}
	can_nr = 0;
}

// #--------------------------# READ ADC FUNCTIONS #---------------------------#

void readRawApps(ADC_HandleTypeDef hadc) {
	sensors.apps1 = adcValues[0];
	sensors.apps2 = adcValues[1];
}


void readRawBse(ADC_HandleTypeDef hadc) {
	sensors.bse1 = adcValues[2];
	sensors.bse2 = adcValues[3];
}

void readRawCurr(ADC_HandleTypeDef hadc) {
	sensors.rawCurrent = adcValues[4];
}

void updateThrottle() {
    sensors.throttle = ((sensors.apps1 + sensors.apps2) / 2);
    sensors.throttle -= THROTTLE_THRESH;
    if (sensors.throttle < 0)
		sensors.throttle = 0;
    else
    	sensors.throttle = (int) ((float) sensors.throttle * 2.2);
}

void updateBrake() {
    sensors.brake = (sensors.bse1 + sensors.bse2) / 2;
}

void updateCurrent() {
    sensors.current = sensors.rawCurrent;
}

void updateSensors() {
	updateThrottle();
	updateBrake();
	updateCurrent();
}

void updateLEDs() {
	// TODO(@ElectronicToast): RESTORE MISMATCH INDICATORS
	/* BSE LED */
	HAL_GPIO_WritePin(GROUP_BSE_LED, PIN_BSE_LED, status.isBrake);
	/* APPS LED */
	HAL_GPIO_WritePin(GROUP_APPS_LED, PIN_APPS_LED, status.isThrottle);
	/* BSPD LED */
	HAL_GPIO_WritePin(GROUP_BSPD_LED, PIN_BSPD_LED, faults.flt_bspd);
	/* BPPC LED */
	HAL_GPIO_WritePin(GROUP_BPPC_LED, PIN_BPPC_LED, faults.flt_bppc);
	/* FLT_R LED */
	HAL_GPIO_WritePin(GROUP_FLT_R_LED, PIN_FLT_R_LED, faults.internal_flt_r);
	/* FLT_NR LED */
	HAL_GPIO_WritePin(GROUP_FLT_NR_LED, PIN_FLT_NR_LED, faults.internal_flt_nr);
}


void clearLEDs() {
	HAL_GPIO_WritePin(GROUP_BSE_LED, PIN_BSE_LED, LO);
    HAL_GPIO_WritePin(GROUP_APPS_LED, PIN_APPS_LED, LO);
    HAL_GPIO_WritePin(GROUP_BSPD_LED, PIN_BSPD_LED, LO);
    HAL_GPIO_WritePin(GROUP_BPPC_LED, PIN_BPPC_LED, LO);
    HAL_GPIO_WritePin(GROUP_FLT_R_LED, PIN_FLT_R_LED, LO);
    HAL_GPIO_WritePin(GROUP_FLT_NR_LED, PIN_FLT_NR_LED, LO);
}


void clearFaults() {
    faults.flt_apps_mismatch = 0;
    faults.flt_bse_mismatch = 0;
    //faults.flt_bspd = 0;
    faults.flt_bppc = 0;
    faults.internal_flt_r = 0;
    //faults.internal_flt_nr = 0;
}

void clearStatuses() {
	status.isBrake = 0;
	status.isThrottle = 0;
}

void clearFlags() {
	flags.rst_all_flts = 0;
	flags.flt_hit = 0;
}

void updateStatuses() {
    status.isThrottle = getIsThrottle();
    status.isBrake = getIsBrake();
}

void updateFaults() {
    faults.flt_apps_mismatch = getAppsMismatch();
    faults.flt_bse_mismatch = getBseMismatch();
    faults.flt_bspd = faults.flt_bspd || getBspdFault();
    faults.flt_bppc = getBppcFault();

    // Set main fault flags (ACTIVE HIGH !!!)
    faults.internal_flt_r  = faults.flt_apps_mismatch || faults.flt_bse_mismatch || faults.flt_bppc;
    just_fault |= faults.internal_flt_r;
    if(just_fault && !faults.internal_flt_r) {
    	flt_delta = 1;
    	just_fault = 0;
    }

    if(flags.rst_all_flts == 0)
    {
    //	HAL_GPIO_WritePin(GROUP_FLT_R_LED, PIN_FLT_R_LED, getFltNR());
    	faults.internal_flt_nr = faults.flt_bspd || getFltNR() || can_nr;
    	if(faults.internal_flt_nr) {
    		can_msg_t msg;
			CAN_short_msg(&msg, create_ID(BID_IO, MID_FAULT_NR), 0);
			CAN_queue_transmit(&msg);
    	}
    }
    else
    {
    	faults.internal_flt_nr = faults.flt_bspd;
    	flags.flt_hit = 0;
    	if(HAL_GetTick() - ignore_nr_start_time > IGNORE_FLT_NR_GRACE_PERIOD)
    	{
    		flags.rst_all_flts = 0;
    	}
    	if(!faults.internal_flt_nr) {
    		can_msg_t msg;
    		CAN_short_msg(&msg, create_ID(BID_IO, MID_PROVIDE_NR_RESET_CONSENT), 0);
    		CAN_queue_transmit(&msg);
    	} else {
    		can_msg_t msg;
    		CAN_short_msg(&msg, create_ID(BID_IO, MID_FAULT_NR), 0);
    		CAN_queue_transmit(&msg);
    	}
    }
}



void assertFaults() {
    // Pull FLT_R line low if there is a flt_r
    // (inversion provided by hardware)
	if(faults.internal_flt_r) {
		can_msg_t msg;
		CAN_short_msg(&msg, create_ID(BID_IO, MID_FAULT), 0);
		CAN_queue_transmit(&msg);
	}
    HAL_GPIO_WritePin(GROUP_MCU_FLT, PIN_MCU_FLT, faults.internal_flt_r);
    HAL_GPIO_WritePin(GROUP_MCU_FLT_NR, PIN_MCU_FLT_NR, faults.internal_flt_nr);
}



// STRUCT NAMES:
// `sensors` - all sensor readings and processed values
// `status` - all status bits and faults
uint16_t getFltR() {
    return HAL_GPIO_ReadPin(GROUP_FLT_R, PIN_FLT_R) == LO;
}

uint16_t getFltNR() {
	if(flags.flt_hit) {
		return 1;
	}
    static uint16_t deb_ctr = 0;
    uint16_t flt_nr = HAL_GPIO_ReadPin(GROUP_FLT_NR, PIN_FLT_NR);
	if((flt_nr == LO) && (deb_ctr < 100)) {
		deb_ctr++;
		return 0;
	}
    else
    {
    		deb_ctr = 0;
    		if(HAL_GPIO_ReadPin(GROUP_FLT_NR, PIN_FLT_NR) == LO) {
    			flags.flt_hit = 1;
    			return 1;
    		}
    		return 0;
    }
}

uint16_t getIsThrottle() {
    return (sensors.throttle > 0);
}

uint16_t getIsBrake() {
    return (sensors.brake > 0);
}

uint16_t getAppsMismatch() {
    return (abs(sensors.apps1 - sensors.apps2) > APPS_DIFF_THRESH);
}

uint16_t getBseMismatch() {
    return (abs(sensors.bse1 - sensors.bse2) > BSE_DIFF_THRESH);
}

uint16_t getBspdFault() {
    return HAL_GPIO_ReadPin(GROUP_BSPD, PIN_BSPD) == LO;
}

uint16_t getBppcFault() {
    if (faults.flt_bppc) {
      if (status.isThrottle == 0) {
        return 0;
      } else {
        return 1;
      }
    }
    return (status.isThrottle && status.isBrake);
}


// #-----------------------------# CAN FUNCTIONS #-----------------------------#

void readCANMessages() {
	can_msg_t msg;
	while(CAN_dequeue_msg(&msg)) {
		uint16_t type = 0b0000011111110000 & msg.identifier;
		if ( (flags.rst_all_flts == 0) && (type == MID_ATTEMPT_RESET) ) {
			flags.rst_all_flts = 1;
			ignore_nr_start_time = HAL_GetTick();
		} else if (type == MID_FAULT_NR) {
			can_nr = 1;
		} else if (type == MID_FLT_CLEAR_ACK) {
			flt_delta = 0;
		}
	}
}

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
    //		[3] - apps mismatch fault (active high)
	//		[2] - bse mismatch fault (active high)
	//      [1] - throttle pedal status (active high pressed)
    //      [0] - brake pedal status (active high pressed)

    // (*) Send throttle status and mismatch fault
    uint16_t msg = ( (faults.flt_apps_mismatch << 3)
				   | (faults.flt_bse_mismatch << 2)
				   | (status.isThrottle << 1)
				   | (status.isBrake));
    can_msg_t can_throttle_msg;
	CAN_short_msg(&can_throttle_msg, create_ID(BID_IO, MID_PEDAL_STATUS), msg);
	CAN_queue_transmit(&can_throttle_msg);
}


// Sends everything else
void can_sendFaultStatus() {

    // MESSAGE BUS:
    //      [1] - BPPC fault (active high)
    //      [0] - BSPD fault (active high)

    uint16_t msg = ( (faults.flt_bppc << 1)
				   | (faults.flt_bspd) );
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
