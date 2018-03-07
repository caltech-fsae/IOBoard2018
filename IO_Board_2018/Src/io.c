/*
 * io.c
 *
 *  Created on: Feb 24, 2018
 *      Author: Adi
 */


#include "io.h"
#include "adc.h"

// Global Variables
/*struct faults_t {
	uint16_t lv_battery_fault;	// set battery fault
	uint16_t interlock_in_fault;
	uint16_t flt_fault;
	uint16_t flt_nr_fault;
	uint16_t imd_fault;
	uint16_t ams_fault;
	uint16_t bspd_fault;
} faults;*/

// #--------------------------# READ ADC FUNCTIONS #---------------------------#

void readApps(uint16_t* apps1, uint16_t* apps2, ADC_HandleTypeDef hadc3) {
    HAL_ADC_Start(&hadc3);

    // Poll the entire ADC3 group for conversion
    if (HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY) == HAL_OK) {
        // Read APPS1
        *apps1 = HAL_ADC_GetValue(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
        // Read APPS2
        *apps2 = HAL_ADC_GetValue(&hadc3);
    }

    HAL_ADC_Stop(&hadc3);
}


void readBse(uint16_t* bse1, uint16_t* bse2, ADC_HandleTypeDef hadc1) {
    HAL_ADC_Start(&hadc1);

    // Poll the entire ADC1 group for conversion
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        // Read BSE1
        *bse1 = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        // Read BSE2
        *bse2 = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
}


void readCurr(uint16_t* currSensor, ADC_HandleTypeDef hadc2) {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK)
        *currSensor = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
}


// #-------------------------# PROCESSING FUNCTIONS #--------------------------#


// Filter a single APPS reading
void filterApps(uint16_t* prevApps, uint16_t* apps) {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
	uint16_t newApps = *apps;
	if (abs(*apps - *prevApps) > APPS_VAL_THRESH)
		*apps = *prevApps;
	// Update old value for next iteration
	*prevApps = newApps;
}


// Filter a single BSE reading
void filterBse(uint16_t* prevBse, uint16_t* bse) {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
	uint16_t newBse = *bse;
	if (abs(*bse - *prevBse) > BSE_VAL_THRESH)
        *bse = prevBse;
    // Update old value for next iteration
    *prevBse = newBse;
}


// Filters the current sensor reading
void filterCurr(uint16_t* prevCurr, uint16_t* curr) {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
    uint16_t newCurr = *curr;
	if (abs(*curr - *prevCurr) > CURRSENSE_VAL_THRESH)
    	*curr = *prevCurr;
    // Update old value for next iteration
    *prevCurr = newCurr;
}

// #-------------------------# DIGITAL READ FUNCTIONS #--------------------------#

int bspdStatus() {
    return (HAL_GPIO_ReadPin(LED_PINS_GROUP, BSPD_PIN) == LO);
}

int fltStatus(){
    return (HAL_GPIO_ReadPin(FLT_PINS_GROUP, FLT_PIN) == LO);
}

int fltNrStatus(){
    return (HAL_GPIO_ReadPin(FLT_PINS_GROUP, FLT_NR_PIN) == LO);
}

// APPS plausibility check fault - if difference exceeds 10% (
// (currently, percent error computed either way)
int appsStatus(uint16_t* apps1, uint16_t* apps2) {
    return (( abs(*apps1 - *apps2) / *apps1) > APPS_DIFF_THRESH) ||
           (( abs(*apps1 - *apps2) / *apps2) > APPS_DIFF_THRESH);
}

// BSE plausibility check (not a fault in the rules, but same idea. Something
// that core board should know)
int bseStatus(uint16_t* bse1, uint16_t* bse2) {
    return (( abs(*bse1 - *bse2) / *bse1) > BSE_DIFF_THRESH) ||
           (( abs(*bse1 - *bse2) / *bse2) > BSE_DIFF_THRESH);
}

// BPPC fault - currently: if either accelerator sensor reads > 25% and either
// brake sensor is "actuated"
int bppcStatus(uint16_t* apps1, uint16_t* apps2, uint16_t* bse1, uint16_t* bse2) {
    return ( (*apps1 > BPPC_QTR_THROTTLE) || (*apps2 > BPPC_QTR_THROTTLE) ) &&
           ( (*bse1 > BPPC_BRK_THRESH) || (*bse2 > BPPC_BRK_THRESH) );
}

void clearFaultLEDs()
{
	HAL_GPIO_WritePin(LED_PINS_GROUP, BSPD_LED_PIN, LO);
	HAL_GPIO_WritePin(LED_PINS_GROUP, APPS_LED_PIN, LO);
	HAL_GPIO_WritePin(LED_PINS_GROUP, BSE_LED_PIN, LO);
	HAL_GPIO_WritePin(LED_PINS_GROUP, BPPC_LED_PIN, LO);
	HAL_GPIO_WritePin(LED_PINS_GROUP, FLT_LED_PIN, LO);
	HAL_GPIO_WritePin(LED_PINS_GROUP, FLTNR_LED_PIN, LO);
}

void displayFaultLEDs(uint16_t* apps1, uint16_t* apps2, uint16_t* bse1, uint16_t* bse2)
{
	/* BSPD LED */
	if (bspdStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSPD_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSPD_LED_PIN, LO);
	/* APPS LED */
	if (appsStatus(apps1, apps2))
		HAL_GPIO_WritePin(LED_PINS_GROUP, APPS_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, APPS_LED_PIN, LO);
	/* BSE LED */
	if (bseStatus(bse1, bse2))
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSE_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSE_LED_PIN, LO);
	/* BPPC LED */
	if (bppcStatus(apps1, apps2, bse1, bse2))
		HAL_GPIO_WritePin(LED_PINS_GROUP, BPPC_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, BPPC_LED_PIN, LO);
	/* FLT LED */
	if (fltStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, FLT_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, FLT_LED_PIN, LO);
	/* FLTNR LED */
	if (fltNrStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, FLTNR_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, FLTNR_LED_PIN, LO);
}

void assertMcuFlt()
{
	HAL_GPIO_WritePin(FLT_PINS_GROUP, MCU_FLT_PIN, HI);
}

void resetMcuFlt()
{
	HAL_GPIO_WritePin(FLT_PINS_GROUP, MCU_FLT_PIN, LO);
}

void checkCANMessages()
{
	can_msg_t msg;
	if(CAN_dequeue_msg(&msg)) {
		uint16_t type = 0b0000011111110000 & msg.identifier;
			//if(type == MID_FAULT_NR)
				//assertFLT_NR();
			//else if(type == MID_FAULT)
				//assertFLT();
	}
}

void sendHeartbeat()
{
	can_msg_t msg;
	CAN_short_msg(&msg, create_ID(BID_IO, MID_HEARTBEAT), 0);
	CAN_queue_transmit(&msg);
}

void mainLoop()
{
	// Read BSPD, FLT, and FLT_NR.
	// Respond by asserting FLT or FLT_NR high accordingly.
	// Read sensor values.
	// Update internal and external faults. CAN
	// Send out torque and brake values. CAN
}

