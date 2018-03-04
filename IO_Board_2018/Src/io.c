/*
 * io.c
 *
 *  Created on: Feb 24, 2018
 *      Author: Adi
 */


#include "io.h"
#include "adc.h"


// #--------------------------# READ ADC FUNCTIONS #---------------------------#

void readApps(uint16_t* apps1, uint16_t* apps2, ADC_HandleTypeDef hadc3) {
    HAL_ADC_Start(&hadc3);

    // Poll the entire ADC2 group for conversion
    if (HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY) == HAL_OK) {
        // Read APPS1
        *apps1 = (&hadc3);
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
        bse1 = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        // Read BSE2
        bse2 = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
}


void readCurr(uint16_t* currSensor, ADC_HandleTypeDef hadc2) {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK) {
        *currSensor = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
}


// #-------------------------# PROCESSING FUNCTIONS #--------------------------#


// Filter a single APPS reading
void filterApps(uint16_t* prevApps, uint16_t* apps) {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
    *apps = (abs(*apps - *prevApps) > APPS_VAL_THRESH)
        ? *prevApps : *apps;

    // Update old value for next iteration
    *prevApps = *apps;
}


// Filter a single BSE reading
void filterBse(uint16_t* prevBse, uint16_t* bse) {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
    *bse = (abs(*bse - *prevBse) > BSE_VAL_THRESH)
        ? *prevBse : *bse;

    // Update old value for next iteration
    *prevBse = *bse;
}


// Filters the current sensor reading
void filterCurr(uint16_t* prevCurr, uint16_t* curr) {
    // Compute new value - if there is a huge difference, then assert the
    // previous value but store it so that it can be compared with the
    // reading from the next iteration. Else, the new value goes through.
    *curr = (abs(*curr - *prevCurr) > CURRSENSE_VAL_THRESH)
        ? *prevCurr : *curr;

    // Update old value for next iteration
    *prevCurr = *curr;
}

// #-------------------------# DIGITAL READ FUNCTIONS #--------------------------#

int bspdStatus() {
    return (HAL_GPIO_ReadPin(LED_PINS_GROUP, BSPD_PIN) == LO);
}

int fltStatus(){
    return (HAL_GPIO_ReadPin(FLT_PIN_GROUP, FLT_PIN) == LO);
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

void displayFaultLEDs()
{
	/* BSPD LED */
	if (bspdStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSPD_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSPD_LED_PIN, LO);
	/* APPS LED */
	if (appsStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, APPS_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, APPS_LED_PIN, LO);
	/* BSE LED */
	if (bseStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSE_LED_PIN, HI);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSE_LED_PIN, LO);
	/* BPPC LED */
	if (bppcStatus())
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
