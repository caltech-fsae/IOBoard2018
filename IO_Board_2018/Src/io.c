/*
 * io.c
 *
 *  Created on: Feb 24, 2018
 *      Author: Adi
 */


#include "io.h"
#include "adc.h"

/*void readApps(uint16_t* apps1, uint16_t* apps2) {
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
*/
void readBse(uint32_t* bse1, uint32_t* bse2) {
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

/*void readCurrSensor(uint32_t* currSensor) {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK) {
        *currSensor = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
}
*/
uint16_t ADC1_read()
{
	/* Samples the ADC1 and returns the value as a uint16_t. Returns FFFF if error */
	/* Must configure "End of Conversion Selection" to
	 * "EOC flag at the end of all conversions" in CubeMX (ADC1 menu) */

	uint16_t ok;
	uint16_t adc_value = 0xFFFF;
	HAL_ADC_Start(&hadc1);
	ok = HAL_ADC_PollForConversion(&hadc1, 1000000);
	if (ok == HAL_OK)
	{
		adc_value = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	return adc_value;
}

void adcInit()
{

}

void clearFaultLEDs()
{
	HAL_GPIO_WritePin(LED_PINS_GROUP, BSPD_LED_PIN, LOW);
	HAL_GPIO_WritePin(LED_PINS_GROUP, APPS_LED_PIN, LOW);
	HAL_GPIO_WritePin(LED_PINS_GROUP, BSE_LED_PIN, LOW);
	HAL_GPIO_WritePin(LED_PINS_GROUP, BPPC_LED_PIN, LOW);
	HAL_GPIO_WritePin(LED_PINS_GROUP, FLT_LED_PIN, LOW);
	HAL_GPIO_WritePin(LED_PINS_GROUP, FLTNR_LED_PIN, LOW);
}

void displayFaultLEDs()
{
	/* BSPD LED */
	if (bspdStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSPD_LED_PIN, HIGH);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSPD_LED_PIN, LOW);
	/* APPS LED */
	if (appsMSMStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, APPS_LED_PIN, HIGH);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, APPS_LED_PIN, LOW);
	/* BSE LED */
	if (bseMSMStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSE_LED_PIN, HIGH);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, BSE_LED_PIN, LOW);
	/* BPPC LED */
	if (bppcStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, BPPC_LED_PIN, HIGH);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, BPPC_LED_PIN, LOW);
	/* FLT LED */
	if (fltStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, FLT_LED_PIN, HIGH);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, FLT_LED_PIN, LOW);
	/* FLTNR LED */
	if (fltnrStatus())
		HAL_GPIO_WritePin(LED_PINS_GROUP, FLTNR_LED_PIN, HIGH);
	else
		HAL_GPIO_WritePin(LED_PINS_GROUP, FLTNR_LED_PIN, LOW);

}

void assertFLT()
{
	HAL_GPIO_WritePin(FLT_PIN_GROUP, FLT_PIN, HIGH);
}

void resetFLT()
{
	HAL_GPIO_WritePin(FLT_PIN_GROUP, FLT_PIN, LOW);
}

int appsMSMStatus()
{
	//int apps1;
	//int apps2;

	return 0;
}

int bseMSMStatus()
{
	//int bse1;
	//int bse2;

	return 0;
}

int bppcStatus()
{
	return 0;
}

int fltStatus()
{
	return 0;
}

int fltnrStatus()
{
	return 0;
}

int bspdStatus()
{
	return !HAL_GPIO_ReadPin(BSPD_PIN_GROUP, BSPD_PIN);
}
