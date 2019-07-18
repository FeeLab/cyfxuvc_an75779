/*
 * auxiliary.h
 *
 *  Created on: Sep 14, 2017
 *      Author: Galen
 */

#ifndef AUXILIARY_H_
#define AUXILIARY_H_

#include <cyu3types.h>

/* I2C address for the LED. */
#define AUX_ADDR_WR		0x98
#define AUX_ADDR_RD		0x99

/* Minimum safe ADC sampling period. */
#define REG_ADC_PERIOD_DEFAULT_MSB 0x00
#define REG_ADC_PERIOD_DEFAULT_LSB 0x34

/*
   Get the current LED brightness.
 */
uint8_t
LedGetBrightness (
        void);

/*
   Set the current LED brightness.
 */
void
LedSetBrightness (
        uint8_t brightness);

/*
   Start sampling from the ADC.
 */
void
AdcStart(
		CyBool_t adc_multiplex,
		uint8_t adc_period_msb,
		uint8_t adc_period_lsb);

/*
   Stop sampling from the ADC.
 */
void
AdcStop(void);

#endif /* AUXILIARY_H_ */
