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
#define AUX_ADDR_WR		  0x98
#define AUX_ADDR_RD		  0x99
#define AUX_ADC_MIN_PER 0x0034

/*
   Get the current LED brightness.
 */
extern uint8_t
LedGetBrightness (
        void);

/*
   Set the current LED brightness.
 */
extern void
LedSetBrightness (
        uint8_t brightness);

/*
   Function    : ScopeAdcStart
   Description : Starting taking and transmitting ADC reads from scope
   Parameters  :
                 multiplex_req - true or false to multiplex.
                 adc_per - ADC period
 */
extern void
ScopeAdcStart (
		CyBool_t multiplex_req,
		uint16_t adc_per);

/*
   Function    : ScopeGetAdcPeriod
   Description : Get period of ADC on scope
   Parameters  : None
 */
extern uint16_t
ScopeGetAdcPeriod (
		void);

/*
   Function    : ScopeAdcStop
   Description : Stop taking and transmitting ADC reads from scope
   Parameters  : None
 */
extern void
ScopeAdcStop (
        void);

#endif /* AUXILIARY_H_ */
