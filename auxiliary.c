/*
 * auxiliary.c
 *
 *  Created on: Sep 14, 2017
 *      Author: Galen
 */

#include "auxiliary.h"
#include "appi2c.h"
#include "util.h"
#include <cyu3types.h>

#define REG_LED_BRIGHTNESS 0x00
#define REG_ADC_CTRL	   0x01
#define REG_ADC_PERIOD	   0x02



/*
   Get the current LED brightness.
 */
uint8_t
LedGetBrightness (
        void)
{
    uint8_t brightness;
    I2CRead(AUX_ADDR_RD, REG_LED_BRIGHTNESS, 1, &brightness);
    return brightness;
}

/*
   Set the current LED brightness.
 */
void
LedSetBrightness (
        uint8_t brightness)
{
	I2CWrite (AUX_ADDR_WR, REG_LED_BRIGHTNESS, 1, &brightness);
}

/*
   Start taking and transmitting ADC reads
 */
void
ScopeAdcStart (
		CyBool_t multiplex_req,
		uint16_t adc_per)
{
	uint8_t buf[2];
	uint8_t ctrl_val;
	FillBuff2B (adc_per, buf);
	I2CWrite2B (AUX_ADDR_WR, REG_ADC_PERIOD, buf[0], buf[1]);
	ctrl_val = multiplex_req ? 0x03 : 0x01;
	I2CWrite (AUX_ADDR_WR, REG_ADC_CTRL, 1, &ctrl_val);
}

/*
   Get current ADC period on scope
 */
uint16_t
ScopeGetAdcPeriod (
		void)
{
	uint8_t buf[2];
	I2CRead2B (AUX_ADDR_RD, REG_ADC_PERIOD, buf);
	return Combine2B(buf);
}

/*
   Stop taking and transmitting ADC reads
 */
void
ScopeAdcStop (
        void)
{
	uint8_t ctrl_val = 0x00;
	I2CWrite (AUX_ADDR_WR, REG_ADC_CTRL, 1, &ctrl_val);
}
