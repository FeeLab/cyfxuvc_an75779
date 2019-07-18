/*
 * auxiliary.c
 *
 *  Created on: Sep 14, 2017
 *      Author: Galen
 */

#include "auxiliary.h"
#include "appi2c.h"
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
   Start sampling from the ADC.
 */
void
AdcStart(
		CyBool_t adc_multiplex,
		uint8_t adc_period_msb,
		uint8_t adc_period_lsb)
{
	uint8_t ctrl_value = adc_multiplex ? 0x03 : 0x01;
	I2CWrite2B (AUX_ADDR_WR, REG_ADC_PERIOD, adc_period_msb, adc_period_lsb);
	I2CWrite (AUX_ADDR_WR, REG_ADC_CTRL, 1, &ctrl_value);
}

/*
   Stop sampling from the ADC.
 */
void
AdcStop(void)
{
	uint8_t ctrl_value = 0x00;
	I2CWrite (AUX_ADDR_WR, REG_ADC_CTRL, 1, &ctrl_value);
}
