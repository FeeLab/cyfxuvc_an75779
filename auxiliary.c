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
