/*
 * auxiliary.c
 *
 *  Created on: Sep 14, 2017
 *      Author: Galen
 */

#include "auxiliary.h"
#include "appi2c.h"
#include <cyu3types.h>

/*
   Get the current LED brightness.
 */
uint8_t
LedGetBrightness (
        void)
{
    uint8_t buf[1];

    I2CReadNoReg (AUX_ADDR_RD, buf);
    return (uint8_t) buf[0];
}

/*
   Set the current LED brightness.
 */
void
LedSetBrightness (
        uint8_t brightness)
{
    I2CWriteNoReg (AUX_ADDR_WR, brightness);
}
