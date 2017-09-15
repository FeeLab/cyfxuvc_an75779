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

#endif /* AUXILIARY_H_ */
