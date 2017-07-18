/*
 ## Cypress FX3 Camera Kit header file (sensor.h)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file defines the parameters and the interface for the EV76C541 image
   sensor driver.
 */

#ifndef _INCLUDED_SENSOR_H_
#define _INCLUDED_SENSOR_H_

#include <cyu3types.h>

/* The SADDR line allows EV76C541 image sensor to select between two different I2C slave address.
   If the SADDR line is high, enable this #define to allow access to the correct I2C address for the sensor.
 */

/* I2C address for the LED. */
#define LED_ADDR_WR		0x98
#define LED_ADDR_RD		0x99

/* I2C address for the Deserializer. */
#define DESER_ADDR_WR	0xC0
#define DESER_ADDR_RD   0xC1

/* I2C address for the Serializer. */
#define SER_ADDR_WR		0xB0
#define SER_ADDR_RD		0xB1

/* I2C Slave address for the image sensor. */
#define SENSOR_ADDR_WR 0xB8             /* Slave address used to write sensor registers. */
#define SENSOR_ADDR_RD 0xB9             /* Slave address used to read from sensor registers. */

#define I2C_MEMORY_ADDR_WR 0xA0         /* I2C slave address used to write to an EEPROM. */
#define I2C_MEMORY_ADDR_RD 0xA1         /* I2C slave address used to read from an EEPROM. */

/* Communication over saturation channel */
#define SATURATION_RECORD_START	0x01
#define SATURATION_RECORD_END	0x02
#define SATURATION_INIT 		0x03
#define SATURATION_FPS5			0x11
#define SATURATION_FPS10		0x12
#define	SATURATION_FPS15		0x13
#define	SATURATION_FPS20		0x14
#define SATURATION_FPS30		0x15
#define	SATURATION_FPS60		0x16

/* Function     : SensorConfigureSerdes
   Description  : Configure the Serdes channel to CMOS board.
   Parameters   : None
 */
extern void
SensorConfigureSerdes (
        void);

/* Function    : SensorInit
   Description : Initialize the EV76C541 sensor.
   Parameters  : None
 */
extern void
SensorInit (
        void);

extern void
SensorStart (
		void);

extern void
SensorStop (
		void);

/* Function    : SensorReset
   Description : Reset the EV76C541 image sensor using FX3 GPIO.
   Parameters  : None
 */
extern void
SensorReset (
        void);

/* Function     : SensorScaling_HD720p_30fps
   Description  : Configure the EV76C541 sensor for 720p 30 fps video stream.
   Parameters   : None
 */
extern void
SensorScaling_752_480_30fps (
        void);

/* Function    : SensorI2cBusTest
   Description : Test whether the EV76C541 sensor is connected on the I2C bus.
   Parameters  : None
 */
extern uint8_t
SensorI2cBusTest (
        void);

/* Function    : SensorGetBrightness
   Description : Get the current brightness setting from the EV76C541 sensor.
   Parameters  : None
 */
extern uint8_t
SensorGetBrightness (
        void);

/* Function    : SensorSetBrightness
   Description : Set the desired brightness setting on the EV76C541 sensor.
   Parameters  :
                 brightness - Desired brightness level.
 */
extern void
SensorSetBrightness (
        uint8_t input);

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

/* TODO #2-4 Add gain control function definitions */
/* Copy the SensorGetBrightness and SensorSetBrightness function definitions from above
 * and paste them below this comment.
 * Rename the functions to SensorGetGain and SensorSetGain respectively.
 */

/* Function    : SensorGetGain
   Description : Get the current gain setting from the EV76C541 sensor.
   Parameters  : None
 */
extern uint8_t
SensorGetGain (
        void);

/* Function    : SensorSetGain
   Description : Set the desired gain setting on the EV76C541 sensor.
   Parameters  :
                 gain - Desired gain level.
 */
extern void
SensorSetGain (
        uint8_t input);

#endif /* _INCLUDED_SENSOR_H_ */

/*[]*/

