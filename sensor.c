/*
 ## Cypress FX3 Camera Kit source file (sensor.c)
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

/* This file implements the I2C based driver for the EV76C541 image sensor used
   in the FX3 HD 720p camera kit.

   Please refer to the Aptina EV76C541 sensor datasheet for the details of the
   I2C commands used to configure the sensor.
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>
#include "sensor.h"
#include "appi2c.h"

#define CONFIRM_TRIES      5

/* ATTiny20 registers */
#define REG_ATTINY_INIT    0xFF

/* E2V registers */
#define SOFT_RESET         0x01
#define ABORT_MBX          0x03
#define REG_CTRL_CFG       0x0B // Has restricted registers!
#define REG_T_FRAME_PERIOD 0x0C
#define ROI1_T_INT_LL      0x0E
#define ROI1_GAIN          0x11
#define FB_STATUS          0x3E
#define CHIP_ID            0x7F

static void Sensor_Configure_EV76C541 (void);
static void SensorStart (void);
static void SensorStop (void);

/*
 * Reset the EV76C541 sensor using GPIO.
 */
void
SensorReset (
        void)
{
    uint8_t buf[1] = { 0x00 };
    CyU3PReturnStatus_t apiRetStatus;

    /* Write to reset register to reset the sensor. */
    apiRetStatus = SensorWrite (SENSOR_ADDR_WR, SOFT_RESET, 1, buf);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "I2C Error, Error Code = %d\n", apiRetStatus);
        return;
    }

    /* Wait for some time to allow proper reset. */
    CyU3PThreadSleep (1);
    return;
}

/* EV76C541 sensor initialization sequence.

   Sensor_Configure_Serdes: configure I2C bridge to CMOS board
   Sensor_Configure_EV76C541: configure EV76C541 on CMOS board

   Select Video Resolution
   SensorChangeConfig     : Update sensor configuration.
*/
void
SensorInit (
        void)
{
    if (SensorI2cBusTest () != CY_U3P_SUCCESS)        /* Verify that the sensor is connected. */
    {
        CyU3PDebugPrint (4, "Error: Reading Sensor ID failed!\r\n");
        return;
    }

    Sensor_Configure_EV76C541 (); /* Configure EV76C541 */

    /* Update sensor configuration based on desired video stream parameters.
     * Using 752x480 at 30fps as default setting.
     */
    SensorScaling_752_480_30fps ();

    SensorStart ();
}

static void
SensorStart (
             void)
{
    // Set control configuration to start running
    // roi_video_en, roi_overlap_en, trig_rqst
    SensorWriteConfirm2B (SENSOR_ADDR_WR, REG_CTRL_CFG, 0x00, 0x0E, CONFIRM_TRIES);
}

static void
SensorStop (
            void)
{
    uint8_t buf[2];
    SensorRead2B (SENSOR_ADDR_RD, REG_CTRL_CFG, buf);
    SensorWrite2B (SENSOR_ADDR_WR, REG_CTRL_CFG, buf[0], buf[1] | 0x01);
    SensorWrite (SENSOR_ADDR_WR, ABORT_MBX, 1, buf[0]); // Any write will trigger an abort
}

/*
 * Verify that the sensor can be accessed over the I2C bus from FX3.
 */
uint8_t
SensorI2cBusTest (
        void)
{
    /* The sensor ID register can be read here to verify sensor connectivity. */
    uint8_t buf[2];

    /* Reading sensor ID */
    if (SensorRead2B (SENSOR_ADDR_RD, CHIP_ID, buf) == CY_U3P_SUCCESS)
    {
        if ((buf[0] == 0x0A) && (buf[1] == 0x00))
        {
            return CY_U3P_SUCCESS;
        }
    }
    return 1;
}

void
SensorConfigureSerdes (void)
{
	uint8_t buf[2];

	// Configure I2C passthrough for imaging sensor
	buf[0] = SENSOR_ADDR_WR;
	SensorWrite (DESER_ADDR_WR, 0x08, 1, buf);
	SensorWrite (DESER_ADDR_WR, 0x10, 1, buf);

	// Configure I2C passthrough for LED
	buf[0] = LED_ADDR_WR;
	SensorWrite (DESER_ADDR_WR, 0x09, 1, buf);
	SensorWrite (DESER_ADDR_WR, 0x11, 1, buf);

	// Configure I2C passthrough for Serializer
	buf[0] = SER_ADDR_WR;
	SensorWrite (DESER_ADDR_WR, 0x07, 1, buf); // Write to SER alias register

	// Disable GPIO 0 and 1 on deserializer
	buf[0] = 0x22; // Disable GPIO 1 and 2
	SensorWrite (DESER_ADDR_WR, 0x1D, 1, buf);

	// Configure VDDIO voltage on serializer, possibly unnecessary
	buf[0] = 0x20; // defaults except for VDDIO mode which is now 1.8V
	SensorWrite (SER_ADDR_WR, 0x01, 1, buf);

	// Configure GPO 0 and 1 on serializer to bring GPO0 high to turn on imaging sensor
	buf[0] = 0x19; // GPO1 enabled with low output, GPO0 enabled with high output
	SensorWrite (SER_ADDR_WR, 0x0D, 1, buf);
}

/*
   Initialize the EV76C541, leaning on the attiny20 heavily
   for initialization. See the mscp_code repository for details of initialization.
   Also refer to the EV76C541 sensor datasheet for details.
 */
static void
Sensor_Configure_EV76C541( void ) //SPI configuration of sensor
{
    // Trigger attiny20 initialization of E2V... this will reset the E2V
    SensorWriteNoReg (SENSOR_ADDR_WR, REG_ATTINY_INIT);
}

/*
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the EV76C541 sensor datasheet for details.
 */
void
SensorScaling_752_480_30fps (
        void)
{
    /*
       Video configuration
	   0x086F frame period is d2159 lines,
	   with a 15.4us line period = 0.0333s frame period (30 fps)
	 */
	SensorWriteConfirm2B (SENSOR_ADDR_WR, REG_T_FRAME_PERIOD, 0x08, 0x6F, CONFIRM_TRIES);

	/*
	   ROI1 int time
	   0x086E = d2158 Int time in number of lines. time for 1 line = 15.4us*57MHz/(CLK_CTRL in MHz)
	   2158 lines at 57MHz clock should get us 30FPS
	 */
	SensorWriteConfirm2B (SENSOR_ADDR_WR, ROI1_T_INT_LL, 0x08, 0x6E, CONFIRM_TRIES);
}

/*
   Get the current brightness setting from the EV76C541 sensor.
 */
uint8_t
SensorGetBrightness ( // FIXME
        void)
{
    return 0x00;
}

/*
   Update the brightness setting for the EV76C541 sensor.
 */
void
SensorSetBrightness ( // FIXME
        uint8_t input)
{
}

/*
   Get the current gain setting from the EV76C541 sensor.
 */
uint8_t
SensorGetGain (
        void)
{
    uint8_t buf[1];

    SensorRead (SENSOR_ADDR_RD, ROI1_GAIN, 1, buf); // Only read first byte
    return (uint8_t)buf[0];
}

/*
   Update the gain setting for the EV76C541 sensor.
 */
void
SensorSetGain (
        uint8_t input)
{
    SensorWrite (SENSOR_ADDR_WR, ROI1_GAIN, 1, &input); // Only write first byte
}
