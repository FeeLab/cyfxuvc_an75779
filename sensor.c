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
#define REG_FB_STATUS	   0x3E

/* Global variables */
static uint16_t exposureMaxLL = 0x0000; // Max exposure for this frame rate in number of lines

static void Sensor_Configure_EV76C541 (void);

static void
FillBuff2B (
		uint16_t input,
		uint8_t *buf)
{
	buf[0] = (uint8_t) ((input & 0xFF00) >> 8);
	buf[1] = (uint8_t) (input & 0x00FF);
}

static uint16_t
Combine2B (
		uint8_t *buf)
{
	return ((uint16_t) buf[0] << 8) | buf[1];
}

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
    apiRetStatus = I2CWrite (SENSOR_ADDR_WR, SOFT_RESET, 1, buf);
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

void
SensorStart (
             void)
{
    // Set control configuration to start running
    // roi_video_en, roi_overlap_en, trig_rqst
	I2CWriteConfirm2B (SENSOR_ADDR_WR, REG_CTRL_CFG, 0x00, 0x0E, CONFIRM_TRIES);
}

void
SensorStop (
            void)
{
    uint8_t buf[2];
    I2CRead2B (SENSOR_ADDR_RD, REG_CTRL_CFG, buf);
    I2CWrite2B (SENSOR_ADDR_WR, REG_CTRL_CFG, buf[0], buf[1] | 0x01);
    I2CWrite (SENSOR_ADDR_WR, ABORT_MBX, 1, buf); // Any write will trigger an abort
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
    if (I2CRead2B (SENSOR_ADDR_RD, CHIP_ID, buf) == CY_U3P_SUCCESS)
    {
        if ((buf[0] == 0x0A) && (buf[1] == 0x00))
        {
            return CY_U3P_SUCCESS;
        }
    }
    return 1;
}

/*
   Initialize the EV76C541, leaning on the attiny20 heavily
   for initialization. See the mscp_code repository for details of initialization.
   Also refer to the EV76C541 sensor datasheet for details.
 */
static void
Sensor_Configure_EV76C541 (
		void) //SPI configuration of sensor
{
    // Trigger attiny20 initialization of E2V... this will reset the E2V
	I2CWriteNoReg (SENSOR_ADDR_WR, REG_ATTINY_INIT);
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
       PLL is configured to run at 114.4 MHz
       CLK_CTRL is PLL/2 = 57.2 MHz
       Line length is h6E * 8 * period(CLK_CTRL) = 15.38 us
       To achieve ~30fps therefore requires d2166 (h0876) lines
	 */
	I2CWrite2B (SENSOR_ADDR_WR, REG_T_FRAME_PERIOD, 0x08, 0x76);

	/*
	   ROI1 int time
	   Should be one less than reg_t_frame
	   h0875 = d2165 Int time in number of lines.
	   2165 lines at 57.2 MHz clock should get us 30FPS
	 */
	I2CWrite2B (SENSOR_ADDR_WR, ROI1_T_INT_LL, 0x08, 0x75);
	exposureMaxLL = 0x0875;
}

uint16_t
SensorGetFeedback (
		void)
{
	uint8_t buf[2];
	I2CRead2B (SENSOR_ADDR_RD, REG_FB_STATUS, buf);
	return Combine2B(buf);
}

/*
   Get the current brightness setting from the EV76C541 sensor.
 */
uint8_t
SensorGetBrightness ( // FIXME
        void)
{
	uint32_t temp;
	uint8_t buf[2];
	I2CRead2B (SENSOR_ADDR_WR, ROI1_T_INT_LL, buf);
	temp = 255 * Combine2B (buf);
    return temp / exposureMaxLL;
}

/*
   Update the brightness setting for the EV76C541 sensor.
 */
void
SensorSetBrightness (
        uint8_t input)
{
	uint32_t temp;
	uint16_t exposureCurLL;
	uint8_t buf[2];

	temp = input * exposureMaxLL; // expand into 32 bit to avoid overflow
	exposureCurLL = temp / 255; // integer division
	FillBuff2B (exposureCurLL, buf);
	I2CWrite (SENSOR_ADDR_WR, ROI1_T_INT_LL, 2, buf);
}

/*
   Get the current gain setting from the EV76C541 sensor.
 */
uint8_t
SensorGetGain (
        void)
{
    uint8_t buf[1];

    I2CRead (SENSOR_ADDR_RD, ROI1_GAIN, 1, buf); // Only read first byte
    return (uint8_t) buf[0];
}

/*
   Update the gain setting for the EV76C541 sensor.
 */
void
SensorSetGain (
        uint8_t input)
{
    I2CWrite (SENSOR_ADDR_WR, ROI1_GAIN, 1, &input); // Only write first byte
}
