/***************************************************************************//**
*   @file   ADXL345.c
*   @brief  Implementation of ADXL345 Driver.
*   @author DBogdan (dragos.bogdan@analog.com)
*			 Ported to Arduino by Dan Nechita
********************************************************************************
* Copyright 2012(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
********************************************************************************
*   SVN Revision: 699
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ADXL345.h"
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
#define HIGH 1
#define LOW 0

/**************************************************************************//**
* @brief Writes data into a register.
*
* @param registerAddress - Address of the register.
* @param registerValue - Data value to write.
*
* @return None.
******************************************************************************/
void ADXL345::SetRegisterValue(unsigned char registerAddress,
unsigned char registerValue)
{
chip_select(LOW);
write(registerAddress);
write(registerValue);
chip_select(HIGH);
}

/***************************************************************************//**
* @brief Reads the value of a register.
*
* @param registerAddress - Address of the register.
*
* @return registerValue - Value of the register.
*******************************************************************************/
unsigned char ADXL345::GetRegisterValue(unsigned char registerAddress){
chip_select(LOW);
write(0x80 + registerAddress);
write(0);
uint8_t data0 = read();
uint8_t data1 = read();
chip_select(HIGH);
return(data1);
}


/***************************************************************************//**
* @brief Set the accelerometer's rage.
*
* @param range - Accelerometer's range.
*                Example: ADXL345_RANGE_PM_2G  -> +- 2g
*                         ADXL345_RANGE_PM_4G  -> +- 4g
*                         ADXL345_RANGE_PM_8G  -> +- 8g
*                         ADXL345_RANGE_PM_16G -> +- 16g
*
* @return None.
*******************************************************************************/
void ADXL345::SetRange(char range)
{
SetRegisterValue(ADXL345_DATA_FORMAT, ADXL345_RANGE(range));
mRange = range;
}

/***************************************************************************//**
* @brief Places the device into standby/measure mode.
*
* @param pwrMode - Power mode.
*                    Example: 0x0 - standby mode.
*                             0x1 - measure mode.
*
* @return None.
*******************************************************************************/
void ADXL345::SetPowerMode(unsigned char pwrMode)

{
unsigned char oldPowerCtl = 0;
unsigned char newPowerCtl = 0;

oldPowerCtl = GetRegisterValue(ADXL345_POWER_CTL);
newPowerCtl = oldPowerCtl & ~ADXL345_PCTL_MEASURE;
newPowerCtl = newPowerCtl | (pwrMode * ADXL345_PCTL_MEASURE);
SetRegisterValue(ADXL345_POWER_CTL, newPowerCtl);
}

/***************************************************************************//**
* @brief Reads the output data of each axis.
*
* @param x - X-axis's output data.
* @param y - Y-axis's output data.
* @param z - Z-axis's output data.
*
* @return None.
*******************************************************************************/
void ADXL345::GetXyz(short* x,
short* y,
short* z)
{
*x  = GetRegisterValue(ADXL345_DATAX1) << 8;
*x += GetRegisterValue(ADXL345_DATAX0);
*y  = GetRegisterValue(ADXL345_DATAY1) << 8;
*y += GetRegisterValue(ADXL345_DATAY0);
*z  = GetRegisterValue(ADXL345_DATAZ1) << 8;
*z += GetRegisterValue(ADXL345_DATAZ0);
}

/***************************************************************************//**
* @brief Reads the output data of each axis. Data is converted in g format.
*
* @param x - X-axis's output data.
* @param y - Y-axis's output data.
* @param z - Z-axis's output data.
*
* @return None.
*******************************************************************************/
void ADXL345::ReadAccel(double* x, double* y, double* z)
{
short xAxis = 0;
short yAxis = 0;
short zAxis = 0;

GetXyz(&xAxis, &yAxis, &zAxis);
*x = (xAxis * 3.9 * (1 << mRange)) / 1000;
*y = (yAxis * 3.9 * (1 << mRange)) / 1000;
*z = (zAxis * 3.9 * (1 << mRange)) / 1000;
}

/***************************************************************************//**
* @brief Enables/disables the tap detection.
*
* @param tapType - Tap type (none, single, double).
*                    Example: 0x0 - disables tap detection.
*                             ADXL345_SINGLE_TAP - enables single tap detection.
*                             ADXL345_DOUBLE_TAP - enables double tap detection.
* @param tapAxes - Axes which participate in tap detection.
*                    Example: 0x0 - disables axes participation.
*                             ADXL345_TAP_X_EN - enables x-axis participation.
*                             ADXL345_TAP_Y_EN - enables y-axis participation.
*                             ADXL345_TAP_Z_EN - enables z-axis participation.
* @param tapDur - Tap duration.
* @param tapLatent - Tap latency.
* @param tapWindow - Tap window.
* @param tapThresh - Tap threshold.
* @param tapInt - Interrupts pin.
*                   Example: 0x0 - interrupts on INT1 pin.
*                            ADXL345_SINGLE_TAP - single tap interrupts on
*                                                 INT2 pin.
*                            ADXL345_DOUBLE_TAP - double tap interrupts on
*                                                 INT2 pin.
*
* @return None.
*******************************************************************************/
void ADXL345::SetTapDetection(unsigned char tapType,
unsigned char tapAxes,
unsigned char tapDur,
unsigned char tapLatent,
unsigned char tapWindow,
unsigned char tapThresh,
unsigned char tapInt)
{
unsigned char oldTapAxes    = 0;
unsigned char newTapAxes    = 0;
unsigned char oldIntMap     = 0;
unsigned char newIntMap     = 0;
unsigned char oldIntEnable  = 0;
unsigned char newIntEnable  = 0;

oldTapAxes = GetRegisterValue(ADXL345_TAP_AXES);
newTapAxes = oldTapAxes & ~(ADXL345_TAP_X_EN |
ADXL345_TAP_Y_EN |
ADXL345_TAP_Z_EN);
newTapAxes = newTapAxes | tapAxes;
SetRegisterValue(ADXL345_TAP_AXES, newTapAxes);
SetRegisterValue(ADXL345_DUR, tapDur);
SetRegisterValue(ADXL345_LATENT, tapLatent);
SetRegisterValue(ADXL345_WINDOW, tapWindow);
SetRegisterValue(ADXL345_THRESH_TAP, tapThresh);
oldIntMap = GetRegisterValue(ADXL345_INT_MAP);
newIntMap = oldIntMap & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
newIntMap = newIntMap | tapInt;
SetRegisterValue(ADXL345_INT_MAP, newIntMap);
oldIntEnable = GetRegisterValue(ADXL345_INT_ENABLE);
newIntEnable = oldIntEnable & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
newIntEnable = newIntEnable | tapType;
SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
* @brief Enables/disables the activity detection.
*
* @param actOnOff - Enables/disables the activity detection.
*                     Example: 0x0 - disables the activity detection.
*                              0x1 - enables the activity detection.
* @param actAxes - Axes which participate in detecting activity.
*                    Example: 0x0 - disables axes participation.
*                             ADXL345_ACT_X_EN - enables x-axis participation.
*                             ADXL345_ACT_Y_EN - enables y-axis participation.
*                             ADXL345_ACT_Z_EN - enables z-axis participation.
* @param actAcDc - Selects dc-coupled or ac-coupled operation.
*                    Example: 0x0 - dc-coupled operation.
*                             ADXL345_ACT_ACDC - ac-coupled operation.
* @param actThresh - Threshold value for detecting activity.
* @patam actInt - Interrupts pin.
*                   Example: 0x0 - activity interrupts on INT1 pin.
*                            ADXL345_ACTIVITY - activity interrupts on INT2 pin.
*
* @return None.
*******************************************************************************/
void ADXL345::SetActivityDetection(unsigned char actOnOff,
unsigned char actAxes,
unsigned char actAcDc,
unsigned char actThresh,
unsigned char actInt)
{
unsigned char oldActInactCtl    = 0;
unsigned char newActInactCtl    = 0;
unsigned char oldIntMap         = 0;
unsigned char newIntMap         = 0;
unsigned char oldIntEnable      = 0;
unsigned char newIntEnable      = 0;

oldActInactCtl = GetRegisterValue(ADXL345_INT_ENABLE);
newActInactCtl = oldActInactCtl & ~(ADXL345_ACT_ACDC |
ADXL345_ACT_X_EN |
ADXL345_ACT_Y_EN |
ADXL345_ACT_Z_EN);
newActInactCtl = newActInactCtl | (actAcDc | actAxes);
SetRegisterValue(ADXL345_ACT_INACT_CTL, newActInactCtl);
SetRegisterValue(ADXL345_THRESH_ACT, actThresh);
oldIntMap = GetRegisterValue(ADXL345_INT_MAP);
newIntMap = oldIntMap & ~(ADXL345_ACTIVITY);
newIntMap = newIntMap | actInt;
SetRegisterValue(ADXL345_INT_MAP, newIntMap);
oldIntEnable = GetRegisterValue(ADXL345_INT_ENABLE);
newIntEnable = oldIntEnable & ~(ADXL345_ACTIVITY);
newIntEnable = newIntEnable | (ADXL345_ACTIVITY * actOnOff);
SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
* @brief Enables/disables the inactivity detection.
*
* @param inactOnOff - Enables/disables the inactivity detection.
*                       Example: 0x0 - disables the inactivity detection.
*                                0x1 - enables the inactivity detection.
* @param inactAxes - Axes which participate in detecting inactivity.
*                      Example: 0x0 - disables axes participation.
*                               ADXL345_INACT_X_EN - enables x-axis.
*                               ADXL345_INACT_Y_EN - enables y-axis.
*                               ADXL345_INACT_Z_EN - enables z-axis.
* @param inactAcDc - Selects dc-coupled or ac-coupled operation.
*                      Example: 0x0 - dc-coupled operation.
*                               ADXL345_INACT_ACDC - ac-coupled operation.
* @param inactThresh - Threshold value for detecting inactivity.
* @param inactTime - Inactivity time.
* @patam inactInt - Interrupts pin.
*                     Example: 0x0 - inactivity interrupts on INT1 pin.
*                              ADXL345_INACTIVITY - inactivity interrupts on
*                                                   INT2 pin.
*
* @return None.
*******************************************************************************/
void ADXL345::SetInactivityDetection(unsigned char inactOnOff,
unsigned char inactAxes,
unsigned char inactAcDc,
unsigned char inactThresh,
unsigned char inactTime,
unsigned char inactInt)
{
unsigned char oldActInactCtl    = 0;
unsigned char newActInactCtl    = 0;
unsigned char oldIntMap         = 0;
unsigned char newIntMap         = 0;
unsigned char oldIntEnable      = 0;
unsigned char newIntEnable      = 0;

oldActInactCtl = GetRegisterValue(ADXL345_INT_ENABLE);
newActInactCtl = oldActInactCtl & ~(ADXL345_INACT_ACDC |
ADXL345_INACT_X_EN |
ADXL345_INACT_Y_EN |
ADXL345_INACT_Z_EN);
newActInactCtl = newActInactCtl | (inactAcDc | inactAxes);
SetRegisterValue(ADXL345_ACT_INACT_CTL, newActInactCtl);
SetRegisterValue(ADXL345_THRESH_INACT, inactThresh);
SetRegisterValue(ADXL345_TIME_INACT, inactTime);
oldIntMap = GetRegisterValue(ADXL345_INT_MAP);
newIntMap = oldIntMap & ~(ADXL345_INACTIVITY);
newIntMap = newIntMap | inactInt;
SetRegisterValue(ADXL345_INT_MAP, newIntMap);
oldIntEnable = GetRegisterValue(ADXL345_INT_ENABLE);
newIntEnable = oldIntEnable & ~(ADXL345_INACTIVITY);
newIntEnable = newIntEnable | (ADXL345_INACTIVITY * inactOnOff);
SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
* @brief Enables/disables the free-fall detection.
*
* @param ffOnOff - Enables/disables the free-fall detection.
*                    Example: 0x0 - disables the free-fall detection.
*                             0x1 - enables the free-fall detection.
* @param ffThresh - Threshold value for free-fall detection.
* @param ffTime - Time value for free-fall detection.
* @param ffInt - Interrupts pin.
*                  Example: 0x0 - free-fall interrupts on INT1 pin.
*                           ADXL345_FREE_FALL - free-fall interrupts on INT2 pin.
*
* @return None.
*******************************************************************************/
void ADXL345::SetFreeFallDetection(unsigned char ffOnOff,
unsigned char ffThresh,
unsigned char ffTime,
unsigned char ffInt)
{
unsigned char oldIntMap     = 0;
unsigned char newIntMap     = 0;
unsigned char oldIntEnable  = 0;
unsigned char newIntEnable  = 0;

SetRegisterValue(ADXL345_THRESH_FF, ffThresh);
SetRegisterValue(ADXL345_TIME_FF, ffTime);
oldIntMap = GetRegisterValue(ADXL345_INT_MAP);
newIntMap = oldIntMap & ~(ADXL345_FREE_FALL);
newIntMap = newIntMap | ffInt;
SetRegisterValue(ADXL345_INT_MAP, newIntMap);
oldIntEnable = GetRegisterValue(ADXL345_INT_ENABLE);
newIntEnable = oldIntEnable & ~ADXL345_FREE_FALL;
newIntEnable = newIntEnable | (ADXL345_FREE_FALL * ffOnOff);
SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
* @brief Calibrates the accelerometer.
*
* @param xOffset - X-axis's offset.
* @param yOffset - Y-axis's offset.
* @param zOffset - Z-axis's offset.
*
* @return None.
*******************************************************************************/
void ADXL345::SetOffset(unsigned char xOffset,
unsigned char yOffset,
unsigned char zOffset)
{
SetRegisterValue(ADXL345_OFSX, xOffset);
SetRegisterValue(ADXL345_OFSY, yOffset);
SetRegisterValue(ADXL345_OFSZ, yOffset);
}

