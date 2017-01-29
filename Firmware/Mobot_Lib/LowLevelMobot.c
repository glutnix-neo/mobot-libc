/*****************************************************************************************//**
* # PhilRobotics | Philippine Electronics and Robotics Enthusiasts Club #
* http://philrobotics.com | http://philrobotics.com/forum | http://facebook.com/philrobotics
* phirobotics.core@philrobotics.com
*
*---------------------------------------------------------------------------------------------
* |Filename:      | "LowLevelMobot.c"                                                   |
* |:----          |:----                                                                |
* |Description:   | Main Library for differential wheel driven Mobile Robots (Mobots)   |
* |Revision:      | v00.02.01                                                           |
* |Author:        | Efren S. Cruzat II                                                  |
* |               |                                                                     |
* |Dependencies:  |                                                                     |
*
* > This program is free software: you can redistribute it and/or modify
* > it under the terms of the GNU General Public License as published by
* > the Free Software Foundation, either version 3 of the License, or
* > (at your option) any later version.
* > This program is distributed in the hope that it will be useful,
* > but WITHOUT ANY WARRANTY; without even the implied warranty of
* > MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* > GNU General Public License for more details.
* > <BR><BR>
* > You should have received a copy of the GNU General Public License
* > along with this program. If not, see http://www.gnu.org/licenses/
* <BR>
*---------------------------------------------------------------------------------------------
* |FW Version   |Date       |Author             |Description                        |
* |:----        |:----      |:----              |:----                              |
* |v00.01.00    |20101216   |ESCII              |Library Initial Release            |
* |v00.01.01    |20120118   |ESCII              |Last known working port(Pinguino)  |
* |v00.02.01    |2013xxxx   |ESCII              |PhilRobokit Port                   |
*********************************************************************************************/
#define __SHOW_MODULE_HEADER__ /*!< \brief This section includes the Module Header on the documentation */
#undef  __SHOW_MODULE_HEADER__

#if (defined(__PINGUINO__))
#include <pin.h>
#include <system.c>
#include <digitalw.c>

#include <mathlib.c>
#include <delay.c>
#include <millis.c>
#endif

#include "LowLevelMobot.h"

// *** private variables **********************************************

// *** private functions **********************************************
static uint8_t readSensorOutput(uint8_t iSensorPin, bool_t blPolarity);

// *** Low Level Motor Control Functions **********************************
void runRightMotor(uint8_t ui8Speed, bool_t blDirection)
{
    if(KBL_REVERSE_DIR == blDirection)
    {
        setPin(KI_RIGHTMOTOR_DIRCTRL);
    }
    else
    {
        clrPin(KI_RIGHTMOTOR_DIRCTRL);
    }

#if (MOTOR_TYPE == USE_DC_FIX_SPEED)
	#if (DRIVER_TYPE == DRIVER_HBRIDGE)
    if(KBL_REVERSE_DIR == blDirection)
    {
        clrPin(KI_RIGHTMOTOR_RUNCTRL);
    }
    else
    {
        setPin(KI_RIGHTMOTOR_RUNCTRL);
    }	
	#else
    setPin(KI_RIGHTMOTOR_RUNCTRL);
	#endif
#elif (MOTOR_TYPE == USE_DC_VAR_SPEED)
	#if (DRIVER_TYPE == DRIVER_HBRIDGE)
    if(KBL_REVERSE_DIR == blDirection)
    {
        setSoftDAC(RIGHT_MOTOR, (KB_MAXSPEED-ui8Speed));
    }
    else
    {
        setSoftDAC(RIGHT_MOTOR, ui8Speed);	
    }	
	#else
    setSoftDAC(RIGHT_MOTOR, ui8Speed);
	#endif
#elif (MOTOR_TYPE == USE_SERVO)
#else
#error undefined MOTOR_TYPE
#endif
}

void runLeftMotor(uint8_t ui8Speed, bool_t blDirection)
{
    if(KBL_REVERSE_DIR == blDirection)
    {
        setPin(KI_LEFTMOTOR_DIRCTRL);
    }
    else
    {
        clrPin(KI_LEFTMOTOR_DIRCTRL);
    }

#if (MOTOR_TYPE == USE_DC_FIX_SPEED)
	#if (DRIVER_TYPE == DRIVER_HBRIDGE)
    if(KBL_REVERSE_DIR == blDirection)
    {
        clrPin(KI_LEFTMOTOR_RUNCTRL);
    }
    else
    {
        setPin(KI_LEFTMOTOR_RUNCTRL);
    }	
	#else
    setPin(KI_LEFTMOTOR_RUNCTRL);
	#endif
#elif (MOTOR_TYPE == USE_DC_VAR_SPEED)
	#if (DRIVER_TYPE == DRIVER_HBRIDGE)
    if(KBL_REVERSE_DIR == blDirection)
    {
        setSoftDAC(LEFT_MOTOR, (KB_MAXSPEED-ui8Speed));
    }
    else
    {
        setSoftDAC(LEFT_MOTOR, ui8Speed);	
    }	
	#else
    setSoftDAC(LEFT_MOTOR, ui8Speed);
	#endif
#elif (MOTOR_TYPE == USE_SERVO)
#else
#error undefined MOTOR_TYPE
#endif
}

void stopRightMotor(void)
{
    clrPin(KI_RIGHTMOTOR_DIRCTRL);
#if (MOTOR_TYPE == USE_DC_FIX_SPEED)
    clrPin(KI_RIGHTMOTOR_RUNCTRL);
#elif (MOTOR_TYPE == USE_DC_VAR_SPEED)
    setSoftDAC(RIGHT_MOTOR, 0);
#elif (MOTOR_TYPE == USE_SERVO)
#else
#error undefined MOTOR_TYPE
#endif
}

void stopLeftMotor(void)
{
    clrPin(KI_LEFTMOTOR_DIRCTRL);
#if (MOTOR_TYPE == USE_DC_FIX_SPEED)
    clrPin(KI_LEFTMOTOR_RUNCTRL);
#elif (MOTOR_TYPE == USE_DC_VAR_SPEED)
    setSoftDAC(LEFT_MOTOR, 0);
#elif (MOTOR_TYPE == USE_SERVO)
#else
#error undefined MOTOR_TYPE
#endif
}

// *** Low Level Sensor Functions *****************************************
static uint8_t readSensorOutput(uint8_t iSensorPin, bool_t blPolarity)
{
    if(blPolarity)
    {
        return !getPinState(iSensorPin);                                    // active low
    }
    else
    {
        return getPinState(iSensorPin);                                     // active high
    }
}
#if 0
uint32_t getRightTachoPeriod(bool_t blActiveEdge)                           // 1 - rising edge, 0 - falling edge
{
    static  bool_t      blPrevTachoState;
    bool_t      blTachoState;
    bool_t      blEdgeDetected;
    static  uint32_t    ui32PrevEdgeMarkTimer;
    uint32_t    ui32EdgeMarkTimer;                                  // will be initialized and used only when edge is detected
    static  uint32_t    ui32PollTimeoutTimer;
    static  uint32_t    ui32TachoPeriod;
    blEdgeDetected = FALSE;
    // poll tacho pin
    blTachoState = readSensorOutput(KI_RIGHTWHEEL_TACHO, !blActiveEdge);

    // detect edge
    if(!blPrevTachoState && blTachoState)
    {
        blEdgeDetected = TRUE;
        ui32EdgeMarkTimer = getMs();
        ui32PollTimeoutTimer = ui32EdgeMarkTimer;
    }

    // check timeout
    if(!blEdgeDetected)
    {
        if(getElapsedMs(ui32PollTimeoutTimer) >= KUL_TACHOPOLLTIMEOUT)
        {
            blEdgeDetected = FALSE;
            ui32TachoPeriod = 0;
        }
    }
    // compute tacho period
    else
    {
        blEdgeDetected = FALSE;
        // IIR 1/4 new, 3/4 old
        ui32TachoPeriod >>= 2;
        ui32TachoPeriod *= 3;
        ui32TachoPeriod += ((ui32EdgeMarkTimer - ui32PrevEdgeMarkTimer) >> 2);
        ui32PrevEdgeMarkTimer = ui32EdgeMarkTimer;
    }

    blPrevTachoState = blTachoState;
    return ui32TachoPeriod;
}

uint32_t getLeftTachoPeriod(bool_t blActiveEdge)                                 // 1 - rising edge, 0 - falling edge
{
    static  bool_t blPrevTachoState;
    bool_t blTachoState;
    bool_t blEdgeDetected;
    static  uint32_t ui32PrevEdgeMarkTimer;
    uint32_t ui32EdgeMarkTimer;                                  // will be initialized and used only when edge is detected
    static  uint32_t ui32PollTimeoutTimer;
    static  uint32_t ui32TachoPeriod;
    blEdgeDetected = FALSE;
    // poll tacho pin
    blTachoState = readSensorOutput(KI_LEFTWHEEL_TACHO, !blActiveEdge);

    // detect edge
    if(!blPrevTachoState && blTachoState)
    {
        blEdgeDetected = TRUE;
        ui32EdgeMarkTimer = getMs();
        ui32PollTimeoutTimer = ui32EdgeMarkTimer;
    }

    // check timeoout
    if(!blEdgeDetected)
    {
        if(getElapsedMs(ui32PollTimeoutTimer) >= KUL_TACHOPOLLTIMEOUT)
        {
            blEdgeDetected = FALSE;
            ui32TachoPeriod = 0;
        }
    }
    // compute tacho period
    else
    {
        blEdgeDetected = FALSE;
        // IIR 1/4 new, 3/4 old
        ui32TachoPeriod >>= 2;
        ui32TachoPeriod *= 3;
        ui32TachoPeriod += ((ui32EdgeMarkTimer - ui32PrevEdgeMarkTimer) >> 2);
        ui32PrevEdgeMarkTimer = ui32EdgeMarkTimer;
    }

    blPrevTachoState = blTachoState;
    return ui32TachoPeriod;
}
#endif

void pollWallSensors(bool_t* blObstacleSensorPolarity)
{
    blWallDetectedonLeft = readSensorOutput(KI_LEFT_WALLSENSOR, *blObstacleSensorPolarity);
    blWallDetectedonCenter = readSensorOutput(KI_CENTER_WALLSENSOR, *blObstacleSensorPolarity);
    blWallDetectedonRight = readSensorOutput(KI_RIGHT_WALLSENSOR, *blObstacleSensorPolarity);
}

void pollLineSensors(bool_t* blLineSensorPolarity)
{
    blLineDetectedonLeft = readSensorOutput(KI_LEFT_LINESENSOR, *blLineSensorPolarity);
    blLineDetectedonCenter = readSensorOutput(KI_CENTER_LINESENSOR, *blLineSensorPolarity);
    blLineDetectedonRight = readSensorOutput(KI_RIGHT_LINESENSOR, *blLineSensorPolarity);
}
// *********************************************************************
// End of LowLevelMobot.c
// *********************************************************************
