/*****************************************************************************************//**
* # PhilRobotics | Philippine Electronics and Robotics Enthusiasts Club #
* http://philrobotics.com | http://philrobotics.com/forum | http://facebook.com/philrobotics
* phirobotics.core@philrobotics.com
*
*---------------------------------------------------------------------------------------------
* |Filename:      | "Telemetry.c"                                                       |
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
* |v00.02.01    |2013xxxx   |ESCII              |PhilRobokit Port                   |
*********************************************************************************************/
#define __SHOW_MODULE_HEADER__ /*!< \brief This section includes the Module Header on the documentation */
#undef  __SHOW_MODULE_HEADER__

#include "Telemetry.h"

static uint32_t  ulPollIntTimer;

// ************************************************************************
// *** Start of Telemetry Class *******************************************
// ************************************************************************

void setupTelemetryID(uint32_t ui32DBaudRate, uint8_t strDProtID[])
{
    uint8_t iCounter;

    for(iCounter = 0; iCounter < 4; iCounter++)
    {
        strDProtocolID[iCounter] = strDProtID[iCounter];
    }

    setupTelemetry(ui32DBaudRate);
}

void setupTelemetry(uint32_t ui32DBaudRate)
{
    setupSerial(ui32DBaudRate);
    ulPollIntTimer = getMs();
}

void pollSensors(uint32_t ui32PollingInterval)
{
    if(getElapsedMs(ulPollIntTimer) >= ui32PollingInterval)
    {
        ulPollIntTimer = getMs();
        pollWallSensorsTelemetry();
        pollLineSensorsTelemetry();
        //   pollTachometers(); esc.inactive
    }
}

// Public Methods
void pollWallSensorsTelemetry(void)
{
    uint8_t ui8ObstacleLocation;
    ui8ObstacleLocation = detectObstacle();
    reportWallSensors(ui8ObstacleLocation);
}

void pollLineSensorsTelemetry(void)
{
    uint8_t ui8LineLocation;
    ui8LineLocation = detectLine();
    reportLineSensors(ui8LineLocation);
}

void reportWallSensors(uint8_t ui8ObstacleLocation)
{
    switch(ui8ObstacleLocation)
    {
        case KI_OBJECT_NEARFRONT:
            printD(strDProtocolID, "rWSR", "NEAR", "FRONT");
            break;

        case KI_OBJECT_LEFTCENTER:
            printD(strDProtocolID, "rWSR", "LEFT", "CENTER");
            break;

        case KI_OBJECT_RIGHTCENTER:
            printD(strDProtocolID, "rWSR", "RIGHT", "CENTER");
            break;

        case KI_OBJECT_BOTHSIDES:
            printD(strDProtocolID, "rWSR", "BOTH", "SIDES");
            break;

        case KI_OBJECT_FARFRONT:
            printD(strDProtocolID, "rWSR", "FAR", "FRONT");
            break;

        case KI_OBJECT_LEFT:
            printD(strDProtocolID, "rWSR", "LEFT", "NULL");
            break;

        case KI_OBJECT_RIGHT:
            printD(strDProtocolID, "rWSR", "RIGHT", "NULL");
            break;

        default:
            printD(strDProtocolID, "rWSR", "NONE", "NULL");
            break;
    }
}

void reportLineSensors(uint8_t ui8LineLocation)
{
    switch(ui8LineLocation)
    {
        case KI_OVER_LINE:
            printD(strDProtocolID, "rLSR", "OVER", "LINE");
            break;

        case KI_LINE_LEFTCENTER:
            printD(strDProtocolID, "rLSR", "LEFT", "CENTER");
            break;

        case KI_LINE_RIGHTCENTER:
            printD(strDProtocolID, "rLSR", "RIGHT", "CENTER");
            break;

        case KI_LINE_LEFT:
            printD(strDProtocolID, "rLSR", "LEFT", "NULL");
            break;

        case KI_LINE_RIGHT:
            printD(strDProtocolID, "rLSR", "RIGHT", "NULL");
            break;

        case KI_LINE_CENTER:
            printD(strDProtocolID, "rLSR", "CENTER", "NULL");
            break;

        default:
            printD(strDProtocolID, "rLSR", "NONE", "NULL");
            break;
    }
}

/* esc.inactive
void pollTachometers(void)
{

}
*/

void printD(uint8_t strDHeader[4], uint8_t strDCommandCode[5], uint8_t strDParam1[7], uint8_t strDParam2[7])
{
    serialWriteString(strDHeader);                          // print D-Protocol Header
    serialWriteString(" ");
    serialWriteString(strDCommandCode);                     // prints Command Code
    serialWriteString(" ");

    if(strDParam1 != "NULL")
    {
        serialWriteString(strDParam1);                      // prints Parameter1
        serialWriteString(" ");

        if(strDParam2 != "NULL")
        {
            serialWriteString(strDParam2);                  // prints Parameter2
            serialWriteString(" ");
        }
    }

    /* prints D-Protocol Packet Terminator */
    serialWrite(0x0D);
    serialWrite(0x0A);
}

// ************************************************************************
// End of Telemetry.c
// ************************************************************************

