/*****************************************************************************************//**
* # PhilRobotics | Philippine Electronics and Robotics Enthusiasts Club #
* http://philrobotics.com | http://philrobotics.com/forum | http://facebook.com/philrobotics
* phirobotics.core@philrobotics.com
*
*---------------------------------------------------------------------------------------------
* |Filename:      	| "Telemetry"                                                 			|
* |:----          	|:----                                                                  |
* |Description:   	| Demonstrate P-Bot telemetry through UART using D-Protocol				
*						visit "http://www.e-gizmo.com/KIT/P-BOT.htm" for more info.			|
* |Author:        	| Efren S. Cruzat II                                                    |
* |               	|                                                                       |
* |Dependencies: 	|                                                                       |
*
* > This program is free software under GNU General Public License.
*********************************************************************************************/
#define __ARDUINO__
#include "Mobot_Lib.h"

/* Bluetooth Address (Little Endian) */
const uint8_t BT_Addr[6]            = {0x12, 0x13, 0x23, 0x17, 0x08, 0x00};                                             // any 6 bytes hex digits
const uint8_t BT_Alias[15]          = {"P-BOT"};                                                                        // max of 15 characters
const uint8_t BT_SecurityCode[4]    = {'1', '2', '3', '4' };                                                            // 4 digit code (ASCII)

/* Initializes E-Gizmo Bluetooth module to be ready for data transfer, Default data rate is 9600bps */
void initEGizmoBTModule(void) 
{
    /* Bluetooth Serial Initialization Commands */
    uint8_t BT_InitBuff_Cmd[13] = {0x02, 0x52, 0x27, 0x06, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};       // 6-11
    uint8_t BT_SetAlias_Cmd[24] = {0x02, 0x52, 0x04, 0x11, 0x00, 0x67, 0x10, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x03};
    uint8_t BT_SetSecurity_Cmd[12] = {0x02, 0x52, 0x17, 0x05, 0x00, 0x6E, 0x04, 0x31, 0x32, 0x33, 0x34, 0x03};
    uint8_t BT_StartCore_Cmd[7] = {0x02, 0x52, 0x66, 0x00, 0x00, 0xB8, 0x03};  

    uint8_t TempCntr0, TempCntr1;

    setupSerial(9600);                                                  // Set UART Baudrate to 9600bps  
  
    /* Parse Blutetooth Address */
    TempCntr1 = 0;  
    for(TempCntr0 = 6; TempCntr0 < 12; TempCntr0++)
    {
        BT_InitBuff_Cmd[TempCntr0] = BT_Addr[TempCntr1];
        TempCntr1++;    
    }
  
    /* Set Blutetooth Address */
    serialWriteBlock(BT_InitBuff_Cmd,13);
    delayMs(250);

    /* Parse Bluetooth Alias */
    TempCntr1 = 0;  
    for(TempCntr0 = 7; TempCntr0 < 22; TempCntr0++)
    {
        BT_SetAlias_Cmd[TempCntr0] = BT_Alias[TempCntr1];
        TempCntr1++;    
    } 

    /* Set Bluetooth Alias */
    serialWriteBlock(BT_SetAlias_Cmd,24); 
    delayMs(250);

    /* Parse Bluetooth Security Code */
    TempCntr1 = 0;  
    for(TempCntr0 = 7; TempCntr0 < 11; TempCntr0++)
    {
        BT_SetSecurity_Cmd[TempCntr0] = BT_SecurityCode[TempCntr1];
        TempCntr1++;    
    } 
  
    /* Set Bluetooth Security Code */
    serialWriteBlock(BT_SetSecurity_Cmd,12);
    delayMs(250);

    /* Start Bluetooth Core */
    serialWriteBlock(BT_StartCore_Cmd,7);
    delayMs(500);

    serialWrite(' '); 
    serialWrite(0x0D);                             
    serialWrite(0x0A);
}

void init()
{
    initEGizmoBTModule();                                   // send commands to establish contact

    setupPBotUART(REMOTECONTROL,WHITE,BLACK,0,9600);              
                                                            // Remote Control Mode, 
                                                            // Detect White Obstacles, 
                                                            // Detect Black Lines 
                                                            // Telemetry Disabled
                                                            // 9600bps baudrate 
}
 
void cycle()
{
    /* Listen to D Commands */
    parseIncomingBytes();
    detectObstacle();
    detectLine();
    interpretDProtocolCommands();
}                                                     
