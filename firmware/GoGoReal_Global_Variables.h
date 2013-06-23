/*
* Copyright (C) 2010-2012 Lucas Anibal Tanure Alves - ME
* Contact   Lucas Tanure [ltanure@gogoreal.com.br]
*   
* This file is part of GoGo Real.
*
* GoGo Real is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GoGo Real is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GoGo Real.  If not, see <http://www.gnu.org/licenses/>.
*/
int16  MotorENPins [MotorCount]={  MTR1_EN, MTR2_EN, MTR3_EN, MTR4_EN};
int16  MotorCWPins [MotorCount]={  MTR1_CW, MTR2_CW, MTR3_CW, MTR4_CW};
int16  MotorCCPins [MotorCount]={  MTR1_CC, MTR2_CC, MTR3_CC, MTR4_CC};
unsigned int CMD_STATE;
int gbl_cur_cmd, gbl_cur_param, gbl_cur_ext, gbl_cur_ext_byte;
int gblBurstModeBits;
int gblBurstModeCounter=0;
int1 gblSlowBurstMode=0;
int1 gblSlowBurstModeTimerHasTicked=0;
int gblCurSensorChannel;
int gblMotorMode=0b00000000;
int gblActiveMotors;
int gblMotorDir=0;
int gblMotorONOFF = 0;
int gblMtrDuty[MotorCount+1] = {0xff,0xff,0xff,0xff,0xff};
unsigned int gblTimer0Counter = MotorCount;
unsigned int gblDutyCycleFlag = 0;
unsigned int gblCurrentDutyIndex = 0;
unsigned char gblMostRecentlyReceivedByte;
int1 gblNewByteHasArrivedFlag = 0;
int1 gblLogoIsRunning = 0;
int1 gblButtonPressed = 0;
int1 gblBtn1AlreadyPressed = 0;
unsigned int16 gblWaitCounter =0;
unsigned int16 gblTimer = 0;
int gblCmdTimeOut = 0;
int gblUsbBuffer[USB_BUFFER_SIZE];
int gblUsbBufferPutIndex=0;
int gblUsbBufferGetIndex=0;
int gblUsbBufferIsFull=FALSE;
int HILOWHasArrivedFlag = 0;
int16 adressHILOW = 0;
char gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
char gblFlashBufferPtr=0;
int16 gblFlashBaseAddress;
int ttTimer0 = 0; 
int   gblStkPtr,  gblErrFlag;
unsigned int16 gblStack[STACK_SIZE];
int   gblInputStkPtr;
unsigned int16 gblInputStack[INPUT_STACK_SIZE];
int16 gblLoopAddress=0;
int16 gblRepeatCount=0;
int1 gblONFORNeedsToFinish=0;
int16 globalVariables[16]={0};
unsigned int16 gblRecordPtr;
unsigned int16  gblMemPtr ,gblRWCount;
