/* Copyright (C) 2010-2012 Lucas Aníbal Tanure Alves - ME
* Contact: Lucas Tanure [lucastanure@gogoreal.com.br]
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

int   gblInputStkPtr;
unsigned int16 gblStack[STACK_SIZE];
int gblStkPtr;
int16 globalVariables[16]={0};
unsigned int16 gblRecordPtr;
unsigned int16  gblMemPtr, gblRWCount;
int16  MotorENPins [MotorCount];
int16  MotorCWPins [MotorCount];
int16  MotorCCPins [MotorCount];
unsigned int CMD_STATE;
int gbl_cur_cmd, gbl_cur_param, gbl_cur_ext, gbl_cur_ext_byte;
int gblBurstModeBits;
int gblBurstModeCounter;
int1 gblSlowBurstMode;
int1 gblSlowBurstModeTimerHasTicked;
int gblCurSensorChannel;
int gblMotorMode;
int gblActiveMotors;
int gblMotorDir;
int gblMotorONOFF;
int gblMtrDuty[MotorCount+1];
unsigned int gblTimer0Counter;
unsigned int gblDutyCycleFlag;
unsigned int gblCurrentDutyIndex; 
unsigned char gblMostRecentlyReceivedByte;
int1 gblNewByteHasArrivedFlag;
int1 gblLogoIsRunning;
int1 gblButtonPressed;
int1 gblBtn1AlreadyPressed;
unsigned int16 gblWaitCounter;
unsigned int16 gblTimer;
int gblCmdTimeOut;
int gblUsbBuffer[USB_BUFFER_SIZE];
int gblUsbBufferPutIndex;
int gblUsbBufferGetIndex;
int gblUsbBufferIsFull=FALSE;
int HILOWHasArrivedFlag;
int16 adressHILOW;
char gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
char gblFlashBufferPtr;
int16 gblFlashBaseAddress;
int ttTimer0; 
int16 gblLoopAddress=0;
int16 gblRepeatCount=0;
int1 gblONFORNeedsToFinish=0;