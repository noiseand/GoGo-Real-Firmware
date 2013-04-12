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

//general loop iterator
unsigned int8 iterator = 0;

//motor
unsigned int16 intrp0_count = 0;
unsigned int16 motor_intrp_count[MotorCount]; //0000
unsigned int8  motorsconfig;//11110000 A on/off,B,C,D,A direction,B,C,D
unsigned int8  motorsActive;// A,B,C,D active to send commands
unsigned int8  motorsMode;// 01010101 // 85
unsigned int16 motor_enable_pin[MotorCount]; //pins
unsigned int16 motor_direct_pin[MotorCount]; //pins
unsigned int16 motor_ground_pin[MotorCount]; //pins
unsigned int16 motor_chip_enable_pin[MotorCount/2]; //pins

//time 
unsigned int16 seconds; // 0
unsigned int16 miliseconds; // 0


//sensor
unsigned int8 currentSensor; //0

//stack
unsigned int8 stkPointer;
unsigned int8 inputStkPointer;
unsigned int16 stack[max_stack_size];
unsigned int16 inputStack[max_stack_size];

//usb 
unsigned int8 usbBuffer[usb_buffer_max_size];
unsigned int8 usbBufferStart; //0
unsigned int8 usbBufferEnd; //0
unsigned int8 usbBufferSize;

//button 
int1 button_pressed;
unsigned int16 time_pressing_button_ms; //0

/*
######################################################################

*/

int   gblInputStkPtr;
unsigned int16 gblStack[32];
int gblStkPtr;
int16 globalVariables[16]={0};
unsigned int16 gblRecordPtr;
unsigned int16  gblMemPtr, gblRWCount;
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
int gblUsbBuffer[32];
int gblUsbBufferPutIndex;
int gblUsbBufferGetIndex;
int gblUsbBufferIsFull=FALSE;
int HILOWHasArrivedFlag;
int16 adressHILOW;
char gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
char gblFlashBufferPtr;
int16 gblFlashBaseAddress;
int16 gblLoopAddress=0;
int16 gblRepeatCount=0;
int1 gblONFORNeedsToFinish=0;
