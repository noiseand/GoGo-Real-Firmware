/* 
 * Copyright (C) 2010-2012 Lucas Aníbal Tanure Alves - ME
 * Contact   Lucas Tanure [ltanure@gmail.com] 
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
int1 beep_is_high = 0;

//Usb Buffer
unsigned int8 usbBuffer[USB_BUFFER_SIZE];
unsigned int8 usbBufferStart = 0;
unsigned int8 usbBufferEnd = 0;
unsigned int8 usbBufferSize = 0;

//Main Control
int1 start_stop_logo_machine = 0;
int1 gblLogoIsRunning = 0;
unsigned int16 time_button_pressed = 0;
unsigned int16 gblTimer = 0;
int gblCurSensorChannel = 0;

//Logo Code Execution
unsigned int16 gblWaitCounter =0;
int1 motor_onfor_needs_to_finish[MotorCount] = {0, 0, 0, 0};
unsigned int16 motor_onfor[MotorCount] = {0, 0, 0, 0};
unsigned char gblMostRecentlyReceivedByte = 0;
int1 gblNewByteHasArrivedFlag = 0;
int16 gblLoopAddress=0;
int16 gblRepeatCount=0;
int16 globalVariables[16]={0};

//Stack
int   gblStkPtr = 0;
int   gblInputStkPtr = 0;
unsigned int16 gblStack[STACK_SIZE];
unsigned int16 gblInputStack[INPUT_STACK_SIZE];


//Motor
int16  MotorENPins [MotorCount]={  MTR1_EN, MTR2_EN, MTR3_EN, MTR4_EN};
int16  MotorCWPins [MotorCount]={  MTR1_CW, MTR2_CW, MTR3_CW, MTR4_CW};
int16  MotorCCPins [MotorCount]={  MTR1_CC, MTR2_CC, MTR3_CC, MTR4_CC};
int gblMotorMode=0b00000000;
int gblActiveMotors;
int gblMotorDir=0;
int gblMotorONOFF = 0;
int gblMtrDuty[MotorCount+1] = {0xff,0xff,0xff,0xff,0xff};
int ttTimer0 = 0;
unsigned int gblTimer0Counter = MotorCount; // Motor duty cycle counter.
unsigned int gblDutyCycleFlag = 0;
unsigned int gblCurrentDutyIndex = 0;


unsigned int8  gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
unsigned int8  gblFlashBufferPtr=0;
unsigned int16 gblFlashBaseAddress = 0;
unsigned int16 gblRecordPtr = 0;
unsigned int16 gblMemPtr = 0;
unsigned int16 gblRWCount = 0;

