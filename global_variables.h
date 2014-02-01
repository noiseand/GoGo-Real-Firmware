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
//Usb Buffer
unsigned int8 usbBuffer[USB_BUFFER_SIZE];
unsigned int8 usbBufferStart = 0;
unsigned int8 usbBufferEnd = 0;
unsigned int8 usbBufferSize = 0;

//Logo Code Execution
unsigned int16 gblWaitCounter =0;

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

unsigned int8  gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
unsigned int8  gblFlashBufferPtr=0;
unsigned int16 gblFlashBaseAddress = 0;
unsigned int16 gblRecordPtr = 0;
unsigned int16 gblMemPtr = 0;
unsigned int16 gblRWCount = 0;

//############################################
//New

//Main Control
int1 start_stop_logo_machine = 0;
int1 gblLogoIsRunning = 0;
unsigned int16 time_button_pressed = 0;
unsigned int16 gblTimer = 0;
int gblCurSensorChannel = 0;
int1 beep_is_high = 0;

//motors Config
unsigned int16 mtrPwmPin[MTR_COUNT]; //pins
unsigned int16 mtrS1[MTR_COUNT]={  M0_S1, M1_S1, M2_S1, M3_S1};
unsigned int16 mtrS2[MTR_COUNT]={  M0_S2, M1_S2, M2_S2, M3_S2};
unsigned int16 mtrEnable[MTR_COUNT]={ M0_ENB, M1_ENB, M2_ENB, M3_ENB};
unsigned int16 mtrChipEnable[MTR_COUNT/2]={  MTR_AB_ENB, MTR_CD_ENB};
//motors Control
int1 intrp0Enabled;
int1 motor_onfor_needs_to_finish[MTR_COUNT] = {0, 0, 0, 0};
unsigned int16 motor_onfor[MTR_COUNT] = {0, 0, 0, 0};

unsigned int16 intrp0_count = 0;
unsigned int8  mtrsOnInterpt;//0000 off , 0000 not controlled by Interrupt
unsigned int8  mtrsDirectionNextTurn; // 1111 thisway, 1111 Turn On on the next Turn
unsigned int8  mtrsActive;// A,B,C,D active to send commands
unsigned int8  mtrsMode;// 01010101 // 85 DC DC DC DC 

unsigned int8  mtrRun[MTR_COUNT];
unsigned int16 mtrWait[MTR_COUNT];
unsigned int16 mtrNextInterrupt[MTR_COUNT];




