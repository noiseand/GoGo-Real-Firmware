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

//motors Config
unsigned int16 mtrEnable[MTR_COUNT]; //pins
unsigned int16 mtrS1[MTR_COUNT]; //pins
unsigned int16 mtrS2[MTR_COUNT]; //pins
unsigned int16 mtrChipEnable[MTR_COUNT/2]; //pins
//motors Control
int1 intrp0Enb;
unsigned int16 intrp0_count = 0;
unsigned int8  mtrsOnInterpt;//0000-0000 A,B,C,D on/off ; A ,B,C,D controlled by Interrupt
unsigned int8  mtrsDirectionNextTurn; // 1111-1111 ThisWay , Turn On on the next Turn
unsigned int8  mtrsActive;// A,B,C,D active to send commands
unsigned int8  mtrsMode;// 01010101 // 85 DC DC DC DC 

unsigned int8  mtrRun[MTR_COUNT];
unsigned int16 mtrWait[MTR_COUNT];
unsigned int16 mtrNextInterrupt[MTR_COUNT];

//time 
unsigned int16 seconds; // 0
unsigned int16 miliseconds; // 0

//sensor
unsigned int8 currentSensor; //0

//stack
unsigned int8 stkPointer; //0
unsigned int8 inputStkPointer; //0
unsigned int16 stack[STACK_SIZE];
unsigned int16 inputStack[STACK_SIZE];

//usb 
unsigned int8 usbBuffer[USB_BUFFER_SIZE];
unsigned int8 usbBufferStart; //0
unsigned int8 usbBufferEnd; //0
unsigned int8 usbBufferSize;//0
//button 
int1 button_pressed;
unsigned int16 time_pressing_button; //0

//unsigned int1 usbReady;//0

/*
######################################################################

*/
int1 gblLogoIsRunning;
int16 globalVariables[16]={0};
unsigned int16 gblRecordPtr;
unsigned int16  gblMemPtr;
int8 gblMostRecentlyReceivedByte;
int8 gblNewByteHasArrivedFlag;
unsigned int16 gblWaitCounter;
unsigned int16 gblTimer;
char gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
char gblFlashBufferPtr;
int16 gblFlashBaseAddress;
int16 gblLoopAddress=0;
int16 gblRepeatCount=0;
int1 gblONFORNeedsToFinish=0;
