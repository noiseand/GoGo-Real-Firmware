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

void clearStack();
void stkPush(unsigned int16 stackItem);
unsigned int16 stkPop();
void inputPush(unsigned int16 stackItem);
unsigned int16 inputPop(void);
void enableTimer0();
void tryDisableTimer0();
void setNextInterupt(unsigned int8 MotorNo);
void tryDisableChip();
void MotorON(int MotorNo);
void MotorOFF(int MotorNo);
void MotorPower(unsigned int8 MotorNo, unsigned int8 power,unsigned int16 waitTime);
void MotorRD(unsigned int8 MotorNo);
void MotorThisWay(unsigned int8 MotorNo);
void MotorThatWay(unsigned int8 MotorNo);
void SetMotorMode(int MotorNo, int mode);
int motorMode(int MotorNo);
void MotorControl(int MotorCmd, unsigned int8 power);
unsigned int16 readSensor(int sensorNo);
void init_board();
void init_variables();
void processCommunication();
unsigned int8 readUsbBuffer();
void updateUsbBuffer();
/*
To  compile
*/

void flashSetWordAddress(int16 address);
void flashFlushBuffer();
void flashWrite(int16 InByte);

