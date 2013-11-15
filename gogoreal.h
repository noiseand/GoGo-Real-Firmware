/* 
 * Copyright (C) 2010-2013 Lucas Ani­bal Tanure Alves - ME
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

/////////////////////////////////////////////////////////////////
//  Function Declaration
/////////////////////////////////////////////////////////////////

void stkPush(unsigned int16 stackItem);
unsigned int16 stkPop(void);
unsigned int8 readUsbBuffer();
void write_logo_code();
void updateUsbBuffer();

void inputPush(unsigned int16 stackItem);
unsigned int16 inputPop(void);
unsigned int16 fetchNextOpcode();
void evalOpcode(unsigned char opcode);
void flashSetWordAddress(int16 address);
void flashWrite(int16 InByte);
void Ping(int Param);
void TalkToMotor(int MotorBits);
void MotorControl(int MotorCmd);
void SetMotorPower(int Power);
void ChangeMotorPower(int delta);
void sortMtrDuty();
void SetMotorMode(int motorMode); // normal or servo

void ENHigh(int groupNo);
void ENLow(int groupNo);

void MotorON(int MotorNo);
void MotorOFF(int MotorNo);
void MotorRD(int MotorNo);
void MotorThisWay(int MotorNo);
void MotorThatWay(int MotorNo);
void MotorCoast(int MotorNo);
void miscControl(int cur_param, int cur_ext, int cur_ext_byte);

void beep();
unsigned int16 readSensor(int sensorNo);

void ProcessInput();
void init_variables();
void intro();
void Halt();
void initBoard();

void timer2ISR();
void version();
