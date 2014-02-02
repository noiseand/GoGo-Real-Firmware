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
/* 
 * Copyright (C) 2010-2013 Lucas AniÂ­bal Tanure Alves - ME
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

// Copyright (C) 2001-2007 Massachusetts Institute of Technology
// Contact   Arnan (Roger) Sipiatkiat [arnans@gmail.com]

//************************* GOGO BR  *******************************************//
//*****  contact  Felipe Augusto Silva  *****************************************//
//*****  email:  fel1310@hotmail.com   *****************************************//
//******************************************************************************//

void stkPush(unsigned int16 stackItem);
unsigned int16 stkPop();
void inputPush(unsigned int16 stackItem);
unsigned int16 inputPop(void);
void sendBytes(unsigned int16 memPtr, unsigned int16 count);
unsigned int16 fetchNextOpcode();
void enableTimer0();
void tryDisableTimer0();
void tryDisableChip();
unsigned int8 motorMode(int MotorNo);
void setNextInterupt(unsigned int8 MotorNo);
void motors_on();
void motor_on(unsigned int8 MotorNo);
void motors_off();
void motor_off(unsigned int8 MotorNo);
void evalOpcode(unsigned char opcode);
void motors_reverse();

void motor_reverse(unsigned int8 MotorNo);

void motors_this_way();

void motor_this_way(unsigned int8 MotorNo);

void motors_that_way();

void motor_that_way(unsigned int8 MotorNo);

void motor_config(unsigned int8 MotorNo, unsigned int8 mode);

void motors_angle(unsigned int8 angle);

void motor_angle(unsigned int8 MotorNo, unsigned int8 angle);

void motors_power(unsigned int8 power);

void motor_power(unsigned int8 MotorNo, unsigned int8 power);

void flashSetWordAddress(int16 address);

void flashBufferedWrite(unsigned int16 InByte);

void flashFlushBuffer();

void flashWrite(int16 InByte);

unsigned int16 readSensor(int sensorNo);

void set_on_for(unsigned int16 delay);
void ProcessInput();

void write_logo_code();

unsigned int8 readUsbBuffer();

void updateUsbBuffer();

void init_variables();

void initBoard();

void beep();

void intro();




void tone(unsigned int16 beep);


