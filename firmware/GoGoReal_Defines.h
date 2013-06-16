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

//motors
#define M0 0
#define M1 1
#define M2 2
#define M3 3
#define SERVO 0 
#define DC 1
#define STEPPER 2

#define MTR_COUNT      4
#define T0_COUNTER     64937
#define MTR_MAX_INTRPS 400 // 400 * 0.05ms == 20ms
#define MTR_MAX_POWER  20

#define MTR_AB_ENB     PIN_B7
#define MTR_CD_ENB     PIN_D6

#define M0_ENB         PIN_B6
#define M0_S1          PIN_B5
#define M0_S2          PIN_B4

#define M1_ENB         PIN_B3
#define M1_S1          PIN_B2
#define M1_S2          PIN_D7

#define M2_ENB         PIN_D5
#define M2_S1          PIN_D4
#define M2_S2          PIN_D3

#define M3_ENB         PIN_D2
#define M3_S1          PIN_C6
#define M3_S2          PIN_C7

//time
#define T1_COUNTER      5535 // 10 ms
#define MIN_TIME_PRESSING_BUTTON 500//ms

//sensor
#define MIN_TIME_CHANGE_SENSOR 100

//statck
#define STACK_SIZE 32

//usb 
#define USB_BUFFER_SIZE 32

//Commands
#define CMD_VERSION     0
#define CMD_LEDON       1
#define CMD_LEDOFF      2
#define CMD_READSENSOR  3
#define CMD_BOOTLOADER  4

#define CMD_MOTORS_BASE    64
#define CMD_ACTIVATEMOTORS 64
#define CMD_MOTORS_ON      65
#define CMD_MOTORS_OFF     66
#define CMD_MOTORS_REVERT  67
#define CMD_MOTORS_THISWAY 68
#define CMD_MOTORS_THATWAY 69
#define CMD_MOTORS_POWER   70
#define CMD_MOTORS_ANGLE   71

/*
######################################################################
*/
#define CMD_READ_SENSOR      0x01
#define CMD_MOTOR_CONTROL   0x02

#define CMD_BURST_MODE      0x05
#define CMD_MISC_CONTROL   0x06
#define CMD_Version         0x07

#define TURN_USER_LED_ON      0
#define TURN_USER_LED_OFF     1

#define MISC_USER_LED    0
#define MISC_BEEP        1
#define MISC_SET_PWM     2
#define MISC_UPLOAD_EEPROM    3
#define MISC_I2C_SETUP  4
#define MISC_I2C_RW     5

#define I2C_START  0
#define I2C_STOP   1
#define I2C_WRITE  2
#define I2C_READ   3




#define EEPROMuploadHeader1   0xEE
#define EEPROMuploadHeader2   0x11

// this is used in main(). Determinds how long to wait for the
// second command byte.
#define RETRY         100


//  Sensor read modes
#define NORMAL_READ  0
#define MAX_READ     1
#define MIN_READ     2
#define MEAN_READ    3


// this const defines the last command that is
// one byte long. Please refer to the CMD bits
// in the GoGo kit serial protocol

#define ONE_BYTE_CMD   3

#define WAITING_FOR_FIRST_HEADER   1
#define WAITING_FOR_SECOND_HEADER   2
#define WAITING_FOR_CMD_BYTE      3
#define WAITING_FOR_SECOND_CMD_BYTE   4
#define CMD_READY               5
#define PIEZO              PIN_C2
#define LOCAL              0
#define REMOTE             1


//////////////////////////////////////////////////
//  Location of the run button vector.
//  This is 0xff0 in the cricket. We map it to
//  the following address.
#define RUN_BUTTON_BASE_ADDRESS           0x7FF0

//////////////////////////////////////////////////
//  Base address for the memory pointer storage
//  - Normally this is kept in the variable gblRecordPtr
//  - But we record this value to the flash mem
//    everytime we record data.
//  - This way we know where is the most recently
//    recorded location. It would be useful to know
//    this in the case of temporary power loss.
//  - It is used for automatic Data upload
// note that the mem pointer occupies two bytes
#define MEM_PTR_LOG_BASE_ADDRESS 0x7608

//////////////////////////////////////////////////
//   User program memory map
//   we storm the cricket logo code in the
//   flash memory to increase the speed.
//
#define FLASH_USER_PROGRAM_BASE_ADDRESS   0x5D80
#define FIRMWARE_END   0x5D30

/////////////////////////////////////////////////
//   Base address for the array memory
#define ARRAY_BASE_ADDRESS      0x6700


/////////////////////////////////////////////////
//   Data recording memory map
#define RECORD_BASE_ADDRESS      0x59C0


// Logo VM state machine definition

#define IDLE                     0

/// Comm protocol Sates

#define SET_PTR_HI_BYTE          128
#define SET_PTR_LOW_BYTE         129
#define READ_BYTES_COUNT_HI      130
#define READ_BYTES_COUNT_LOW     131
#define WRITE_BYTES_COUNT_HI     132
#define WRITE_BYTES_COUNT_LOW    133
#define WRITE_BYTES_SENDING      134
#define CRICKET_NAME             135

/// Comm commands

#define SET_PTR         0x83
#define READ_BYTES      0x84
#define WRITE_BYTES     0x85
#define RUN             0x86
#define CRICKET_CHECK   0x87

//   Op code

#define  CODE_END             0
#define  NUM8                 1
#define  NUM16                2
#define  LIST                 3
#define  EOL                  4
#define  EOLR                 5
#define  LTHING               6
#define  STOP                 7
#define  OUTPUT               8
#define  REPEAT               9
#define  COND_IF              10
#define  COND_IFELSE          11
#define  BEEP                 12
#define  NOTE                 13
#define  WAITUNTIL            14
#define  LOOP                 15
#define  WAIT                 16
#define  TIMER                17
#define  RESETT               18
#define  SEND                 19
#define  IR                   20
#define  NEWIR                21
#define  RANDOM               22
#define  OP_PLUS               23
#define  OP_MINUS               24
#define  OP_MULTIPLY            25
#define  OP_DIVISION            26
#define  OP_REMAINDER         27
#define  OP_EQUAL               28
#define  OP_GREATER            29
#define  OP_LESS               30
#define  OP_AND               31
#define  OP_OR                  32
#define  OP_XOR               33
#define  OP_NOT               34
#define  SETGLOBAL            35
#define  GETGLOBAL            36
#define  ASET                  37
#define  AGET                  38
#define  RECORD               39
#define  RECALL               40
#define  RESETDP               41
#define  SETDP                  42
#define  ERASE                  43
#define  WHEN                  44
#define  WHENOFF               45
#define  M_A                  46
#define  M_B                  47
#define  M_AB                  48
#define  M_ON                  49
#define  M_ONFOR               50
#define  M_OFF                  51
#define  M_THISWAY            52
#define  M_THATWAY            53
#define  M_RD                  54
#define  SENSOR1               55
#define  SENSOR2               56
#define  SWITCH1               57
#define  SWITCH2               58
#define  SETPOWER               59
#define  BRAKE                  60
#define  BSEND                  61
#define  BSR                  62
#define  M_C                  63
#define  M_D                  64
#define  M_CD                  65
#define  M_ABCD               66
#define  FASTSEND               67
#define  REALLY_STOP          68
#define  EB                     69
#define  DB                     70
#define  LOW_BYTE               71
#define  HIGH_BYTE            72

/// These code are unique to the GoGo board
#define  SENSOR3              73
#define  SENSOR4              74
#define  SENSOR5              75
#define  SENSOR6              76
#define  SENSOR7              77
#define  SENSOR8              78
#define  SWITCH3              79
#define  SWITCH4              80
#define  SWITCH5              81
#define  SWITCH6              82
#define  SWITCH7              83
#define  SWITCH8              84

#define ULED_ON               85
#define ULED_OFF              86

#define SERVO_SET_H           87
#define SERVO_LT              88
#define SERVO_RT              89

#define TALK_TO_MOTOR        90   // this replaces the M_A, M_B, M_C, ... commands.
// it will allow a more flexible motor addressing
// i.e. abc, would now work.
// Only the GoGo Compiler (not Jackal/Logo Blocks)
// will make use of this.

#define CL_I2C_START          91
#define CL_I2C_STOP           92
#define CL_I2C_WRITE          93
#define CL_I2C_READ           94


