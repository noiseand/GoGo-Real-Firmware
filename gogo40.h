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

// Copyright (C) 2001-2007 Massachusetts Institute of Technology
// Contact   Arnan (Roger) Sipiatkiat [arnans@gmail.com]

//************************* GOGO BR  *******************************************//
//*****  contact  Felipe Augusto Silva  *****************************************//
//*****  email:  fel1310@hotmail.com   *****************************************//
//******************************************************************************//

#Define BOARD_VERSION 1
#Define FIRMWARE_VERSION 1

#define T1_COUNTER      28036
#define CMD_PING        0
#define CMD_BOOTLOADER  4
#define CMD_MODE        5
#define CMD_LOGO_TURN_ON        6
#define CMD_NOT_LOGO_TURN_ON 7
#define CMD_BEEP        196
#define CMD_LED_ON      192
#define CMD_LED_OFF     193
#define CMD_READ_SENSOR 32
#define CMD_TALK_TO_MOTOR   127
#define CMD_MOTOR_CONTROL   200
#define CMD_MOTOR_POWER      201
#define MISC_SET_PWM     202


#define MTR_ON       0
#define MTR_OFF      1
#define MTR_RD       2
#define MTR_THISWAY  3
#define MTR_THATWAY  4
#define MTR_COAST    5

#define LOGO_TURN_ON_ADDR 0x02
#define LOGO_TURN_ON_FLAG 0xDB


//old

#define defaultPort      0
#define channelSwitchDelay   50 

#define MISC_I2C_SETUP  4
#define MISC_I2C_RW     5

#define I2C_START  0
#define I2C_STOP   1
#define I2C_WRITE  2
#define I2C_READ   3


#define EEPROMuploadHeader1   0xEE
#define EEPROMuploadHeader2   0x11


/// How many motors does the board have.
#define MotorCount      4

/// Motor Modes
#define MOTOR_NORMAL    0
#define MOTOR_SERVO     1

// Motor Pin Mappings

#define MTR1_CW         PIN_B5
#define MTR1_CC         PIN_B4
#define MTR1_EN         PIN_B6

#define MTR2_CW         PIN_B2
#define MTR2_CC         PIN_D7
#define MTR2_EN         PIN_B3

#define MTR3_CW         PIN_D4
#define MTR3_CC         PIN_D3
#define MTR3_EN         PIN_D5

#define MTR4_CW         PIN_C6
#define MTR4_CC         PIN_C7
#define MTR4_EN         PIN_D2

#define PIC_TRIS_B   0b00000011
#define PIC_TRIS_A   0b00101111
#define PIC_TRIS_C   0b00000000
#define PIC_TRIS_D   0b00000010
#define PIC_TRIS_E   0b00000111


#define MOTOR_AB_EN     PIN_B7   // controls the power to the motor chip
#define MOTOR_CD_EN     PIN_D6   // controls the power to the motor chip


//=========================================================
// I2C Stuff
#define I2C_SCL            PIN_B1
#define I2C_SDA            PIN_B0
//=========================================================
#define PIEZO              PIN_C2
// ========================================================
// Serial Buffer Constants

#define USB_BUFFER_SIZE 20

#define USB_NO_DATA                  0
#define USB_SUCCESS                  1
#define USB_OVERFLOW                 2

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

#define STACK_SIZE              32
#define INPUT_STACK_SIZE        32


/// Logo VM state machine definition

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


//////////////////////////////////////
//
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

#define TALK_TO_MOTOR        90

#define CL_I2C_START          91
#define CL_I2C_STOP           92
#define CL_I2C_WRITE          93
#define CL_I2C_READ           94

// Tone sounds

#define mute    0 //T2_DISABLED,     0,  1 //Mute
#define B3   4061 //T2_DIV_BY_16,  253,  6 //B3       261.63    //Prescaler 1:16; Postscaler 1:6; TMR2 Preload = 253; Actual Interrupt Time : 2,0245 ms
#define C4   3837 //T2_DIV_BY_16,  239,  6 //C        261.63    //Prescaler 1:16; Postscaler 1:6; TMR2 Preload = 239; Actual Interrupt Time : 1.9125 ms
#define CC4  3613 //T2_DIV_BY_16,  225,  6 //C#       277.18    //Prescaler 1:16; Postscaler 1:6; TMR2 Preload = 225; Actual Interrupt Time : 1.8005 ms
#define D4   3421 //T2_DIV_BY_16,  213,  6 //D        293.66    //Prescaler 1:16; Postscaler 1:6; TMR2 Preload = 213; Actual Interrupt Time : 1.7045 ms
#define DC4  3867 //T2_DIV_BY_16,  241,  5 //D#       311.13    //Prescaler 1:16; Postscaler 1:5; TMR2 Preload = 241; Actual Interrupt Time : 1.607083333 ms
#define E4   3643 //T2_DIV_BY_16,  227,  5 //E        329.63    //Prescaler 1:16; Postscaler 1:5; TMR2 Preload = 227; Actual Interrupt Time : 1.51375 ms
#define F4   3451 //T2_DIV_BY_16,  215,  5 //F        349.23    //Prescaler 1:16; Postscaler 1:5; TMR2 Preload = 215; Actual Interrupt Time : 1.43375 ms
#define FC4  4080 //T2_DIV_BY_4,   253, 16 //F#       369.99    //Prescaler 1:4; Postscaler 1:16; TMR2 Preload = 253; Actual Interrupt Time : 1.350666667 ms
#define G4   3856 //T2_DIV_BY_4,   239, 16 //G        392.00    //Prescaler 1:4; Postscaler 1:16; TMR2 Preload = 239; Actual Interrupt Time : 1.276 ms
#define GC4  3886 //T2_DIV_BY_4,   241, 15 //G#       415.30    //Prescaler 1:4; Postscaler 1:15; TMR2 Preload = 241; Actual Interrupt Time : 1.20625 ms
#define A4   3916 //T2_DIV_BY_4,   243, 14 //A        440.00    //Prescaler 1:4; Postscaler 1:14; TMR2 Preload = 243; Actual Interrupt Time : 1.135166667 ms
#define AC4  3978 //T2_DIV_BY_4,   247, 13 //A#       466.16    //Prescaler 1:4; Postscaler 1:13; TMR2 Preload = 247; Actual Interrupt Time : 1.071416667 ms
#define B4   4072 //T2_DIV_BY_4,   253, 12 //B        493.88    //Prescaler 1:4; Postscaler 1:12; TMR2 Preload = 253; Actual Interrupt Time : 1.013 ms
#define C5   3848 //T2_DIV_BY_4,   239, 12 //C5       523.25    //Prescaler 1:4; Postscaler 1:12; TMR2 Preload = 239; Actual Interrupt Time : 957 us
#define CC5  3958 //T2_DIV_BY_4,   246, 11 //C#5      1108.73   //Prescaler 1:4; Postscaler 1:11; TMR2 Preload = 246; Actual Interrupt Time : 902,916666667 us
#define D5   3734 //T2_DIV_BY_4,   232, 11 //D5       1108.73   //Prescaler 1:4; Postscaler 1:11; TMR2 Preload = 232; Actual Interrupt Time : 851,583333333 us
#define DC5  3876 //T2_DIV_BY_4,   241, 10 //D#5      1244.51   //Prescaler 1:4; Postscaler 1:10; TMR2 Preload = 241; Actual Interrupt Time : 804,166666667 us
#define E5   4066 //T2_DIV_BY_4,   253,  9 //E5       1244.51   //Prescaler 1:4; Postscaler 1: 9; TMR2 Preload = 253; Actual Interrupt Time : 759,75 us
#define F5   3826 //T2_DIV_BY_4,   238,  9 //F5       1396.91   //Prescaler 1:4; Postscaler 1: 9; TMR2 Preload = 238; Actual Interrupt Time : 714,75 us
#define FC5  4064 //T2_DIV_BY_4,   253,  8 //F#5      1479.98   //Prescaler 1:4; Postscaler 1: 8; TMR2 Preload = 253; Actual Interrupt Time : 675,333333333 us                   
#define G5   3840 //T2_DIV_BY_4,   239,  8 //G5       1567.98   //Prescaler 1:4; Postscaler 1: 8; TMR2 Preload = 239; Actual Interrupt Time : 638 us
#define GC5  3616 //T2_DIV_BY_4,   225,  8 //G#5      1661.22   //Prescaler 1:4; Postscaler 1: 8; TMR2 Preload = 225; Actual Interrupt Time : 600,666666667 us
#define A5   3902 //T2_DIV_BY_4,   243,  7 //A5       1760.00   //Prescaler 1:4; Postscaler 1: 7; TMR2 Preload = 243; Actual Interrupt Time : 567,583333333 us                                 
#define AC5  3694 //T2_DIV_BY_4,   230,  7 //A#5      1864.66   //Prescaler 1:4; Postscaler 1: 7; TMR2 Preload = 230; Actual Interrupt Time : 537,25 us
#define B5   4060 //T2_DIV_BY_4,   253,  6 //B5       1975.53   //Prescaler 1:4; Postscaler 1: 6; TMR2 Preload = 253; Actual Interrupt Time : 506,5 
