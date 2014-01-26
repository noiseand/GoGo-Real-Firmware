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
//************************* GOGOBR   *******************************************//
//*****  bootloader  ***********************************************************//
//*****  autor:  Felipe Augusto Silva  *****************************************//
//*****  email:  fel1310@hotmail.com   *****************************************//
//******************************************************************************//
#include <18f4550.H>
#device ADC=10 *=16
#fuses HSPLL,NOWDT,NOPROTECT,USBDIV,PLL5,NODEBUG,CPUDIV1,VREGEN,NOLVP,NOMCLR
#use delay(clock=48000000)
#use i2c(master, sda=PIN_B0, scl=PIN_B1, FORCE_HW,slow)

#include <usb_cdc.h>

//definição de pinos da placa
#define RUN_BUTTON   PIN_D1
#define RUN_LED      PIN_A4
#define USER_LED     PIN_D0
#define CMD_MODE     5


#define LOADER_SIZE        (0x17EF)
#define local_flag         0x25
#reserve local_flag

#define EEPROM_FLAG_ADDR         0x0
#define EEPROM_FLAG_CODE         0xCE

#define APPLICATION_START  (LOADER_SIZE+1)
#define APPLICATION_END    (getenv("PROGRAM_MEMORY")-1)
#define APPLICATION_ISR    (APPLICATION_START+8)

#define EEPROM_ERASE_SIZE  getenv("FLASH_ERASE_SIZE")
#define EEPROM_WRITE_SIZE  getenv("FLASH_WRITE_SIZE")

#ifdef _bootloader
#ROM getenv("EEPROM_ADDRESS")={EEPROM_FLAG_CODE}
#define LOADER_ISR 0x28
#build(interrupt=LOADER_ISR)
#endif

#ifndef _bootloader
#build(reset=APPLICATION_START, interrupt=APPLICATION_ISR)
#org 0, LOADER_SIZE {}
#endif