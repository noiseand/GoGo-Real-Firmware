/* Copyright (C) 2010-2012 Lucas Anibal Tanure Alves - ME
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

//
// Copyright (C) 2001-2007 Massachusetts Institute of Technology
// Contact   Arnan (Roger) Sipiatkiat [arnans@gmail.com]

//************************* GOGO BR  *******************************************//
//*****  contact  Felipe Augusto Silva  *****************************************//
//*****  email:  fel1310@hotmail.com   *****************************************//
//******************************************************************************//

#define _bootloader
#include "bootloader.h"

// degine os bytes de comunicacao com o firmware downloader
#define READY_FOR_NEXT  0x11
#define FINISH_FLAG     0x55
#define BOOTLOADER_OVERWRITE  0x80

#define USBBufferSize 32  // USB input buffer size


int8 flag_de_bootloader;
#locate flag_de_bootloader=local_flag

/*
 Goto the interrupt vector of the application.
*/
#org 0x08,0x17
void high_isr(void) {
    if (bit_test(flag_de_bootloader,0)) {
        goto_address(LOADER_ISR);
    } else {
        goto_address(APPLICATION_ISR);
    }
}

#org 0x18,0x27
void low_isr(void) {
    if (bit_test(flag_de_bootloader,0)) {
        goto_address(LOADER_ISR+0x10);
    } else {
        goto_address(APPLICATION_ISR+0x10);
    }
}

void write_program(int32 location, char *src, int16 size){
   int8 block[EEPROM_ERASE_SIZE];
   int32 block_start;
   int8 i;
   int8 num;
   block_start = location & (~((int32)EEPROM_ERASE_SIZE-1));
   i=location-block_start;
   while (size) 
   {
      read_program_memory(block_start, block, sizeof(block));  //read entire block to ram buffer
      if (size>(EEPROM_ERASE_SIZE-i)){
        num=EEPROM_ERASE_SIZE-i;
      } else {
        num=size;
      }
      memcpy(&block[i],src,num);    //modify ram buffer
      erase_program_eeprom(block_start);     //erase entire block
      write_program_memory(block_start, block, sizeof(block));    //write modified block
      src+=num;
      block_start+=EEPROM_ERASE_SIZE;
      i=0;
      size-=num;
   }
}

int a2i(char asciiByte) {
    if (asciiByte >= 'A'){
        return((asciiByte) - 'A' + 10);
    } else { 
        return( asciiByte - '0');
    }
}

void main() {
  int go=0;
  int i=0,j=0;
  int1 notDone = 1;
  unsigned int16 extended_linear_address = 0;
  unsigned int16 writeAddr =0 ;
  int Buffer[USBBufferSize];
  int recLen;
  char recType;
  if (input(RUN_BUTTON)) {
    go = 1;
  }else{
    int eeprom = read_EEPROM(EEPROM_FLAG_ADDR);
    if(eeprom == EEPROM_FLAG_CODE){
      go = 1;
      write_eeprom(EEPROM_FLAG_ADDR,0x00);
      delay_ms(1000);
    }
  }
  if (go == 1) {
    flag_de_bootloader=TRUE;
    usb_cdc_init();
    usb_init();
    while (!usb_enumerated());
    output_high(RUN_LED);
    do {
      output_high(USER_LED);
      while (usb_cdc_getc() != ':');
      recLen = (a2i(usb_cdc_getc()) << 4) + a2i(usb_cdc_getc());// numero de bytes de dados
      writeAddr = (a2i(usb_cdc_getc()) << 4) + a2i(usb_cdc_getc());
      writeAddr <<= 4;
      writeAddr += a2i(usb_cdc_getc());
      writeAddr <<= 4;
      writeAddr += a2i(usb_cdc_getc());
      usb_cdc_getc();
      recType = usb_cdc_getc();
      for (i=0;i<recLen;i++) {
        Buffer[i] = (a2i(usb_cdc_getc()) << 4) + a2i(usb_cdc_getc());
      }
      output_low(USER_LED);
      if (recType == '1'){
        printf(usb_cdc_putc, "%c", FINISH_FLAG);
        notDone = 0;
      }
      else if (recType == '4') {
        extended_linear_address = (int16) ((Buffer[0] << 8) + Buffer[1]);
        printf(usb_cdc_putc, "%c", READY_FOR_NEXT);
      }
      else if (recType == '0') {
        if (extended_linear_address > 0) {
          if (extended_linear_address == 0xf0) {
            for (i=0,j=0;i<(recLen/2);i++,j=j+2){
              write_eeprom((int)(writeAddr + i), Buffer[j]);
            }
          } else if (extended_linear_address == 0x30 ) {}
        } else {
          if (writeAddr <= LOADER_SIZE) {
            printf(usb_cdc_putc, "%c", BOOTLOADER_OVERWRITE);
          } else {
            write_program(writeAddr,Buffer,recLen);
          }
        }
        printf(usb_cdc_putc, "%c", READY_FOR_NEXT);
      }
    } while (notDone);
    delay_ms(100);
    output_low(RUN_LED);
    output_low(USER_LED);
    reset_cpu();
  }
  flag_de_bootloader=FALSE;
  goto_address(APPLICATION_START);
}

#ORG default
