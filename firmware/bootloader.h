//************************* GOGOBR   *******************************************//
//*****  bootloader  ***********************************************************//
//*****  autor:  Felipe Augusto Silva  *****************************************//
//*****  email:  fel1310@hotmail.com   *****************************************//
//******************************************************************************//
#include <18f4550.H>
#device ADC=10 *=16
#fuses HSPLL,NOWDT,NOPROTECT,USBDIV,PLL5,NODEBUG,CPUDIV1,VREGEN,NOLVP
#use delay(clock=48000000)
#use i2c(master, sda=PIN_B0, scl=PIN_B1, FORCE_HW,slow)

#include <usb_cdc.h>

//definição de pinos da placa
#define RUN_BUTTON   PIN_D1
#define RUN_LED      PIN_A4
#define USER_LED     PIN_D0
#define CMD_MODE     5


#define LOADER_SIZE        (0x174F)
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
   #define LOADER_ISR 0x28
   #build(interrupt=LOADER_ISR)
#endif

#ifndef _bootloader
   #build(reset=APPLICATION_START, interrupt=APPLICATION_ISR)
   #org 0, LOADER_SIZE {}
#endif
