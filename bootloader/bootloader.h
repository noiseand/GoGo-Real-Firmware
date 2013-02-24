//************************* GOGOBR   *******************************************//
//*****  bootloader  ***********************************************************//
//*****  autor:  Felipe Augusto Silva  *****************************************//
//*****  email:  fel1310@hotmail.com   *****************************************//
//******************************************************************************//
#include <18f4550.H>
#device ADC=10 *=16
#fuses HSPLL,NOWDT,NOPROTECT,USBDIV,PLL5,NODEBUG,CPUDIV1,VREGEN,NOLVP
#use delay(clock=48000000)
#use i2c(master, sda=PIN_B0, scl=PIN_B1, FORCE_HW)

#include <usb_cdc.h>

//definição de pinos da placa
#define RUN_BUTTON   PIN_D1
#define RUN_LED      PIN_A4
#define USER_LED     PIN_D0
#define USER_LED2    PIN_C0
#define USER_LED3    PIN_C1

//================ GOGOUSB bootloader ===============================================//
#define LOADER_SIZE        (0x183F)

/* Bootloader Flag:
local_flag=TRUE; while using bootloader
local_flag=FALSE; otherwise
*/
#define local_flag         0x25
#reserve local_flag


#define APPLICATION_START  (LOADER_SIZE+1)
#define APPLICATION_END    (getenv("PROGRAM_MEMORY")-1)
#define APPLICATION_ISR    (APPLICATION_START+8)


#ifdef _bootloader
   #define EEPROM_ERASE_SIZE  getenv("FLASH_ERASE_SIZE")
   #define EEPROM_WRITE_SIZE  getenv("FLASH_WRITE_SIZE")
   #define LOADER_ISR 0x28
   #build(interrupt=LOADER_ISR)
#endif

#ifndef _bootloader
   #build(reset=APPLICATION_START, interrupt=APPLICATION_ISR)
   #org 0, LOADER_SIZE {}
#endif
