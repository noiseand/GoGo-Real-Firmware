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
#case
#include "bootloader.h"
#include <GoGoReal_Defines.h>
#include <GoGoReal_Variables.h>
#include <GoGoReal.h>
#include <stdlib.H>
#include <logovm.c>

#use fast_io(A)
#use fast_io(B)
#use fast_io(C)
#use fast_io(D)
#use fast_io(E)

// getBit (Byte >> motor) & 1
// setBit Byte |= (1<<motor)
// clear Byte &= ~(1<<motor)
// toggle number ^= 1 << motor;

#int_rtcc
void motor_Control() {
  for(iterator = 0; iterator < MotorCount;iterator++){
    if((motor_intrp_count[iterator] != max_number_interrupts) && ((motorsconfig >> iterator) & 1)){
      if(intrp0_count == 0){ //turn on
   if(motor_intrp_count[iterator] > 0){output_high(motor_enable_pin[iterator]);}
      }else{ // turn off 
        if(motor_intrp_count[iterator] == intrp0_count){output_low(motor_enable_pin[iterator]);}
      }
    }
  }
  intrp0_count++;
  if(intrp0_count == max_number_interrupts){intrp0_count = 0;}
}

void MotorON(int MotorNo) {
  output_low(motor_ground_pin[MotorNo]);
  output_high(motor_direct_pin[MotorNo]);
  output_high(motor_chip_enable_pin[MotorNo/2]);
  if(motor_intrp_count[MotorNo] == max_number_interrupts){
    output_high(motor_enable_pin[MotorNo]);
  } else {
    //enable timer0 interrupt
  }
  motorsconfig |= (1<<MotorNo); //set on
}

void MotorOFF(int MotorNo) {
  output_low(motor_ground_pin[MotorNo]);
  output_low(motor_direct_pin[MotorNo]);
  output_low(motor_enable_pin[MotorNo]);
  motorsconfig &= ~(1<<MotorNo); // set off
  chip_enable_off();
  motorControl_TurnOff();
}


void MotorRD(unsigned int8 MotorNo) {
  motorsconfig ^= 1 << (MotorNo + MotorCount); // toggle bit
  unsigned int16 temp_pin = motor_direct_pin[MotorNo];
  motor_direct_pin[MotorNo] = motor_ground_pin[MotorNo];
  motor_ground_pin[MotorNo] = temp_pin;
  output_low(motor_ground_pin[MotorNo]);
  if(motor_intrp_count[MotorNo] == max_number_interrupts && ((motorsconfig >> iterator) & 1)){
    output_high(motor_enable_pin[iterator]);
  }
}

void MotorThisWay(int MotorNo) {
  if(!((motorsconfig >> (MotorNo+MotorCount)) & 1)){
    MotorRD(MotorNo);
  }
}

void MotorThatWay(int MotorNo) {
  if((motorsconfig >> (MotorNo+MotorCount)) & 1){
    MotorRD(MotorNo);
  }
}

void motorControl_TurnOff(){
  for(iterator = 0; iterator < MotorCount;iterator++){
    if((motorsconfig >> iterator) & 1){
      return;
    }
  }
  //disable timer0
}

void chip_enable_off(){
  if(!(((motorsconfig >> m0) & 1) && ((motorsconfig >> m1) & 1))){
    output_low(motor_chip_enable_pin[0]);
  }
  if(!(((motorsconfig >> m2) & 1) && ((motorsconfig >> m3) & 1))){
    output_low(motor_chip_enable_pin[1]);
  }
}

void SetMotorMode(int MotorNo, int mode) {
  if(mode == SERVO){ // 00
    motorsMode &= ~(1<<(MotorNo*2));
    motorsMode &= ~(1<<((MotorNo*2)+1));
    MotorThisWay(MotorNo);
    MotorON(MotorNo);
    return;
  }
  if(mode == DC){ // 01
    motorsMode |= (1<<(MotorNo*2));
    motorsMode &= ~(1<<((MotorNo*2)+1));
    return;
  }
  if(mode == STEPPER){ //10
    motorsMode &= ~(1<<(MotorNo*2));
    motorsMode |= (1<<((MotorNo*2)+1));
    return;
  }
}

int motorMode(int MotorNo) {
  if((motorsMode >> ((MotorNo*2)+1)) & 1){
    return STEPPER;
  }
  if((motorsMode >> (MotorNo*2)) & 1){
    return DC;
  }
  return SERVO;
}

//TODO
void MotorControl(int MotorCmd, int power) {
  for (iterator=0;iterator<MotorCount;iterator++) {
    if ((motorsActive >> iterator) & 1) {
      int8 mtrMode = DC;
      switch (MotorCmd) {
        case CMD_MOTORS_ANGLE:
          motor_intrp_count[iterator] = power;
          mtrMode = SERVO;
          break;
        case CMD_MOTORS_ON:
          MotorON(iterator);
          break;
        case CMD_MOTORS_OFF:
          MotorOFF(iterator);
          break;
        case CMD_MOTORS_REVERT:
          MotorRD(iterator);
          break;
        case CMD_MOTORS_THISWAY:
          MotorThisWay(iterator);
          break;
        case CMD_MOTORS_THATWAY:
          MotorThatWay(iterator);
          break;
        case CMD_MOTORS_POWER:
          motor_intrp_count[iterator] = power*4;
          break;
      }
      if(motorMode(iterator) != mtrMode){ SetMotorMode(iterator, mtrMode); }
    }
  }
}

#int_timer1
void clock_and_button() { //10 ms
  miliseconds += 10;
  if(miliseconds == 1000){
    seconds++;
    miliseconds = 0;
  }
  if(time_pressing_button_ms > min_time_pressing_button_ms){
    if(input(RUN_BUTTON)) {
      time_pressing_button_ms = 0;
      button_pressed = 1;
    }
  }
  time_pressing_button_ms += 10;
}

unsigned int16 readSensor(int sensorNo) {
  if (currentSensor != sensorNo) {
    set_adc_channel(sensorNo);
    delay_us(min_time_change_sensor);
    currentSensor=sensorNo;
   }
   return read_adc();
}


void init_board() {
   output_low(PIN_C0);
   output_low(PIN_C1);
   output_low(RUN_LED);
   output_low(USER_LED);

   set_tris_a(0b00101111);
   set_tris_b(0b00000011);
   set_tris_c(0b00000000);
   set_tris_d(0b00000010);
   set_tris_e(0b00000111);
   
   setup_adc_ports(AN0_TO_AN7);
   setup_adc(ADC_CLOCK_INTERNAL);
   set_adc_channel(currentSensor);

   set_timer1(T1_COUNTER);
   set_rtcc(T0_COUNTER);
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_2); // 10ms
   setup_timer_0(RTCC_INTERNAL | T0_DIV_1); // 50us
   enable_interrupts(INT_RTCC);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(INT_RDA);
   enable_interrupts(GLOBAL);

}

void init_variables() {
  //general loop iterator
  iterator = 0;
  //motor
  intrp0_count = 0;
  for (iterator=0;iterator<MotorCount;iterator++){motor_intrp_count[iterator] = 0;}
  motorsconfig = 0x11110000;
  motorsActive = 0;
  motorsMode = 0x01010101;
  motor_chip_enable_pin[0] = MOTOR_AB_EN;
  motor_chip_enable_pin[1] = MOTOR_CD_EN;
  motor_enable_pin[m0] = m0_enb;
  motor_enable_pin[m1] = m1_enb;
  motor_enable_pin[m2] = m2_enb;
  motor_enable_pin[m3] = m3_enb;
  motor_direct_pin[m0] = m0_high;
  motor_direct_pin[m1] = m1_high;
  motor_direct_pin[m2] = m2_high;
  motor_direct_pin[m3] = m3_high;
  motor_ground_pin[m0] = m0_gnd;
  motor_ground_pin[m1] = m1_gnd;
  motor_ground_pin[m2] = m2_gnd;
  motor_ground_pin[m3] = m3_gnd;
  //time 
  seconds = 0;
  miliseconds = 0;
  //button
  time_pressing_button_ms = 0;
  button_pressed = 0;
  //sensor
  currentSensor = 0; 
  //stacks
  stkPointer = 0;
  inputStkPointer =0;
  // usb
  //usbBuffer[usb_buffer_size]
  usbBufferStart=0;
  usbBufferEnd=0;
  usbBufferSize=0;
}


void processCommunication(){
  if(usbBufferSize == 0){return;}
  unsigned int8 command = readUsbBuffer();
  if(command < CMD_MOTORS_BASE){
    if(command == CMD_VERSION){
      printf(usb_cdc_putc,"%c%c%c", 0, 2, 0);
      return;
    }
    if(command == CMD_LEDON){
      output_high(USER_LED);
      return;
    }
    if(command == CMD_LEDOFF){
      output_low(USER_LED);
      return;
    }
    if(command == CMD_READSENSOR){
      if(usbBufferSize>0){
        unsigned int16 SensorVal = readSensor(readUsbBuffer());
        printf(usb_cdc_putc,"%c%c", SensorVal >> 8, SensorVal & 0xff);
      }
      return;
    }
  }else{
    if(command == CMD_ACTIVATEMOTORS){
      if(usbBufferSize>0){
        motorsActive = readUsbBuffer();
      }
      return;
    }
    if(command == CMD_MOTORS_POWER){
      if(usbBufferSize>0){
        MotorControl(command, readUsbBuffer());
      }
      return;
    }
    MotorControl(command,0);
  }
}


unsigned int8 readUsbBuffer() {
   unsigned int8 command = usbBuffer[usbBufferStart];
   usbBufferStart++;
   usbBufferStart %= usb_buffer_max_size;
   usbBufferSize--;
   printf(usb_cdc_putc,"%c", command);
   return command;
}

void updateUsbBuffer() {
  while(usb_cdc_kbhit() && usbBufferSize < usb_buffer_max_size){
    usbBuffer[usbBufferEnd] = usb_cdc_getc();
    usbBufferEnd ++;
    usbBufferEnd %= usb_buffer_max_size;
    usbBufferSize++;
  }
}

int main(){
  init_variables();
  init_board();
  
  usb_cdc_init();
  usb_init();
  usb_task();
 
  while (1) {
    updateUsbBuffer();
    processCommunication();
  }
}

/*
To  compile
*/

void flashSetWordAddress(int16 address) {

   gblFlashBaseAddress = address;
   gblFlashBaseAddress &= ~(int32)((getenv("FLASH_ERASE_SIZE"))-1);

   gblFlashBufferPtr = address - gblFlashBaseAddress;
   read_program_memory(gblFlashBaseAddress, gblFlashBuffer, getenv("FLASH_ERASE_SIZE"));
}

void flashFlushBuffer() {
  if(!(gblFlashBaseAddress <= FIRMWARE_END)){
    erase_program_eeprom(gblFlashBaseAddress);
    write_program_memory(gblFlashBaseAddress, gblFlashBuffer, getenv("FLASH_ERASE_SIZE"));
  }
}

void flashWrite(int16 InByte) {
   gblFlashBuffer[gblFlashBufferPtr++] = (int)InByte;
   gblFlashBuffer[gblFlashBufferPtr++] = (int)(InByte>>8);
   flashFlushBuffer();
}

