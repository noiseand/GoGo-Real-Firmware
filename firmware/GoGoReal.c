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

//is controled by interrupt
//((motorsOnInterpt >> (MotorNo)) & 1)
//is on
//((motorsOnInterpt >> (MotorNo+MTR_COUNT)) & 1)
//next turn 
//((motorsDirectionNextTurn >> (MotorNo)) & 1)
//direction
//((motorsDirectionNextTurn >> (MotorNo+MTR_COUNT)) & 1)
  
void enableTimer0(){
  intrp0_count = 0;
  intrp0Enb =1;
  set_rtcc(T0_COUNTER);
  enable_interrupts(INT_RTCC);
}

void tryDisableTimer0(){
  if(((mtrsOnInterpt >> M0) & 1) && ((mtrsOnInterpt >> (M0+MTR_COUNT)) & 1)){return;}
  if(((mtrsOnInterpt >> M1) & 1) && ((mtrsOnInterpt >> (M1+MTR_COUNT)) & 1)){return;}
  if(((mtrsOnInterpt >> M2) & 1) && ((mtrsOnInterpt >> (M2+MTR_COUNT)) & 1)){return;}
  if(((mtrsOnInterpt >> M3) & 1) && ((mtrsOnInterpt >> (M3+MTR_COUNT)) & 1)){return;}
  intrp0Enb=0;
  disable_interrupts(INT_RTCC);
}

void setNextInterupt(unsigned int8 MotorNo){
  mtrNextInterrupt[MotorNo] = intrp0_count % (mtrRun[MotorNo]+mtrWait[MotorNo]);
  mtrNextInterrupt[MotorNo] = intrp0_count - mtrNextInterrupt[MotorNo];
  mtrNextInterrupt[MotorNo]+= mtrRun[MotorNo];
  if(mtrNextInterrupt[MotorNo] > intrp0_count){
    mtrsDirectionNextTurn &= ~(1<<MotorNo);
    return;
  }
  mtrsDirectionNextTurn |= (1<<MotorNo);
  mtrNextInterrupt[MotorNo] += mtrWait[MotorNo];
}

void tryDisableChip(){
  if((mtrsOnInterpt >> (M0+MTR_COUNT)) & 1){
    if((mtrsOnInterpt >> (M1+MTR_COUNT)) & 1){
      output_low(mtrChipEnable[0]);
    }
  }
  if((mtrsOnInterpt >> (M2+MTR_COUNT)) & 1){
    if((mtrsOnInterpt >> (M3+MTR_COUNT)) & 1){
      output_low(mtrChipEnable[1]);
    }
  }
}

#int_rtcc
void motor_ControlV2() {
  int8 mtrIterator = 0;
  for(mtrIterator = 0; mtrIterator < MTR_COUNT;mtrIterator++){
    if(((mtrsOnInterpt >> (mtrIterator)) & 1) && ((mtrsOnInterpt >> (mtrIterator+MTR_COUNT)) & 1)){//is on and controled by timer0
      if(intrp0_count == mtrNextInterrupt[mtrIterator]){
        if((mtrsDirectionNextTurn >> (mtrIterator)) & 1){
          mtrNextInterrupt[mtrIterator] += mtrRun[mtrIterator];
          output_high(mtrEnable[mtrIterator]);
        } else{
          mtrNextInterrupt[mtrIterator] += mtrWait[mtrIterator];
          output_low(mtrEnable[mtrIterator]);
        }
        mtrsDirectionNextTurn ^= 1 << mtrIterator;
        mtrNextInterrupt[mtrIterator] %= MTR_MAX_INTRPS;
      }
    }
  }
  intrp0_count++;
  intrp0_count %= MTR_MAX_INTRPS;
  set_rtcc(T0_COUNTER);
}

void MotorON(int MotorNo) {
  if((mtrsOnInterpt >> (MotorNo)) & 1){
    if(intrp0Enb==1 && (motorMode(MotorNo)==DC)) {
      setNextInterupt(MotorNo);
    }else{
      mtrNextInterrupt[MotorNo] = 0;
      enableTimer0();
    }
  } else {
    output_high(mtrEnable[MotorNo]);
  }
  output_high(mtrChipEnable[MotorNo/2]);
  mtrsOnInterpt |= (1<<(MotorNo+MTR_COUNT)); //set on
}

void MotorOFF(int MotorNo) {
  mtrsOnInterpt &= ~(1<<(MotorNo+MTR_COUNT)); // set off
  tryDisableTimer0();
  tryDisableChip();
  output_low(mtrEnable[MotorNo]);
}

void MotorAngle(unsigned int8 MotorNo, unsigned int8 power,unsigned int16 waitTime){
  mtrsDirectionNextTurn |= (1<<MotorNo); // set turnOn on next time
  mtrNextInterrupt[MotorNo] = 0;
  mtrRun[MotorNo] = power;
  mtrWait[MotorNo] = waitTime;
}

void MotorPower(unsigned int8 MotorNo, unsigned int8 power,unsigned int16 waitTime){
  mtrNextInterrupt[MotorNo] = 0; //give me time
  mtrRun[MotorNo] = power;
  mtrWait[MotorNo] = waitTime;
  if(waitTime == 0){
    mtrsOnInterpt &= ~(1<<MotorNo); //disable timer0 controll
    tryDisableTimer0();
    if((mtrsOnInterpt >> (MotorNo+MTR_COUNT)) & 1){
      output_high(mtrEnable[MotorNo]);
    }
  }else{
    mtrsOnInterpt |= (1<<MotorNo); // set controll by timer0
    if((mtrsOnInterpt >> (MotorNo+MTR_COUNT)) & 1){
      if(intrp0Enb==1){
        if(motorMode(MotorNo) == DC){
          setNextInterupt(MotorNo);
        }
      }else{
        mtrsDirectionNextTurn |= (1<<MotorNo); // set turnOn on next time
        mtrNextInterrupt[MotorNo] = 0;
        enableTimer0();
      }
    }
  }
}

void MotorRD(unsigned int8 MotorNo) {
  if((mtrsDirectionNextTurn >> (MotorNo+MTR_COUNT)) & 1){
    MotorThatWay(MotorNo);
  }else{
    MotorThisWay(MotorNo);
  }
}

void MotorThisWay(unsigned int8 MotorNo) {
  output_high(mtrS1[MotorNo]);
  output_low(mtrS2[MotorNo]);
  mtrsDirectionNextTurn |= (1<<(MotorNo+MTR_COUNT));
}

void MotorThatWay(unsigned int8 MotorNo) {
  output_low(mtrS1[MotorNo]);
  output_high(mtrS2[MotorNo]);
  mtrsDirectionNextTurn &= ~(1<<(MotorNo+MTR_COUNT));
}

void verifyMotorsMode(int mode) {
  int iterator = 0;
  for (iterator=0;iterator<MTR_COUNT;iterator++){
    if ((mtrsActive >> iterator) & 1) {
      if(motorMode(iterator)!=mode){
        SetMotorMode(iterator,mode);
      }
    }
  }
}

void SetMotorMode(int MotorNo, int mode) {
  if(mode == SERVO){ // 00
    mtrsOnInterpt |= (1<<MotorNo); // set controll by timer0
    MotorThatWay(MotorNo);
    MotorON(MotorNo);
    if(motorMode(MotorNo)==DC){
      unsigned int16 tmp = mtrEnable[MotorNo];
      mtrEnable[MotorNo] = mtrS1[MotorNo];
      mtrS1[MotorNo] = tmp;
    }
    output_high(mtrS1[MotorNo]); // enable motor
    mtrsMode &= ~(1<<(MotorNo*2));
    mtrsMode &= ~(1<<((MotorNo*2)+1));
    return;
  }
  if(mode == DC){ // 01
    if(motorMode(MotorNo)==SERVO){
      output_low(mtrS1[MotorNo]); // disable motor
      unsigned int16 tmp = mtrEnable[MotorNo];
      mtrEnable[MotorNo] = mtrS1[MotorNo];
      mtrS1[MotorNo] = tmp;
    }
    mtrsMode |= (1<<(MotorNo*2));
    mtrsMode &= ~(1<<((MotorNo*2)+1));
    return;
  }
  if(mode == STEPPER){ //10
    mtrsMode &= ~(1<<(MotorNo*2));
    mtrsMode |= (1<<((MotorNo*2)+1));
    return;
  }
}

int motorMode(int MotorNo) {
  if((mtrsMode >> ((MotorNo*2)+1)) & 1){
    return STEPPER;
  }
  if((mtrsMode >> (MotorNo*2)) & 1){
    return DC;
  }
  return SERVO;
}

//TODO
void MotorControl(int MotorCmd, unsigned int8 power) {
  int iterator = 0;
  for (iterator=0;iterator<MTR_COUNT;iterator++) {
    if ((mtrsActive >> iterator) & 1) {
      switch (MotorCmd) {
        case CMD_MOTORS_ANGLE:
          unsigned int16 waitTime = MTR_MAX_INTRPS-power;
          MotorAngle(iterator,power,waitTime);
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
          int runTime = power * 2;
          MotorPower(iterator,runTime,MTR_MAX_POWER-runTime);
          break;
      }
    }
  }
}

#int_timer1
void clock_and_button() { //10 ms
  set_timer1(T1_COUNTER);
  miliseconds += 10;
  if(miliseconds == 1000){
    seconds++;
    miliseconds = 0;
  }
  if(time_pressing_button > MIN_TIME_PRESSING_BUTTON){
    if(input(RUN_BUTTON)) {
      time_pressing_button = 0;
      button_pressed = 1;
    }
  }
  time_pressing_button += 10;
}

unsigned int16 readSensor(int sensorNo) {
  if (currentSensor != sensorNo) {
    set_adc_channel(sensorNo);
    delay_us(MIN_TIME_CHANGE_SENSOR);
    currentSensor=sensorNo;
   }
   return read_adc();
}


void init_board() {
  output_low(PIN_C0); // not used
  output_low(PIN_C1); // not used
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
  disable_interrupts(INT_RTCC);
  enable_interrupts(INT_TIMER1);
  enable_interrupts(GLOBAL);
   
  MotorOFF(M0);
  MotorOFF(M1);
  MotorOFF(M2);
  MotorOFF(M3);
  MotorThisWay(M0);
  MotorThisWay(M1);
  MotorThisWay(M2);
  MotorThisWay(M3);
}

void init_variables() {
  //motor
  int i = 0;
  for (i=0;i<MTR_COUNT;i++){
    mtrRun[i] = 10;
    mtrWait[i] = 10;
    mtrNextInterrupt[i] = 0;
  }
  intrp0_count = 0;
  intrp0Enb = 0;
  mtrsOnInterpt = 0x00001111; //off , timer0 control
  mtrsDirectionNextTurn = 0x11111111;
  mtrsActive = 0;
  mtrsMode = 0x01010101; //DC
  mtrChipEnable[0] = MTR_AB_ENB;
  mtrChipEnable[1] = MTR_CD_ENB;
  mtrEnable[M0] = M0_ENB;
  mtrEnable[M1] = M1_ENB;
  mtrEnable[M2] = M2_ENB;
  mtrEnable[M3] = M3_ENB;
  mtrS1[M0] = M0_S1;
  mtrS1[M1] = M1_S1;
  mtrS1[M2] = M2_S1;
  mtrS1[M3] = M3_S1;
  mtrS2[M0] = M0_S2;
  mtrS2[M1] = M1_S2;
  mtrS2[M2] = M2_S2;
  mtrS2[M3] = M3_S2;
  //time 
  seconds = 0;
  miliseconds = 0;
  //button
  time_pressing_button = 0;
  button_pressed = 0;
  //sensor
  currentSensor = 0; 
  //stacks
  stkPointer = 0;
  inputStkPointer =0;
  // usb
  usbBufferStart=0;
  usbBufferEnd=0;
  usbBufferSize=0;
  
  //usbReady = 0;
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
        mtrsActive = readUsbBuffer();
      }
      return;
    }
    if(command == CMD_MOTORS_ANGLE){
      verifyMotorsMode(SERVO);
      if(usbBufferSize>0){
        MotorControl(command, readUsbBuffer());
      }
      return;
    }
    if(command == CMD_MOTORS_POWER){
      verifyMotorsMode(DC);
      if(usbBufferSize>0){
        MotorControl(command, readUsbBuffer());
      }
      return;
    }
    verifyMotorsMode(DC);
    MotorControl(command,0);
  }
}

unsigned int8 readUsbBuffer() {
  if(usbBufferSize>0){
    unsigned int8 command = usbBuffer[usbBufferStart];
    usbBufferStart++;
    usbBufferStart %= USB_BUFFER_SIZE;
    usbBufferSize--;
    printf(usb_cdc_putc,"%c",command);
    return command;
  }
}

void updateUsbBuffer() {
  while(usb_cdc_kbhit() && (usbBufferSize < USB_BUFFER_SIZE)){
    usbBuffer[usbBufferEnd] = usb_cdc_getc();
    usbBufferEnd ++;
    usbBufferEnd %= USB_BUFFER_SIZE;
    usbBufferSize++;
    //usbReady = 1;
  }
}

void main(void){
  init_variables();
  init_board();
  
  usb_cdc_init();
  usb_init();
  
  for(;;){
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


