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

void SetMotorMode(int MotorNo, int motorMode) {
  if(motorMode == servo_mode){ // 00
    motorsMode &= ~(1<<(MotorNo*2));
    motorsMode &= ~(1<<((MotorNo*2)+1));
    return;
  }
  if(motorMode == dc_mode){ // 01
    motorsMode |= (1<<(MotorNo*2));
    motorsMode &= ~(1<<((MotorNo*2)+1));
    return;
  }
  if(motorMode == stepper_mode){ //10
    motorsMode &= ~(1<<(MotorNo*2));
    motorsMode |= (1<<((MotorNo*2)+1));
    return;
  }
}

int motorMode(int MotorNo) {
  if((motorsMode >> ((MotorNo*2)+1)) & 1){
    return stepper_mode;
  }
  if((motorsMode >> (MotorNo*2)) & 1){
    return dc_mode;
  }
  return servo_mode;
}

void motorPower(int MotorNo, int power) {
  if (motorMode(MotorNo) == dc_mode){
    power *= 4;
  }
  if (motorMode(MotorNo) == servo_mode){
    if(power > 40){
      return;
    }
  }
  motor_intrp_count[MotorNo] = power;
}

//TODO
void MotorControl(int MotorCmd) {
   for (iterator=0;iterator<MotorCount;iterator++) {
      if ((motorsActive >> iterator) & 1) {
         switch (MotorCmd) {
            case MTR_ON:
               MotorON(iterator);
               break;
            case MTR_COAST:
            case MTR_OFF:
               MotorOFF(iterator);
               break;
            case MTR_RD:
               MotorRD(iterator);
               break;
            case MTR_THISWAY:
               MotorThisWay(iterator);
               break;
            case MTR_THATWAY:
               MotorThatWay(iterator);
               break;
         }
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

   set_tris_a(PIC_TRIS_A);
   set_tris_b(PIC_TRIS_B);
   set_tris_c(PIC_TRIS_C);
   set_tris_d(PIC_TRIS_D);
   set_tris_e(PIC_TRIS_E);
   
   setup_adc_ports(AN0_TO_AN7);
   setup_adc(ADC_CLOCK_INTERNAL);
   set_adc_channel(0);

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
  unsigned int8 command;
  if(usbBufferSize>0){
    readUsbBuffer(&command);
    printf(usb_cdc_putc,"%c", command);
  }
}

unsigned int8 readUsbBuffer(unsigned int8 *charPtr) {
   if (usbBufferSize == 0) {
      //*charPtr = 0;
      return USB_NO_DATA;
   }
   *charPtr = usbBuffer[usbBufferStart];
   usbBufferStart++;
   usbBufferStart %= usb_buffer_max_size;
   usbBufferSize--;
   return USB_SUCCESS;
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
######################################################################


void miscControl(int cur_param, int cur_ext, int cur_ext_byte) {
   switch (cur_param) {
      case MISC_USER_LED:
         if (cur_ext == TURN_USER_LED_ON) {
            output_high(USER_LED);
         } else {
            output_low(USER_LED);
         }
         break;
      case MISC_BEEP:
         play_music();
         break;
      case MISC_SET_PWM:
         MotorControl(MTR_ON);
         MotorControl(MTR_THISWAY);
         SetMotorMode(MOTOR_SERVO);
         SetMotorPower(cur_ext_byte);
         break;
      case MISC_UPLOAD_EEPROM:
         break;
      case MISC_I2C_SETUP:
         switch (cur_ext) {
            case I2C_START:
               i2c_start();
               break;
            case I2C_STOP:
               i2c_stop();
               break;
            case I2C_WRITE:
               i2c_write(cur_ext_byte);
               break;
            case I2C_READ:
               i2c_read(0);
               break;
         }
         break;
   }
}

void flashSetWordAddress(int16 address) {

   gblFlashBaseAddress = address;
   gblFlashBaseAddress &= ~(int32)((getenv("FLASH_ERASE_SIZE"))-1);

   gblFlashBufferPtr = address - gblFlashBaseAddress;
   read_program_memory(gblFlashBaseAddress, gblFlashBuffer, getenv("FLASH_ERASE_SIZE"));
}

void flashBufferedWrite(unsigned int16 InByte) {
   gblFlashBuffer[gblFlashBufferPtr++] = (int) InByte;
   gblFlashBuffer[gblFlashBufferPtr++] = (int)(InByte>>8);

   if (!(gblFlashBufferPtr < (getenv("FLASH_ERASE_SIZE")))) {
   if(!(gblFlashBaseAddress <= FIRMWARE_END)){
      erase_program_eeprom(gblFlashBaseAddress);
      write_program_memory(gblFlashBaseAddress, gblFlashBuffer, getenv("FLASH_ERASE_SIZE"));
      gblFlashBufferPtr = 0;
      gblFlashBaseAddress += getenv("FLASH_ERASE_SIZE");
      read_program_memory(gblFlashBaseAddress, gblFlashBuffer, getenv("FLASH_ERASE_SIZE"));
    }
   }
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

void ProcessInput() {
  int InByte, buff_status;
  int1 doNotStopRunningProcedure;
   buff_status = readUsbBuffer(&InByte);
  while (buff_status == USB_SUCCESS) {
    gblCmdTimeOut = 0 ;
    gblMostRecentlyReceivedByte = InByte;
    gblNewByteHasArrivedFlag = 1;
     printf(usb_cdc_putc, "%c",InByte);
    switch (CMD_STATE) {
      case WAITING_FOR_FIRST_HEADER:
        switch (InByte) {
          case InHeader1:
            CMD_STATE = WAITING_FOR_SECOND_HEADER;
            doNotStopRunningProcedure = 1;
            break;
          case SET_PTR:
            CMD_STATE = SET_PTR_HI_BYTE;
            break;
          case READ_BYTES:
            CMD_STATE = READ_BYTES_COUNT_HI;
            break;
          case WRITE_BYTES:
            CMD_STATE = WRITE_BYTES_COUNT_HI;
            break;
          case RUN:
            doNotStopRunningProcedure = 1;
            output_high(RUN_LED);
            gblWaitCounter = 0;
            gblONFORNeedsToFinish = 0;
            clearStack();
            gblNewByteHasArrivedFlag = 0;
            gblLogoIsRunning = 1;
            break;
          case CRICKET_CHECK:
            CMD_STATE = CRICKET_NAME;
            break;
        };
        if (!doNotStopRunningProcedure) {
          gblLogoIsRunning = 0;
          gblWaitCounter = 0;
          output_low(RUN_LED);
          gblBurstModeBits = 0;
        }
        doNotStopRunningProcedure=0;
        break;
      case WAITING_FOR_SECOND_HEADER:
        if (InByte == InHeader2){
              CMD_STATE = WAITING_FOR_CMD_BYTE;
            }
        break;
      case WAITING_FOR_CMD_BYTE:
        gbl_cur_cmd =(InByte & 0b11100000) >> 5;
        gbl_cur_param =(InByte & 0b00011100) >> 2;
        gbl_cur_ext =(InByte & 0b00000011);
        if (gbl_cur_cmd > ONE_BYTE_CMD) {
          CMD_STATE = WAITING_FOR_SECOND_CMD_BYTE;
        } else {
          CMD_STATE = CMD_READY;
        }
        break;
      case WAITING_FOR_SECOND_CMD_BYTE:
        gbl_cur_ext_byte = InByte;
        CMD_STATE = CMD_READY;
        break;
      case CMD_READY:
        break;
      case SET_PTR_HI_BYTE:
        gblMemPtr = (unsigned int16) InByte << 8;
        CMD_STATE = SET_PTR_LOW_BYTE;
        break;
      case SET_PTR_LOW_BYTE:
        gblMemPtr = gblMemPtr | InByte;
        CMD_STATE = WAITING_FOR_FIRST_HEADER;
        if ((gblMemPtr&0xff0) == 0xff0) {
          gblMemPtr = (RUN_BUTTON_BASE_ADDRESS + ((gblMemPtr&0xf)*2)) - FLASH_USER_PROGRAM_BASE_ADDRESS ;
          set_pwm1_duty(50);
        } else {
          gblMemPtr *= 2;
        }
        flashSetWordAddress(FLASH_USER_PROGRAM_BASE_ADDRESS + gblMemPtr);
        break;
      case READ_BYTES_COUNT_HI:
        gblRWCount = (unsigned int16) InByte << 8;
        CMD_STATE = READ_BYTES_COUNT_LOW;
        break;
      case READ_BYTES_COUNT_LOW:
        gblRWCount = gblRWCount | InByte;
        sendBytes(gblMemPtr, gblRWCount);
        gblMemPtr += gblRWCount;
        CMD_STATE = WAITING_FOR_FIRST_HEADER;
        break;
      case WRITE_BYTES_COUNT_HI:
        gblRWCount = (unsigned int16) InByte << 8;
        CMD_STATE = WRITE_BYTES_COUNT_LOW;
        break;
      case WRITE_BYTES_COUNT_LOW:
        gblRWCount = gblRWCount | InByte;
        CMD_STATE = WRITE_BYTES_SENDING;
        break;
      case WRITE_BYTES_SENDING:
        set_pwm1_duty(0);
        if(HILOWHasArrivedFlag == 2){
          adressHILOW += InByte;
          HILOWHasArrivedFlag = 0;
          flashBufferedWrite(adressHILOW);
          if (--gblRWCount < 1) {
            flashFlushBuffer();
            CMD_STATE = WAITING_FOR_FIRST_HEADER;
          }
        } else {
          if(HILOWHasArrivedFlag == 1){
            adressHILOW = (int16) InByte;
            adressHILOW <<= 8;
            HILOWHasArrivedFlag = 2;
          } else {
            if(InByte ==128){
              HILOWHasArrivedFlag = 1;
              adressHILOW = 0;
              flashBufferedWrite(InByte);
              if (--gblRWCount < 1) {
                flashFlushBuffer();
                CMD_STATE = WAITING_FOR_FIRST_HEADER;
                }
            } else{
              flashBufferedWrite(InByte);
              if (--gblRWCount < 1) {
                flashFlushBuffer();
                CMD_STATE = WAITING_FOR_FIRST_HEADER;
              }
            }
          }
        }
        printf(usb_cdc_putc,"%c",255-InByte);
        break;
      case CRICKET_NAME:
        printf(usb_cdc_putc,"%c",0x37);
        CMD_STATE = WAITING_FOR_FIRST_HEADER;
        break;
      default:
        CMD_STATE = WAITING_FOR_FIRST_HEADER;
        break;
    }
    if (CMD_STATE == CMD_READY){
        break;
    }
      buff_status = readUsbBuffer(&InByte);
  }
   if (buff_status == USB_OVERFLOW) {
     CMD_STATE = WAITING_FOR_FIRST_HEADER;
  }
}



void main() {
   int16 SensorVal;
   int16 uploadLen,counter;
   int16 foo;

   init_variables();
   initBoard();

   usb_cdc_init();
   usb_init();
   usb_task();

   while (1) {
      ReadUsb();
      ProcessInput();
      if (CMD_STATE == CMD_READY) {
         switch (gbl_cur_cmd) {
            case CMD_PING:
              printf(usb_cdc_putc,"%c%c%c", ReplyHeader1, ReplyHeader2, ACK_BYTE);
              printf(usb_cdc_putc,"%c%c%c", 0x01, 0x30, 0x01);
              break;
            case CMD_READ_SENSOR:
              SensorVal = readSensor(gbl_cur_param);
              printf(usb_cdc_putc,"%c%c%c%c",ReplyHeader1, ReplyHeader2, SensorVal >> 8, SensorVal & 0xff);
              break;
            case CMD_MOTOR_CONTROL:
              MotorControl(gbl_cur_param);
              printf(usb_cdc_putc,"%c%c%c", ReplyHeader1, ReplyHeader2, ACK_BYTE);
              break;
            case CMD_MOTOR_POWER:
              SetMotorPower(gbl_cur_param);
              printf(usb_cdc_putc,"%c%c%c", ReplyHeader1, ReplyHeader2, ACK_BYTE);
              break;
            case CMD_TALK_TO_MOTOR:
              gblActiveMotors = gbl_cur_ext_byte;
              printf(usb_cdc_putc,"%c%c%c", ReplyHeader1, ReplyHeader2, ACK_BYTE);
              break;
            case CMD_BURST_MODE:
              SetBurstMode(gbl_cur_ext_byte, gbl_cur_ext);
              printf(usb_cdc_putc,"%c%c%c", ReplyHeader1, ReplyHeader2, ACK_BYTE);
              break;
            case CMD_MISC_CONTROL:
              miscControl(gbl_cur_param, gbl_cur_ext, gbl_cur_ext_byte);
              printf(usb_cdc_putc,"%c%c%c", ReplyHeader1, ReplyHeader2, ACK_BYTE);
              break;
         }
        if ((gbl_cur_cmd == CMD_MISC_CONTROL) && (gbl_cur_param == MISC_UPLOAD_EEPROM)) {
            uploadLen = ((((int16)gbl_cur_ext<<8) + gbl_cur_ext_byte)<<5);

            if (uploadLen == 0)
               uploadLen = read_program_eeprom(MEM_PTR_LOG_BASE_ADDRESS);

            uploadLen<<=1;
            printf(usb_cdc_putc, "%c%c%c%c", EEPROMuploadHeader1, EEPROMuploadHeader2, uploadLen & 0xff, uploadLen >> 8);
            uploadLen>>=1;
            for (counter = 0 ; counter < uploadLen ; counter++) {
               foo = read_program_eeprom((RECORD_BASE_ADDRESS + counter));
               printf(usb_cdc_putc, "%c%c", foo&0xff, foo>>8);
            }
         }
         CMD_STATE = WAITING_FOR_FIRST_HEADER;
      } else {
         DoSensorStuff();
      }

      if (gblLogoIsRunning) {
         if (!gblWaitCounter)
            evalOpcode(fetchNextOpcode());
      }

      if (gblCmdTimeOut > CMD_TIMEOUT_PERIOD) {
         CMD_STATE = WAITING_FOR_FIRST_HEADER;
         gblCmdTimeOut = 0;
      }

   }
}
*/
