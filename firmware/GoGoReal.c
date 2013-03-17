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

#int_rtcc
void clock_isr() {
   int i;
   unsigned int minDuty;
   unsigned int nextDutyIndex;
   unsigned int periodTilNextInterrupt;
   ttTimer0++;
   if (ttTimer0 == 2){
   ttTimer0 =0;
   do {
      if (gblTimer0Counter < MotorCount) {
         if (getBit(gblMotorONOFF, gblCurrentDutyIndex) == ON) {
            if (gblMtrDuty[gblCurrentDutyIndex] < 255) {
               if (getBit(gblMotorMode, gblCurrentDutyIndex) == MOTOR_NORMAL) {
                  if (getBit(gblMotorDir, gblCurrentDutyIndex))
                     output_low(MotorCWPins[gblCurrentDutyIndex]);
                  else
                     output_low(MotorCCPins[gblCurrentDutyIndex]);
               } else
                  output_low(MotorCCPins[gblCurrentDutyIndex]);
            }
         }
      } else {
         for (i=0 ; i<MotorCount ; i++) {
            if (getBit(gblMotorONOFF, i) == ON) {
               if (gblMtrDuty[i] > 0) {
                  if (getBit(gblMotorMode, i) == MOTOR_NORMAL) {
                     if (getBit(gblMotorDir, i))
                        output_high(MotorCWPins[i]);
                     else
                        output_high(MotorCCPins[i]);
                  } else
                     output_high(MotorCCPins[i]);
               }
            }
         }
      }
      minDuty = 255;
      for (i=0;i<=MotorCount;i++) {
         if ((minDuty >= gblMtrDuty[i]) && !(getBit(gblDutyCycleFlag,i))) {
            minDuty = gblMtrDuty[i];
            nextDutyIndex = i;
         }
      }
      setBit(&gblDutyCycleFlag, nextDutyIndex);
      if (gblTimer0Counter < MotorCount)
         periodTilNextInterrupt = minDuty - gblMtrDuty[gblCurrentDutyIndex];
      else
         periodTilNextInterrupt = minDuty;
      gblCurrentDutyIndex = nextDutyIndex;
      if (gblTimer0Counter == MotorCount-1)
         gblDutyCycleFlag = 0;
      if (gblTimer0Counter < MotorCount)
         gblTimer0Counter++;
      else
         gblTimer0Counter = 0;
   } while ((periodTilNextInterrupt == 0) && (gblTimer0Counter > 0));


   if (gblTimer0Counter == 0) {
      gblSlowBurstModeTimerHasTicked=1;
   }
}
      set_rtcc(255-periodTilNextInterrupt);
}

#int_timer1
void timer1ISR() {
   gblTimer++;

   if (CMD_STATE != WAITING_FOR_FIRST_HEADER) {
      gblCmdTimeOut++;
   }

   if (gblWaitCounter > 0) {
      gblWaitCounter--;
   }

   if (input(RUN_BUTTON)) {
      if (!gblBtn1AlreadyPressed) {
         gblButtonPressed = !gblButtonPressed;
         gblBtn1AlreadyPressed=1;
         gblWaitCounter = 0;
         gblONFORNeedsToFinish = 0;
         if (!gblLogoIsRunning) {
            srand(gblTimer);
            output_high(RUN_LED);
            //leitura da regiao onde comecara o codigo logo
            gblMemPtr = (read_program_eeprom(RUN_BUTTON_BASE_ADDRESS)<<8)+read_program_eeprom(RUN_BUTTON_BASE_ADDRESS+2);
            gblMemPtr *= 2;
            clearStack();
            gblNewByteHasArrivedFlag=0;
            gblLogoIsRunning = 1;
         } else {
            gblLogoIsRunning = 0;
            output_low(RUN_LED);
         }
      }
   } else if (gblBtn1AlreadyPressed) {
      gblBtn1AlreadyPressed=0;
   }
   set_timer1(T1_COUNTER);
}

#int_timer2
void timer2ISR() {}

short getBit(int InByte, int BitNo) {
   return ((InByte >> BitNo) & 1);
}

void setBit(int *InByte, int BitNo) {
   *InByte |= (1<<BitNo);
}

void clearBit(int *InByte, int BitNo) {
   *InByte &= ~(1<<BitNo);
}

void MotorControl(int MotorCmd) {
   int i;
   for (i=0;i<MotorCount;i++) {
      if ((gblActiveMotors >> i) & 1) {
         SetMotorMode(MOTOR_NORMAL);
         switch (MotorCmd) {
            case MTR_ON:
               MotorON(i);
               break;
            case MTR_OFF:
               MotorOFF(i);
               break;
            case MTR_RD:
               MotorRD(i);
               break;
            case MTR_THISWAY:
               MotorThisWay(i);
               break;
            case MTR_THATWAY:
               MotorThatWay(i);
               break;
            case MTR_COAST:
               MotorCoast(i);
               break;
         }
      }
   }
}

void SetMotorPower(int Power) {
   int i;
   for (i=0;i<MotorCount;i++) {
      if ((gblActiveMotors >> i) & 1) {
         switch (Power) {
            case 1:
               Power = 36;
               break;
            case 2:
               Power = 73;
               break;
            case 3:
               Power = 110;
               break;
            case 4:
               Power = 146;
               break;
            case 5:
               Power = 183;
               break;
            case 6:
               Power = 219;
               break;
            case 7:
               Power = 255;
               break;
         }
         gblMtrDuty[i] = Power;
      }
   }
}

void ChangeMotorPower(int delta) {
   int i;
   for (i=0;i<MotorCount;i++) {
      if ((gblActiveMotors >> i) & 1) {
         gblMtrDuty[i] = gblMtrDuty[i] + delta;
      }
   }
}



void SetMotorMode(int motorMode) {
   int i;
   for (i=0;i<MotorCount;i++) {
      if ((gblActiveMotors >> i) & 1)
         if (motorMode == MOTOR_NORMAL)
            clearBit(&gblMotorMode, i);
         else
            setBit(&gblMotorMode, i);
   }
}

void ENHigh(int MotorNo) {
 //   int foo;
  //  foo = MotorNo - MotorNo%2;
   output_high(MotorENPins[MotorNo]);
   //if (foo == 0) {
      output_high(MOTOR_AB_EN);
   //} else {
      output_high(MOTOR_CD_EN);
   //}
}

void ENLow(int MotorNo) {
    int foo;
   output_low(MotorENPins[MotorNo]);
    if(MotorNo%2){
        foo=MotorNo-1;
    }else{
        foo=MotorNo+1;
    }
    if (!(getBit(gblMotorONOFF,foo))){
        return;
    }
   if (MotorNo<2){
      output_low(MOTOR_AB_EN);
        return;
   } else{
      output_low(MOTOR_CD_EN);
        return;
   }
}


void MotorON(int MotorNo) {
   int16 MtrCC, MtrCW;
   MtrCW = MotorCWPins[MotorNo];
   MtrCC = MotorCCPins[MotorNo];
   if (getBit(gblMotorDir,MotorNo)) {
      output_low(MtrCC);
      output_high(MtrCW);
   } else {
      output_high(MtrCC);
      output_low(MtrCW);
   }
   ENHigh(MotorNo);
   setBit(&gblMotorONOFF,MotorNo);
}

void MotorOFF(int MotorNo) {
   int16 MtrCC, MtrCW;
   MtrCW = MotorCWPins[MotorNo];
   MtrCC = MotorCCPins[MotorNo];
   output_high(MtrCC);
   output_high(MtrCW);
    output_low(MotorENPins[MotorNo]);
    ENLow(MotorNo);
   clearBit(&gblMotorONOFF,MotorNo);
}

void MotorRD(int MotorNo) {
   int16 MtrCC, MtrCW;
   MtrCW = MotorCWPins[MotorNo];
   MtrCC = MotorCCPins[MotorNo];
   if (getBit(gblMotorDir,MotorNo)){
        output_low(MtrCW);
      output_high(MtrCC);
        clearBit(&gblMotorDir,MotorNo);
   }else{
        output_high(MtrCW);
      output_low(MtrCC);
      setBit(&gblMotorDir,MotorNo);
   }
}

void MotorThisWay(int MotorNo) {
   int16 MtrCC, MtrCW;
   MtrCW = MotorCWPins[MotorNo];
   MtrCC = MotorCCPins[MotorNo];
   setBit(&gblMotorDir,MotorNo);
   output_low(MtrCC);
   output_high(MtrCW);
}


void MotorThatWay(int MotorNo) {
   int16 MtrCC, MtrCW;
   MtrCW = MotorCWPins[MotorNo];
   MtrCC = MotorCCPins[MotorNo];
   clearBit(&gblMotorDir,MotorNo);
   output_low(MtrCW);
   output_high(MtrCC);
}

void MotorCoast(int MotorNo) {
   int16 MtrCC, MtrCW;
   MtrCW = MotorCWPins[MotorNo];
   MtrCC = MotorCCPins[MotorNo];
   clearBit(&gblMotorONOFF,MotorNo);
   output_low(MtrCW);
   output_low(MtrCC);
}

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
         beep();
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
void beep() {
   set_pwm1_duty(50);
   delay_ms(50);
   set_pwm1_duty(0);
}

void DoSensorStuff() {
   long SensorVal;

   if (!gblSlowBurstMode || gblSlowBurstModeTimerHasTicked) {
      if ((gblBurstModeBits>>gblBurstModeCounter) & 1) {
         SensorVal=readSensor(gblBurstModeCounter);
         printf(usb_cdc_putc,"%c%c%c", 0x0c, (gblBurstModeCounter << 5) | (SensorVal >> 8), SensorVal & 0xff);
      }
      gblBurstModeCounter = (gblBurstModeCounter+1) % 8;
      gblSlowBurstModeTimerHasTicked = 0;
   }
}

void SetBurstMode(int SensorBits, int Mode) {
   gblBurstModeBits = SensorBits;
   if (Mode > 0)
      gblSlowBurstMode = 1;
   else
      gblSlowBurstMode = 0;
}

unsigned int16 readSensor(int sensorNo) {
   if (gblCurSensorChannel != sensorNo) {
      set_adc_channel(sensorNo);
	  delay_us(channelSwitchDelay);
      gblCurSensorChannel=sensorNo;
   }
   return read_adc();
}


char readUsbBuffer(char *charPtr) {
   int errorCode;

   if (gblUsbBufferIsFull == TRUE) {
      gblUsbBufferIsFull = FALSE;
      errorCode = USB_OVERFLOW;
      gblUsbBufferPutIndex = 0;
      gblUsbBufferGetIndex = 0;
      *charPtr = 0;
   } else if (gblUsbBufferGetIndex == gblUsbBufferPutIndex) {
      errorCode = USB_NO_DATA;
      *charPtr = 0;
   } else {
      *charPtr = gblUsbBuffer[gblUsbBufferGetIndex];
      gblUsbBufferGetIndex++;
      if (gblUsbBufferGetIndex >= USB_BUFFER_SIZE) {
         gblUsbBufferGetIndex = 0;
      }
      errorCode = USB_SUCCESS;
   }

   return(errorCode);
}


void init_variables() {
  gblBurstModeBits = 0;
  CMD_STATE = WAITING_FOR_FIRST_HEADER;
  gblLogoIsRunning=0;
  gblStkPtr=0;
  gblInputStkPtr=0;
  gblRecordPtr = read_program_eeprom(MEM_PTR_LOG_BASE_ADDRESS);
  MotorENPins[0] = MTR1_EN;
  MotorENPins[1] = MTR2_EN;
  MotorENPins[2] = MTR3_EN;
  MotorENPins[3] = MTR4_EN;
  
  MotorCWPins[0] = MTR1_CW;
  MotorCWPins[1] = MTR2_CW;
  MotorCWPins[2] = MTR3_CW;
  MotorCWPins[3] = MTR4_CW;
  
  MotorCCPins[0] = MTR1_CC;
  MotorCCPins[1] = MTR2_CC;
  MotorCCPins[2] = MTR3_CC;
  MotorCCPins[3] = MTR4_CC;
  CMD_STATE = 0;
  gbl_cur_cmd = 0;
  gbl_cur_param= 0;
  gbl_cur_ext= 0;
  gbl_cur_ext_byte= 0;
  gblBurstModeBits= 0;
  gblBurstModeCounter=0;
  gblSlowBurstMode=0;
  gblSlowBurstModeTimerHasTicked=0;
  gblCurSensorChannel = 0;
  gblMotorMode=0;
  gblActiveMotors= 0;
  gblMotorDir=0;
  gblMotorONOFF = 0;
  gblTimer0Counter = MotorCount;
  gblDutyCycleFlag = 0;
  gblCurrentDutyIndex = 0; 
  gblMostRecentlyReceivedByte = 0;
  gblNewByteHasArrivedFlag = 0;
  gblLogoIsRunning = 0;
  gblButtonPressed = 0;
  gblBtn1AlreadyPressed = 0;
  gblWaitCounter =0;
  gblTimer = 0;
  gblCmdTimeOut = 0;
  //gblUsbBuffer[USB_BUFFER_SIZE];
  gblUsbBufferPutIndex=0;
  gblUsbBufferGetIndex=0;
  gblUsbBufferIsFull=FALSE;
  HILOWHasArrivedFlag = 0;
  adressHILOW = 0;
  //gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
  gblFlashBufferPtr=0;
  gblFlashBaseAddress = 0;
  ttTimer0 = 0;
  
  gblMtrDuty[0] = 0xff;
  gblMtrDuty[1] = 0xff;
  gblMtrDuty[2] = 0xff;
  gblMtrDuty[3] = 0xff;
  gblMtrDuty[4] = 0xff;
}

void intro() {
   set_pwm1_duty(50);
   output_high(USER_LED);
   output_high(RUN_LED);
   delay_ms(50);
   set_pwm1_duty(0);
   delay_ms(50);
   output_low(USER_LED);
   output_low(RUN_LED);
   set_pwm1_duty(50);
   delay_ms(50);
   set_pwm1_duty(0);
   delay_ms(0);
   output_high(USER_LED);
   output_high(RUN_LED);
   delay_ms(100);
   output_low(USER_LED);
   output_low(RUN_LED);
}

void Halt() {
   while (1) {
      output_high(RUN_LED);
      delay_ms(50);
      output_low(RUN_LED);
      delay_ms(500);
   }
}



void initBoard() {
   int i,j;
   output_low(PIN_C0);
   output_low(PIN_C1);
   gblActiveMotors = 0;
   set_tris_a(PIC_TRIS_A);
   set_tris_b(PIC_TRIS_B);
   set_tris_c(PIC_TRIS_C);
   set_tris_d(PIC_TRIS_D);
   set_tris_e(PIC_TRIS_E);
   setup_adc_ports(AN0_TO_AN7);
   setup_adc(ADC_CLOCK_INTERNAL);
   gblCurSensorChannel=defaultPort;
   set_adc_channel(defaultPort);
   output_low(MOTOR_AB_EN);
   output_low(MOTOR_CD_EN);
   output_low(RUN_LED);
   output_low(USER_LED);
   for (i=0,j=0 ; i<MotorCount ; i++) {
      output_low(MotorENPins[i]);
      output_low(MotorCWPins[i]);
      output_low(MotorCCPins[i]);
   }

   setup_ccp1(CCP_PWM);
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
   setup_timer_2(T2_DIV_BY_16,250,16);
   enable_interrupts(GLOBAL);
   setup_timer_0(RTCC_INTERNAL| RTCC_DIV_256 | RTCC_8_BIT);
   set_rtcc(0);
   enable_interrupts(INT_RTCC);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(INT_TIMER2);
   enable_interrupts(INT_RDA);
   set_timer1(T1_COUNTER);
   intro();
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

   while ((buff_status = readUsbBuffer(&InByte)) == USB_SUCCESS) {
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
            if (InByte == InHeader2) CMD_STATE = WAITING_FOR_CMD_BYTE;
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
            }
            else{
              if(HILOWHasArrivedFlag == 1){
                adressHILOW = (int16) InByte;
                adressHILOW <<= 8;
                HILOWHasArrivedFlag = 2;
              }
              else{
                if(InByte ==128){
                  HILOWHasArrivedFlag = 1;
                  adressHILOW = 0;
                  flashBufferedWrite(InByte);
                  if (--gblRWCount < 1) {
                    flashFlushBuffer();
                    CMD_STATE = WAITING_FOR_FIRST_HEADER;
                  }
                }
                else{
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
      if (CMD_STATE == CMD_READY) break;
   }

   if (buff_status == USB_OVERFLOW) {
      CMD_STATE = WAITING_FOR_FIRST_HEADER;
   }
}


void ReadUsb() {

   while (usb_cdc_kbhit()) {
      if (gblUsbBufferIsFull == FALSE) {
         gblUsbBuffer[gblUsbBufferPutIndex] = usb_cdc_getc();
         gblUsbBufferPutIndex++;
         if (gblUsbBufferPutIndex >= USB_BUFFER_SIZE)
            gblUsbBufferPutIndex = 0;
         if (gblUsbBufferPutIndex == gblUsbBufferGetIndex)
            gblUsbBufferIsFull = TRUE;
      }
      delay_ms(5);
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


