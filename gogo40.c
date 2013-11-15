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

#include "bootloader.h"
#case
#include "stdlib.h"

#include <GOGO40.H>
#include "gogoreal.h"
#include "global_variables.h"
#include "evalOpcode.c"

#use fast_io(A)
#use fast_io(B)
#use fast_io(C)
#use fast_io(D)
#use fast_io(E)

void stkPush(unsigned int16 stackItem) {
    if (gblStkPtr<STACK_SIZE) {
        gblStack[gblStkPtr] = stackItem;
        gblStkPtr++;
    }
}

unsigned int16 stkPop() {
    if (gblStkPtr>0) {
        gblStkPtr--;
        return gblStack[gblStkPtr];
    }
    gblLogoIsRunning = 0; //Stop Logo Code
    return 0;
}

void inputPush(unsigned int16 stackItem) {
    if (gblInputStkPtr<INPUT_STACK_SIZE) {
        gblInputStack[gblInputStkPtr++] = stackItem;
    }
}

unsigned int16 inputPop(void) {
    if (gblInputStkPtr>0) {
        return(gblInputStack[--gblInputStkPtr]);
    }
    gblLogoIsRunning = 0; //Stop Logo Code
    return 0;
}

void sendBytes(unsigned int16 memPtr, unsigned int16 count) {
    while (count-- > 0){
        printf(usb_cdc_putc,"%c",read_program_eeprom(FLASH_USER_PROGRAM_BASE_ADDRESS + memPtr++));
    }
}

unsigned int16 fetchNextOpcode() {
    unsigned int16 opcode;

    // if an ONFOR command was launched we must turn motor off before
    // continuing. When gblONFORNeedsToFinish is falged, we simulate
    // a motor off command.
    if (gblONFORNeedsToFinish) {
        gblONFORNeedsToFinish = 0;
        return(M_OFF);
    } else {
        opcode = read_program_eeprom(FLASH_USER_PROGRAM_BASE_ADDRESS + gblMemPtr);
        gblMemPtr+=2;
    }

    return opcode;
}



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
        if (((gblMotorONOFF >> gblCurrentDutyIndex) & 1) == ON) {
          if (gblMtrDuty[gblCurrentDutyIndex] < 255) {
            if (((gblMotorMode >> gblCurrentDutyIndex) & 1) == MOTOR_NORMAL) {
              if (((gblMotorDir >> gblCurrentDutyIndex) & 1)){
                output_low(MotorCWPins[gblCurrentDutyIndex]);
              } else {
                output_low(MotorCCPins[gblCurrentDutyIndex]);
              }
            } else {
              output_low(MotorCCPins[gblCurrentDutyIndex]);
            }
          }
        }
      } else {
        for (i=0 ; i<MotorCount ; i++) {
          if (((gblMotorONOFF >> i) & 1) == ON) {
            if (gblMtrDuty[i] > 0) {
              if (((gblMotorMode >> i) & 1) == MOTOR_NORMAL) {
                if (((gblMotorDir >> i) & 1)){
                  output_high(MotorCWPins[i]);
                } else {
                  output_high(MotorCCPins[i]);
                }
              } else {
                output_high(MotorCCPins[i]);
              }
            }
          }
        }
      }
      minDuty = 255;
      for (i=0;i<=MotorCount;i++) {
        if ((minDuty >= gblMtrDuty[i]) && !(((gblDutyCycleFlag >> i) & 1))) {
          minDuty = gblMtrDuty[i];
          nextDutyIndex = i;
        }
      }
      gblDutyCycleFlag |= (1<<nextDutyIndex);
      if (gblTimer0Counter < MotorCount){
        periodTilNextInterrupt = minDuty - gblMtrDuty[gblCurrentDutyIndex];
      } else {
        periodTilNextInterrupt = minDuty;
      }
      gblCurrentDutyIndex = nextDutyIndex;
      if (gblTimer0Counter == MotorCount-1){
        gblDutyCycleFlag = 0;
      }
      if (gblTimer0Counter < MotorCount){
        gblTimer0Counter++;
      } else {
         gblTimer0Counter = 0;
      }
    } while ((periodTilNextInterrupt == 0) && (gblTimer0Counter > 0));
  }
  set_rtcc(255-periodTilNextInterrupt);
}

//Timer1
//Prescaler 1:8; TMR1 Preload = 28036; Actual Interrupt Time : 25 ms
#int_timer1
void timer1ISR() {
    set_timer1 (T1_COUNTER);
    gblTimer++;
    if (gblWaitCounter > 0) {
        gblWaitCounter--;
    }
    if (input (RUN_BUTTON)) {
        if(gblTimer - time_button_pressed > 15){ // 375 ms to wait a new press button
            time_button_pressed = gblTimer;
            start_stop_logo_machine = TRUE;
        }
    }
}

#int_timer2
void timer2ISR() {
}

void MotorControl(unsigned int8 MotorCmd) {
    int i;
    for (i = 0; i < MotorCount; i++) {
        if ((gblActiveMotors >> i) & 1) {
            SetMotorMode (MOTOR_NORMAL);
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

void SetMotorPower(unsigned int8 Power) {
    int i;
    for (i = 0; i < MotorCount; i++) {
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
    for (i = 0; i < MotorCount; i++) {
        if ((gblActiveMotors >> i) & 1) {
            gblMtrDuty[i] = gblMtrDuty[i] + delta;
        }
    }
}

void SetMotorMode(int motorMode) {
   int i;
   for (i=0;i<MotorCount;i++) {
      if ((gblActiveMotors >> i) & 1){
        if (motorMode == MOTOR_NORMAL){
           gblMotorMode &= ~(1<<i);
        } else {
          gblMotorMode |= (1<<i);
        }
      }
   }
}

void ENHigh(int MotorNo) {
   output_high(MotorENPins[MotorNo]);
   output_high(MOTOR_AB_EN);
   output_high(MOTOR_CD_EN);
}

void ENLow(int MotorNo) {
    int foo;
    output_low(MotorENPins[MotorNo]);
    if(MotorNo%2){
        foo=MotorNo-1;
    }else{
        foo=MotorNo+1;
    }
    if (!((gblMotorONOFF >> foo) & 1)){
        return;
    }
    if (MotorNo<2){
        output_low(MOTOR_AB_EN);
    } else {
        output_low(MOTOR_CD_EN);
    }
}

void MotorON(int MotorNo) {
   if ((gblMotorDir >> MotorNo) & 1) {
      output_low(MotorCCPins[MotorNo]);
      output_high(MotorCWPins[MotorNo]);
   } else {
      output_high(MotorCCPins[MotorNo]);
      output_low(MotorCWPins[MotorNo]);
   }
   ENHigh(MotorNo);
   gblMotorONOFF |= (1<<MotorNo);
}

void MotorOFF(int MotorNo) {
  output_high(MotorCCPins[MotorNo]);
  output_high(MotorCWPins[MotorNo]);
  output_low(MotorENPins[MotorNo]);
  ENLow(MotorNo);
  gblMotorONOFF &= ~(1<<MotorNo);
  
}

void MotorRD(int MotorNo) {
   if ((gblMotorDir >> MotorNo) & 1){
     MotorThatWay(MotorNo);
   } else {
     MotorThisWay(MotorNo);
   }
}

void MotorThisWay(int MotorNo) {
  output_high(MotorCWPins[MotorNo]);
  output_low(MotorCCPins[MotorNo]);  
  gblMotorDir |= (1<<MotorNo);
}

void MotorThatWay(int MotorNo) {
  output_low(MotorCWPins[MotorNo]);
  output_high(MotorCCPins[MotorNo]);
  gblMotorDir &= ~(1<<MotorNo);
}

void MotorCoast(int MotorNo) {
  output_low(MotorCWPins[MotorNo]);
  output_low(MotorCCPins[MotorNo]);
  gblMotorONOFF &= ~(1<<MotorNo);
}

void beep() {
    set_pwm1_duty(50);
    delay_ms(50);
    set_pwm1_duty(0);
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



void flashSetWordAddress(int16 address) {
    gblFlashBaseAddress = address;
    gblFlashBaseAddress &= ~(int32)((getenv("FLASH_ERASE_SIZE")) - 1);
    gblFlashBufferPtr = address - gblFlashBaseAddress;
    read_program_memory(gblFlashBaseAddress, gblFlashBuffer,getenv("FLASH_ERASE_SIZE"));
}

void flashBufferedWrite(unsigned int16 InByte) {
    gblFlashBuffer[gblFlashBufferPtr++] = (int) InByte;
    gblFlashBuffer[gblFlashBufferPtr++] = (int)(InByte>>8);
    if (!(gblFlashBufferPtr < (getenv("FLASH_ERASE_SIZE")))) {
        if(!(gblFlashBaseAddress <= FIRMWARE_END)) {
            erase_program_eeprom(gblFlashBaseAddress);
            write_program_memory(gblFlashBaseAddress, gblFlashBuffer, getenv("FLASH_ERASE_SIZE"));
            gblFlashBufferPtr = 0;
            gblFlashBaseAddress += getenv("FLASH_ERASE_SIZE");
            read_program_memory(gblFlashBaseAddress, gblFlashBuffer, getenv("FLASH_ERASE_SIZE"));
        }
    }
}

void flashFlushBuffer() {
    if (!(gblFlashBaseAddress <= FIRMWARE_END)) {
        erase_program_eeprom (gblFlashBaseAddress);
        write_program_memory(gblFlashBaseAddress, gblFlashBuffer,getenv("FLASH_ERASE_SIZE"));
    }
}

void flashWrite(int16 InByte) {
    gblFlashBuffer[gblFlashBufferPtr++] = (int) InByte;
    gblFlashBuffer[gblFlashBufferPtr++] = (int) (InByte >> 8);
    flashFlushBuffer();
}

unsigned int16 readSensor(int sensorNo) {
    if (gblCurSensorChannel != sensorNo) {
        set_adc_channel(sensorNo);
        gblCurSensorChannel=sensorNo;
        delay_us(channelSwitchDelay);
    }
    return read_adc();
}



void ProcessInput() {
    unsigned int8 InByte;
    while (usbBufferSize>0) {
        InByte = readUsbBuffer();
        gblMostRecentlyReceivedByte = InByte;
        gblNewByteHasArrivedFlag = 1;
        switch (InByte) {
                    case CMD_PING:
                        printf(usb_cdc_putc, "%c%c",BOARD_VERSION, FIRMWARE_VERSION);
                        break;
                    case CMD_READ_SENSOR:
                        InByte = readUsbBuffer();
                        unsigned int16 sensor_value = readSensor(InByte);
                        printf(usb_cdc_putc, "%c%c",sensor_value >> 8, sensor_value & 0xff);
                        break;
                    case SET_PTR:
                        InByte = readUsbBuffer();
                        gblMemPtr = (unsigned int16) InByte << 8;
                        InByte = readUsbBuffer();
                        gblMemPtr = gblMemPtr | InByte;
                        if ((gblMemPtr & 0xff0) == 0xff0) {
                            gblMemPtr = (RUN_BUTTON_BASE_ADDRESS + ((gblMemPtr & 0xf) * 2)) - FLASH_USER_PROGRAM_BASE_ADDRESS;
                        } else {
                            gblMemPtr *= 2;
                        }
                        flashSetWordAddress(FLASH_USER_PROGRAM_BASE_ADDRESS + gblMemPtr);
                        break;
                    case READ_BYTES:
                        InByte = readUsbBuffer();
                        gblRWCount = (unsigned int16) InByte << 8;
                        InByte = readUsbBuffer();
                        gblRWCount = gblRWCount | InByte;
                        sendBytes(gblMemPtr, gblRWCount);
                        gblMemPtr += gblRWCount;
                        break;
                    case WRITE_BYTES:
                        InByte = readUsbBuffer();
                        gblRWCount = (unsigned int16) InByte << 8;
                        InByte = readUsbBuffer();
                        gblRWCount = gblRWCount | InByte;
                        write_logo_code();
                        break;
                    case RUN:
                        start_stop_logo_machine = 1;
                        gblLogoIsRunning = 0;
                        break;
                    case CMD_BEEP:
                        beep();
                        break;
                    case CMD_LED_ON:
                        output_high(USER_LED);
                        break;
                    case CMD_LED_OFF:
                        output_low(USER_LED);
                        break;
                    case CMD_TALK_TO_MOTOR:
                        gblActiveMotors = readUsbBuffer();
                        break;
                    case CMD_MOTOR_CONTROL:
                        InByte = readUsbBuffer();
                        MotorControl(InByte);
                        break;
                    case CMD_MOTOR_POWER:
                        InByte = readUsbBuffer();
                        SetMotorPower(InByte);
                        break;
                    case MISC_SET_PWM:
                        InByte = readUsbBuffer();
                        MotorControl(MTR_THISWAY);
                        SetMotorMode(MOTOR_SERVO);
                        SetMotorPower(InByte);
                        MotorControl(MTR_ON);
                        break;
                    default:
                        break;
        };
    }
}

void write_logo_code(){
    unsigned int8 InByte;
    unsigned int16 adressHILOW = 0;
    while(gblRWCount>0){
        InByte = readUsbBuffer();
        if(InByte==128){
            flashBufferedWrite(InByte);
            adressHILOW = (unsigned int16) readUsbBuffer() << 8;
            adressHILOW += readUsbBuffer();
            flashBufferedWrite(adressHILOW);
            gblRWCount--;
        }
        else{
            flashBufferedWrite(InByte);
        }
        gblRWCount--;
    }
    flashFlushBuffer();
}

unsigned int8 readUsbBuffer() {
    if(usbBufferSize < 1){
        updateUsbBuffer();
    }
    if(usbBufferSize>0){
        unsigned int8 command = usbBuffer[usbBufferStart];
        usbBufferStart++;
        usbBufferStart %= USB_BUFFER_SIZE;
        usbBufferSize--;
        return command;
    }
}

void updateUsbBuffer() {
    unsigned int8 last_byte;
    while(usb_cdc_kbhit() && (usbBufferSize < USB_BUFFER_SIZE)){
        last_byte = usb_cdc_getc();
        usbBuffer[usbBufferEnd] = last_byte;
        usbBufferEnd ++;
        usbBufferEnd %= USB_BUFFER_SIZE;
        usbBufferSize++;
        printf(usb_cdc_putc,"%c",last_byte);
        delay_ms(5);
    }
}

void init_variables() {
    usbBufferStart = 0;
    usbBufferEnd = 0;
    usbBufferSize = 0;
    
    
    gblLogoIsRunning = 0;
    time_button_pressed = 0; // last time that run button was pressed 
    start_stop_logo_machine = FALSE;
    gblWaitCounter =0; // wait to execute logo code
    gblCurSensorChannel = 0;
    gblMotorMode=0b00000000;
    gblActiveMotors= 0;
    gblMotorDir=0;
    gblMotorONOFF = 0;
    gblTimer0Counter = MotorCount;
    gblDutyCycleFlag = 0;
    gblCurrentDutyIndex = 0;
    gblMostRecentlyReceivedByte = 0;
    gblNewByteHasArrivedFlag = 0;
    gblLogoIsRunning = 0;
    gblWaitCounter = 0;
    gblTimer = 0;
    HILOWHasArrivedFlag = 0;
    gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
    gblFlashBufferPtr=0;
    gblFlashBaseAddress = 0;
    ttTimer0 = 0; 
    gblStkPtr = 0;
    gblInputStkPtr = 0;
    gblInputStack[INPUT_STACK_SIZE];
    gblRecordPtr = read_program_eeprom(MEM_PTR_LOG_BASE_ADDRESS);
    gblMemPtr = 0;
    gblRWCount = 0;
    gblLoopAddress=0;
    gblRepeatCount=0;
    gblONFORNeedsToFinish=0;
    gblCurSensorChannel = defaultPort;
    
    int i;
    for (i = 0; i < MotorCount+1; i++) {
        gblMtrDuty[i] = 0xff;
    }
}

void initBoard() {
    init_variables();
    int i;
    set_tris_a (PIC_TRIS_A);
    set_tris_b (PIC_TRIS_B);
    set_tris_c (PIC_TRIS_C);
    set_tris_d (PIC_TRIS_D);
    set_tris_e (PIC_TRIS_E);
    setup_port_a(AN0_TO_AN7);
    setup_adc (ADC_CLOCK_INTERNAL);
    set_adc_channel(defaultPort);
    output_low(MOTOR_AB_EN);
    output_low(MOTOR_CD_EN);
    output_low(RUN_LED);
    output_low(USER_LED);
    output_low(PIN_C0);
    output_low(PIN_C1);
    for (i = 0; i < MotorCount; i++) {
        output_low (MotorENPins[i]);
        output_low (MotorCWPins[i]);
        output_low (MotorCCPins[i]);
    }
    setup_ccp1(CCP_PWM);
    setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
    setup_timer_2(T2_DIV_BY_16, 250, 16);
    setup_timer_0(RTCC_INTERNAL | RTCC_DIV_256 | RTCC_8_BIT);
    enable_interrupts (INT_RTCC);
    enable_interrupts (INT_TIMER1);
    enable_interrupts (INT_TIMER2);
    set_rtcc(0);
    set_timer1(T1_COUNTER);
    enable_interrupts(GLOBAL);
    intro();
}

void main() {
    initBoard();
    int16 uploadLen, counter;
    int16 foo;

    usb_cdc_init();
    usb_init();
    while (1) {
        updateUsbBuffer();
        ProcessInput();
        if(start_stop_logo_machine){
            if (gblLogoIsRunning){
                gblLogoIsRunning = 0;
                output_low (RUN_LED);
            }else{
                srand (gblTimer);
                //leitura da regiao onde começara o codigo logo
                gblMemPtr = (read_program_eeprom(RUN_BUTTON_BASE_ADDRESS) << 8) + read_program_eeprom(RUN_BUTTON_BASE_ADDRESS + 2);
                gblMemPtr *= 2;
                gblStkPtr = 0;
                gblInputStkPtr = 0;
                gblNewByteHasArrivedFlag = 0;
                gblLogoIsRunning = 1;
                output_high(RUN_LED);
            }
            gblWaitCounter = 0;
            gblONFORNeedsToFinish = 0;
            start_stop_logo_machine = FALSE;
        }
        if (gblLogoIsRunning) {
            if (!gblWaitCounter){
                evalOpcode(fetchNextOpcode());
            }
        }
    }
}


