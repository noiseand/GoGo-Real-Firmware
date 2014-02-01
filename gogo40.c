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
    unsigned int16 opcode = read_program_eeprom(FLASH_USER_PROGRAM_BASE_ADDRESS + gblMemPtr);
    gblMemPtr+=2;
    return opcode;
}

void enableTimer0(){
    intrp0_count = 0;
    intrp0Enabled = 1;
    set_rtcc(T0_COUNTER);
    enable_interrupts(INT_RTCC);
}

void tryDisableTimer0(){
    if(((mtrsOnInterpt >> M0) & 1) ){return;}//controled by timer0 ??
    if(((mtrsOnInterpt >> M1) & 1) ){return;}//controled by timer0 ??
    if(((mtrsOnInterpt >> M2) & 1) ){return;}//controled by timer0 ??
    if(((mtrsOnInterpt >> M3) & 1) ){return;}//controled by timer0 ??
    intrp0Enabled=0;
    disable_interrupts(INT_RTCC);
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

unsigned int8 motorMode(int MotorNo) {
    if((mtrsMode >> ((MotorNo*2)+1)) & 1){
        return STEPPER;
    }
    if((mtrsMode >> (MotorNo*2)) & 1){
        return DC;
    }
    return SERVO;
}

void setNextInterupt(unsigned int8 MotorNo){
    mtrNextInterrupt[MotorNo] = intrp0_count - (intrp0_count % MTR_DC_MAX_POWER);
    mtrNextInterrupt[MotorNo]+= mtrRun[MotorNo];
    if(mtrNextInterrupt[MotorNo] > intrp0_count){
        mtrsDirectionNextTurn &= ~(1<<MotorNo);
        return;
    }
    mtrsDirectionNextTurn |= (1<<MotorNo);
    mtrNextInterrupt[MotorNo] += mtrWait[MotorNo];
}


#int_rtcc
void motor_ControlV2() {
    int8 mtrIterator = 0;
    for(mtrIterator = 0; mtrIterator < MTR_COUNT;mtrIterator++){
        if((mtrsOnInterpt >> mtrIterator) & 1){//controled by timer0 ??
            if(intrp0_count == mtrNextInterrupt[mtrIterator]){
                if((mtrsDirectionNextTurn >> (mtrIterator)) & 1){
                    mtrNextInterrupt[mtrIterator] += mtrRun[mtrIterator];
                    output_high(mtrPwmPin[mtrIterator]);
                } else{
                    mtrNextInterrupt[mtrIterator] += mtrWait[mtrIterator];
                    output_low(mtrPwmPin[mtrIterator]);
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

void MotorON(unsigned int8 MotorNo) {
    mtrsOnInterpt |= (1<<(MotorNo+MTR_COUNT)); //set on
    unsigned int8 mode = motorMode(MotorNo);
    if(mode == SERVO){ // 00
        mtrNextInterrupt[MotorNo] = 0;
        output_high(mtrEnable[MotorNo]);
        output_high(mtrS2[MotorNo]);
        output_low(mtrS1[MotorNo]);// pwm
        enableTimer0();
    }
    if(mode == DC){ // 01
        if((mtrsDirectionNextTurn >> (MotorNo+MTR_COUNT)) & 1){
            MotorThisWay(MotorNo);
        }else{
            MotorThatWay(MotorNo);
        }
        if((mtrsOnInterpt >> MotorNo) & 1){//controled by timer0 ??
            if(intrp0Enabled == 1) {
                setNextInterupt(MotorNo);
            }else{
                mtrNextInterrupt[MotorNo] = 0;
                enableTimer0();
            }
        } else {
            output_high(mtrEnable[MotorNo]);
        }
    }
    if(mode == STEPPER){ //10

    }
    output_high(mtrChipEnable[MotorNo/2]);
}

void MotorOFF(unsigned int8 MotorNo) {
    mtrsOnInterpt &= ~(1<<MotorNo); //not controled by timer0
    mtrsOnInterpt &= ~(1<<(MotorNo+MTR_COUNT)); // set off
    tryDisableTimer0();
    tryDisableChip();
    output_low(mtrEnable[MotorNo]);
    output_low(mtrS1[MotorNo]);
    output_low(mtrS2[MotorNo]);
    mtrsDirectionNextTurn |= (1<<MotorNo); // set turnOn on next time
    mtrNextInterrupt[MotorNo] = 0;
}

void MotorRD(unsigned int8 MotorNo) {
    if(motorMode(MotorNo) != DC){return;}
    if((mtrsDirectionNextTurn >> (MotorNo+MTR_COUNT)) & 1){
        MotorThatWay(MotorNo);
    }else{
        MotorThisWay(MotorNo);
    }
}

void MotorThisWay(unsigned int8 MotorNo) {
    if(motorMode(MotorNo) != DC){return;}
    mtrsDirectionNextTurn |= (1<<(MotorNo+MTR_COUNT));
    if((mtrsOnInterpt >> (MotorNo+MTR_COUNT)) & 1){
        output_high(mtrS1[MotorNo]);
        output_low(mtrS2[MotorNo]);
    }
}

void MotorThatWay(unsigned int8 MotorNo) {
    if(motorMode(MotorNo) != DC){return;}
    mtrsDirectionNextTurn &= ~(1<<(MotorNo+MTR_COUNT));
    if((mtrsOnInterpt >> (MotorNo+MTR_COUNT)) & 1){
        output_low(mtrS1[MotorNo]);
        output_high(mtrS2[MotorNo]);
    }
}

void configureMotorMode(unsigned int8 MotorNo, unsigned int8 mode) {
    if(mode == SERVO){ // 00
        mtrsOnInterpt |= (1<<MotorNo); // set controll by timer0
        mtrPwmPin[MotorNo] = mtrS1[MotorNo];//where pwm
        mtrsMode &= ~(1<<(MotorNo*2));
        mtrsMode &= ~(1<<((MotorNo*2)+1));
        return;
    }
    if(mode == DC){ // 01
        mtrPwmPin[MotorNo] = mtrEnable[MotorNo];//where pwm
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

void MotorAngle(unsigned int8 MotorNo, unsigned int8 angle){
    if(motorMode(MotorNo) != SERVO){return;}
    unsigned int16 waitTime = (unsigned int16) MTR_MAX_INTRPS-angle;
    if((mtrsOnInterpt >> (MotorNo+MTR_COUNT)) & 1){
        while(intrp0_count == mtrRun[MotorNo]);
    }else{
        mtrsDirectionNextTurn |= (1<<MotorNo); // set turnOn on next time
        mtrNextInterrupt[MotorNo] = 0;
    }
    mtrRun[MotorNo] = angle;
    mtrWait[MotorNo] = waitTime;
}

void MotorPower(unsigned int8 MotorNo, unsigned int8 power){
    if(motorMode(MotorNo) != DC){return;}
    unsigned int16 waitTime = (unsigned int16) MTR_DC_MAX_POWER-power;
    mtrsOnInterpt &= ~(1<<MotorNo);//give me time, so be not controled by timer0
    mtrRun[MotorNo] = power;
    mtrWait[MotorNo] = waitTime;
    if(waitTime == 0){ // full power
        //already not controled by timer0
        tryDisableTimer0();
        if((mtrsOnInterpt >> (MotorNo+MTR_COUNT)) & 1){
            output_high(mtrPwmPin[MotorNo]);
        }
    }else{
        if((mtrsOnInterpt >> (MotorNo+MTR_COUNT)) & 1){
            if(intrp0Enabled==1){
                setNextInterupt(MotorNo);
            }else{
                mtrsDirectionNextTurn |= (1<<MotorNo); // set turnOn on next time
                mtrNextInterrupt[MotorNo] = 0;
                enableTimer0();
            }
        }
        mtrsOnInterpt |= (1<<MotorNo); // set controll by timer0
    }
}

void MotorControl(int MotorCmd, unsigned int8 param) {
    int MotorNo = 0;
    for (MotorNo=0;MotorNo<MTR_COUNT;MotorNo++) {
        if ((mtrsActive >> MotorNo) & 1) {
            switch (MotorCmd) {
                case OPC_MOTORS_ANGLE:
                    MotorAngle(MotorNo,param);
                    break;
                case OPC_MOTORS_ON:
                    MotorON(MotorNo);
                    break;
                case OPC_MOTORS_OFF:
                    MotorOFF(MotorNo);
                    break;
                case OPC_MOTORS_REVERT:
                    MotorRD(MotorNo);
                    break;
                case OPC_MOTORS_THISWAY:
                    MotorThisWay(MotorNo);
                    break;
                case OPC_MOTORS_THATWAY:
                    MotorThatWay(MotorNo);
                    break;
                case OPC_MOTORS_POWER:
                    MotorPower(MotorNo,param);// 1 - 10 
                    break;
                case OPC_MOTORS_CONFIG:
                    configureMotorMode(MotorNo,param);
                    break;
            }
        }
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

void set_on_for(unsigned int16 delay) {
    int i;
    for (i = 0; i < MTR_COUNT; i++) {
        if ((mtrsActive >> i) & 1) {
            motor_onfor[i] = gblTimer + delay;
            motor_onfor_needs_to_finish[i] = 1;
        }
    }
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
                start_stop_logo_machine = 1;
                gblLogoIsRunning = 1;
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
            case OPC_ACTIVATE_MOTORS:
                mtrsActive = readUsbBuffer();
                break;
            //case CMD_MOTOR_CONTROL:
            //    InByte = readUsbBuffer();
            //    MotorControl(InByte,0);
            //    break;
            case CMD_MOTORS_POWER:
                InByte = readUsbBuffer();
                MotorControl(CMD_MOTORS_POWER,InByte);
                break;
            case CMD_MOTORS_ANGLE:
                InByte = readUsbBuffer();
                MotorControl(OPC_MOTORS_CONFIG,STEPPER);
                MotorControl(OPC_MOTORS_ANGLE,InByte);
                MotorControl(OPC_MOTORS_ON,0);
                break;
            case CMD_MODE:
                printf(usb_cdc_putc, "%c",70);//F
                break;
            case CMD_BOOTLOADER:
                disable_interrupts(GLOBAL);
                write_eeprom(EEPROM_FLAG_ADDR,EEPROM_FLAG_CODE);
                #asm
                reset 
                #endasm 
            case CMD_LOGO_TURN_ON:
                write_eeprom(LOGO_TURN_ON_ADDR,LOGO_TURN_ON_FLAG);
                break;
            case CMD_NOT_LOGO_TURN_ON:
                write_eeprom(LOGO_TURN_ON_ADDR,0);
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
    mtrsActive= 0;
    gblMostRecentlyReceivedByte = 0;
    gblNewByteHasArrivedFlag = 0;
    gblLogoIsRunning = 0;
    gblWaitCounter = 0;
    gblTimer = 0;
    beep_is_high = 0;
    gblFlashBuffer[getenv("FLASH_ERASE_SIZE")];
    gblFlashBufferPtr=0;
    gblFlashBaseAddress = 0;
    gblStkPtr = 0;
    gblInputStkPtr = 0;
    gblInputStack[INPUT_STACK_SIZE];
    gblRecordPtr = read_program_eeprom(MEM_PTR_LOG_BASE_ADDRESS);
    gblMemPtr = 0;
    gblRWCount = 0;
    gblLoopAddress=0;
    gblRepeatCount=0;
    gblCurSensorChannel = defaultPort;
}

void initBoard() {

    int i;
    set_tris_a (PIC_TRIS_A);
    set_tris_b (PIC_TRIS_B);
    set_tris_c (PIC_TRIS_C);
    set_tris_d (PIC_TRIS_D);
    set_tris_e (PIC_TRIS_E);
    setup_port_a(AN0_TO_AN7);
    setup_adc (ADC_CLOCK_INTERNAL);
    set_adc_channel(defaultPort);
    output_low(RUN_LED);
    output_low(USER_LED);
    output_low(PIN_C0);
    output_low(PIN_C1);
    output_low(PIN_C2);//beep

    setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
    setup_timer_0(RTCC_INTERNAL | T0_DIV_1); // 50us
    disable_interrupts(INT_RTCC);
    enable_interrupts(INT_TIMER1);
    set_rtcc(T0_COUNTER);
    set_timer1(T1_COUNTER);
    enable_interrupts(GLOBAL);
    
    unsigned int8 motorNo = 0;
    for(motorNo=0;motorNo<MTR_COUNT;motorNo++){
        configureMotorMode(MotorNo, DC);
        MotorOFF(motorNo);
        MotorThisWay(motorNo);
    }
}

void beep(){
    tone(B5);
    delay_ms(100);
    tone(mute);
}

void intro() {
    beep();
    output_high(USER_LED);
    output_high(RUN_LED);
    delay_ms(100);
    output_low(USER_LED);
    output_low(RUN_LED);
    beep();
}

//Timer1
//Prescaler 1:8; TMR1 Preload = 28036; Actual Interrupt Time : 25 ms
#int_timer1
void timer1ISR() {
    set_timer1(T1_COUNTER);
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
void timer2ITR() {
    if(beep_is_high == 1){
        output_high(PIN_C2);
    } else {
        output_low(PIN_C2);
    }
    beep_is_high = ~beep_is_high;
}

void tone(unsigned int16 beep){
    if(beep == 0){
        disable_interrupts(INT_TIMER2);
    } else {
        unsigned int8 postscaler = (beep & 0b00001110) >> 1;
        unsigned int8 load = beep >> 4;
        if(beep % 2 == 0){
            setup_timer_2(T2_DIV_BY_16, beep, postscaler);
        } else {
            setup_timer_2(T2_DIV_BY_4, beep, postscaler);
        }
        enable_interrupts(INT_TIMER2);
    }
}

void main() {
    disable_interrupts(GLOBAL);
    init_variables();
    initBoard();

    usb_cdc_init();
    usb_init();
    int eeprom = read_eeprom(LOGO_TURN_ON_ADDR);
    if(eeprom == LOGO_TURN_ON_FLAG){
        start_stop_logo_machine = 1;
        gblLogoIsRunning = 0;
    }
    intro();
    int8 i;
    while (1) {
        updateUsbBuffer();
        ProcessInput();
        if(start_stop_logo_machine){
            if (gblLogoIsRunning){
                gblLogoIsRunning = 0;
                output_low (RUN_LED);
            }else{
                srand (gblTimer);
                gblMemPtr = (read_program_eeprom(RUN_BUTTON_BASE_ADDRESS) << 8) + read_program_eeprom(RUN_BUTTON_BASE_ADDRESS + 2);
                gblMemPtr *= 2;
                gblStkPtr = 0;
                gblInputStkPtr = 0;
                gblNewByteHasArrivedFlag = 0;
                gblLogoIsRunning = 1;
                output_high(RUN_LED);
            }
            gblWaitCounter = 0;
            start_stop_logo_machine = FALSE;
        }
        if (gblLogoIsRunning) {
            if (!gblWaitCounter){
                evalOpcode(fetchNextOpcode());
            }
        }
        for (i = 0; i < MTR_COUNT; i++) {
            if(motor_onfor_needs_to_finish[i] == 1){
                if (gblTimer > motor_onfor[i]){
                    MotorOFF(i);
                    motor_onfor_needs_to_finish[i] = 0;
                }
            }
        }  
    }
}
