/*
* Copyright (C) 2010-2012 Lucas Anibal Tanure Alves - ME
* Contact   Lucas Tanure [ltanure@gogoreal.com.br]
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

void stkPush(unsigned int16 stackItem) {
  if (gblStkPtr<STACK_SIZE) {
    gblStack[gblStkPtr] = stackItem;
    gblStkPtr++;
  }
}

unsigned int16 stkPop() {
  if (gblStkPtr>0) {
    gblStkPtr--;
    int16 stackItemstkPop = gblStack[gblStkPtr];
    return stackItemstkPop;
  }
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
}

void clearStack() {
  gblStkPtr=gblInputStkPtr=0;
}


void sendBytes(unsigned int16 memPtr, unsigned int16 count) {
    while (count-- > 0){
        printf(usb_cdc_putc,"%c",read_program_eeprom(FLASH_USER_PROGRAM_BASE_ADDRESS + memPtr++));
    }
}


unsigned int16 fetchNextOpcode() {
    unsigned int16 opcode;
    if (gblONFORNeedsToFinish) {
      gblONFORNeedsToFinish = 0;
      return(M_OFF);
    } else {
        opcode = read_program_eeprom(FLASH_USER_PROGRAM_BASE_ADDRESS + gblMemPtr);
        gblMemPtr+=2;
    }
    return opcode;
}

void evalOpcode(unsigned char opcode) {
   int i;
   int16 opr1, opr2, opr3;
   unsigned int16 genPurpose=0;
   if (opcode & 0b10000000) {
        genPurpose=gblMemPtr+2;
        gblMemPtr = ((((unsigned int16)opcode & 0b00111111) << 8) + 2*fetchNextOpcode());
      opr1 = fetchNextOpcode();
      if (opcode & 0b01000000) {
         for (i=0 ; i < (opr1+3) ; i++) {
            inputPop();
         }
      }
      for (i=0 ; i<opr1 ; i++) {
         inputPush(stkPop());
      }
      inputPush(gblStkPtr);
      inputPush(genPurpose);
      inputPush(gblInputStkPtr - (opr1+2));
      return;
   }
   switch (opcode) {
      case  CODE_END:
         gblLogoIsRunning = 0;
         output_low(RUN_LED);
         gblLoopAddress = 0;
         gblRepeatCount = 0;
         break;
      case  NUM8:
         int8 Hi3 = fetchNextOpcode();
         stkPush(Hi3);
         break;
      case  NUM16:
         int8 Hi2 = fetchNextOpcode();
         int8 Lo2 = fetchNextOpcode();
         unsigned int16 teste4 = Hi2 << 8;
         teste4 += Lo2;
         stkPush(teste4);
         break;
      case  LIST:
         stkPush(gblMemPtr+2); //incremento de 2 em 2
         gblMemPtr += (2*fetchNextOpcode());
         break;
      case  EOL:
         genPurpose = stkPop();
         if (genPurpose > gblMemPtr) {
            gblMemPtr = genPurpose;
         } else {
            gblMemPtr = genPurpose;
            gblRepeatCount = stkPop();
            if (gblRepeatCount > 1){
               gblRepeatCount--;}
            if (gblRepeatCount != 1) {
               stkPush(gblRepeatCount);
               stkPush(gblMemPtr);
            }
         }
         break;
      case  EOLR:
         if (stkPop()) {
            stkPop();
            gblMemPtr = stkPop();
         } else {
            gblMemPtr = stkPop();
            stkPush(gblMemPtr);
            delay_ms(5);
         }
         break;
      case  LTHING:
         genPurpose = 2*fetchNextOpcode();
         opr1 = inputPop();
         inputPush(opr1);
         stkPush(gblInputStack[opr1 + genPurpose]);
         break;
      case   STOP:
      case OUTPUT:
         if (opcode == OUTPUT){
            genPurpose = stkPop(); // this is the output value
         }
         opr1 = inputPop();
         gblMemPtr = inputPop();
         opr2 = inputPop();
         while (gblStkPtr > opr2){
            stkPop();
        }
         while (gblInputStkPtr > opr1){
            inputPop();
        }
         if (opcode == OUTPUT){
            stkPush(genPurpose);
        }
         break;

      case  REPEAT:
         gblLoopAddress = stkPop();
         gblRepeatCount = stkPop();
         stkPush(gblMemPtr);
         if (gblRepeatCount > 1) {
            stkPush(gblRepeatCount);
            stkPush(gblLoopAddress);
            gblMemPtr = gblLoopAddress;
         } else if (gblRepeatCount == 1) {
            gblMemPtr = gblLoopAddress;
         } else {
            gblMemPtr = stkPop();
         }
         break;
      case  COND_IF:
         opr1=stkPop();
         opr2=stkPop();
         if (opr2) {
            stkPush(gblMemPtr);
            gblMemPtr=opr1;
         }
         break;
      case  COND_IFELSE:
         opr1=stkPop();
         opr2=stkPop();
         opr3=stkPop();
         stkPush(gblMemPtr);
         if (opr3) {
            gblMemPtr=opr2;
         } else {
            gblMemPtr=opr1;
         }
         break;

      case  BEEP:
         beep();
         break;
      case  NOTE:
         break;
      case  WAITUNTIL:
         gblLoopAddress = stkPop();
         stkPush(gblMemPtr);
         stkPush(gblLoopAddress);
         gblMemPtr = gblLoopAddress;
         break;
      case  LOOP:
         gblLoopAddress = stkPop();
         gblRepeatCount = 0;
         stkPush(0);
         stkPush(gblLoopAddress);
         gblMemPtr = gblLoopAddress;
         break;
      case  WAIT:
         gblWaitCounter = stkPop()*2;
         break;
      case  TIMER:
         stkPush(gblTimer);
         break;
      case  RESETT:
         gblTimer = 0;
         break;
      case  SEND:
         genPurpose = stkPop();
         break;
      case   IR:
         stkPush(gblMostRecentlyReceivedByte);
         gblNewByteHasArrivedFlag = 0;
         break;
      case   NEWIR:
         stkPush(gblNewByteHasArrivedFlag);
         break;
      case  RANDOM:
         stkPush(rand());
         break;
      case  OP_PLUS:
      case  OP_MINUS:
      case  OP_MULTIPLY:
      case  OP_DIVISION:
      case  OP_REMAINDER:
      case  OP_EQUAL:
      case  OP_GREATER:
      case  OP_LESS:
      case  OP_AND:
      case  OP_OR:
      case  OP_XOR:
         opr2=stkPop();
         opr1=stkPop();
         switch (opcode) {
            case  OP_PLUS:
               opr1+=opr2;
               break;
            case  OP_MINUS:
               opr1-=opr2;
               break;
            case  OP_MULTIPLY:
               opr1*=opr2;
               break;
            case  OP_DIVISION:
               opr1/=opr2;
               break;
            case  OP_REMAINDER:
               opr1%=opr2;
               break;
            case  OP_EQUAL:
               opr1=(opr1==opr2);
               break;
            case  OP_GREATER:
               opr1=(opr1>opr2);
               break;
            case  OP_LESS:
               opr1=(opr1<opr2);
               break;
            case  OP_AND:
               opr1=(opr1&&opr2);
               break;
            case  OP_OR:
               opr1=(opr1||opr2);
               break;
            case  OP_XOR:
               opr1=(opr1^opr2);
               break;
         };
         stkPush(opr1);
         break;

      case  OP_NOT:
         stkPush(!stkPop());
         break;
      case  SETGLOBAL:
         genPurpose = stkPop();
         globalVariables[stkPop()] = genPurpose;
         break;
      case  GETGLOBAL:
         stkPush(globalVariables[stkPop()]);
         break;
      case  ASET:
         opr2 = stkPop();
         opr1 = stkPop() * 2;
         genPurpose = ARRAY_BASE_ADDRESS + stkPop();

         flashSetWordAddress(genPurpose + opr1);
         flashWrite(opr2);

         break;
      case  AGET:
         opr1 = stkPop() * 2;
         genPurpose = ARRAY_BASE_ADDRESS + stkPop();
         opr2 = read_program_eeprom(genPurpose + opr1);
         stkPush(opr2);

         break;
      case  RECORD:
         genPurpose = stkPop();
         flashSetWordAddress(RECORD_BASE_ADDRESS + gblRecordPtr++);
         flashWrite(genPurpose);
         flashSetWordAddress(MEM_PTR_LOG_BASE_ADDRESS);
         flashWrite(gblRecordPtr);
         break;

      case  RECALL:
         genPurpose = read_program_eeprom(RECORD_BASE_ADDRESS + gblRecordPtr++);
         stkPush(genPurpose);
         break;

      case  RESETDP:
         gblRecordPtr = 0;
         break;

      case  SETDP:
         gblRecordPtr = stkPop() * 2;
         break;

      case  ERASE:
         opr1 = stkPop() * 2;
         for (genPurpose=0 ; genPurpose<opr1 ; genPurpose++) {
            flashSetWordAddress(RECORD_BASE_ADDRESS + genPurpose);
            flashWrite(0);
         }
         gblRecordPtr = 0;
         break;
      case  WHEN:
         break;
      case  WHENOFF:
         break;
      case  M_A:
         gblActiveMotors = 0b00000001;  // set bit 0
         break;
      case  M_B:
         gblActiveMotors = 0b00000010;  // set bit 1
         break;
      case  M_AB:
         gblActiveMotors = 0b00000011;
         break;
      case  M_OFF:
         i++;
      case  M_THATWAY:
         i++;
      case  M_THISWAY:
         i++;
      case  M_RD:
         i++;
      case  BRAKE:
         i++;
      case  M_ON:
      case  M_ONFOR:
         MotorControl(i);
         if (opcode == M_ONFOR) {
            gblWaitCounter = stkPop()*2;
            gblONFORNeedsToFinish = 1;
         }
         break;
      case  SETPOWER:
         SetMotorPower(stkPop());
         break;
      case  BSEND:
      case  BSR:
         Halt();
         break;
      case  M_C:
         gblActiveMotors = 0b00000100;
         break;
      case  M_D:
         gblActiveMotors = 0b00001000;
         break;
      case  M_CD:
         gblActiveMotors = 0b00001100;
         break;
      case  M_ABCD:
         gblActiveMotors = 0b00001111;
         break;
      case  REALLY_STOP:
         gblLogoIsRunning = 0;
         output_low(RUN_LED);
         break;
      case  EB:
         stkPush(read_program_eeprom(stkPop()));
         break;
      case  DB:
         opr1 = stkPop();
         opr2 = stkPop();

         flashSetWordAddress(opr2);
         flashWrite(opr1);

         break;

      case  LOW_BYTE:  // returns low byte
         stkPush(stkPop() & 0xff);
         break;
      case  HIGH_BYTE:  // returns high byte
         stkPush(stkPop() >> 8);
         break;
      case  SENSOR1:
      case  SENSOR2:
      case  SENSOR3:
      case  SENSOR4:
      case  SENSOR5:
      case  SENSOR6:
      case  SENSOR7:
      case  SENSOR8:
         if (opcode < SENSOR3) {
            i = opcode - SENSOR1;
         } else {
            i = opcode - SENSOR3 + 2;
         }
         stkPush(readSensor(i));
         break;
      case  SWITCH1:
      case  SWITCH2:
      case  SWITCH3:
      case  SWITCH4:
      case  SWITCH5:
      case  SWITCH6:
      case  SWITCH7:
      case  SWITCH8:
         if (opcode < SWITCH3) {
            i = opcode - SWITCH1;
         } else {
            i = opcode - SWITCH3 + 2;
         }
         stkPush(readSensor(i)>>9);
         break;
      case ULED_ON:
         USER_LED_ON;
         break;
      case ULED_OFF:
         USER_LED_OFF;
         break;
      case SERVO_SET_H:
      case SERVO_LT:
      case SERVO_RT:
         MotorControl(MTR_ON);
         MotorControl(MTR_THISWAY);
         SetMotorMode(MOTOR_SERVO);
         i = stkPop();
         if (opcode == SERVO_SET_H) {
            SetMotorPower(i);
         } else if (opcode == SERVO_LT)
            ChangeMotorPower(i);
         else
            ChangeMotorPower(-1*i);

         break;
      case TALK_TO_MOTOR:
         opr1 = stkPop(); // this is the motor bits
         TalkToMotor(opr1);
         break;
      case CL_I2C_START:
         i2c_start();
         break;

      case CL_I2C_STOP:
         i2c_stop();
         break;

      case CL_I2C_WRITE:
         i2c_write(stkPop());
         break;

      case CL_I2C_READ:
         stkPush(i2c_read(stkPop()));
         break;

      default:
         Halt();
  };
}
