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

/////////////////////////////////////////////////////
//
//  Logo Virtual Machine
//
//  Written by Arnan (Roger) Sipitakiat
//
//  Logo running on the GoGo Board was created based
//  on the Cricket Logo.
//
//  Cricket Logo creators include:
//    Fred Martin
//    Brian Silverman
//    Mitchel Resnick
//    Robbie Berg
//
/////////////////////////////////////////////////////

void evalOpcode(unsigned char opcode) {

    int i;
    int16 opr1, opr2, opr3;
    unsigned int16 genPurpose = 0;

    //////////////////////////////////////////////////////////////////////////////
    /// if opcode is a prcedure call (msb is 1)
    //printf(usb_cdc_putc," executar %Lu \n", gblMemPtr);
    if (opcode & 0b10000000) {
        genPurpose = gblMemPtr + 2;  // this is the return address
        // update the mem pointer to point at the procedure Note That due to PIC 18f4550 the offset is 2 Times fetchNextOpcode()
        gblMemPtr = ((((unsigned int16)opcode & 0b00111111) << 8) + 2*fetchNextOpcode());
        ////printf(usb_cdc_putc," gblMemPtr %Lu #\n",gblMemPtr);
        // then fetch the new opcode
        // The first opcode in a procedure is the number of parameters.
        opr1 = fetchNextOpcode();

        /// if the second msb is set -> tail recursion
        if (opcode & 0b01000000) {
            // - Because this is tail recursion, we have to overwrite preveous procedure inputs
            //   with the new ones. This loop removes the old inputs.
            // - In addition to the procedure input we have three extra entries: the data stack pointer ,
            //   the procedure return address, and the procedure input base address in the input stack.
            for (i = 0; i < (opr1 + 3); i++) {
                inputPop();
            }
        }

        // Pop the procedure inputs from the main data stack and move them
        // to the input stack
        for (i = 0; i < opr1; i++) {
            inputPush (stkPop());
        }

        inputPush (gblStkPtr); // save the data stack pointer (we use this with STOP opcode to clear the
        // data stack of the current procedure.
        inputPush (genPurpose);  // save the return address

        inputPush(gblInputStkPtr - (opr1 + 2)); // pushes a proc input base address index.
        // you'll need to read the documentation
        // to fully understand why. Meanwhile, see how it
        // is used in case LTHING
        // - we add 2 because we also keep the data stack pointer and
        //   the return address in this stack.

        return;
    }

    ////////////////////////////////////////////////////////////////////////////////////////

    switch (opcode) {
    case CODE_END:
        gblLogoIsRunning = 0;
        output_low (RUN_LED);
        // clear thes variables just in case.
        gblLoopAddress = 0;
        gblRepeatCount = 0;
        break;
    case NUM8:
        ////printf(usb_cdc_putc,"NUM8 ");
        int8 Hi3 = fetchNextOpcode();
        //printf(usb_cdc_putc," %d \n",Hi3);
        stkPush(Hi3);
        break;
    case NUM16:
        //printf(usb_cdc_putc,"NUM16 ");
        int8 Hi2 = fetchNextOpcode();
        int8 Lo2 = fetchNextOpcode();
        unsigned int16
        teste4 = Hi2 << 8;
        teste4 += Lo2;
        //printf(usb_cdc_putc," %Lu \n",teste4);
        stkPush (teste4);
        break;
    case LIST:
        //printf(usb_cdc_putc,"LIST ");
        stkPush(gblMemPtr + 2); //incremento de 2 em 2
        gblMemPtr += (2 * fetchNextOpcode());
        //printf(usb_cdc_putc,"LIST  gblMemPtr %Lu \n",gblMemPtr);
        break;
    case EOL:
        //printf(usb_cdc_putc,"EOL ");
        genPurpose = stkPop();
        //printf(usb_cdc_putc,"EOL %Lu \n",genPurpose);
        if (genPurpose > gblMemPtr) {
            //printf(usb_cdc_putc,"genPurpose >>>> gblMemPtr");
            gblMemPtr = genPurpose;
        } else {
            //printf(usb_cdc_putc,"genPurpose <<<< gblMemPtr");
            gblMemPtr = genPurpose;
            gblRepeatCount = stkPop();   // repeat count
            //printf(usb_cdc_putc," gblRepeatCount %Lu \n",genPurpose);
            if (gblRepeatCount > 1)
                gblRepeatCount--;
            if (gblRepeatCount != 1) {
                stkPush (gblRepeatCount);
                stkPush (gblMemPtr);
            }
        }
        break;
    case EOLR:
        //printf(usb_cdc_putc,"EOLR\n");
        if (stkPop()) {   // if condition is true
            stkPop();        // throw away the loop address
            gblMemPtr = stkPop(); // fetch the next command address
        } else { // if condition if false -> keep on looping.
            gblMemPtr = stkPop();
            stkPush (gblMemPtr);
            delay_ms(5); // this prevents the waituntil loop to execute too rapidly
            // which has proven to cause some problems when reading
            // sensor values.
        }
        break;

        /////////////////////////////////////////////////////////////
        // retrieve procedure input
    case LTHING:
        genPurpose = 2 * fetchNextOpcode();  // index of the input variable
        opr1 = inputPop();  // base address in the input stack
        inputPush(opr1);    // push the base address back to the stack.
        stkPush (gblInputStack[opr1 + genPurpose]);
        break;

        /////////////////////////////////////////////////////////////
        // return to the parent procedure
    case STOP:
    case OUTPUT:
        if (opcode == OUTPUT){
            genPurpose = stkPop(); // this is the output value
        }
        opr1 = inputPop();  // this is the proc-input stack base address
        gblMemPtr = inputPop(); // this is the return address
        opr2 = inputPop();  // this is the data stack index;
        // remove any remaining data that belongs to the current procedure from the data stack
        // Usually this is important for the STOP opcode.
        while (gblStkPtr > opr2){
            stkPop();
        }
        // remove the procedure inputs from the input stack
        while (gblInputStkPtr > opr1){
            inputPop();
        }
        // Output will push the output to the stack
        if (opcode == OUTPUT){
            stkPush (genPurpose);
        }
        break;

    case REPEAT:
        gblLoopAddress = stkPop();
        gblRepeatCount = stkPop();
        // these will be poped by EOL
        stkPush (gblMemPtr);  // address after repeat is complete

        if (gblRepeatCount > 1) {
            stkPush (gblRepeatCount);
            stkPush (gblLoopAddress); // address while still repeating
            gblMemPtr = gblLoopAddress;
        } else if (gblRepeatCount == 1) {
            gblMemPtr = gblLoopAddress;
        } else {  // if loop count = 0
            gblMemPtr = stkPop();
        }
        break;

    case COND_IF:
        opr1 = stkPop();  // if true pointer address
        opr2 = stkPop();  // condition
        if (opr2) {
            stkPush(gblMemPtr);
            gblMemPtr = opr1;
        }
        break;

    case COND_IFELSE:
        opr1 = stkPop(); // if false pointer address
        opr2 = stkPop(); // if true pointer address
        opr3 = stkPop(); // condition
        stkPush(gblMemPtr);
        if (opr3) {
            gblMemPtr = opr2;
        } else {
            gblMemPtr = opr1;
        }
        break;

    case BEEP:
        //printf(usb_cdc_putc,"BEEP\n");
        beep();
        break;

    case NOTE:
        break;

    case WAITUNTIL:
        gblLoopAddress = stkPop();
        // these will be poped by EOLR
        stkPush(gblMemPtr);  // address after repeat is complete
        stkPush (gblLoopAddress); // address while still repeating
        gblMemPtr = gblLoopAddress;
        break;

    case LOOP:
        //printf(usb_cdc_putc,"LOOP\n");
        gblLoopAddress = stkPop(); // the begining of loop
        gblRepeatCount = 0; // disable this counter (loop forever)
        stkPush(0);   // this distinguishes LOOP from Repeat. (see EOL)
        stkPush(gblLoopAddress); // push loop address back into the stack
        // so that EOL will loop
        gblMemPtr = gblLoopAddress;
        break;

    case WAIT:
        gblWaitCounter = stkPop() * 2;        // the main() loop will pause until
        // gblWaitCounter is 0. Timer1 ISR
        // subtracts its value every 0.1 sec.
        break;

    case TIMER:
        stkPush (gblTimer); // gblTimer increases every 1ms. See in RTCC interrupt
        break;

    case RESETT:
        gblTimer = 0;
        break;

    case SEND:
        genPurpose = stkPop();
        break;

    case IR:
        stkPush (gblMostRecentlyReceivedByte);
        gblNewByteHasArrivedFlag = 0;
        break;

    case NEWIR:
        stkPush (gblNewByteHasArrivedFlag);
        break;

    case RANDOM:
        stkPush (rand());break;

    case OP_PLUS:
    case OP_MINUS:
    case OP_MULTIPLY:
    case OP_DIVISION:
    case OP_REMAINDER:
    case OP_EQUAL:
    case OP_GREATER:
    case OP_LESS:
    case OP_AND:
    case OP_OR:
    case OP_XOR:
        opr2=stkPop();  // second operand
        opr1=stkPop();// first operand
        switch (opcode) {
    case OP_PLUS:
            opr1+=opr2;
            break;
    case OP_MINUS:
            opr1-=opr2;
            break;
    case OP_MULTIPLY:
            opr1*=opr2;
            break;
    case OP_DIVISION:
            opr1/=opr2;
            break;
    case OP_REMAINDER:
            opr1%=opr2;
            break;
    case OP_EQUAL:
            opr1=(opr1==opr2);
            break;
    case OP_GREATER:
            opr1=(opr1>opr2);
            break;
    case OP_LESS:
            opr1=(opr1<opr2);
            break;
    case OP_AND:
            opr1=(opr1&&opr2);
            break;
    case OP_OR:
            opr1=(opr1||opr2);
            break;
    case OP_XOR:
            opr1=(opr1^opr2);
            break;
        };
        stkPush(opr1);
        break;

    case OP_NOT:
        stkPush(!stkPop());
        break;

        ///////////////////////////////////////////////////////////////////////
        // Global variables
    case SETGLOBAL:
        genPurpose = stkPop();// this is the value
        globalVariables[stkPop()] = genPurpose;
        break;
    case GETGLOBAL:
        stkPush(globalVariables[stkPop()]);
        break;

        ///////////////////////////////////////////////////////////////////////
        //  Global Array

    case ASET:
        opr2 = stkPop();// this is the value to be stored
        opr1 = stkPop() * 2;// this is the array index. Each entry is two bytes wide.
        genPurpose = ARRAY_BASE_ADDRESS + stkPop();// this is the base address of the array.

        flashSetWordAddress(genPurpose + opr1);
        flashWrite(opr2);

        break;
    case AGET:
        opr1 = stkPop() * 2;// this is the array index. Each entry is two bytes wide.
        genPurpose = ARRAY_BASE_ADDRESS + stkPop();// this is the base address of the array.
        opr2 = read_program_eeprom(genPurpose + opr1);
        stkPush(opr2);
        break;
        //  Data collection commands
    case RECORD:
        genPurpose = stkPop();
        // PCM parts (14 bit PICs like the 16F877) uses an external EEPROM for data Logging storage
        flashSetWordAddress(RECORD_BASE_ADDRESS + gblRecordPtr++);
        flashWrite(genPurpose);
        // save current record pointer location
        flashSetWordAddress(MEM_PTR_LOG_BASE_ADDRESS);
        flashWrite(gblRecordPtr);
        break;
    case RECALL:
        genPurpose = read_program_eeprom(RECORD_BASE_ADDRESS + gblRecordPtr++);
        stkPush(genPurpose);
        break;
    case RESETDP:
        gblRecordPtr = 0;
        break;
    case SETDP:
        gblRecordPtr = stkPop() * 2;
        break;
    case ERASE:
        opr1 = stkPop() * 2;
        for (genPurpose=0; genPurpose<opr1; genPurpose++) {
            flashSetWordAddress(RECORD_BASE_ADDRESS + genPurpose);
            flashWrite(0);
        }
        gblRecordPtr = 0;
        break;
    case WHEN:
        break;
    case WHENOFF:
        break;
    case M_A:
        gblActiveMotors = 0b00000001;  // set bit 0
        break;
    case M_B:
        gblActiveMotors = 0b00000010;// set bit 1
        break;
    case M_AB:
        gblActiveMotors = 0b00000011;
        break;
        // Look at how M_ON, M_ONFOR, and M_OFF work carefully.
        // - M_ON, M_ONFOR starts by turning motors on.
        // - M_ON breaks right after while M_ONFOR continues.
    case M_OFF:
        i++;
    case M_THATWAY:
        i++;
    case M_THISWAY:
        i++;
    case M_RD:
        i++;
    case BRAKE:
        i++;
    case M_ON:
    case M_ONFOR:
        // Moved to motorCortol() instead
        // SetMotorMode(MOTOR_NORMAL);
        MotorControl(i);
        if (opcode == M_ONFOR) {
            gblWaitCounter = stkPop()*2; // the main() loop will pause until
            // gblWaitCounter is 0. Timer1 ISR
            // subtracts its value every 0.1 sec.
            gblONFORNeedsToFinish = 1;// this will cause fetchNextOpcode()
        }
        break;
    case SETPOWER:
        SetMotorPower(stkPop());
        break;
    case BSEND:
    case BSR:
        // These two opcodes are not supported.
        // So, halt the board if they are used.
        Halt();
        break;
    case M_C:
        gblActiveMotors = 0b00000100;// set bit 2
        break;
    case M_D:
        gblActiveMotors = 0b00001000;// set bit 3
        break;
    case M_CD:
        gblActiveMotors = 0b00001100;
        break;
    case M_ABCD:
        gblActiveMotors = 0b00001111;
        break;
    case REALLY_STOP:
        gblLogoIsRunning = 0;
        output_low(RUN_LED);
        break;
    case EB:// reads byte from memory
        stkPush(read_program_eeprom(stkPop()));
        break;
    case DB:// deposit byte to memory
        /// Note: I have checked this code. I might have swapped opr1 and opr2
        opr1 = stkPop();// value to write
        opr2 = stkPop();// memory address
        flashSetWordAddress(opr2);
        flashWrite(opr1);

        break;
    case LOW_BYTE:// returns low byte
        stkPush(stkPop() & 0xff);
        break;
    case HIGH_BYTE:// returns high byte
        stkPush(stkPop() >> 8);
        break;
    case SENSOR1:
    case SENSOR2:
    case SENSOR3:
    case SENSOR4:
    case SENSOR5:
    case SENSOR6:
    case SENSOR7:
    case SENSOR8:
        // we need the following IF because the opcode for sensor1&2 are separate from the rest.
        // If this wasn't the case we could have just done .. i = opcode - SENSOR1;
        if (opcode < SENSOR3) {
            i = opcode - SENSOR1;
        } else {
            i = opcode - SENSOR3 + 2;
        }

        stkPush(readSensor(i));
        break;

        // read sensor and treat it as a on-off switch (0 or 1)
    case SWITCH1:
    case SWITCH2:
    case SWITCH3:
    case SWITCH4:
    case SWITCH5:
    case SWITCH6:
    case SWITCH7:
    case SWITCH8:
        if (opcode < SWITCH3) {
            i = opcode - SWITCH1;
        } else {
            i = opcode - SWITCH3 + 2;
        }
        stkPush(readSensor(i)>>9);
        break;
    case ULED_ON:
        output_high(USER_LED);
        break;
    case ULED_OFF:
        output_low(USER_LED);
        break;
    case SERVO_SET_H:
    case SERVO_LT:
    case SERVO_RT:
        // Caution: SetMotorMode() must be called AFTER the
        // MotorControl() commands

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
        gblActiveMotors = stkPop();
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
	break;
    };
}
