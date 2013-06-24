//
// stack.c - Provides stack objects for the Logo Virtual Machine
//
// Copyright (C) 2001-2007 Massachusetts Institute of Technology
// Contact   Arnan (Roger) Sipiatkiat [arnans@gmail.com]

//************************* GOGO BR  *******************************************//
//*****  contact  Felipe Augusto Silva  *****************************************//
//*****  email:  fel1310@hotmail.com   *****************************************//
//******************************************************************************//

// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.






/////////////////////////////////////////////
//  Main Data Stack

void stkErro() {
	/////////////////////
	//  Halt the board if stak overflows
	while (!input(RUN_BUTTON)) {
		output_low(RUN_LED);
		output_high(USER_LED);
		delay_ms(100);
		output_high(RUN_LED);
		output_low(USER_LED);
		delay_ms(100);
	}
}

void stkPush(unsigned int16 stackItem) {

	if (gblStkPtr<STACK_SIZE) {
		gblStack[gblStkPtr] = stackItem;
    //printf(usb_cdc_putc,"stkPush %d %Lu\n",gblStkPtr, stackItem);
    gblStkPtr++;
    
	} else {
		gblErrFlag |= 0b00000001;  // set err bit 1 "stack overflow"
    printf(usb_cdc_putc,"stack overflow\n");
		stkErro();
	}
}

unsigned int16 stkPop() {
	if (gblStkPtr>0) {
    gblStkPtr--;
    int16 stackItemstkPop = gblStack[gblStkPtr];
    //printf(usb_cdc_putc,"stkPop %d %Lu\n",gblStkPtr, stackItemstkPop);
		return stackItemstkPop;
	} else {
		gblErrFlag |= 0b00000010;  // set error bit 2 "stack empty"
    printf(usb_cdc_putc,"stack empty\n");
		stkErro();
	}
}


/////////////////////////////////////////////
//  Procedure Input Stack


void inputPush(unsigned int16 stackItem) {
	if (gblInputStkPtr<INPUT_STACK_SIZE) {
		gblInputStack[gblInputStkPtr++] = stackItem;
	} else {
		gblErrFlag |= 0b00000100;  // set err bit 3 "stack overflow"
    //printf(usb_cdc_putc,"stack overflow\n");
		stkErro();
	}
}

unsigned int16 inputPop(void) {
	if (gblInputStkPtr>0) {
		return(gblInputStack[--gblInputStkPtr]);
	} else {
		gblErrFlag |= 0b00001000;  // set error bit 4 "stack empty"
    //printf(usb_cdc_putc,"stack empty\n");
		stkErro();
	}
}

void clearStack() {
	gblStkPtr=gblInputStkPtr=0;
}
