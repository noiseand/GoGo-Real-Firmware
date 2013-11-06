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
 gblLogoIsRunning = 0;//Stop Logo Code
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
  gblLogoIsRunning = 0;//Stop Logo Code
  return 0;
}
