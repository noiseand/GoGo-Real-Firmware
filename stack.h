//
// stack.h - declair constants for the Logo VM's stacks
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



#define STACK_SIZE      		32
#define INPUT_STACK_SIZE		32

int   gblStkPtr,   // ptr to the top of the data stack
gblErrFlag;  // error flag
unsigned int16 gblStack[STACK_SIZE];

int   gblInputStkPtr;   // ptr to the top of the procedure input stack
unsigned int16 gblInputStack[INPUT_STACK_SIZE];


