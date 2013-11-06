//
// logovm.c - Logo compiler support routines
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
//    Fred G. Martin
//    Brian Silverman
//    Mitchel Resnick
//    Robbie Berg
//
/////////////////////////////////////////////////////

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
