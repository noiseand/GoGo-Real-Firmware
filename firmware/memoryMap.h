//
// memoryMap.c - Defines the memory segments and their addresses
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


//////////////////////////////////////////////////
//  Location of the run button vector.
//  This is 0xff0 in the cricket. We map it to
//  the following address.
#define RUN_BUTTON_BASE_ADDRESS           0x7FF0

//////////////////////////////////////////////////
//  Base address for the memory pointer storage
//  - Normally this is kept in the variable gblRecordPtr
//  - But we record this value to the flash mem
//    everytime we record data.
//  - This way we know where is the most recently
//    recorded location. It would be useful to know
//    this in the case of temporary power loss.
//  - It is used for automatic Data upload
// note that the mem pointer occupies two bytes
#define MEM_PTR_LOG_BASE_ADDRESS 0x7608

//////////////////////////////////////////////////
//   User program memory map
//   we storm the cricket logo code in the
//   flash memory to increase the speed.
//
#define FLASH_USER_PROGRAM_BASE_ADDRESS   0x5D80
#define FIRMWARE_END   0x5D30

/////////////////////////////////////////////////
//   Base address for the array memory
#define ARRAY_BASE_ADDRESS      0x6700


/////////////////////////////////////////////////
//   Data recording memory map
#define RECORD_BASE_ADDRESS      0x59C0
