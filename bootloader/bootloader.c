//
// Copyright (C) 2001-2007 Massachusetts Institute of Technology
// Contact   Arnan (Roger) Sipiatkiat [arnans@gmail.com]

//************************* GOGO BR  *******************************************//
//*****  contact  Felipe Augusto Silva  *****************************************//
//*****  email:  fel1310@hotmail.com   *****************************************//
//******************************************************************************//
#define _bootloader
#include "bootloader.h"

// degine os bytes de comunicacao com o firmware downloader
#define READY_FOR_NEXT  0x11
#define FINISH_FLAG     0x55
#define BOOTLOADER_OVERWRITE  0x80

#define USBBufferSize 32  // USB input buffer size


int8 flag_de_bootloader;
#locate flag_de_bootloader=local_flag

/*
 Goto the interrupt vector of the application.
*/
#org 0x08,0x17
void high_isr(void) {
    if (bit_test(flag_de_bootloader,0)) {
        goto_address(LOADER_ISR);
    } else {
        goto_address(APPLICATION_ISR);
    }
}

#org 0x18,0x27
void low_isr(void) {
    if (bit_test(flag_de_bootloader,0)) {
        goto_address(LOADER_ISR+0x10);
    } else {
        goto_address(APPLICATION_ISR+0x10);
    }
}

void write_program(int32 location, char *src, int16 size)
{
   int8 block[EEPROM_ERASE_SIZE];
   int32 block_start;
   int8 i;
   int8 num;

   block_start = location & (~((int32)EEPROM_ERASE_SIZE-1));
   i=location-block_start;

   while (size) 
   {
      read_program_memory(block_start, block, sizeof(block));  //read entire block to ram buffer

      if (size>(EEPROM_ERASE_SIZE-i)) {num=EEPROM_ERASE_SIZE-i;} else {num=size;}

      memcpy(&block[i],src,num);    //modify ram buffer

      erase_program_eeprom(block_start);     //erase entire block

      write_program_memory(block_start, block, sizeof(block));    //write modified block

      src+=num;
      block_start+=EEPROM_ERASE_SIZE;
      i=0;
      size-=num;
   }
}

// converts ascii text to integer
// i.e. '1' = 1, 'A' = 10
int a2i(char asciiByte) {

    if (asciiByte >= 'A' && asciiByte <= 'F')
        return((asciiByte) - 'A' + 10);
    else if (asciiByte >= '0' && asciiByte <= '9')
        return( asciiByte - '0');
}


void load() {
    // contadores e flags
    int i=0,j=0;
    int1 notDone = 1;
    // enderecos
    unsigned int16 extended_linear_address = 0;
    unsigned int16 writeAddr =0 ;
    // variaveis para dados
    int Buffer[USBBufferSize];
    int aux[32];
    // configuracoes de escrita
    int recLen;
    char recType;
	
    do {
        usb_task();

        delay_ms(10);

        while (usb_cdc_getc() != ':') ;

        for (i=0;i<6;i++) {// leitura de dados padrões
            aux[i] = usb_cdc_getc();
        }

        recLen = (a2i(aux[0]) << 4) + a2i(aux[1]);// numero de bytes de dados

        writeAddr = a2i(aux[2]);
        writeAddr <<= 4;
        writeAddr += a2i(aux[3]);
        writeAddr <<= 4;
        writeAddr += a2i(aux[4]);
        writeAddr <<= 4;
        writeAddr += a2i(aux[5]);

        usb_cdc_getc(); // recebe o zero antes do recType
        recType = usb_cdc_getc();// tipo de escrita , esta prevista 0, 1 e 4

        // Manipulando os dados
        for (i=0;i<(recLen*2);i++) {// pegando dados
            aux[i] = usb_cdc_getc();
        }

        for (i=0,j=0;i<(recLen*2);i=i+2,j++) {// montando os bytes
            Buffer[j] = a2i(aux[i]);
            Buffer[j] <<= 4;
            Buffer[j] += a2i(aux[i+1]);
        }

        if (recType == '1'){ //End Of File record
			printf(usb_cdc_putc, "%c", FINISH_FLAG);
            notDone = 0;
		}

        else if (recType == '4') {//Extended Linear Address Record, allowing for fully 32 bit addressing
            extended_linear_address = Buffer[0];
            extended_linear_address <<= 8;
            extended_linear_address += Buffer[1];
            printf(usb_cdc_putc, "%c", READY_FOR_NEXT);
        }

        else if (recType == '0') {//data record
            if (extended_linear_address > 0) {// recebendo codigo da eeprom , bits de configuração, id ,

                if (extended_linear_address == 0xf0) {
                    for (i=0,j=0;i<(recLen/2);i++,j=j+2)
                        write_eeprom((int)(writeAddr + i), Buffer[j]);
                } else if (extended_linear_address == 0x30 ) {
                    // Não se pode reescrever os bits de configuração
                    // caso isso aconteça o bootloader não ira funcionar
                }
            } else {// extended_linear_address == 0 , recebendo o codigo principal
                if (writeAddr <= LOADER_SIZE) {
                    printf(usb_cdc_putc, "%c", BOOTLOADER_OVERWRITE);
                } else {
                    write_program(writeAddr,Buffer,recLen);
                }
            }

            printf(usb_cdc_putc, "%c", READY_FOR_NEXT);
        }

    } while (notDone);
   delay_ms(2000);   //give time for packet to flush

}



void main() {

    if (input(RUN_BUTTON)) {
        flag_de_bootloader=TRUE;

        usb_cdc_init();	
        usb_init(); //Initializes the USB hardware

        while (!usb_enumerated()); //continue only if the device has been enumerated by the PC

		output_high(RUN_LED);
        output_high(USER_LED);

		load();
		
		output_low(RUN_LED);
		output_low(USER_LED);
	
		reset_cpu();
		delay_ms(50);

    }
	
	flag_de_bootloader=FALSE;
	goto_address(APPLICATION_START);
}

#ORG default
