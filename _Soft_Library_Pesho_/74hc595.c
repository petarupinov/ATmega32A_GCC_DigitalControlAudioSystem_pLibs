/*************************************************************************
*** LIBRARY: SHIFT REGISTER 74HC595                          *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: 74hc595.c, v0.02, 29.11.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "74hc595.h"
#include "utility.h"		// using for debug and others

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/************************************************
** DEFINITION SHIFT REGISTER 74HC595 FUNCTIONS **
************************************************/

/*********************************************
** INITIZLIZATION OF SHIFT REGISTER 74HC595 **
*********************************************/
void SHIFTREG_INIT()
{
}

/***********************************************************
** WRITE/TRANSMIT 8bits (1Byte) TO SHIFT REGISTER 74HC595 **
***********************************************************/
void SHIFTREG_DATA_ONE(unsigned char data)	// HELP: SHIFTREG shift left out, msb is first
{
	unsigned char storeMSB;
	unsigned char x;
	storeMSB = data;

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		SHIFTREG_SCK_low();

		if(storeMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
		{	
			SHIFTREG_SDI_high();	// izvejdane na log "1" v MSB	// SHIFTREG |= (1<<SHIFTREG_SDI_PIN);	// SHIFTREG = PORTA
		}
		else
		{
			SHIFTREG_SDI_low();	// izvejdane na log "0" v MSB	// SHIFTREG &= ~(1<<SHIFTREG_SDI_PIN);	// SHIFTREG = PORTA
		}

		SHIFTREG_SCK_high();		

		storeMSB = storeMSB << 1;	// shiftvane na << nalqvo
	}

	SHIFTREG_RCK_low();
	_delay_us(170);			//rcall 180 us
	SHIFTREG_RCK_high();
	_delay_us(170);			//rcall 180 us

}

/*************************************************************
** WRITE/TRANSMIT MORE DATA BYTES TO SHIFT REGISTER 74HC595 **
*************************************************************/
void SHIFTREG_DATA(char data [], int numsymbols)	// HELP: LCD_EXECUTE_DATA(char masive_of_byte_symbols, int number_of_masive_byte_symbols) // LCD shift left out, msb is first
{
	for(int count_ns = 0; count_ns < numsymbols; count_ns++)	// count_ns < numsymbols+1 -> za string ima nijda ot dobavqne na +1 za posledniq simvol ili ot smqna na znaka <=
	{
		unsigned char storeMSB;
		unsigned char x;
		storeMSB = data[count_ns];

		for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
		{
			SHIFTREG_SCK_low();

			if(storeMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
			{
				SHIFTREG_SDI_high();	// izvejdane na log "1" v MSB	// PORTA |= (1<<LCD_SDI_PIN);
			}
			else
			{
				SHIFTREG_DI_low();	// izvejdane na log "0" v MSB	// PORTA &= ~(1<<LCD_SDI_PIN);
			}

			SHIFTREG_SCK_high();		

			storeMSB = storeMSB << 1;	// shiftvane na << nalqvo
		}

		SHIFTREG_RCK_low();
		_delay_us(170);			//rcall 180 us
		SHIFTREG_RCK_high();
		_delay_us(170);			//rcall 180 us
	}
}

/*************************************************************
** WRITE/TRANSMIT MORE DATA BYTES TO SHIFT REGISTER 74HC595 **
*************************************************************/
void SHIFTREG_pDATA(unsigned char *data)	// HELP: SHIFTREG shift left out, msb is first
{
	unsigned char storeMSB;
	unsigned char x;

	while(*data)
	{
		storeMSB = *data++;

		for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
		{
			SHIFTREG_SCK_low();

			if(storeMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
			{	
				SHIFTREG_SDI_high();	// izvejdane na log "1" v MSB	// SHIFTREG |= (1<<SHIFTREG_SDI_PIN);	// SHIFTREG = PORTA
			}
			else
			{
				SHIFTREG_SDI_low();	// izvejdane na log "0" v MSB	// SHIFTREG &= ~(1<<SHIFTREG_SDI_PIN);	// SHIFTREG = PORTA
			}

			SHIFTREG_SCK_high();		

			storeMSB = storeMSB << 1;	// shiftvane na << nalqvo
		}

		SHIFTREG_RCK_low();
		_delay_us(170);			//rcall 180 us
		SHIFTREG_RCK_high();
		_delay_us(170);			//rcall 180 us
	}
}
/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
