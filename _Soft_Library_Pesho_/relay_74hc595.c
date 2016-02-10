/*************************************************************************
*** LIBRARY: RELAY + SHIFT REGISTER 74HC595                  *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: relay_74hc595.c, v0.02, 29.11.2015            *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "../ATmega32A_GCC_DigitalControlAudioSystem_pLibs.h"
#include "relay_74hc595.h"
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
void RELAYS_IN_INIT()
{
}

void RELAYS_OUT_INIT()
{
}

/****************************************************************************************************************
** DEFINITION RELAYS IN FUNCTIONS, WRITE/TRANSMIT 8bits (1Byte) TO SHIFT REGISTER 74HC595 AND CHOOSE RELAYS IN **
****************************************************************************************************************/
void RELAYS_IN_CHOOSE(unsigned char rel_in)	// HELP: RELAYS_IN_CHOOSE(unsigned char byte_of_choosing_combination_of_relay_in) // 74HC595 shift right out, lsb is first
{
	unsigned char storeLSB;
	unsigned char x;
	storeLSB = rel_in;

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		RELIN_SCK_low();

		if(storeLSB & 0x01)	// maska & za log "1" na LSB 0b00000001
		{
			RELIN_SDI_high();	// izvejdane na log "1" v MSB	// PORTC |= (1<<RELIN_SDI_PIN);
		}
		else
		{
			RELIN_SDI_low();	// izvejdane na log "0" v MSB	// PORTC &= ~(1<<RELIN_SDI_PIN);	
		}
		
		storeLSB = storeLSB >> 1;	// shiftvane na >> nadqsno

		RELIN_SCK_high();
	}

	RELIN_RCK_low();
	_delay_us(170);			//rcall 180 us
	RELIN_RCK_high();
	_delay_us(170);			//rcall 180 us

}

void relays_in1_2ch()
{
	RELAYS_IN_CHOOSE(0b00100000);	// RELE 1
}

void relays_in1_6ch()
{
	RELAYS_IN_CHOOSE(0b01100001);	// RELE 1,2,3
}

void relays_in2_2ch()
{
	RELAYS_IN_CHOOSE(0b00001010);	// RELE 4,5
}

void relays_in2_6ch()
{
	RELAYS_IN_CHOOSE(0b10011010);	// RELE 4,5,6,7
}

void relays_in3_2ch()
{
	RELAYS_IN_CHOOSE(0b00001100);	// RELE 8,5
}

void relays_in3_6ch()
{
	RELAYS_IN_CHOOSE(0b10011100);	// RELE 8,5,6,7
}

void relays_in_off()
{
	RELAYS_IN_CHOOSE(0b00000000);	// 0b00000000 // [8][7][6][5][4][3][2][1] // RELETA IZKLIUCHENI 
}

void relays_in_init()
{
	RELAYS_IN_CHOOSE(0b00000000);	// 0b00000000 // [8][7][6][5][4][3][2][1] // RELETA IZKLIUCHENI 
}
/******************************************************************************************************************
** DEFINITION RELAYS OUT FUNCTIONS, WRITE/TRANSMIT 8bits (1Byte) TO SHIFT REGISTER 74HC595 AND CHOOSE RELAYS OUT **
******************************************************************************************************************/
void RELAYS_OUT_CHOOSE(unsigned char rel_out)	// HELP: RELAYS_OUT_CHOOSE(unsigned char byte_of_choosing_combination_of_relay_in) // 74HC595 shift right out, lsb is first
{
	unsigned char storeLSB;
	unsigned char x;
	storeLSB = rel_out;

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		RELOUT_SCK_low();

		if(storeLSB & 0x01)	// maska & za log "1" na LSB 0b00000001
		{
			RELOUT_SDI_high();	// izvejdane na log "1" v MSB	// PORTC |= (1<<RELIN_SDI_PIN);
		}
		else
		{
			RELOUT_SDI_low();	// izvejdane na log "0" v MSB	// PORTC &= ~(1<<RELIN_SDI_PIN);	
		}
		
		storeLSB = storeLSB >> 1;	// shiftvane na >> nadqsno

		RELOUT_SCK_high();
	}

	RELOUT_RCK_low();
	_delay_us(170);			//rcall 180 us
	RELOUT_RCK_high();
	_delay_us(170);			//rcall 180 us

}

void relays_out_1ch()
{
	RELAYS_OUT_CHOOSE(0b10000000);	// RELE 1
}

void relays_out_6ch()
{
	RELAYS_OUT_CHOOSE(0b11111100);	// RELE 1,2,3,4,5,6
}
void relays_out_off()
{
	RELAYS_OUT_CHOOSE(0b00000000);	// RELE 1,2,3,4,5,6
}
void relays_out_init()
{
	RELAYS_OUT_CHOOSE(0b00000000);	// RELE 1,2,3,4,5,6
}
/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
