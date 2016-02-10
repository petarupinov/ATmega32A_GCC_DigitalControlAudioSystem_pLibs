/*************************************************************************
*** LIBRARY: UTILITY                                         *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: utility.c, v0.01, 18.10.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>		// for PORTs: DDRC, PORTC
#include "utility.h"	// needed to delclare serPin as pin of port

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/****************************
** DEFINITION OF FUNCTIONS **
****************************/

#define serPin 0

/**************************************************************
**** CONVERT NORMAL DECIMAL NUMBERS TO BINARY CODED DECIMAL ***
**************************************************************/
unsigned char decToBcd(unsigned char val)
{
	return ( (val/10*16) + (val%10) );	// Get DEC format -> convert DEC to HEX
}

/**************************************************************
**** CONVERT BINARY CODED DECIMAL TO NORMAL DECIMAL NUMBERS ***
**************************************************************/
unsigned char bcdToDec(unsigned char val)
{
	return ( (val/16*10) + (val%16) );	// Get HEX format -> convert HEX to DEC
}

/**************************************************************
**** SERIAL OUTPUT *** SHIFT RIGHT >> OUT LSB BIT IS FIRST ****
**************************************************************/
//#define serPin 3
void shiftRightOutLsbFirst()
{
	unsigned char conbyte = 0x44;
	unsigned char regALSB;
	unsigned char x;
	regALSB = conbyte;
	DDRC |= (1<<serPin);	// serial pin output

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		if(regALSB & 0x01)	// maska & za log "1" na LSB 0b00000001
		{
			PORTC |= (1<<serPin);	// izvejdane na log "1" v LSB
		}
		else
		{
			PORTC &= ~(1<<serPin);	// izvejdane na log "0" v LSB
		}
		
		regALSB = regALSB >> 1;	// shiftvane na >> nadqsno
	}

}

/*************************************************************
**** SERIAL OUTPUT *** SHIFT LEFT << OUT MSB BIT IS FIRST ****
*************************************************************/
//#define serPin 3
void shiftLeftOutMsbFirst()
{
	unsigned char conbyte = 0x44;
	unsigned char regAMSB;
	unsigned char x;
	regAMSB = conbyte;
	DDRC |= (1<<serPin);	// serial pin output

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		if(regAMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
		{
			PORTC |= (1<<serPin);	// izvejdane na log "1" v MSB
		}
		else
		{
			PORTC &= ~(1<<serPin);	// izvejdane na log "0" v MSB
		}
		
		regAMSB = regAMSB << 1;	// shiftvane na << nalqvo
	}

}

/*************************************************************
**** SERIAL INPUT **** SHIFT RIGHT >> IN LSB BIT IS FIRST ****
*************************************************************/
//#define serPin 3
void shiftRightInLsbFirst()
{
	unsigned char x;
	unsigned char REGA = 0;
	
	DDRC &= ~(1<<serPin);	// serial pin input

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		REGA = REGA >> 1;	// shift REGA to right one bit
		REGA |= (PINC & (1<<serPin)) << (7-serPin);	// copy bit serPin of PORTC to MSB of REGA
	}

}

/*************************************************************
**** SERIAL INPUT **** SHIFT LEFT << IN MSB BIT IS FIRST *****
*************************************************************/
//#define serPin 3
void shiftLeftInMsbFirst()
{
	unsigned char x;
	unsigned char REGA = 0;
	
	DDRC &= ~(1<<serPin);	// serial pin input

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		REGA = REGA << 1;	// shift REGA to left one bit
		REGA |= (PINC & (1<<serPin)) >> serPin;	// copy bit serPin of PORTC to LSB of REGA
	}

}

/*************************************************************
*********** FUNCTION SOFTWARE DELAY IN MILISECONDS ***********
*************************************************************/
void delay_ms(int miliSec)  //for 1 Mhz crystal
{
	//  miliSec = miliSec * 16;	// for 16MHz
	int i,j;
	for(i=0;i<miliSec;i++)
	{
		for(j=0;j<100;j++)
		{
		  asm("nop");
		  asm("nop");
		}
	}
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
