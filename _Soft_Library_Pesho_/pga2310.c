/*************************************************************************
*** LIBRARY: PGA2310 / PGA2311 with SPI (Serial Peripheral Interface) ****
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: pga2310.c, v0.01, 18.10.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one SPI        *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "spi.h"
#include "pga2310.h"

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/****************************************
** DEFINITION PGA2310 U6 SPI FUNCTIONS **
****************************************/
void PGA2310_U6_SPI(unsigned char pgaVolumeLeft, unsigned char pgaVolumeRight)	//PGA2310_U6_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// PB4 - /SS ENABLE
	PGA2310_U6_SPI_CS_low();	// PB3 - ENABLE PGA2310 U6 SPI

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = pgaVolumeLeft;		//volume left;
	while(!(SPSR & (1<<SPIF)))
	{
	}

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = pgaVolumeRight;		//volume right;
	while(!(SPSR & (1<<SPIF)))
	{
	}

	PGA2310_U6_SPI_CS_high();	// PB3 - DISABLE PGA2310 U6 SPI
//	PORTB = (1<<PB4);			// PB4 - /SS DISABLE
}

/****************************************
** DEFINITION PGA2310 U7 SPI FUNCTIONS **
****************************************/
void PGA2310_U7_SPI(unsigned char pgaVolumeLeft, unsigned char pgaVolumeRight)	//PGA2310_U7_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// PB4 - /SS ENABLE
	PGA2310_U7_SPI_CS_low();	// PB3 - ENABLE PGA2310 U7 SPI

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = pgaVolumeLeft;		//volume_left;
	while(!(SPSR & (1<<SPIF)))
	{
	}

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = pgaVolumeRight;		//volume right;
	while(!(SPSR & (1<<SPIF)))
	{
	}

	PGA2310_U7_SPI_CS_high();	// PB3 - DISABLE PGA2310 U7 SPI
//	PORTB = (1<<PB4);			// PB4 - /SS DISABLE
}

/****************************************
** DEFINITION PGA2310 U8 SPI FUNCTIONS **
****************************************/
void PGA2310_U8_SPI(unsigned char pgaVolumeLeft, unsigned char pgaVolumeRight)	//PGA2310_U8_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// PB4 - /SS ENABLE
	PGA2310_U8_SPI_CS_low();	// PB3 - ENABLE PGA2310 U8 SPI

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = pgaVolumeLeft;		//volume left;
	while(!(SPSR & (1<<SPIF)))
	{
	}

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = pgaVolumeRight;		//volume right;
	while(!(SPSR & (1<<SPIF)))
	{
	}

	PGA2310_U8_SPI_CS_high();	// PB3 - DISABLE PGA2310 U8 SPI
//	PORTB = (1<<PB4);			// PB4 - /SS DISABLE
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
