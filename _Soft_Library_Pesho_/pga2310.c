/*************************************************************************
*** LIBRARY: PGA2310 / PGA2311 with SPI (Serial Peripheral Interface) ****
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: pga2310.c, v0.04, 29.11.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one SPI        *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "../ATmega32A_GCC_DigitalControlAudioSystem_pLibs.h"
#include "spi.h"
#include "pga2310.h"
#include "utility.h"		// using for debug and others

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

#define ZERO_FILL 0b00000000

//unsigned char volumeValue [VOLUME_MAX] = { 0x00, 0x28, 0x32, 0x3C, 0x46, 0x50, 0x5A, 0x64, 0x6E, 0x78, 0x82, 0x8C, 0x96, 0xA0, 0xAA, 0xB4, 0xBE, 0xC8, 0xD2, 0xD7 };
//					values of volume  ->	0,    40,   50,   60,   70,   80,   90,   100,  110,  120,  130,  140,  150,  160,  170,  180,  190,  200,  210,  215	<-	values of volume
//				index of values of volume   0      1     2     3     4     5     6      7     8     9    10    11    12    13    14    15    16    17    18    19


/**************************************
** DEFINITION PGA2310 INITIALIZATION **
**************************************/
void pga2310_reset()
{
	spi_init();
	
	PGA2310_U6_SPI_CS_low();	// CHIP SELECT BIT // PB3 - /SS ENABLE
	spi_write_two_bytes(ZERO_FILL, ZERO_FILL);	// left and right channel
	PGA2310_U6_SPI_CS_high();	// CHIP SELECT BIT // PB3 - /SS DISABLE

	PGA2310_U7_SPI_CS_low();	// CHIP SELECT BIT // PA6 - /SS ENABLE
	spi_write_two_bytes(ZERO_FILL, ZERO_FILL);	// left and right channel
	PGA2310_U7_SPI_CS_high();	// CHIP SELECT BIT // PA6 - /SS DISABLE

	PGA2310_U8_SPI_CS_low();	// CHIP SELECT BIT // PA6 - /SS ENABLE
	spi_write_two_bytes(ZERO_FILL, ZERO_FILL);	// left and right channel
	PGA2310_U8_SPI_CS_high();	// CHIP SELECT BIT // PA7 - /SS DISABLE
}

/**************************************
** DEFINITION PGA2310 INITIALIZATION **
**************************************/
void pga2310_init()
{
	spi_init();
	
	PGA2310_U6_SPI_CS_low();	// CHIP SELECT BIT // PB3 - /SS ENABLE
	spi_write_two_bytes(ZERO_FILL, ZERO_FILL);	// left and right channel
	PGA2310_U6_SPI_CS_high();	// CHIP SELECT BIT // PB3 - /SS DISABLE

	PGA2310_U7_SPI_CS_low();	// CHIP SELECT BIT // PA6 - /SS ENABLE
	spi_write_two_bytes(ZERO_FILL, ZERO_FILL);	// left and right channel
	PGA2310_U7_SPI_CS_high();	// CHIP SELECT BIT // PA6 - /SS DISABLE

	PGA2310_U8_SPI_CS_low();	// CHIP SELECT BIT // PA6 - /SS ENABLE
	spi_write_two_bytes(ZERO_FILL, ZERO_FILL);	// left and right channel
	PGA2310_U8_SPI_CS_high();	// CHIP SELECT BIT // PA7 - /SS DISABLE
}

/*************************************
** DEFINITION PGA2310 VOLUME UPDATE **
*************************************/
void PGA2310_Volume_Update(unsigned char pgaVolumeLeft, unsigned char pgaVolumeRight)
{
	PGA2310_U6_SPI_CS_low();	// CHIP SELECT BIT // PB3 - /SS ENABLE
	spi_write_two_bytes(pgaVolumeLeft, pgaVolumeRight);
	PGA2310_U6_SPI_CS_high();	// CHIP SELECT BIT // PB3 - /SS DISABLE

	PGA2310_U7_SPI_CS_low();	// CHIP SELECT BIT // PA6 - /SS ENABLE
	spi_write_two_bytes(pgaVolumeLeft, pgaVolumeRight);
	PGA2310_U7_SPI_CS_high();	// CHIP SELECT BIT // PA6 - /SS DISABLE

	PGA2310_U8_SPI_CS_low();	// CHIP SELECT BIT // PA7 - /SS ENABLE
	spi_write_two_bytes(pgaVolumeLeft, pgaVolumeRight);
	PGA2310_U8_SPI_CS_high();	// CHIP SELECT BIT // PA7 - /SS DISABLE

//	return SUCCESS;
}

// OLD ARCHITECTURE
/****************************************
** DEFINITION PGA2310 U6 SPI FUNCTIONS **
****************************************/
/*
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
*/
/****************************************
** DEFINITION PGA2310 U7 SPI FUNCTIONS **
****************************************/
/*
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
*/
/****************************************
** DEFINITION PGA2310 U8 SPI FUNCTIONS **
****************************************/
/*
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
*/

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
