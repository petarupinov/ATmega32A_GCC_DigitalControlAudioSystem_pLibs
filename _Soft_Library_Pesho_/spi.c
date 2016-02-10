/*************************************************************************
*** LIBRARY: SPI / Serial Peripheral Interface               *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: spi.c, v0.03, 29.11.2015                      *************
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
#include "utility.h"		// using for debug and others

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/*****************************
** DEFINITION SPI FUNCTIONS **
*****************************/

/**************************
** INITIZLIZATION OF SPI **
***************************/
void spi_init()
{
	SPSR = (0<<SPIF)|(0<<WCOL)|(0<<SPI2X);
//	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);			//0b01010010	// SPR1   = 1 - 16 000 000 / 64  = 250 000 = 250kHz  // KOMENTAR ZARADI SIMULACIQTA - PROTEUS BLOKIRANE ZARADI BIT V REGISTAR
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);	//0b01010010	// SPR0,1 = 1 - 16 000 000 / 128 = 125 000 = 125kHz	 // KAKVA KOMBINACIQ OT 4-te BITa VODI DO RAZBLOKIRANETO ???
	SPDR = 0b00000000;

	/*
	PGA2310_U6_SPI_CS_high();	// /SS - DISABLE
	PGA2310_U6_SPI(0b00000000, 0b00000000);

	PGA2310_U7_SPI_CS_high();	// /SS - DISABLE
	PGA2310_U7_SPI(0b00000000, 0b00000000);

	PGA2310_U8_SPI_CS_high();	// /SS - DISABLE
	PGA2310_U8_SPI(0b00000000, 0b00000000);
	*/

	SPCR = (0<<SPE);
}

/***************************************
** DEFINITION SPI FUNCTION START ONLY **
***************************************/
void spi_start()
{
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0); //	SPCR = (1<<SPE);
}

/**************************************
** DEFINITION SPI FUNCTION STOP ONLY **
**************************************/
void spi_stop()
{
	SPCR = (0<<SPE);
}

/*****************************************
** DEFINITION SPI FUNCTION WRITE 1 BYTE **
*****************************************/
void spi_write_one_byte(unsigned char data)	// void PGA2310_U7_SPI(byte volume_left, byte volume_right)	//PGA2310_U7_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// DON'T FORGET	CHIP SELECT BIT // PB4 - NOT USED HERE
//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	spi_start();
	SPDR = data;				//volume_right;
	while(!(SPSR & (1<<SPIF)))
	{
	}
}

/******************************************
** DEFINITION SPI FUNCTION WRITE 2 BYTES **
******************************************/
void spi_write_two_bytes(unsigned char data1, unsigned char data2)	// void PGA2310_U6_SPI(byte volume_left, byte volume_right)	//PGA2310_U6_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// DON'T FORGET	CHIP SELECT BIT // PB4 - NOT USED HERE
//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	spi_start();
	SPDR = data1;				//volume_left;
	while(!(SPSR & (1<<SPIF)))
	{
	}

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = data2;				//volume_right;
	while(!(SPSR & (1<<SPIF)))
	{
	}
}

/*********************************************
** DEFINITION SPI FUNCTION WRITE MORE BYTES **
*********************************************/
void spi_write_more_bytes(unsigned char *data)	// void PGA2310_U8_SPI(byte volume_left, byte volume_right)	//PGA2310_U8_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// DON'T FORGET	CHIP SELECT BIT // PB4 - /SS ENABLE
	while(*data++)
	{
	//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
		spi_start();
		SPDR = *data;			//volume_left;
		while(!(SPSR & (1<<SPIF)))
		{
		}
	}
}

/************************************** 	/ READ SPI NOT FINISHED
** DEFINITION SPI FUNCTION READ BYTE ** 	/ READ SPI NOT FINISHED
*************************************** 	/ READ SPI NOT FINISHED */
unsigned char spi_read_more_bytes()	// void PGA2310_U8_SPI(byte volume_left, byte volume_right)	//PGA2310_U8_SPI(0b00001111);
{
	unsigned char data = 0b00000000;
	//	PORTB = (0<<PB4);			// DON'T FORGET	CHIP SELECT BIT // PB4 - NOT USED HERE
	//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = data;			//volume_left;
	while(!(SPSR & (1<<SPIF)))
	{
	}
	return data;
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
