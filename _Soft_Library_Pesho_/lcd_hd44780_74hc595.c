/*************************************************************************
*** LIBRARY: LCD DISPLAY HITACHI HD44780 + SHIFT REGISTER 74HC595 ********
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: lcd_hd44780_74hc595.c, v0.03, 29.11.2015      *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function 
#include "../ATmega32A_GCC_DigitalControlAudioSystem_pLibs.h"
#include "lcd_hd44780_74hc595.h"
#include "utility.h"		// using for debug and others

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/*************************************
** DEFINITION LCD DISPLAY FUNCTIONS **
*************************************/

/**********************************
** INITIZLIZATION OF LCD DISPLAY **
**********************************/
void LCD_INIT()
{
	LCD_COMMAND(LCD_8BIT_2ROWS_FONT5X7);		// 0b00111000	// 1. Function set: 8-bit interface data (DL = 1), 2-line display (N = 1), 5 x 7 dot character font (F = 0)
//	_delay_us(40);	// 37 uS	// comment for simulation
	LCD_COMMAND(LCD_ENTRY_MODE_INC_NOSHIFT);	// 0b00000110	// 2. Entry mode set: Increment by 1 (I/D = 1), No shift (S = 0)
//	_delay_us(40);	// 37 uS	// comment for simulation
	LCD_COMMAND(LCD_MOVE_FIRST);								// 3.
//	_delay_us(40);	// 37 uS	// comment for simulation
	LCD_COMMAND(LCD_CLEAR);						// 0b00000001	// 4. Display clear; // !!! ... from old code LCD_CLEAR can't be first command !!!
	_delay_us(1200);	// 1.53 mS	or use this _delay_ms(2); LCD_EXECUTE_COMMAND() = 440us, 440+1200 = 1640 uS
//	LCD_COMMAND(LCD_ON_BLINK_CURSOR);			// 0b00001111	// 5. Display on/off control: Display on (D = 1), Cursor on (C = 1), Blinking on (B = 1)
//	_delay_us(40);	// 37 uS	// comment for simulation
//	LCD_COMMAND(LCD_OFF);										// 5. Display off
}

/**************************************
** CLEAR ALL CONTAINS ON LCD DISPLAY **
**************************************/
void LCD_CLEAR_CONTAIN()
{
	LCD_COMMAND(LCD_CLEAR);						// 0b00000001	// 1. Display clear; // !!! ... from old code LCD_CLEAR can't be first command !!!
	_delay_us(1200);	// 1.53 mS	or use this _delay_ms(2); LCD_EXECUTE_COMMAND() = 440us, 440+1200 = 1640 uS
}

/******************************************
** WRITE/TRANSMIT COMMAND TO LCD DISPLAY **
******************************************/
void LCD_EXECUTE_COMMAND(unsigned char command)	// HELP: LCD_EXECUTE_COMMAND(unsigned char byte_choose_a_command) // LCD shift left out, msb is first
{
	unsigned char conbyte = command;
	unsigned char storeMSB;
	unsigned char x;
	storeMSB = conbyte;

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		LCD_SCK_low();

		if(storeMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
		{	
			LCD_SDI_high();	// izvejdane na log "1" v MSB	// PORTA |= (1<<LCD_SDI_PIN);
		}
		else
		{
			LCD_SDI_low();	// izvejdane na log "0" v MSB	// PORTA &= ~(1<<LCD_SDI_PIN);	
		}

		LCD_SCK_high();		

		storeMSB = storeMSB << 1;	// shiftvane na << nalqvo
	}
//update from down comment
	LCD_REGSELECT_low();	// RS = 0
	LCD_READWRITE_low();	// RW = 0
	LCD_ENABLE_high();		// EN = 1
//update from down comment
	LCD_RCK_low();
	_delay_us(170);			//rcall 180 us
	LCD_RCK_high();
	_delay_us(170);			//rcall 180 us
/*
	LCD_REGSELECT_low();	// RS = 0
	LCD_READWRITE_low();	// RW = 0
	LCD_ENABLE_high();		// EN = 1
	_delay_us(100);			//rcall 180 us
*/
	LCD_ENABLE_low();		// EN = 0
	_delay_us(100);			//rcall 180 us

}

/**************************************************
** WRITE/TRANSMIT MORE DATA BYTES TO LCD DISPLAY **
**************************************************/
void LCD_EXECUTE_DATA(char data [], int numsymbols)	// HELP: LCD_EXECUTE_DATA(char masive_of_byte_symbols, int number_of_masive_byte_symbols) // LCD shift left out, msb is first
{
	for(int count_ns = 0; count_ns < numsymbols; count_ns++)	// count_ns < numsymbols+1 -> za string ima nijda ot dobavqne na +1 za posledniq simvol ili ot smqna na znaka <=
	{
		unsigned char storeMSB;
		unsigned char x;
		storeMSB = data[count_ns];

		for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
		{
			LCD_SCK_low();

			if(storeMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
			{
				LCD_SDI_high();	// izvejdane na log "1" v MSB	// PORTA |= (1<<LCD_SDI_PIN);
			}
			else
			{
				LCD_SDI_low();	// izvejdane na log "0" v MSB	// PORTA &= ~(1<<LCD_SDI_PIN);
			}

			LCD_SCK_high();		

			storeMSB = storeMSB << 1;	// shiftvane na << nalqvo
		}
//update from down comment
		LCD_REGSELECT_high();	// RS = 1
		LCD_READWRITE_low();	// RW = 0
		LCD_ENABLE_high();		// EN = 1
//update from down comment
		LCD_RCK_low();
		_delay_us(170);			//rcall 180 us
		LCD_RCK_high();
		_delay_us(170);			//rcall 180 us
/*
		LCD_REGSELECT_high();	// RS = 1
		LCD_READWRITE_low();	// RW = 0
		LCD_ENABLE_high();		// EN = 1
		_delay_us(100);
*/
		LCD_ENABLE_low();		// EN = 0
		_delay_us(100);
	}

//	LCD_EXECUTE_DATA_LAST();	// flush -> posleden simvol ili gore v cikala count_ns < numsymbols+1 ili count_ns <= numsymbols
}

/************************************************
** WRITE/TRANSMIT ONE DATA BYTE TO LCD DISPLAY **
************************************************/
void LCD_EXECUTE_DATA_ONE(unsigned char data)	// HELP: LCD_EXECUTE_DATA(unsigned char data_byte_symbols) // LCD shift left out, msb is first
{
	unsigned char storeMSB;
	unsigned char x;
	storeMSB = data;

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		LCD_SCK_low();

		if(storeMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
		{
			LCD_SDI_high();	// izvejdane na log "1" v MSB	// PORTA |= (1<<LCD_SDI_PIN);
		}
		else
		{
			LCD_SDI_low();	// izvejdane na log "0" v MSB	// PORTA &= ~(1<<LCD_SDI_PIN);
		}

		LCD_SCK_high();		

		storeMSB = storeMSB << 1;	// shiftvane na << nalqvo
	}
//update from down comment
	LCD_REGSELECT_high();	// RS = 1
	LCD_READWRITE_low();	// RW = 0
	LCD_ENABLE_high();		// EN = 1
//update from down comment

	LCD_RCK_low();
	_delay_us(170);			//rcall 180 us
	LCD_RCK_high();
	_delay_us(170);			//rcall 180 us
/*
	LCD_REGSELECT_high();	// RS = 1
	LCD_READWRITE_low();	// RW = 0
	LCD_ENABLE_high();		// EN = 1
	_delay_us(100);
*/
	LCD_ENABLE_low();		// EN = 0
	_delay_us(100);
}

/*****************************************************************************
** WRITE/TRANSMIT LAST DATA BYTE TO LCD DISPLAY AND FLUSH SN74HC595 BUFFER  **
*****************************************************************************/
void LCD_EXECUTE_DATA_LAST()	// flush -> izchistvane na bufer - prinuditelno izpisvane na posleden simvol
{
	LCD_ENABLE_low();
	LCD_READWRITE_low();
	LCD_REGSELECT_high();
	_delay_us(40);				// from 100 or 180 //rcall 180 us
	LCD_ENABLE_high();
	LCD_READWRITE_low();
	LCD_REGSELECT_high();
	_delay_us(40);				// from 100 or 180 //rcall 180 us
}

/****************************
*** NEW LCD FUNCTIONALITY ***
****************************/

/******************************************
** WRITE/TRANSMIT COMMAND TO LCD DISPLAY **
******************************************/
void LCD_COMMAND(unsigned char command)	// HELP: LCD_EXECUTE_COMMAND(unsigned char byte_choose_a_command) // LCD shift left out, msb is first
{
	unsigned char conbyte = command;
	unsigned char storeMSB;
	unsigned char x;
	storeMSB = conbyte;

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		LCD_SCK_low();

		if(storeMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
		{	
			LCD_SDI_high();	// izvejdane na log "1" v MSB	// PORTA |= (1<<LCD_SDI_PIN);
		}
		else
		{
			LCD_SDI_low();	// izvejdane na log "0" v MSB	// PORTA &= ~(1<<LCD_SDI_PIN);	
		}

		LCD_SCK_high();		

		storeMSB = storeMSB << 1;	// shiftvane na << nalqvo
	}
//update from down comment
	LCD_REGSELECT_low();	// RS = 0
	LCD_READWRITE_low();	// RW = 0
	LCD_ENABLE_high();		// EN = 1
//update from down comment
	LCD_RCK_low();
	_delay_us(170);			//rcall 180 us
	LCD_RCK_high();
	_delay_us(170);			//rcall 180 us

	LCD_ENABLE_low();		// EN = 0
	_delay_us(100);			//rcall 180 us
}

/*********************************************************
** WRITE/TRANSMIT MORE DATA STRING BYTES TO LCD DISPLAY **
*********************************************************/
void LCD_DATA_STRING(char *data)
{
// USE THIS: char symbols [] = "PESHO"; lcdExecuteDataString(symbols);
	while(*data)// != '\0')	// is there or isn't there ?? String Terminator last charachter '\0'
	{
		unsigned char storeMSB;
		unsigned char x;
		storeMSB = *data++;

		for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
		{
			LCD_SCK_low();

			if(storeMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
			{
				LCD_SDI_high();	// izvejdane na log "1" v MSB	// PORTA |= (1<<LCD_SDI_PIN);
			}
			else
			{
				LCD_SDI_low();	// izvejdane na log "0" v MSB	// PORTA &= ~(1<<LCD_SDI_PIN);
			}

			LCD_SCK_high();		

			storeMSB = storeMSB << 1;	// shiftvane na << nalqvo
		}
//update from down comment
		LCD_REGSELECT_high();	// RS = 1
		LCD_READWRITE_low();	// RW = 0
		LCD_ENABLE_high();		// EN = 1
//update from down comment
		LCD_RCK_low();
		_delay_us(170);			//rcall 180 us
		LCD_RCK_high();
		_delay_us(170);			//rcall 180 us
/*
		LCD_REGSELECT_high();	// RS = 1
		LCD_READWRITE_low();	// RW = 0
		LCD_ENABLE_high();		// EN = 1
		_delay_us(100);
*/
		LCD_ENABLE_low();		// EN = 0
		_delay_us(100);
	}
}

/******************************************************
** WRITE/TRANSMIT MORE DATA INT BYTES TO LCD DISPLAY **
******************************************************/
// USE THIS: lcdDataUInt( (unsigned int)k + 1 );
void LCD_DATA_UINT(unsigned int data)		// void lcdDataUInt(const unsigned int data)
{
	char buffer[10];
	LCD_DATA_STRING(utoa(data, buffer, 10));	// 10 -> DECIMAL
}

/******************************************************
** WRITE/TRANSMIT MORE DATA INT BYTES TO LCD DISPLAY **
******************************************************/
// USE THIS: lcdDataInt( (int)k + 1 );
void LCD_DATA_INT(int data)		// void lcdDataInt(const int data)
{
	char buffer[10];
	LCD_DATA_STRING(itoa(data, buffer, 10));	// 10 -> DECIMAL
}

/*******************************************************
** WRITE/TRANSMIT MORE DATA LONG BYTES TO LCD DISPLAY **
*******************************************************/
// USE THIS: lcdDataLong( (long)k + 1 );
void LCD_DATA_LONG(long data)		// void lcdDataLong(const long data)
{
	char buffer[10];
	LCD_DATA_STRING(ltoa(data, buffer, 10));	// 10 -> DECIMAL
}

/****************************************************************
** WRITE/TRANSMIT MORE DATA UNSIGNED LONG BYTES TO LCD DISPLAY **
****************************************************************/
// USE THIS: lcdDataULong( (unsigned long)k + 1 );
void LCD_DATA_ULONG(unsigned long data)		// void lcdDataULong(const unsigned long data)
{
	char buffer[10];
	LCD_DATA_STRING(ultoa(data, buffer, 10));	// 10 -> DECIMAL
}

/****************************************************
** DEFINITION LCD DISPLAY GENERATOR OF NEW SYMBOLS **
****************************************************/
int rows, cols;
unsigned char symbolGenerator[][LCD_CGRAM_SYMBOL_CONTAIN_8BYTES] =	// [rows][cols=8]
{
//     0x0E is FIRST TOP PATTERN BYTE, 		     0x1F is LAST BOTTOM PATTERN BYTE
	 { 0x0E, 0x1B, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F },	// Battery Charging   0%	// addr 0-7
	 { 0x0E, 0x1B, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F },	// Battery Charging  16%	// addr 8-15
	 { 0x0E, 0x1B, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F },	// Battery Charging  32%	// addr16-23
	 { 0x0E, 0x1B, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F },	// Battery Charging  48%	// addr24-31
	 { 0x0E, 0x1B, 0x11, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },	// Battery Charging  64%	// addr32-39
	 { 0x0E, 0x1B, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },	// Battery Charging  80%	// addr40-47
	 { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F }		// Battery Charging 100%	// addr48-55
};

void LCD_CGRAM_CUSTOM_SYMBOLS()
{
	for(rows=0; rows<7; rows++)
	{
		LCD_EXECUTE_COMMAND(LCD_CGRAM_STORE_ADDR_CHAR0+(rows*LCD_CGRAM_SYMBOL_CONTAIN_8BYTES));	// 0x40 = 0b0100000 SET CGRAM BASE 0 ADDRESS and OFFSET ADDRESS TO NEXT CHARACTER (LCD_CGRAM_STORE_ADDR_CHAR0+(row*8))
		for(int cols=0; cols<LCD_CGRAM_SYMBOL_CONTAIN_8BYTES; cols++)
		{
			LCD_EXECUTE_DATA_ONE(symbolGenerator[rows][cols]);
		}
	}
}
/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
