/*************************************************************************
*** LIBRARY: LCD DISPLAY HITACHI HD44780 + SHIFT REGISTER 74HC595 ********
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: lcd_hd44780_74hc595.c, v5, 16.10.2015         *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "lcd_hd44780_74hc595.h"

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
	LCD_EXECUTE_COMMAND(LCD_8BIT_2ROWS_FONT5X7);		// 0b00111000	// 2. Function set: 8-bit interface data (DL = 1), 2-line display (N = 1), 5 x 7 dot character font (F = 0)
//	_delay_us(40);	// 37 uS	// comment for simulation
	LCD_EXECUTE_COMMAND(LCD_ON_BLINK_CURSOR);			// 0b00001111	// 3. Display on/off control: Display on (D = 1), Cursor on (C = 1), Blinking on (B = 1)
//	_delay_us(40);	// 37 uS	// comment for simulation
	LCD_EXECUTE_COMMAND(LCD_ENTRY_MODE_INC_NOSHIFT);	// 0b00000110	// 4. Entry mode set: Increment by 1 (I/D = 1), No shift (S = 0)
//	_delay_us(40);	// 37 uS	// comment for simulation
	LCD_EXECUTE_COMMAND(LCD_MOVE_FIRST);
//	_delay_us(40);	// 37 uS	// comment for simulation
	LCD_EXECUTE_COMMAND(LCD_CLEAR);						// 0b00000001	// 1. Display clear; // !!! ... from old code LCD_CLEAR can't be first command !!!
//	_delay_us(1600);	// 1.53 mS	// comment for simulation
}

/**************************************
** CLEAR ALL CONTAINS ON LCD DISPLAY **
**************************************/
void LCD_CLEAR_CONTAINS()
{
	LCD_EXECUTE_COMMAND(LCD_CLEAR);						// 0b00000001	// 1. Display clear; // !!! ... from old code LCD_CLEAR can't be first command !!!
	_delay_us(1600);	// 1.53 mS	or use this _delay_ms(2);
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

/*********************************************************
** WRITE/TRANSMIT MORE DATA STRING BYTES TO LCD DISPLAY **
*********************************************************/
void lcdDataString(char *data)
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
// USE THIS: lcdDataInt( (int)k + 1 );
void lcdDataInt(int data)		// void lcdDataInt(const int data)
{
	char buffer[10];
	lcdDataString(itoa(data, buffer, 10));	// 10 -> DECIMAL
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
