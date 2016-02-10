/*************************************************************************
*** LIBRARY: LCD DISPLAY HITACHI HD44780                     *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: lcd_hd44780.h, v0.01, 18.10.2015              *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _LCD_HD44780_H_
#define _LCD_HD44780_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/********************************
** DEFINITION LCD DISPLAY PORT **
********************************/
// unsigned char LCDPORT;
// PORTA = LCDPORT
#define	LCDPORT				PORTA	// DEFINITION PORT 

/***************************
** DEFINITION LCD DISPLAY **
***************************/
#define LCD_ENABLE_PIN		PA3		// PORTA3 LCD ENABLE PIN
#define LCD_ENABLE_PORT		PORTA	// PORTA3 LCD ENABLE PIN
#define LCD_READWRITE_PIN	PA4		// PORTA6 LCD READ/WRITE PIN
#define LCD_READWRITE_PORT	PORTA	// PORTA6 LCD READ/WRITE PIN
#define LCD_REGSELECT_PIN	PA5		// PORTA5 LCD REGISTER SELECT PIN
#define LCD_REGSELECT_PORT	PORTA	// PORTA5 LCD REGISTER SELECT PIN

#define LCD_ENABLE_low()		(LCD_ENABLE_PORT&=~_BV(LCD_ENABLE_PIN))
#define LCD_ENABLE_high()		(LCD_ENABLE_PORT|=_BV(LCD_ENABLE_PIN))
#define LCD_READWRITE_low()		(LCD_READWRITE_PORT&=~_BV(LCD_READWRITE_PIN))
#define LCD_READWRITE_high()	(LCD_READWRITE_PORT|=_BV(LCD_READWRITE_PIN))
#define LCD_REGSELECT_low()		(LCD_REGSELECT_PORT&=~_BV(LCD_REGSELECT_PIN))
#define LCD_REGSELECT_high()	(LCD_REGSELECT_PORT|=_BV(LCD_REGSELECT_PIN))

/*************************************
** DEFINITION LCD DISPLAY CONSTANTS **
*************************************/
// access 0b00000001
#define LCD_CLEAR				0b00000001		// LCD DISPLAY CLEAR 
//const byte LCD_CLEAR			= 0b00000001;
// access 0b0000001x
#define LCD_MOVE_FIRST			0b00000010		// LCD DISPLAY MOVE CURSOR TO FIRST ROW AND FIRST SYMBOL (SEGMENT)
//const byte LCD_MOVE_FIRST		= 0b00000010;
// access 0b00001xxx
#define LCD_ENTRY_MODE_INC_NOSHIFT		0b00000110	// LCD ENTRY MODE INCREMENT by 1 AND NO SHIFT
//const byte LCD_ENTRY_MODE_INC_NOSHIFT	= 0b00000010;
#define LCD_ENTRY_MODE_DEC_NOSHIFT		0b00000100	// LCD ENTRY MODE DECREMENT by 1 AND NO SHIFT
//const byte LCD_ENTRY_MODE_DEC_NOSHIFT	= 0b00000010;
#define LCD_ENTRY_MODE_INC_SHIFT		0b00000111	// LCD ENTRY MODE INCREMENT by 1 AND SHIFT
//const byte LCD_ENTRY_MODE_INC_SHIFT	= 0b00000010;
#define LCD_ENTRY_MODE_DEC_SHIFT		0b00000101	// LCD ENTRY MODE DECREMENT by 1 AND SHIFT
//const byte LCD_ENTRY_MODE_DEC_SHIFT	= 0b00000010;
// access 0b000001xx
#define LCD_OFF					0b00001000		// LCD DISPLAY OFF 
//const byte LCD_OFF			= 0b00001000;
#define LCD_ON					0b00001100		// LCD DISPLAY ON without CURSOR
//const byte LCD_ON				= 0b00001100;
#define LCD_ON_WITH_CURSOR		0b00001110		// LCD DISPLAY ON with CURSOR
//const byte LCD_ON_WITH_CURSOR	= 0b00001110;
#define LCD_ON_BLINK_CURSOR		0b00001111		// LCD DISPLAY ON with BLINKing CURSOR 
//const byte LCD_ON_BLINK_CURSOR= 0b00001111;
// access 0b001xxxxx
#define LCD_8BIT_1ROW_FONT5X7	0b00110000		// LCD DISPLAY SELECT 8 DATA BITS, CHOOSE 1 ROW LCD, SELECT FONT 5x7
//const byte LCD_8BIT_1ROW_FONT5X7	= 0b00110000;
#define LCD_8BIT_1ROW_FONT5X10	0b00110100		// LCD DISPLAY SELECT 8 DATA BITS, CHOOSE 1 ROW LCD, SELECT FONT 5x10
//const byte LCD_8BIT_1ROW_FONT5X10	= 0b00110100;
#define LCD_8BIT_2ROWS_FONT5X7	0b00111000		// LCD DISPLAY SELECT 8 DATA BITS, CHOOSE 2 ROW LCD, SELECT FONT 5x7
//const byte LCD_8BIT_2ROWS_FONT5X7	= 0b00111000;
#define LCD_8BIT_2ROWS_FONT5X10	0b00111100		// LCD DISPLAY SELECT 8 DATA BITS, CHOOSE 2 ROW LCD, SELECT FONT 5x10
//const byte LCD_8BIT_2ROWS_FONT5X10= 0b00111100;
// access 0b00000000 ROW 1
#define LCD_SELECT_1ROW			0b00000000		// LCD DISPLAY SELECT FIRST ROW TO VISUALIZING
//const byte LCD_SELECT_1ROW	= 0b00000000;
//const byte LCD_SELECT_1ROW	= 0b10000000;	// 0x80, 0x81, 0x82, ... 0x8F (for 16 column), ... 0x93 (for 20 column), ... 0xA7 (for 40 column)
// access 0b11000000 ROW 2
#define LCD_SELECT_2ROW			0b11000000		// LCD DISPLAY SELECT SECOND ROW TO VISUALIZING
//const byte LCD_SELECT_2ROW	= 0b11000000;	// 0xC0, 0xC1, 0xC2, ... 0xCF (for 16 column), ... 0xD3 (for 20 column), ... 0xE7 (for 40 column)
// access 0bb10010100 ROW 3
#define LCD_SELECT_3ROW			0b10010100		// LCD DISPLAY SELECT SECOND ROW TO VISUALIZING
//const byte LCD_SELECT_3ROW	= 0b10010100;	// 0x94, 0x95, 0x96, ... 0xA7 (for 20 column)
// access 0bb10010100 ROW 4
#define LCD_SELECT_4ROW			0b11010100		// LCD DISPLAY SELECT SECOND ROW TO VISUALIZING
//const byte LCD_SELECT_4ROW	= 0b11010100;	// 0xD4, 0xD5, 0xD6, ... 0xE7 (for 20 column)

/*********************************************************************
** DEFINITION LCD DISPLAY STORE ADDRES FOR GENERATOR OF NEW SYMBOLS **
*********************************************************************/
#define LCD_CGRAM_STORE_ADDR_CHAR_MIN	0b01000000	// 0x40	// ADDRESS BASE - first bit of CGRAM, Every CHAR contains 8 bytes with pattern pixels
#define LCD_CGRAM_STORE_ADDR_CHAR0		0b01000000	// 0x40	// ADDRESS for CHAR0 - 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47
#define LCD_CGRAM_STORE_ADDR_CHAR1		0b01001000	// 0x48	// ADDRESS for CHAR1 - 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F
#define LCD_CGRAM_STORE_ADDR_CHAR2		0b01010000	// 0x50	// ADDRESS for CHAR2 - 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57
#define LCD_CGRAM_STORE_ADDR_CHAR3		0b01011000	// 0x58	// ADDRESS for CHAR3 - 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F
#define LCD_CGRAM_STORE_ADDR_CHAR4		0b01100000	// 0x60	// ADDRESS for CHAR4 - 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67
#define LCD_CGRAM_STORE_ADDR_CHAR5		0b01101000	// 0x68	// ADDRESS for CHAR5 - 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F
#define LCD_CGRAM_STORE_ADDR_CHAR6		0b01110000	// 0x77	// ADDRESS for CHAR6 - 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77
#define LCD_CGRAM_STORE_ADDR_CHAR7		0b01111000	// 0x77	// ADDRESS for CHAR7 - 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F
#define LCD_CGRAM_STORE_ADDR_CHAR_MAX	0b01111111	// 0x7F	// ADDRESS END - lastest bit of CGRAM

/********************************************************************
** DEFINITION LCD DISPLAY ARRAY STORE FOR GENERATOR OF NEW SYMBOLS **
********************************************************************/
#define LCD_CGRAM_SYMBOL_CONTAIN_8BYTES	8	// every symbol contains 8 bytes pixel pattern
#define LCD_CGRAM_NUMBER_CHARACTERS		7	// every symbol is one row, every row is generated symbol

/*
typedef enum
{						//Bit:  43210 - Hex
	value_byte0 = 0x0E,	// 0b00001110
	value_byte1 = 0x1B,	// 0b00011011
	value_byte2 = 0x11,	// 0b00010001
	value_byte3 = 0x11,	// 0b00010001
	value_byte4 = 0x11,	// 0b00010001
	value_byte5 = 0x11,	// 0b00010001
	value_byte6 = 0x11,	// 0b00010001
	value_byte7 = 0x1F	// 0b00011111
} battery0percenCharging // Battery Charging   0%

typedef enum
{						//Bit:  43210 - Hex
	value_byte0 = 0x0E,	// 0b00001110
	value_byte1 = 0x1B,	// 0b00011011
	value_byte2 = 0x11,	// 0b00010001
	value_byte3 = 0x11,	// 0b00010001
	value_byte4 = 0x11,	// 0b00010001
	value_byte5 = 0x11,	// 0b00010001
	value_byte6 = 0x1F,	// 0b00011111
	value_byte7 = 0x1F	// 0b00011111
} battery16percenCharging // Battery Charging   16%

typedef enum
{						//Bit:  43210 - Hex
	value_byte0 = 0x0E,	// 0b00001110
	value_byte1 = 0x1B,	// 0b00011011
	value_byte2 = 0x11,	// 0b00010001
	value_byte3 = 0x11,	// 0b00010001
	value_byte4 = 0x11,	// 0b00010001
	value_byte5 = 0x1F,	// 0b00011111
	value_byte6 = 0x1F,	// 0b00011111
	value_byte7 = 0x1F	// 0b00011111
} battery32percenCharging // Battery Charging   32%

typedef enum
{						//Bit:  43210 - Hex
	value_byte0 = 0x0E,	// 0b00001110
	value_byte1 = 0x1B,	// 0b00011011
	value_byte2 = 0x11,	// 0b00010001
	value_byte3 = 0x11,	// 0b00010001
	value_byte4 = 0x1F,	// 0b00011111
	value_byte5 = 0x1F,	// 0b00011111
	value_byte6 = 0x1F,	// 0b00011111
	value_byte7 = 0x1F	// 0b00011111
} battery48percenCharging // Battery Charging   48%

typedef enum
{						//Bit:  43210 - Hex
	value_byte0 = 0x0E,	// 0b00001110
	value_byte1 = 0x1B,	// 0b00011011
	value_byte2 = 0x11,	// 0b00010001
	value_byte3 = 0x1F,	// 0b00011111
	value_byte4 = 0x1F,	// 0b00011111
	value_byte5 = 0x1F,	// 0b00011111
	value_byte6 = 0x1F,	// 0b00011111
	value_byte7 = 0x1F	// 0b00011111
} battery64percenCharging // Battery Charging   64%

typedef enum
{						//Bit:  43210 - Hex
	value_byte0 = 0x0E,	// 0b00001110
	value_byte1 = 0x1B,	// 0b00011011
	value_byte2 = 0x1F,	// 0b00011111
	value_byte3 = 0x1F,	// 0b00011111
	value_byte4 = 0x1F,	// 0b00011111
	value_byte5 = 0x1F,	// 0b00011111
	value_byte6 = 0x1F,	// 0b00011111
	value_byte7 = 0x1F	// 0b00011111
} battery80percenCharging // Battery Charging   80%

typedef enum
{						//Bit:  43210 - Hex
	value_byte0 = 0x0E,	// 0b00001110
	value_byte1 = 0x1F,	// 0b00011111
	value_byte2 = 0x1F,	// 0b00011111
	value_byte3 = 0x1F,	// 0b00011111
	value_byte4 = 0x1F,	// 0b00011111
	value_byte5 = 0x1F,	// 0b00011111
	value_byte6 = 0x1F,	// 0b00011111
	value_byte7 = 0x1F	// 0b00011111
} battery80percenCharging // Battery Charging   100%

typedef enum
{						//Bit:  43210 - Hex ???????????????????
	value_byte0 = 0x02,	// 0b00000010
	value_byte1 = 0x03,	// 0b00000011
	value_byte2 = 0x02,	// 0b00000010
	value_byte3 = 0x02,	// 0b00000010
	value_byte4 = 0x0E,	// 0b00001110
	value_byte5 = 0x1E,	// 0b00011110
	value_byte6 = 0x0C,	// 0b00000110
	value_byte7 = 0x00	// 0b00000000
} melodyNote // melodyNote

typedef enum
{						//Bit:  43210 - Hex
	value_byte0 = 0x04,	// 0b00000100
	value_byte1 = 0x0E,	// 0b00001110
	value_byte2 = 0x0E,	// 0b00001110
	value_byte3 = 0x0E,	// 0b00001110
	value_byte4 = 0x1F,	// 0b00011111
	value_byte5 = 0x0F,	// 0b00000000
	value_byte6 = 0x04,	// 0b00000100
	value_byte7 = 0x00	// 0b00000000
} bell // bell
*/
/*
     Bit: 4 3 2 1 0 - Hex
    Row1: 0 0 1 0 0 - 0x04
    Row2: 0 1 1 1 0 - 0x0E
    Row3: 0 1 1 1 0 - 0x0E
    Row4: 0 1 1 1 0 - 0x0E
    Row5: 1 1 1 1 1 - 0x1F
    Row6: 0 0 0 0 0 - 0x00
    Row7: 0 0 1 0 0 - 0x04
    Row8: 0 0 0 0 0 - 0x00

// http://www.circuitvalley.com/2012/02/lcd-custom-character-hd44780-16x2.html
// http://www.spikenzielabs.com/SpikenzieLabs/LCD_How_To.html
// http://www.imagesco.com/articles/lcd/06.html
// http://ftp1.digi.com/support/documentation/0220057_b.pdf
// http://www.quinapalus.com/hd44780udg.html
// http://omerk.github.io/lcdchargen/
// http://mikeyancey.com/hamcalc/lcd_characters.php

extern unsigned char symbolGenerator[][8] =	// [rows][cols=8]
{
	 { 0x0E, 0x1B, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F },	// Battery Charging   0%	// addr 0-7
	 { 0x0E, 0x1B, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F },	// Battery Charging  16%	// addr 8-15
	 { 0x0E, 0x1B, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F },	// Battery Charging  32%	// addr16-23
	 { 0x0E, 0x1B, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F },	// Battery Charging  48%	// addr24-31
	 { 0x0E, 0x1B, 0x11, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },	// Battery Charging  64%	// addr32-39
	 { 0x0E, 0x1B, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },	// Battery Charging  80%	// addr40-47
	 { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F }		// Battery Charging 100%	// addr48-55
};

// OR

unsigned char *generationSymbols[BUFFER_NUMBER_OF_CHARS] =
{
	 0b00001110, 0b00011011, 0b00010001, 0b00010001, 0b00010001, 0b00010001, 0b00010001, 0b00011111,	// Battery Charging   0%
	 0b00001110, 0b00011011, 0b00010001, 0b00010001, 0b00010001, 0b00010001, 0b00011111, 0b00011111,	// Battery Charging  16%
	 0b00001110, 0b00011011, 0b00010001, 0b00010001, 0b00010001, 0b00011111, 0b00011111, 0b00011111,	// Battery Charging  32%
	 0b00001110, 0b00011011, 0b00010001, 0b00010001, 0b00011111, 0b00011111, 0b00011111, 0b00011111,	// Battery Charging  48%
	 0b00001110, 0b00011011, 0b00010001, 0b00011111, 0b00011111, 0b00011111, 0b00011111, 0b00011111,	// Battery Charging  64%
	 0b00001110, 0b00011011, 0b00011111, 0b00011111, 0b00011111, 0b00011111, 0b00011111, 0b00011111,	// Battery Charging  80%
	 0b00001110, 0b00011111, 0b00011111, 0b00011111, 0b00011111, 0b00011111, 0b00011111, 0b00011111		// Battery Charging 100%
}

	LCD_EXECUTE_COMMAND(LCD_CGRAM_STORE_ADDR_CHAR0);	// 0x40 = 0b0100000 SET CGRAM BASE 0 ADDRESS
	// BATTERY CHARGED 0%
	LCD_EXECUTE_DATA_ONE(0x0E);		// byte 0 send
	LCD_EXECUTE_DATA_ONE(0x1B);		// byte 1 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 2 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 3 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 4 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 5 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 6 send
	LCD_EXECUTE_DATA_ONE(0x1F);		// byte 7 send

	// BATTERY CHARGED 16%
	LCD_EXECUTE_COMMAND(LCD_CGRAM_STORE_ADDR_CHAR1);
	LCD_EXECUTE_DATA_ONE(0x0E);		// byte 0 send
	LCD_EXECUTE_DATA_ONE(0x1B);		// byte 1 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 2 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 3 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 4 send
	LCD_EXECUTE_DATA_ONE(0x11);		// byte 5 send
	LCD_EXECUTE_DATA_ONE(0x1F);		// byte 6 send
	LCD_EXECUTE_DATA_ONE(0x1F);		// byte 7 send
*/


/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void LCD_INIT();
void LCD_CLEAR_CONTAINS();
void LCD_EXECUTE_COMMAND(unsigned char command);
void LCD_EXECUTE_DATA(char data [], int numsymbols);
void LCD_EXECUTE_DATA_ONE(unsigned char data);
void LCD_EXECUTE_DATA_LAST();

void lcdDataString(char *data);
void lcdDataInt(int data);
void lcdCommand(char command);

void LCD_CGRAM_CUSTOM_SYMBOLS();	// store new generated chars from array 

/*
void uart_init();
void uart_transmit(char uart_data [], int numsymbols);
void uart_transmit_one(unsigned char uart_data);
void uart_transmit_DEC_to_BCD(unsigned char rtc_data);

// NEW FUNCTIONS
void transmitUart(unsigned char data);
void transmitUartString(char *data);
void transmitUartInt(int data);
unsigned char receiveUart(void);
*/

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _LCD_HD44780_H_
