/*************************************************************************
*** LIBRARY: LCD DISPLAY HITACHI HD44780 + SHIFT REGISTER 74HC595 ********
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: lcd_hd44780_74hc595.h, v4, 15.10.2015         *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _LCD_HD44780_74HC595_H_
#define _LCD_HD44780_74HC595_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/***************************
** DEFINITION LCD DISPLAY **
***************************/
#define LCD_ENABLE_PIN     PA3		// PORTA3 LCD ENABLE PIN
#define LCD_ENABLE_PORT    PORTA	// PORTA3 LCD ENABLE PIN
#define LCD_READWRITE_PIN  PA4		// PORTA6 LCD READ/WRITE PIN
#define LCD_READWRITE_PORT PORTA	// PORTA6 LCD READ/WRITE PIN
#define LCD_REGSELECT_PIN  PA5		// PORTA5 LCD REGISTER SELECT PIN
#define LCD_REGSELECT_PORT PORTA	// PORTA5 LCD REGISTER SELECT PIN
#define LCD_SDI_PIN        PA0		// PORTA0 LCD SDI - SERIAL DATA INPUT
#define LCD_SDI_PORT       PORTA	// PORTA0 LCD SDI - SERIAL DATA INPUT
#define LCD_SCK_PIN        PA1		// PORTA1 LCD SCK - SERIAL SHIFT CLOCK
#define LCD_SCK_PORT       PORTA	// PORTA1 LCD SCK - SERIAL SHIFT CLOCK
#define LCD_RCK_PIN        PA2		// PORTA2 LCD RCK - SERIAL LATCH CLOCK
#define LCD_RCK_PORT       PORTA	// PORTA2 LCD RCK - SERIAL LATCH CLOCK

#define LCD_ENABLE_low()		(LCD_ENABLE_PORT&=~_BV(LCD_ENABLE_PIN))
#define LCD_ENABLE_high()		(LCD_ENABLE_PORT|=_BV(LCD_ENABLE_PIN))
#define LCD_READWRITE_low()		(LCD_READWRITE_PORT&=~_BV(LCD_READWRITE_PIN))
#define LCD_READWRITE_high()	(LCD_READWRITE_PORT|=_BV(LCD_READWRITE_PIN))
#define LCD_REGSELECT_low()		(LCD_REGSELECT_PORT&=~_BV(LCD_REGSELECT_PIN))
#define LCD_REGSELECT_high()	(LCD_REGSELECT_PORT|=_BV(LCD_REGSELECT_PIN))
#define LCD_SDI_low()			(LCD_SDI_PORT&=~_BV(LCD_SDI_PIN))
#define LCD_SDI_high()			(LCD_SDI_PORT|=_BV(LCD_SDI_PIN))
#define LCD_SCK_low()			(LCD_SCK_PORT&=~_BV(LCD_SCK_PIN))
#define LCD_SCK_high()			(LCD_SCK_PORT|=_BV(LCD_SCK_PIN))
#define LCD_RCK_low()			(LCD_RCK_PORT&=~_BV(LCD_RCK_PIN))
#define LCD_RCK_high()			(LCD_RCK_PORT|=_BV(LCD_RCK_PIN))

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

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void LCD_INIT();
void LCD_EXECUTE_COMMAND(unsigned char command);
void LCD_EXECUTE_DATA(char data [], int numsymbols);
void LCD_EXECUTE_DATA_ONE(unsigned char data);
void LCD_EXECUTE_DATA_LAST();

void lcdDataString(char *data);
void lcdDataInt(int data);
void lcdCommand(char command);

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

#endif // _LCD_HD44780_74HC595_H_
