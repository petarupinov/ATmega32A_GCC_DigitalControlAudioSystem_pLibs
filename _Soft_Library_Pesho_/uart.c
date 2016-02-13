/*************************************************************************
*** LIBRARY: UART/USART (Transmit/Receive)                   *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: uart.c, v0.03, 29.11.2015                     *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one UART/USART *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>
#include <stdlib.h>			// utoa(), itoa(), ultoa(), ltoa(), - function
#include "../ATmega32A_GCC_DigitalControlAudioSystem_pLibs.h"
#include "uart.h"
#include "lcd_hd44780_74hc595.h"	// print using MCU Frequency
#include "utility.h"		// using for debug and others

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/***************************************
** DEFINITION OF UART/USART FUNCTIONS **
***************************************/
/*
enum
{
	
}
*/
/*********************************
** INITIZLIZATION OF UART/USART **
*********************************/
void uart_init(void)
{
	switch(F_CPU)
	{
		case 16000000UL:
		{
			UBRRL = 103;	// Baudrate: 9600; (Error = 0.2%; 16MHz)
			UBRRH = 0;
			break;
		}
		case 8000000UL:
		{
			UBRRL = 51;		// Baudrate: 9600; (Error = 0.2%; 8MHz)
			UBRRH = 0;
			break;
		}
		case 4000000UL:
		{
			UBRRL = 25;		// Baudrate: 9600; (Error = 0.2%; 4MHz)
			UBRRH = 0;		
			break;
		}
		case 2000000UL:
		{
			UBRRL = 12;		// Baudrate: 9600; (Error = 0.2%; 2MHz)
			UBRRH = 0;
			break;
		}
		case 1000000UL:
		{
			UBRRL = 6;		// Baudrate: 9600; (Error = -7.0%; 1MHz)
			UBRRH = 0;
			break;
		}
		default:
		{
			break;
		}
	}
#ifdef DEBUG_SETTING
	LCD_COMMAND(LCD_ON);						// LCD ON without CURSOR
	LCD_COMMAND(LCD_SELECT_4ROW);	// select row 2								// and next is update volume lcd information
	LCD_DATA_STRING("Freq: ");		// 20 symbols			
	LCD_DATA_ULONG(F_CPU);			// 20 symbols
	LCD_DATA_STRING(" MHz");		// 20 symbols
#endif
	// Razpoznavane na baudrate (skorost): 1. Izprashta se byte. 2. Poluchava se byte. 3. Sravnqva se polucheniq byte == izprateniq byte. // This is LOOP TX->RX
	// 4. Ako byte pri sravnqvaneto e edin i sasht to skorostta e izbranata v momenta, ako byte e razlichen - da se probva sas sledvashta baudrate.

	UCSRC = 0b10000110;		// URSEL = 1 (Accessing to UBRRH or UCSRC, is read as zero when reading UBRRH. The URSEL must be zero when writing the UBRRH.); UMSEL = 0 (Asynchronous Operation); UPM1 = 0, UPM0 = 0 (Parity Mode Disabled); USBS = 0 (1-Stop Bit); UCSZ2 = 0, UCSZ1 = 1, UCSZ0 = 1 (8-DataBits); UCPOL = 0 Polarity TX & RX (Rising XCK Edge -> Transmitted Data Changed (Output of TxDPin), Falling XCK Edge -> Received Data Sampled (Input on RxDPin))
	UCSRB = 0b00001000;		// Enable only TXEN	- Transmitting Uart/Usart	// UCSRB = 0b10011000;		// Enable TXEN,RXEN,RXCIE	Transmitting and Receiving Uart/Usart
	UDR = 0b00000000;		// INITIALIZATION NULL OF UART DATA

#ifdef DEBUG_INFO
	transmitUartString("\r\n");
#endif
#if DEBUG_SETTING
	transmitUartString("[UART INFO] Serial Port Settings - Baud rate: 9600, Data bits: 8 bits, Stop bits: 1 bit, Parity: None, Flow control: None or XON/XOFF\r\n");
#endif
}

/*******************************************
** TRANSMIT/SEND MORE BYTES OF UART/USART **
*******************************************/
void uart_transmit(char uart_data [], int numsymbols)
{
	for(int count_ns = 0; count_ns < numsymbols; count_ns++)	// count_ns < numsymbols -> za string nqma nijda ot dobavqne na +1 za posledniq simvol ili ot smqna na znaka <=
	{
		//UDR = uart_data[count_ns];
		while(!(UCSRA & (1<<UDRE)))	// cikal za proverka na gotovnost - dali e izpraten simbola i v gotovnost li e za nov simvol
		{
		}
		UDR = uart_data[count_ns];
	}
}

/***************************************
** TRANSMIT/SEND 1 BYTE OF UART/USART **
***************************************/
void uart_transmit_one(unsigned char uart_data)
{
	while(!(UCSRA & (1<<UDRE)))	// cikal za proverka na gotovnost - dali e izpraten simbola i v gotovnost li e za nov simvol
	{
	}
	UDR = uart_data;
}

/*****************************************************************		// NOT FINISHED
** TRANSMIT/SEND 1 BYTE CONVERTED FROM DEC TO BCD OF UART/USART **		// NOT FINISHED
*****************************************************************/		// NOT FINISHED
void uart_transmit_DEC_to_BCD(unsigned char dec_to_bcd_data)
{// pravi se preobrazuvane ot DEC to BCD i preobrazuvane za izvejdane na parviq simvol s filtar
	
	while(!(UCSRA & (1<<UDRE)))	// cikal za proverka na gotovnost - dali e izpraten simbola i v gotovnost li e za nov simvol
	{
	}
	UDR = ('0'+ (dec_to_bcd_data>>4));

	while(!(UCSRA & (1<<UDRE)))	// cikal za proverka na gotovnost - dali e izpraten simbola i v gotovnost li e za nov simvol
	{
	}
	UDR = ('0'+ (dec_to_bcd_data & 0x0F));

/*
// Convert normal decimal numbers to binary coded decimal
	byte decToBcd(byte val)
	{
  		return ( (val/10*16) + (val%10) );
	}

// Convert binary coded decimal to normal decimal numbers
	byte bcdToDec(byte val)
	{
  		return ( (val/16*10) + (val%16) );
	}
*/
}

// NEW FUNCTIONS

/***************************************		// NOT FINISHED
** TRANSMIT/SEND 1 BYTE OF UART/USART **		// NOT FINISHED
***************************************/		// NOT FINISHED
// USE THIS: uart_puts(" = "); OR transmitUart(data); data++; transmitUart(data);
void transmitUart(unsigned char data)
{
	while (!(UCSRA & (1<<UDRE)))
	{};							/* Wait for empty transmit buffer */
	UDR = data;					/* Put data into buffer, sends the data */
}

/**************************************************		// NOT FINISHED
** TRANSMIT/SEND MORE STRING BYTES OF UART/USART **		// NOT FINISHED
**************************************************/		// NOT FINISHED
// USE THIS: char symbols [] = "PESHO"; transmitUartString(symbols);
void transmitUartString(char *data)		// void transmitUartString(const char *data)
{
	while(*data)
	{
		transmitUart(*data++);
	}
}

/**************************************************		// NOT FINISHED
** TRANSMIT/SEND unsigned int BYTE OF UART/USART **		// NOT FINISHED
**************************************************/		// NOT FINISHED
// USE THIS: transmitUartUInt( (unsigned int)k + 1 );
void transmitUartUInt(unsigned int data)		// void transmitUartUInt(const unsigned int data)
{
	char buffer[10];
	transmitUartString(utoa(data, buffer, 10));		// 10 -> DECIMAL
}

/*****************************************		// NOT FINISHED
** TRANSMIT/SEND int BYTE OF UART/USART **		// NOT FINISHED
*****************************************/		// NOT FINISHED
// USE THIS: transmitUartInt( (int)k + 1 );
void transmitUartInt(int data)		// void transmitUartInt(const int data)
{
	char buffer[10];
	transmitUartString(itoa(data, buffer, 10));		// 10 -> DECIMAL
}

/***************************************************		// NOT FINISHED
** TRANSMIT/SEND unsigned long BYTE OF UART/USART **		// NOT FINISHED
***************************************************/		// NOT FINISHED
// USE THIS: transmitUartULong( (unsigned long)k + 1 );
void transmitUartULong(unsigned long data)		// void transmitUartULong(const unsigned long data)
{
	char buffer[10];
	transmitUartString(ultoa(data, buffer, 10));		// 10 -> DECIMAL
}

/******************************************		// NOT FINISHED
** TRANSMIT/SEND long BYTE OF UART/USART **		// NOT FINISHED
******************************************/		// NOT FINISHED
// USE THIS: transmitUartLong( (long)k + 1 );
void transmitUartLong(long data)		// void transmitUartLong(const long data)
{
	char buffer[10];
	transmitUartString(ltoa(data, buffer, 10));		// 10 -> DECIMAL
}

/********************************************		// NOT FINISHED
** TRANSMIT/SEND double TYPE OF UART/USART **		// NOT FINISHED
********************************************/		// NOT FINISHED
// USE THIS: transmitUartInt( (int)k + 1 );
void transmitUartDouble(double data)		// void transmitUartInt(const int data)
{
	char buffer[10];
	transmitUartString(itoa(data, buffer, 10));		// 10 -> DECIMAL
}

/**************************************************		// NOT FINISHED
** RECEIVE/READ unsigned char BYTE OF UART/USART **		// NOT FINISHED
**************************************************/		// NOT FINISHED
unsigned char receiveUart(void)
{
	while (!(UCSRA & (1<<RXC)))
	{};							/* Wait for data to be received */
	return UDR;					/* Get and return received data from buffer */
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
