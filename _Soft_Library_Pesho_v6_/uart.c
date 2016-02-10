/*************************************************************************
*** LIBRARY: UART/USART (Transmit/Receive)                   *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: uart.c, v3, 31.08.2015                        *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one UART/USART *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>
#include <stdlib.h>			// utoa(), itoa(), ultoa(), ltoa(), - function
#include "uart.h"

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/***************************************
** DEFINITION OF UART/USART FUNCTIONS **
***************************************/

/*********************************
** INITIZLIZATION OF UART/USART **
*********************************/
void uart_init()
{
	UBRRL = 103;			// 9600, 0, 0 (Error = 0.2%; 16MHz)
	UBRRH = 0;

	UCSRC = 0b10000110;		// URSEL = 1; UCSZ1 = 1; UCSZ0 = 1; 8-bit
	UCSRB = 0b10011000;		// TXEN,RXEN,RXCIE
	UDR = 0b00000000;		// INITIALIZATION NULL OF UART DATA
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
