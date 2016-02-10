/*************************************************************************
*** LIBRARY: UART/USART (Transmit/Receive)                   *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: uart.h, v0.03, 29.11.2015                     *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one UART/USART *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _UART_H_
#define _UART_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/*****************************
** DEFINITION OF UART/USART **
*****************************/
/*
#define UART_RX_PIN    PD0		// PORTD0 UART/USART RECEIVE, INPUT RX PIN
#define UART_RX_PORT   PORTD	// PORTD0 UART/USART RECEIVE, INPUT RX PIN
#define UART_TX_PIN    PD1		// PORTD1 UART/USART TRANSMIT, OUTPUT TX PIN
#define UART_TX_PORT   PORTD	// PORTD1 UART/USART TRANSMIT, OUTPUT TX PIN

#define UART_RX_low()    (UART_RX_PORT&=~_BV(UART_RX_PIN))
#define UART_RX_high()   (UART_RX_PORT|=_BV(UART_RX_PIN))
#define UART_TX_low()    (UART_TX_PORT&=~_BV(UART_TX_PIN))
#define UART_TX_high()   (UART_TX_PORT|=_BV(UART_TX_PIN))
*/
/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void uart_init(void);
void uart_transmit(char uart_data [], int numsymbols);
void uart_transmit_one(unsigned char uart_data);
void uart_transmit_DEC_to_BCD(unsigned char rtc_data);

// NEW FUNCTIONS
void transmitUart(unsigned char data);
void transmitUartString(char *data);
void transmitUartUInt(unsigned int data);
void transmitUartInt(int data);
void transmitUartULong(unsigned long data);
void transmitUartLong(long data);
unsigned char receiveUart(void);

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _UART_H_
