/*************************************************************************
*** LIBRARY: ROTATION ENCODER (ED1112S and more)             *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: rotation_encoder.c, v0.05, 28.11.2015         *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "rotation_encoder.h"
#include "lcd_hd44780_74hc595.h"	// for LCD_

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/*********************************
** DEFINITION ENCODER FUNCTIONS **
*********************************/

/*********************************************
** READ/SCAN ROTARY ENCODER OF NIK BARZAKOV **
*********************************************/
char rotaryEncoderNikBarzakov()
{
	char tempEncoder = 0;
	if((ENCODER_A_low()) && (ENCODER_B_low()))			// A0, B0
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_high()) && (ENCODER_B_low()))		// A1, B0
		{
			// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
// VOLUME UP
			tempEncoder = 1;	//	tempEncoder++;
			// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
		}
	}
	else if((ENCODER_A_high()) && (ENCODER_B_low()))	// A1, B0
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_low()) && (ENCODER_B_low()))		// A0, B0
		{
			// <--- Counter Clockwise; Zavartane po posoka obratno na chasovnikovata strelka.
// VOLUME DOWN
			tempEncoder = -1;	//	tempEncoder--;
			// <--- Counter Clockwise; Zavartane po posoka obratno na chasovnikovata strelka.
		}
	}
	else
	{
		// do nothing
	}
	return tempEncoder;
}

/*************************************************
** READ/SCAN ROTARY ENCODER VERSION 1 / MODEL 1 **
*************************************************/
char rotaryEncoderVer1()	// Check imediate now bits in PIN register.
{
	char tempEncoder = 0;							// A1, B1
	if((ENCODER_A_low()) && (ENCODER_B_high()))			// A0, B1
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_low()) && (ENCODER_B_low()))		// A0, B0
		{												// A1, B0
			tempEncoder = 1;
//			volumeIndex++;
//			PORTD = volumeMassive[volumeIndex];
			// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
		}

	}													// A1. B1
	else if((ENCODER_A_high()) && (ENCODER_B_low()))	// A1, B0
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_low()) && (ENCODER_B_low()))		// A0, B0
		{												// A0, B1
			tempEncoder = -1;
//			volumeIndex--;
//			PORTD = volumeMassive[volumeIndex];
			// <--- Counter Clockwise; Zavartane po posoka obratno na chasovnikovata strelka.
		}
	}
	else
	{
		// do nothing
	}
	return tempEncoder;
}

/*************************************************
** READ/SCAN ROTARY ENCODER VERSION 2 / MODEL 2 **
*************************************************/
char rotaryEncoderVer2()	// Read from PIN register and store 3bits to Buffres A and B, check Buffer with Encoder combination.
{
	char tempEncoder = 0;
	unsigned char x;
	unsigned char bufferREGA = 0;
	unsigned char bufferREGB = 0;	
	unsigned char PINA_IN = 0;	//	PINA = PINA_IN;	// PIN_IN is input pin, not variable
	
//	DDRA &= ~(1<<ENCODER_A);	// serial pin input
//	DDRA &= ~(1<<ENCODER_B);	// serial pin input

	for(x=5; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		bufferREGA = bufferREGA >> 1;	// shift bufferREGA to right one bit
		bufferREGA |= (PINA_IN & (1<<ENCODER_A)) << (x-ENCODER_A);	// copy bit serPinA of PORTA to MSB of bufferREGA

		bufferREGB = bufferREGB >> 1;	// shift bufferREGB to right one bit
		bufferREGB |= (PINA_IN & (1<<ENCODER_B)) << (x-ENCODER_B);	// copy bit serPinA of PORTA to MSB of bufferREGB
		_delay_us(50);	// delay before next check bits
	}

	if((bufferREGA == 0b10000000) && (bufferREGB == 0b00000000))	// 		if((bufferREGA == 0b00100000) && (bufferREGB == 0b10000000))
	{
		tempEncoder = 1;
//		tempEncoder++;
//		volumeIndex++;
//		PORTD = volumeMassive[volumeIndex];

		bufferREGA = bufferREGB = 0b00000000;
		// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
	}
	else if((bufferREGA == 0b00000000) && (bufferREGB == 0b10000000))	//		else if((bufferREGA == 0b10000000) && (bufferREGB == 0b00100000))
	{
		tempEncoder = -1;
//		tempEncoder--;
//		volumeIndex--;
//		PORTD = volumeMassive[volumeIndex];

		bufferREGA = bufferREGB = 0b00000000;
		// <--- Counter Clockwise; Zavartane po posoka obratno na chasovnikovata strelka.
	}
	else
	{
		// do nothing
	}
	return tempEncoder;
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
