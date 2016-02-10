/*************************************************************************
*** LIBRARY: ROTATION ENCODER (ED1112S and more)             *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: rotation_encoder.c, v2, 02.09.2015            *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "rotation_encoder.h"

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/*********************************
** DEFINITION ENCODER FUNCTIONS **
*********************************/

/*********************************************
** READ/SCAN ROTARY ENCODER OF NIK BARZAKOV **
*********************************************/
char rotaryEncoderNikBarzakov(tempEncoder)
{
	if((ENCODER_A_low()) && (ENCODER_B_low()))			// A0, B0
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_high()) && (ENCODER_B_low()))		// A1, B0
		{
			// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
// VOLUME UP
			tempEncoder++;
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
			tempEncoder--;
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
void rotaryEncoderVer1()	// Check imediate now bits in PIN register.
{														// A1, B1
	if((ENCODER_A_low()) && (ENCODER_B_high()))			// A0, B1
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_low()) && (ENCODER_B_low()))		// A0, B0
		{												// A1, B0
			volumeIndex++;
			PORTD = volumeMassive[volumeIndex];
			// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
		}

	}													// A1. B1
	else if((ENCODER_A_high()) && (ENCODER_B_low()))	// A1, B0
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_low()) && (ENCODER_B_low()))		// A0, B0
		{												// A0, B1
			volumeIndex--;
			PORTD = volumeMassive[volumeIndex];
			// <--- Counter Clockwise; Zavartane po posoka obratno na chasovnikovata strelka.
		}
	}
	else
	{
			// do nothing
	}
}

/*************************************************
** READ/SCAN ROTARY ENCODER VERSION 2 / MODEL 2 **
*************************************************/
void rotaryEncoderVer2()	// Read from PIN register and store 3bits to Buffres A and B, check Buffer with Encoder combination.
{
	unsigned char x;
	unsigned char bufferREGA = 0;
	unsigned char bufferREGB = 0;	

//	DDRA &= ~(1<<ENCODER_A);	// serial pin input
//	DDRA &= ~(1<<ENCODER_B);	// serial pin input

	for(x=5; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		bufferREGA = bufferREGA >> 1;	// shift bufferREGA to right one bit
		bufferREGA |= (PINA & (1<<ENCODER_A)) << (x-ENCODER_A);	// copy bit serPinA of PORTA to MSB of bufferREGA

		bufferREGB = bufferREGB >> 1;	// shift bufferREGB to right one bit
		bufferREGB |= (PINA & (1<<ENCODER_B)) << (x-ENCODER_B);	// copy bit serPinA of PORTA to MSB of bufferREGB
		_delay_us(50);	// delay before next check bits
	}

	if((bufferREGA == 0b10000000) && (bufferREGB == 0b00000000))	// 		if((bufferREGA == 0b00100000) && (bufferREGB == 0b10000000))
	{
		volumeIndex++;
		PORTD = volumeMassive[volumeIndex];

		bufferREGA = bufferREGB = 0b00000000;
		// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
	}
	else if((bufferREGA == 0b00000000) && (bufferREGB == 0b10000000))	//		else if((bufferREGA == 0b10000000) && (bufferREGB == 0b00100000))
	{
		volumeIndex--;
		PORTD = volumeMassive[volumeIndex];

		bufferREGA = bufferREGB = 0b00000000;
		// <--- Counter Clockwise; Zavartane po posoka obratno na chasovnikovata strelka.
	}
	else
	{
		// do nothing
	}
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
