/*************************************************************************
*** LIBRARY: I2C / IIC / ATmel Two Wire Interface            *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: i2c_twi.c, v0.02, 29.11.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one I2C / TWI  *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "i2c_twi.h"
#include "utility.h"		// using for debug and others

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/***********************************
** DEFINITION I2C / TWI FUNCTIONS **
***********************************/

/**************************
** INITIZLIZATION OF I2C **
***************************/
void i2c_init()
{
/*======================================|
|===== FORMULE FOR TWI (I2C) USING =====|
|=======================================|
|=======================================|
|==                                   ==|
|==                 Fcpu              ==|
|==   SCL = ---------------------     ==|
|==       (16+(2*TWBR*(4^TWPS))       ==|
|==                                   ==|
|=======================================|
|======= TABLE OF VALUES OF TWPS =======|
|==                                   ==|
|==          (4^TWPS) = value:        ==\_
|==                                       \======================================================================|
|== TWPS = 0 => (4^TWPS)= 1  // TWSR = (0<<TWPS1)|(0<<TWPS0) => TWSR = 0bxxxxxx00 => TWSR = 0 (DEC) => 4^0 =  1 =|
|== TWPS = 1 => (4^TWPS)= 4  // TWSR = (0<<TWPS1)|(1<<TWPS0) => TWSR = 0bxxxxxx01 => TWSR = 1 (DEC) => 4^1 =  4 =|
|== TWPS = 2 => (4^TWPS)=16  // TWSR = (1<<TWPS1)|(0<<TWPS0) => TWSR = 0bxxxxxx10 => TWSR = 2 (DEC) => 4^2 = 16 =|
|== TWPS = 3 => (4^TWPS)=64  // TWSR = (1<<TWPS1)|(1<<TWPS0) => TWSR = 0bxxxxxx11 => TWSR = 3 (DEC) => 4^3 = 64 =|
|===============================================================================================================*/

	TWBR = 0b00010010;	// HEX=0x12	or DEC=18	// FCPU = 16 000 000 |-> SCL = 16000000 / (16+(2*TWBR*(4^TWPS)) = 16000000 / (16+(2*18*(4))) = 16000000 / 160 = 100 000 Hz = 100 kHz
	TWSR = (0<<TWPS1)|(1<<TWPS0);	// SCL = 16 000 000 / 64  = 250 000 = 250kHz
	TWCR = (1<<TWEN);	//	TWCR = 0x04; // TWCR = (x<<TWINT)|(x<<TWEA)|(x<<TWSTA)|(x<<TWSTO)|(x<<TWEN)|(x<<TWIE);	//0b01010010	// SPR0,1 = 1 - 16 000 000 / 128 = 125 000 = 125kHz
	TWDR = 0b00000000;
}

/*****************************************
** START CONDITION OF I2C COMMUNICATION **
*****************************************/
void i2c_start(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while((TWCR & (1<<TWINT)) == 0)
	{
	}
}

/****************************************
** STOP CONDITION OF I2C COMMUNICATION **
****************************************/
void i2c_stop()
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

/*******************************************
** WRITE/SEND 1 BYTE OF I2C COMMUNICATION **
*******************************************/
void i2c_write(unsigned char data)
{
	TWDR = data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while((TWCR & (1<<TWINT)) == 0)
	{
	}
}

/**********************************************
** READ/RECEIVE BYTE(S) OF I2C COMMUNICATION **
**********************************************/
unsigned char i2c_read(unsigned char isLast)
{
	if(isLast == 0 )
	{
		TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);	// to next receive byte
	}
	else
	{
		TWCR = (1<<TWINT)|(1<<TWEN);			// to end of communication
	}
	
	while((TWCR & (1<<TWINT)) == 0)
	{
	}
	return TWDR;
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
