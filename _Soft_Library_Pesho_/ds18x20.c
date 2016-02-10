/*************************************************************************
*** LIBRARY: 1-wire (one wire) Interface (Transmit/Receive)  *************
*** AUTHOR:   http://www.mikrocontroller.net                 *************
*** FILE NAME: ds18x20.c, v3, 31.08.2015                     *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*** LINK: http://www.mikrocontroller.net/attachment/7853/ds18s20.c *******
*************************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include "ds18x20.h"

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/*********************************
** DEFINITION ENCODER FUNCTIONS **
*********************************/

unsigned char scratchpad [10] = {};

/*
void delay_us(unsigned short us)
{
    TCCR1B |= (1<<CS10);                //Prescaler 1 + Start des Timers
    TCNT1 = 0;
    while(TCNT1 < (XTAL*us));
}
*/

unsigned char reset(void)
{
    DDRx |= (1<<Px);            //Ausgang
	PORTx &= ~(1<<Px);
    _delay_us(480);
	DDRx &= ~(1<<Px);
    _delay_us(80);
	if(!(PINx & (1<<Px)))     //Prüfe Slave-Antwort
	{	
		_delay_us(450);
        return 1;
	}
    else
	{
        return 0;
	}
}

unsigned char read_bit(void)
{
    DDRx |= (1<<Px);            //Ausgang
	PORTx &= ~(1<<Px);
    _delay_us(1);
	DDRx &= ~(1<<Px);
    _delay_us(12);
    if(!(PINx & (1<<Px)))       //Abtastung innerhalb von 15µs
	{    
		return 0;
	}
	else
    {
		return 1;
	}
}

void write_bit(unsigned char bitval)    //kann 0 oder 1 sein
{
    DDRx |= (1<<Px);            //Ausgang
	PORTx &= ~(1<<Px);
    if(bitval)
	{
        PORTx |= (1<<Px);      //H-Pegel
    }
	_delay_us(110);        
    DDRx &= ~(1<<Px);
    PORTx &= ~(1<<Px);
}

unsigned char read_byte(void)
{
    unsigned char byte = 0;
    for(unsigned char i=0; i<8; i++)
	{
        if(read_bit ())
		{
            byte |= (1<<i);
		}
        _delay_us(120);
    }
    return byte;
}

void write_byte(unsigned char byte)
{
    for(unsigned char i=0; i<8; i++)
	{
        if(byte & (1<<i))
		{
			write_bit(1);
		}
        else
		{
            write_bit(0);
		}
	}
    _delay_us(120);
}  

unsigned char read_scratchpad(void)
{

	if(reset())
	{
		write_byte(0xCC);
		write_byte(0x44);
		wait_ready();
		if(reset())
		{
			write_byte(0xCC);
			write_byte(0xBE);
			for(unsigned char i=0; i<9; i++)
			{
				scratchpad [i] = read_byte();
			}
			return 1;
		}
	}
	return 0;
}

void wait_ready(void)
{
	while(!(read_bit()));
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
