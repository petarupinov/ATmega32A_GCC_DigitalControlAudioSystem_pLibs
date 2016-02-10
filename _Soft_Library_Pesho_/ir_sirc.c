/*************************************************************************
*** LIBRARY: SIRC - SONY INFRARED DECODER (TSOP2240 and more) ************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: ir_sirc.c, v0.03, 04.12.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "ir_sirc.h"
#include "utility.h"		// using for debug and others, byte is custom defined type

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/************************************************
** SCAN SIRC 12-bit DEVICE ADDRESS AND COMMAND **
************************************************/
void GetSIRC12(void)
{
	char x;
	char lTime;

//StartLook:
	irAddress = irCommand = 0;

	while(irPin);				//wait for it to be low
	lTime = 0;					//reset the counter

	while(irPin == 0)			//while the pin is low which is our pulse count
    {
		lTime++;				//increment every 200uS until pin is high
		_delay_us(200);			//200uS delay
	}

	if((lTime <= 10) || (lTime >= 14))	//Start too short or long and restart
	{
		return;		//goto StartLook;
	}
	else
	{
		lTime = 0;
		for(x=0;x<7;x++)			//repeat 7 times for command
		{
			irCommand >>= 1;			//if it was skipped or is done ORing then shift over the 1

			while(irPin);			//wait for it to be low
			lTime = 0;				//reset the counter

			while(irPin == 0)		//while the pin is low which is our pulse count
			{
			    lTime++;			//increment every 200uS until pin is high
				_delay_us(200);		//200uS delay
			}

			if(lTime >= 6)			//If its high then OR a 1 in else skip
				irCommand |= 0x40;		//if its less than 6 its a 0 so dont OR it
		}

		for(x=0;x<5;x++)			//repeat 5 times for address/device
		{
			irAddress >>= 1;			//if it was skipped or is done ORing then shift over the 1

			while(irPin);			//wait for it to be low
			lTime = 0;				//reset the counter

			while(irPin == 0)		//while the pin is low which is our pulse count
			{
				lTime++;			//increment every 200uS until pin is high
				_delay_us(200);		//200uS delay
			}

			if(lTime >= 6)			//If its high then OR a 1 in else skip
				irAddress |= 0x10;		//if its less than 6 its a 0 so dont OR it			
		}
	}
	return;
}

/************************************************
** SCAN SIRC 15-bit DEVICE ADDRESS AND COMMAND **
************************************************/
void GetSIRC15(void)
{
	char x;
	char lTime;

//StartLook:
	irAddress = irCommand = 0;

	while(irPin);				//wait for it to be low
	lTime = 0;					//reset the counter

	while(irPin == 0)			//while the pin is low which is our pulse count
    {
		lTime++;				//increment every 200uS until pin is high
		_delay_us(200);			//200uS delay
	}

	if((lTime <= 10) || (lTime >= 14))	//Start too short or long and restart
	{
		return;		//goto StartLook;
	}
	else
	{
		lTime = 0;
		for(x=0;x<7;x++)			//repeat 7 times for command
		{
			irCommand >>= 1;			//if it was skipped or is done ORing then shift over the 1

			while(irPin);			//wait for it to be low
			lTime = 0;				//reset the counter

			while(irPin == 0)		//while the pin is low which is our pulse count
			{
			    lTime++;			//increment every 200uS until pin is high
				_delay_us(200);		//200uS delay
			}

			if(lTime >= 6)			//If its high then OR a 1 in else skip
				irCommand |= 0x40;		//if its less than 6 its a 0 so dont OR it
		}

		for(x=0;x<8;x++)			//repeat 8 times for address/device
		{
			irAddress >>= 1;			//if it was skipped or is done ORing then shift over the 1

			while(irPin);			//wait for it to be low
			lTime = 0;				//reset the counter

			while(irPin == 0)		//while the pin is low which is our pulse count
			{
				lTime++;			//increment every 200uS until pin is high
				_delay_us(200);		//200uS delay
			}

			if(lTime >= 6)			//If its high then OR a 1 in else skip
				irAddress |= 0x10;		//if its less than 6 its a 0 so dont OR it			
		}
	}
	return;
}

/************************************************
** SCAN SIRC 20-bit DEVICE ADDRESS AND COMMAND **
************************************************/
void GetSIRC20(void)
{
	char x;
	char lTime;

//StartLook:
	irExtended = irAddress = irCommand = 0;

	while(irPin);				//wait for it to be low
	lTime = 0;					//reset the counter

	while(irPin == 0)			//while the pin is low which is our pulse count
    {
		lTime++;				//increment every 200uS until pin is high
		_delay_us(200);			//200uS delay
	}

	if((lTime <= 10) || (lTime >= 14))	//Start too short or long and restart
	{
		return;		//goto StartLook;
	}
	else
	{
		lTime = 0;
		for(x=0;x<7;x++)			//repeat 7 times for command
		{
			irCommand >>= 1;			//if it was skipped or is done ORing then shift over the 1

			while(irPin);			//wait for it to be low
			lTime = 0;				//reset the counter

			while(irPin == 0)		//while the pin is low which is our pulse count
			{
			    lTime++;			//increment every 200uS until pin is high
				_delay_us(200);		//200uS delay
			}

			if(lTime >= 6)			//If its high then OR a 1 in else skip
				irCommand |= 0x40;		//if its less than 6 its a 0 so dont OR it
		}
		
		for(x=0;x<5;x++)			//repeat 5 times for address/device
		{
			irAddress >>= 1;			//if it was skipped or is done ORing then shift over the 1

			while(irPin);			//wait for it to be low
			lTime = 0;				//reset the counter

			while(irPin == 0)		//while the pin is low which is our pulse count
			{
				lTime++;			//increment every 200uS until pin is high
				_delay_us(200);		//200uS delay
			}

			if(lTime >= 6)			//If its high then OR a 1 in else skip
				irAddress |= 0x10;		//if its less than 6 its a 0 so dont OR it			
		}
		
		for(x=0;x<8;x++)			//repeat 8 times for extended bits
		{
			irExtended >>= 1;			//if it was skipped or is done ORing then shift over the 1

			while(irPin);			//wait for it to be low
			lTime = 0;				//reset the counter

			while(irPin == 0)		//while the pin is low which is our pulse count
			{
				lTime++;			//increment every 200uS until pin is high
				_delay_us(200);		//200uS delay
			}

			if(lTime >= 6)			//If its high then OR a 1 in else skip
				irExtended |= 0x10;		//if its less than 6 its a 0 so dont OR it			
		}
	}
	return;
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
