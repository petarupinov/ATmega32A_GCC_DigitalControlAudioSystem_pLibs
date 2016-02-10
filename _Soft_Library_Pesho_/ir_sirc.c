/*************************************************************************
*** LIBRARY: SIRC - SONY INFRARED DECODER (TSOP2240 and more) ************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: ir_sirc.c, v0.02, 29.11.2015                  *************
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

/************************************
** DEFINITION IR DECODER FUNCTIONS **
************************************/
void IR_DECODER()
{
	byte byteSS0, byteSS1, byteMM0, byteMM1, byteHH0, byteHH1, byteDD0, byteDD1, byteMont0, byteMont1, byteYY0, byteYY1; // variables for convert DEC to BCD for LCD and UART for Time and Date

	GetSIRC12();
	if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A && irCommand == IR_REMOTE_COMMAND_RM_677A_STANDBY) || (irAddress == IR_REMOTE_CAR_DEVICE_RM_X157 && irCommand == IR_REMOTE_COMMAND_RM_X157_OFF)) && flagPower==0)		// IR POWER -> ON
	{
		ampliferOn();
		flagPower = 1;			// filter za buton ON
		_delay_ms(1000);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A && irCommand == IR_REMOTE_COMMAND_RM_677A_STANDBY) || (irAddress == IR_REMOTE_CAR_DEVICE_RM_X157 && irCommand == IR_REMOTE_COMMAND_RM_X157_OFF)) && flagPower==1)	// IR POWER -> OFF
	{
		ampliferOff();
		flagPower = 0;			// filter za buton OFF
//		break;
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_VOLUP)) && flagPower==1)					// Sony TV & CarAudio IR Remote Device - "VOLUME UP"
	{	// VOLUME UP
		volumeUp();
//		break;
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_VOLDN)) && flagPower==1)					// Sony TV & CarAudio IR Remote Device - "VOLUME DOWN"
	{	// VOLUME DOWN
		volumeDown();
//		break;
	}
	else if((((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ATT)) && mute==0) && flagPower==1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
	{	// MUTE
		muteOn();
		mute = 1;
//		break;
	}
	else if((((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ATT)) && mute==1) && flagPower==1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> OFF
	{	// UNMUTE
		muteOff();
		mute = 0;
//		break;
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_SOURCE)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "SOURCE"
	{
		setClock();
		_delay_ms(200);	
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_MENU)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "MENU"
	{
		showClock();
		_delay_ms(200);	
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_DSPL)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "DSPL"
	{
		LCD_INIT();
		LED_high_DISPLAYLED_low();
		_delay_ms(200);	
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_SCRL)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "SCRL" -> TEMPERATURE
	{
		temperature();
		_delay_ms(200);
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_MODE)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "MODE"
	{
		setupMode();
		_delay_ms(200);
//		break;
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_LIST)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "LIST"
	{
		about();
		_delay_ms(200);	
	}
	else if((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_UP))
	{
		if(flagRTC == 1)
		{
			// HOURS INCREMENT
			hours++;
			if(hours > 23)		// 12, 24?
			{
				hours = 0;
			}
			LCD_INIT();								// LCD INITIZLIZATION
			LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
			lcdDataString("Hours: ");
			// lcdDataInt(hours);
			byteHH0 = ('0'+ (hours>>4));			// convert DEC to BCD Hours
			byteHH1 = ('0'+ (hours & 0x0F));		// convert DEC to BCD Hours
			LCD_EXECUTE_DATA_ONE(byteHH0);
			LCD_EXECUTE_DATA_ONE(byteHH1);
		}
		//else if(flagAny)
		//{
		//}
		//else
		//{}
	}
	else if((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_DOWN))
	{
		if(flagRTC == 1)
		{	// HOURS DECREMENT
			hours--;
			if(hours < 0)
			{
				hours = 23;		// 12, 24?
			}
			LCD_INIT();								// LCD INITIZLIZATION
			LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
			lcdDataString("Hours: ");
			// lcdDataInt(hours);
			byteHH0 = ('0'+ (hours>>4));			// convert DEC to BCD Hours
			byteHH1 = ('0'+ (hours & 0x0F));		// convert DEC to BCD Hours
			LCD_EXECUTE_DATA_ONE(byteHH0);
			LCD_EXECUTE_DATA_ONE(byteHH1);
		}
		//else if(flagAny)
		//{
		//}
		//else
		//{}
	}
	else if((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ENTER))
	{	
		if(flagRTC == 1)
		{
			// store variables to to rtc ds1307
			i2c_start();
			i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
			i2c_write(RTC_DS1307_I2C_HOURS);			// HOURS ADDRESS REGISTER ACCESS
			i2c_write(hours);							// HOURS DATA VALUE
			i2c_stop();
/*
			i2c_start();
			i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
			i2c_write(RTC_DS1307_I2C_MINUTES);			// MINUTES ADDRESS REGISTER ACCESS
			i2c_write(minutes);							// MINUTES DATA VALUE
			i2c_stop();

			i2c_start();
			i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
			i2c_write(RTC_DS1307_I2C_SECONDS);			// SECONDS ADDRESS REGISTER ACCESS
			i2c_write(seconds);							// SECONDS DATA VALUE
			i2c_stop();
*/
			flagRTC = 0;
		}
	}
	else
	{
		// DO NOTING
	}
	_delay_ms(200);
}

/************************************************
** SCAN SIRC 12-bit DEVICE ADDRESS AND COMMAND **
************************************************/
void GetSIRC12()
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
void GetSIRC15()
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
void GetSIRC20()
{
	char x;
	char lTime;

//StartLook:
	irExtened = irAddress = irCommand = 0;

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
			irExtened >>= 1;			//if it was skipped or is done ORing then shift over the 1

			while(irPin);			//wait for it to be low
			lTime = 0;				//reset the counter

			while(irPin == 0)		//while the pin is low which is our pulse count
			{
				lTime++;			//increment every 200uS until pin is high
				_delay_us(200);		//200uS delay
			}

			if(lTime >= 6)			//If its high then OR a 1 in else skip
				irExtened |= 0x10;		//if its less than 6 its a 0 so dont OR it			
		}
	}
	return;
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
