/*************************************************************************
*** LIBRARY: Real Time Clock (DS1307) with I2C Interface     *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: rtc.c, v0.02, 26.10.2015                      *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one I2C / TWI  *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include "i2c_twi.h"		// Using for I2C communication to DS1307 !!!

/****************************************
** DEFINITION RTC DS1307 CONSTANT DAYS **
****************************************/
// const char *dayOfWeeks [] = {"    Error  Days", "   Sunday", "   Monday", "  Tuesday", " Wednesday", "  Thursday", "   Friday", "  Saturday"};
// const char *dayOfWeeksUart [] = {"ErD", "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/**********************************
** DEFINITION RTC_1307 FUNCTIONS **
**********************************/

/******************************
** INITIZLIZATION RTC DS1307 **
******************************/
void rtc_ds1307_init()
{
	unsigned char i;
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// MINUTES ADDRESS REGISTER ACCESS
	for(i=0; i<8; i++)
	{
		i2c_write(0x00);	// SECOND, MINUTE, HOUR, DAY, DATE, MONTH, YEAR, CONTROL
	}
	i2c_stop();
}

/*********************
** RESET RTC DS1307 **
*********************/
void rtc_ds1307_reset()
{
	unsigned char i;
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// MINUTES ADDRESS REGISTER ACCESS
	for(i=0; i<8; i++)
	{
		i2c_write(0x00);	// SECOND, MINUTE, HOUR, DAY, DATE, MONTH, YEAR, CONTROL
	}
	i2c_stop();
}

/*********************
** SETUP RTC DS1307 **
*********************/
void rtc_ds1307_set()
{
// Time: HH:MM:SS
// Date: MM/DD/YY
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// SECONDS ADDRESS REGISTER ACCESS
	i2c_write(0b00000011);	// 0x05				// SECONDS DATA VALUE
	i2c_write(0b00100000);	//0x02				// MINUTES DATA VALUE
	i2c_write(0b00100010);	// 0x21				// HOURS DATA VALUE	// 21h ==> MSB 0010 = 2xh; 0001 = x1h LSB
	i2c_stop();
// DAY
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	i2c_write(RTC_DS1307_I2C_DAY);				// DAY ADDRESS REGISTER ACCESS
//	i2c_write(0x02);							// DAY DATA VALUE
//	i2c_stop();
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_DATE);				// DATE ADDRESS REGISTER ACCESS
	i2c_write(0b00100001);						// DATE DATA VALUE
	i2c_write(0b00000001);						// MONTH DATA VALUE
	i2c_write(0b00010011);						// YEAR DATA VALUE	(xx13 == 2013)
// CONTROL
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	i2c_write(RTC_DS1307_I2C_CONTROL);			// CONTROL ADDRESS REGISTER ACCESS
//	i2c_write(0x0D);						// CONTROL DATA VALUE
	i2c_stop();
}

/************************
** GET TIME RTC DS1307 **
************************/
void rtc_ds1307_get()
{
	byte bufferSecond, bufferMinute, bufferHour, bufferDayOfWeek, bufferDate, bufferMonth, bufferYear, bufferControl, bufferRamAddress0, bufferRamAddress1; // bufferDayOfWeek (address 0x03)
	byte byteSecond, byteMinute, byteHour, byteDayOfWeek, byteDate, byteMonth, byteYear; // variables for convert DEC to BCD for LCD and UART for Time and Date

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// START READ ADDRESS REGISTER DATA, it is SECONDS ADDRESS REGISTER ACCESS

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_READ);		// RTC DS1307 ADDRESS ACCESS READ
	bufferSecond	= i2c_read(0);			// SECONDS DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferMinute	= i2c_read(0);			// MINUTES DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferHour		= i2c_read(0);			// HOURS DATA VALUE		// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	bufferDayOfWeek	= i2c_read(0);			// DATE DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferDate		= i2c_read(0);			// DATE DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferMonth		= i2c_read(0);			// MONTHS DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferYear		= i2c_read(0);			// YEARS DATA VALUE		// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	
	bufferControl	= i2c_read(0);			// GET CONTROL DATA
	bufferRamAddress0 = i2c_read(0);		// GET DATA FROM RAM ADDRESS 0x08 OF RTC DS1307
	bufferRamAddress1 = i2c_read(1);		// GET DATA FROM RAM ADDRESS 0x09 OF RTC DS1307
	
	i2c_stop();

	byteSecond	= ((bufferSecond/16*10) + (bufferSecond%16));	// bcdToDec		// Get HEX format -> convert HEX to DEC
	byteMinute	= ((bufferMinute/16*10) + (bufferMinute%16));	// bcdToDec		// Get HEX format -> convert HEX to DEC
	byteHour	= ((bufferHour/16*10) + (bufferHour%16));		// bcdToDec		// Get HEX format -> convert HEX to DEC

	byteDayOfWeek = ((bufferDayOfWeek/16*10) + (bufferDayOfWeek%16));	// bcdToDec		// Get HEX format -> convert HEX to DEC

	byteDate	= ((bufferDate/16*10) + (bufferDate%16));		// bcdToDec		// Get HEX format -> convert HEX to DEC
	byteMonth	= ((bufferMonth/16*10) + (bufferMonth%16));		// bcdToDec		// Get HEX format -> convert HEX to DEC
	byteYear	= ((bufferYear/16*10) + (bufferYear%16));		// bcdToDec		// Get HEX format -> convert HEX to DEC
// -----------------------------------
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
	lcdDataString("                    ");	// 20 intervals
// -----------------------------------
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
	lcdDataString(dayOfWeeks[byteDayOfWeek]);	// ukazatel koito sochi adresa (ne stoinostta na parviq element) na elementa i parviq simvol na stringa
// ---/ OLD STYLE \---
/*	switch(byteDayOfWeek)					// ? dobavqne na izkustvena '0', kogato se izvika chislo bez parvata desetica, ot 0 do 9
	{								// ? dobavqne na izkustvena '0', kogato se izvika chislo bez parvata desetica, ot 0 do 9
		case 1:
		{
			lcdDataString("   Sunday");
			break;
		}
		case 2:
		{
			lcdDataString("   Monday");
			break;
		}
		case 3:
		{
			lcdDataString("  Tuesday");
			break;
		}
		case 4:
		{
			lcdDataString(" Wednesday");
			break;
		}
		case 5:
		{
			lcdDataString("  Thursday");
			break;
		}
		case 6:
		{
			lcdDataString("   Friday");
			break;
		}
		case 7:
		{
			lcdDataString("  Saturday");
			break;
		}
		default:
		{
			lcdDataString("    Error  Days");
		}
	}
*/
// ---\ OLD STYLE /---
	LCD_EXECUTE_DATA_ONE(' ');		// 1 space between DayOfWeek and Date

	if(byteDate<10)					// 10?? meseci 1-12
	{								// dobavqne na izkustvena '0' za desetici
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0' za desetici
	}								// dobavqne na izkustvena '0' za desetici
	lcdDataInt(byteDate);
	LCD_EXECUTE_DATA_ONE('.');
	if(byteMonth<13)				// 13?? meseci 1-31
	{								// dobavqne na izkustvena '0' za desetici
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0' za desetici
	}								// dobavqne na izkustvena '0' za desetici
	lcdDataInt(byteMonth);
	LCD_EXECUTE_DATA_ONE('.');
	if(byteYear<10)
	{								// 10?? godini 1900 -> '00, ako e X0, kade X e ot 0-9 desetici na godini
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0' za desetici
	}								// dobavqne na izkustvena '0' za desetici
	lcdDataInt(byteYear);			// dobavqne na izkustvena '0' za desetici
// -----------------------------------
	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);	// select row 3
// -----------------------------------
	lcdDataString("      ");	// 6 space before Time
	if(byteHour<10)					// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	{								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	}								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	lcdDataInt(byteHour);
	LCD_EXECUTE_DATA_ONE(':');
	if(byteMinute<10)				// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	{								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	}								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	lcdDataInt(byteMinute);
	LCD_EXECUTE_DATA_ONE(':');
	if(byteSecond<10)
	{								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	}								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	lcdDataInt(byteSecond);			// dobavqne na izkustvena '0', za desetici, ot 0 do 9
// -----------------------------------
	LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);	// select row 4
	lcdDataString("                    ");	// 20 intervals
// -----------------------------------
//-------------------------------- OLD CONCEPTION --------------------------------
//	byteSecond = ('0'+ (bufferSecond>>4));		// convert DEC to BCD Seconds	// 0-F, kato ima ogranichenie 0-9 ot samiq DS1307 ili bi trqbvalo da ima proverka za greshka, ako stoinostta e > 9 !!!
//	byteSecond = ('0'+ (bufferSecond & 0x0F));	// convert DEC to BCD Minutes
}

/************************
** SET TIME RTC DS1307 **
************************/
void set_clock(unsigned char temp)
{
	byte bufferSecond, bufferMinute, bufferHour, bufferDayOfWeek, bufferDate, bufferMonth, bufferYear, bufferControl, bufferRamAddress0, bufferRamAddress1; // bufferDayOfWeek (address 0x03)
	byte byteSecond, byteMinute, byteHour, byteDayOfWeek, byteDate, byteMonth, byteYear; // variables for convert DEC to BCD for LCD and UART for Time and Date
	unsigned char flagExitSetupClock = true, flagSetClock = true, flagChoose = false, flagSetHour = true, flagSetMinute = true, flagSetSecond = true, flagSetDay = true, flagSetDate = true, flagSetMonth = true, flagSetYear = true;
	unsigned char flagNext;				//	unsigned char flagNextNibble = true;

	LCD_CLEAR_ALL();
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("Setup Clock: YES/NO?");
	while(flagExitSetupClock)
	{
		if((ENCODER_A_low()) && flagSetClock==true)
		{
			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
			lcdDataString("YES                 ");
			flagChoose = true;
			flagNext = true;
		}
		else if((ENCODER_B_low()) && flagSetClock==true)
		{
			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
			lcdDataString("NO                  ");
			flagChoose = true;
			flagNext = false;
		}
		else if(BUTTON_ESC_low() && flagSetClock==true && flagChoose == true)
		{	
			flagChoose = false;
			if(flagNext)
			{
				LCD_CLEAR_ALL();
				LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
				lcdDataString("Hour:               ");
				while(flagNext)
				{
					if((ENCODER_A_low()) && flagSetHour==true)
					{
						LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
						lcdDataString("RotHour+            ");
						flagChoose = true;
						//flagNext = false;
					}
					else if((ENCODER_B_low()) && flagSetHour==true)
					{
						LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
						lcdDataString("RotHour-            ");
						flagChoose = true;
						//flagNext = false;
					}
					else if(BUTTON_ESC_low() && flagSetHour==true && flagChoose == true)
					{
						flagChoose = false;
						if(flagNext)
						{
							LCD_CLEAR_ALL();
							LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
							lcdDataString("Minute:             ");
							while(flagNext)
							{
								if((ENCODER_A_low()) && flagSetMinute==true)
								{
									LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
									lcdDataString("RotMin+             ");
									flagChoose = true;
									//flagNext = false;
								}
								else if((ENCODER_B_low()) && flagSetMinute==true)
								{
									LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
									lcdDataString("RotMin-             ");
									flagChoose = true;
									//flagNext = false;
								}
								else if(BUTTON_ESC_low() && flagSetSecond==true && flagChoose == true)
								{
									flagChoose = false;
									if(flagNext)
									{
										LCD_CLEAR_ALL();
										LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
										lcdDataString("Second:             ");
										while(flagNext)
										{
											if((ENCODER_A_low()) && flagSetSecond==true)
											{
												LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
												lcdDataString("RotSec+             ");
												flagChoose = true;
												//flagNext = false;
											}
											else if((ENCODER_B_low()) && flagSetSecond==true)
											{
												LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
												lcdDataString("RotSec-             ");
												flagChoose = true;
												//flagNext = false;
											}
											else if(BUTTON_ESC_low() && flagSetDay==true && flagChoose == true)
											{
												flagChoose = false;
												if(flagNext)
												{
													LCD_CLEAR_ALL();
													LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
													lcdDataString("Day:                ");
													while(flagNext)
													{
														if((ENCODER_A_low()) && flagSetDay==true)
														{
															LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
															lcdDataString("RotDay+             ");
															flagChoose = true;
															//flagNext = false;
														}
														else if((ENCODER_B_low()) && flagSetDay==true)
														{
															LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
															lcdDataString("RotDay-             ");
															flagChoose = true;
															//flagNext = false;
														}
														else if(BUTTON_ESC_low() && flagSetDate==true && flagChoose == true)
														{
															flagChoose = false;
															if(flagNext)
															{
																LCD_CLEAR_ALL();
																LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
																lcdDataString("Date:               ");
																while(flagNext)
																{
																	if((ENCODER_A_low()) && flagSetDate==true)
																	{
																		LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																		lcdDataString("RotDate+            ");
																		flagChoose = true;
																		//flagNext = false;
																	}
																	else if((ENCODER_B_low()) && flagSetDate==true)
																	{
																		LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																		lcdDataString("RotDate-            ");
																		flagChoose = true;
																		//flagNext = false;
																	}
																	else if(BUTTON_ESC_low() && flagSetMonth==true && flagChoose == true)
																	{
																		flagChoose = false;
																		if(flagNext)
																		{
																			LCD_CLEAR_ALL();
																			LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
																			lcdDataString("Month:              ");
																			while(flagNext)
																			{
																				if((ENCODER_A_low()) && flagSetMonth==true)
																				{
																					LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																					lcdDataString("RotMonth+           ");
																					flagChoose = true;
																					//flagNext = false;
																				}
																				else if((ENCODER_B_low()) && flagSetMonth==true)
																				{
																					LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																					lcdDataString("RotMonth-           ");
																					flagChoose = true;
																					//flagNext = false;
																				}
																				else if(BUTTON_ESC_low() && flagSetYear==true && flagChoose == true)
																				{
																					flagChoose = false;
																					if(flagNext)
																					{
																						LCD_CLEAR_ALL();
																						LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
																						lcdDataString("Year:               ");
																						while(flagNext)
																						{
																							if((ENCODER_A_low()) && flagSetYear==true)
																							{
																								LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																								lcdDataString("RotYear+            ");
																								flagChoose = true;
																								//flagNext = false;
																							}
																							else if((ENCODER_B_low()) && flagSetYear==true)
																							{
																								LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																								lcdDataString("RotYear-            ");
																								flagChoose = true;
																								//flagNext = false;
																							}
																							else if(BUTTON_ESC_low() && flagSetYear==true && !flagChoose == true)
																							{
																								flagChoose = false;
																								if(flagNext)
																								{
																									LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
																									lcdDataString("EXIT RTC SETUP EXIT!");
																								//	while(flagNext)
																								//	{
																								//		lcdDataString("EXIT RTC SETUP EXIT1");
																								//		LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																								//		lcdDataString("EXIT RTC SETUP EXIT2");
																								//	}
																								}
																							// EXIT
																							//	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																							//	lcdDataString("EXIT CLOCK          ");
																							//	_delay_ms(1000);
																							//	flagExitSetupClock = false;
																								// EXIT
																							//	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																							//	lcdDataString("NO                  ");
																							//	flagChoose = true;
																							//	flagNext = false;
																							//	break;
																							}
																							//else{}
																						}
																					}
																				}
																			}
																		}
																	}
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
			else if(!flagNext)
			{
				LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
				lcdDataString("EXIT CLOCK          ");
				_delay_ms(1000);
				flagExitSetupClock = false;
			}
			else
			{
				
			}
		}
	}
// ---/ OLD STYLE \---
/*	// CHASOVNIKA E SVEREN
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// MINUTES ADDRESS REGISTER ACCESS
//	i2c_write(temp);							// MINUTES ADDRESS REGISTER ACCESS	//	i2c_write(bufferMinute+1);	// 0x00+1	// MINUTES DATA VALUE
	i2c_write(0x30);	// SECOND	hex 30 = dec 48 secundi	(gledai HEX cifrite)
	i2c_write(0x06);	// MINUTE	hex  5 = dec  5 minuti	(gledai HEX cifrite)
	i2c_write(0x21);	// HOUR		hex 21 = dec 33 chas	(gledai HEX cifrite)
	i2c_write(0x02);	// DAY		hex  2 = dec  2 den ot sedmicata (gledai HEX cifrite)
	i2c_write(0x24);	// DATE		hex 24 = dec 36 data	(gledai HEX cifrite)
	i2c_write(0x08);	// MONTH	hex  8 = dec  8 mesec	(gledai HEX cifrite)
	i2c_write(0x15);	// YEAR		hex 15 = dec 21 godina	(gledai HEX cifrite)
	i2c_stop();

	LCD_CLEAR_ALL();	//LCD_INIT();
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("   CLOCK  IS  SET   ");
*/
	LCD_CLEAR_ALL();
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("CHASOVNIKA E SVEREN!");
	_delay_ms(500);
	LCD_CLEAR_ALL();
	_delay_ms(500);
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("CHASOVNIKA E SVEREN!");
	_delay_ms(500);
	LCD_CLEAR_ALL();
	_delay_ms(500);
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("CHASOVNIKA E SVEREN!");
	_delay_ms(500);
	LCD_CLEAR_ALL();



//	RTC_1307_SET();

/*
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	for(int RTC_Registers = 0; RTC_Registers < 8; RTC_Registers++) // SECONDS ADDRESS REGISTER ACCESS is 0x00
//	{
//		i2c_write(0b00000000);	// CLEAR RTC REGISTERS: 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // FROM SECONDS to CONTROL REGISTER
//	}
//	i2c_stop();
*/
// ---\ OLD STYLE /---
}

/******************************************************
** SHOW DIFFERENT VIEW OF DATA/TIME/CLOCK RTC DS1307 **
******************************************************/
void show_clock()
{
	rtc_ds1307_get();
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
