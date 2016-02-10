/*************************************************************************
*** LIBRARY: Real Time Clock (DS1307) with I2C Interface     *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: rtc.h, v1, 01.09.2015                         *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one I2C / TWI  *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _RTC_H_
#define _RTC_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/**************************
** DEFINITION RTC DS1307 **
**************************/
#define RTC_DS1307_I2C_SDA_PIN  PC1		// PINC1		// RTC I2C SDA
#define RTC_DS1307_I2C_SDA_PORT PORTC	// PORTC		// RTC I2C SDA
#define RTC_DS1307_I2C_SDA_DDR  DDRC	// DDRC			// RTC I2C SDA

#define RTC_DS1307_I2C_SCL_PIN  PC0		// PINC0		// RTC I2C SCL
#define RTC_DS1307_I2C_SCL_PORT PORTC	// PORTC		// RTC I2C SCL

#define RTC_DS1307_I2C_SDA_low()		(RTC_DS1307_I2C_SDA_PORT&=~_BV(RTC_DS1307_I2C_SDA_PIN))
#define RTC_DS1307_I2C_SDA_high()		(RTC_DS1307_I2C_SDA_PORT|=_BV(RTC_DS1307_I2C_SDA_PIN))
#define RTC_DS1307_I2C_SCL_low()		(RTC_DS1307_I2C_SCL_PORT&=~_BV(RTC_DS1307_I2C_SCL_PIN))
#define RTC_DS1307_I2C_SCL_high()		(RTC_DS1307_I2C_SCL_PORT|=_BV(RTC_DS1307_I2C_SCL_PIN))

/************************************
** DEFINITION RTC DS1307 CONSTANTS **
************************************/
#define RTC_DS1307_I2C_ADDRESS_READ   0xD1	// RTC DS1307 SLAVE ADDRESS  READ: 0xD1 = 0b11010001 = 0b 1 1 0 1 0 0 0 R/W -> R = 1
#define RTC_DS1307_I2C_ADDRESS_WRITE  0xD0	// RTC DS1307 SLAVE ADDRESS WRITE: 0xD0 = 0b11010000 = 0b 1 1 0 1 0 0 0 R/W -> W = 0
#define RTC_DS1307_I2C_SECONDS		  0x00
#define RTC_DS1307_I2C_MINUTES		  0x01
#define RTC_DS1307_I2C_HOURS		  0x02
#define RTC_DS1307_I2C_DAY			  0x03
#define RTC_DS1307_I2C_DATE			  0x04
#define RTC_DS1307_I2C_MONTH		  0x05
#define RTC_DS1307_I2C_YEAR			  0x06
#define RTC_DS1307_I2C_CONTROL		  0x07

#define RTC_DS1307_I2C_RAM_BEGIN	  0x08
#define RTC_DS1307_I2C_RAM_END		  0x3F

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void RTC_1307_GET();
void RTC_1307_SET();

void showClock();
void setClock(unsigned char temp);
//void setClock();

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _I2C_TWI_H_
