/*************************************************************************
*** LIBRARY: SHIFT REGISTER 74HC595                          *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: 74hc595.h, v0.02, 29.11.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _74HC595_H_
#define _74HC595_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/
/*******************************************
** DEFINITION SHIFT REGISTER 74HC595 PORT **
*******************************************/
// unsigned char SHIFTREG;
// PORTA = SHIFTREG
#define	SHIFTREG			PORTA	// DEFINITION PORT 

/**************************************
** DEFINITION SHIFT REGISTER 74HC595 **
**************************************/
#define SHIFTREG_SDI_PIN		PA0		// PORTA0 LCD SDI - SERIAL DATA INPUT
#define SHIFTREG_SDI_PORT		PORTA	// PORTA0 LCD SDI - SERIAL DATA INPUT
#define SHIFTREG_SCK_PIN		PA1		// PORTA1 LCD SCK - SERIAL SHIFT CLOCK
#define SHIFTREG_SCK_PORT		PORTA	// PORTA1 LCD SCK - SERIAL SHIFT CLOCK
#define SHIFTREG_RCK_PIN		PA2		// PORTA2 LCD RCK - SERIAL LATCH CLOCK
#define SHIFTREG_RCK_PORT		PORTA	// PORTA2 LCD RCK - SERIAL LATCH CLOCK

#define SHIFTREG_SDI_low()		(SHIFTREG_SDI_PORT&=~_BV(SHIFTREG_SDI_PIN))
#define SHIFTREG_SDI_high()		(SHIFTREG_SDI_PORT|=_BV(SHIFTREG_SDI_PIN))
#define SHIFTREG_SCK_low()		(SHIFTREG_SCK_PORT&=~_BV(SHIFTREG_SCK_PIN))
#define SHIFTREG_SCK_high()		(SHIFTREG_SCK_PORT|=_BV(SHIFTREG_SCK_PIN))
#define SHIFTREG_RCK_low()		(SHIFTREG_RCK_PORT&=~_BV(SHIFTREG_RCK_PIN))
#define SHIFTREG_RCK_high()		(SHIFTREG_RCK_PORT|=_BV(SHIFTREG_RCK_PIN))

/************************************************
** DEFINITION SHIFT REGISTER 74HC595 CONSTANTS **
************************************************/

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void SHIFTREG_INIT();
void SHIFTREG_DATA_ONE(unsigned char data);
void SHIFTREG_DATA(char data [], int numsymbols);
void SHIFTREG_pDATA(char *data);

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _74HC595_H_
