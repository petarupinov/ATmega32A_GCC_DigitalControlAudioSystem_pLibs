/*************************************************************************
*** LIBRARY: RELAYS + SHIFT REGISTER 74HC595                 *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: relay_74hc595.h, v0.01, 25.11.2015            *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _RELAY_74HC595_H_
#define _RELAY_74HC595_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/
/**************************
* DEFINITION POWER SUPPLY *
**************************/
#define REL_POWER_PIN  PC2		// PORTC2 POWER SUPPLY Q2 RELAY for TRANSFORMER-1-220V (1,2,3,4,5 channels x 100w), TRANSFORMER-2-220V (6 channel x 300w)
#define REL_POWER_PORT PORTC	// PORTC2 POWER SUPPLY Q2 RELAY for TRANSFORMER-1-220V (1,2,3,4,5 channels x 100w), TRANSFORMER-2-220V (6 channel x 300w)

#define REL_POWER_low()  (REL_POWER_PORT&=~_BV(REL_POWER_PIN))
#define REL_POWER_high() (REL_POWER_PORT|=_BV(REL_POWER_PIN))

/**************************
** DEFINITION RELAYS IN  **
**************************/
#define RELIN_SDI_PIN  PC7		// PORTC7 SDI - SERIAL DATA INPUT
#define RELIN_SDI_PORT PORTC	// PORTC7 SDI - SERIAL DATA INPUT
#define RELIN_SCK_PIN  PC6		// PORTC6 SCK - SERIAL SHIFT CLOCK
#define RELIN_SCK_PORT PORTC	// PORTC6 SCK - SERIAL SHIFT CLOCK
#define RELIN_RCK_PIN  PC3		// PORTC3 RCK - SERIAL LATCH CLOCK
#define RELIN_RCK_PORT PORTC	// PORTC3 RCK - SERIAL LATCH CLOCK

#define RELIN_SDI_low()  (RELIN_SDI_PORT&=~_BV(RELIN_SDI_PIN))
#define RELIN_SDI_high() (RELIN_SDI_PORT|=_BV(RELIN_SDI_PIN))
#define RELIN_SCK_low()  (RELIN_SCK_PORT&=~_BV(RELIN_SCK_PIN))
#define RELIN_SCK_high() (RELIN_SCK_PORT|=_BV(RELIN_SCK_PIN))
#define RELIN_RCK_low()  (RELIN_RCK_PORT&=~_BV(RELIN_RCK_PIN))
#define RELIN_RCK_high() (RELIN_RCK_PORT|=_BV(RELIN_RCK_PIN))

//	RELAYS_IN_CHOOSE(0b00000001);	// 0b00000001	// RELE 3
//	RELAYS_IN_CHOOSE(0b00000010);	// 0b00000010	// RELE 4
//	RELAYS_IN_CHOOSE(0b00000100);	// 0b00000100	// RELE 8
//	RELAYS_IN_CHOOSE(0b00001000);	// 0b00001000	// RELE 5
//	RELAYS_IN_CHOOSE(0b00010000);	// 0b00010000	// RELE 7
//	RELAYS_IN_CHOOSE(0b00100000);	// 0b00100000	// RELE 1
//	RELAYS_IN_CHOOSE(0b01000000);	// 0b01000000	// RELE 2
//	RELAYS_IN_CHOOSE(0b10000000);	// 0b10000000	// RELE 6

/**************************
** DEFINITION RELAYS OUT **
**************************/
#define RELOUT_SDI_PIN  PB4		// PORTB4 SDI - SERIAL DATA INPUT
#define RELOUT_SDI_PORT PORTB	// PORTB4 SDI - SERIAL DATA INPUT
#define RELOUT_SCK_PIN  PD6		// PORTD6 SCK - SERIAL SHIFT CLOCK
#define RELOUT_SCK_PORT PORTD	// PORTD6 SCK - SERIAL SHIFT CLOCK
#define RELOUT_RCK_PIN  PD7		// PORTD7 RCK - SERIAL LATCH CLOCK
#define RELOUT_RCK_PORT PORTD	// PORTD7 RCK - SERIAL LATCH CLOCK

#define RELOUT_SDI_low()  (RELOUT_SDI_PORT&=~_BV(RELOUT_SDI_PIN))
#define RELOUT_SDI_high() (RELOUT_SDI_PORT|=_BV(RELOUT_SDI_PIN))
#define RELOUT_SCK_low()  (RELOUT_SCK_PORT&=~_BV(RELOUT_SCK_PIN))
#define RELOUT_SCK_high() (RELOUT_SCK_PORT|=_BV(RELOUT_SCK_PIN))
#define RELOUT_RCK_low()  (RELOUT_RCK_PORT&=~_BV(RELOUT_RCK_PIN))
#define RELOUT_RCK_high() (RELOUT_RCK_PORT|=_BV(RELOUT_RCK_PIN))

/********************************************************
** DEFINITION RELAY + SHIFT REGISTER 74HC595 CONSTANTS **
********************************************************/

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void RELAYS_IN_CHOOSE(unsigned char rel_in);
void relays_in1_2ch();
void relays_in1_6ch();
void relays_in2_2ch();
void relays_in2_6ch();
void relays_in3_2ch();
void relays_in3_6ch();
void relays_in_off();
void relays_in_init();
void RELAYS_OUT_CHOOSE(unsigned char rel_out);
void relays_out_1ch();
void relays_out_6ch();
void relays_out_off();
void relays_out_init();

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _relay_74HC595_H_
