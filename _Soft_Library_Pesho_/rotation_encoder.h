/*************************************************************************
*** LIBRARY: ROTATION ENCODER (ED1112S and more)             *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: rotation_encoder.h, v0.03, 22.11.2015         *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _ROTATION_ENCODER_H_
#define _ROTATION_ENCODER_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/**************************
** DEFINITION OF ENCODER **
**************************/
#define ENCODER_A  PC5		// PINC5
#define ENCODER_B  PC4		// PINC4

#define ENCODER_A_low()  (bit_is_clear(PINC,ENCODER_A))
#define ENCODER_A_high() (bit_is_set(PINC,ENCODER_A))
#define ENCODER_B_low()  (bit_is_clear(PINC,ENCODER_B))
#define ENCODER_B_high() (bit_is_set(PINC,ENCODER_B))

// char tempEncoder = 0;	// can be (+) or (-)

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
unsigned char rotaryEncoderNikBarzakov(unsigned char volume);
unsigned char rotaryEncoderVer1(unsigned char volume);
unsigned char rotaryEncoderVer2(unsigned char volume);

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _ROTATION_ENCODER_H_
