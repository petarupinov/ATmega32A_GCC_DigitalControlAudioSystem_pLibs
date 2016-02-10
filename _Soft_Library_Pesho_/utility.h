/*************************************************************************
*** LIBRARY: UTILITY                                         *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: utility.h, v1, 07.09.2015                     *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _UTILITY_H_
#define _UTILITY_H_

/******************
** INCLUDE FILES **
******************/

//#include <stdbool.h>		// type boolean true/false

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/***********************************************************
** DEFINITION OF CUSTOM TYPES BYTE, WORD, DWORD (, QWORD) **
***********************************************************/

//typedef unsigned char  byte;	// 1 BYTE
//typedef unsigned short word;	// 2 BYTES	// word == dword ???
//typedef unsigned int   word;	// 2 BYTES	// word == dword ???
//typedef unsigned long  dword;	// 4 BYTES

typedef unsigned char  byte;	// 1 BYTE
typedef unsigned short word;	// 2 BYTES	// word == dword ???
typedef unsigned int   dword;	// 2 BYTES	// word == dword ???
typedef unsigned long  qword;	// 4 BYTES

/*******************************************
** DEFINITION OF CUSTOM TYPE BOOL/BOOLEAN **
*******************************************/

//typedef int bool;			// http://stackoverflow.com/questions/1921539/using-boolean-values-in-c

typedef enum bool	// const typedef enum bool	// bool or boolean // http://stackoverflow.com/questions/1909825/error-with-the-declaration-of-enum
{
	false,			// false = 0
	true			// true  = 1
} booltype;


/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
unsigned char decToBcd(unsigned char val);
unsigned char bcdToDec(unsigned char val);
void shiftRightOutLsbFirst();
void shiftLeftOutMsbFirst();
void shiftRightInLsbFirst();
void shiftLeftInMsbFirst();
void delay_ms(int miliSec);

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _UTILITY_H_
