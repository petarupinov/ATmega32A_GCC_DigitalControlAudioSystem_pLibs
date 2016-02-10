/*************************************************************************
*** LIBRARY: SIRC - SONY INFRARED DECODER (TSOP2240 and more) ************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: ir_sirc.h, v0.02, 29.11.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers                     *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _IR_SIRC_H_
#define _IR_SIRC_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/**************************
** DEFINITION OF IR SIRC **
**************************/

#define irPin (PIND & (1<<PIND2))	//	#define irPin (PINB & (1<<PINB0))

unsigned char irAddress;
unsigned char irCommand;
unsigned char irExtened;

/****************************************************
** DEFINITION SONY REMOTE CONTROL DEVICE CONSTANTS **
****************************************************/

#define IR_REMOTE_TV_DEVICE_RM_677A			0x01	// Sony TV Remote Control
#define IR_REMOTE_CAR_DEVICE_RM_X157		0x04	// Sony Car Audio Remote Control

#define IR_REMOTE_COMMAND_RM_677A_STANDBY	0x15	// Sony TV Remote Control Button STANDBY
#define IR_REMOTE_COMMAND_RM_677A_VOLUP		0x12	// Sony TV Remote Control Button VOL+
#define IR_REMOTE_COMMAND_RM_677A_VOLDN		0x13	// Sony TV Remote Control Button VOL-

#define IR_REMOTE_COMMAND_RM_X157_OFF		0x0D	// Sony Car Audio Remote Control Button OFF
#define IR_REMOTE_COMMAND_RM_X157_ATT		0x14	// Sony Car Audio Remote Control Button ATT
#define IR_REMOTE_COMMAND_RM_X157_SOURCE	0x46	// Sony Car Audio Remote Control Button SOURCE
#define IR_REMOTE_COMMAND_RM_X157_SOUND		0x10	// Sony Car Audio Remote Control Button SOUND
#define IR_REMOTE_COMMAND_RM_X157_MODE		0x47	// Sony Car Audio Remote Control Button MODE
#define IR_REMOTE_COMMAND_RM_X157_MENU		0x0A	// Sony Car Audio Remote Control Button MENU
#define IR_REMOTE_COMMAND_RM_X157_LIST		0x27	// Sony Car Audio Remote Control Button LIST
#define IR_REMOTE_COMMAND_RM_X157_UP		0x33	// Sony Car Audio Remote Control Button UP / +
#define IR_REMOTE_COMMAND_RM_X157_LEFT		0x35	// Sony Car Audio Remote Control Button LEFT / |<<
#define IR_REMOTE_COMMAND_RM_X157_RIGHT		0x34	// Sony Car Audio Remote Control Button RIGHT / >>|
#define IR_REMOTE_COMMAND_RM_X157_DOWN		0x32	// Sony Car Audio Remote Control Button DOWN / -
#define IR_REMOTE_COMMAND_RM_X157_ENTER		0x45	// Sony Car Audio Remote Control Button ENTER
#define IR_REMOTE_COMMAND_RM_X157_DSPL		0x28	// Sony Car Audio Remote Control Button SCRL
#define IR_REMOTE_COMMAND_RM_X157_SCRL		0x23	// Sony Car Audio Remote Control Button SCRL
#define IR_REMOTE_COMMAND_RM_X157_DIGIT1	0x00	// Sony Car Audio Remote Control Button DIGIT1 / REP
#define IR_REMOTE_COMMAND_RM_X157_DIGIT2	0x01	// Sony Car Audio Remote Control Button DIGIT2 / SHUF
#define IR_REMOTE_COMMAND_RM_X157_DIGIT3	0x02	// Sony Car Audio Remote Control Button DIGIT3
#define IR_REMOTE_COMMAND_RM_X157_DIGIT4	0x03	// Sony Car Audio Remote Control Button DIGIT4
#define IR_REMOTE_COMMAND_RM_X157_DIGIT5	0x04	// Sony Car Audio Remote Control Button DIGIT5
#define IR_REMOTE_COMMAND_RM_X157_DIGIT6	0x05	// Sony Car Audio Remote Control Button DIGIT6
#define IR_REMOTE_COMMAND_RM_X157_VOLUP		0x12	// Sony Car Audio Remote Control Button VOL+
#define IR_REMOTE_COMMAND_RM_X157_VOLDN		0x13	// Sony Car Audio Remote Control Button VOL-


/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void IR_DECODER();		// funkciq za razpoznavane na signal po IR
void GetSIRC12();		// funkciq za dekodirane na 12-bit SIRC
void GetSIRC15();		// funkciq za dekodirane na 15-bit SIRC
void GetSIRC20();		// funkciq za dekodirane na 20-bit SIRC

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _IR_SIRC_H_
