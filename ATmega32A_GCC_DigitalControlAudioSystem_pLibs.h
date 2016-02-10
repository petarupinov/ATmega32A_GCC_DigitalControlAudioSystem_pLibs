/************************************************************************************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com                          ***
*** FILE NAME: ATmega32A_GCC_DigitalControlAudioSystem_pLibs.h, v0.01, 06.12.2015 ***
*** SOFT IDE: AVR-GCC compiler                                                    ***
*** HARD uCU: ATmel AVR Microcontrollers                                          ***
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                                        ***
************************************************************************************/

#ifndef _ATMEGA32A_GCC_DIGITALCONTROLAUDIOSYSTEM_PLIBS_H_
#define _ATMEGA32A_GCC_DIGITALCONTROLAUDIOSYSTEM_PLIBS_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/*******************************
** DEFINITION PROJECT VERSION **
*******************************/
#define FIRMWARE_VERSION 27

/*******************************
** DEFINITION DEBUG CONSTANTS **
*******************************/
#define DEBUG_SETTING	1	// ENABLE/DISABLE DEBUG SETTINGS INFORMATION with comment or uncomment this row
#define DEBUG_INFO		1	// ENABLE/DISABLE DEBUG STATUS INFORMATION with comment or uncomment this row
#define DEBUG_ERROR		1	// ENABLE/DISABLE DEBUG ERROR INFORMATION with comment or uncomment this row

/*********************************************
** VARIABLES, CONSTANTS, ARRAYS, STRUCTURES **
*********************************************/

typedef int bool;
#define TRUE 1
#define FALSE 0

// TEST SUCCESS OR FAIL FUNCTIONS
#define SUCCESS 0
#define FAIL -1

#define ERROR -1
#define ERR_AMP_COULD_NOT_START -2
#define ERR_I2C_IS_NOT_WORK -3					// There is a problem, to solve it: Replace and reflash controller.
#define ERR_I2C_RTC_NOT_WORK -4					// There is a problem, to solve it: Replace DS1307 and setup.
#define ERR_I2C_EEPROM_MEMORY_IS_NOT_WORK -5	// There is a problem, to solve it: Replace AT24C64.

// Trqbva da imam funkciq koqto da testva devaisite po platkata, koqto da se puska v opredeleni situacii, zashtoto:
// - test EEPROM read all memmory, write to MCU RAM, fill (write) with zeroes and read all zeroes to test, vrashtane na dannite ot mcu ram v eeprom. Ot EEPROM-a ne trqbva da se zapisva vseki pat stoinost za da se proveri dali raboti eeprom-a - 
// - test RTC and rtc memory, read and write settings to mcu ram, test and return config to rtc
// - test PGA
// - test all relay coils WHILE AMPLIFER IS OFF!!! test without power relays
// - test DISPLAY
// - test Rotary encoder - is work good ?
// - test Buttons press button edi koi si, it is OK or other it is NOT OK
// - test IR receiver
// - test FANS with timer and without timer
// - test Temperature sensors DS18S20

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/**************************
** DEFINITION OF BUTTONS **
**************************/
#define BUTTON_ON_OFF  PB1	// PINB1		// FRONT PANEL DOWN LEFT POSITION	// HEADER 2 pins - CON5 (blue and white)
#define BUTTON_ESC     PB2	// PINB2		// FRONT PANEL UP RIGHT POSITION	// HEADER 2 pins - CON8 (yellow and white) (PACO: 2/6)
#define BUTTON_ENCODER PD3	// PIND3 (NOT WORK) // FRONT PANEL UP LEFT POSITION		// HEADER Encoder 5 pins DE (orange and white) (PACO: INPUT)

#define BUTTON_ON_OFF_low()   (bit_is_clear(PINB,BUTTON_ON_OFF))
#define BUTTON_ON_OFF_high()  (bit_is_set(PINB,BUTTON_ON_OFF))
#define BUTTON_ESC_low()      (bit_is_clear(PINB,BUTTON_ESC))
#define BUTTON_ESC_high()     (bit_is_set(PINB,BUTTON_ESC))
#define BUTTON_ENCODER_low()  (bit_is_clear(PIND,BUTTON_ENCODER))
#define BUTTON_ENCODER_high() (bit_is_set(PIND,BUTTON_ENCODER))

/****************************************************
**** DEFINITION OF LED AND DISPLAY BACKLIGHT LED ****
****************************************************/
#define LED_DISPLAYLED_PIN  PD4
#define LED_DISPLAYLED_PORT PORTD

#define LED_low_DISPLAYLED_high()	(LED_DISPLAYLED_PORT&=~_BV(LED_DISPLAYLED_PIN))				// LED_DISPLAYLED_PORT = (LED_DISPLAYLED_PORT) & (~_BV(LED_DISPLAYLED_PIN))
#define LED_high_DISPLAYLED_low()	(LED_DISPLAYLED_PORT|=_BV(LED_DISPLAYLED_PIN))				// LED_DISPLAYLED_PORT = (LED_DISPLAYLED_PORT) | (_BV(LED_DISPLAYLED_PIN))

/**************************
**** DEFINITION OF FAN ****
**************************/
#define FAN_PIN  PD5
#define FAN_PORT PORTD

#define FAN_low()  (FAN_PORT&=~_BV(FAN_PIN))
#define FAN_high() (FAN_PORT|=_BV(FAN_PIN))

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void port_init(void);			// initialization of all ports and pins

void ext0_intrpt_init(void);	// external interrupt 0 on PD2 - initialization
void ext0_intrpt_on(void);		// external interrupt 0 on PD2 - ENABLE new IR DETECTION
void ext0_intrpt_off(void);		// external interrupt 0 on PD2 - DISABLE new IR DETECTION
void ext1_intrpt_init(void);	// external interrupt 1 on PD3 - initialization
void ext2_intrpt_init(void);	// external interrupt 2 on PB2 - initialization

void irDecode(void);

//void timer0_init();
void timer1_init(void);
void timer1_on(void);
void timer1_on_speed1(void);
void timer1_off(void);
void timer2_init(void);
void FAN_PWM_ON(void);
void FAN_PWM_SPEED1(void);
void FAN_PWM_OFF(void);

void timer2_init(void);
void timer2_on(void);
void timer2_off(void);
void init_all(void);
void ampliferOn(void);
void ampliferOff(void);
void volumeProcess(void);
void volumeUpdate(void);
void commonEncoder(void);

void temperature();				// temperature function
unsigned char oneWireLeft();	// izmervane s temperaturen sensor left
unsigned char oneWireRight();	// izmervane s temperaturen sensor right
char temperMeasur(unsigned char byte0, unsigned char byte1, unsigned char byte6, unsigned char byte7);		// presmqtane na temperatur

void about(void);
/********************************************************************************************
******************************* END DECLARATION OF FUNCTIONS ********************************
********************************************************************************************/

#endif // _ATMEGA32A_GCC_DIGITALCONTROLAUDIOSYSTEM_PLIBS_H_
