/*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;********************************************************************************************************************;;
;;********************************************************************************************************************;;
;;************************************** Eng. Petar Upinov & Plamen Stoyanov *****************************************;;
;;**************************************************** 06.12.2012 ****************************************************;;
;;********************************************************************************************************************;;
;;********************************************************************************************************************;;
;;************************************* ATmega32 Digital Control Audio System ****************************************;;
;;********************************************************************************************************************;;
;;************************************************** Crystal 16MHz ***************************************************;;
;;************************************* 1. Edit Fuse bits: High 0xD9 ; Low 0xFF **************************************;;
;;************************************* 2. Edit Fuse bits: High 0xDA ; Low 0xFF **************************************;;
;;************************************* 3. Edit Fuse bits: High 0xCB ; Low 0xFF **************************************;;
;;************************************* 4. Edit Fuse bits: High 0xCA ; Low 0xFF **************************************;;
;;********************************************************************************************************************;;
;;********************************************************************************************************************;;
;;**  1. Edit on date 19.12.2012 *************************************************************************************;;
;;**  2. Edit on date 20.12.2012 *************************************************************************************;;
;;**  3. Edit on date 21.12.2012 *************************************************************************************;;
;;**  4. Edit on date 22.12.2012 *************************************************************************************;;
;;**  5. Edit on date 24.12.2012 *************************************************************************************;;
;;**  6. Edit on date 25.12.2012 *************************************************************************************;;
;;**  7. Edit on date 01.01.2013 *************************************************************************************;;
;;**  8. Edit on date 08.01.2013 *************************************************************************************;;
;;**  9. Edit on date 09.01.2013 *************************************************************************************;;
;;** 10. Edit on date 09.01.2013 *************************************************************************************;;
;;** 11. Edit on date 09.01.2013 *************************************************************************************;;
;;** 12. Edit on date 10.01.2013 *************************************************************************************;;
;;** 13. Edit on date 14.01.2013 *************************************************************************************;;
;;** 14. Edit on date 15.01.2013 *************************************************************************************;;
;;** 15. Edit on date 16.01.2013 *************************************************************************************;;
;;** 16. Edit on date 16.01.2013 *************************************************************************************;;
;;** 17. Edit on date 16.01.2013 *************************************************************************************;;
;;** 18. Edit on date 18.01.2013 *************************************************************************************;;
;;** 19. Edit on date 19.01.2013 *************************************************************************************;;
;;** 20. Edit on date 21.01.2013 *************************************************************************************;;
;;** 21. Edit on date 21.01.2013 *************************************************************************************;;
;;** 22. Edit on date 22.01.2013 *************************************************************************************;;
;;** 23. Edit on date 29.01.2013 *************************************************************************************;;
;;** 24. Edit on date 12.06.2013 *************************************************************************************;;
;;** 25. Edit on date 13.06.2013 *************************************************************************************;;
;;** 26. Edit on date 13.06.2013 *************************************************************************************;;
;;** 27. Edit on date 13.06.2013 *************************************************************************************;;
;;** 28. Edit on date 17.06.2013 *************************************************************************************;;
;;** 29. Edit on date 18.06.2013 *************************************************************************************;;
;;** 30. Edit on date 20.06.2013 *************************************************************************************;;
;;** 31. Edit on date 26.06.2013 *************************************************************************************;;
;;** 32. Edit on date 27.06.2013 *************************************************************************************;;
;;** 33. Edit on date 27.06.2013 *************************************************************************************;;
;;** 34. Edit on date 30.06.2013 *************************************************************************************;;
;;** 35. Edit on date 10.07.2013 - change INT0-2 *********************************************************************;;
;;** 36. Edit on date 11.08.2013 *************************************************************************************;;
;;** 37. Edit on date 30.09.2013 *************************************************************************************;;
;;** 38. Edit on date 03.10.2013 *************************************************************************************;;
;;** 39. Edit on date 05.10.2013 - add lib uart **********************************************************************;;
;;** 40. Edit on date 07.10.2013 - add lib lcd  **********************************************************************;;
;;** 41. Edit on date 07.10.2013 wrk-btns, enc, ir *******************************************************************;;
;;** 42. Edit on date 13.10.2013 -add lib ds18x20 ********************************************************************;;
;;** 43. Edit on date 13.10.2013 On Off Display & Led ****************************************************************;;
;;** 44. Edit on date 14.08.2015 Set clock: hours, minutes, seconds **************************************************;;
;;********************************************************************************************************************;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;*/

/******************
** INCLUDE FILES **
******************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function

#include "uart.h"
#include "lcd_hd44780_74hc595.h"
#include "ds18x20.h"

// -----= IR REMOTE DECODER =-----

#include <avr/eeprom.h>

char flagPower = 0;		// filter dali amplifer e on ili off 

#define irPin (PIND & (1<<PIND2))	//	#define irPin (PINB & (1<<PINB0))

unsigned char irAddress;
unsigned char irCommand;

//unsigned char	eepromCommand = 0x00;
//unsigned char   eepromAddress = 0x04;

// -----= IR REMOTE DECODER =-----
#define IR_REMOTE_DEVICE_RM_677A	0x01	// Sony TV Remote Control
#define IR_REMOTE_DEVICE_RM_X157	0x04	// Sony Car Audio Remote Control

#define IR_REMOTE_DEVICE_COMMAND_RM_677A_STANDBY	0x15	// Sony TV Remote Control Button STANDBY
#define IR_REMOTE_DEVICE_COMMAND_RM_677A_VOLUP		0x12	// Sony TV Remote Control Button VOL+
#define IR_REMOTE_DEVICE_COMMAND_RM_677A_VOLDN		0x13	// Sony TV Remote Control Button VOL-

#define IR_REMOTE_DEVICE_COMMAND_RM_X157_OFF		0x0D	// Sony Car Audio Remote Control Button OFF
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_ATT		0x14	// Sony Car Audio Remote Control Button ATT
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE		0x46	// Sony Car Audio Remote Control Button SOURCE
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_SOUND		0x10	// Sony Car Audio Remote Control Button SOUND
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_MODE		0x47	// Sony Car Audio Remote Control Button MODE
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_MENU		0x0A	// Sony Car Audio Remote Control Button MENU
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_LIST		0x27	// Sony Car Audio Remote Control Button LIST
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_UP			0x33	// Sony Car Audio Remote Control Button UP / +
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_LEFT		0x35	// Sony Car Audio Remote Control Button LEFT / |<<
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_RIGHT		0x34	// Sony Car Audio Remote Control Button RIGHT / >>|
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_DOWN		0x32	// Sony Car Audio Remote Control Button DOWN / -
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER		0x45	// Sony Car Audio Remote Control Button ENTER
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_DSPL		0x28	// Sony Car Audio Remote Control Button SCRL
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_SCRL		0x23	// Sony Car Audio Remote Control Button SCRL
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_DIGIT1		0x00	// Sony Car Audio Remote Control Button DIGIT1 / REP
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_DIGIT2		0x01	// Sony Car Audio Remote Control Button DIGIT2 / SHUF
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_DIGIT3		0x02	// Sony Car Audio Remote Control Button DIGIT3
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_DIGIT4		0x03	// Sony Car Audio Remote Control Button DIGIT4
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_DIGIT5		0x04	// Sony Car Audio Remote Control Button DIGIT5
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_DIGIT6		0x05	// Sony Car Audio Remote Control Button DIGIT6
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_VOLUP		0x12	// Sony Car Audio Remote Control Button VOL+
#define IR_REMOTE_DEVICE_COMMAND_RM_X157_VOLDN		0x13	// Sony Car Audio Remote Control Button VOL-


/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/*****************
** DEFINE TYPES **
*****************/

//typedef unsigned char  byte;
//typedef unsigned short bit;
//typedef unsigned int   word;
//typedef unsigned long  dword;

typedef unsigned char  byte;
typedef unsigned short word;
typedef unsigned int   dword;
typedef unsigned long  qword;


//asm volatile ("nop");

#define VOLUME_MAX 20
#define VOLUME_MIN 0

	byte volumeNull  = 0b00000000;
	byte volumeRight = 0b00000000;	//0b00111111;
	byte volumeLeft  = 0b00000000;	//0b00111111;
	byte volumeAll   = 0b00000000;

	byte volumeMuteBuff  = 0b00000000;
	char mute = 0;

	byte volumeIndex, volumeIndex_buffer, volume_view;	// volumeIndex - counter for index of volumeValue, volume_view - LCD view counter

	byte volumeValue [VOLUME_MAX] = { 0x00, 0x28, 0x32, 0x3C, 0x46, 0x50, 0x5A, 0x64, 0x6E, 0x78, 0x82, 0x8C, 0x96, 0xA0, 0xAA, 0xB4, 0xBE, 0xC8, 0xD2, 0xD7 };
//       values of volume  ->	0,    40,   50,   60,   70,   80,   90,   100,  110,  120,  130,  140,  150,  160,  170,  180,  190,  200,  210,  215	<-	values of volume
// index of values of volume    0      1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19

// rtc clock
	unsigned char dataClock = 0b00000000;
	unsigned char seconds = 0, minutes = 0, hours = 0, days = 1, dates = 1, months = 1;
	unsigned int years = 2010;

// temper sensor
unsigned char i=0;				// counter cycle
unsigned char store [10] = {};	// data bytes massive
unsigned char byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9; // bytes ot temperaturen sensor


	/**********************************
** DEFINITION OF STATUS REGISTER **
**********************************/
//#define I 7		// SREG I
//#define T 6		// SREG T
//#define H 5		// SREG H
//#define S 4		// SREG S
//#define V 3		// SREG V
//#define N 2		// SREG N
//#define Z 1		// SREG Z
//#define C 0		// SREG C

/**************************
** DEFINITION OF PGA2310 **
**************************/
#define PGA2310_SPI_SDI_PIN    PB5		// PORTA5 PGA2310 SERIAL DATA INPUT PIN		MOSI
#define PGA2310_SPI_SDI_PORT   PORTB	// PORTA5 PGA2310 SERIAL DATA INPUT PIN		MOSI
#define PGA2310_SPI_SDO_PIN    PB6		// PORTB6 PGA2310 SERIAL DATA OUTPUT PIN	MISO
#define PGA2310_SPI_SDO_PORT   PORTB	// PORTB6 PGA2310 SERIAL DATA OUTPUT PIN	MISO
#define PGA2310_SPI_SCK_PIN    PB7		// PORTA7 PGA2310 SERIAL CLOCK INPUT PIN	SCK
#define PGA2310_SPI_SCK_PORT   PORTB	// PORTA7 PGA2310 SERIAL CLOCK INPUT PIN	SCK
#define PGA2310_U6_SPI_CS_PIN  PB3		// PORTB3 PGA2310 U6 CS ENABLE/DISABLE PIN	/CS
#define PGA2310_U6_SPI_CS_PORT PORTB	// PORTB3 PGA2310 U6 CS ENABLE/DISABLE PIN	/CS
#define PGA2310_U7_SPI_CS_PIN  PA6		// PORTA6 PGA2310 U7 CS ENABLE/DISABLE PIN	/CS
#define PGA2310_U7_SPI_CS_PORT PORTA	// PORTA6 PGA2310 U7 CS ENABLE/DISABLE PIN	/CS
#define PGA2310_U8_SPI_CS_PIN  PA7		// PORTA7 PGA2310 U8 CS ENABLE/DISABLE PIN	/CS
#define PGA2310_U8_SPI_CS_PORT PORTA	// PORTA7 PGA2310 U8 CS ENABLE/DISABLE PIN	/CS

#define PGA2310_SPI_SDI_low()    (PGA2310_SPI_SDI_PORT&=~_BV(PGA2310_SPI_SDI_PIN))
#define PGA2310_SPI_SDI_high()   (PGA2310_SPI_SDI_PORT|=_BV(PGA2310_SPI_SDI_PIN))
#define PGA2310_SPI_SDO_low()    (PGA2310_SPI_SDO_PORT&=~_BV(PGA2310_SPI_SDO_PIN))
#define PGA2310_SPI_SDO_high()   (PGA2310_SPI_SDO_PORT|=_BV(PGA2310_SPI_SDO_PIN))
#define PGA2310_SPI_SCK_low()    (PGA2310_SPI_SCK_PORT&=~_BV(PGA2310_SPI_SCK_PIN))
#define PGA2310_SPI_SCK_high()   (PGA2310_SPI_SCK_PORT|=_BV(PGA2310_SPI_SCK_PIN))
#define PGA2310_U6_SPI_CS_low()  (PGA2310_U6_SPI_CS_PORT&=~_BV(PGA2310_U6_SPI_CS_PIN))
#define PGA2310_U6_SPI_CS_high() (PGA2310_U6_SPI_CS_PORT|=_BV(PGA2310_U6_SPI_CS_PIN))
#define PGA2310_U7_SPI_CS_low()  (PGA2310_U7_SPI_CS_PORT&=~_BV(PGA2310_U7_SPI_CS_PIN))
#define PGA2310_U7_SPI_CS_high() (PGA2310_U7_SPI_CS_PORT|=_BV(PGA2310_U7_SPI_CS_PIN))
#define PGA2310_U8_SPI_CS_low()  (PGA2310_U8_SPI_CS_PORT&=~_BV(PGA2310_U8_SPI_CS_PIN))
#define PGA2310_U8_SPI_CS_high() (PGA2310_U8_SPI_CS_PORT|=_BV(PGA2310_U8_SPI_CS_PIN))

/**************************
* DEFINITION LCD DISPLAY  *
**************************/

/*************************************
** DEFINITION LCD DISPLAY CONSTANTS **
*************************************/


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

/**************************
** DEFINITION OF BUTTONS **
**************************/
#define BUTTON_ON_OFF  PB1	// PINB1
#define BUTTON_ESC     PB2	// PINB2
#define BUTTON_ENCODER PD3	// PIND3

#define BUTTON_ON_OFF_low()   (bit_is_clear(PINB,BUTTON_ON_OFF))
#define BUTTON_ON_OFF_high()  (bit_is_set(PINB,BUTTON_ON_OFF))
#define BUTTON_ESC_low()      (bit_is_clear(PINB,BUTTON_ESC))
#define BUTTON_ESC_high()     (bit_is_set(PINB,BUTTON_ESC))
#define BUTTON_ENCODER_low()  (bit_is_clear(PIND,BUTTON_ENCODER))
#define BUTTON_ENCODER_high() (bit_is_set(PIND,BUTTON_ENCODER))

/**************************
** DEFINITION OF ENCODER **
**************************/
#define ENCODER_A  PC5		// PINC5
#define ENCODER_B  PC4		// PINC4

#define ENCODER_A_low()  (bit_is_clear(PINC,ENCODER_A))
#define ENCODER_A_high() (bit_is_set(PINC,ENCODER_A))
#define ENCODER_B_low()  (bit_is_clear(PINC,ENCODER_B))
#define ENCODER_B_high() (bit_is_set(PINC,ENCODER_B))

/**************************
* DEFINITION POWER SUPPLY *
**************************/
#define REL_POWER_PIN  PC2		// PORTC2 POWER SUPPLY Q2 RELAY for TRANSFORMER-1-220V (1,2,3,4,5 channels x 100w), TRANSFORMER-2-220V (6 channel x 300w)
#define REL_POWER_PORT PORTC	// PORTC2 POWER SUPPLY Q2 RELAY for TRANSFORMER-1-220V (1,2,3,4,5 channels x 100w), TRANSFORMER-2-220V (6 channel x 300w)

#define REL_POWER_low()  (REL_POWER_PORT&=~_BV(REL_POWER_PIN))
#define REL_POWER_high() (REL_POWER_PORT|=_BV(REL_POWER_PIN))

/****************************************************
**** DEFINITION OF LED AND DISPLAY BACKLIGHT LED ****
****************************************************/
#define LED_DISPLAYLED_PIN  PD4
#define LED_DISPLAYLED_PORT PORTD

#define LED_low_DISPLAYLED_high()	(LED_DISPLAYLED_PORT&=~_BV(LED_DISPLAYLED_PIN))
#define LED_high_DISPLAYLED_low()	(LED_DISPLAYLED_PORT|=_BV(LED_DISPLAYLED_PIN))

/**************************
**** DEFINITION OF FAN ****
**************************/
#define FAN_PIN  PD5
#define FAN_PORT PORTD

#define FAN_low()  (FAN_PORT&=~_BV(FAN_PIN))
#define FAN_high() (FAN_PORT|=_BV(FAN_PIN))

/**************************
*  DEFINITION IR RECEIVER *
**************************/
#define IR_RECEIVER  PD2	// PIND2		// TSOP2240 for 40kHz IR Receiver for SONY, SIRC Protocol

#define IR_RECEIVER_low()  (bit_is_clear(PIND,IR_RECEIVER))
#define IR_RECEIVER_high() (bit_is_set(PIND,IR_RECEIVER))

/**************************
* DEFINITION EEPROM 24C64 *
**************************/
#define EEPROM_AT24C64_I2C_SDA_PIN  PC1		// PINC1		// EEPROM I2C SDA
#define EEPROM_AT24C64_I2C_SDA_PORT PORTC	// PORTC		// EEPROM I2C SDA
#define EEPROM_AT24C64_I2C_SDA_DDR  DDRC	// DDRC			// EEPROM I2C SDA

#define EEPROM_AT24C64_I2C_SCL_PIN  PC0		// PINC0		// EEPROM I2C SCL
#define EEPROM_AT24C64_I2C_SCL_PORT PORTC	// PORTC		// EEPROM I2C SCL

/**************************************
** DEFINITION EEPROM 24C64 CONSTANTS **
**************************************/
#define EEPROM_AT24C64_I2C_ADDRESS_READ			0xA1	// EEPROM 24C64 SLAVE ADDRESS  READ: 0xA1 = 0b10100001 = 0b 1 0 1 0 NC 0 0 R/W -> R = 1
#define EEPROM_AT24C64_I2C_ADDRESS_WRITE		0xA0	// EEPROM 24C64 SLAVE ADDRESS WRITE: 0xA0 = 0b10100000 = 0b 1 0 1 0 NC 0 0 R/W -> W = 0
#define EEPROM_AT24C64_I2C_HIGH_BYTE_ADDRESS	0x00	// | DEVICE_ADDRESS(R/W) | HIGH_BYTE_ADDRESS | LOW_BYTE_ADDRESS | DATA_BYTE_0 | DATA_BYTE_1 | .... | DATA_BYTE_N |
#define EEPROM_AT24C64_I2C_LOW_BYTE_ADDRESS		0x00	// DEVICE_ADDRESS(R/W), 0bxxx00000, 0b00000000

/**************************
** DEFINITION RTC DS1307 **
**************************/
#define RTC_DS1307_I2C_SDA_PIN  PC1		// PINC1		// RTC I2C SDA
#define RTC_DS1307_I2C_SDA_PORT PORTC	// PORTC		// RTC I2C SDA
#define RTC_DS1307_I2C_SDA_DDR  DDRC	// DDRC			// RTC I2C SDA

#define RTC_DS1307_I2C_SCL_PIN  PC0		// PINC0		// RTC I2C SCL
#define RTC_DS1307_I2C_SCL_PORT PORTC	// PORTC		// RTC I2C SCL

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


/**************************
** DEFINITION OF DS18S20 **
**************************/
#define TEMPSENSOR_PIN  PB0		// PINB0
#define TEMPSENSOR_PORT PORTB	// PORTB
#define TEMPSENSOR_DDR  DDRB	// DDRB

#define TEMPSENSOR_IN()  (TEMPSENSOR_DDR&=~_BV(TEMPSENSOR_PIN))			// DIRECTION IN  OF DATA OF DS18S20
#define TEMPSENSOR_OUT() (TEMPSENSOR_DDR|=_BV(TEMPSENSOR_PIN))			// DIRECTION OUT OF DATA OF DS18S20

#define TEMPSENSOR_SEND_low()  (TEMPSENSOR_PORT&=~_BV(TEMPSENSOR_PIN))	// SEND DATA "0" TO DS18S20
#define TEMPSENSOR_SEND_high() (TEMPSENSOR_PORT|=_BV(TEMPSENSOR_PIN))		// SEND DATA "1" TO DS18S20

#define TEMPSENSOR_RECEIVE_low()  (bit_is_clear(PINB,TEMPSENSOR_PIN))		// RECEIVE DATA "0" FROM DS18S20
#define TEMPSENSOR_RECEIVE_high() (bit_is_set(PINB,TEMPSENSOR_PIN))		// RECEIVE DATA "1" FROM DS18S20

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/

void port_init();
void ext0_intrpt_init();
void ext0_intrpt_on();
void ext0_intrpt_off();
void ext1_intrpt_init();
void ext2_intrpt_init();
void ext2_intrpt_on();
void ext2_intrpt_off();
void timer0_init();
void timer1_init();
void timer1_on();
void timer1_off();
void timer2_init();
void FAN_PWM_ON();
void FAN_PWM_OFF();
//void uart_init();											// uart.h, uart.c
void spi_init();
void spi_start();
void spi_stop();
void i2c_init();
void i2c_start(void);
void i2c_stop();
void i2c_write(byte data);
byte i2c_read(byte isLast);
void RELAYS_IN_CHOOSE(byte rel_in);
void relays_in1_2ch();
void relays_in1_6ch();
void relays_in2_2ch();
void relays_in2_6ch();
void relays_in3_2ch();
void relays_in3_6ch();
void relays_in_off();
void relays_in_init();
void RELAYS_OUT_CHOOSE(byte rel_out);
void relays_out_1ch();
void relays_out_6ch();
void relays_out_off();
void relays_out_init();
//void LCD_INIT();												// lcd_hd44780_74hc595.h, lcd_hd44780_74hc595.c
//void LCD_EXECUTE_COMMAND(byte command);						// lcd_hd44780_74hc595.h, lcd_hd44780_74hc595.c
//void LCD_EXECUTE_DATA(char data [], int numsymbols);			// lcd_hd44780_74hc595.h, lcd_hd44780_74hc595.c
//void LCD_EXECUTE_DATA_ONE(byte data);							// lcd_hd44780_74hc595.h, lcd_hd44780_74hc595.c
//void LCD_EXECUTE_DATA_LAST();									// lcd_hd44780_74hc595.h, lcd_hd44780_74hc595.c
void PGA2310_U6_SPI(byte volume_left, byte volume_right);
void PGA2310_U7_SPI(byte volume_left, byte volume_right);
void PGA2310_U8_SPI(byte volume_left, byte volume_right);
//void uart_transmit(char uart_data [], int numsymbols);		// uart.h, uart.c
//void uart_transmit_one(byte uart_data);						// uart.h, uart.c
//void uart_transmit_DEC_to_BCD(unsigned char rtc_data);		// uart.h, uart.c
void init_all();
void button_pressed_on_off();
void RTC_1307_GET();
void RTC_1307_SET();
void EEPROM_AT24C64_READ();
void EEPROM_AT24C64_WRITE();

void rotaryEncoderNikBarzakov();

void IR_DECODER();		// funkciq za razpoznavane na signal po IR
void GetSIRC();			// funkciq za dekodirane na IR

unsigned char oneWireLeft();	// izmervane s temperaturen sensor left
unsigned char oneWireRight();	// izmervane s temperaturen sensor right
char temperMeasur(unsigned char byte0, unsigned char byte1, unsigned char byte6, unsigned char byte7);		// presmqtane na temperatur
void uartSorting();				// podrejdane na izhoda z temperatura

void showClock();
void setClock();

void ampliferOn();		// amplifer, power on function
void ampliferOff();		// amplifer, power off function
void volumeUp();		// volume up function
void volumeDown();		// volume down function
void muteOn();			// mute on function
void muteOff();			// mute off function
void temperature();		// temperature function
void setupMode();		// setup mode function
void about();			// about function

/********************************************************************************************
******************************* END DECLARATION OF FUNCTIONS ********************************
********************************************************************************************/


/********************************************************************************************
********************************* START OF INITIALIZATIONS **********************************
********************************************************************************************/

/*****************************************
** INITIZLIZATION OF INPUT/OUTPUT PORTS **
*****************************************/
void port_init()
{	

// PORT A connections
	DDRA  = 0b11111111;		//  PA7:U8/PGA2310,CS; PA6:U7/PGA2310,CS; PA5:CON2LCD,RS; PA4:CON2LCD,RW; PA3:CON2LCD,E; PA2:U2/74HC595,RCLK(LCD); PA1:U2/74HC595,SCLK(LCD); PA0:U2/74HC595,DATA(LCD);
	PORTA = 0b00000000;		//	Interfaces: NOT USED. PULLUP DISABLE in PORTA.

// PORT B connections
	DDRB  = 0b10111000;		//  PB7:U6,U7,U8/PGA2310,SCLK; PB6:U6,U7,U8/PGA2310,SDO(CON33,36,39); PB5:U6,U7,U8/PGA2310,SDI; PB4:U12/74HC595,DATA(REL_OUT); PB3:U6/PGA2310,CS; PB2:CON8/BUTTON,"ESCAPE",INT2; PB1:CON5/BUTTON,"ON/OFF"; PB0:CON14TempSensor/DS18S20,DQ,IN/OUT;
	PORTB = 0b00000111;		//	Interfaces: EXTERNAL INTERRUPT 2; SPI. set pullup to IRTSOP2240 / BUTTON(on/off) / DS18s20.

// PORT C connections
	DDRC  = 0b11001100;		//  PC7:U10/74HC595,DATA(REL_IN); PC6:U10/74HC595,SCLK(REL_IN); PC5:EncoderED1112S,A; PC4:EncoderED1112S,B; PC3:U10/74HC595,RCLK(REL_IN); PC2:Q2/BD237(RELAY_POWER_SUPPLY); PC1:DS1307/24C64,I2C/SDA; PC0:DS1307/24C64,I2C/SCL;
	PORTC = 0b00000000;		//	Interfaces: I2C. PULLUP DISABLE in PORTC.

// PORT D connections
	DDRD  = 0b11110000;		//  PD7:U12/74HC595,RCLK(REL_OUT); PD6:U12/74HC595,SCLK(REL_OUT); PD5:Q1/BD237(FAN_CONTROL_PWM); PD4:LED5/"ON/OFF AUDIO SYSTEM"; PD3:CON11/EncoderBUTTON,"MENU/ENTER"/INT1; PD2:CON12/IR/TSOP2240,/INT0; PD1:CON14/MAX232,UART/TXD; PD0:CON14/MAX232,UART/RXD;
	PORTD = 0b00001100;		//	Interfaces: TIMER1; UART/USART; EXTERNAL INTERRUPT 0, 1. PULLUP DISABLE in PORTD.

//	return 1;
//	DDRD&=~_BV(0);// DDD0 = "0"  (DDD0=DDRD nulev bit) set PORTD pin0 to zero as input
//	PORTD|=_BV(0);// PD0  = "1"  Enable pull up
//	PORTD|=_BV(1);// PD1  = "1"  led
//	DDRD|=_BV(1); // DDD1 = "1"  (DDD1=DDRD parvi bit) set PORTD pin1 to one as output

}

/*******************************************
** INITIZLIZATION OF EXTERNAL INTERRUPT 0 **
*******************************************/
void ext0_intrpt_init()
{
	MCUCR = 0b00000010;	// SETUP EXT INT 0, ISC01 = 1, ISC00 = 0: Falling edge on INT0 activates the interrupt; ISC01 = 1, ISC00 = 1: Rising edge on INT0 activates the interrupt;

// IN FUNCTIONS:
//	GICR   = 0b01000000;	// INT0 = 0: Disable External Interrupt on INT0; INT0 = 1: Enable External Interrupt on INT0;
//	GIFR   = 0b01000000;	// Clear INT0 flag.
}

/*******************************************
** INITIZLIZATION OF EXTERNAL INTERRUPT 1 **
*******************************************/
void ext1_intrpt_init()
{
}

/*******************************************
** INITIZLIZATION OF EXTERNAL INTERRUPT 2 **
*******************************************/
void ext2_intrpt_init()
{
	MCUCSR = 0b00000000;	// SETUP EXT INT 2, ISC2 = 0: Falling edge on INT2 activates the interrupt; ISC2 = 1: Rising edge on INT2 activates the interrupt;

// IN FUNCTIONS:
//	GICR   = 0b00100000;	// INT2 = 0: Disable External Interrupt on INT2; INT2 = 1: Enable External Interrupt on INT2;
//	GIFR   = 0b00100000;	// Clear INT2 flag.
}

/*****************************
** INITIZLIZATION OF TIMER0 **
*****************************/
void timer0_init()
{

    //8-bit timer for measuring delay between IR pulses
//	TCCR0 = 0b00000011; //0b00000010 = CLK /8 ; //0b00000011 = CLK / 64; //0b00000101 = CLK / 1024
//	TCNT0 = 0; //reset the timer


	// http://extremeelectronics.co.in/avr-tutorials/avr-timers-an-introduction/
	// http://www.electroons.com/electroons/timer_delay.html
}

/*****************************
** INITIZLIZATION OF TIMER1 **
*****************************/
void timer1_init()
{
// http://www.mikroe.com/forum/viewtopic.php?f=72&t=51076

	TIMSK  = 0b00000000;	// maskov registar za prekasvaniq
	TCNT1H = 0b00000000;
	TCNT1L = 0b00000000;

// TABLE 16.4, Mode 9

//	TCCR1A = 0b01010001;	// OC1A - PWM, OC1B - Disabled, normal port.
//	TCCR1B = 0b00010000;

//	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);	//	TCCR1A = 0b10100001; //0b10100001	// nastroika na 2 kanala rejim na rabota na SHIM
//	TCCR1B = (1 << WGM12)  | (1 << CS12)   | (1 << CS10);  //  TCCR1B = 0b00000101;	// TIMER1 is OFF; 0b00000001 ; nastroika na 2 kanala rejim na rabota na SHIM i preddelitel 8
							// nastrtoika na SHIM (trqbva da se napravi da moje da se promenq)
							// regulirane na skorost na dvigatelq FAN i svetodioda LED
							// CS1: 000 - Timer Stoped;          001 - Timer No Prescaling; 
							//      010 - Timer Prescalling 8;   011 - Timer Prescalling 64;
							//      100 - Timer Prescalling 256; 101 - Timer Prescalling 1024.

//	OCR1AH = 200; //90;			// 0   = 0b00000000 (DEC = BIN)	// FAN
//	OCR1AL = 200; //90;			// 200 = 0b11001000 (DEC = BIN)	// FAN

//	OCR1BH = 100; //20;			// 0   = 0b00000000 (DEC = BIN)	// LED
//	OCR1BL = 100; //20;			// 200 = 0b11001000 (DEC = BIN)	// LED

//	TIMSK = (1 << OCIE1A);
}
void timer1_on_speed1()
{
	TCCR1A = 0b10000001;		// 0b10100001 - OC1A,OC1B - PWM;  0b10000001 - OC1A PWM, OC1B - Disabled, normal port.
	TCCR1B = 0b00010001;

	OCR1AH = 0; // FAN PWM ON
	OCR1AL = 1; // FAN PWM ON

//	OCR1BH = 0; // LED PWM ON
//	OCR1BL = 1; // LED PWM ON
}
void timer1_off()
{
	TCCR1A = 0b00000000;		// DISABLED OCOC1A - PWM, OC1B - Disabled, normal port.
	TCCR1B = 0b00000000;		// 

	OCR1AH = 0; // FAN PWM OFF
	OCR1AL = 0; // FAN PWM OFF

//	OCR1BH = 0; // LED PWM OFF
//	OCR1BL = 0; // LED PWM OFF
}

/*****************************
** INITIZLIZATION OF TIMER2 **
*****************************/
void timer2_init()
{
}

/***************************
** INITIZLIZATION OF UART **
***************************/
/*
void uart_init()
{
	UBRRL = 103;			// 9600, 0, 0 (Error = 0.2%; 16MHz)
	UBRRH = 0;

	UCSRC = 0b10000110;		// URSEL = 1; UCSZ1 = 1; UCSZ0 = 1; 8-bit
	UCSRB = 0b10011000;		// TXEN,RXEN,RXCIE
	UDR = 0b00000000;		// INITIALIZATION NULL OF UART DATA
}
*/
/**************************
** INITIZLIZATION OF SPI **
***************************/
void spi_init()
{
	SPSR = (0<<SPIF)|(0<<WCOL)|(0<<SPI2X);
//	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);			//0b01010010	// SPR1   = 1 - 16 000 000 / 64  = 250 000 = 250kHz  // KOMENTAR ZARADI SIMULACIQTA - PROTEUS BLOKIRANE ZARADI BIT V REGISTAR
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);	//0b01010010	// SPR0,1 = 1 - 16 000 000 / 128 = 125 000 = 125kHz	 // KAKVA KOMBINACIQ OT 4-te BITa VODI DO RAZBLOKIRANETO ???
	SPDR = 0b00000000;

	PGA2310_U6_SPI_CS_high();	// /SS - DISABLE
	PGA2310_U6_SPI(0b00000000, 0b00000000);

	PGA2310_U7_SPI_CS_high();	// /SS - DISABLE
	PGA2310_U7_SPI(0b00000000, 0b00000000);

	PGA2310_U8_SPI_CS_high();	// /SS - DISABLE
	PGA2310_U8_SPI(0b00000000, 0b00000000);

	SPCR = (0<<SPE);
}
void spi_start()
{
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0); //	SPCR = (1<<SPE);
}
void spi_stop()
{
	SPCR = (0<<SPE);
}

/**************************
** INITIZLIZATION OF I2C **
***************************/
void i2c_init()
{

/*======================================|
|===== FORMULE FOR TWI (I2C) USING =====|
|=======================================|
|=======================================|
|==                                   ==|
|==                 Fcpu              ==|
|==   SCL = ---------------------     ==|
|==       (16+(2*TWBR*(4^TWPS))       ==|
|==                                   ==|
|=======================================|
|======= TABLE OF VALUES OF TWPS =======|
|==                                   ==|
|==          (4^TWPS) = value:        ==\_
|==                                       \======================================================================|
|== TWPS = 0 => (4^TWPS)= 1  // TWSR = (0<<TWPS1)|(0<<TWPS0) => TWSR = 0bxxxxxx00 => TWSR = 0 (DEC) => 4^0 =  1 =|
|== TWPS = 1 => (4^TWPS)= 4  // TWSR = (0<<TWPS1)|(1<<TWPS0) => TWSR = 0bxxxxxx01 => TWSR = 1 (DEC) => 4^1 =  4 =|
|== TWPS = 2 => (4^TWPS)=16  // TWSR = (1<<TWPS1)|(0<<TWPS0) => TWSR = 0bxxxxxx10 => TWSR = 2 (DEC) => 4^2 = 16 =|
|== TWPS = 3 => (4^TWPS)=64  // TWSR = (1<<TWPS1)|(1<<TWPS0) => TWSR = 0bxxxxxx11 => TWSR = 3 (DEC) => 4^3 = 64 =|
|===============================================================================================================*/

	TWBR = 0b00010010;	// HEX=0x12	or DEC=18	// FCPU = 16 000 000 |-> SCL = 16000000 / (16+(2*TWBR*(4^TWPS)) = 16000000 / (16+(2*18*(4))) = 16000000 / 160 = 100 000 Hz = 100 kHz
	TWSR = (0<<TWPS1)|(1<<TWPS0);	// SCL = 16 000 000 / 64  = 250 000 = 250kHz
	TWCR = (1<<TWEN);	//	TWCR = 0x04; // TWCR = (x<<TWINT)|(x<<TWEA)|(x<<TWSTA)|(x<<TWSTO)|(x<<TWEN)|(x<<TWIE);	//0b01010010	// SPR0,1 = 1 - 16 000 000 / 128 = 125 000 = 125kHz
	TWDR = 0b00000000;
}

/********************************************************************************************
********************************** END OF INITIALIZATIONS ***********************************
********************************************************************************************/

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/***************************************
** DEFINITION OF EXTERNAL INTERRUPT 0 **
***************************************/
void ext0_intrpt_on()
{
	GICR   = 0b01000000;	// INT0 = 0: Disable External Interrupt on INT0; INT0 = 1: Enable External Interrupt on INT0;
	GIFR   = 0b01000000;	// Clear INT0 flag.
}
void ext0_intrpt_off()
{
	GICR   = 0b00000000;	// INT0 = 0: Disable External Interrupt on INT0; INT0 = 1: Enable External Interrupt on INT0;
	GIFR   = 0b01000000;	// Clear INT0 flag.
}

/***************************************
** DEFINITION OF EXTERNAL INTERRUPT 1 **
***************************************/

/***************************************
** DEFINITION OF EXTERNAL INTERRUPT 2 **
***************************************/
void ext2_intrpt_on()
{
	GICR   = 0b00100000;	// INT2 = 0: Disable External Interrupt on INT2; INT2 = 1: Enable External Interrupt on INT2;
	GIFR   = 0b00100000;	// Clear INT2 flag.
}
void ext2_intrpt_off()
{
	GICR   = 0b00000000;	// INT2 = 0: Disable External Interrupt on INT2; INT2 = 1: Enable External Interrupt on INT2;
	GIFR   = 0b00100000;	// Clear INT2 flag.
}

/***********************************
******** DEFINITIONS OF FAN ********
***********************************/
void FAN_PWM_SPEED1()
{
	timer1_on_speed1();
}
void FAN_PWM_OFF()
{
	timer1_off();
}

/***********************************
** DEFINITION RELAYS IN FUNCTIONS **
***********************************/
void RELAYS_IN_CHOOSE(byte rel_in)	// HELP: RELAYS_IN_CHOOSE(unsigned char byte_of_choosing_combination_of_relay_in) // 74HC595 shift right out, lsb is first
{
	unsigned char storeLSB;
	unsigned char x;
	storeLSB = rel_in;

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		RELIN_SCK_low();

		if(storeLSB & 0x01)	// maska & za log "1" na LSB 0b00000001
		{
			RELIN_SDI_high();	// izvejdane na log "1" v MSB	// PORTC |= (1<<RELIN_SDI_PIN);
		}
		else
		{
			RELIN_SDI_low();	// izvejdane na log "0" v MSB	// PORTC &= ~(1<<RELIN_SDI_PIN);	
		}
		
		storeLSB = storeLSB >> 1;	// shiftvane na >> nadqsno

		RELIN_SCK_high();
	}

	RELIN_RCK_low();
	_delay_us(170);			//rcall 180 us
	RELIN_RCK_high();
	_delay_us(170);			//rcall 180 us

}

void relays_in1_2ch()
{
	RELAYS_IN_CHOOSE(0b00100000);	// RELE 1
}

void relays_in1_6ch()
{
	RELAYS_IN_CHOOSE(0b01100001);	// RELE 1,2,3
}

void relays_in2_2ch()
{
	RELAYS_IN_CHOOSE(0b00001010);	// RELE 4,5
}

void relays_in2_6ch()
{
	RELAYS_IN_CHOOSE(0b10011010);	// RELE 4,5,6,7
}

void relays_in3_2ch()
{
	RELAYS_IN_CHOOSE(0b00001100);	// RELE 8,5
}

void relays_in3_6ch()
{
	RELAYS_IN_CHOOSE(0b10011100);	// RELE 8,5,6,7
}

void relays_in_off()
{
	RELAYS_IN_CHOOSE(0b00000000);	// 0b00000000 // [8][7][6][5][4][3][2][1] // RELETA IZKLIUCHENI 
}

void relays_in_init()
{
	RELAYS_IN_CHOOSE(0b00000000);	// 0b00000000 // [8][7][6][5][4][3][2][1] // RELETA IZKLIUCHENI 
}
/************************************
** DEFINITION RELAYS OUT FUNCTIONS **
************************************/
void RELAYS_OUT_CHOOSE(byte rel_out)	// HELP: RELAYS_OUT_CHOOSE(unsigned char byte_of_choosing_combination_of_relay_in) // 74HC595 shift right out, lsb is first
{
	unsigned char storeLSB;
	unsigned char x;
	storeLSB = rel_out;

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		RELOUT_SCK_low();

		if(storeLSB & 0x01)	// maska & za log "1" na LSB 0b00000001
		{
			RELOUT_SDI_high();	// izvejdane na log "1" v MSB	// PORTC |= (1<<RELIN_SDI_PIN);
		}
		else
		{
			RELOUT_SDI_low();	// izvejdane na log "0" v MSB	// PORTC &= ~(1<<RELIN_SDI_PIN);	
		}
		
		storeLSB = storeLSB >> 1;	// shiftvane na >> nadqsno

		RELOUT_SCK_high();
	}

	RELOUT_RCK_low();
	_delay_us(170);			//rcall 180 us
	RELOUT_RCK_high();
	_delay_us(170);			//rcall 180 us

}

void relays_out_1ch()
{
	RELAYS_OUT_CHOOSE(0b10000000);	// RELE 1
}

void relays_out_6ch()
{
	RELAYS_OUT_CHOOSE(0b11111100);	// RELE 1,2,3,4,5,6
}
void relays_out_off()
{
	RELAYS_OUT_CHOOSE(0b00000000);	// RELE 1,2,3,4,5,6
}
void relays_out_init()
{
	RELAYS_OUT_CHOOSE(0b00000000);	// RELE 1,2,3,4,5,6
}


/*************************************
** DEFINITION LCD DISPLAY FUNCTIONS **
*************************************/
// lcd_hd44780_74hc595.h, lcd_hd44780_74hc595.c


/****************************************
** DEFINITION PGA2310 U6 SPI FUNCTIONS **
****************************************/
void PGA2310_U6_SPI(byte volume_left, byte volume_right)	//PGA2310_U6_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// PB4 - /SS ENABLE
	PGA2310_U6_SPI_CS_low();	// PB3 - ENABLE PGA2310 U6 SPI

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = volume_left;			//volume_left;
	while(!(SPSR & (1<<SPIF)))
	{
	}

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = volume_right;		//volume_right;
	while(!(SPSR & (1<<SPIF)))
	{
	}

	PGA2310_U6_SPI_CS_high();	// PB3 - DISABLE PGA2310 U6 SPI
//	PORTB = (1<<PB4);			// PB4 - /SS DISABLE
}

/****************************************
** DEFINITION PGA2310 U7 SPI FUNCTIONS **
****************************************/
void PGA2310_U7_SPI(byte volume_left, byte volume_right)	//PGA2310_U7_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// PB4 - /SS ENABLE
	PGA2310_U7_SPI_CS_low();	// PB3 - ENABLE PGA2310 U7 SPI

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = volume_left;			//volume_left;
	while(!(SPSR & (1<<SPIF)))
	{
	}

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = volume_right;		//volume_right;
	while(!(SPSR & (1<<SPIF)))
	{
	}

	PGA2310_U7_SPI_CS_high();	// PB3 - DISABLE PGA2310 U7 SPI
//	PORTB = (1<<PB4);			// PB4 - /SS DISABLE
}

/****************************************
** DEFINITION PGA2310 U8 SPI FUNCTIONS **
****************************************/
void PGA2310_U8_SPI(byte volume_left, byte volume_right)	//PGA2310_U8_SPI(0b00001111);
{
//	PORTB = (0<<PB4);			// PB4 - /SS ENABLE
	PGA2310_U8_SPI_CS_low();	// PB3 - ENABLE PGA2310 U8 SPI

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = volume_left;			//volume_left;
	while(!(SPSR & (1<<SPIF)))
	{
	}

//	SPSR = 0b00000000;			// http://www.embeddedrelated.com/groups/lpc2000/show/16257.php
	SPDR = volume_right;		//volume_right;
	while(!(SPSR & (1<<SPIF)))
	{
	}

	PGA2310_U8_SPI_CS_high();	// PB3 - DISABLE PGA2310 U8 SPI
//	PORTB = (1<<PB4);			// PB4 - /SS DISABLE
}

/***********************************
** DEFINITION I2C (TWI) FUNCTIONS **
***********************************/
void i2c_start(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while((TWCR & (1<<TWINT)) == 0)
	{
	}
}

void i2c_stop()
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void i2c_write(byte data)
{
	TWDR = data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while((TWCR & (1<<TWINT)) == 0)
	{
	}
}

byte i2c_read(byte isLast)
{
	if(isLast == 0 )
	{
		TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);
	}
	else
	{
		TWCR = (1<<TWINT)|(1<<TWEN);
	}
	
	while((TWCR & (1<<TWINT)) == 0)
	{
	}
	return TWDR;
}

/**********************************
** DEFINITION RTC_1307 FUNCTIONS **
**********************************/
void RTC_1307_SET()
{
/*
// Time: HH:MM:SS
// Date: MM/DD/YY
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
// SECONDS
	i2c_write(RTC_DS1307_I2C_SECONDS);			// SECONDS ADDRESS REGISTER ACCESS
	i2c_write(0b00000011);	// 0x05				// SECONDS DATA VALUE
// MINUTES
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	i2c_write(RTC_DS1307_I2C_MINUTES);			// MINUTES ADDRESS REGISTER ACCESS
	i2c_write(0b00100000);	//0x02				// MINUTES DATA VALUE
//	i2c_stop();
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
// HOURS
//	i2c_write(RTC_DS1307_I2C_HOURS);			// HOURS ADDRESS REGISTER ACCESS
	i2c_write(0b00100010); // 21h ==> MSB 0010 = 2xh; 0001 = x1h LSB // HOURS DATA VALUE

	i2c_stop();
// DAY
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	i2c_write(RTC_DS1307_I2C_DAY);				// DAY ADDRESS REGISTER ACCESS
//	i2c_write(0x02);							// DAY DATA VALUE
//	i2c_stop();
// DATE
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_DATE);				// DATE ADDRESS REGISTER ACCESS
	i2c_write(0b00100001);						// DATE DATA VALUE
//	i2c_stop();
// MONTH
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	i2c_write(RTC_DS1307_I2C_MONTH);			// MONTH ADDRESS REGISTER ACCESS
	i2c_write(0b00000001);						// MONTH DATA VALUE
//	i2c_stop();
// YEAR
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	i2c_write(RTC_DS1307_I2C_YEAR);				// YEAR ADDRESS REGISTER ACCESS
	i2c_write(0b00010011);						// YEAR DATA VALUE	(xx13 == 2013)
//	i2c_stop();
// CONTROL
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	i2c_write(RTC_DS1307_I2C_CONTROL);			// CONTROL ADDRESS REGISTER ACCESS
//	i2c_write(0x0D);						// CONTROL DATA VALUE
	i2c_stop();

// Time: HH:MM:SS
// Date: MM/DD/YY
*/
}

void RTC_1307_GET()
{
	byte rtcReceiveSeconds, rtcReceiveMinutes, rtcReceiveHours, rtcReceiveDate, rtcReceiveMonth, rtcReceiveYear; // rtcReceiveDay (address 0x03)
	byte byteSS0, byteSS1, byteMM0, byteMM1, byteHH0, byteHH1, byteDD0, byteDD1, byteMont0, byteMont1, byteYY0, byteYY1; // variables for convert DEC to BCD for LCD and UART for Time and Date

// TIME
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// SECONDS ADDRESS REGISTER ACCESS

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_READ);		// RTC DS1307 ADDRESS ACCESS READ
	rtcReceiveSeconds	= i2c_read(0);			// SECONDS DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	rtcReceiveMinutes	= i2c_read(0);			// MINUTES DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	rtcReceiveHours		= i2c_read(1);			// HOURS DATA VALUE		// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	i2c_stop();

//	byteSS0 = ((rtcReceiveSeconds/16*10) + (rtcReceiveSeconds%16));	// bcdToDec
//	byteMM0 = ((rtcReceiveMinutes/16*10) + (rtcReceiveMinutes%16));	// bcdToDec
//	byteHH0 = ((rtcReceiveHours/16*10) + (rtcReceiveHours%16));		// bcdToDec

	byteSS0 = ('0'+ (rtcReceiveSeconds>>4));		// convert DEC to BCD Seconds
	byteSS1 = ('0'+ (rtcReceiveSeconds & 0x0F));	// convert DEC to BCD Seconds
//	byteMM0 = ('0'+ (rtcReceiveMinutes>>4));		// convert DEC to BCD Minutes
//	byteMM1 = ('0'+ (rtcReceiveMinutes & 0x0F));	// convert DEC to BCD Minutes
	byteHH0 = ('0'+ (rtcReceiveHours>>4));			// convert DEC to BCD Hours
	byteHH1 = ('0'+ (rtcReceiveHours & 0x0F));		// convert DEC to BCD Hours

	LCD_INIT();									// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("        CLOCK       ");
	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);		// select row 3

	uart_transmit("Time: ", 6);				// "Time is: " UART
	LCD_EXECUTE_DATA("Time: ", 6);			// "Time is: " LCD (za LCD stringa se broi ot 0 do 5 = 6 simvola)
//	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 1
//	uart_transmit(byteHH0, 0);					// Hx:xx:xx
//	uart_transmit(byteHH1, 0);					// xH:xx:xx
//	uart_transmit_DEC_to_BCD(rtcReceiveHours);	// RTC DS1307 to UART BCD HOURS
	uart_transmit_one(byteHH0);
	LCD_EXECUTE_DATA_ONE(byteHH0);
	uart_transmit_one(byteHH1);
	LCD_EXECUTE_DATA_ONE(byteHH1);
	LCD_EXECUTE_DATA_ONE(':');
	uart_transmit(":", 1);						// razdelitel " : "
//	uart_transmit_DEC_to_BCD(rtcReceiveMinutes);// RTC DS1307 to UART BCD MINUTES
//	uart_transmit(byteMM0, 0);					// xx:Mx:xx
//	uart_transmit(byteMM1, 0);					// xx:xM:xx
	uart_transmit_one(byteMM0);
//	LCD_EXECUTE_DATA_ONE(byteMM0);
lcdDataInt(byteMM0);
	uart_transmit_one(byteMM1);
//	LCD_EXECUTE_DATA_ONE(byteMM1);
lcdDataInt(byteMM1);
	LCD_EXECUTE_DATA_ONE(':');
	uart_transmit(":", 1);						// razdelitel " : "
//	uart_transmit_DEC_to_BCD(rtcReceiveSeconds);// RTC DS1307 to UART BCD SECONDS
//	uart_transmit(byteSS0, 0);					// xx:xx:Sx
//	uart_transmit(byteSS1, 0);					// xx:xx:xS
	uart_transmit_one(byteSS0);
	LCD_EXECUTE_DATA_ONE(byteSS0);
	uart_transmit_one(byteSS1);
	LCD_EXECUTE_DATA_ONE(byteSS1);
	uart_transmit("\r\n", 2);					// nov red "\r\n"

// DATE
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_DATE);				// SECONDS ADDRESS REGISTER ACCESS

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_READ);		// RTC DS1307 ADDRESS ACCESS READ
	rtcReceiveDate	= i2c_read(0);				// DATE DATA VALUE		// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	rtcReceiveMonth	= i2c_read(0);				// MONTH DATA VALUE		// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	rtcReceiveYear	= i2c_read(1);				// YEAR DATA VALUE		// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	i2c_stop();

	byteDD0 = ('0'+ (rtcReceiveHours>>4));
	byteDD1 = ('0'+ (rtcReceiveHours & 0x0F));
	byteMont0 = ('0'+ (rtcReceiveHours>>4));
	byteMont1 = ('0'+ (rtcReceiveHours & 0x0F));
	byteYY0 = ('0'+ (rtcReceiveHours>>4));
	byteYY1 = ('0'+ (rtcReceiveHours & 0x0F));

	LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);		// select row 4
	
	uart_transmit("Date: ", 6);				// "Date is: " UART
	LCD_EXECUTE_DATA("Date: ", 6);			// "Date is: " LCD (za LCD stringa se broi ot 0 do 5 = 6 simvola)
//	uart_transmit_DEC_to_BCD(rtcReceiveDate);	// RTC DS1307 to UART BCD DATE
	uart_transmit_one(byteDD0);
	LCD_EXECUTE_DATA_ONE(byteDD0);
	uart_transmit_one(byteDD1);
	LCD_EXECUTE_DATA_ONE(byteDD1);
	LCD_EXECUTE_DATA_ONE('.');
	uart_transmit(".", 1);
//	uart_transmit_DEC_to_BCD(rtcReceiveMonth);	// RTC DS1307 to UART BCD MONTH
	uart_transmit_one(byteMont0);
	LCD_EXECUTE_DATA_ONE(byteMont0);
	uart_transmit_one(byteMont1);
	LCD_EXECUTE_DATA_ONE(byteMont1);
	LCD_EXECUTE_DATA_ONE('.');
	uart_transmit(".", 1);						// razdelitel " : "
//	uart_transmit_DEC_to_BCD(rtcReceiveYear);	// RTC DS1307 to UART BCD YEAR
	uart_transmit_one(byteYY0);
	LCD_EXECUTE_DATA_ONE(byteYY0);
	uart_transmit_one(byteYY1);
	LCD_EXECUTE_DATA_ONE(byteYY1);
	uart_transmit("\r\n", 2);					// nov red "\r\n"

}

void setClock()
{

//	byte rtcReceiveSeconds, rtcReceiveMinutes, rtcReceiveHours, rtcReceiveDate, rtcReceiveMonth, rtcReceiveYear; // rtcReceiveDay (address 0x03)
	byte byteSS0, byteSS1, byteMM0, byteMM1, byteHH0, byteHH1, byteDD0, byteDD1, byteMont0, byteMont1, byteYY0, byteYY1; // variables for convert DEC to BCD for LCD and UART for Time and Date
	unsigned char success;	// variable for success setup all clock setting datas


// test segment begin
	LCD_INIT();									// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("Press ENTER to set");
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
	lcdDataString("Date and Time");
	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);		// select row 2
	lcdDataString("or press Source");
	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);		// select row 2
	lcdDataString("to exit");

	success = 1;
	while(success)	// SETUP DATE AND TIME
	{
		GetSIRC();
		if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER))	// SET DATE AND TIME
		{	// SET DATE AND TIME
			success = 0;
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE))	// EXIT DATE AND TIME
		{	// EXIT DATE AND TIME
			break;	// success = 0;
		}
		else
		{
		}
	}

// set time variables
	success = 1;
	while(success)	// SETUP HOURS
	{

		GetSIRC();
		if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_UP))	// increment hours
		{	// HOURS INCREMENT
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
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_DOWN))	// decrement hours
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
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER))	// exit hours
		{	// HOURS EXIT
			success = 0;	// break;
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE))	// EXIT DATE AND TIME
		{	// EXIT DATE AND TIME
			break;	// success = 0;
		}
		else{}
	}

	success = 1;
	while(success)	// SETUP MINUTES
	{
		GetSIRC();
		if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_UP))	// increment minutes
		{	// MINUTES INCREMENT
			minutes++;
			if(minutes > 59)
			{
				minutes = 0;
			}
			LCD_INIT();								// LCD INITIZLIZATION
			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
			LCD_EXECUTE_DATA("Min: ", 5);
			// lcdDataInt(minutes);
			byteMM0 = ('0'+ (minutes>>4));			// convert DEC to BCD Minutes
			byteMM1 = ('0'+ (minutes & 0x0F));		// convert DEC to BCD Minutes
			LCD_EXECUTE_DATA_ONE(byteMM0);
			LCD_EXECUTE_DATA_ONE(byteMM1);
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_DOWN))	// decrement minutes
		{	// MINUTES DECREMENT
			minutes--;
			if(minutes < 0)
			{
				minutes = 59;
			}
			LCD_INIT();								// LCD INITIZLIZATION
			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
			LCD_EXECUTE_DATA("Min: ", 5);
			// lcdDataInt(minutes);
			byteMM0 = ('0'+ (minutes>>4));			// convert DEC to BCD Minutes
			byteMM1 = ('0'+ (minutes & 0x0F));		// convert DEC to BCD Minutes
			LCD_EXECUTE_DATA_ONE(byteMM0);
			LCD_EXECUTE_DATA_ONE(byteMM1);
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER))	// exit minutes
		{	// MINUTES EXIT
			success = 0;	// break;
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE))	// EXIT DATE AND TIME
		{	// EXIT DATE AND TIME
			break;	// success = 0;
		}
		else{}
	}

	success = 1;
	while(success)	// SETUP SECONDS
	{
		GetSIRC();
		if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_UP))	// increment seconds
		{	// SECONDS INCREMENT
			seconds++;
			if(seconds > 59)
			{
				seconds = 0;
			}
			LCD_INIT();								// LCD INITIZLIZATION
			LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);		// select row 3
			lcdDataString("Sec: ");
			// lcdDataInt(seconds);
			byteSS0 = ('0'+ (seconds>>4));			// convert DEC to BCD Seconds
			byteSS1 = ('0'+ (seconds & 0x0F));		// convert DEC to BCD Seconds
			LCD_EXECUTE_DATA_ONE(byteSS0);
			LCD_EXECUTE_DATA_ONE(byteSS1);
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_DOWN))	// decrement seconds
		{	// SECONDS DECREMENT
			seconds--;
			if(seconds < 0)
			{
				seconds = 59;
			}
			LCD_INIT();								// LCD INITIZLIZATION
			LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);		// select row 3
			lcdDataString("Sec: ");
			// lcdDataInt(seconds);
			byteSS0 = ('0'+ (seconds>>4));			// convert DEC to BCD Seconds
			byteSS1 = ('0'+ (seconds & 0x0F));		// convert DEC to BCD Seconds
			LCD_EXECUTE_DATA_ONE(byteSS0);
			LCD_EXECUTE_DATA_ONE(byteSS1);
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER))	// exit seconds
		{	// SECONDS EXIT
			success = 0;	// break;
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE))	// EXIT DATE AND TIME
		{	// EXIT DATE AND TIME
			break;	// success = 0;
		}
		else{}
	}

	success = 1;
	while(success)	// SETUP DAYS
	{
		GetSIRC();
		if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_UP))	// increment days
		{	// DAYS INCREMENT
			days++;
			if(days > 7)
			{
				days = 1;
			}
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_DOWN))	// decrement days
		{	// DAYS DECREMENT
			days--;
			if(days < 1)
			{
				days = 7;
			}
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER))	// exit days
		{	// DAYS EXIT
			success = 0;	// break;
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE))	// EXIT DATE AND TIME
		{	// EXIT DATE AND TIME
			break;	// success = 0;
		}
		else{}
	}

	success = 1;
	while(success)	// SETUP DATES
	{
		GetSIRC();
		if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_UP))	// increment dates
		{	// DATES INCREMENT
			dates++;
			if(dates > 31)		// 28, 29, 30, 31?
			{
				dates = 1;
			}
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_DOWN))	// decrement dates
		{	// DATES DECREMENT
			dates--;
			if(dates < 1)
			{
				dates = 31;		// 28, 29, 30, 31?
			}
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER))	// exit dates
		{	// DATES EXIT
			success = 0;	// break;
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE))	// EXIT DATE AND TIME
		{	// EXIT DATE AND TIME
			break;	// success = 0;
		}
		else{}
	}

	success = 1;
	while(success)	// SETUP MONTHS
	{
		GetSIRC();
		if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_UP))	// increment months
		{	// MONTHS INCREMENT
			months++;
			if(months > 12)
			{
				months = 1;
			}
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_DOWN))	// decrement months
		{	// MONTHS DECREMENT
			months--;
			if(months < 1)
			{
				months = 12;
			}
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER))	// exit months
		{	// MONTHS EXIT
			success = 0;	// break;
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE))	// EXIT DATE AND TIME
		{	// EXIT DATE AND TIME
			break;	// success = 0;
		}
		else{}
	}

	success = 1;
	while(success)	// SETUP YEARS
	{
		GetSIRC();
		if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_UP))	// increment years
		{	// YEARS INCREMENT
			years++;
			if(years > 2100)
			{
				years = 2010;
			}
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_DOWN))	// decrement years
		{	// YEARS DECREMENT
			years--;
			if(years < 2010)
			{
				years = 2100;
			}
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_ENTER))	// exit years
		{	// YEARS EXIT
			success = 0;	// break;
		}
		else if((irAddress == IR_REMOTE_DEVICE_RM_X157) && (irCommand == IR_REMOTE_DEVICE_COMMAND_RM_X157_SOURCE))	// EXIT DATE AND TIME
		{	// EXIT DATE AND TIME
			break;	// success = 0;
		}
		else{}
	}

// store variables to to rtc ds1307
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_HOURS);			// HOURS ADDRESS REGISTER ACCESS
	i2c_write(hours);							// HOURS DATA VALUE
	i2c_stop();

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
/*
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_DAY);				// DAY ADDRESS REGISTER ACCESS
	i2c_write(days);							// DAY DATA VALUE
	i2c_stop();

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_DATE);				// DATE ADDRESS REGISTER ACCESS
	i2c_write(dates);							// DATE DATA VALUE
	i2c_stop();

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_MONTH);			// MONTH ADDRESS REGISTER ACCESS
	i2c_write(months);							// MONTH DATA VALUE
	i2c_stop();

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_YEAR);				// YEAR ADDRESS REGISTER ACCESS
	i2c_write(years);							// YEAR DATA VALUE
	i2c_stop();
*/
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
//	lcdDataString("      SET CLOCK     ");
	lcdDataString("  ? CLOCK IS SET ?  ");

//	wait time to look message ".....clock is set...." and look what is the time
//	showClock();
// test segment end


/*
// TIME WORKED

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// SECONDS ADDRESS REGISTER ACCESS

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_READ);		// RTC DS1307 ADDRESS ACCESS READ
	rtcReceiveSeconds	= i2c_read(0);			// SECONDS DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	rtcReceiveMinutes	= i2c_read(0);			// MINUTES DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	rtcReceiveHours		= i2c_read(1);			// HOURS DATA VALUE		// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	i2c_stop();



	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("      SET CLOCK     ");

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_MINUTES);			// MINUTES ADDRESS REGISTER ACCESS
	i2c_write(rtcReceiveMinutes+1);	// 0x00+1			// MINUTES DATA VALUE
	i2c_stop();
*/

//	RTC_1307_SET();

/*
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
// SECONDS
//	i2c_write(RTC_DS1307_I2C_SECONDS);			// SECONDS ADDRESS REGISTER ACCESS
//	for(int RTC_Registers = 0; RTC_Registers < 8; RTC_Registers++)
//	{
//		i2c_write(0b00000000);	// CLEAR RTC REGISTERS: 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // FROM SECONDS to CONTROL REGISTER
//	}
//	i2c_stop();
*/
}

void showClock()
{
	RTC_1307_GET();
}

/****************************************
** DEFINITION EEPROM AT24C64 FUNCTIONS **
****************************************/
void EEPROM_AT24C64_WRITE()
{
// EEPROM WRITE
	i2c_start();
	i2c_write(EEPROM_AT24C64_I2C_ADDRESS_WRITE);	// EEPROM AT24C64 ADDRESS ACCESS WRITE
// HIGH and LOW BYTE ADDRESS
	i2c_write(EEPROM_AT24C64_I2C_HIGH_BYTE_ADDRESS);	// HIGH STORE ADDRESS
	i2c_write(EEPROM_AT24C64_I2C_LOW_BYTE_ADDRESS);		// LOW  STORE ADDRESS	
// Write data
	i2c_write(0x30);	// Data is stored: 0x30 - '0'
	i2c_write(0x31);	// Data is stored: 0x31 - '1'
	i2c_write(0x32);	// Data is stored: 0x32 - '2'
	i2c_write(0x33);	// Data is stored: 0x33 - '3'
	i2c_write(0x34);	// Data is stored: 0x34 - '4'
	i2c_stop();
}

void EEPROM_AT24C64_READ()
{
	byte eepromReceiveByte0, eepromReceiveByte1, eepromReceiveByte2, eepromReceiveByte3, eepromReceiveByte4;
// EEPROM READ
	i2c_start();
	i2c_write(EEPROM_AT24C64_I2C_ADDRESS_WRITE);		// EEPROM AT24C64 ADDRESS ACCESS WRITE
// HIGH and LOW BYTE ADDRESS
	i2c_write(EEPROM_AT24C64_I2C_HIGH_BYTE_ADDRESS);	// HIGH STORE ADDRESS
	i2c_write(EEPROM_AT24C64_I2C_LOW_BYTE_ADDRESS);		// LOW  STORE ADDRESS	

	i2c_start();
	i2c_write(EEPROM_AT24C64_I2C_ADDRESS_READ);			// EEPROM AT24C64 ADDRESS ACCESS READ
	eepromReceiveByte0	= i2c_read(0);					// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte1	= i2c_read(0);					// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte2	= i2c_read(0);					// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte3	= i2c_read(0);					// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte4	= i2c_read(1);					// EEPROM DATA READ BYTE	// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	i2c_stop();

	LCD_INIT();									// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);		// select row 4

	uart_transmit("EEPROM READ BYTE: ", 18);	// "EEPROM READ BYTE: " UART
	LCD_EXECUTE_DATA("EEPROM: ", 8);			// "EEPROM: " LCD (za LCD stringa se broi ot 0 do 5 = 6 simvola)

	uart_transmit_one(eepromReceiveByte0);
	uart_transmit_one(eepromReceiveByte1);
	uart_transmit_one(eepromReceiveByte2);
	uart_transmit_one(eepromReceiveByte3);
	uart_transmit_one(eepromReceiveByte4);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte0);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte1);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte2);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte3);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte4);
}

/******************************
** DEFINITION UART FUNCTIONS **
******************************/
// uart.h, uart.c

/*********************************
** DEFINITION ENCODER FUNCTIONS **
*********************************/
void rotaryEncoderNikBarzakov()
{
	if((ENCODER_A_low()) && (ENCODER_B_low()))			// A0, B0
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_high()) && (ENCODER_B_low()))		// A1, B0
		{
			// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
// VOLUME UP
			volumeUp();
			// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
		}
	}
	else if((ENCODER_A_high()) && (ENCODER_B_low()))	// A1, B0
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_low()) && (ENCODER_B_low()))		// A0, B0
		{
			// <--- Counter Clockwise; Zavartane po posoka obratno na chasovnikovata strelka.
// VOLUME DOWN
			volumeDown();
		}
	}
	else
	{
			// do nothing
	}
}
/************************************
** DEFINITION IR DECODER FUNCTIONS **
************************************/
void IR_DECODER()
{
	GetSIRC();
	if(((irAddress == 0x01 && irCommand == 0x15) || (irAddress == 0x04 && irCommand == 0x0D)) && flagPower==0)		// IR POWER -> ON
	{
		ampliferOn();
		flagPower = 1;			// filter za buton ON
		_delay_ms(1000);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
	}
	else if(((irAddress == 0x01 && irCommand == 0x15) || (irAddress == 0x04 && irCommand == 0x0D)) && flagPower==1)	// IR POWER -> OFF
	{
		ampliferOff();
		flagPower = 0;			// filter za buton OFF
//		break;
	}
	else if(((irAddress == 0x01 || irAddress == 0x04) && (irCommand == 0x12)) && flagPower==1)					// Sony TV & CarAudio IR Remote Device - "VOLUME UP"
	{	// VOLUME UP
		volumeUp();
//		break;
	}
	else if(((irAddress == 0x01 || irAddress == 0x04) && (irCommand == 0x13)) && flagPower==1)					// Sony TV & CarAudio IR Remote Device - "VOLUME DOWN"
	{	// VOLUME DOWN
		volumeDown();
//		break;
	}
	else if((((irAddress == 0x01 || irAddress == 0x04) && (irCommand == 0x14)) && mute==0) && flagPower==1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
	{	// MUTE
		muteOn();
		mute = 1;
//		break;
	}
	else if((((irAddress == 0x01 || irAddress == 0x04) && (irCommand == 0x14)) && mute==1) && flagPower==1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> OFF
	{	// UNMUTE
		muteOff();
		mute = 0;
//		break;
	}
	else if(((irAddress == 0x04) && (irCommand == 0x46)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "SOURCE"
	{
		setClock();
		_delay_ms(200);	
	}
	else if(((irAddress == 0x04) && (irCommand == 0x0A)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "MENU"
	{
		showClock();
		_delay_ms(200);	
	}
	else if(((irAddress == 0x04) && (irCommand == 0x28)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "DSPL"
	{
		LCD_INIT();
		LED_high_DISPLAYLED_low();
		_delay_ms(200);	
	}
	else if(((irAddress == 0x04) && (irCommand == 0x23)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "SCRL" -> TEMPERATURE
	{
		temperature();
		_delay_ms(200);
	}
	else if(((irAddress == 0x04) && (irCommand == 0x47)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "MODE"
	{
		setupMode();
		_delay_ms(200);
//		break;
	}
	else if(((irAddress == 0x04) && (irCommand == 0x27)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "LIST"
	{
		about();
		_delay_ms(200);	
	}
	else
	{
		// DO NOTING
	}
	_delay_ms(200);
}


void GetSIRC()
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

/********************
**** AMPLIFER ON ****
********************/
void ampliferOn()
{
	FAN_PWM_SPEED1();	// KOMENTAR ZARADI SIMULACIQTA - MNOGO BAVI PRI SIMULACIQ S TIMER1
	LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1
//			FAN_high();		// PORTD5 - FAN ON (logic "1")	NON PWM, NON TIMER1

//			volumeLeft = volumeRight = volumeValue [0];	// nulurane na volume control pri vsqko puskane

	spi_start();
	PGA2310_U6_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U7_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U8_SPI(volumeLeft, volumeRight);
	spi_stop();

	transmitUartString("Amplifer On\r\n");
//		uart_transmit("<AMPLIFER ON>\r\n", 15);	// "\r\n" - 2 symbols (not 4 symbols)

	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
//		LCD_EXECUTE_DATA(" << AMPLIFER  ON >> ",20);	// char "DATA", int 13 of chars of "DATA"
lcdDataString("    Amplifer On");
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
	LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
//			LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);	// select row 3
//			LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
//			LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);	// select row 4
//			LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
	LCD_EXECUTE_COMMAND(LCD_ON);			// LCD ON without CURSOR

// RELAYS ON
	REL_POWER_high();// RELAY POWER ON TRAFs
	_delay_ms(4000);
	relays_in1_6ch();	// RELAYS IN1 CHANNELS 6
	_delay_ms(1000);
	relays_out_6ch();	// RELAYS OUT CHANNELS 6

//			PGA2310_U8_SPI(volumeLeft, volumeRight);	// 'A', 'A', 0b01111110, 0b01111110
}

/********************
**** AMPLIFER OFF ****
********************/
void ampliferOff()
{
//			FAN_low();		// PORTD5 - FAN OFF (logic "0")  NON PWM, NON TIMER1

// RELAYS OFF
	relays_out_off();	// RELAYS OUT CHANNELS 6
	_delay_ms(1000);
	relays_in_off();	// RELAYS IN1 CHANNELS 6
	_delay_ms(1000);
	REL_POWER_low();// RELAY POWER OFF

	transmitUartString("Standby\r\n");
//				uart_transmit("<STANDBY>\r\n", 11);		// "\r\n" - 2 symbols (not 4 symbols)

	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
lcdDataString("       Standby");
//				LCD_EXECUTE_DATA(" >>  <STANDBY>   << ",20);		// char "DATA", int 13 of chars of "DATA"
//				_delay_ms(500);	// izchakvane - migasht efekt
//				LCD_INIT();								// LCD INITIZLIZATION
//				_delay_ms(250);	// izchakvane - migasht efekt, natiskane i otpuskane na buton - filtar treptqsht kontakt buton
	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);	// select row 1
lcdDataString("    Amplifer Off");
//				LCD_EXECUTE_DATA(" >> <IR STANDBY> << ",20);		// char "DATA", int 13 of chars of "DATA"
	_delay_ms(500);	//

	_delay_ms(50);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton

	FAN_PWM_OFF();
//			LCD_EXECUTE_COMMAND(LCD_OFF);			// LCD OFF
	LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1
}

/***************************
**** VOLUME UP FUNCTION ****
***************************/
void volumeUp()
{
// volume control up (begin)
	if(volumeIndex > (VOLUME_MAX - 2))
	{
		volumeIndex = (VOLUME_MAX - 1);
	}
	else
	{
		volumeIndex++;
	}
	volumeLeft = volumeRight = volumeValue [volumeIndex];

	spi_start();
	PGA2310_U6_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U7_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U8_SPI(volumeLeft, volumeRight);
	spi_stop();
// volume control up (end)

// lcd display view (begin)
	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
lcdDataString("VOLUME UP");
//	LCD_EXECUTE_DATA("IR VOLUME UP",12);
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
lcdDataInt(volumeIndex);

	transmitUartString("Volume: ");
	transmitUartInt(volumeIndex);
	transmitUartString("\r\n");

//	volume_view = ('0'+ (volumeLeft>>4));
//	LCD_EXECUTE_DATA_ONE(volume_view);
//	volume_view = ('0'+ (volumeLeft & 0x0F));
//	LCD_EXECUTE_DATA_ONE(volume_view);
// lcd display view (end)
}

/*****************************
**** VOLUME DOWN FUNCTION ****
*****************************/
void volumeDown()
{
// volume control down (begin)
	if(volumeIndex < (VOLUME_MIN + 1))
	{
		volumeIndex = VOLUME_MIN;
	}
	else
	{
		volumeIndex--;
	}

	volumeLeft = volumeRight = volumeValue [volumeIndex];

	spi_start();
	PGA2310_U6_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U7_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U8_SPI(volumeLeft, volumeRight);
	spi_stop();
// volume control down (begin)

// lcd display view (begin)
	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
lcdDataString("VOLUME DOWN");
//	LCD_EXECUTE_DATA("IR VOLUME DOWN",14);
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
lcdDataInt(volumeIndex);

	transmitUartString("Volume: ");
	transmitUartInt(volumeIndex);
	transmitUartString("\r\n");

//	volume_view = ('0'+ (volumeLeft>>4));
//	LCD_EXECUTE_DATA_ONE(volume_view);
//	volume_view = ('0'+ (volumeLeft & 0x0F));
//	LCD_EXECUTE_DATA_ONE(volume_view);
// lcd display view (end)*/
}

/********************************
**** VOLUME MUTE ON FUNCTION ****
********************************/
void muteOn()
{
// volume control down (begin)
	volumeMuteBuff = volumeValue [volumeIndex];
	volumeLeft = volumeRight = volumeValue [VOLUME_MIN];


	spi_start();
	PGA2310_U6_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U7_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U8_SPI(volumeLeft, volumeRight);
	spi_stop();
// volume control down (begin)

// lcd display view (begin)
	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
lcdDataString("MUTE ON");
//	LCD_EXECUTE_DATA("MUTE ON",7);
//	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2

	transmitUartString("Volume: Mute \r\n");

//	volume_view = ('0'+ (volumeLeft>>4));
//	LCD_EXECUTE_DATA_ONE(volume_view);
//	volume_view = ('0'+ (volumeLeft & 0x0F));
//	LCD_EXECUTE_DATA_ONE(volume_view);
// lcd display view (end)
}

/*********************************
**** VOLUME MUTE OFF FUNCTION ****
*********************************/
void muteOff()
{
// volume control down (begin)
	volumeValue [volumeIndex] = volumeMuteBuff;
	volumeLeft = volumeRight = volumeValue [volumeIndex];

	spi_start();
	PGA2310_U6_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U7_SPI(volumeLeft, volumeRight);
	spi_stop();

	spi_start();
	PGA2310_U8_SPI(volumeLeft, volumeRight);
	spi_stop();
// volume control down (begin)

// lcd display view (begin)
	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
//	LCD_EXECUTE_DATA("MUTE OFF",8);
lcdDataString("MUTE OFF");
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
lcdDataInt(volumeIndex);

	transmitUartString("Volume: ");
	transmitUartInt(volumeIndex);
	transmitUartString("\r\n");

//	volume_view = ('0'+ (volumeLeft>>4));
//	LCD_EXECUTE_DATA_ONE(volume_view);
//	volume_view = ('0'+ (volumeLeft & 0x0F));
//	LCD_EXECUTE_DATA_ONE(volume_view);
// lcd display view (end)
}

/*****************************
**** TEMPERATURE FUNCTION ****
*****************************/
void temperature()
{
	LED_low_DISPLAYLED_high();
	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
lcdDataString("    TEPERATURE    ");		//
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
lcdDataString("LEFT  SENSOR: ");				//

	oneWireLeft();
	for(i=0; i<9; i++)
	{
		uartSorting();
	}
	temperMeasur(byte0, byte1, byte6, byte7);
//lcdDataString("?? C"); // ot gornata funkciq

	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);	// select row 3
lcdDataString("RIGHT SENSOR: ");			//
	oneWireRight();
	for(i=0; i<9; i++)
	{
		uartSorting();
	}
	temperMeasur(byte0, byte1, byte6, byte7);
//lcdDataString("?? C"); // ot gornata funkciq

	LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);	// select row 4
lcdDataString("             DS18x20");		//
}

/****************************
**** SETUP MODE FUNCTION ****
****************************/
void setupMode()
{
	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
	LCD_EXECUTE_DATA("<<<  SETUP MODE  >>>",20);	// char "DATA", int 13 of chars of "DATA"
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
	LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
}

/***********************
**** ABOUT FUNCTION ****
***********************/
void about()
{
	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	LCD_EXECUTE_DATA(" INTELIGENT AUDIO",17);	// char "DATA", int 16 of chars of "DATA"
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
	LCD_EXECUTE_DATA(" AMPLIFER with CPU",18);	// char "DATA", int 17 of chars of "DATA"
	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);		// select row 3
	LCD_EXECUTE_DATA(" FW. Version beta44",19);	// char "DATA", int 18 of chars of "DATA"
	LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);		// select row 4
	LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);// char "DATA", int 20 of chars of "DATA"
}

/*******************************************
**** 1-WIRE DS18x20 Temperature Sensors ****
*******************************************/
unsigned char oneWireLeft()
{
transmitUartString("TEMPERATURE SENSOR LEFT: 10 DB 09 A5 01 08 00 C1 \r\n");
	if(reset())				// Master issues reset pulse. DS18S20s respond with presence pulse.
	{
		write_byte(0x55);	// Master issues Match ROM command.
		// 64-bit ROM CODE
		write_byte(0x10);	// Byte 0
		write_byte(0xDB);	// Byte 1
		write_byte(0x09);	// Byte 2
		write_byte(0xA5);	// Byte 3
		write_byte(0x01);	// Byte 4
		write_byte(0x08);	// Byte 5
		write_byte(0x00);	// Byte 6
		write_byte(0xC1);	// Byte 7
		// 64-bit ROM CODE

		write_byte(0x44);	// Master issues Convert T command.
		wait_ready();		// Master applies strong pullup to DQ for the duration of the conversion (tCONV).
		if(reset())			// Master issues reset pulse. DS18S20s respond with presence pulse.
		{
			write_byte(0x55);	// Master issues Match ROM command.
			// 64-bit ROM CODE
			write_byte(0x10);	// Byte 0
			write_byte(0xDB);	// Byte 1
			write_byte(0x09);	// Byte 2
			write_byte(0xA5);	// Byte 3
			write_byte(0x01);	// Byte 4
			write_byte(0x08);	// Byte 5
			write_byte(0x00);	// Byte 6
			write_byte(0xC1);	// Byte 7
			// 64-bit ROM CODE

			write_byte(0xBE);	// Master issues Read Scratchpad command.
			for(i=0; i<9; i++)
			{
				store [i] = read_byte();	//	Master reads entire scratchpad including CRC. The master then recalculates the CRC of the first eight data bytes from the scratchpad and compares the calculated CRC with the read CRC (byte 9). If they match, the master continues; if not, the read operation is repeated.
			}
//transmitUartString("RETURN 1\r\n");
			return 1;
		}
//transmitUartString("RETURN 0\r\n");
//transmitUartString("END WORKING\r\n");
	}
	return 0;
}

unsigned char oneWireRight()
{
transmitUartString("TEMPERATURE SENSOR RIGHT: 10 6D F4 8F 02 08 00 B1 \r\n");
	if(reset())				// Master issues reset pulse. DS18S20s respond with presence pulse.
	{
		write_byte(0x55);	// Master issues Match ROM command.
		// 64-bit ROM CODE
		write_byte(0x10);	// Byte 0
		write_byte(0x6D);	// Byte 1
		write_byte(0xF4);	// Byte 2
		write_byte(0x8F);	// Byte 3
		write_byte(0x02);	// Byte 4
		write_byte(0x08);	// Byte 5
		write_byte(0x00);	// Byte 6
		write_byte(0xB1);	// Byte 7
		// 64-bit ROM CODE

		write_byte(0x44);	// Master issues Convert T command.
		wait_ready();		// Master applies strong pullup to DQ for the duration of the conversion (tCONV).
		if(reset())			// Master issues reset pulse. DS18S20s respond with presence pulse.
		{
			write_byte(0x55);	// Master issues Match ROM command.
			// 64-bit ROM CODE
			write_byte(0x10);	// Byte 0
			write_byte(0x6D);	// Byte 1
			write_byte(0xF4);	// Byte 2
			write_byte(0x8F);	// Byte 3
			write_byte(0x02);	// Byte 4
			write_byte(0x08);	// Byte 5
			write_byte(0x00);	// Byte 6
			write_byte(0xB1);	// Byte 7
			// 64-bit ROM CODE

			write_byte(0xBE);	// Master issues Read Scratchpad command.
			for(i=0; i<9; i++)
			{
				store [i] = read_byte();	//	Master reads entire scratchpad including CRC. The master then recalculates the CRC of the first eight data bytes from the scratchpad and compares the calculated CRC with the read CRC (byte 9). If they match, the master continues; if not, the read operation is repeated.
			}
//transmitUartString("RETURN 1\r\n");
			return 1;
		}
//transmitUartString("RETURN 0\r\n");
//transmitUartString("END WORKING\r\n");
	}
	return 0;
}

char temperMeasur(unsigned char byte0, unsigned char byte1, unsigned char byte6, unsigned char byte7)
{

	char tC = 0;
	char temper = 0;
	double k = 0;
	double j = 0;

	byte0 = store [0];
	byte1 = store [1];
	byte6 = store [6];
	byte7 = store [7];

	k = ((byte7 - byte6) / byte7) + 0.25;

	if((byte1 == 0x00) && (byte0 == 0x00))
	{
		tC = (byte0/2);
		j = tC - k;
		transmitUartInt(tC);
		transmitUartString(".0 C");		// ne e smetnato sled desetichnata zapetaq
lcdDataInt(tC);
lcdDataString(".0 C");

//		transmitUartString(".");
//		transmitUartInt((unsigned int)j);
//		transmitUartString(" C");
		transmitUartString("\r\n");
	}
	else if((byte1 == 0x00) && (byte0 != 0x00))
	{
		transmitUartString("+");
		tC = (byte0/2);
		j = tC - k;
		transmitUartInt(tC);
		transmitUartString(".0 C");		// ne e smetnato sled desetichnata zapetaq
lcdDataInt(tC);
lcdDataString(".0 C");

//		transmitUartString(".");
//		transmitUartInt((unsigned int)j);
//		transmitUartString(" C");
		transmitUartString("\r\n");
	}
	else if((byte1 == 0xFF) && (byte0 != 0x00))
	{
		transmitUartString("-");
//		tC = ((byte0 - 255.5) / 2);		// ne e dobre obraboteno za otricatelni chisla
		tC = ((byte0 - 255) / 2);
		j = tC - k;
		transmitUartInt(tC);
		transmitUartString(".0 C");		// ne e smetnato sled desetichnata zapetaq
lcdDataInt(tC);
lcdDataString(".0 C");

//		transmitUartString(".");
//		transmitUartInt((unsigned int)j);
//		transmitUartString(" C");
		transmitUartString("\r\n");
	}
	else
	{
lcdDataString("ERROR!");

		transmitUartString("ERROR TEMPERATURE\r\n");
		return 1;
	}

	return temper;
}

void uartSorting()
{
	transmitUartString("byte ");
	transmitUartInt(i);
	transmitUartString(" : ");
	transmitUartInt(store[i]);
	transmitUartString("\r\n");
}

/********************************************
**** SHIFT RIGHT >> OUT LSB BIT IS FIRST ****
********************************************/

#define serPin 3

void shift_right_out_lsb_first()
{
	unsigned char conbyte = 0x44;
	unsigned char regALSB;
	unsigned char x;
	regALSB = conbyte;
	DDRC |= (1<<serPin);	// serial pin output

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		if(regALSB & 0x01)	// maska & za log "1" na LSB 0b00000001
		{
			PORTC |= (1<<serPin);	// izvejdane na log "1" v LSB
		}
		else
		{
			PORTC &= ~(1<<serPin);	// izvejdane na log "0" v LSB
		}
		
		regALSB = regALSB >> 1;	// shiftvane na >> nadqsno
	}

}
/********************************************
**** SHIFT RIGHT >> OUT LSB BIT IS FIRST ****
********************************************/

/*******************************************
**** SHIFT LEFT << OUT MSB BIT IS FIRST ****
*******************************************/
//#define serPin 3

void shift_left_out_msb_first()
{
	unsigned char conbyte = 0x44;
	unsigned char regAMSB;
	unsigned char x;
	regAMSB = conbyte;
	DDRC |= (1<<serPin);	// serial pin output

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		if(regAMSB & 0x80)	// maska & za log "1" na MSB 0b10000000
		{
			PORTC |= (1<<serPin);	// izvejdane na log "1" v MSB
		}
		else
		{
			PORTC &= ~(1<<serPin);	// izvejdane na log "0" v MSB
		}
		
		regAMSB = regAMSB << 1;	// shiftvane na << nalqvo
	}

}
/*******************************************
**** SHIFT LEFT << OUT MSB BIT IS FIRST ****
*******************************************/

/*******************************************
**** SHIFT RIGHT >> IN LSB BIT IS FIRST ****
*******************************************/

//#define serPin 3

void shift_right_in_lsb_first()
{
	unsigned char x;
	unsigned char REGA = 0;
	
	DDRC &= ~(1<<serPin);	// serial pin input

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		REGA = REGA >> 1;	// shift REGA to right one bit
		REGA |= (PINC & (1<<serPin)) << (7-serPin);	// copy bit serPin of PORTC to MSB of REGA
	}

}
/*******************************************
**** SHIFT RIGHT >> IN LSB BIT IS FIRST ****
*******************************************/

/******************************************
**** SHIFT LEFT << IN MSB BIT IS FIRST ****
******************************************/

//#define serPin 3

void shift_left_in_msb_first()
{
	unsigned char x;
	unsigned char REGA = 0;
	
	DDRC &= ~(1<<serPin);	// serial pin input

	for(x=0; x<8; x++)		// cikal za predavane seriino na 1 byte
	{
		REGA = REGA << 1;	// shift REGA to left one bit
		REGA |= (PINC & (1<<serPin)) >> serPin;	// copy bit serPin of PORTC to LSB of REGA
	}

}
/******************************************
**** SHIFT LEFT << IN MSB BIT IS FIRST ****
******************************************/

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

/********************************************************************************************
*********************************** START OF INTERRUPTS *************************************
********************************************************************************************/

/*************************************
**** EXTERNAL INTERRUPT VECTOR 02 ****
*************************************/
ISR(INT0_vect)
{
	ext0_intrpt_off();	// DISABLE new IR DETECTION

// LOGIC CHECK BEGIN
// VERIFY PRESSED IR BUTTON and switch to low line of IR pin PD2
	unsigned char low_level = 0;
    if(irPin == 0)
	{		//while the pin is low which is our pulse count
		for(unsigned char i=0; i<3; i++)
		{
			low_level++;	//increment every 200uS until pin is high
			_delay_us(2);	//2uS delay
		}

		if(low_level == 3)
		{
			IR_DECODER();
		}
		else
		{
		}
    }
// LOGIC CHECK END

	ext0_intrpt_on();	// ENABLE new IR DETECTION
}

//************************************************************
//***** FUNCTION FOR SOFTWARE DELAY OF 1 mSEC (appx.) ******* 
//************************************************************
void delay_ms(int miliSec)  //for 1 Mhz crystal
{

//  miliSec = miliSec * 16;	// for 16MHz

  int i,j;
  
  for(i=0;i<miliSec;i++)
    for(j=0;j<100;j++)
	{
	  asm("nop");
	  asm("nop");
	}
}

/*************************************
**** EXTERNAL INTERRUPT VECTOR 03 ****
*************************************/
ISR(INT1_vect)
{
}

/*************************************
**** EXTERNAL INTERRUPT VECTOR 04 ****
*************************************/
ISR(INT2_vect)
{
//	ext2_intrpt_off();
//	ext2_intrpt_on();
}

/*************************************
****** UART INTERRUPT VECTOR 14 ******
*************************************/
ISR(USART_RXC_vect)		// file "avr/interrupt.h"
{
unsigned char receive;
unsigned char STATUS;

	STATUS = SREG;
	cli();			// file "avr/interrupt.h"

	receive = UDR;
/*	receive = receive & 0b00011111;
	if (receive & 0b00011111)
	{
		RELAYS_IN_CHOOSE(receive);
	}
*/
/*
	if(receive == 0x30)	// keyboard ascii of number 0
	{
		RELAYS_IN_CHOOSE(0b00011111);
	}
	else if(receive == 0x31)	// keyboard ascii of number 1
	{
		RELAYS_IN_CHOOSE(0b00000001);
	}
	else if(receive == 0x32)	// keyboard ascii of number 2
	{
		RELAYS_IN_CHOOSE(0b00000010);
	}
	else if(receive == 0x33)	// keyboard ascii of number 3
	{
		RELAYS_IN_CHOOSE(0b00000100);
	}
	else if(receive == 0x34)	// keyboard ascii of number 4
	{
		RELAYS_IN_CHOOSE(0b00001000);
	}
	else if(receive == 0x35)	// keyboard ascii of number 5
	{
		RELAYS_IN_CHOOSE(0b00010000);
	}
	else if(receive == ((0x36)&&(0x37)&&(0x38)&&(0x39)))	// keyboard ascii of number 6,7,8,9
	{	// TAZI PROVERKA E IZLISHNA ZARADI DOLNIQ ELSE, no e dobre izpolzvana zaradi primer za maska &&
		RELAYS_IN_CHOOSE(0b00000000);
	}
	else
	{
		receive = 0b00000000;
		RELAYS_IN_CHOOSE(receive);
	}


*/
	SREG = STATUS;
	sei();			// file "avr/interrupt.h"
}

/*************************************
****** UART INTERRUPT VECTOR 15 ******
*************************************/
ISR(USART_UDRE_vect)	// file "avr/interrupt.h"
{
}

/*************************************
****** TWI INTERRUPT VECTOR 20 *******
*************************************/
ISR(TWI_vect)
{
}

/********************************************************************************************
************************************ END OF INTERRUPTS **************************************
********************************************************************************************/

/********************************************************************************************
*********************************** START OF APPLICATION ************************************
********************************************************************************************/

void init_all()
{
	port_init();
	ext0_intrpt_init();		// SONY IR REMOTE
//	ext1_intrpt_init();
//	ext2_intrpt_init();
//	timer0_init();
	timer1_init();	// KOMENTAR ZARADI SIMULACIQTA - MNOGO BAVI PRI SIMULACIQ S TIMER1
//	timer2_init();
	uart_init();
	spi_init();
	i2c_init();
	relays_in_init();
	relays_out_init();
	LCD_INIT();

}

void buttons_press()
{
	volumeIndex = volume_view = 0;	// vinagi VOLUME = 0 pri startirane
	
	while(1)
	{
		if(BUTTON_ON_OFF_low() && flagPower==0)			// PINB1 - BUTTON ON/OFF -> ON
		{
			ampliferOn();
			flagPower = 1;			// filter za buton ON
			_delay_ms(1000);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
		}
		else if(BUTTON_ON_OFF_low() && flagPower==1)	// PINB1 - BUTTON ON/OFF -> OFF
		{
			ampliferOff();
			flagPower = 0;			// filter za buton OFF
		}
		else if(BUTTON_ESC_low() && flagPower==1)
		{
			volumeUp();
			_delay_ms(200);

//			EEPROM_AT24C64_WRITE();
//			RTC_1307_SET();
//			LCD_EXECUTE_DATA("STORE EEPROM",12);
//			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
//			LCD_EXECUTE_DATA(" SET CLOCK ",11);
//			uart_transmit("STORE EEPROM\r\n", 14);
//			uart_transmit("SET CLOCK\r\n", 11);
		}
		else if(BUTTON_ENCODER_low() && flagPower==1)
		{
			volumeDown();
			_delay_ms(200);

//			RTC_1307_GET();
//			EEPROM_AT24C64_READ();
//			LCD_EXECUTE_DATA("STORE EEPROM",12);
//			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
//			LCD_EXECUTE_DATA(" SET CLOCK ",11);
//			uart_transmit("STORE EEPROM\r\n", 14);
//			uart_transmit("SET CLOCK\r\n", 11);

//					while(1)
//					{
//						RTC_1307_GET();
//						_delay_ms(500);
//			
//						if( (BUTTON_ESC_low()) || (BUTTON_ON_OFF_low()) || (BUTTON_ENCODER_low()))
//						{
//							break;
//						}
//					}
		}
		else if(BUTTON_ESC_low() && flagPower==0)
		{
			setupMode();
			_delay_ms(1000);
		}
		else if(BUTTON_ENCODER_low() && flagPower==0)
		{
			about();
			_delay_ms(1000);
		}
		else if(flagPower==1)
		{
			rotaryEncoderNikBarzakov();
		}

	}
}

/********************************************************************************************
************************************ END OF APPLICATION *************************************
********************************************************************************************/

/********************************************************************************************
******************************** START OF MAIN APPLICATION **********************************
********************************************************************************************/

int main(void)
{

	init_all();				// inicializacia na vsichko
	ext0_intrpt_on();		// ENABLE interrupts to access IR DETECTION as call to function "IR_DECODER()" for -> SONY IR REMOTE
//	ext2_intrpt_on();

	sei();							// file "avr/interrupt.h"
//	SREG = (1<<I);

	LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1
	while(1)
	{
		buttons_press();	// izchakvane za natiskane na buton
	}
	return 1;
}

/********************************************************************************************
********************************* END OF MAIN APPLICATION ***********************************
********************************************************************************************/









/********************************************************************************************
****************************** START USED LIBRARIES AND HELP ********************************
*********************************************************************************************

***************
** WEB SITES **
***************
http://www.protostack.com/blog/2010/05/introduction-to-74hc595-shift-register-controlling-16-leds/
http://www.microcontroller.net
http://www.avrbeginners.net

=== SPI ===
http://www.avrbeginners.net/architecture/spi/spi.html
http://www.embeddedrelated.com/groups/lpc2000/show/16257.php

=== TWI ===
http://www.avrbeginners.net/architecture/twi/twi.html
http://www.mikrocontroller.net/topic/18864

=== IR ===
// http://www.electro-tech-online.com/threads/attiny2313-sirc-help-please.93815/

=== RTC DS1307 ===
http://wiring.org.co/learning/libraries/realtimeclock.html
http://www.glacialwanderer.com/hobbyrobotics/?p=12

=== LCD HD47800 ===
http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=116009

***************
**** BOOKS ****
***************
"C Programming in AVR Studio", SJVALLEY
"AVR GCC Tutorial (WinAVR)" by takashi
"The AVR microcontroller and embedded system: using Assembly and C", MUHAMMAD ALI MAZIDI, SERMAD NAIMI, SEPER NAIMI, 2011 Pearson Education, Inc., publishing as Prentice Hall

"Atmel AVR Microcontroller Primer: Programming and Interfacing", STEVEN F. BARRETT, DANIEL J. PACK, Morgan & Claypool Publishers


****************
** DATASHEETS **
****************
"doc8155.pdf" - ATmega32A(Secured), from http://www.atmel.com/.....


*********************************************************************************************
******************************* END USED LIBRARIES AND HELP *********************************
********************************************************************************************/


/********************************************************************************************
****************************** START MORE USED CODE AND HELP ********************************
********************************************************************************************/


/*

//	int Var1,Var2;	// http://www.ustr.net/infrared/sony.shtml

	while(1)	// IR RED // include util/delay.h
	{
		if(IR_RECEIVER_low())	// bit_is_clear(PINB,IR_RECEIVER)
		{
			_delay_us(3*600);

			if(IR_RECEIVER_high())
			{
				LED_high_DISPLAYLED_low(); // LED ON
			}
			else
			{
				//LED_low_DISPLAYLED_high();
			}
		}
		else
		{
			break;
		}
	}

	for(;;)
	{
	}

	return 1;
}


*/


//#define IR_RECEIVER  PB2	// PINB2		// TSOP2240 for 40kHz IR Receiver for SONY, SIRC Protocol

//#define IR_RECEIVER_low()  bit_is_clear(PINB,IR_RECEIVER)
//#define IR_RECEIVER_high() bit_is_set(PINB,IR_RECEIVER)


/*
Start
Logic "0"
PIN is LOW  for	_delay_us(3*600);
PIN is HIGH for	_delay_us(600);


Logic "0"
PIN is LOW  for	_delay_us(600);
PIN is HIGH for	_delay_us(600);

Logic "1"
PIN is LOW  for	_delay_us(600);
PIN is HIGH for	_delay_us(2*600);
*/
/*
The following procedure to detect and identify the code, will work
with ANY microcontroller / microprocessor.

1)  Set Var1 = 8,  Var2 = 0

2)  Start by waiting the signal to go DOWN - This will be the START Bit  (T3 time).

3)  Wait for the signal to go UP - This will be the start of the bit.

4)  Wait for the signal to go DOWN - This is the real thing, if short, bit = 0, if long = 1.

5)  Now Wait 750 to 950 microseconds.

6)  Measure the Signal Level.

7)  If the Signal is UP - Received Bit is ZERO
    - Set Carry Bit = 0
    - Rotate Right Var1 (Carry Bit enters MSB Var1)
    - Rotate Right Var2 (Var1 Carry Bit enters MSB Var2 and Var2 bit 0 goes to Carry).
    - Check Carry Bit, if ON Goto [9], if OFF Goto [4]

8)  If the Signal is DOWN - Received Bit is ONE
     - Set Carry Bit = 1
     - Rotate Right Var1 (Carry Bit enters MSB Var1)
     - Rotate Right Var2 (Var1 Carry Bit enters MSB Var2 and Var2 bit 0 goes to Carry).
     - Check Carry Bit, if ON Goto [9], if OFF Goto [3]
     Observe that here it goes back to [3] and not [4] as in [7]
     This is because if the signal still DOWN, you need to wait it goes UP,
     in [7] it is already in UP level, so it goes directly to the next step [4].

9)  Here it already read all 12 bits.
     Var1 Contains 8 bits, Var2 contains 4.
     The Left 5 bits of Var1 is the Address.
     The Right 3 bits of Var1 + 4 Left bits of Var2 form the Command. 

     If you shift right Var1 + Var2 3 times, you will have the Right 5 bits of Var1 = Address
     and the Left 7 bits of Var2 = Command.

By doing this sequence, you only need a timming routine of 750 to 950
microseconds, and don't need to measure individual bits.

Observe loaded VAR1 with hex 8 (binary 00001000) at entry, this bit will exit
into Carry only after 12 times "Rotate Right  VAR1 + VAR2", since it is 4 bits on Var1
plus  8 bits of Var2 to this happen.  It is used as a Rotation Right Counter Flag.

*/

//static const uint8_t
//PROGMEM alphabet_table [] = { o,n,o,f,f };
/********************************************************************************************
******************************* END MORE USED CODE AND HELP *********************************
********************************************************************************************/
