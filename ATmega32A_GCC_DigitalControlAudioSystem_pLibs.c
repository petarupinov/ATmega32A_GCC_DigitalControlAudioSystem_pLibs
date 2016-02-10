/*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;*****************************************************************************************;;
;;*****************************************************************************************;;
;;************** Eng. Petar Upinov ********************************************************;;
;;***************** 07.10.2015 ************************************************************;;
;;*****************************************************************************************;;
;;*****************************************************************************************;;
;;******** ATmega32 DCAS with new Library *************************************************;;
;;*** (ATmega32 Digital Control Audio System) *********************************************;;
;;*****************************************************************************************;;
;;**************** Crystal 16MHz **********************************************************;;
;;*** 1. Edit Fuse bits: High 0xCA ; Low 0xFF *********************************************;;
;;*****************************************************************************************;;
;;*****************************************************************************************;;
;;** 1. Edit on date 07.10.2015 ***********************************************************;;
;;** 2. Edit on date 14.10.2015 - bit field struct test & type bool ***********************;;
;;** 3. Edit on date 15.10.2015 - update LCD lib h ****************************************;;
;;** 4. Edit on date 15.10.2015 - correct LCD init, can't be first clr ********************;;
;;** 5. Edit on date 16.10.2015 - update and correct LCD clear lib c h, struct with flags *;;
;;** 6. Edit on date 17.10.2015 - update with custom characters ***************************;;
;;** 7. Edit on date 18.10.2015 - update all library files ********************************;;
;;** 8. Edit on date 26.10.2015 - update spi and pga2310 **********************************;;
;;** 9. Edit on date 26.10.2015 - update i2c, rtc, 24c64 **********************************;;
;;**10. Edit on date 26.10.2015 - update **************************************************;;
;;**11. Edit on date 26.10.2015 - update rotation encoder and little bit uart *************;;
;;**12. Edit on date 18.11.2015 - update utility lib version ******************************;;
;;**13. Edit on date 19.11.2015 - add functions ampliferOn/Off ****************************;;
;;**14. Edit on date 22.11.2015 - button power and escape works ***************************;;
;;*****************************************************************************************;;
;;** Used library version: _Soft_Library_Pesho_v0.06 ^^lcd updated^^ **********************;;
;;*****************************************************************************************;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;*/

/*************************************
** INCLUDE INTEGRATED LIBRARY FILES **
*************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
//#include <stdbool.h>		// type boolean true/false


/******************************************
** INCLUDE SPECIFIED PESHO LIBRARY FILES **
******************************************/

// Add C files to project: "ir_sirc.c", "rotation_encoder.c", "rtc.c", "spi.c", "utility.c"

#include "_Soft_Library_Pesho_/24c64.h"
#include "_Soft_Library_Pesho_/ds18x20.h"
#include "_Soft_Library_Pesho_/i2c_twi.h"
#include "_Soft_Library_Pesho_/ir_sirc.h"
#include "_Soft_Library_Pesho_/lcd_hd44780_74hc595.h"
#include "_Soft_Library_Pesho_/pga2310.h"
#include "_Soft_Library_Pesho_/rotation_encoder.h" // <---o
#include "_Soft_Library_Pesho_/rtc.h"
#include "_Soft_Library_Pesho_/spi.h"
#include "_Soft_Library_Pesho_/uart.h"	// <--------------o
#include "_Soft_Library_Pesho_/utility.h"

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




// https://en.wikipedia.org/wiki/Bit_field
// http://stackoverflow.com/questions/24933242/when-to-use-bit-fields-in-c
//#define flagStatusBtnOnOffBit0 0
struct flagStatus	 // bit fields from struct
{
	unsigned int flagPower	: 1;	// bit0: '0' = Power OFF, '1' = Power ON
	unsigned int flagMute	: 1;	// bit1: '0' = Mute OFF, '1' = Mute ON

} *flagStatusBits;

/*
struct flagStatusBtnOnOff
{
	unsigned int bit0 : 1;	// = 0;	// bit0 from struct bit field
	unsigned int bit1 : 1;	// = 1;	// bit1 from struct bit field
}flagStatusBtnRegister;
*/

unsigned char n = 0;

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

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void port_init();
void timer2_init();
void timer2_on();
void timer2_off();
void init_all();
char ampliferOn();
void ampliferOff();

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

/*****************************
** INITIZLIZATION OF TIMER2 **
*****************************/
void timer2_init()
{
	SFIOR = 0b00000010;		// Prescaler Reset Timer2 (bit1 –> PSR2)
	TCCR2 = 0b10000001;		// 0b10100001 - OC1A,OC1B - PWM;  0b10000001 - OC1A PWM, OC1B - Disabled, normal port.
	OCR2 = 0; // FAN PWM ON
}

/********************************************************************************************
********************************** END OF INITIALIZATIONS ***********************************
********************************************************************************************/

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/*************************************
******** DEFINITIONS OF TIMER ********
*************************************/
void timer2_on()	// Timer2 On
{
	TCCR2 = 0b10000001;		// 0b10100001 - OC1A,OC1B - PWM;  0b10000001 - OC1A PWM, OC1B - Disabled, normal port.
	OCR2 = 1; // FAN PWM ON
}

void timer2_off()	// Timer2 Off
{
	TCCR2 = 0b00000000;		// DISABLED OCOC1A - PWM, OC1B - Disabled, normal port.
	OCR2 = 0; // FAN PWM OFF
}

/********************
**** AMPLIFER ON ****
********************/
char ampliferOn()
{
	LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1

	LCD_CLEAR_CONTAIN();						// clear all contain on display
	LCD_COMMAND(LCD_SELECT_1ROW);				// select row 1
	LCD_DATA_STRING("    Amplifer On     ");	// 20 symbols
	LCD_COMMAND(LCD_SELECT_2ROW);				// select row 2
	LCD_DATA_STRING("P.UPINOV  P.STOYANOV");	// 20 symbols //	LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
	LCD_COMMAND(LCD_ON);						// LCD ON without CURSOR


//	FAN_PWM_SPEED1();	// KOMENTAR ZARADI SIMULACIQTA - MNOGO BAVI PRI SIMULACIQ S TIMER1
//			FAN_high();		// PORTD5 - FAN ON (logic "1")	NON PWM, NON TIMER1

//			volumeLeft = volumeRight = volumeValue [0];	// nulurane na volume control pri vsqko puskane

//	spi_start();
//	PGA2310_U6_SPI(volumeLeft, volumeRight);
//	spi_stop();

//	spi_start();
//	PGA2310_U7_SPI(volumeLeft, volumeRight);
//	spi_stop();

//	spi_start();
//	PGA2310_U8_SPI(volumeLeft, volumeRight);
//	spi_stop();

//	transmitUartString("Amplifer On\r\n");
//		uart_transmit("<AMPLIFER ON>\r\n", 15);	// "\r\n" - 2 symbols (not 4 symbols)

//	LCD_INIT();								// LCD INITIZLIZATION
//	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
//		LCD_EXECUTE_DATA(" << AMPLIFER  ON >> ",20);	// char "DATA", int 13 of chars of "DATA"
//lcdDataString("    Amplifer On");
//	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
//	LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
//			LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);	// select row 3
//			LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
//			LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);	// select row 4
//			LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
//	LCD_EXECUTE_COMMAND(LCD_ON);			// LCD ON without CURSOR

// RELAYS ON
//	REL_POWER_high();// RELAY POWER ON TRAFs		// PESHO COMMENT 14.08.2015, 21:10
//	_delay_ms(4000);								// PESHO COMMENT 14.08.2015, 21:10
//	relays_in1_6ch();	// RELAYS IN1 CHANNELS 6	// PESHO COMMENT 14.08.2015, 21:10
//	_delay_ms(700);									// PESHO COMMENT 14.08.2015, 21:10
//	relays_out_6ch();	// RELAYS OUT CHANNELS 6	// PESHO COMMENT 14.08.2015, 21:10

//			PGA2310_U8_SPI(volumeLeft, volumeRight);	// 'A', 'A', 0b01111110, 0b01111110
	return SUCCESS;
}

/********************
**** AMPLIFER OFF ****
********************/
void ampliferOff()
{
	LCD_CLEAR_CONTAIN();

	LCD_COMMAND(LCD_SELECT_1ROW);				// select row 1
	LCD_DATA_STRING("    Amplifer Off    ");	// 20 symbols

	LCD_COMMAND(LCD_OFF);						// LCD ON without CURSOR
	LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1

//			FAN_low();		// PORTD5 - FAN OFF (logic "0")  NON PWM, NON TIMER1

// RELAYS OFF
//	relays_out_off();	// RELAYS OUT CHANNELS 6	// PESHO COMMENT 14.08.2015, 21:10
//	_delay_ms(700);								// PESHO COMMENT 14.08.2015, 21:10
//	relays_in_off();	// RELAYS IN1 CHANNELS 6	// PESHO COMMENT 14.08.2015, 21:10
//	_delay_ms(700);								// PESHO COMMENT 14.08.2015, 21:10
//	REL_POWER_low();// RELAY POWER OFF				// PESHO COMMENT 14.08.2015, 21:10

//	transmitUartString("Standby\r\n");
//				uart_transmit("<STANDBY>\r\n", 11);		// "\r\n" - 2 symbols (not 4 symbols)

//	LCD_INIT();								// LCD INITIZLIZATION
//	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
//lcdDataString("       Standby");
//				LCD_EXECUTE_DATA(" >>  <STANDBY>   << ",20);		// char "DATA", int 13 of chars of "DATA"
//				_delay_ms(500);	// izchakvane - migasht efekt
//				LCD_INIT();								// LCD INITIZLIZATION
//				_delay_ms(250);	// izchakvane - migasht efekt, natiskane i otpuskane na buton - filtar treptqsht kontakt buton
//	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);	// select row 1
//lcdDataString("    Amplifer Off");
//				LCD_EXECUTE_DATA(" >> <IR STANDBY> << ",20);		// char "DATA", int 13 of chars of "DATA"
//	_delay_ms(500);	//

//	_delay_ms(50);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton

//	FAN_PWM_OFF();
//			LCD_EXECUTE_COMMAND(LCD_OFF);			// LCD OFF
//	LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1
}
/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

/********************************************************************************************
*********************************** START OF INTERRUPTS *************************************
********************************************************************************************/

/*****************************************
**** RESET EXT INTERRUPT  VECTOR 00 ******
*****************************************/
/*
ISR(RESET_vect)
{
}
*/

/*****************************************
**** EXTERNAL INTERRUPT 0 VECTOR 01 ******
*****************************************/
ISR(INT0_vect)
{
}

/*****************************************
**** EXTERNAL INTERRUPT 1 VECTOR 02 ******
*****************************************/
ISR(INT1_vect)
{
}

/*****************************************
**** EXTERNAL INTERRUPT 2 VECTOR 03 ******
*****************************************/
ISR(INT2_vect)
{
}

/*****************************************
**** TIMER 2 OUTPUT COMPARE VECTOR 04 ****
*****************************************/
ISR(TIMER2_COMP_vect)
{
}

/*****************************************
**** TIMER 2 OVERFLOW VECTOR 05 **********
*****************************************/
ISR(TIMER2_OVF_vect)
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
//	timer2_init();
	LCD_INIT();

}

void buttons_press()
{
	char test = 0;
	unsigned char pgaVolumeLeft, pgaVolumeRight;
	pgaVolumeLeft = pgaVolumeRight = 0b00001111;

	while(1)
	{
		if(BUTTON_ON_OFF_low() && flagStatusBits->flagPower == 0)	// obj ptr flagStatusBtnRegister from struct flagStatusBtnOnOff
		{
			test = ampliferOn();
			if(SUCCESS == test)
			{
				flagStatusBits->flagPower = 1;			// filter za buton ON
				_delay_ms(1000);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
			}
		}
		else if(BUTTON_ON_OFF_low() && flagStatusBits->flagPower == 1)
		{
			ampliferOff();
			flagStatusBits->flagPower = 0;			// filter za buton OFF
			_delay_ms(500);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton

		}
		else if(BUTTON_ESC_low() && flagStatusBits->flagPower == 1)
		{
			LCD_DATA_STRING("PRESSED BTN ESCAPE  ");	// 20 symbols
			LCD_COMMAND(LCD_ON);
			_delay_ms(500);
//			volumeUp();
//			_delay_ms(200);
		}
		else if(BUTTON_ENCODER_low() && flagStatusBits->flagPower == 1)
		{
			LCD_DATA_STRING("PRESSED BTN ENCODER ");	// 20 symbols
			LCD_COMMAND(LCD_ON);	// LCD_COMMAND(LCD_OFF);
			_delay_ms(500);
//			volumeDown();
//			_delay_ms(200);

		}
		else if(BUTTON_ESC_low() && flagStatusBits->flagPower == 0)
		{
//			LCD_COMMAND(LCD_ON);
//			_delay_ms(500);
//			setupMode();
//			_delay_ms(1000);
		}
		else if(BUTTON_ENCODER_low() && flagStatusBits->flagPower == 0)
		{
//			LCD_COMMAND(LCD_OFF);
//			_delay_ms(500);
//			about();
//			_delay_ms(1000);
		}
		else if(flagStatusBits->flagPower == 1)
		{
//			rotaryEncoderNikBarzakov();
		}














/****************************************************************************/

/*			if(flagStatusBits->flagPower == 1)
			{

				_delay_ms(200);
			}
		}*/



/*		if(BUTTON_ON_OFF_low() && flagPower==0)			// PINB1 - BUTTON ON/OFF -> ON
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
		}
		else if(BUTTON_ENCODER_low() && flagPower==1)
		{
			volumeDown();
			_delay_ms(200);

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
*/
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

	sei();							// file "avr/interrupt.h"
//	SREG = (1<<I);

	LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1
	while(1)
	{
//		struct flagStatusBtnOnOff flagStatusBtnRegister;	// obj flagStatusBtnRegister from struct flagStatusBtnOnOff
//		flagStatusBtnRegister.bit0 = 0;
		
		buttons_press();	// izchakvane za natiskane na buton
	}
	return 1;
}

/********************************************************************************************
********************************* END OF MAIN APPLICATION ***********************************
********************************************************************************************/

