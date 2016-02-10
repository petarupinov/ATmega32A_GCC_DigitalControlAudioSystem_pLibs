/*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;*********************************************************************;;
;;*********************************************************************;;
;;************** Eng. Petar Upinov ************************************;;
;;***************** 07.10.2015 ****************************************;;
;;*********************************************************************;;
;;*********************************************************************;;
;;******** ATmega32 DCAS with new Library *****************************;;
;;*** (ATmega32 Digital Control Audio System) *************************;;
;;*********************************************************************;;
;;**************** Crystal 16MHz **************************************;;
;;*** 1. Edit Fuse bits: High 0xCA ; Low 0xFF *************************;;
;;*********************************************************************;;
;;*********************************************************************;;
;;** 1. Edit on date 07.10.2015 ***************************************;;
;;** 2. Edit on date 14.10.2015 - bit field struct test & type bool ***;;
;;** 3. Edit on date 15.10.2015 - update LCD lib h ********************;;
;;*********************************************************************;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;*/

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

#include "_Soft_Library_Pesho_v6_/24c64.h"
#include "_Soft_Library_Pesho_v6_/ds18x20.h"
#include "_Soft_Library_Pesho_v6_/i2c_twi.h"
#include "_Soft_Library_Pesho_v6_/ir_sirc.h"
#include "_Soft_Library_Pesho_v6_/lcd_hd44780_74hc595.h"
#include "_Soft_Library_Pesho_v6_/pga2310.h"
#include "_Soft_Library_Pesho_v6_/rotation_encoder.h"
#include "_Soft_Library_Pesho_v6_/rtc.h"
#include "_Soft_Library_Pesho_v6_/spi.h"
#include "_Soft_Library_Pesho_v6_/uart.h"
#include "_Soft_Library_Pesho_v6_/utility.h"

/*********************************************
** VARIABLES, CONSTANTS, ARRAYS, STRUCTURES **
*********************************************/

typedef int bool;
#define TRUE 1
#define FALSE 0

// https://en.wikipedia.org/wiki/Bit_field
// http://stackoverflow.com/questions/24933242/when-to-use-bit-fields-in-c
//#define flagStatusBtnOnOffBit0 0
struct flagStatusBtnOnOff
{
	unsigned int bit0 : 1;	// = 0;	// bit0 from struct bit field
	unsigned int bit1 : 1;	// = 1;	// bit1 from struct bit field
} *flagStatusBtnRegister;

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
#define BUTTON_ON_OFF  PB1	// PINB1
#define BUTTON_ESC     PB2	// PINB2
#define BUTTON_ENCODER PD3	// PIND3

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

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

/********************************************************************************************
*********************************** START OF INTERRUPTS *************************************
********************************************************************************************/

/********************************************************************************************
*********************************** START OF INTERRUPTS *************************************
********************************************************************************************/

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
	timer2_init();
	LCD_INIT();
}

void buttons_press()
{
	while(1)
	{
		if(BUTTON_ON_OFF_low())	// obj ptr flagStatusBtnRegister from struct flagStatusBtnOnOff
		{
			if(flagStatusBtnRegister->bit0 == 0)
			{
				flagStatusBtnRegister->bit0 = 1;
			}
			else
			{
				flagStatusBtnRegister->bit0 = 0;
			}
		}

		if(flagStatusBtnRegister->bit0 == 0)
		{
			LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1
			// flagStatusBtnRegister->bit0 = 0;
			LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);
			LCD_EXECUTE_DATA_ONE('A');
			//lcdDataString("FIRST ROW");
			_delay_ms(200);
		}
		else
		{
			LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1
			// flagStatusBtnRegister->bit0 = 1;
			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);
			LCD_EXECUTE_DATA_ONE('B');
			//lcdDataString("SECOND ROW");
			_delay_ms(200);
		}

/*		if(flagStatusBtnRegister->bit0 == 0)
		{
			LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1
		}
		else
		{
			LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1
		}
*/
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

