/*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;***********************************************;;
;;***********************************************;;
;;************** Eng. Petar Upinov **************;;
;;***************** 06.12.2012 ******************;;
;;***********************************************;;
;;***********************************************;;
;;******** ATmega32 DCAS Display RTC only *******;;
;;*** (ATmega32 Digital Control Audio System) ***;;
;;***********************************************;;
;;**************** Crystal 16MHz ****************;;
;;*** 1. Edit Fuse bits: High 0xCA ; Low 0xFF ***;;
;;***********************************************;;
;;***********************************************;;
;;** 1. Edit on date 19.08.2015 *****************;;
;;** 2. Edit on date 21.08.2015 *****************;;
;;***********************************************;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;*/

/******************
** INCLUDE FILES **
******************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
//#include <stdbool.h>		// type boolean true/false
//typedef int bool;			// http://stackoverflow.com/questions/1921539/using-boolean-values-in-c

typedef enum bool	// const typedef enum bool	// bool or boolean // http://stackoverflow.com/questions/1909825/error-with-the-declaration-of-enum
{
	false,			// false = 0
	true			// true  = 1
};

#include "lcd_hd44780_74hc595.h"

// -----= Real Time Clock =-----
// flags
unsigned char clockFlag = 0;
unsigned char temp = 0;		// set hours
unsigned char flagHoursHighNibble = 0;
unsigned char counter = 0;

const char *dayOfWeeks [] = {"    Error  Days", "   Sunday", "   Monday", "  Tuesday", " Wednesday", "  Thursday", "   Friday", "  Saturday"};

// -----= IR REMOTE DECODER =-----
// ir flags
char flagPower = 0;		// filter dali amplifer e on ili off 
char flagEncoderA = 0;	// filter dali e natisnat EncoderA button
char flagEncoderB = 0;	// filter dali e natisnat EncoderB button
unsigned char flagRotaryEncoderNikBarzakov = 0;	// izpolzvan li e rotaionniq enkoder ? NE->flagRotaryEncoderNikBarzakov=0; DA->flagRotaryEncoderNikBarzakov = 1;
unsigned char mute = 0;		// change name for case in ir scan decode commands
unsigned char flagRTC = 0;	// set/unset RTC

// ir pins
#define irPin (PIND & (1<<PIND2))	//	#define irPin (PINB & (1<<PINB0))

// ir data
unsigned char irAddress;
unsigned char irCommand;

// ir constants
// -----= IR REMOTE DECODER =-----
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


// rtc clock
	unsigned char dataClock = 0b00000000;
//	unsigned char seconds = 0, minutes = 0, hours = 0, days = 1, dates = 1, months = 1;
//	unsigned int years = 2010;



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

/****************************************************
**** DEFINITION OF LED AND DISPLAY BACKLIGHT LED ****
****************************************************/
#define LED_DISPLAYLED_PIN  PD4
#define LED_DISPLAYLED_PORT PORTD

#define LED_low_DISPLAYLED_high()	(LED_DISPLAYLED_PORT&=~_BV(LED_DISPLAYLED_PIN))
#define LED_high_DISPLAYLED_low()	(LED_DISPLAYLED_PORT|=_BV(LED_DISPLAYLED_PIN))

/**************************
*  DEFINITION IR RECEIVER *
**************************/
#define IR_RECEIVER  PD2	// PIND2		// TSOP2240 for 40kHz IR Receiver for SONY, SIRC Protocol

#define IR_RECEIVER_low()  (bit_is_clear(PIND,IR_RECEIVER))
#define IR_RECEIVER_high() (bit_is_set(PIND,IR_RECEIVER))

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

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/

// ---- main functions ----

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

void i2c_init();
void i2c_start(void);
void i2c_stop();
void i2c_write(byte data);
byte i2c_read(byte isLast);

void init_all();
void button_pressed_on_off();
void RTC_1307_GET();
void RTC_1307_SET();

void rotaryEncoderNikBarzakov();

void IR_DECODER();		// funkciq za razpoznavane na signal po IR
void GetSIRC();			// funkciq za dekodirane na IR

// ---- app functions ----

void showClock();
void setClock(unsigned char temp);

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
}

//=====================================================================================================================
void RTC_1307_GET()
{
	byte bufferSecond, bufferMinute, bufferHour, bufferDayOfWeek, bufferDate, bufferMonth, bufferYear, bufferControl, bufferRamAddress0, bufferRamAddress1; // bufferDayOfWeek (address 0x03)
	byte byteSecond, byteMinute, byteHour, byteDayOfWeek, byteDate, byteMonth, byteYear; // variables for convert DEC to BCD for LCD and UART for Time and Date

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// START READ ADDRESS REGISTER DATA, it is SECONDS ADDRESS REGISTER ACCESS

	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_READ);		// RTC DS1307 ADDRESS ACCESS READ
	bufferSecond	= i2c_read(0);			// SECONDS DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferMinute	= i2c_read(0);			// MINUTES DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferHour		= i2c_read(0);			// HOURS DATA VALUE		// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	bufferDayOfWeek	= i2c_read(0);			// DATE DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferDate		= i2c_read(0);			// DATE DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferMonth		= i2c_read(0);			// MONTHS DATA VALUE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	bufferYear		= i2c_read(0);			// YEARS DATA VALUE		// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	
	bufferControl	= i2c_read(0);			// GET CONTROL DATA
	bufferRamAddress0 = i2c_read(0);		// GET DATA FROM RAM ADDRESS 0x08 OF RTC DS1307
	bufferRamAddress1 = i2c_read(1);		// GET DATA FROM RAM ADDRESS 0x09 OF RTC DS1307
	
	i2c_stop();

	byteSecond	= ((bufferSecond/16*10) + (bufferSecond%16));	// bcdToDec		// Get HEX format -> convert HEX to DEC
	byteMinute	= ((bufferMinute/16*10) + (bufferMinute%16));	// bcdToDec		// Get HEX format -> convert HEX to DEC
	byteHour	= ((bufferHour/16*10) + (bufferHour%16));		// bcdToDec		// Get HEX format -> convert HEX to DEC

	byteDayOfWeek = ((bufferDayOfWeek/16*10) + (bufferDayOfWeek%16));	// bcdToDec		// Get HEX format -> convert HEX to DEC

	byteDate	= ((bufferDate/16*10) + (bufferDate%16));		// bcdToDec		// Get HEX format -> convert HEX to DEC
	byteMonth	= ((bufferMonth/16*10) + (bufferMonth%16));		// bcdToDec		// Get HEX format -> convert HEX to DEC
	byteYear	= ((bufferYear/16*10) + (bufferYear%16));		// bcdToDec		// Get HEX format -> convert HEX to DEC
// -----------------------------------
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
	lcdDataString("                    ");	// 20 intervals
// -----------------------------------
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
	lcdDataString(dayOfWeeks[byteDayOfWeek]);	// ukazatel koito sochi adresa (ne stoinostta na parviq element) na elementa i parviq simvol na stringa
// ---/ OLD STYLE \---
/*	switch(byteDayOfWeek)					// ? dobavqne na izkustvena '0', kogato se izvika chislo bez parvata desetica, ot 0 do 9
	{								// ? dobavqne na izkustvena '0', kogato se izvika chislo bez parvata desetica, ot 0 do 9
		case 1:
		{
			lcdDataString("   Sunday");
			break;
		}
		case 2:
		{
			lcdDataString("   Monday");
			break;
		}
		case 3:
		{
			lcdDataString("  Tuesday");
			break;
		}
		case 4:
		{
			lcdDataString(" Wednesday");
			break;
		}
		case 5:
		{
			lcdDataString("  Thursday");
			break;
		}
		case 6:
		{
			lcdDataString("   Friday");
			break;
		}
		case 7:
		{
			lcdDataString("  Saturday");
			break;
		}
		default:
		{
			lcdDataString("    Error  Days");
		}
	}
*/
// ---\ OLD STYLE /---
	LCD_EXECUTE_DATA_ONE(' ');		// 1 space between DayOfWeek and Date

	if(byteDate<10)					// 10?? meseci 1-12
	{								// dobavqne na izkustvena '0' za desetici
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0' za desetici
	}								// dobavqne na izkustvena '0' za desetici
	lcdDataInt(byteDate);
	LCD_EXECUTE_DATA_ONE('.');
	if(byteMonth<13)				// 13?? meseci 1-31
	{								// dobavqne na izkustvena '0' za desetici
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0' za desetici
	}								// dobavqne na izkustvena '0' za desetici
	lcdDataInt(byteMonth);
	LCD_EXECUTE_DATA_ONE('.');
	if(byteYear<10)
	{								// 10?? godini 1900 -> '00, ako e X0, kade X e ot 0-9 desetici na godini
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0' za desetici
	}								// dobavqne na izkustvena '0' za desetici
	lcdDataInt(byteYear);			// dobavqne na izkustvena '0' za desetici
// -----------------------------------
	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);	// select row 3
// -----------------------------------
	lcdDataString("      ");	// 6 space before Time
	if(byteHour<10)					// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	{								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	}								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	lcdDataInt(byteHour);
	LCD_EXECUTE_DATA_ONE(':');
	if(byteMinute<10)				// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	{								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	}								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	lcdDataInt(byteMinute);
	LCD_EXECUTE_DATA_ONE(':');
	if(byteSecond<10)
	{								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
		LCD_EXECUTE_DATA_ONE('0');	// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	}								// dobavqne na izkustvena '0', za desetici, ot 0 do 9
	lcdDataInt(byteSecond);			// dobavqne na izkustvena '0', za desetici, ot 0 do 9
// -----------------------------------
	LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);	// select row 4
	lcdDataString("                    ");	// 20 intervals
// -----------------------------------
//-------------------------------- OLD CONCEPTION --------------------------------
//	byteSecond = ('0'+ (bufferSecond>>4));		// convert DEC to BCD Seconds	// 0-F, kato ima ogranichenie 0-9 ot samiq DS1307 ili bi trqbvalo da ima proverka za greshka, ako stoinostta e > 9 !!!
//	byteSecond = ('0'+ (bufferSecond & 0x0F));	// convert DEC to BCD Minutes
}

//=====================================================================================================================
void setClock(unsigned char temp)
{
	byte bufferSecond, bufferMinute, bufferHour, bufferDayOfWeek, bufferDate, bufferMonth, bufferYear, bufferControl, bufferRamAddress0, bufferRamAddress1; // bufferDayOfWeek (address 0x03)
	byte byteSecond, byteMinute, byteHour, byteDayOfWeek, byteDate, byteMonth, byteYear; // variables for convert DEC to BCD for LCD and UART for Time and Date
	unsigned char flagExitSetupClock = true, flagSetClock = true, flagChoose = false, flagSetHour = true, flagSetMinute = true, flagSetSecond = true, flagSetDay = true, flagSetDate = true, flagSetMonth = true, flagSetYear = true;
	unsigned char flagNext;				//	unsigned char flagNextNibble = true;

	LCD_CLEAR_ALL();
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("Setup Clock: YES/NO?");
	while(flagExitSetupClock)
	{
		if((ENCODER_A_low()) && flagSetClock==true)
		{
			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
			lcdDataString("YES                 ");
			flagChoose = true;
			flagNext = true;
		}
		else if((ENCODER_B_low()) && flagSetClock==true)
		{
			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
			lcdDataString("NO                  ");
			flagChoose = true;
			flagNext = false;
		}
		else if(BUTTON_ESC_low() && flagSetClock==true && flagChoose == true)
		{	
			flagChoose = false;
			if(flagNext)
			{
				LCD_CLEAR_ALL();
				LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
				lcdDataString("Hour:               ");
				while(flagNext)
				{
					if((ENCODER_A_low()) && flagSetHour==true)
					{
						LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
						lcdDataString("RotHour+            ");
						flagChoose = true;
						//flagNext = false;
					}
					else if((ENCODER_B_low()) && flagSetHour==true)
					{
						LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
						lcdDataString("RotHour-            ");
						flagChoose = true;
						//flagNext = false;
					}
					else if(BUTTON_ESC_low() && flagSetHour==true && flagChoose == true)
					{
						flagChoose = false;
						if(flagNext)
						{
							LCD_CLEAR_ALL();
							LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
							lcdDataString("Minute:             ");
							while(flagNext)
							{
								if((ENCODER_A_low()) && flagSetMinute==true)
								{
									LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
									lcdDataString("RotMin+             ");
									flagChoose = true;
									//flagNext = false;
								}
								else if((ENCODER_B_low()) && flagSetMinute==true)
								{
									LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
									lcdDataString("RotMin-             ");
									flagChoose = true;
									//flagNext = false;
								}
								else if(BUTTON_ESC_low() && flagSetSecond==true && flagChoose == true)
								{
									flagChoose = false;
									if(flagNext)
									{
										LCD_CLEAR_ALL();
										LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
										lcdDataString("Second:             ");
										while(flagNext)
										{
											if((ENCODER_A_low()) && flagSetSecond==true)
											{
												LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
												lcdDataString("RotSec+             ");
												flagChoose = true;
												//flagNext = false;
											}
											else if((ENCODER_B_low()) && flagSetSecond==true)
											{
												LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
												lcdDataString("RotSec-             ");
												flagChoose = true;
												//flagNext = false;
											}
											else if(BUTTON_ESC_low() && flagSetDay==true && flagChoose == true)
											{
												flagChoose = false;
												if(flagNext)
												{
													LCD_CLEAR_ALL();
													LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
													lcdDataString("Day:                ");
													while(flagNext)
													{
														if((ENCODER_A_low()) && flagSetDay==true)
														{
															LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
															lcdDataString("RotDay+             ");
															flagChoose = true;
															//flagNext = false;
														}
														else if((ENCODER_B_low()) && flagSetDay==true)
														{
															LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
															lcdDataString("RotDay-             ");
															flagChoose = true;
															//flagNext = false;
														}
														else if(BUTTON_ESC_low() && flagSetDate==true && flagChoose == true)
														{
															flagChoose = false;
															if(flagNext)
															{
																LCD_CLEAR_ALL();
																LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
																lcdDataString("Date:               ");
																while(flagNext)
																{
																	if((ENCODER_A_low()) && flagSetDate==true)
																	{
																		LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																		lcdDataString("RotDate+            ");
																		flagChoose = true;
																		//flagNext = false;
																	}
																	else if((ENCODER_B_low()) && flagSetDate==true)
																	{
																		LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																		lcdDataString("RotDate-            ");
																		flagChoose = true;
																		//flagNext = false;
																	}
																	else if(BUTTON_ESC_low() && flagSetMonth==true && flagChoose == true)
																	{
																		flagChoose = false;
																		if(flagNext)
																		{
																			LCD_CLEAR_ALL();
																			LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
																			lcdDataString("Month:              ");
																			while(flagNext)
																			{
																				if((ENCODER_A_low()) && flagSetMonth==true)
																				{
																					LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																					lcdDataString("RotMonth+           ");
																					flagChoose = true;
																					//flagNext = false;
																				}
																				else if((ENCODER_B_low()) && flagSetMonth==true)
																				{
																					LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																					lcdDataString("RotMonth-           ");
																					flagChoose = true;
																					//flagNext = false;
																				}
																				else if(BUTTON_ESC_low() && flagSetYear==true && flagChoose == true)
																				{
																					flagChoose = false;
																					if(flagNext)
																					{
																						LCD_CLEAR_ALL();
																						LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
																						lcdDataString("Year:               ");
																						while(flagNext)
																						{
																							if((ENCODER_A_low()) && flagSetYear==true)
																							{
																								LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																								lcdDataString("RotYear+            ");
																								flagChoose = true;
																								//flagNext = false;
																							}
																							else if((ENCODER_B_low()) && flagSetYear==true)
																							{
																								LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																								lcdDataString("RotYear-            ");
																								flagChoose = true;
																								//flagNext = false;
																							}
																							else if(BUTTON_ESC_low() && flagSetYear==true && !flagChoose == true)
																							{
																								flagChoose = false;
																								if(flagNext)
																								{
																									LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
																									lcdDataString("EXIT RTC SETUP EXIT!");
																								//	while(flagNext)
																								//	{
																								//		lcdDataString("EXIT RTC SETUP EXIT1");
																								//		LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																								//		lcdDataString("EXIT RTC SETUP EXIT2");
																								//	}
																								}
																							// EXIT
																							//	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																							//	lcdDataString("EXIT CLOCK          ");
																							//	_delay_ms(1000);
																							//	flagExitSetupClock = false;
																								// EXIT
																							//	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
																							//	lcdDataString("NO                  ");
																							//	flagChoose = true;
																							//	flagNext = false;
																							//	break;
																							}
																							//else{}
																						}
																					}
																				}
																			}
																		}
																	}
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
			else if(!flagNext)
			{
				LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
				lcdDataString("EXIT CLOCK          ");
				_delay_ms(1000);
				flagExitSetupClock = false;
			}
			else
			{
				
			}
		}
	}
// ---/ OLD STYLE \---
/*	// CHASOVNIKA E SVEREN
	i2c_start();
	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
	i2c_write(RTC_DS1307_I2C_SECONDS);			// MINUTES ADDRESS REGISTER ACCESS
//	i2c_write(temp);							// MINUTES ADDRESS REGISTER ACCESS	//	i2c_write(bufferMinute+1);	// 0x00+1	// MINUTES DATA VALUE
	i2c_write(0x30);	// SECOND	hex 30 = dec 48 secundi	(gledai HEX cifrite)
	i2c_write(0x06);	// MINUTE	hex  5 = dec  5 minuti	(gledai HEX cifrite)
	i2c_write(0x21);	// HOUR		hex 21 = dec 33 chas	(gledai HEX cifrite)
	i2c_write(0x02);	// DAY		hex  2 = dec  2 den ot sedmicata (gledai HEX cifrite)
	i2c_write(0x24);	// DATE		hex 24 = dec 36 data	(gledai HEX cifrite)
	i2c_write(0x08);	// MONTH	hex  8 = dec  8 mesec	(gledai HEX cifrite)
	i2c_write(0x15);	// YEAR		hex 15 = dec 21 godina	(gledai HEX cifrite)
	i2c_stop();

	LCD_CLEAR_ALL();	//LCD_INIT();
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("   CLOCK  IS  SET   ");
*/
	LCD_CLEAR_ALL();
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("CHASOVNIKA E SVEREN!");
	_delay_ms(500);
	LCD_CLEAR_ALL();
	_delay_ms(500);
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("CHASOVNIKA E SVEREN!");
	_delay_ms(500);
	LCD_CLEAR_ALL();
	_delay_ms(500);
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	lcdDataString("CHASOVNIKA E SVEREN!");
	_delay_ms(500);
	LCD_CLEAR_ALL();



//	RTC_1307_SET();

/*
//	i2c_start();
//	i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
//	for(int RTC_Registers = 0; RTC_Registers < 8; RTC_Registers++) // SECONDS ADDRESS REGISTER ACCESS is 0x00
//	{
//		i2c_write(0b00000000);	// CLEAR RTC REGISTERS: 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // FROM SECONDS to CONTROL REGISTER
//	}
//	i2c_stop();
*/
// ---\ OLD STYLE /---
}

//=====================================================================================================================
void showClock()
{
	RTC_1307_GET();
}

/*********************************
** DEFINITION ENCODER FUNCTIONS **
*********************************/
void rotaryEncoderNikBarzakov()
{
//	unsigned char tempDigitA = 0, tempDigitB = 0;

	if((ENCODER_A_low()) && flagPower==1)			// PINB1 - BUTTON ON/OFF -> ON
	{
//		ampliferOn();	// izvikvane na potrebitelska funkciq
		LCD_CLEAR_ALL();	//LCD_INIT();
		lcdDataString("Pressed ENC_A AmpOn");	// izpisvane na tekst
		_delay_ms(250);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
//	HOURS INCREMENT
		if(temp>10)
		{
			temp = 0;
		}
		temp++;
		LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
		lcdDataString("Temp: ");	// lcdDataInt(temp);
		lcdDataInt(temp);
//		LCD_EXECUTE_DATA_ONE(tempDigitA);	// lcdDataInt(byte0);
//		LCD_EXECUTE_DATA_ONE(tempDigitB);	// lcdDataInt(byte1);
	}
	else if((ENCODER_B_low()) && flagPower==1)	// PINB1 - BUTTON ON/OFF -> OFF
	{
//		ampliferOff();	// izvikvane na potrebitelska funkciq
		LCD_CLEAR_ALL();	//LCD_INIT();
		lcdDataString("Pressed ENC_B AmpOn");	// izpisvane na tekst
		_delay_ms(250);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
//	HOURS DECREMENT
		if(temp<1)
		{
			temp = 9;
		}
		temp--;
		LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
		lcdDataString("Temp: ");	// lcdDataInt(temp);
		lcdDataInt(temp);
//		LCD_EXECUTE_DATA_ONE(tempDigitA);	// lcdDataInt(byte0);
//		LCD_EXECUTE_DATA_ONE(tempDigitB);	// lcdDataInt(byte1);
	}
	else if((ENCODER_A_low()) && flagPower==0)			// PINB1 - BUTTON ON/OFF -> ON
	{
//		ampliferOn();	// izvikvane na potrebitelska funkciq
		LCD_CLEAR_ALL();	//LCD_INIT();
		lcdDataString("Pressed ENC_A AmpOff");	// izpisvane na tekst
		_delay_ms(250);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
//		flagEncoderA = 1;
	}
	else if((ENCODER_B_low()) && flagPower==0)	// PINB1 - BUTTON ON/OFF -> OFF
	{
//		ampliferOff();	// izvikvane na potrebitelska funkciq
		LCD_CLEAR_ALL();	//LCD_INIT();
		lcdDataString("Pressed ENC_B AmpOff");	// izpisvane na tekst
		_delay_ms(250);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
//		flagEncoderB = 1;
	}
	else
	{}
/*
	if((ENCODER_A_low()) && (ENCODER_B_low()))			// A0, B0
	{
		_delay_us(50);	// delay before next check bits
		if((ENCODER_A_high()) && (ENCODER_B_low()))		// A1, B0
		{
			// ---> Clockwise; Zavartane po posoka na chasovnikovata strelka.
// VOLUME UP
//			volumeUp();
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
//			volumeDown();
		}
	}
	else
	{
			// do nothing
	}
*/
	flagRotaryEncoderNikBarzakov = 1;
}
/************************************
** DEFINITION IR DECODER FUNCTIONS **
************************************/
// SCAN for ADDRESS and COMMAND
void IR_DECODER()
{
	GetSIRC();
	if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A && irCommand == IR_REMOTE_COMMAND_RM_677A_STANDBY) || (irAddress == IR_REMOTE_CAR_DEVICE_RM_X157 && irCommand == IR_REMOTE_COMMAND_RM_X157_OFF)) && flagPower==0)		// IR POWER -> ON
	{
		ampliferOn();
		flagPower = 1;			// filter za buton ON
		_delay_ms(1000);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A && irCommand == IR_REMOTE_COMMAND_RM_677A_STANDBY) || (irAddress == IR_REMOTE_CAR_DEVICE_RM_X157 && irCommand == IR_REMOTE_COMMAND_RM_X157_OFF)) && flagPower==1)	// IR POWER -> OFF
	{
		ampliferOff();
		flagPower = 0;			// filter za buton OFF
//		break;
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_VOLUP)) && flagPower==1)					// Sony TV & CarAudio IR Remote Device - "VOLUME UP"
	{	// VOLUME UP
//		volumeUp();
//		break;
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_VOLDN)) && flagPower==1)					// Sony TV & CarAudio IR Remote Device - "VOLUME DOWN"
	{	// VOLUME DOWN
//		volumeDown();
//		break;
	}
	else if((((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ATT)) && mute==0) && flagPower==1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
	{	// MUTE
//		muteOn();
		mute = 1;
//		break;
	}
	else if((((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ATT)) && mute==1) && flagPower==1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> OFF
	{	// UNMUTE
//		muteOff();
		mute = 0;
//		break;
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_SOURCE)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "SOURCE"
	{
		setClock(temp);
		_delay_ms(200);	
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_MENU)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "MENU"
	{
		showClock();
		_delay_ms(200);	
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_DSPL)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "DSPL"
	{
		LCD_CLEAR_ALL();	//LCD_INIT();
		LED_high_DISPLAYLED_low();
		_delay_ms(200);	
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_SCRL)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "SCRL" -> TEMPERATURE
	{
//		temperature();
		_delay_ms(200);
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_MODE)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "MODE"
	{
		setupMode();
		_delay_ms(200);
//		break;
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_LIST)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "LIST"
	{
		about();
		_delay_ms(200);	
	}
	else if((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_UP))
	{
		if(flagRTC == 1)
		{
//			HOURS INCREMENT
			temp++;
			LCD_CLEAR_ALL();	//LCD_INIT();
			LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
			lcdDataString("Hours: ");				// lcdDataInt(temp);
		}
	}
	else if((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_DOWN))
	{
//		HOURS DECREMENT
		if(flagRTC == 1)
		{
			temp--;
			LCD_CLEAR_ALL();	//LCD_INIT();
			LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
			lcdDataString("Hours: ");				// lcdDataInt(temp);
		}
	}
	else if((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ENTER))
	{	
		if(flagRTC == 1)
		{
			// store variables to to rtc ds1307
			i2c_start();
			i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
			i2c_write(RTC_DS1307_I2C_HOURS);			// HOURS ADDRESS REGISTER ACCESS
			i2c_write(temp);							// HOURS DATA VALUE
			i2c_stop();

			flagRTC = 0;
		}
	}
	else
	{
		// DO NOTING
	}
	_delay_ms(200);
}

// DECODE ADDRESS and COMMAND
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
	LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1

	LCD_CLEAR_ALL();	//LCD_INIT();
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
}

/********************
**** AMPLIFER OFF ****
********************/
void ampliferOff()
{
	LCD_CLEAR_ALL();	//LCD_INIT();
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

//			LCD_EXECUTE_COMMAND(LCD_OFF);			// LCD OFF
// !!! LCD komandata na dolniq red pri realni usloviq trqbva da se mahne komentara za da se izkliuchi podsvetkata mu!!!
//	LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1
}

/****************************
**** SETUP MODE FUNCTION ****
****************************/
void setupMode()
{
	LCD_CLEAR_ALL();	//LCD_INIT();
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
	LCD_CLEAR_ALL();	//LCD_INIT();
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);		// select row 1
	LCD_EXECUTE_DATA(" INTELIGENT AUDIO",17);	// char "DATA", int 16 of chars of "DATA"
	LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);		// select row 2
	LCD_EXECUTE_DATA(" AMPLIFER with CPU",18);	// char "DATA", int 17 of chars of "DATA"
	LCD_EXECUTE_COMMAND(LCD_SELECT_3ROW);		// select row 3
	LCD_EXECUTE_DATA(" FW. Version beta43",19);	// char "DATA", int 18 of chars of "DATA"
	LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);		// select row 4
	LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);// char "DATA", int 20 of chars of "DATA"
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
//	ext0_intrpt_init();		// SONY IR REMOTE
//	ext1_intrpt_init();
//	ext2_intrpt_init();
//	timer0_init();
//	timer1_init();	// KOMENTAR ZARADI SIMULACIQTA - MNOGO BAVI PRI SIMULACIQ S TIMER1
//	timer2_init();
	i2c_init();

	LCD_INIT();								// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_1ROW);	// select row 1
	LED_low_DISPLAYLED_high();				// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1
	LCD_EXECUTE_COMMAND(LCD_ON);			// >=-***-=< LCD DISPLAY ON >=-***-=< // LCD ON without CURSOR
}

void buttons_press()
{
//	volumeIndex = volume_view = 0;	// vinagi VOLUME = 0 pri startirane

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
		else if(BUTTON_ENCODER_low() && flagPower==1)
		{
			flagRotaryEncoderNikBarzakov = 1;
			_delay_ms(200);

//			RTC_1307_SET();
//			LCD_EXECUTE_DATA("STORE EEPROM",12);
//			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
//			LCD_EXECUTE_DATA(" SET CLOCK ",11);
		}
		else if(BUTTON_ENCODER_low() && flagPower==0)
		{
			setupMode();
			_delay_ms(1000);
		}
		else if(BUTTON_ESC_low() && flagPower==0)
		{	
			LCD_EXECUTE_COMMAND(LCD_CLEAR);	//LCD_CLEAR_ALL();	//LCD_INIT();
			while(flagPower==0)
			{
				showClock();
				if(BUTTON_ON_OFF_low() && flagPower==0)			// PINB1 - BUTTON ON/OFF -> ON
				{
					flagPower = 1;
				}
				_delay_ms(1000);
			}
//			about();
		}
		else if(BUTTON_ESC_low() && flagPower==1)
		{
			setClock(temp);
/*			switch(clockFlag)
			{
				case 1:
				{
					setClock(temp);	// set RTC_DS1307_I2C_SECONDS	// clock Flag = 1
					break;
				}
				case 2:
				{
					setClock(temp);	// set RTC_DS1307_I2C_MINUTES	// clock Flag = 1
					break;
				}
				case 3:
				{
					setClock(temp);	// set RTC_DS1307_I2C_HOURS	// clock Flag = 1
					break;
				}
				case 4:
				{
					setClock(temp);	// set RTC_DS1307_I2C_DAY	// clock Flag = 1
					break;
				}
				case 5:
				{
					setClock(temp);	// set RTC_DS1307_I2C_DATE	// clock Flag = 1
					break;
				}
				case 6:
				{
					setClock(temp);	// set RTC_DS1307_I2C_MONTH	// clock Flag = 1
					break;
				}
				case 7:
				{
					setClock(temp);	// set RTC_DS1307_I2C_YEAR	// clock Flag = 1
					break;
				}
				default:
				{
				}
*/
/*				case 5:
				{
					setClock();	// set RTC_DS1307_I2C_CONTROL	// clock Flag = 1
					break;
				}
				case 5:
				{
					setClock();	// set FIRST_ADDRESS_RAM	// clock Flag = 1
					break;
				}
			}
*/

//			RTC_1307_GET();
//			LCD_EXECUTE_DATA("STORE EEPROM",12);
//			LCD_EXECUTE_COMMAND(LCD_SELECT_2ROW);	// select row 2
//			LCD_EXECUTE_DATA(" SET CLOCK ",11);

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
			_delay_ms(200);
		}
		else if(flagPower==1)
		{
			rotaryEncoderNikBarzakov();
		}
		else if(flagPower==0)
		{
			rotaryEncoderNikBarzakov();
		}
		else
		{}
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
//	ext0_intrpt_on();		// ENABLE interrupts to access IR DETECTION as call to function "IR_DECODER()" for -> SONY IR REMOTE
//	ext2_intrpt_on();

	sei();							// file "avr/interrupt.h"
//	SREG = (1<<I);

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
