/*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;*****************************************************************************************;;
;;************** Eng. Petar Upinov ********************************************************;;
;;***************** 07.10.2015 ************************************************************;;
;;*****************************************************************************************;;
;;******** ATmega32 DCAS with new Library *************************************************;;
;;***** (ATmega32 Digital Control Audio System) *******************************************;;
;;*****************************************************************************************;;
;;**************** Crystal 16MHz **********************************************************;;
;;****** Edit Fuse bits: High 0xCA ; Low 0xFF *********************************************;;
;;*****************************************************************************************;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;*/

/*************************************
** INCLUDE INTEGRATED LIBRARY FILES **
*************************************/

#include <avr/io.h>
#include <avr/interrupt.h>	// sei();
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
//#include <stdbool.h>		// type boolean true/false

/******************************************
** INCLUDE SPECIFIED PESHO LIBRARY FILES **
******************************************/

// Add C files to project: "ir_sirc.c", "rotation_encoder.c", "rtc.c", "spi.c", "utility.c"
#include "ATmega32A_GCC_DigitalControlAudioSystem_pLibs.h"
#include "_Soft_Library_Pesho_/24c64.h"					// not using
#include "_Soft_Library_Pesho_/ds18x20.h"				// not using
#include "_Soft_Library_Pesho_/i2c_twi.h"				// using
#include "_Soft_Library_Pesho_/ir_sirc.h"				// not using
#include "_Soft_Library_Pesho_/lcd_hd44780_74hc595.h"	// using
#include "_Soft_Library_Pesho_/pga2310.h"				// using
#include "_Soft_Library_Pesho_/relay_74hc595.h"			// using
#include "_Soft_Library_Pesho_/rotation_encoder.h"		// using
#include "_Soft_Library_Pesho_/rtc.h"					// not using
#include "_Soft_Library_Pesho_/spi.h"					// using
#include "_Soft_Library_Pesho_/uart.h"					// using
#include "_Soft_Library_Pesho_/utility.h"				// using

/********************************************************************************************
********************************* START OF INITIALIZATIONS **********************************
********************************************************************************************/

// STATUS FLAGS
struct flagStatus	 // ako bade obqveno ime na strukturata, shte mogat da badat sazdavani i drugi strukturni promenlivi (obekti) i ukazateli ot tazi struktura
{
	unsigned int flagPower	: 1;	// bit0: '0' = Power OFF, '1' = Power ON	// ne sa inicializirani
	unsigned int flagMute	: 1;	// bit1: '0' = Mute OFF, '1' = Mute ON		// ne sa inicializirani
} fSB, *flagStatusBits;		// sazdadeni fiksirani kam tazi struktura: strukturna promenliva (obekt) i ukazatel


// INFRA RED RECEIVER and VOLUME STEPS
#define REMOTE_VOLUME_UP 1
#define REMOTE_VOLUME_DOWN -1
#define VOLUME_LIMIT_POSITIONS 20
unsigned char volumeBuffer = 0;				// fro MUTE function
unsigned char volumeIndex = VOLUME_MUTE;	// defined in pga2310.h
unsigned char volumeValue [VOLUME_LIMIT_POSITIONS] = { 0x00, 0x28, 0x32, 0x3C, 0x46, 0x50, 0x5A, 0x64, 0x6E, 0x78, 0x82, 0x8C, 0x96, 0xA0, 0xAA, 0xB4, 0xBE, 0xC8, 0xD2, 0xD7 };
//       values of volume  ->	0,    40,   50,   60,   70,   80,   90,   100,  110,  120,  130,  140,  150,  160,  170,  180,  190,  200,  210,  215	<-	values of volume
// index of values of volume    0      1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19
typedef enum
{
  VOLUME_0 = 0x00,   //   0
  VOLUME_1 = 0x28,   //  40
  VOLUME_2 = 0x32,   //  50
  VOLUME_3 = 0x3C,   //  60
  VOLUME_4 = 0x46,   //  70
  VOLUME_5 = 0x50,   //  80
  VOLUME_6 = 0x5A,   //  90
  VOLUME_7 = 0x64,   // 100
  VOLUME_8 = 0x6E,   // 110
  VOLUME_9 = 0x78,   // 120
  VOLUME_10 = 0x82,  // 130
  VOLUME_11 = 0x8C,  // 140
  VOLUME_12 = 0x96,  // 150
  VOLUME_13 = 0xA0,  // 160
  VOLUME_14 = 0xAA,  // 170
  VOLUME_15 = 0xB4,  // 180
  VOLUME_16 = 0xBE,  // 190
  VOLUME_17 = 0xC8,  // 200
  VOLUME_18 = 0xD2,  // 210
  VOLUME_19 = 0xD7,  // 215
} VOLUME_LIMIT_POSITIONS_1;


// TEMPERATURE SENSOR
unsigned char storeTemp [10] = { 0 };	// data bytes massive
unsigned char leftTempSensorRomCode[8]	= { 0x10, 0xDB, 0x09, 0xA5, 0x01, 0x08, 0x00, 0xC1 };
unsigned char rightTempSensorRomCode[8]	= { 0x10, 0x6D, 0xF4, 0x8F, 0x02, 0x08, 0x00, 0xB1 };

unsigned int isr2;	// = 2 bytes range[0-65535] How overflows timer2 have done ?

typedef enum
{
	FAN_SPEED_ABSOLUTE_MIN	= 0,	// = 0
	FAN_SPEED_0				= 0,	// = 0
	FAN_SPEED_MIN 			= 1,	// = 1
	FAN_SPEED_1 			= 1,	// = 1
	FAN_SPEED_2				= 2,	// = 2
	FAN_SPEED_3				= 3,	// = 3
	FAN_SPEED_4				= 4,	// = 4
	FAN_SPEED_5				= 5,	// = 5
	FAN_SPEED_6				= 6,	// = 6
	FAN_SPEED_7				= 7,	// = 7
	FAN_SPEED_MAX			= 8,	// = 8
	FAN_LIMIT_POSITIONS		= 8,	// = 8
};

// FAN SPEED STEPS
//#define FAN_LIMIT_POSITIONS 8
//#define FAN_SPEED_ABSOLUTE_MIN 0
//#define FAN_SPEED_MIN 1		// min buffer position
//#define FAN_SPEED_MAX 7		// max buffer position
unsigned char fanSpeed = FAN_SPEED_MAX-FAN_SPEED_MIN;	// FAN SPEED OCR1AL = 150
unsigned char fanSpeedStep [FAN_LIMIT_POSITIONS] = { 0x00, 150, 160, 175, 190, 200, 225, 250 };
unsigned char byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9; // bytes ot temperaturen sensor

/*****************************************
** INITIZLIZATION OF INPUT/OUTPUT PORTS **
*****************************************/
void port_init(void)
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
void ext0_intrpt_init(void)
{
	MCUCR = 0b00000010;	// SETUP EXT INT 0, ISC01 = 1, ISC00 = 0: Falling edge on INT0 activates the interrupt; ISC01 = 1, ISC00 = 1: Rising edge on INT0 activates the interrupt;

// IN FUNCTIONS:
//	GICR = 0b01000000;	// INT0 = 0: Disable External Interrupt on INT0; INT0 = 1: Enable External Interrupt on INT0;
//	GIFR = 0b01000000;	// Clear INT0 flag.
}

/********************************
** EXTERNAL INTERRUPT 0 ENABLE **
********************************/
void ext0_intrpt_on(void)		// Enable external interrupt 0 (PD0 - ENABLE new IR DETECTION)
{
	GICR |= 0b01000000;	// INT0 = 0: Disable External Interrupt on INT0; INT0 = 1: Enable External Interrupt on INT0;
	GIFR |= 0b01000000;	// Clear INT0 flag.
}

/*********************************
** EXTERNAL INTERRUPT 0 DISABLE **
*********************************/
void ext0_intrpt_off(void)		// Disable external interrupt 0 (PD0 - DISABLE new IR DETECTION)
{
	GICR |= 0b00000000;	// INT0 = 0: Disable External Interrupt on INT0; INT0 = 1: Enable External Interrupt on INT0;
	GIFR |= 0b01000000;	// Clear INT0 flag.
}

/*******************************************
** INITIZLIZATION OF EXTERNAL INTERRUPT 1 **
*******************************************/
void ext1_intrpt_init(void)
{
}

/*******************************************
** INITIZLIZATION OF EXTERNAL INTERRUPT 2 **
*******************************************/
void ext2_intrpt_init(void)
{
//	MCUCSR = 0b00000000;	// SETUP EXT INT 2, ISC2 = 0: Falling edge on INT2 activates the interrupt; ISC2 = 1: Rising edge on INT2 activates the interrupt;

// IN FUNCTIONS:
//	GICR   = 0b00100000;	// INT2 = 0: Disable External Interrupt on INT2; INT2 = 1: Enable External Interrupt on INT2;
//	GIFR   = 0b00100000;	// Clear INT2 flag.
}

/*****************************
** INITIZLIZATION OF TIMER0 **
*****************************/
/*void timer0_init()
{

    //8-bit timer for measuring delay between IR pulses
//	TCCR0 = 0b00000011; //0b00000010 = CLK /8 ; //0b00000011 = CLK / 64; //0b00000101 = CLK / 1024
//	TCNT0 = 0; //reset the timer


	// http://extremeelectronics.co.in/avr-tutorials/avr-timers-an-introduction/
	// http://www.electroons.com/electroons/timer_delay.html
}*/

/*****************************
** INITIZLIZATION OF TIMER1 **
*****************************/
void timer1_init(void)
{
// http://www.mikroe.com/forum/viewtopic.php?f=72&t=51076

	TIMSK  |= 0b00000000;	// maskov registar za prekasvaniq, ne e nujno da ima prekasvane po timer1, izpolzvame go samo za rabota po izvod
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

/*****************************
** INITIZLIZATION OF TIMER2 **
*****************************/
void timer2_init(void)
{
#ifdef DEBUG_INFO
	transmitUartString("[UART INFO] Fan is on\r\n");
	transmitUartString("[UART INFO] Fan rotation with max speed\r\n");
#endif
//	FAN_high();			// PORTD5 - FAN ON (logic "1")	NON PWM, NON TIMER1
fanSpeed = FAN_SPEED_MAX-FAN_SPEED_MIN;	// amplifer run with max fan speed
fan_pwm_control_speed();	// KOMENTAR ZARADI SIMULACIQTA - MNOGO BAVI PRI SIMULACIQ S TIMER1
_delay_ms(3000);			// wait 3 seconds for maximum rotating fans

#ifdef DEBUG_ERROR
	transmitUartString("[UART INFO] Speed of fans are controlled by auto program\r\n");
#endif
//==================================================================================================================
	TCNT2 &= 0b00000000;	// Clear Counter Timer2
//   TCCR2 = FOC | WGM20 | COM21 | COM20 | WGM21 | CS22 | CS21 | CS20 |
	TCCR2 = 0b00001010;	// 0b00001xxx, xxx = 001 /1, 010 /8, 011 /32, 100 /64, 101 /128, 110 /256, 111 /1024			//   TCCR2 |= (1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20); // configure timer2 for CTC mode(WGM21), 1024 prescaller (CS20,21,22)
	TIMSK |= (1 << OCIE2); // enable the CTC interrupt	//	TIMSK |= 0b01000000;	// TIMSK |= (1 << TOIE2); // Disable Internal Interrupts on Timer2

	OCR2 = 255;	//255;
//	OCR2 = 255;
//	OCR2 = 15625;			// (16 000 000Hz / 1024 prescaller) = 15625 Hz; 8biregister = 255+0=256; 255/15625 = 0,01632 sec; 0,01632*62 = 1.01184sec
							// No ne moje da dostigne 15625, zashtoto 8-bit register OCR2 moje da dostigne ot 0 do 255, za tova moje bi 255/255=1 sec??
//==================================================================================================================

//	SFIOR |= 0b00000010;	// Prescaler Reset Timer2 (bit1 �> PSR2)
//	TCCR2 = 0b01100001;		// 0b01100001 - WGM20,COM21,CS20 - PWM, Phase correct, No prescaller divide or division by 1
//	OCR2 = 100;				// Compare match by Overflow Timer with ~15 times [us]


//	TCCR2 |= (1 << CS22)|(1 << CS21);  	// set up timer with prescaler = 256
//	TCNT2 = 0;							// initialize counter
//	TIMSK |= (1 << TOIE2);				// enable overflow interrupt
//	sei();								// enable global interrupts
	// initialize overflow counter variable
	isr2 = 0;
}
void timer2Internal_intrpt_off(void)
{
//	TIMSK |= 0b00000000;		// OCIE2 [bit7] = 0: Disable Internal Interrupt on Timer2CompareMatch; OCIE2 [bit7] = 1: Enable Internal Interrupt on Timer2CompareMatch;
//	TIFR  |= 0b10000000;		// OCF2  [bit7] = 1: Clear Timer2CompareMatch flag.
}

void timer2Internal_intrpt_on(void)
{
//	TIMSK |= 0b10000000;		// OCIE2 [bit7] = 0: Disable Internal Interrupt on Timer2CompareMatch; OCIE2 [bit7] = 1: Enable Internal Interrupt on Timer2CompareMatch;
//	TIFR  |= 0b10000000;		// OCF2  [bit7] = 1: Clear Timer2CompareMatch flag.
}

/********************************************************************************************
********************************** END OF INITIALIZATIONS ***********************************
********************************************************************************************/

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/***************************************
******** DEFINITIONS OF TIMER 0 ********
***************************************/
/*
void timer0_on()
{
}
void timer0_off()
{
}
*/
/***************************************
******** DEFINITIONS OF TIMER 1 ********
***************************************/
void timer1_on_speed(void)
{
//		ldi	r16, 0			; maskov registar za prekasvaniq
//		out	TIMSK, r16		; ne e nujno da ima prekasvane
	TCCR1A = 0b10100001;	// 0b10000001;	// nastroika na 2 kanala rejim na rabota na SHIM		// 0b10100001 - OC1A,OC1B - PWM;  0b10000001 - OC1A PWM, OC1B - Disabled, normal port.
	TCCR1B = 0b00000001;	// 0b00010001;	// nastroika na 2 kanala rejim na rabota na SHIM i preddelitel 8

// CHANNEL A
	OCR1AH = 0;		// 0; // FAN PWM ON		// out	OCR1AH, r16		; 0   = 0b00000000 (DEC = BIN)
	OCR1AL = fanSpeedStep [fanSpeed];//100;	// 1; // FAN PWM ON		// out	OCR1AL, r17		; 200 = 0b11001000 (DEC = BIN)
#ifdef DEBUG_INFO
	transmitUartString("[UART INFO] Set calculated fanSpeed value: ");		// uart debug information string
	transmitUartInt(fanSpeed);		// uart debug information string
	transmitUartString("\r\n");		// uart debug information string
#endif
// CHANNEL B
//	OCR1BH = 0; // LED PWM ON				// out	OCR1BH, r16		; 0   = 0b00000000 (DEC = BIN)
//	OCR1BL = 1; // LED PWM ON				// out	OCR1BL, r17		; 200 = 0b11001000 (DEC = BIN)
}

void timer1_off(void)
{
	TCCR1A = 0b00000000;	// DISABLED OCOC1A - PWM, OC1B - Disabled, normal port.
	TCCR1B = 0b00000000;	//
	SFIOR |= 0b00000001;	// Prescaler Reset Timer1/0 (bit0  PSR10)



	OCR1AH = 0; // FAN PWM OFF
	OCR1AL = 0; // FAN PWM OFF

//	OCR1BH = 0; // LED PWM OFF
//	OCR1BL = 0; // LED PWM OFF
}

/***************************************
******** DEFINITIONS OF TIMER 2 ********
***************************************/
void timer2_on(void)
{
//   TCCR2 = 0b00001010;	// 0b00001xxx, xxx = 001 /1, 010 /8, 011 /32, 100 /64, 101 /128, 110 /256, 111 /1024			//   TCCR2 |= (1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20); // configure timer2 for CTC mode(WGM21), 1024 prescaller (CS20,21,22)
//   OCR2 = 255;			//255;
}

void timer2_off(void)
{
//	TCCR2 = 0b00000000;	// TCCR2 |= (1 << CS20); // (1 << CS20) 0b01100001 - WGM20,COM21,CS20 - PWM, Phase correct, No prescaller divide or division by 1
//	OCR2 = 0;
//	SFIOR |= 0b00000010;	// Prescaler Reset Timer2 (bit1 > PSR2)
}

/***********************************
******** DEFINITIONS OF FAN ********
***********************************/
void fan_pwm_control_speed(void)
{
	timer1_on_speed();
}
void fan_pwm_off(void)
{
	timer1_off();
}

/************************************
** DEFINITION IR DECODER FUNCTIONS **
************************************/
void irDecode(void)
{
//	byte byteSS0, byteSS1, byteMM0, byteMM1, byteHH0, byteHH1, byteDD0, byteDD1, byteMont0, byteMont1, byteYY0, byteYY1; // variables for convert DEC to BCD for LCD and UART for Time and Date

	GetSIRC12();
	if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A && irCommand == IR_REMOTE_COMMAND_RM_677A_STANDBY) || (irAddress == IR_REMOTE_CAR_DEVICE_RM_X157 && irCommand == IR_REMOTE_COMMAND_RM_X157_OFF)) && flagStatusBits->flagPower == 0)		// IR POWER -> ON
	{		
		ampliferOn();
//		_delay_ms(1000);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A && irCommand == IR_REMOTE_COMMAND_RM_677A_STANDBY) || (irAddress == IR_REMOTE_CAR_DEVICE_RM_X157 && irCommand == IR_REMOTE_COMMAND_RM_X157_OFF)) && flagStatusBits->flagPower == 1)	// IR POWER -> OFF
	{
		ampliferOff();
//		flagPower = 0;			// filter za buton OFF
//		break;
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_VOLUP)) && flagStatusBits->flagPower == 1)	// Sony TV & CarAudio IR Remote Device - "VOLUME UP"
	{	// VOLUME UP
		volumeProcessRemote(REMOTE_VOLUME_UP);
//		break;
	}
	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_VOLDN)) && flagStatusBits->flagPower == 1)	// Sony TV & CarAudio IR Remote Device - "VOLUME DOWN"
	{	// VOLUME DOWN
		volumeProcessRemote(REMOTE_VOLUME_DOWN);
//		break;
	}

	else if(((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ATT)) && flagStatusBits->flagPower == 1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
	{	// MUTE
		volumeMute();
//		break;
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_DOWN)) && flagStatusBits->flagPower == 1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
	{	// FAN STEP UP
		if(fanSpeed < FAN_SPEED_ABSOLUTE_MIN + 1)//0)
		{
			fanSpeed = FAN_SPEED_ABSOLUTE_MIN;	//0;	// 0-7
		}
		else
		{
			fanSpeed--;	// sabirane s polojitelno chislo, kratak zapis na: volumeIndex = volumeIndex + temp;
		}
//		fanSpeedStep [fanSpeed];// = { 0x00, 100, 125, 150, 175, 200, 225, 250 };
		LCD_COMMAND(LCD_SELECT_4ROW);	// select row 3								// and next is update volume lcd information
		LCD_DATA_STRING("Fan Step: ");	// 20 symbols			
		LCD_DATA_INT(fanSpeed);		// 20 symbols
		LCD_DATA_STRING(" or ");
		LCD_DATA_INT(fanSpeedStep[fanSpeed]);
		fan_pwm_control_speed();
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_UP)) && flagStatusBits->flagPower == 1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
	{	// FAN STEP DOWN
		if(fanSpeed > FAN_LIMIT_POSITIONS - 2)//7)
		{
			fanSpeed = FAN_LIMIT_POSITIONS - 1;	//7;	// 0-7
		}
		else
		{
			fanSpeed++;	// sabirane s polojitelno chislo, kratak zapis na: volumeIndex = volumeIndex + temp;
		}
//		fanSpeedStep [fanSpeed];// = { 0x00, 100, 125, 150, 175, 200, 225, 250 };
		LCD_COMMAND(LCD_SELECT_4ROW);	// select row 3								// and next is update volume lcd information
		LCD_DATA_STRING("Fan Step: ");	// 20 symbols			
		LCD_DATA_INT(fanSpeed);		// 20 symbols
		LCD_DATA_STRING(" or ");
		LCD_DATA_INT(fanSpeedStep[fanSpeed]);
		fan_pwm_control_speed();
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_SCRL)) && (flagStatusBits->flagPower == 0 || flagStatusBits->flagPower == 1))						// Sony CarAudio IR Remote Device - "SCRL" -> TEMPERATURE
	{
		temperature();
		_delay_ms(200);
	}
/*	else if((((irAddress == IR_REMOTE_TV_DEVICE_RM_677A || irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ATT)) && mute==1) && flagPower==1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> OFF
	{	// UNMUTE
		muteOff();
		mute = 0;
//		break;
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_SOURCE)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "SOURCE"
	{
		setClock();
		_delay_ms(200);	
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_MENU)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "MENU"
	{
		showClock();
		_delay_ms(200);	
	}
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_DSPL)) && (flagPower==0 || flagPower==1))						// Sony CarAudio IR Remote Device - "DSPL"
	{
		LCD_INIT();
		LED_high_DISPLAYLED_low();
		_delay_ms(200);	
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
			// HOURS INCREMENT
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
		//else if(flagAny)
		//{
		//}
		//else
		//{}
	}
	else if((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_DOWN))
	{
		if(flagRTC == 1)
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
		//else if(flagAny)
		//{
		//}
		//else
		//{}
	}
	else if((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ENTER))
	{	
		if(flagRTC == 1)
		{
			// store variables to to rtc ds1307
			i2c_start();
			i2c_write(RTC_DS1307_I2C_ADDRESS_WRITE);	// RTC DS1307 ADDRESS ACCESS WRITE
			i2c_write(RTC_DS1307_I2C_HOURS);			// HOURS ADDRESS REGISTER ACCESS
			i2c_write(hours);							// HOURS DATA VALUE
			i2c_stop();
*//*
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
*//*
			flagRTC = 0;
		}
	}
*/
/*
	else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_MODE)) && flagStatusBits->flagPower == 1)						// Sony CarAudio IR Remote Device - "MODE"
	{
		#ifdef DEBUG_INFO
			transmitUartString("[SETUP MODE]");		// uart debug information string
			transmitUartString("\r\n");			// uart debug information string
		#endif
		LCD_COMMAND(LCD_SELECT_2ROW);	// select row 3								// and next is update volume lcd information
		LCD_DATA_STRING("[SETUP MODE]");	// 20 symbols
		LCD_COMMAND(LCD_SELECT_3ROW);	// select row 3								// and next is update volume lcd information
		LCD_DATA_STRING("Press UP/DOWN");	// 20 symbols
		//	LCD_DATA_INT(fanSpeed);		// 20 symbols
		while(exitSetupMode)
		GetSIRC12();
		LCD_COMMAND(LCD_SELECT_4ROW);	// select row 3								// and next is update volume lcd information
		if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_DOWN)) && flagStatusBits->flagPower == 1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
		{// MENU UP
			indexMenuTable++;
			if(indexMenuTable>2)
			{
				indexMenuTable=0;
			}
			LCD_DATA_STRING(chooseMenu[indexMenuTable]);	// 20 symbols
			#ifdef DEBUG_INFO
				transmitUartString("[SETUP MODE]: Pressed UP");		// uart debug information string
				transmitUartString("\r\n");			// uart debug information string
				transmitUartString(chooseMenu[indexMenuTable]);		// uart debug information string
				transmitUartString("\r\n");			// uart debug information string
			#endif

		}
		else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_UP)) && flagStatusBits->flagPower == 1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
		{//MENU DOWN
			indexMenuTable--;
			if(indexMenuTable<0)
			{
				indexMenuTable=2;
			}
			LCD_DATA_STRING(chooseMenu[indexMenuTable]);	// 20 symbols
			#ifdef DEBUG_INFO
				transmitUartString("[SETUP MODE]: Pressed DOWN");		// uart debug information string
				transmitUartString("\r\n");			// uart debug information string
				transmitUartString(chooseMenu[indexMenuTable]);		// uart debug information string
				transmitUartString("\r\n");			// uart debug information string
			#endif
		}
		else if(((irAddress == IR_REMOTE_CAR_DEVICE_RM_X157) && (irCommand == IR_REMOTE_COMMAND_RM_X157_ENTER)) && flagStatusBits->flagPower == 1)		// Sony TV & CarAudio IR Remote Device - "MUTE" -> ON
		{//MENU Enter
	//		setupMode(chooseMenu[menuTable]);	// 20 symbols
			exitSetupMode = 0;
			#ifdef DEBUG_INFO
				transmitUartString("[SETUP MODE]: Pressed MENU to EXIT");		// uart debug information string
				transmitUartString("\r\n");			// uart debug information string
			#endif
		}
		_delay_ms(200);
//		break;
	}
*/
	else
	{
		// DO NOTING
	}
	_delay_ms(200);
}

/********************
**** AMPLIFER ON ****
********************/
void ampliferOn(void)
{
	flagStatusBits->flagPower = 1;		// flag for amplifer on

// UART MESSAGE
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Amplifer is on\r\n");
	#endif
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Display on and status led off\r\n");
	#endif

// LED OFF FUNC
	LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1

// LCD FUNC & MESSAGE
	LCD_CLEAR_CONTAIN();						// clear all contain on display
	LCD_COMMAND(LCD_SELECT_1ROW);				// select row 1
	LCD_DATA_STRING("    Amplifer On     ");	// 20 symbols
	LCD_COMMAND(LCD_SELECT_2ROW);				// select row 2
	LCD_DATA_STRING("P.UPINOV  P.STOYANOV");	// 20 symbols //	LCD_EXECUTE_DATA("P.UPINOV  P.STOYANOV",20);	// char "DATA", int 13 of chars of "DATA"
	LCD_COMMAND(LCD_ON);						// LCD ON without CURSOR

// FANS FUNC & MESSAGE -> MOVED IN timer2
//	#ifdef DEBUG_INFO
//		transmitUartString("[UART INFO] Fan is on\r\n");
//		transmitUartString("[UART INFO] Fan rotation with max speed\r\n");
//		transmitUartString("[UART INFO] Fan is always on, it isn't sensitive to temperature, because DS18S20 is disabling\r\n");
//	#endif
////	FAN_high();			// PORTD5 - FAN ON (logic "1")	NON PWM, NON TIMER1
//	fanSpeed = FAN_SPEED_MAX;	// amplifer run with max fan speed
//	fan_pwm_control_speed();	// KOMENTAR ZARADI SIMULACIQTA - MNOGO BAVI PRI SIMULACIQ S TIMER1
//	timer2_on();	// enable auto regular fan by temp sensor

// RELAYS ON FUNC & MESSAGE
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Try to switch on relays for power 220V\r\n");		// uart debug information string
	#endif
	REL_POWER_high();// RELAY POWER ON TRAFs		// PESHO COMMENT 14.08.2015, 21:10
	_delay_ms(4000);								// PESHO COMMENT 14.08.2015, 21:10
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Try to switch on relays in for all 6 channels\r\n");		// uart debug information string
	#endif
	relays_in1_6ch();	// RELAYS IN1 CHANNELS 6	// PESHO COMMENT 14.08.2015, 21:10
	_delay_ms(700);									// PESHO COMMENT 14.08.2015, 21:10
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Try to switch on relays out for all 6 channels\r\n");		// uart debug information string
	#endif
	relays_out_6ch();	// RELAYS OUT CHANNELS 6	// PESHO COMMENT 14.08.2015, 21:10
	_delay_ms(1000);	// izchakvane pri natiskane za vkliuchvane i otpuskane na buton - filtar treptqsht kontakt buton

// FANS FUNC & MESSAGE
//	#ifdef DEBUG_INFO
//		transmitUartString("[UART INFO] Fan rotation with min speed\r\n");
//		transmitUartString("[UART INFO] Fan manual controlling with remote menu up to speed step up and menu down button to speed step down\r\n");
//	#endif
//	fanSpeed = FAN_SPEED_MIN;	// amplifer works with min fan speed
//	fan_pwm_control_speed();
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] Amplifer is auto controlling fans!\r\n");
	#endif
//	timer2_init();	// Fan controlling, when the fan will be setup, this init to be moved in init_all();

	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] Amplifer was switched on!\r\n");
	#endif
}

/*********************
**** AMPLIFER OFF ****
*********************/
void ampliferOff(void)
{
	flagStatusBits->flagPower = 0;		// flag for amplifer off

	LCD_COMMAND(LCD_SELECT_1ROW);				// select row 1
	LCD_DATA_STRING("    Amplifer Off    ");	// 20 symbols

	LCD_COMMAND(LCD_OFF);						// LCD ON without CURSOR

//			FAN_low();		// PORTD5 - FAN OFF (logic "0")  NON PWM, NON TIMER1

// RELAYS OFF FUNC & MESSAGE
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Try to switch off relays out for all 6 channels\r\n");		// uart debug information string
	#endif
	relays_out_off();	// RELAYS OUT CHANNELS 6	// PESHO COMMENT 14.08.2015, 21:10
	_delay_ms(700);								// PESHO COMMENT 14.08.2015, 21:10
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Try to switch off relays in for all 6 channels\r\n");		// uart debug information string
	#endif
	relays_in_off();	// RELAYS IN1 CHANNELS 6	// PESHO COMMENT 14.08.2015, 21:10
	_delay_ms(700);								// PESHO COMMENT 14.08.2015, 21:10
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Try to switch off relays for power 220V\r\n");		// uart debug information string
	#endif
	REL_POWER_low();// RELAY POWER OFF				// PESHO COMMENT 14.08.2015, 21:10

// FANS FUNC & MESSAGE
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] Fan is off\r\n");
	#endif
	fan_pwm_off();	// KOMENTAR ZARADI SIMULACIQTA - MNOGO BAVI PRI SIMULACIQ S TIMER1

// FANS FUNC & MESSAGE
	LCD_CLEAR_CONTAIN();
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Display off and status led on\r\n");
	#endif

// LED ON FUNC
	LED_high_DISPLAYLED_low();		// PORTD4 - LED ON (logic "1"), DISPLAY BACKLIGHT OFF (logic "1"),  NON PWM, NON TIMER1

// UART MESSAGE
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Amplifer is off\r\n");
	#endif

	_delay_ms(500);	// izchakvane pri natiskane za izkliuchvane i otpuskane na buton - filtar treptqsht kontakt buton
}

/*********************************************
**** VOLUME PROCESS FUNCTION FROM ENCODER ****
*********************************************/
void volumeProcess(void)
{
	signed char temp = 0;//, tempEnc = 0, tempRem = 0;				// zadaljitelno signed char!!! ima osobenost pri vrashtaneto na rezultat ot funkciq!!!
	temp = rotaryEncoderNikBarzakov();
	if(0==temp)
	{
		// do nothing, encoder havn't been rotated  // ne e bil zavartan
	}
	else if(-1==temp)
	{
		// encoder is decrement
		if(volumeIndex < (VOLUME_MUTE + 1))
		{
			volumeIndex = VOLUME_MUTE;
		}
		else
		{
			volumeIndex += temp;	// sabirane s polojitelno chislo, kratak zapis na: volumeIndex = volumeIndex + temp;
		}
		volumeUpdate();
	}
	else if(1==temp)
	{
		// encoder is increment
		if (volumeIndex > (VOLUME_LIMIT_POSITIONS - 2))
		{
			volumeIndex = (VOLUME_LIMIT_POSITIONS - 1);
		}
		else
		{
			volumeIndex += temp;	// sabirane s polojitelno chislo, kratak zapis na: volumeIndex = volumeIndex + temp;
		}
		volumeUpdate();
	}
}

/********************************************
**** VOLUME PROCESS FUNCTION FROM REMOTE ****
********************************************/
void volumeProcessRemote(signed char temp)
{
	if(-1==temp)
	{
		// encoder is decrement
		if(volumeIndex < (VOLUME_MUTE + 1))
		{
			volumeIndex = VOLUME_MUTE;
		}
		else
		{
			volumeIndex += temp;	// sabirane s polojitelno chislo, kratak zapis na: volumeIndex = volumeIndex + temp;
		}
		volumeUpdate();
	}
	else if(1==temp)
	{
		// encoder is increment
		if (volumeIndex > (VOLUME_LIMIT_POSITIONS - 2))
		{
			volumeIndex = (VOLUME_LIMIT_POSITIONS - 1);
		}
		else
		{
			volumeIndex += temp;	// sabirane s polojitelno chislo, kratak zapis na: volumeIndex = volumeIndex + temp;
		}
		volumeUpdate();
	}
}

/************************************
**** VOLUME MUTE/UNMUTE FUNCTION ****
************************************/
void volumeMute(void)
{
	if(flagStatusBits->flagMute == 0)
	{
		volumeBuffer = volumeIndex;		// strore volume volue
		volumeIndex = VOLUME_MUTE;		// MUTE ON
		flagStatusBits->flagMute = 1;	// MUTE ON
		#ifdef DEBUG_INFO
			transmitUartString("[UART INFO] Volume mute is on\r\n");
		#endif
	}
	else
	{
		volumeIndex = volumeBuffer;		// MUTE OFF
		volumeBuffer = VOLUME_MUTE;		// clear volume volue
		flagStatusBits->flagMute = 0;	// MUTE OFF
		#ifdef DEBUG_INFO
			transmitUartString("[UART INFO] Volume mute is off\r\n");
		#endif
	}
	volumeUpdate();
}

/*************************************
**** VOLUME UPDATE and LCD UPDATE ****
*************************************/
void volumeUpdate(void)
{
	PGA2310_Volume_Update(volumeValue[volumeIndex], volumeValue[volumeIndex]);	// update volume value on all channels
	LCD_COMMAND(LCD_SELECT_4ROW);	// select row 3								// and next is update volume lcd information
//		LCD_DATA_STRING("Volume: ");	// 20 symbols
	if (volumeIndex > 9)
	{
		LCD_DATA_STRING("Volume: ");	// 20 symbols
	}
	else
	{
		LCD_DATA_STRING("Volume: 0");	// 20 symbols
	}
	LCD_DATA_INT(volumeIndex);			// 20 symbols
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] Volume: ");		// uart debug information string
		transmitUartInt(volumeIndex);		// uart debug information string 
		transmitUartString("\r\n");			// uart debug information string
	#endif
}

/**********************************************
**** ROTARY ENCODER for ALL OTHER FUNCTION ****
**********************************************/
void commonEncoder(void)	// not finished
{
	static signed char saveValue = 0;	// zadaljitelno signed char!!! ima osobenost pri vrashtaneto na rezultat ot funkciq!!! static ???
	signed char temp = 0;				// zadaljitelno signed char!!! ima osobenost pri vrashtaneto na rezultat ot funkciq!!!
	temp = rotaryEncoderNikBarzakov();
	if(0==temp)
	{
		// do nothing, encoder havn't been rotated  // ne e bil zavartan
	}
	else if(-1==temp)
	{
		// encoder is decrement
		if(saveValue < -127)
		{
			saveValue = 127;	// SIGNED CHAR MIN VALUE = -127
		}
		else
		{
			saveValue += temp;	// sabirane s polojitelno chislo, kratak zapis na: volumeIndex = volumeIndex + temp;
//			fanSpeedStep [FAN_LIMIT_POSITIONS]
		}
// LCD PRINT VALUE
	LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1
	LCD_COMMAND(LCD_ON);						// LCD ON without CURSOR
		LCD_COMMAND(LCD_SELECT_4ROW);	// select row 3								// and next is update volume lcd information
		if (saveValue > 99)
		{
			LCD_DATA_STRING("Volume: ");	// 20 symbols
		}
		else if (saveValue > 9)
		{
			LCD_DATA_STRING("Volume: 0");	// 20 symbols
		}
		else
		{
			LCD_DATA_STRING("Volume: 00");	// 20 symbols			
		}
		LCD_DATA_INT(saveValue);		// 20 symbols
	}
	else if(1==temp)
	{
		// encoder is increment
		if (saveValue > 128)
		{
			saveValue = 128;	// SIGNED CHAR MAX VALUE = +128
		}
		else
		{
			saveValue += temp;	// sabirane s polojitelno chislo, kratak zapis na: volumeIndex = volumeIndex + temp;
		}

	LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1
	LCD_COMMAND(LCD_ON);						// LCD ON without CURSOR
		LCD_COMMAND(LCD_SELECT_4ROW);	// select row 3								// and next is update volume lcd information
		if (saveValue > 99)
		{
			LCD_DATA_STRING("Volume: ");	// 20 symbols
		}
		else if (saveValue > 9)
		{
			LCD_DATA_STRING("Volume: 0");	// 20 symbols
		}
		else
		{
			LCD_DATA_STRING("Volume: 00");	// 20 symbols			
		}
		LCD_DATA_INT(saveValue);		// 20 symbols
	}
}

/*****************************
**** TEMPERATURE FUNCTION ****
*****************************/
void temperature()
{
	unsigned char i;
//	LED_low_DISPLAYLED_high();
//	LCD_INIT();								// LCD INITIZLIZATION
	LCD_CLEAR_CONTAIN();

	LCD_COMMAND(LCD_SELECT_1ROW);	// select row 1
	LCD_DATA_STRING("     TEMERATURE     ");		//
	LCD_COMMAND(LCD_SELECT_2ROW);	// select row 2
	LCD_DATA_STRING("LEFT  SENSOR: ");				//

	oneWireLeft();
	for(i=0; i<9; i++)
	{
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] byte ");
		transmitUartInt(i);
		transmitUartString(" : ");
		transmitUartInt(storeTemp[i]);
		transmitUartString("\r\n");
	#endif
	}
	temperMeasur(byte0, byte1, byte6, byte7);
//lcdDataString("?? C"); // ot gornata funkciq

	LCD_COMMAND(LCD_SELECT_3ROW);	// select row 3
	LCD_DATA_STRING("RIGHT SENSOR: ");			//
	oneWireRight();
	for(i=0; i<9; i++)
	{
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] byte ");
		transmitUartInt(i);
		transmitUartString(" : ");
		transmitUartInt(storeTemp[i]);
		transmitUartString("\r\n");
	#endif
	}
	temperMeasur(byte0, byte1, byte6, byte7);
//lcdDataString("?? C"); // ot gornata funkciq

	LCD_COMMAND(LCD_SELECT_4ROW);	// select row 4
	LCD_DATA_STRING("             DS18x20");		//
}

void autoControlTemperature()
{
	unsigned char i;
	char resultL, resultR, resultCompare;

	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] Left Temperature Sensor\r\n");
	#endif
	oneWireLeft();
	resultL = temperMeasur(byte0, byte1, byte6, byte7);
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] RESULT LEFT Sensor = ");
		transmitUartInt(resultL);
		transmitUartString("\r\n");
	#endif

	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] Right Temperature Sensor\r\n");
	#endif
	oneWireRight();
	resultR = temperMeasur(byte0, byte1, byte6, byte7);
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] RESULT RIGHT Sensor = ");
		transmitUartInt(resultR);
		transmitUartString("\r\n");
	#endif

	resultCompare = resultL > resultR ? resultL : resultR;
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] Bigger temperature value between left or right sensors is: resultCompare = ");
		transmitUartInt(resultCompare);
		transmitUartString("\r\n");
	#endif

//	{ 0x00, 150, 160, 175, 190, 200, 225, 250 };
	if((resultCompare <= 1) || (resultCompare <= 20))
	{
		fanSpeed = fanSpeedStep[FAN_SPEED_0]; // fan value = 0
	}
	else if((resultCompare <= 21) || (resultCompare <= 26))
	{
		fanSpeed = fanSpeedStep[FAN_SPEED_1]; // fan value = 150
	}
	else if((resultCompare <= 27) || (resultCompare <= 29)) //35
	{
		fanSpeed = fanSpeedStep[FAN_SPEED_2]; // fan value = 160
	}
	else if((resultCompare <= 30) || (resultCompare <= 32)) //40
	{
		fanSpeed = fanSpeedStep[FAN_SPEED_3]; // fan value = 175
	}
	else if((resultCompare <= 33) || (resultCompare <= 35))//50
	{
		fanSpeed = fanSpeedStep[FAN_SPEED_4]; // fan value = 190
	}
	else if((resultCompare <= 36) || (resultCompare <= 37))//55
	{
		fanSpeed = fanSpeedStep[FAN_SPEED_5]; // fan value = 200
	}
	else if((resultCompare <= 38) || (resultCompare <= 40))//60
	{
		fanSpeed = fanSpeedStep[FAN_SPEED_6]; // fan value = 225
	}
	else if(resultCompare > 40)//65
	{
		fanSpeed = fanSpeedStep[FAN_SPEED_7]; // fan value = 250
	}
	else
	{
		#ifdef DEBUG_ERROR
			transmitUartString("Error: Temperature isn't correct value to controlling fan!");
			transmitUartInt(fanSpeed);
			transmitUartString("\r\n");
		#endif
	}

	#ifdef DEBUG_ERROR
		transmitUartString("Auto controlled fan value step. Set value: ");
		transmitUartString(" fanSpeedStep[");
		transmitUartInt(fanSpeed);
		transmitUartString("] \r\n");
	#endif
	fan_pwm_control_speed();	// update fan speed
}

/*******************************************
**** 1-WIRE DS18x20 Temperature Sensors ****
*******************************************/
unsigned char oneWireLeft()
{
	unsigned char i;

	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] TEMPERATURE SENSOR LEFT ROMCODE: 10 DB 09 A5 01 08 00 C1 \r\n");		// uart debug information string
		transmitUartString("[UART INFO] TEMP REGISTERS DATA RAW DUMP\r\n");		// uart debug information string
	#endif	
	if(reset())				// Master issues reset pulse. DS18S20s respond with presence pulse.
	{
		write_byte(0x55);	// Master issues Match ROM command.
		for(i=0; i<8; i++)
		{
			write_byte(leftTempSensorRomCode[i]);	// 64-bit ROM CODE
		}
/*		write_byte(0x10);	// Byte 0
		write_byte(0xDB);	// Byte 1
		write_byte(0x09);	// Byte 2
		write_byte(0xA5);	// Byte 3
		write_byte(0x01);	// Byte 4
		write_byte(0x08);	// Byte 5
		write_byte(0x00);	// Byte 6
		write_byte(0xC1);	// Byte 7
		// 64-bit ROM CODE
*/
		write_byte(0x44);	// Master issues Convert T command.
		wait_ready();		// Master applies strong pullup to DQ for the duration of the conversion (tCONV).
		if(reset())			// Master issues reset pulse. DS18S20s respond with presence pulse.
		{
			write_byte(0x55);	// Master issues Match ROM command.
			for(i=0; i<8; i++)
			{
				write_byte(leftTempSensorRomCode[i]);	// 64-bit ROM CODE
			}
/*			// 64-bit ROM CODE
			write_byte(0x10);	// Byte 0
			write_byte(0xDB);	// Byte 1
			write_byte(0x09);	// Byte 2
			write_byte(0xA5);	// Byte 3
			write_byte(0x01);	// Byte 4
			write_byte(0x08);	// Byte 5
			write_byte(0x00);	// Byte 6
			write_byte(0xC1);	// Byte 7
			// 64-bit ROM CODE
*/
			write_byte(0xBE);	// Master issues Read Scratchpad command.
			for(i=0; i<9; i++)
			{
				storeTemp [i] = read_byte();	//	Master reads entire scratchpad including CRC. The master then recalculates the CRC of the first eight data bytes from the scratchpad and compares the calculated CRC with the read CRC (byte 9). If they match, the master continues; if not, the read operation is repeated.
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
	unsigned char i;

	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] TEMPERATURE SENSOR RIGHT ROMCODE: 10 6D F4 8F 02 08 00 B1 \r\n");		// uart debug information string
		transmitUartString("[UART INFO] TEMP REGISTERS DATA RAW DUMP\r\n");		// uart debug information string
	#endif	
	if(reset())				// Master issues reset pulse. DS18S20s respond with presence pulse.
	{
		write_byte(0x55);	// Master issues Match ROM command.
		for(i=0; i<8; i++)
		{
			write_byte(rightTempSensorRomCode[i]);	// 64-bit ROM CODE
		}
/*		// 64-bit ROM CODE
		write_byte(0x10);	// Byte 0
		write_byte(0x6D);	// Byte 1
		write_byte(0xF4);	// Byte 2
		write_byte(0x8F);	// Byte 3
		write_byte(0x02);	// Byte 4
		write_byte(0x08);	// Byte 5
		write_byte(0x00);	// Byte 6
		write_byte(0xB1);	// Byte 7
		// 64-bit ROM CODE
*/
		write_byte(0x44);	// Master issues Convert T command.
		wait_ready();		// Master applies strong pullup to DQ for the duration of the conversion (tCONV).
		if(reset())			// Master issues reset pulse. DS18S20s respond with presence pulse.
		{
			write_byte(0x55);	// Master issues Match ROM command.
			for(i=0; i<8; i++)
			{
				write_byte(rightTempSensorRomCode[i]);	// 64-bit ROM CODE
			}
/*			// 64-bit ROM CODE
			write_byte(0x10);	// Byte 0
			write_byte(0x6D);	// Byte 1
			write_byte(0xF4);	// Byte 2
			write_byte(0x8F);	// Byte 3
			write_byte(0x02);	// Byte 4
			write_byte(0x08);	// Byte 5
			write_byte(0x00);	// Byte 6
			write_byte(0xB1);	// Byte 7
			// 64-bit ROM CODE
*/
			write_byte(0xBE);	// Master issues Read Scratchpad command.
			for(i=0; i<9; i++)
			{
				storeTemp [i] = read_byte();	//	Master reads entire scratchpad including CRC. The master then recalculates the CRC of the first eight data bytes from the scratchpad and compares the calculated CRC with the read CRC (byte 9). If they match, the master continues; if not, the read operation is repeated.
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

	byte0 = storeTemp [0];
	byte1 = storeTemp [1];
	byte6 = storeTemp [6];
	byte7 = storeTemp [7];

	k = ((byte7 - byte6) / byte7) + 0.25;

	if((byte1 == 0x00) && (byte0 == 0x00))
	{
		tC = (byte0/2);
		j = tC - k;
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Temperature: ");		// uart debug information string
		transmitUartInt(tC);		// uart debug information string 
		transmitUartString(".0 C\r\n");			// uart debug information string
	#endif
//	LCD_COMMAND(LCD_SELECT_4ROW);	// select row 4
	LCD_DATA_INT(tC);		//
	LCD_DATA_STRING(".0 C");		//
	}
	else if((byte1 == 0x00) && (byte0 != 0x00))
	{
//		transmitUartString("+");		// POSITIVE TEMPERATURE
		tC = (byte0/2);
		j = tC - k;
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] Temperature: +");		// uart debug information string
		transmitUartInt(tC);		// uart debug information string 
		transmitUartString(".0 C\r\n");			// uart debug information string
	#endif
//	LCD_COMMAND(LCD_SELECT_4ROW);	// select row 4
	LCD_DATA_INT(tC);		//
	LCD_DATA_STRING(".0 C");		//
	}
	else if((byte1 == 0xFF) && (byte0 != 0x00))
	{
//		transmitUartString("-");		// NEGATIVE TEMPERATURE
//		tC = ((byte0 - 255.5) / 2);		// ne e dobre obraboteno za otricatelni chisla
		tC = ((byte0 - 255) / 2);
		j = tC - k;
	#ifdef DEBUG_INFO
		transmitUartString("[UART INFO] Temperature: -");		// uart debug information string
		transmitUartInt(tC);		// uart debug information string 
		transmitUartString(".0 C\r\n");			// uart debug information string
	#endif
//	LCD_COMMAND(LCD_SELECT_4ROW);	// select row 4
	LCD_DATA_INT(tC);		//
	LCD_DATA_STRING(".0 C");		//
	}
	else
	{
		//lcdDataString("ERROR!");	// ERROR not return to display!!!!
	#ifdef DEBUG_ERROR
		transmitUartString("[UART ERROR] ERROR TEMPERATURE\r\n");		// uart debug information string
	#endif
		return 1;
	}
	temper = tC;

	return temper;
}

void about(void)
{
	#ifdef DEBUG_ERROR
		transmitUartString("[UART INFO] =====================================================\r\n");
		transmitUartString("[UART INFO] \tAuthors and creators: P.Upinov and P.Stoyanov\r\n");
		transmitUartString("[UART INFO] \tDevice name: Digital Control Audio System\r\n");
		transmitUartString("[UART INFO] \tFirmware version beta ");
		transmitUartInt(FIRMWARE_VERSION);
		transmitUartString("\r\n");
		transmitUartString("[UART INFO] =====================================================\r\n");
		transmitUartString("[UART INFO] Da dobavq upravlenie na:\
						\r\n[UART INFO] - FAN smart controlling			\
						\r\n[UART INFO] - DS18S20						\
						\r\n[UART INFO] - RTC							\
						\r\n[UART INFO] - Memory\r\n");
	#endif
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
	ext0_intrpt_off();	// DISABLE new IR DETECTION
	cli();				//	SREG &= ~(1 << I);

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
			irDecode();
		}
		else
		{
		}
    }
// LOGIC CHECK END
    sei();				//	SREG |= (1 << I);
	ext0_intrpt_on();	// ENABLE new IR DETECTION
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
	//	timer2Internal_intrpt_off();	// DISABLE new INTERNAL TIMER 2 INTERRUPT
	TIMSK &= ~(1 << OCIE2); // enable the CTC interrupt
	// LOGIC CHECK BEGIN

	isr2++;
	if(isr2 == 2)	// isr = 2, TCCR = 0x----1111; isr = 62, TCCR = 0x----1001
	{
		#ifdef DEBUG_ERROR
			transmitUartString("[UART INFO] Timer2_COMP isr2 = ");
			transmitUartInt(isr2);
			transmitUartString(", time = ");
			transmitUartString(" \r\n");
		#endif

		autoControlTemperature();
	}
	else if(isr2 >= 65535)
	{
		isr2 = 0;
	}

	TIMSK |= (1 << OCIE2); // enable the CTC interrupt
	// LOGIC CHECK END
	//	timer2Internal_intrpt_on();		// ENABLE new INTERNAL TIMER 2 INTERRUPT
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
	port_init();		// 1. IO init and configure all port
	LCD_INIT();			// 2. LCD init and reset all lcd contain
	uart_init();		// 3. UART debug init
	timer1_init();		// 4. FAN INIT
	timer2_init();		// 5. Auto controlled fan by temperature sensors when timer2 is interrupted
	about();			// x. Any debug important information

	pga2310_init();		// SPI init and reset all (U6, U7, U8) PGA2310 volume values to null
	relays_in_init();	// nujno e daden reset be da bade izkliuche usilvatelq, togava reletata za vhod i izhod (bez power relay) sa ostanali vkliucheni
	relays_out_init();	// nujno e daden reset be da bade izkliuche usilvatelq, togava reletata za vhod i izhod (bez power relay)sa ostanali vkliucheni

}

void buttons_press()
{
	flagStatusBits = &fSB;
	flagStatusBits->flagPower=0;	// inicializirane s nuli, no nai veroqtno poradi tova che e globalna stru
	flagStatusBits->flagMute=0;		// inicializirane

/*
// DEBUG
		LED_low_DISPLAYLED_high();		// PORTD4 - LED OFF (logic "0"), DISPLAY BACKLIGHT ON (logic "0"),  NON PWM, NON TIMER1
		LCD_CLEAR_CONTAIN();						// clear all contain on display
		LCD_COMMAND(LCD_ON);						// LCD ON without CURSOR

		LCD_COMMAND(LCD_SELECT_3ROW);	// select row 3
		LCD_DATA_STRING("flagPower = ");	// 20 symbols
		LCD_DATA_INT(flagStatusBits->flagPower);		// 20 symbols
		LCD_COMMAND(LCD_SELECT_4ROW);	// select row 3
		LCD_DATA_STRING("flagMute = ");	// 20 symbols
		LCD_DATA_INT(flagStatusBits->flagMute);		// 20 symbols
	_delay_ms(2000);
*/
	while(1)
	{
		if(BUTTON_ON_OFF_low() && flagStatusBits->flagPower == 0)//fSB.flagPower == 0)//flagStatusBits->flagPower == 0)	// obj ptr flagStatusBtnRegister from struct flagStatusBtnOnOff
		{
//			flagStatusBits->flagPower = 1;			// filter za buton ON
			ampliferOn();
//			_delay_ms(1000);	// izchakvane za natiskane i otpuskane na buton - filtar treptqsht kontakt buton
		}
		else if(BUTTON_ON_OFF_low() && flagStatusBits->flagPower == 1)//fSB.flagPower == 1)//flagStatusBits->flagPower == 1)
		{
//			flagStatusBits->flagPower = 0;			// filter za buton OFF
			ampliferOff();
		}
		else if(BUTTON_ESC_low() && flagStatusBits->flagPower == 1)//fSB.flagPower == 1)//flagStatusBits->flagPower == 1)
		{
//			LCD_DATA_STRING("PRESSED BTN ESCAPE  ");	// 20 symbols
//			LCD_COMMAND(LCD_ON);
			LCD_CLEAR_CONTAIN();
			_delay_ms(500);
//			volumeUp();
//			_delay_ms(200);
		}
		else if(BUTTON_ENCODER_low() && flagStatusBits->flagPower == 1)//fSB.flagPower == 1)//flagStatusBits->flagPower == 1)
		{
//			LCD_DATA_STRING("PRESSED BTN ENCODER ");	// 20 symbols
//			LCD_COMMAND(LCD_ON);	// LCD_COMMAND(LCD_OFF);
			_delay_ms(500);
//			volumeDown();
//			_delay_ms(200);

		}
		else if(BUTTON_ESC_low() && flagStatusBits->flagPower == 0)//fSB.flagPower == 0)//flagStatusBits->flagPower == 0)
		{
//			temperature();
//			LCD_CLEAR_CONTAIN();	// kogato e izkliuchen
//			LCD_COMMAND(LCD_ON);
			_delay_ms(500);
//			setupMode();
//			_delay_ms(1000);
		}
		else if(BUTTON_ENCODER_low() && flagStatusBits->flagPower == 0)//fSB.flagPower == 0)//flagStatusBits->flagPower == 0)
		{
			temperature();
//			LCD_COMMAND(LCD_OFF);
			_delay_ms(500);
//			about();
//			_delay_ms(1000);
		}
		else if(flagStatusBits->flagPower == 1)//fSB.flagPower == 1)//flagStatusBits->flagPower == 1)
		{
			volumeProcess();
		}
		else if(flagStatusBits->flagPower == 0)
		{
//			commonEncoderFanSpeedSteping();
//			commonEncoder();
		}
		else
		{
		}
//		autoControlTemperature();
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
	init_all();				// PREDI DA TRAGNEM // inicializacia na vsichko
	ext0_intrpt_on();		// PREDI DA TRAGNEM // ENABLE interrupts to access IR DETECTION as call to function "IR_DECODER()" for -> SONY IR REMOTE
//	ext2_intrpt_on();
//	temperature();

	sei();					// PREDI DA TRAGNEM 		// file "avr/interrupt.h"
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

