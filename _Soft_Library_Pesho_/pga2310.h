/*************************************************************************
*** LIBRARY: PGA2310 / PGA2311 with SPI (Serial Peripheral Interface) ****
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: pga2310.h, v2, 08.09.2015                     *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one SPI        *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _PGA2310_H_
#define _PGA2310_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/************************************
** DEFINITION OF PGA2310 CONSTANTS **
************************************/
#define VOLUME_MUTE					0x00	//	 0; 0x00; volume without sound, software mute
#define VOLUME_ABSOLUTE_MINIMUM		0x00	//   0; 0x00; volume without sound, software mute
#define VOLUME_MINIMUM				0x28	//  40; 0x28; MINIMUM volume with very low sound gain
#define VOLUME_MAXIMUM				0xD7	// 215; 0xD7; MAXIMUM volume with very high sound gain low distortion
//#define VOLUME_ABSOLUTE_MAXIMUM 0xFF	// 255; 0xFF; ABSOLUTE MAXIMUM volume sound with VERY HIGH Distortion

/********************************
** DEFINITION OF PGA2310 PORTS **
********************************/
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

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void PGA2310_U6_SPI(unsigned char pgaVolumeLeft, unsigned char pgaVolumeRight);
void PGA2310_U7_SPI(unsigned char pgaVolumeLeft, unsigned char pgaVolumeRight);
void PGA2310_U8_SPI(unsigned char pgaVolumeLeft, unsigned char pgaVolumeRight);

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _PGA2310_H_
