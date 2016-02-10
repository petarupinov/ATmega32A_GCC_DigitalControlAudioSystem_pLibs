/*************************************************************************
*** LIBRARY: SPI / Serial Peripheral Interface               *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: spi.h, v0.03, 29.11.2015                      *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one SPI        *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _SPI_H_
#define _SPI_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/**********************
** DEFINITION OF SPI **
**********************/
#define DEVICE_SDI_PIN    PB5	// PORTA5 DEVICE SERIAL DATA INPUT PIN MOSI
#define DEVICE_SDI_PORT   PORTB	// PORTA5 DEVICE SERIAL DATA INPUT PIN MOSI
#define DEVICE_SDO_PIN    PB6	// PORTB6 DEVICE SERIAL DATA OUTPUT PIN	MISO
#define DEVICE_SDO_PORT   PORTB	// PORTB6 DEVICE SERIAL DATA OUTPUT PIN	MISO
#define DEVICE_SCK_PIN    PB7	// PORTA7 DEVICE SERIAL CLOCK INPUT PIN	SCK
#define DEVICE_SCK_PORT   PORTB	// PORTA7 DEVICE SERIAL CLOCK INPUT PIN	SCK

#define DEVICE_CS_PIN     PB4	// PORTB4 DEVICE CS ENABLE/DISABLE PIN /CS
#define DEVICE_CS_PORT    PORTB	// PORTB4 DEVICE CS ENABLE/DISABLE PIN /CS

#define DEVICE_SDI_low()    (DEVICE_SDI_PORT&=~_BV(DEVICE_SDI_PIN))
#define DEVICE_SDI_high()   (DEVICE_SDI_PORT|=_BV(DEVICE_SDI_PIN))
#define DEVICE_SDO_low()    (DEVICE_SDO_PORT&=~_BV(DEVICE_SDO_PIN))
#define DEVICE_SDO_high()   (DEVICE_SDO_PORT|=_BV(DEVICE_SDO_PIN))
#define DEVICE_SCK_low()    (DEVICE_SCK_PORT&=~_BV(DEVICE_SCK_PIN))
#define DEVICE_SCK_high()   (DEVICE_SCK_PORT|=_BV(DEVICE_SCK_PIN))

#define DEVICE_CS_low()     (SPI_CS_PORT&=~_BV(DEVICE_CS_PIN))
#define DEVICE_CS_high()    (SPI_CS_PORT|=_BV(DEVICE_CS_PIN))

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void spi_init();
void spi_start();
void spi_stop();
void spi_write_one_byte(unsigned char data);
void spi_write_two_bytes(unsigned char data1, unsigned char data2);		//void DEVICE1_SPI(byte volume_left, byte volume_right);
void spi_write_more_bytes(unsigned char *data);
unsigned char spi_read_more_bytes();

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _SPI_H_
