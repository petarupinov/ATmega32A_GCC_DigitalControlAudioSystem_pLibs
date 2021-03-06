/*************************************************************************
*** LIBRARY: EEPROM (24C64) with I2C Interface               *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: 24c64.c, v0.03, 29.11.2015                    *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one I2C / TWI  *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>			// itoa() - function
#include <avr/eeprom.h>		// working
#include "../ATmega32A_GCC_DigitalControlAudioSystem_pLibs.h"
#include "i2c_twi.h"
#include "24c64.h"
#include "utility.h"		// using for debug and others, byte is custom defined type

/********************************************************************************************
************************************ START OF FUNCTIONS *************************************
********************************************************************************************/

/**************************************
** DEFINITION EEPROM 24C64 FUNCTIONS **
**************************************/

/********************************
** INITIZLIZATION EEPROM 24C64 **
********************************/


/*************************************
** RESET AND ZERO FILL EEPROM 24C64 **
*************************************/
void eeprom_24c64_reset()
{
	unsigned int i;	// 16 bits
// EEPROM WRITE
	i2c_start();
	i2c_write(EEPROM_24C64_I2C_ADDRESS_WRITE);	// EEPROM AT24C64 ADDRESS ACCESS WRITE
// HIGH and LOW BYTE ADDRESS
	i2c_write(EEPROM_24C64_I2C_HIGH_BYTE_ADDRESS_MIN);	// HIGH STORE ADDRESS
	i2c_write(EEPROM_24C64_I2C_LOW_BYTE_ADDRESS_MIN);	// LOW  STORE ADDRESS	
// Write data
	for(i=0; i<8191; i++)	// from 0x0000 to 0x1FFF
	{
		i2c_write(0b00000000);	// Data is stored: 0x00 from Address 0x00 to 0xFF
	}
	i2c_stop();
}

/**************************
** WRITE TO EEPROM 24C64 **
**************************/
void eeprom_24c64_write()
{
// Za da se vijda tova koeto se e zapisalo v pametta, a ne ot tozi bibliotechen fail !!!!!!!!!
// ZA TAZI CEL TRQBVA DA IMA UKAZATEL/MASIV KOITO DA VARNE REZULTATA OT CHETENETO NA PAMETTA
// SAMO AKO E NUJNO DA SE VIJDA KAKVO E BILO ZAPISANO !

// EEPROM WRITE
	i2c_start();
	i2c_write(EEPROM_24C64_I2C_ADDRESS_WRITE);	// EEPROM AT24C64 ADDRESS ACCESS WRITE
// HIGH and LOW BYTE ADDRESS
	i2c_write(EEPROM_24C64_I2C_HIGH_BYTE_ADDRESS_MIN);	// HIGH STORE ADDRESS
	i2c_write(EEPROM_24C64_I2C_HIGH_BYTE_ADDRESS_MIN);	// LOW  STORE ADDRESS	
// Write data
	i2c_write(0x30);	// Data is stored: 0x30 - '0'
	i2c_write(0x31);	// Data is stored: 0x31 - '1'
	i2c_write(0x32);	// Data is stored: 0x32 - '2'
	i2c_write(0x33);	// Data is stored: 0x33 - '3'
	i2c_write(0x34);	// Data is stored: 0x34 - '4'
	i2c_stop();
}

/***************************
** READ FROM EEPROM 24C64 **
***************************/
void eeprom_24c64_read()
{
	// TOVA TRQBVA DA SE IMPLEMENTIRA TAM KADETO SE IZVIKVA READ-a v PROJECT FILE, za da se vijda tova koeto se e zapisalo v pametta, a ne ot tozi bibliotechen fail !!!!!!!!!
	// ZA TAZI CEL TRQBVA DA IMA UKAZATEL/MASIV KOITO DA VARNE REZULTATA OT CHETENETO NA PAMETTA
	// SAMO AKO E NUJNO DA SE VIJDA KAKVO E BILO ZAPISANO !
	
	byte eepromReceiveByte0, eepromReceiveByte1, eepromReceiveByte2, eepromReceiveByte3, eepromReceiveByte4, eepromReceiveByte5;
// EEPROM READ
	i2c_start();
	i2c_write(EEPROM_24C64_I2C_ADDRESS_WRITE);		// EEPROM 24C64 ADDRESS ACCESS WRITE
// HIGH and LOW BYTE ADDRESS
	i2c_write(EEPROM_24C64_I2C_HIGH_BYTE_ADDRESS_MIN);	// HIGH STORE ADDRESS
	i2c_write(EEPROM_24C64_I2C_HIGH_BYTE_ADDRESS_MIN);	// LOW  STORE ADDRESS	

	i2c_start();
	i2c_write(EEPROM_24C64_I2C_ADDRESS_READ);		// EEPROM 24C64 ADDRESS ACCESS READ
	eepromReceiveByte0	= i2c_read(0);				// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte1	= i2c_read(0);				// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte2	= i2c_read(0);				// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte3	= i2c_read(0);				// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte4	= i2c_read(0);				// EEPROM DATA READ BYTE	// i2c_read(0) parametar raven na 0 prodaljava komunikaciqta kato potvarjdava ACK
	eepromReceiveByte5	= i2c_read(1);				// EEPROM DATA READ BYTE	// i2c_read(1) parametar razlichen ot 0 spira komunikaciqta NACK
	i2c_stop();
/*
	// TOVA TRQBVA DA SE IMPLEMENTIRA TAM KADETO SE IZVIKVA READ-a v PROJECT FILE, za da se vijda tova koeto se e zapisalo v pametta, a ne ot tozi bibliotechen fail !!!!!!!!!
	// ZA TAZI CEL TRQBVA DA IMA UKAZATEL/MASIV KOITO DA VARNE REZULTATA OT CHETENETO NA PAMETTA
	// SAMO AKO E NUJNO DA SE VIJDA KAKVO E BILO ZAPISANO !
	
	LCD_INIT();									// LCD INITIZLIZATION
	LCD_EXECUTE_COMMAND(LCD_SELECT_4ROW);		// select row 4

	uart_transmit("EEPROM READ BYTE: ", 18);	// "EEPROM READ BYTE: " UART
	LCD_EXECUTE_DATA("EEPROM: ", 8);			// "EEPROM: " LCD (za LCD stringa se broi ot 0 do 5 = 6 simvola)

	uart_transmit_one(eepromReceiveByte0);
	uart_transmit_one(eepromReceiveByte1);
	uart_transmit_one(eepromReceiveByte2);
	uart_transmit_one(eepromReceiveByte3);
	uart_transmit_one(eepromReceiveByte4);
	uart_transmit_one(eepromReceiveByte5);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte0);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte1);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte2);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte3);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte4);
	LCD_EXECUTE_DATA_ONE(eepromReceiveByte5);
*/
}

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/
