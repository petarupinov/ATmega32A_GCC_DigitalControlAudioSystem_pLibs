/*************************************************************************
*** LIBRARY: EEPROM (24C64) with I2C Interface               *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: 24c64.h, v0.01, 18.10.2015                    *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one I2C / TWI  *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _24C64_H_
#define _24C64_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

//unsigned char	eepromCommand = 0x00;
//unsigned char eepromAddress = 0x04;

/****************************
** DEFINITION EEPROM 24C64 **
****************************/
#define EEPROM_24C64_I2C_SDA_PIN  PC1	// PINC1		// EEPROM I2C SDA
#define EEPROM_24C64_I2C_SDA_PORT PORTC	// PORTC		// EEPROM I2C SDA
#define EEPROM_24C64_I2C_SDA_DDR  DDRC	// DDRC			// EEPROM I2C SDA

#define EEPROM_24C64_I2C_SCL_PIN  PC0	// PINC0		// EEPROM I2C SCL
#define EEPROM_24C64_I2C_SCL_PORT PORTC	// PORTC		// EEPROM I2C SCL

#define EEPROM_24C64_I2C_SDA_low()		(EEPROM_24C64_I2C_SDA_PORT&=~_BV(EEPROM_24C64_I2C_SDA_PIN))
#define EEPROM_24C64_I2C_SDA_high()		(EEPROM_24C64_I2C_SDA_PORT|=_BV(EEPROM_24C64_I2C_SDA_PIN))
#define EEPROM_24C64_I2C_SCL_low()		(EEPROM_24C64_I2C_SCL_PORT&=~_BV(EEPROM_24C64_I2C_SCL_PIN))
#define EEPROM_24C64_I2C_SCL_high()		(EEPROM_24C64_I2C_SCL_PORT|=_BV(EEPROM_24C64_I2C_SCL_PIN))

/**************************************
** DEFINITION EEPROM 24C64 CONSTANTS **
**************************************/
#define EEPROM_24C64_I2C_ADDRESS_READ		0xA1	// EEPROM 24C64 SLAVE ADDRESS  READ: 0xA1 = 0b10100001 = 0b 1 0 1 0 NC 0 0 R/W -> R = 1		*******-> NC connect to GND ('0') !!!!!!!!!!!
#define EEPROM_24C64_I2C_ADDRESS_WRITE		0xA0	// EEPROM 24C64 SLAVE ADDRESS WRITE: 0xA0 = 0b10100000 = 0b 1 0 1 0 NC 0 0 R/W -> W = 0		*******-> NC connect to GND ('0') !!!!!!!!!!!
#define EEPROM_24C64_I2C_HIGH_BYTE_ADDRESS	0x00	// | DEVICE_ADDRESS(R/W) | HIGH_BYTE_ADDRESS | LOW_BYTE_ADDRESS | DATA_BYTE_0 | DATA_BYTE_1 | .... | DATA_BYTE_N |
#define EEPROM_24C64_I2C_LOW_BYTE_ADDRESS	0x00	// DEVICE_ADDRESS(R/W), 0bxxx00000, 0b00000000

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/

void EEPROM_24C64_READ();
void EEPROM_24C64_WRITE();
// void i2c_init();
// void i2c_start(void);
// void i2c_stop();
// void i2c_write(unsigned char data);
// unsigned char i2c_read(unsigned char isLast);

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _24C64_H_
