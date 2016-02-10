/*************************************************************************
*** LIBRARY: I2C / IIC / ATmel Two Wire Interface            *************
*** AUTHOR:  PETAR UPINOV, email: petar.upinov@gmail.com     *************
*** FILE NAME: i2c_twi.h, v0.01, 18.10.2015                  *************
*** SOFT IDE: AVR-GCC compiler                               *************
*** HARD uCU: ATmel AVR Microcontrollers with one I2C / TWI  *************
*** TEST: ATmega8535@16MHz, ATmega32@16MHz                   *************
*************************************************************************/

#ifndef _I2C_TWI_H_
#define _I2C_TWI_H_

/******************
** INCLUDE FILES **
******************/

/********************************************************************************************
*********************************** START OF DEFINISIONS ************************************
********************************************************************************************/

/**********************
** DEFINITION OF SPI **
**********************/
#define I2C_SCL_PIN    PB0		// PORTC0 TWO-WIRE INTERFACE SERIAL CLOCK, OUTPUT PIN I2C
#define I2C_SCL_PORT   PORTC	// PORTC0 TWO-WIRE INTERFACE SERIAL CLOCK, OUTPUT PIN I2C
#define I2C_SDA_PIN    PC1		// PORTC1 TWO-WIRE INTERFACE SERIAL DATA, INPUT/OUTPUT PIN I2C
#define I2C_SDA_PORT   PORTC	// PORTC1 TWO-WIRE INTERFACE SERIAL DATA, INPUT/OUTPUT PIN I2C

#define I2C_SCL_low()    (I2C_SCL_PORT&=~_BV(I2C_SCL_PIN))
#define I2C_SCL_high()   (I2C_SCL_PORT|=_BV(I2C_SCL_PIN))
#define I2C_SDA_low()    (I2C_SDA_PORT&=~_BV(I2C_SDA_PIN))
#define I2C_SDA_high()   (I2C_SDA_PORT|=_BV(I2C_SDA_PIN))

/********************************************************************************************
************************************ END OF DEFINISIONS *************************************
********************************************************************************************/

/********************************************************************************************
****************************** START DECLARATION OF FUNCTIONS *******************************
********************************************************************************************/
void i2c_init();
void i2c_start(void);
void i2c_stop();
void i2c_write(unsigned char data);
unsigned char i2c_read(unsigned char isLast);

/********************************************************************************************
************************************* END OF FUNCTIONS **************************************
********************************************************************************************/

#endif // _I2C_TWI_H_
