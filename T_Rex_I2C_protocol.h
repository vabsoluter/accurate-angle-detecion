/*
 * New MOTOR_I2C_protocol.h
 *
 *  Created on: 07 окт. 2016 г.
 *      Author: Антон
 */

#ifndef T_REX_I2C_PROTOCOL_H_
#define T_REX_I2C_PROTOCOL_H_

/* --Информационная часть-- */
/*
The T'REX sample code expects 27 bytes of data to be sent to it in a specific order, this is the "command data packet"
If you do not send the correct number of bytes in the correct sequence then the T'REX will ignore the data and set the error flag
Your software can use this error flag to re-send datapackets that may have been corrupted due to electrical interferance

Master to Slave data packet - 4 bytes

byte	Start		0xF0
byte	command
byte	lmspeed
byte	rmspeed

When requested, the T'REX sample code will send a data packet reporting it's status

Slave to Master data packet - 2 bytes

byte  Start
byte  errorflag

If the Motor controller receives faulty data (possibly due to electrical interference) it will report the problem using the error flag
Error Flag Bits

BIT0: wrong start byte or number of bytes received
BIT1: incorrect command					- must be from 0 to 6
BIT2: incorrect motor speed             - left and right motor speeds must be an integer from 5 to +220
BIT3: incorrect motor delta speed       - must be an integer from 0 to 20


Note: All integers must be sent MSB first

*/



#endif /* T_REX_I2C_PROTOCOL_H_ */
