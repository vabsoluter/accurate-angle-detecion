/*
 * Master_i2c.h
 *
 *  Created on: 06 дек. 2015 г.
 *      Author: Антон
 */

#ifndef MASTER_I2C_H_
#define MASTER_I2C_H_

#define startbyte 0x0F
#define I2Caddress 0x07


mraa_i2c_context i2c;

int MasterSend(char sbyte, char command, char lspeed, char rspeed);
int MasterReceive();
#endif /* MASTER_I2C_H_ */
