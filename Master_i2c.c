/*
 * MasterSend.c
 *
 *  Created on: 06 äåê. 2015 ã.
 *      Author: Àíòîí
 */

#include "mraa.h"

#include <stdio.h>
#include <unistd.h>


#include "Master_i2c.h"

int MasterSend(char sbyte, char command, char lspeed, char rspeed)
{
	uint8_t* tx_buf;
	int i;
	  tx_buf=malloc(4*sizeof(char));
	  for (i=0;i<4;i++)
	  {
		  tx_buf[i]=0;
	  }

	tx_buf[0]=sbyte;					// start byte
	tx_buf[1]=command;					// command number
	tx_buf[2]=lspeed;					// left  motor speed
	tx_buf[3]=rspeed;					// left  motor speed

	mraa_i2c_write(i2c,tx_buf,4);		// transmit bufer to Motor controller

	//printf("Master Command Data Packet Sent\n");

	return 0;
}

int MasterReceive()
{//================================================================= Error Checking ==========================================================
  char d;
  int i=0;
  uint8_t* buf;
  buf=malloc(2*sizeof(char));
  for (i=0;i<2;i++)
  {
	  buf[i]=0;
  }
  mraa_i2c_read(i2c,buf,2);
  //sleep(1);
  //Wire.requestFrom(I2Caddress,24);                                // request 24 bytes from device 007
/*
  while(Wire.available()<24)                                      // wait for entire data packet to be received
  {
    if(i==0) printf("Waiting for slave to send data.");     // Only print message once (i==0)
    if(i>0) printf(".");                                    // print a dot for every loop where buffer<24 bytes
    i++;                                                          // increment i so that message only prints once.
    if(i>79)
    {
      printf("\n");
      i=1;
    }
  }
  */
  //d=Wire.read();                                                  // read start byte from buffer
  //d=mraa_i2c_read_byte(i2c);
  d=buf[0];
  //printf("Start byte:%X\n",buf[0]);
  if(d!=startbyte)                                                // if start byte not equal to 0x0F
  {
    //printf("%u\n",d);
    //while(Wire.available()>0)                                     // empty buffer of bad data
    //{
    	//d=mraa_i2c_read_byte(i2c);
    //}
    //printf("  Wrong Start Byte\n");                         // error message
    return -1;                                                       // quit
  }

  //================================================================ Read Data ==============================================================
  printf("Slave Error Message:\n");                           // slave error report
  printf("%X\n",buf[1]);

  return 0;
}
