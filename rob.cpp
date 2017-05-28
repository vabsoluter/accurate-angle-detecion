
#include <stdlib.h>
#include <iostream>
using namespace std;

#include <mraa/i2c.hpp>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include "mraa.hpp"
#include "itg3200.h"
#include "Kalman.h"

#define MAX_BUFFER_LENGTH 6
#define HMC5883L_I2C_ADDR 0x1E

// configuration registers
#define HMC5883L_CONF_REG_A 0x00
#define HMC5883L_CONF_REG_B 0x01

// mode register
#define HMC5883L_MODE_REG 0x02

// data register
#define HMC5883L_X_MSB_REG 0
#define HMC5883L_X_LSB_REG 1
#define HMC5883L_Z_MSB_REG 2
#define HMC5883L_Z_LSB_REG 3
#define HMC5883L_Y_MSB_REG 4
#define HMC5883L_Y_LSB_REG 5
#define DATA_REG_SIZE 6

// status register
#define HMC5883L_STATUS_REG 0x09

// ID registers
#define HMC5883L_ID_A_REG 0x0A
#define HMC5883L_ID_B_REG 0x0B
#define HMC5883L_ID_C_REG 0x0C

#define HMC5883L_CONT_MODE 0x00
#define HMC5883L_DATA_REG 0x03

// scales
#define GA_0_88_REG 0x00 << 5
#define GA_1_3_REG 0x01 << 5
#define GA_1_9_REG 0x02 << 5
#define GA_2_5_REG 0x03 << 5
#define GA_4_0_REG 0x04 << 5
#define GA_4_7_REG 0x05 << 5
#define GA_5_6_REG 0x06 << 5
#define GA_8_1_REG 0x07 << 5

// digital resolutions
#define SCALE_0_73_MG 0.73
#define SCALE_0_92_MG 0.92
#define SCALE_1_22_MG 1.22
#define SCALE_1_52_MG 1.52
#define SCALE_2_27_MG 2.27
#define SCALE_2_56_MG 2.56
#define SCALE_3_03_MG 3.03
#define SCALE_4_35_MG 4.35

int running = 0;
float direction = 0.0;
float directionDeg = 0.0;
int16_t x = 0, y = 0, z = 0;
uint8_t rx_tx_buf[MAX_BUFFER_LENGTH];

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing nicely\n");
        running = -1;
    }
}

/*
 * get heading by electric compass sensor
 * (magnitometr)
 */

float getHeading()
{
		mraa::I2c* i2c;
		i2c = new mraa::I2c(0);

		i2c->address(HMC5883L_I2C_ADDR);
		rx_tx_buf[0] = HMC5883L_CONF_REG_B;
		rx_tx_buf[1] = GA_1_3_REG;
		i2c->write(rx_tx_buf, 2);

		i2c->address(HMC5883L_I2C_ADDR);
		rx_tx_buf[0] = HMC5883L_MODE_REG;
		rx_tx_buf[1] = HMC5883L_CONT_MODE;
		i2c->write(rx_tx_buf, 2);

		signal(SIGINT, sig_handler);

		i2c->address(HMC5883L_I2C_ADDR);
		i2c->writeByte(HMC5883L_DATA_REG);

		i2c->address(HMC5883L_I2C_ADDR);
		i2c->read(rx_tx_buf, DATA_REG_SIZE);

		x = (rx_tx_buf[HMC5883L_X_MSB_REG] << 8) | rx_tx_buf[HMC5883L_X_LSB_REG];
		z = (rx_tx_buf[HMC5883L_Z_MSB_REG] << 8) | rx_tx_buf[HMC5883L_Z_LSB_REG];
		y = (rx_tx_buf[HMC5883L_Y_MSB_REG] << 8) | rx_tx_buf[HMC5883L_Y_LSB_REG];

		// scale and calculate direction
		direction = atan2(y * SCALE_0_92_MG, x * SCALE_0_92_MG);

		// check if the signs are reversed
		if (direction < 0)
			direction += 2 * M_PI;
		directionDeg = direction * 180 / M_PI;
		delete i2c;
	return directionDeg;
}

int main(int argc, char *argv[])
{
	Kalman kalmanZ;
	kalmanZ.setAngle(0.0); // Начальные углы.
	kalmanZ.setQangle(0.007);    // 0.007
	kalmanZ.setQbias(0.003);     // 0.003
	kalmanZ.setRmeasure(0.001);  // 0.001

		float startHeading = getHeading();
		if (startHeading > 180.0)
			startHeading = -(360.0 - startHeading);
		float currentHeading = 0.0;

		float deg = 90.0;
		float *ang;
		float angleZGyro = 0.0;
		float rotateAngleGyro = 0.0;
		float angleMagnet = 0.0;
		upm::Itg3200* gyro = new upm::Itg3200(1);
		gyro->calibrate();

		float timer;
		float oldTime = clock();
		float kalAngleZ;
		while (1) {
			gyro->update();
			ang = gyro->getRotation();
			timer = clock();

			currentHeading = getHeading();
			if (currentHeading > 180.0)
				currentHeading = -(360.0 - currentHeading);

			angleZGyro += ((float) (ang[2]) * ((float) (timer - oldTime)/ CLOCKS_PER_SEC));
			rotateAngleGyro = angleZGyro*1.4;								 // домножаем на коэaффициент - установлен экспериментально
			angleMagnet = 1.8*(fabs(startHeading)-fabs(currentHeading)); // домножаем на коэaффициент - установлен экспериментально
			kalAngleZ = kalmanZ.getAngle(angleMagnet, rotateAngleGyro, (float)(timer - oldTime)/ CLOCKS_PER_SEC);
			//	fprintf(stdout, "kalman: %f; simple: %f\n", kalAngleZ, rotateAngle);
			oldTime = timer;

			//fprintf(stdout, "angleMagnet %5.2f\t rotateAngle %5.2f\t kalAngleZ %5.2f\n", angleMagnet, rotateAngleGyro, kalAngleZ);
			//fprintf(stdout, "kalAngleZ %5.2f\n", kalAngleZ);
			//usleep(100000);
		}
		if(abs(kalAngleZ)>= abs((float)deg)-5.0)
			return kalAngleZ;
	return 0;
}

/**
 * Function implements algorytm of accurate angle dedection
 * using electronic compass sensor and gyroscope
 * float deg required angle in degrees
 * returns float angle in degrees
 */
float getAccurateAngle(float deg)
{
	Kalman kalmanZ;
	kalmanZ.setAngle(0.0); // start angle.
	kalmanZ.setQangle(0.007);
	kalmanZ.setQbias(0.003);
	kalmanZ.setRmeasure(0.001);

		float startHeading = getHeading();
		if (startHeading > 180.0)
			startHeading = -(360.0 - startHeading);
		float currentHeading = 0.0;

		float *ang;
		float angleZGyro = 0.0;
		float rotateAngleGyro = 0.0;
		float angleMagnet = 0.0;
		upm::Itg3200* gyro = new upm::Itg3200(1);
		gyro->calibrate();

		float timer;
		float oldTime = clock();
		float kalAngleZ;
		while (1) {
			gyro->update();
			ang = gyro->getRotation();
			timer = clock();

			currentHeading = getHeading();
			if (currentHeading > 180.0)
				currentHeading = -(360.0 - currentHeading);

			angleZGyro += ((float) (ang[2]) * ((float) (timer - oldTime)/ CLOCKS_PER_SEC));
			rotateAngleGyro = angleZGyro*1.4;								 // multiplying coefficient. Established experimentally
			angleMagnet = 1.8*(fabs(startHeading)-fabs(currentHeading)); // multiplying coefficient. Established experimentally
			kalAngleZ = kalmanZ.getAngle(angleMagnet, rotateAngleGyro, (float)(timer - oldTime)/ CLOCKS_PER_SEC);
			oldTime = timer;
		}
		if(abs(kalAngleZ)>= abs((float)deg)-5.0)
			return kalAngleZ;
}

