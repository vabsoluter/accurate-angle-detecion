
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

// Регистры конфигурации
#define HMC5883L_CONF_REG_A 0x00
#define HMC5883L_CONF_REG_B 0x01

// Регистр режима
#define HMC5883L_MODE_REG 0x02

// Регистр данных
#define HMC5883L_X_MSB_REG 0
#define HMC5883L_X_LSB_REG 1
#define HMC5883L_Z_MSB_REG 2
#define HMC5883L_Z_LSB_REG 3
#define HMC5883L_Y_MSB_REG 4
#define HMC5883L_Y_LSB_REG 5
#define DATA_REG_SIZE 6

// Статус-регистр
#define HMC5883L_STATUS_REG 0x09

// ИД-регистры
#define HMC5883L_ID_A_REG 0x0A
#define HMC5883L_ID_B_REG 0x0B
#define HMC5883L_ID_C_REG 0x0C

#define HMC5883L_CONT_MODE 0x00
#define HMC5883L_DATA_REG 0x03

// Сдвиги
#define GA_0_88_REG 0x00 << 5
#define GA_1_3_REG 0x01 << 5
#define GA_1_9_REG 0x02 << 5
#define GA_2_5_REG 0x03 << 5
#define GA_4_0_REG 0x04 << 5
#define GA_4_7_REG 0x05 << 5
#define GA_5_6_REG 0x06 << 5
#define GA_8_1_REG 0x07 << 5

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
 * Получение угла поворота при помощи магнитометра
 * (электронного компаса)
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

		// получение направления
		direction = atan2(y, x);
		// домножаем на отрицательный коэффициент - установлен экспериментально.
	    directionDeg = direction * 180 * -2.2/M_PI;

		delete i2c;
	return directionDeg;
}

int main(int argc, char *argv[])
{
	Kalman kalmanZ;
	kalmanZ.setAngle(0.0); // Начальный угол.

	/*Установка коэффициентов для фильтра Калмана*/
	kalmanZ.setQangle(0.007);
	kalmanZ.setQbias(0.003);
	kalmanZ.setRmeasure(0.001);

	//получение начального угла поворота
	float startHeading = getHeading();
	float currentHeading = 0.0;

	// Начальная инициализация переменных
	float *ang; // Создание массива для получения угла с гироскопа
	float angleZGyro = 0.0;
	float rotateAngleGyro = 0.0;
	float angleMagnet = 0.0;
	upm::Itg3200* gyro = new upm::Itg3200(1);
	gyro->calibrate();

	float timer;
	// Запись времени начала измерения
	float oldTime = clock();
	float kalAngleZ;

		while (1) {

			//получение угла поворота гироскопом и запоминаем момент времени измерения
			gyro->update();
			ang = gyro->getRotation();
			timer = clock();

			// Получение угла поворота магнитометром
			currentHeading = getHeading();
			// Получение угла поворота гироскопом
			angleZGyro += ((float) (ang[2]) * ((float) (timer - oldTime)/ CLOCKS_PER_SEC));
			rotateAngleGyro = angleZGyro*1.4; //умножаем на коэффициент - установлен экспериментально								 // домножаем на коэaффициент - установлен экспериментально

			// Подсчёт угла отклонения (дирекционного) по магнитометру
			if(startHeading >= 0.0 && startHeading <= 180.0 && currentHeading >= 0.0 && currentHeading <= 180.0) {
				// Случай 1
				angleMagnet = currentHeading - startHeading;

			} else if(startHeading < 0.0 && startHeading > -180.0 && currentHeading < 0.0 && currentHeading > -180.0) {
					// Случай 4
					angleMagnet = currentHeading - startHeading;
				} else if(startHeading > 0.0 && startHeading <= 180.0 && currentHeading < 0.0 && currentHeading > -180.0) {
					// Случай 2
							//смена знаков в нуле
							if(fabs(currentHeading) < 90.0) {
								angleMagnet = currentHeading - startHeading;
							} else {
								// Смена знаков в 180
								angleMagnet = 180.0 - fabs(currentHeading) + 180.0 - fabs(startHeading);
							}
					} else if (startHeading < 0.0 && startHeading > -180.0 && currentHeading > 0.0 && currentHeading < 180.0) {
						// Случай 3
						// Смена знаков в нуле
							if(fabs(currentHeading) < 90.0) {
								angleMagnet = fabs(currentHeading) - fabs(startHeading);
								} else {
								// Смена знаков в 180
								angleMagnet = 180.0 - fabs(currentHeading) + 180.0 - fabs(startHeading);
							}
					}

			// Выполняем фильтрацию
			kalAngleZ = kalmanZ.getAngle(angleMagnet, rotateAngleGyro, (float)(timer - oldTime)/ CLOCKS_PER_SEC);
			oldTime = timer; // текущее время запоминаем
			prevHeading = currentHeading; // текущее измерение запоминаем

			fprintf(stdout, "angleMagnet %5.2f\t angleZGyro %5.2f\t kalAngleZ %5.2f\n", angleMagnet, rotateAngleGyro, kalAngleZ);
//			fprintf(stdout, "kalAngleZ %5.2f\n", kalAngleZ);
//			fprintf(stdout, "angleMagnet %f\n", sign(angleMagnet));
			usleep(10000);
		}

	return 0;
}

/**
 * Функция реализует алгоритм определения точного угла поворота,
 * используя гироскоп и магнитометр
 * на входе получает требуемое значение угла поворота в градусах
 * возвращает точное значение угла поворота в градусах
 */
float getAccurateAngle(float deg)
{
	Kalman kalmanZ;
		kalmanZ.setAngle(0.0); // Начальный угол.

		/*Установка коэффициентов для фильтра Калмана*/
		kalmanZ.setQangle(0.007);
		kalmanZ.setQbias(0.003);
		kalmanZ.setRmeasure(0.001);

		//получение начального угла поворота
		float startHeading = getHeading();
		float currentHeading = 0.0;
		float prevHeading = 0.0; // предыдущее значение угла

		// Начальная инициализация переменных
		float *ang; // Создание массива для получения угла с гироскопа
		float angleZGyro = 0.0;
		float rotateAngleGyro = 0.0;
		float angleMagnet = 0.0;
		upm::Itg3200* gyro = new upm::Itg3200(1);
		gyro->calibrate();

		float timer;
		// Запись времени начала измерения
		float oldTime = clock();
		float kalAngleZ;

			while (1) {

				//получение угла поворота гироскопом и запоминаем момент времени измерения
				gyro->update();
				ang = gyro->getRotation();
				timer = clock();

				// Получение угла поворота магнитометром
				currentHeading = getHeading();
				// Получение угла поворота гироскопом
				angleZGyro += ((float) (ang[2]) * ((float) (timer - oldTime)/ CLOCKS_PER_SEC));
				rotateAngleGyro = angleZGyro*1.4; //умножаем на коэффициент - установлен экспериментально								 // домножаем на коэaффициент - установлен экспериментально

				// Подсчёт угла отклонения (дирекционного) по магнитометру
				if(startHeading > 0.0 && startHeading <= 180.0 && currentHeading > 0.0 && currentHeading < 180.0) {
					// Случай 1
					angleMagnet = currentHeading - startHeading;
				} else if(startHeading < 0.0 && startHeading > -180.0 && currentHeading < 0.0 && currentHeading > -180.0) {
						// Случай 4
						angleMagnet = currentHeading - startHeading;
					} else if(startHeading < 0.0 && startHeading > -180.0 && currentHeading > 0.0 && currentHeading < 180.0) {
						// Случай 2
						// Проверка на смену знаков
							if((currentHeading < 0.0 && prevHeading > 0.0) || (currentHeading > 0.0 && prevHeading < 0.0)) {
								//смена знаков в нуле
								if(fabs(currentHeading) < 90.0) {
									angleMagnet = currentHeading - startHeading;
								} else {
									// Смена знаков в 180
									angleMagnet = 180.0 - fabs(currentHeading) + 180.0 - fabs(startHeading);
								}
							}
						} else if (startHeading < 0.0 && startHeading > -180.0 && currentHeading > 0.0 && currentHeading < 180.0) {
							// Случай 3
							// Проверка на смену знаков
							if((currentHeading < 0.0 && prevHeading > 0.0) || (currentHeading > 0.0 && prevHeading < 0.0)) {
								// Смена знаков в нуле
								if(fabs(currentHeading) < 90.0) {
									angleMagnet = fabs(currentHeading) - fabs(startHeading);
								} else {
									// Смена знаков в 180
									angleMagnet = 180.0 - fabs(currentHeading) + 180.0 - fabs(startHeading);
								}
							}
						}

				// выполняем фильтрацию
				kalAngleZ = kalmanZ.getAngle(angleMagnet, rotateAngleGyro, (float)(timer - oldTime)/ CLOCKS_PER_SEC);
				oldTime = timer; // текущее вресмя запоминаем
				prevHeading = currentHeading; // текущее измерение запоминаем

				usleep(100000);

				if(abs(kalAngleZ)>= abs((float)deg)-5.0)
					return kalAngleZ;
			}
}

