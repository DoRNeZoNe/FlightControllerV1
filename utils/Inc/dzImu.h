/*
 * dzImu.h
 *
 *  Created on: Mar 12, 2023
 *      Author: Vipul Gupta
 */

#ifndef DZFLIGHTCONTROLLER_UTILS_INC_DZIMU_H_
#define DZFLIGHTCONTROLLER_UTILS_INC_DZIMU_H_

#include "DZ_MPU_REGISTER_DESC.h"
#include "GLOBAL_VARIABLES.h"
#include "dzLogger.h"
#include "dzI2c.h"

class dzImu {

private:

dzLogger* loggerObj;
dzI2c* i2cObj;

double biasIterations = 500;

// Timer variables
uint32_t tickStart = HAL_GetTick();

// Init variables
uint8_t wakeUpImuByte = 0x00;
uint8_t dataRateByte = 0x07; // 1Khz

// Data Structure to save the data

typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    double Ax;
    double Ay;
    double Az;

    double Gx;
    double Gy;
    double Gz;

    double AxBias;
    double AyBias;
    double AzBias;

    double GxBias;
    double GyBias;
    double GzBias;

    double angleAccX;
    double angleAccY;

    double angleGyroX;
    double angleGyroY;

    double pitch;
    double roll;

    double KalmanAngleX;
    double KalmanAngleY;

} IMU_DATA_t;

float accResolution, gyroResolution;
uint8_t imuGyroScale;
uint8_t imuAccScale;

public:
IMU_DATA_t *imuDataStore;

private:
	void setAccResolution(uint8_t aScale);
	void setGyroResolution(uint8_t gScale);

public:

	// Constructors
	dzImu(dzI2c* i2cObject, accScale accResolution, gyroScale gyroResolution, dzLogger* logger);

	// Init functions
	HAL_StatusTypeDef init();

	// Read Data functions
	// Accelerometer
	HAL_StatusTypeDef readRawAccData();
	HAL_StatusTypeDef processAccData();
	HAL_StatusTypeDef readAndProcessAcc();
	HAL_StatusTypeDef getAccBias();

	// Gyroscope
	HAL_StatusTypeDef readRawGyroData();
	HAL_StatusTypeDef processGyroData();
	HAL_StatusTypeDef readAndProcessGyro();
	HAL_StatusTypeDef getGyroBias();

	// Angle Functions
	HAL_StatusTypeDef anglesViaComplimentaryFilter();

	// Logger Function
	void dzLog(char *msg);

};

#endif /* DZFLIGHTCONTROLLER_UTILS_INC_DZIMU_H_ */
