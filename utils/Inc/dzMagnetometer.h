/*
 * dzMagnetometer.h
 *
 *  Created on: Apr 29, 2023
 *      Author: Vipul Gupta
 */

#ifndef UTILS_INC_DZMAGNETOMETER_H_
#define UTILS_INC_DZMAGNETOMETER_H_

#include "DZ_MPU_REGISTER_DESC.h"
#include "GLOBAL_VARIABLES.h"
#include "dzLogger.h"
#include "dzI2c.h"

class dzMagnetometer {

private:

// MAG init variables
uint8_t magReset = 0x1;
uint8_t magPowerDown = 0x00;
uint8_t magFuseRomMode = 0x0F;

uint8_t mag16BitOutput = 0x10;
uint8_t magContinuousMeasurementMode2 = 0b0110;
uint8_t magContinousMeasurementAnd16BitOutput = mag16BitOutput | magContinuousMeasurementMode2;

// MAG Status bits
uint8_t magDataReadyStatus = 0x01;
uint8_t magDataOverRunStatus = 0x08;

bool calibrate = false;

// I2C object
dzI2c* magI2cObj;

// Logger Object
dzLogger* loggerObj;

typedef struct
{

    int16_t Mag_X_RAW;
    int16_t Mag_Y_RAW;
    int16_t Mag_Z_RAW;

    float Mx;
    float My;
    float Mz;

    float magAdjustmentX;
    float magAdjustmentY;
    float magAdjustmentZ;

    float magSoftIronCorrectionX;
    float magSoftIronCorrectionY;
    float magSoftIronCorrectionZ;

    float magHardIronCorrectionX;
    float magHardIronCorrectionY;
    float magHardIronCorrectionZ;

} MAG_DATA_t;

float magResolution;

public:
MAG_DATA_t *magDataStore;

private:

	HAL_StatusTypeDef isMagConnected();
	HAL_StatusTypeDef setMode();

	void setMagResolution(magScale magResolution);
	void setAdjustmentValue();
	void softAndHardIronErrors();

public:
	dzMagnetometer(dzI2c* magI2cObject, dzLogger* logger);

	//Init Function
	HAL_StatusTypeDef magInit();

	// Read
	HAL_StatusTypeDef readRawMagData();
	HAL_StatusTypeDef processMagData();
	HAL_StatusTypeDef readAndProcessMagData();

	// Logger
	void dzLog(char *msg);

};

#endif /* UTILS_INC_DZMAGNETOMETER_H_ */
