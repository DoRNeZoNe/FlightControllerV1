/*
 * MadgwickFilter.h
 *
 *  Created on: Mar 12, 2023
 *      Author: Vipul Gupta
 */

#ifndef DZFLIGHTCONTROLLER_UTILS_INC_MADGWICKFILTER_H_
#define DZFLIGHTCONTROLLER_UTILS_INC_MADGWICKFILTER_H_

#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include <string.h>
#include <stm32f4xx.h>
#include "GLOBAL_VARIABLES.h"
#include "dzLogger.h"

class MadgwickFilter {

private:

	// Logger Object
	dzLogger *logger;

	// Acc Gyro Mag
	float ax, ay, az, gx, gy, gz, mx, my, mz;

    // for madgwick
    float GyroMeasError = DZ_PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = DZ_PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	float deltaT = 0.0f;                              			// integration interval for both filter schemes
	int previousMillis = HAL_GetTick(), currentMillis = 0;
public:
	float pitch, yaw, roll;
	float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};            			// vector to hold quaternion

public:
	MadgwickFilter(dzLogger *logger);
	void updateRollPitchYaw(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ);
	void MadgwickQuaternionUpdate();
	void quaternionToRollPitchYaw();

};

#endif /* DZFLIGHTCONTROLLER_UTILS_INC_MADGWICKFILTER_H_ */
