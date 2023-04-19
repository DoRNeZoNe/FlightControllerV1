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

	int delt_t = 0; // used to control display output rate
	int count = 0;  // used to control display output rate

	// parameters for 6 DoF sensor fusion calculations
	float GyroMeasError = DZ_PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
	float GyroMeasDrift = DZ_PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
	float pitch, yaw, roll;
	float deltat = 0.0f;                              // integration interval for both filter schemes
	int lastUpdate = HAL_GetTick(), firstUpdate = 0, Now = 0;     // used to calculate integration interval                               // used to calculate integration interval
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

	typedef struct {

	    double Ax;
	    double Ay;
	    double Az;

	    double Gx;
	    double Gy;
	    double Gz;

	} madgwickInput_t;

public:
	MadgwickFilter(dzLogger *logger);
	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
};

#endif /* DZFLIGHTCONTROLLER_UTILS_INC_MADGWICKFILTER_H_ */
