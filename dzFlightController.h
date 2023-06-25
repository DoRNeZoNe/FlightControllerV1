/*
 * dzFlightController.h
 *
 *  Created on: May 14, 2023
 *      Author: Vipul Gupta
 */

#ifndef UTILS_DZFLIGHTCONTROLLER_H_
#define UTILS_DZFLIGHTCONTROLLER_H_

#include "dzImu.h"
#include "dzMagnetometer.h"
#include "dzPwm.h"
#include "dzIbus.h"
#include "MadgwickFilter.h"

class dzFlightController {

// Connection Handles
private:
	UART_HandleTypeDef* halUartHandle;
	I2C_HandleTypeDef* halI2cHandle;
	TIM_HandleTypeDef* halTimerHandle;

private:
dzI2c *i2cMpuObj;
dzI2c *i2cMagObj;
dzLogger *logger;
dzImu *mpu;
dzMagnetometer *mag;
dzIbus *ibus;
MadgwickFilter *feedbackFilter;

// 4 Motors
dzPwm *motorTR;
dzPwm *motorTL;
dzPwm *motorBR;
dzPwm *motorBL;

// Motor Output
uint16_t motorOutputTR;
uint16_t motorOutputTL;
uint16_t motorOutputBR;
uint16_t motorOutputBL;

public:
	dzFlightController(UART_HandleTypeDef* huart, I2C_HandleTypeDef* i2c, TIM_HandleTypeDef* htim);

private:
	void initLogger();
	void initIbus();
	void initFeedbackFilter();
	void initMPU();
	void initMotors();

	// Imu Functions
	void stablizeFeedBack();

	// LED functions
	void readyToFlyLED();

public:

	// Input Related Function
	void updateIbusData();

	// Imu
	void updateImuData();
	void updateRollPitchYaw();
	void updateFeedback();

	// Motors
	void updateMotors();
};

#endif /* UTILS_DZFLIGHTCONTROLLER_H_ */
