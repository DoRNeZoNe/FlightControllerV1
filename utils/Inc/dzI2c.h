/*
 * dzI2c.h
 *
 *  Created on: Mar 8, 2023
 *      Author: Vipul Gupta
 */

#ifndef DZFLIGHTCONTROLLER_UTILS_DZI2C_H_
#define DZFLIGHTCONTROLLER_UTILS_DZI2C_H_

#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "stm32f4xx.h"

class dzI2c {

// Variables
public:
	uint32_t timeOut = 100;
	uint16_t DeviceAddress;
	I2C_HandleTypeDef* i2c;

public:
	dzI2c(I2C_HandleTypeDef* i2c, uint16_t DeviceAddress);
	HAL_StatusTypeDef write(uint8_t destinationAddress, uint8_t* sourceAddress, uint16_t sourceSize);
	HAL_StatusTypeDef read(uint8_t sourceAddress, uint8_t* destinationAddress, uint16_t bytesToRead);
};

#endif /* DZFLIGHTCONTROLLER_UTILS_DZI2C_H_ */
