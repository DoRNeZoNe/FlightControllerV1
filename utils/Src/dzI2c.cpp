/*
 * dzI2c.cpp
 *
 *  Created on: Mar 8, 2023
 *      Author: Vipul Gupta
 */

#include "dzI2c.h"

dzI2c::dzI2c(I2C_HandleTypeDef* i2c, uint16_t DeviceAddress) {

	this->i2c = i2c;
	this->DeviceAddress = DeviceAddress;

}

HAL_StatusTypeDef dzI2c::write(uint8_t destinationAddress, uint8_t* sourceAddress, uint16_t sourceSize){
	return HAL_I2C_Mem_Write(this->i2c, DeviceAddress, destinationAddress, 1, sourceAddress, sourceSize, this->timeOut);
}

HAL_StatusTypeDef dzI2c::read(uint8_t sourceAddress, uint8_t* destinationAddress, uint16_t bytesToRead){
	return HAL_I2C_Mem_Read(this->i2c, DeviceAddress, sourceAddress, 1, destinationAddress, bytesToRead, this->timeOut);
}

