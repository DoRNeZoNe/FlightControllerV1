/*
 * dzLogger.cpp
 *
 *  Created on: Mar 11, 2023
 *      Author: Vipul Gupta
 */

#include "dzLogger.h"

dzLogger::dzLogger(UART_HandleTypeDef *huart) {
	this->huart = huart;
}

void dzLogger::logViaUart(char* msg){
	char dataStr[50];
	sprintf(dataStr, msg);
	HAL_UART_Transmit(this->huart, (uint8_t*)dataStr, strlen(dataStr), HAL_MAX_DELAY);
}
