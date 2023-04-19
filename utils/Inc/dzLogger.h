/*
 * dzLogger.h
 *
 *  Created on: Mar 11, 2023
 *      Author: Vipul Gupta
 */

#ifndef DZFLIGHTCONTROLLER_UTILS_DZLOGGER_H_
#define DZFLIGHTCONTROLLER_UTILS_DZLOGGER_H_

#include <stm32f4xx.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

class dzLogger {
public:
	UART_HandleTypeDef *huart;
	dzLogger(UART_HandleTypeDef *huart);
	void logViaUart(char* msg);
};

#endif /* DZFLIGHTCONTROLLER_UTILS_DZLOGGER_H_ */
