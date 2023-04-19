/*
 * dzIbus.h
 *
 *  Created on: Mar 30, 2023
 *      Author: Vipul Gupta
 */

#ifndef INC_DZIBUS_H_
#define INC_DZIBUS_H_

#include "stm32f4xx_hal.h"


#define IBUS_USER_CHANNELS		6
#define IBUS_LENGTH				0x20
#define IBUS_COMMAND40			0x40
#define IBUS_MAX_CHANNLES		14
#define CHECK_SUM				0xffff

class dzIbus {

public:
	UART_HandleTypeDef* huart;
	uint8_t rawInputData[IBUS_LENGTH] = {0};
	uint16_t processedInputData[IBUS_USER_CHANNELS] = {0};

public:
	dzIbus(UART_HandleTypeDef* huart2);
	void processData();
	void validateAndProcessData();
	uint8_t isReponseValid();
	uint8_t isCheckSumValid();
};

#endif /* INC_DZIBUS_H_ */
