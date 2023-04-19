/*
 * dzIbus.cpp
 *
 *  Created on: Mar 30, 2023
 *      Author: Vipul Gupta
 */

#include <dzIbus.h>

dzIbus::dzIbus(UART_HandleTypeDef* huart2) {

	this->huart = huart2;
	HAL_UART_Receive_DMA(this->huart, this->rawInputData, IBUS_LENGTH);

}

uint8_t dzIbus::isReponseValid(){
	return (rawInputData[0] == IBUS_LENGTH && rawInputData[1] == IBUS_COMMAND40);
}

void dzIbus::processData(){
	for(int pi = 0, ri = 2; pi < IBUS_USER_CHANNELS; pi++, ri += 2)
	{
		processedInputData[pi] = rawInputData[ri + 1] << 8 | rawInputData[ri];
	}
}

void dzIbus::validateAndProcessData(){
	if(!this->isReponseValid())
		return;
	if(!this->isCheckSumValid())
		return;
	this->processData();
}

uint8_t dzIbus::isCheckSumValid(){
 	uint16_t checkSumCal = CHECK_SUM;
	uint16_t checkSumFromData;

	for(int i = 0; i < 30; i++)
	{
		checkSumCal -= this->rawInputData[i];
	}

	checkSumFromData = this->rawInputData[31] << 8 | this->rawInputData[30];
	return (checkSumFromData == checkSumCal);
}
