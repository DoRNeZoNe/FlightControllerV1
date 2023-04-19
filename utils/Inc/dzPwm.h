/*
 * dzPwm.h
 *
 *  Created on: Mar 26, 2023
 *      Author: Vipul Gupta
 */

#ifndef DZFLIGHTCONTROLLER_UTILS_INC_DZPWM_H_
#define DZFLIGHTCONTROLLER_UTILS_INC_DZPWM_H_

#include "DZ_MPU_REGISTER_DESC.h"
#include "GLOBAL_VARIABLES.h"
#include "dzLogger.h"
#include "dzI2c.h"

class dzPwm {

private:
	int pulseWidthInMs = 20;
	uint8_t timerFrequency = 50;

	uint16_t minInputInUs = 1000;
	uint16_t maxInputInUs = 2000;

	uint16_t prescaler;
	uint16_t ARR;

	volatile uint32_t* ccrRegister;

public:

	TIM_HandleTypeDef* timerHandle;
	timerChannels channel;

	dzPwm(TIM_HandleTypeDef* htim, timerChannels channel);
	void generateWave(uint16_t pulseInUs);
	void setCcrRegister();
	uint32_t getTimerChannel();
	uint16_t calculatePulseForCCRx(uint16_t pulseInUs);

};

#endif /* DZFLIGHTCONTROLLER_UTILS_INC_DZPWM_H_ */
