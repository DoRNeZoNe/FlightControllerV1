/*
 * dzPwm.cpp
 *
 *  Created on: Mar 26, 2023
 *      Author: Vipul Gupta
 */

#include <dzPwm.h>

dzPwm::dzPwm(TIM_HandleTypeDef *htim, timerChannels channel)
{

	this->timerHandle = htim;
	this->channel = this->getTimerChannel(channel);

	this->ARR = this->timerHandle->Instance->ARR;
	this->prescaler = this->timerHandle->Instance->PSC;

	this->setCcrRegister();

	HAL_TIM_PWM_Start(htim, this->channel);
}

void dzPwm::generateWave(uint16_t pulseInUs)
{

	uint16_t CCRxValue = calculatePulseForCCRx(pulseInUs);
	*this->ccrRegister = CCRxValue;
}

uint32_t dzPwm::getTimerChannel(timerChannels channel)
{

	switch ((uint8_t)this->channel)
	{
	case TIM_CH1:
		return TIM_CHANNEL_1;
	case TIM_CH2:
		return TIM_CHANNEL_2;
	case TIM_CH3:
		return TIM_CHANNEL_3;
	case TIM_CH4:
		return TIM_CHANNEL_4;
	}
	return 0xff;
}

void dzPwm::setCcrRegister()
{
	switch ((uint8_t)this->channel)
	{
	case TIM_CH1:
		this->ccrRegister = &this->timerHandle->Instance->CCR1;
		break;
	case TIM_CH2:
		this->ccrRegister = &this->timerHandle->Instance->CCR2;
		break;
	case TIM_CH3:
		this->ccrRegister = &this->timerHandle->Instance->CCR3;
		break;
	case TIM_CH4:
		this->ccrRegister = &this->timerHandle->Instance->CCR4;
		break;
	}
}

uint16_t dzPwm::calculatePulseForCCRx(uint16_t pulseInUs)
{

	float pulseInMs = pulseInUs / 1e3;
	uint16_t CCRxValue = (this->ARR / this->pulseWidthInMs) * pulseInMs;
	return CCRxValue;
}
