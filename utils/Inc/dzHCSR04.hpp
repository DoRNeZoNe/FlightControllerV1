#ifndef HCS04R_HPP
#define HCS04R_HPP

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "dzLogger.h"
class DZHCS04R {
private:
	dzLogger logger;
    uint32_t IC_Val1 = 0;
    uint32_t IC_Val2 = 0;
    uint32_t Difference = 0;
    uint8_t Is_First_Captured = 0;
    uint8_t Distance = 1;
    GPIO_TypeDef* TRIG_PORT;
    uint16_t TRIG_PIN;
public:
    void DZHAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim);
    void DZHCSR04_Read(TIM_HandleTypeDef* htim, GPIO_TypeDef* trigPort, uint16_t trigPin);
    int DZGetDistance();
    DZHCS04R(UART_HandleTypeDef *huart) : logger(huart) {}
};

#endif
