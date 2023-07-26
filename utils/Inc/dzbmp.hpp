#ifndef bmp_H
#define bmp_H

#include "stm32f4xx_hal.h"
#include "dzLogger.h"
#define BMP280_ADDRESS 0xEC
#define SUPPORT_64BIT 1
//#define SUPPORT_32BIT 1

#define RESET_REG 0xE0
#define CTRL_HUM_REG 0xF2
#define CTRL_MEAS_REG 0xF4
#define CONFIG_REG 0xF5
#define ID_REG 0xD0
#define PRESS_MSB_REG 0xF7

#define MODE_FORCED 0x01

class BMP280 {
public:
  BMP280(I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef* huart);

  int BMP280_Config(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter);
  void BMP280_Measure();
  void BMP280_WakeUP();
  float getTemperature();
  float getPressure();
  float getHumidity();
  float getAltitude();

private:
  I2C_HandleTypeDef *i2cHandle;
  dzLogger logger;
  uint8_t chipID;
  uint8_t TrimParam[36];
  int32_t tRaw, pRaw, hRaw;
  uint16_t dig_T1, dig_P1, dig_H1, dig_H3;
  int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5, dig_H6;
  int32_t t_fine;
  float Temperature, Pressure, Humidity,Altitude;

  void TrimRead();
  int BMPReadRaw();
  int32_t BMP280_compensate_T_int32(int32_t adc_T);
  uint32_t BMP280_compensate_P_int64(int32_t adc_P);
  uint32_t BMP280_compensate_H_int32(int32_t adc_H);
};

#endif
