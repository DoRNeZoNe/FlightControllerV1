/*
 * GLOBAL_VARIABLES.h
 *
 *  Created on: Mar 12, 2023
 *      Author: Vipul Gupta
 */

#ifndef DZFLIGHTCONTROLLER_UTILS_INC_GLOBAL_VARIABLES_H_
#define DZFLIGHTCONTROLLER_UTILS_INC_GLOBAL_VARIABLES_H_


// MPU Important Addresses
#define IMU_I2C_ADDRESS 0xD0

// Mathematical Variables
#define DZ_PI 3.14159265358979323846

// To configure acc and gyro
enum accScale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum gyroScale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum timerChannels {
	TIM_CH1 = 0,
	TIM_CH2,
	TIM_CH3,
	TIM_CH4,
};


#endif /* DZFLIGHTCONTROLLER_UTILS_INC_GLOBAL_VARIABLES_H_ */
