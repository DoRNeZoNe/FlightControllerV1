/*
 * dzFlightController.cpp
 *
 *  Created on: May 14, 2023
 *      Author: Vipul Gupta
 */

#include "dzFlightController.h"

dzFlightController::dzFlightController(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef* htim) {

	// Initializing handles
	this->halUartHandle = huart;
	this->halI2cHandle = hi2c;
	this->halTimerHandle = htim;

	// Init Logger
	this->initLogger();

	// Intialise Motors for output
	this->initMotors();

	// Initialise Ibus for Input
	this->initIbus();

	// Intialise MPU & FeedbackFilter
	this->initMPU();
	this->initFeedbackFilter();
	this->stablizeFeedBack();

	// LED functions
	this->readyToFlyLED();

}

// Init Function

void dzFlightController::initLogger(){
	this->logger = new dzLogger(this->halUartHandle);
	this->logger->logViaUart((char*)"Logger Initialised\r\n");
}

void dzFlightController::initIbus(){
	this->ibus = new dzIbus(this->halUartHandle);
	this->logger->logViaUart((char*)"Input Channel initialised\r\n");
}

void dzFlightController::initMPU(){

	this->i2cMpuObj = new dzI2c(this->halI2cHandle, IMU_I2C_ADDRESS);
	this->i2cMagObj = new dzI2c(this->halI2cHandle, AK8963_I2C_ADDR);
	this->mpu = new dzImu(this->i2cMpuObj, AFS_2G, GFS_250DPS, this->logger);
	this->mag = new dzMagnetometer(this->i2cMagObj, this->logger);

}

void dzFlightController::initMotors(){

	motorTR = new dzPwm(this->halTimerHandle, TIM_CH1);
	motorTL = new dzPwm(this->halTimerHandle, TIM_CH2);
	motorBR = new dzPwm(this->halTimerHandle, TIM_CH3);
	motorBL = new dzPwm(this->halTimerHandle, TIM_CH4);

	// Give 1000US pulse
	this->motorTR->generateWave(1000);
	this->motorTL->generateWave(1000);
	this->motorBR->generateWave(1000);
	this->motorBL->generateWave(1000);

	HAL_Delay(2000);

	this->logger->logViaUart((char*)"Motors initialised\r\n");

}


// Input Related Function

void dzFlightController::updateIbusData(){
	this->ibus->validateAndProcessData();
	char stringBuffer[100];
	sprintf(stringBuffer, "%d | %d | %d | %d | %d | %d\r\n", this->ibus->processedInputData[0],this->ibus->processedInputData[1],this->ibus->processedInputData[2],this->ibus->processedInputData[3],this->ibus->processedInputData[4],this->ibus->processedInputData[5]);
	this->logger->logViaUart(stringBuffer);
}


// Feedback Related Function

void dzFlightController::initFeedbackFilter(){
	this->feedbackFilter = new MadgwickFilter(this->logger);
}

void dzFlightController::stablizeFeedBack(){
	this->logger->logViaUart((char*)"\r\nStablizing IMU Data...\r\n");
	for(int i = 0; i< 1000; i++){
		this->updateFeedback();
		HAL_Delay(10);
	}
	this->logger->logViaUart((char*)"\r\nStablization Ended...\r\n");
}

void dzFlightController::updateFeedback(){
	this->updateImuData();
	this->updateRollPitchYaw();
}

void dzFlightController::updateImuData(){

	// Read and process Acc And Gyro
	this->mpu->readAndProcessAcc();
	this->mpu->readAndProcessGyro();

	// Read and process Mag
	this->mag->readAndProcessMagData();

}

void dzFlightController::updateRollPitchYaw(){

	this->feedbackFilter->updateRollPitchYaw(\
			this->mpu->imuDataStore->Ax,\
			this->mpu->imuDataStore->Ay,\
			this->mpu->imuDataStore->Az,\
			this->mpu->imuDataStore->Gx,\
			this->mpu->imuDataStore->Gy,\
			this->mpu->imuDataStore->Gz,\
			this->mag->magDataStore->Mx,\
			this->mag->magDataStore->My,\
			this->mag->magDataStore->Mz \
	);

    char stringData[50];
    sprintf(stringData, "X => %f | Y => %f | Z => %f\r\n", this->feedbackFilter->roll,this->feedbackFilter->pitch, this->feedbackFilter->yaw);
    this->logger->logViaUart(stringData);

}

// Motor Related function

void dzFlightController::updateMotors(){

	this->motorTR->generateWave(motorOutputTR);
	this->motorTL->generateWave(motorOutputTL);
	this->motorBR->generateWave(motorOutputBR);
	this->motorBL->generateWave(motorOutputBL);

}


// LED functions

void dzFlightController::readyToFlyLED(){

}
