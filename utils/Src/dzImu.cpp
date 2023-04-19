/*
 * dzImu.cpp
 *
 *  Created on: Mar 12, 2023
 *      Author: Vipul Gupta
 */

#include <dzImu.h>

dzImu::dzImu(dzI2c* i2cObject, accScale accResolution, gyroScale gyroResolution, dzLogger* logger) {

	this->i2cObj = i2cObject;
	if(logger){
		this->loggerObj = logger;
	}

	this->imuDataStore = (IMU_DATA_t*)calloc(1, sizeof(IMU_DATA_t));

	// Set Resolution multiplier for gyro and acc
	this->setAccResolution(accResolution);
	this->setGyroResolution(gyroResolution);

	this->init();

}

HAL_StatusTypeDef dzImu::init() {

	uint8_t check;

	i2cObj->read(WHO_AM_I_MPU9150, &check, 1);
	if (check == 0x68){
		this->dzLog((char*)"\r\nsuccessfully detected****\r\n");
	}
	else {
		this->dzLog((char*)"\r\nNot detected****\r\n");
		return HAL_ERROR;
	}

	// power management register 0X6B we should write all 0's to wake the sensor up
	i2cObj->write(PWR_MGMT_1, &wakeUpImuByte, sizeof(wakeUpImuByte));

	// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	i2cObj->write(SMPLRT_DIV, &dataRateByte, sizeof(dataRateByte));

    // Set accelerometer configuration in ACCEL_CONFIG Register
	i2cObj->write(ACCEL_CONFIG, &imuAccScale, sizeof(imuAccScale));

    // Set Gyroscopic configuration in GYRO_CONFIG Register
	i2cObj->write(GYRO_CONFIG, &imuGyroScale, sizeof(imuGyroScale));

	this->dzLog((char*)"Successfully Initialized****\r\n");

	return HAL_OK;

}

// Read Data Functions
HAL_StatusTypeDef dzImu::readRawAccData(){

	uint8_t data[6];
	i2cObj->read(ACCEL_XOUT_H, data, sizeof(data));

	this->imuDataStore->Accel_X_RAW = (uint16_t)(data[0] << 8 | data[1]);
	this->imuDataStore->Accel_Y_RAW = (uint16_t)(data[2] << 8 | data[3]);
	this->imuDataStore->Accel_Z_RAW = (uint16_t)(data[4] << 8 | data[5]);

	return HAL_OK;

}

HAL_StatusTypeDef dzImu::readRawGyroData(){

	uint8_t data[6];
	i2cObj->read(GYRO_XOUT_H, data, sizeof(data));

	this->imuDataStore->Gyro_X_RAW = (uint16_t)(data[0] << 8 | data[1]);
	this->imuDataStore->Gyro_Y_RAW = (uint16_t)(data[2] << 8 | data[3]);
	this->imuDataStore->Gyro_Z_RAW = (uint16_t)(data[4] << 8 | data[5]);

	return HAL_OK;

}

HAL_StatusTypeDef dzImu::processAccData(){
	this->imuDataStore->Ax = (this->imuDataStore->Accel_X_RAW * this->accResolution) - this->imuDataStore->AxBias;
	this->imuDataStore->Ay = (this->imuDataStore->Accel_Y_RAW * this->accResolution) - this->imuDataStore->AyBias;
	this->imuDataStore->Az = (this->imuDataStore->Accel_Z_RAW * this->accResolution) - this->imuDataStore->AzBias;
	return HAL_OK;
}


HAL_StatusTypeDef dzImu::processGyroData(){
	this->imuDataStore->Gx = (this->imuDataStore->Gyro_X_RAW * this->gyroResolution) - this->imuDataStore->GxBias;
	this->imuDataStore->Gy = (this->imuDataStore->Gyro_Y_RAW * this->gyroResolution) - this->imuDataStore->GyBias;
	this->imuDataStore->Gz = (this->imuDataStore->Gyro_Z_RAW * this->gyroResolution) - this->imuDataStore->GzBias;
	return HAL_OK;
}

HAL_StatusTypeDef dzImu::readAndProcessAcc(){
	return (this->readRawAccData() == HAL_OK) && (this->processAccData() == HAL_OK) ? HAL_OK:HAL_ERROR;
}

HAL_StatusTypeDef dzImu::readAndProcessGyro(){
	return (this->readRawGyroData() == HAL_OK) && (this->processGyroData() == HAL_OK) ? HAL_OK:HAL_ERROR;
}

HAL_StatusTypeDef dzImu::getAccBias(){

	double accSum[3] = {0};

	for(int i=0; i<this->biasIterations; i++){
		this->readRawAccData();
		accSum[0] += this->imuDataStore->Accel_X_RAW * this->accResolution;
		accSum[1] += this->imuDataStore->Accel_Y_RAW * this->accResolution;
		accSum[2] += this->imuDataStore->Accel_Z_RAW * this->accResolution;
	}

	this->imuDataStore->AxBias = accSum[0]/this->biasIterations;
	this->imuDataStore->AyBias = accSum[1]/this->biasIterations;
	this->imuDataStore->AzBias = accSum[2]/this->biasIterations;

	char printBuffer[100];
	sprintf(printBuffer, "AxBias => %f | AyBias => %f\r\n", this->imuDataStore->AxBias, this->imuDataStore->AyBias);
	this->dzLog(printBuffer);


	return HAL_OK;

}

HAL_StatusTypeDef dzImu::getGyroBias(){

	double gyroSum[3] = {0};

	for(int i=0; i<this->biasIterations; i++){
		this->readRawGyroData();
		gyroSum[0] += this->imuDataStore->Gyro_X_RAW * this->gyroResolution;
		gyroSum[1] += this->imuDataStore->Gyro_Y_RAW * this->gyroResolution;
		gyroSum[2] += this->imuDataStore->Gyro_Z_RAW * this->gyroResolution;
	}

	this->imuDataStore->GxBias = gyroSum[0]/this->biasIterations;
	this->imuDataStore->GyBias = gyroSum[1]/this->biasIterations;
	this->imuDataStore->GzBias = gyroSum[2]/this->biasIterations;

	char printBuffer[100];
	sprintf(printBuffer, "GxBias => %f | GyBias => %f\r\n", this->imuDataStore->GxBias, this->imuDataStore->GyBias);
	this->dzLog(printBuffer);

	return HAL_OK;

}

// Angle functions
HAL_StatusTypeDef dzImu::anglesViaComplimentaryFilter(){

	this->readAndProcessAcc();
	this->readAndProcessGyro();

    double dt = (double)(HAL_GetTick() - this->tickStart) / 1000;
    this->tickStart = HAL_GetTick();

    this->imuDataStore->angleAccX = (atan(this->imuDataStore->Ay / sqrt(pow(this->imuDataStore->Ax, 2) + pow(this->imuDataStore->Az, 2))) * 180 / DZ_PI);
    this->imuDataStore->angleAccY = (atan(-1 * this->imuDataStore->Ax / sqrt(pow(this->imuDataStore->Ay, 2) + pow(this->imuDataStore->Az, 2))) * 180 / DZ_PI);

	this->imuDataStore->angleGyroX += (this->imuDataStore->Gx*dt);
	this->imuDataStore->angleGyroY += (this->imuDataStore->Gy*dt);

	this->imuDataStore->roll = 0.96 * this->imuDataStore->angleGyroX + 0.04 * this->imuDataStore->angleAccX;
	this->imuDataStore->pitch = 0.96 * this->imuDataStore->angleGyroY + 0.04 * this->imuDataStore->angleAccY;

	char printBuffer[100];
	sprintf(printBuffer, "Roll => %f | Pitch => %f\r\n", this->imuDataStore->roll, this->imuDataStore->pitch);
	this->dzLog(printBuffer);

	return HAL_OK;

}

// Logger Functions
void dzImu::dzLog(char *msg){

	if(this->loggerObj){
		this->loggerObj->logViaUart(msg);
	}

}

// Private Functions

void dzImu::setGyroResolution(uint8_t gScale) {
	float gyroRes;
	switch (gScale)
	{
    	// Possible gyro scales (and their register bit settings) are:
    	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
		case GFS_250DPS:
			gyroRes = 250.0/32768.0;
			break;
		case GFS_500DPS:
			gyroRes = 500.0/32768.0;
			break;
		case GFS_1000DPS:
			gyroRes = 1000.0/32768.0;
			break;
		case GFS_2000DPS:
			gyroRes = 2000.0/32768.0;
			break;
		default:
			gyroRes = 250.0/32768.0;
	}
	this->gyroResolution = gyroRes;

	// Config Regsiter OffSet for scale = 3
	this->imuGyroScale = (uint8_t)gScale << 3;
}


void dzImu::setAccResolution(uint8_t aScale) {

	float accRes;
	switch (aScale)
	{
    	// Possible accelerometer scales (and their register bit settings) are:
    	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		case AFS_2G:
			accRes = 2.0/32768.0;
			break;
		case AFS_4G:
			accRes = 4.0/32768.0;
			break;
		case AFS_8G:
			accRes = 8.0/32768.0;
			break;
		case AFS_16G:
			accRes = 16.0/32768.0;
			break;
		default:
			accRes = 2.0/32768.0;
	}
	this->accResolution = accRes;

	// Config Regsiter OffSet for scale = 3
	this->imuAccScale = (uint8_t)aScale << 3;
}


