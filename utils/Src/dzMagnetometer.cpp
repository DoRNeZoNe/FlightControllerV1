/*
 * dzMagnetometer.cpp
 *
 *  Created on: Apr 29, 2023
 *      Author: Vipul Gupta
 */

#include <dzMagnetometer.h>

dzMagnetometer::dzMagnetometer(dzI2c* magI2cObject, dzLogger* logger) {

	this->magI2cObj = magI2cObject;
	if(logger){
		this->loggerObj = logger;
	}

	this->magDataStore = (MAG_DATA_t*)calloc(1, sizeof(MAG_DATA_t));

	this->magInit();

}

HAL_StatusTypeDef dzMagnetometer::magInit(){

	if(this->isMagConnected() == HAL_ERROR){
		return HAL_ERROR;
	}

	this->setMagResolution(MAG_16_BIT);
	this->setAdjustmentValue();
	this->setMode();
	this->softAndHardIronErrors();

	this->dzLog((char*)"MAG Successfully Initialized****\r\n");

	return HAL_OK;

}

HAL_StatusTypeDef dzMagnetometer::readRawMagData(){
	uint8_t statusReg1;
	magI2cObj->read(AK8975A_ST1, &statusReg1, sizeof(statusReg1));

	// wait for magnetometer data ready bit to be set
    if (statusReg1 & magDataReadyStatus) {

    	// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
        uint8_t raw_data[7];

        // Read the six raw data and ST2 registers sequentially into data array
    	magI2cObj->read(AK8975A_XOUT_L, raw_data, sizeof(raw_data));

    	// End data read by reading ST2 register
        uint8_t statusReg2 = raw_data[6];

        // Check if magnetic sensor overflow set, if not then report data
        if (!(statusReg2 & magDataOverRunStatus)) {
        	// Data stored as little Endian
            this->magDataStore->Mag_X_RAW = ((int16_t)raw_data[1] << 8) | raw_data[0];
            this->magDataStore->Mag_Y_RAW = ((int16_t)raw_data[3] << 8) | raw_data[2];
            this->magDataStore->Mag_Z_RAW = ((int16_t)raw_data[5] << 8) | raw_data[4];
            return HAL_OK;
        }
    }

    return HAL_ERROR;

}

HAL_StatusTypeDef dzMagnetometer::processMagData(){

	this->magDataStore->Mx = (float)(magDataStore->Mag_X_RAW * magResolution * magDataStore->magAdjustmentX - magDataStore->magHardIronCorrectionX ) * magDataStore->magSoftIronCorrectionX;
	this->magDataStore->My = (float)(magDataStore->Mag_Y_RAW * magResolution * magDataStore->magAdjustmentY - magDataStore->magHardIronCorrectionY ) * magDataStore->magSoftIronCorrectionY;
	this->magDataStore->Mz = (float)(magDataStore->Mag_Z_RAW * magResolution * magDataStore->magAdjustmentZ - magDataStore->magHardIronCorrectionZ ) * magDataStore->magSoftIronCorrectionZ;

	return HAL_OK;

}

HAL_StatusTypeDef dzMagnetometer::readAndProcessMagData(){

	if(this->readRawMagData() == HAL_OK){
		this->processMagData();
		return HAL_OK;
	}
	return HAL_ERROR;
}

// Logger Functions
void dzMagnetometer::dzLog(char *msg){

	if(this->loggerObj){
		this->loggerObj->logViaUart(msg);
	}

}

// Private Function

HAL_StatusTypeDef dzMagnetometer::isMagConnected(){
	uint8_t check=122;
	magI2cObj->read(WHO_AM_I_AK8975A, &check, 1);
	if (check == 72){
		this->dzLog((char*)"MAG successfully detected****\r\n");
		return HAL_OK;
	}
	else {
		this->dzLog((char*)"MAG Not detected****\r\n");
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef dzMagnetometer::setMode(){
	this->magI2cObj->write(AK8975A_CNTL_2, &magReset, sizeof(magReset));
	HAL_Delay(10);
	this->magI2cObj->write(AK8975A_CNTL_1, &magPowerDown, sizeof(magPowerDown));
	HAL_Delay(100);
	this->magI2cObj->write(AK8975A_CNTL_1, &magContinousMeasurementAnd16BitOutput, sizeof(magContinousMeasurementAnd16BitOutput));
	HAL_Delay(10);

	return HAL_OK;
}

void dzMagnetometer::setAdjustmentValue(){

	uint8_t adjustmentValues[3];
	magI2cObj->write(AK8975A_CNTL_1, &magPowerDown, sizeof(magPowerDown));
    HAL_Delay(10);
    magI2cObj->write(AK8975A_CNTL_1, &magFuseRomMode, sizeof(magFuseRomMode));
    HAL_Delay(10);

    magI2cObj->read(AK8975A_ASAX, adjustmentValues, sizeof(adjustmentValues));
    this->magDataStore->magAdjustmentX = (float)(adjustmentValues[0] - 128) / 256. + 1.;
    this->magDataStore->magAdjustmentY = (float)(adjustmentValues[1] - 128) / 256. + 1.;
    this->magDataStore->magAdjustmentZ = (float)(adjustmentValues[2] - 128) / 256. + 1.;

}

void dzMagnetometer::softAndHardIronErrors(){

	this->dzLog((char*)"Getting Soft and Hard Iron Errors");
	uint16_t totalSamples = 1500;

    int32_t hardIron[3] = {0, 0, 0}, softIron[3] = {0, 0, 0};
    int16_t MagMax[3] = {-32767, -32767, -32767};
    int16_t MagMin[3] = {32767, 32767, 32767};

    if(this->calibrate == true){
        for (uint16_t i = 0; i < totalSamples; i++) {

            this->readRawMagData();

            // For X
            if (this->magDataStore->Mag_X_RAW > MagMax[0]) MagMax[0] = this->magDataStore->Mag_X_RAW;
            if (this->magDataStore->Mag_X_RAW < MagMin[0]) MagMin[0] = this->magDataStore->Mag_X_RAW;

            // For Y
            if (this->magDataStore->Mag_Y_RAW > MagMax[1]) MagMax[1] = this->magDataStore->Mag_Y_RAW;
            if (this->magDataStore->Mag_Y_RAW < MagMin[1]) MagMin[1] = this->magDataStore->Mag_Y_RAW;

            // For Z
            if (this->magDataStore->Mag_Z_RAW > MagMax[2]) MagMax[2] = this->magDataStore->Mag_Z_RAW;
            if (this->magDataStore->Mag_Z_RAW < MagMin[2]) MagMin[2] = this->magDataStore->Mag_Z_RAW;

        	if(!(i%100))this->dzLog((char*)".");

            // at 100 Hz ODR, new mag data is available every 10 ms
            HAL_Delay(12);

        }
        this->dzLog((char*)"\r\n");
        // Hard Iron Corrections
        hardIron[0] = (MagMax[0] + MagMin[0]) / 2;  // get average x mag bias in counts
        hardIron[1] = (MagMax[1] + MagMin[1]) / 2;  // get average y mag bias in counts
        hardIron[2] = (MagMax[2] + MagMin[2]) / 2;  // get average z mag bias in counts

        magDataStore->magHardIronCorrectionX = (float)hardIron[0] * magResolution * magDataStore->magAdjustmentX;
        magDataStore->magHardIronCorrectionY = (float)hardIron[1] * magResolution * magDataStore->magAdjustmentY;
        magDataStore->magHardIronCorrectionZ = (float)hardIron[2] * magResolution * magDataStore->magAdjustmentZ;

        // Soft Iron Correction
        //*** multiplication by mag_bias_factory added in accordance with the following comment
        //*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
        softIron[0] = (float)(MagMax[0] - MagMin[0]) * magDataStore->magAdjustmentX / 2;
        softIron[1] = (float)(MagMax[1] - MagMin[1]) * magDataStore->magAdjustmentX / 2;
        softIron[2] = (float)(MagMax[2] - MagMin[2]) * magDataStore->magAdjustmentX / 2;

        float averageRead = softIron[0] + softIron[1] + softIron[2];
        averageRead /= 3.0;

        magDataStore->magSoftIronCorrectionX = averageRead / ((float)softIron[0]);
        magDataStore->magSoftIronCorrectionY = averageRead / ((float)softIron[1]);
        magDataStore->magSoftIronCorrectionZ = averageRead / ((float)softIron[2]);

    } else{

    	// Default Hard and Soft Iron Correction
        magDataStore->magHardIronCorrectionX = 68.983635;
        magDataStore->magHardIronCorrectionY = 721.698364;
        magDataStore->magHardIronCorrectionZ = 152.732727;

        magDataStore->magSoftIronCorrectionX = 0.981481;
        magDataStore->magSoftIronCorrectionY = 1.039216;
        magDataStore->magSoftIronCorrectionZ = 0.981481;

    }

//	char stringData[50];
//	sprintf(stringData, "X => %f | Y => %f | Z => %f\r\n", magDataStore->magHardIronCorrectionX, magDataStore->magHardIronCorrectionY, magDataStore->magHardIronCorrectionZ);
//	this->dzLog(stringData);
//
//	sprintf(stringData, "X => %f | Y => %f | Z => %f\r\n", magDataStore->magSoftIronCorrectionX, magDataStore->magSoftIronCorrectionY, magDataStore->magSoftIronCorrectionZ);
//	this->dzLog(stringData);

}

void dzMagnetometer::setMagResolution(magScale mgScale) {
    switch (mgScale) {
        // Possible magnetometer scales (and their register bit settings) are:
        // 14 bit resolution (0) and 16 bit resolution (1)
        // Proper scale to return milliGauss
        case MAG_14_BIT:
            this->magResolution = 10. * 4912. / 8190.0;
            break;
        case MAG_16_BIT:
        	this->magResolution = 10. * 4912. / 32760.0;
        	break;
        default:
        	this->magResolution = 0.;
    }

}


