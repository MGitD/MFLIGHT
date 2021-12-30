/*
 * MPU6050.c
 *
 *  Created on: Dec 8, 2021
 *      Author: Martin Chistiansson
 */

//INCLUDES
#include "MPU6050.h"

uint8_t MPU6050_initialise(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){

	// set up sensor
	HAL_Delay(100);

//set struct params
	dev->i2cHandle = i2cHandle; // dont know check this one
	dev->gyro_ds[0] = 0.0f;     // set all values to zero for gyro
	dev->gyro_ds[1] = 0.0f;
	dev->gyro_ds[2] = 0.0f;
	dev->acc_ms2[0] = 0.0f;     // set all values to zero for acc
	dev->acc_ms2[1] = 0.0f;
	dev->acc_ms2[2] = 0.0f;

	uint8_t errnum = 0;         // keep track of errors

	//uint8_t regData = 0x03; // MPU6050_GYRO_FS_2000  0x03
	//MPU6050_write_REG(dev, MPU6050_REG_GYRO_CONFIG, &regData); // set the gyro to full scale

	HAL_StatusTypeDef status;  // define a status to debug

	// check ID of the device, if not ok, exit init func
	uint8_t regData;
	status = MPU6050_read_REG(dev, MPU6050_RA_DEVID, &regData); // why so often HAL_BUSY?
	errnum += (status != HAL_OK);

	if (regData != 0x68){
	return status;             // if device ID not ok
	}

	// config MPU6050


    // wake up sensor
	regData = 0x00;
	status = MPU6050_write_REG(dev, MPU6050_RA_PWR_MGMT_1, &regData);
	errnum += (status != HAL_OK);

	// no DLPF 0x07, DLPF 0x00
	regData = 0x07;
	status = MPU6050_write_REG(dev, MPU6050_RA_CONFIG, &regData);
	errnum += (status != HAL_OK);

	// +-16g
	regData = (0x03 << 3);
	status = MPU6050_write_REG(dev, MPU6050_RA_ACCEL_CONFIG, &regData);
	errnum += (status != HAL_OK);


	// 8KHZ sample rate
	regData = 0x00;
	status = MPU6050_write_REG(dev, MPU6050_RA_SMPLRT_DIV, &regData);
	errnum += (status != HAL_OK);

	// -+250d/s regData = (0x00 << 0);
	regData = (0x02 << 3); // +-1000d/s
	status = MPU6050_write_REG(dev, MPU6050_REG_GYRO_CONFIG, &regData);
	errnum += (status != HAL_OK);




	return errnum;               // 0 if all ok
}

// DATA ACQUISITION
void MPU6050_read_acc(MPU6050 *dev){

	// each axis is composed of 16bits from two registers (8bit-each)
	uint8_t regData[6];

	// read acc from XOUT_H and up to ZOUT_L (6 bytes)
	MPU6050_read_REGS(dev, MPU6050_RA_ACCEL_XOUT_H, regData, 6);

	// MSBs shifted into a int16_t then add LSBs at end
	int16_t raw_acc_x;
	int16_t raw_acc_y;
	int16_t raw_acc_z;
	raw_acc_x = (int16_t)  (regData[0] << 8 |  regData[1]);
	raw_acc_y = (int16_t)  (regData[2] << 8 |  regData[3]);
	raw_acc_z = (int16_t)  (regData[4] << 8 |  regData[5]);


	// set dev measure variables
	dev->acc_ms2[0] = (9.82f/2048.0f) * raw_acc_x;
	dev->acc_ms2[1] = (9.82f/2048.0f) * raw_acc_y;
	dev->acc_ms2[2] = (9.82f/2048.0f) * raw_acc_z;

}

void MPU6050_read_gyro(MPU6050 *dev){

	// each axis is composed of 16bits from two registers (8bit-each)
	uint8_t regData[6];

	// read acc from XOUT_H and up to ZOUT_L (6 bytes)
	MPU6050_read_REGS(dev, MPU6050_REG_GYRO_XOUT_H, regData, 6);

	// MSBs shifted into a int16_t then add LSBs at end
	int16_t raw_gyro_x;
	int16_t raw_gyro_y;
	int16_t raw_gyro_z;
	raw_gyro_x = ( (int16_t) regData[0] ) << 8 |  regData[1];
	raw_gyro_y = ( (int16_t) regData[2] ) << 8 |  regData[3];
	raw_gyro_z = ( (int16_t) regData[4] ) << 8 |  regData[5];


	// set dev measure variables
	dev->gyro_ds[0] = (1.0f/32.8f) * (float) raw_gyro_x;
	dev->gyro_ds[1] = (1.0f/32.8f) * (float) raw_gyro_y;
	dev->gyro_ds[2] = (1.0f/32.8f) * (float) raw_gyro_z;




}




//LOW LEVEL FUNCTIONS

HAL_StatusTypeDef MPU6050_read_REG(MPU6050 *dev, uint8_t reg, uint8_t *data){

	uint8_t del = 10;
return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1 ,del);

}

HAL_StatusTypeDef MPU6050_read_REGS(MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t length){
	uint8_t del = 10;
return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length ,del);

}

HAL_StatusTypeDef MPU6050_write_REG(MPU6050 *dev, uint8_t reg, uint8_t *data){
	uint8_t del = 10;
return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, del);

}

