/*
 * MPU6050.h
 *
 *  Created on: Dec 8, 2021
 *      Author: Martin Christiansson
 */

// #ifndef INC_MPU6050_H_
// #define INC_MPU6050_H_

#ifndef MPU6050_I2C_DRIVER_H
#define MPU6050_I2C_DRIVER_H

#include "stm32f4xx_hal.h"    //needed for I2C communication

// DEFINES
#define MPU6050_I2C_ADDRESS (0x68 << 1) // fill in address according to datasheet
#define DMP_MEM_START_ADDR 0x6E // from BF dont kbow what it is, does bot exist in datasheet
#define DMP_MEM_R_W        0x6F // from BF dont kbow what it is, does bot exist in datasheet
#define MPU6050_SMPLRT_DIV      0       // 8000Hz when MPU6050 Digital Low Pass FIlter disable page.12 (register map)
#define MPU6050_RA_DEVID      0x75         // device id
#define MPU6050_GYRO_FS_2000        0x03
#define MPU6050_ACCEL_FS_16         0x03

// REGISTERS  these are these are gyro readings
#define MPU6050_REG_GYRO_XOUT_H      0x43
#define MPU6050_REG_GYRO_XOUT_L      0x44
#define MPU6050_REG_GYRO_YOUT_H      0x45
#define MPU6050_REG_GYRO_YOUT_L      0x46
#define MPU6050_REG_GYRO_ZOUT_H      0x47
#define MPU6050_REG_GYRO_ZOUT_L      0x48
#define MPU6050_REG_GYRO_CONFIG      0x1B
#define MPU6050_RA_CONFIG           0x1A  // write seven for 8KHZ no DLPF
#define MPU6050_RA_SMPLRT_DIV       0x19  // write zero to get 8KHZ sampling
#define MPU6050_GCONFIG_FS_SEL_BIT      4  //write 0x03
#define MPU6050_RA_ACCEL_CONFIG     0x1C   //write 0x00 to get +-2g
#define MPU6050_RA_ACCEL_XOUT_H     0x3B   //read only Xdata
#define MPU6050_RA_ACCEL_XOUT_L     0x3C   //read only Xdata
#define MPU6050_RA_ACCEL_YOUT_H     0x3D   //read only Ydata
#define MPU6050_RA_ACCEL_YOUT_L     0x3E   //read only Ydata
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F   //read only Zdata
#define MPU6050_RA_ACCEL_ZOUT_L     0x40   //read only Zdata
#define MPU6050_RA_PWR_MGMT_1       0x6B   // write 0x00 to wake up sensor

// struct
typedef struct{

	I2C_HandleTypeDef *i2cHandle;  // i2c handle
	float gyro_ds[3];              // gyro data degrees/s
	float acc_ms2[3];			   // acc data ms2

} MPU6050;


// INITIALISATION
uint8_t MPU6050_initialise(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle); // MPU6050 struct & i2c handle

// DATA ACQUISITION
void MPU6050_read_gyro(MPU6050 *dev);
void MPU6050_read_acc(MPU6050 *dev);

// LOW LEVEL FUNCTIONS
HAL_StatusTypeDef MPU6050_read_REG(MPU6050 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MPU6050_read_REGS(MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t length);
HAL_StatusTypeDef MPU6050_write_REG(MPU6050 *dev, uint8_t reg, uint8_t *data);



#endif /* INC_MPU6050_H_ */
