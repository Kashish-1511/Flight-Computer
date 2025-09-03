/*
 * lsm6do32.h
 *
 *  Created on: Jul 17, 2025
 *      Author: ambalv
 */

#ifndef INC_LSM6DSO32_H_
#define INC_LSM6DSO32_H_


#include "stm32f7xx_hal.h"

#define LSM6DSO32_I2C_ADDR  (0x6A << 1)  // 7-bit address shifted for HAL

// Register addresses
#define LSM6DSO32_ADDR     (0x6A << 1)  // 8-bit address for HAL
#define WHO_AM_I_REG       0x0F
#define CTRL1_XL           0x10  // Accelerometer control
#define CTRL2_G            0x11  // Gyroscope control
#define OUTX_L_G           0x22  // Gyro X low byte
#define OUTX_L_XL          0x28  // Accel X low byte


typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} LSM6DSO32_Data;

extern I2C_HandleTypeDef hi2c1;

void LSM6DSO32_Init(I2C_HandleTypeDef *hi2c);
void LSM6DSO32_ReadAccelGyro(I2C_HandleTypeDef *hi2c, int16_t *accel, int16_t *gyro);

#endif /* INC_LSM6DSO32_H_ */
