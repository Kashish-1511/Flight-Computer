/*
 * lsm6do32.c
 *
 *  Created on: Jul 17, 2025
 *      Author: ambal
 */


#include "lsm6dso32.h"
#include <stdio.h>
void LSM6DSO32_ReadAccelGyro(I2C_HandleTypeDef *hi2c, int16_t *accel, int16_t *gyro)
{
    uint8_t data[12];  // 6 bytes for gyro + 6 for accel

    // Read gyro: OUTX_L_G (0x22) to OUTZ_H_G (0x27)
    HAL_I2C_Mem_Read(hi2c, LSM6DSO32_ADDR, OUTX_L_G, 1, &data[0], 6, HAL_MAX_DELAY);

    // Read accel: OUTX_L_XL (0x28) to OUTZ_H_XL (0x2D)
    HAL_I2C_Mem_Read(hi2c, LSM6DSO32_ADDR, OUTX_L_XL, 1, &data[6], 6, HAL_MAX_DELAY);

    // Gyro
    gyro[0] = (int16_t)(data[1] << 8 | data[0]);  // X
    gyro[1] = (int16_t)(data[3] << 8 | data[2]);  // Y
    gyro[2] = (int16_t)(data[5] << 8 | data[4]);  // Z

    // Accel
    accel[0] = (int16_t)(data[7] << 8 | data[6]);   // X
    accel[1] = (int16_t)(data[9] << 8 | data[8]);   // Y
    accel[2] = (int16_t)(data[11] << 8 | data[10]); // Z
}
//------------------------------------------------------------------------

void LSM6DSO32_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t ctrl1_xl = 0x60;  // ODR = 416 Hz, ±2g
    uint8_t ctrl2_g  = 0x60;  // ODR = 416 Hz, ±250 dps
    uint8_t check = 0;
    HAL_I2C_Mem_Read(&hi2c1, LSM6DSO32_ADDR, 0x10, 1, &check, 1, HAL_MAX_DELAY);
    printf("CTRL1_XL = 0x%02X\r\n", check);  // should be 0x60
    HAL_I2C_Mem_Write(hi2c, LSM6DSO32_ADDR, CTRL1_XL, 1, &ctrl1_xl, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(hi2c, LSM6DSO32_ADDR, CTRL2_G,  1, &ctrl2_g,  1, HAL_MAX_DELAY);
}
