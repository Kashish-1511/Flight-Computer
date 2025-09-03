/*
 * adxl375.c
 *
 *  Created on: Jul 15, 2025
 *      Author: ambal
 */


#include "adxl375.h"

#define ADXL375_REG_DEVID        0x00
#define ADXL375_REG_POWER_CTL    0x2D
#define ADXL375_REG_DATAX0       0x32
#define ADXL375_REG_DATA_FORMAT  0x31

HAL_StatusTypeDef ADXL375_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // Set data format: full resolution, ±200g (0x0B)
    data = 0x0B;
    HAL_I2C_Mem_Write(hi2c, ADXL375_I2C_ADDR, ADXL375_REG_DATA_FORMAT, 1, &data, 1, HAL_MAX_DELAY);

    // Enable measurement mode
    data = 0x08;
    HAL_I2C_Mem_Write(hi2c, ADXL375_I2C_ADDR, ADXL375_REG_POWER_CTL, 1, &data, 1, HAL_MAX_DELAY);

    return HAL_OK;
}

void ADXL375_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t *accel) {
    uint8_t buffer[6];

    HAL_I2C_Mem_Read(hi2c, ADXL375_I2C_ADDR, ADXL375_REG_DATAX0, 1, buffer, 6, HAL_MAX_DELAY);

    accel[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    accel[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    accel[2] = (int16_t)((buffer[5] << 8) | buffer[4]);

}

