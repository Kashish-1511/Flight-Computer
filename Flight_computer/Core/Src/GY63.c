/*
 * GY63.c
 *
 *  Created on: Aug 14, 2025
 *      Author: ambal
 */


#include "gy63.h"
#include "string.h"

static uint32_t GY63_ReadADC(I2C_HandleTypeDef *hi2c) {
    uint8_t buf[3];
    HAL_I2C_Mem_Read(hi2c, MS5611_ADDR, CMD_ADC_READ, I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

HAL_StatusTypeDef GY63_Init(GY63_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c) {
    uint8_t cmd;
    HAL_StatusTypeDef status;

    // Reset
    cmd = CMD_RESET;
    status = HAL_I2C_Master_Transmit(hi2c, MS5611_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    HAL_Delay(3); // Reset time

    // Read PROM coefficients
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t prom_cmd = 0xA0 + (i * 2);
        uint8_t buf[2];
        status = HAL_I2C_Master_Transmit(hi2c, MS5611_ADDR, &prom_cmd, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) return status;
        status = HAL_I2C_Master_Receive(hi2c, MS5611_ADDR, buf, 2, HAL_MAX_DELAY);
        if (status != HAL_OK) return status;
        dev->C[i] = (buf[0] << 8) | buf[1];
    }
    return HAL_OK;
}

HAL_StatusTypeDef GY63_Read(I2C_HandleTypeDef *hi2c, uint32_t *raw_pressure, uint32_t *raw_temperature) {
    uint8_t cmd;
    HAL_StatusTypeDef status;

    // Start D1 (pressure) conversion
    cmd = CMD_CONV_D1_BASE | OSR_256; // OSR can be changed
    status = HAL_I2C_Master_Transmit(hi2c, MS5611_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    HAL_Delay(3); // wait for conversion
    *raw_pressure = GY63_ReadADC(hi2c);

    // Start D2 (temperature) conversion
    cmd = CMD_CONV_D2_BASE | OSR_256;
    status = HAL_I2C_Master_Transmit(hi2c, MS5611_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    HAL_Delay(5);
    *raw_temperature = GY63_ReadADC(hi2c);

    return HAL_OK;
}

void GY63_Convert(const GY63_HandleTypeDef *dev, uint32_t D1, uint32_t D2, float *pressure_mbar, float *temperature_c) {
    int32_t dT, TEMP;
    int64_t OFF, SENS;

    dT   = (int32_t)D2 - ((int32_t)dev->C[5] * 256);
    TEMP = 2000 + ((int64_t)dT * dev->C[6]) / 8388608;

    OFF  = ((int64_t)dev->C[2] * 65536) + (((int64_t)dev->C[4] * dT) / 128);
    SENS = ((int64_t)dev->C[1] * 32768) + (((int64_t)dev->C[3] * dT) / 256);

    *temperature_c = TEMP / 100.0f;
    *pressure_mbar = (((D1 * SENS) / 2097152) - OFF) / 32768 / 100.0f;
}
