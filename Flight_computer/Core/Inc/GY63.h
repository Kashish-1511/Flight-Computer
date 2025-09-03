/*
 * GY63.h
 *
 *  Created on: Aug 14, 2025
 *      Author: ambal
 */

#ifndef INC_GY63_H_
#define INC_GY63_H_


#include "stm32f7xx_hal.h"

#define MS5611_ADDR       (0x77 << 1)  // GY-63 I2C address

#define CMD_RESET         0x1E
#define CMD_CONV_D1_BASE  0x40
#define CMD_CONV_D2_BASE  0x50
#define CMD_ADC_READ      0x00

// Oversampling options
#define OSR_256   0x00
#define OSR_512   0x02
#define OSR_1024  0x04
#define OSR_2048  0x06
#define OSR_4096  0x08

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t C[7]; // PROM calibration coefficients
} GY63_HandleTypeDef;

HAL_StatusTypeDef GY63_Init(GY63_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef GY63_Read(I2C_HandleTypeDef *hi2c, uint32_t *raw_pressure, uint32_t *raw_temperature);
void GY63_Convert(const GY63_HandleTypeDef *dev, uint32_t D1, uint32_t D2, float *pressure_mbar, float *temperature_c);



#endif /* INC_GY63_H_ */
