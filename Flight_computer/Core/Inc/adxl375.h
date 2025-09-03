/*
 * adxl375.h
 *
 *  Created on: Jul 15, 2025
 *      Author: ambal
 */

#ifndef INC_ADXL375_H_
#define INC_ADXL375_H_


#include "stm32f7xx_hal.h"  // Change for your MCU

#define ADXL375_I2C_ADDR     (0x53 << 1)  // HAL uses 8-bit address

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ADXL375_Data_t;

HAL_StatusTypeDef ADXL375_Init(I2C_HandleTypeDef *hi2c);
void ADXL375_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t *accel);

#endif /* INC_ADXL375_H_ */
