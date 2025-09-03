#ifndef __SD_H
#define __SD_H

#include "stm32f7xx_hal.h" // For HAL types and SPI/GPIO
#include "main.h"          // For GPIO pin definitions if they are there (e.g. SD_CS_Pin, SD_CS_GPIO_Port)
#include <stdint.h>
#include <stdbool.h>

// Define CS Pin and Port if not already defined in main.h specific to SD
// It's better if these are defined in main.h or a dedicated pin_config.h
// For this example, we'll assume they might be directly used in SD.c
// Or, ensure MX_GPIO_Init in main.c configures these pins.
#define SD_CS_GPIO_Port GPIOD
#define SD_CS_Pin       GPIO_PIN_6
// Function Prototypes
bool SD_DetectCard(void);
// Data Logging Functions
bool SD_Init(void);
bool SD_WriteBlock(uint32_t blockAddr, const uint8_t *data);
bool SD_ReadBlock(uint32_t blockAddr, uint8_t *buffer);
void LogLineToSD(const char *line);
void ReadSDCardAndSendViaUSART2(void);
#endif /* __SD_H */
