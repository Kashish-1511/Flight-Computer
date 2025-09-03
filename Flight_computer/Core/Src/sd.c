/*
 * sd.c
 *
 *  Created on: Jul 3, 2025
 *      Author: ambal
 */
#include "SD.h"
#include <string.h> // For memset if used
#include "stdio.h"
#include <stdbool.h>
#include <stdint.h>
// UART handle for debugging
extern UART_HandleTypeDef huart3; // Assuming huart2 is used for debug prints
extern SPI_HandleTypeDef hspi1;

// --- Logging Globals ---
#define SD_BLOCK_SIZE 512// static uint8_t sd_log_data_buffer[SD_BLOCK_SIZE]; // Old binary buffer

//----------------------------Defining Registers of SD card for initialization------------------------------------------------------------------------
#define CMD0    (0x40 | 0)
#define CMD8    (0x40 | 8)
#define CMD55   (0x40 | 55)
#define ACMD41  (0x40 | 41)
#define CMD58   (0x40 | 58)

//===============================Necessary function for CS and Transmit/receive Data========================================================================================================
void SD_Select(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

void SD_Deselect(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

uint8_t SD_SPI_TxRx(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}
//------------------------------Detecting SD card with cmd--------------------------------------------------------------------
bool SD_DetectCard(void) {
    // 1. Send 80 dummy clocks with CS high
    SD_Select();
    for (int i = 0; i < 10; i++) {
        SD_SPI_TxRx(0xFF);  // Send dummy bytes
    }

    // 2. Send CMD0 (GO_IDLE_STATE)
    SD_Select();

    SD_SPI_TxRx(0x40 | 0);     // CMD0
    SD_SPI_TxRx(0x00);         // Arg[31:24]
    SD_SPI_TxRx(0x00);         // Arg[23:16]
    SD_SPI_TxRx(0x00);         // Arg[15:8]
    SD_SPI_TxRx(0x00);         // Arg[7:0]
    SD_SPI_TxRx(0x95);         // Valid CRC for CMD0

    // 3. Wait for response (R1 = 0x01 = IDLE)
    uint8_t response = 0xFF;
    for (int i = 0; i < 8; i++) {
        response = SD_SPI_TxRx(0xFF);
        if (response == 0x01) break;
    }

    SD_Deselect();
    SD_SPI_TxRx(0xFF); // Final 8 clocks

    // 4. Send result over USART2
    if (response == 0x01) {
        const char *msg = "SD card detected and responded to CMD0.\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return true;
    } else {
        const char *msg = "No SD card or no CMD0 response.\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return false;
    }
}
//-------------------------Initialization-------------------------------------------------------------------------------------
bool SD_Init(void) {
	SD_Deselect();
	printf("DESELECTED");
    uint8_t r1;
    uint8_t ocr[4];
// 3. CMD8: check voltage and SD version
  SD_Select();
  printf("SELECTED");
  SD_SPI_TxRx(CMD8);
  SD_SPI_TxRx(0x00);
  SD_SPI_TxRx(0x00);
  SD_SPI_TxRx(0x01);  // 2.7–3.6V
  SD_SPI_TxRx(0xAA);  // check pattern
  SD_SPI_TxRx(0x87);  // Valid CRC for CMD8
  // AITING FOR R1 TO RECIEVE DATA
  for (int i = 0; i < 10; i++) {
      r1 = SD_SPI_TxRx(0xFF);
      if (r1 != 0xFF) break;
      HAL_Delay(1);
  }
  printf("CMD8 R1 = 0x%02X\r\n", r1);
  for (int i = 0; i < 4; i++) ocr[i] = SD_SPI_TxRx(0xFF);
SD_Deselect();

  if (r1 != 0x01 || ocr[3] != 0xAA) {
	 /// printf("SD CMD8 failed: r1=0x%02X, R7 = %02X %02X %02X %02X\r\n",r1, ocr[0], ocr[1], ocr[2], ocr[3]);
      printf("SD CMD8 failed, r1 = 0x%02X\r\n", r1);
      return false;
  }

  printf("SD CMD8 OK, SD v2.x card detected\r\n");


  // 4. ACMD41: initialize card (HCS=1 for SDHC support)
  for (int i = 0; i < 100; i++) {
      // Send CMD55
      SD_Select();
      SD_SPI_TxRx(CMD55);
      SD_SPI_TxRx(0x00);
      SD_SPI_TxRx(0x00);
      SD_SPI_TxRx(0x00);
      SD_SPI_TxRx(0x00);
      SD_SPI_TxRx(0x01);  // Dummy CRC
      r1 = SD_SPI_TxRx(0xFF);
      for (int j = 0; j < 8; j++) {
          if (r1 != 0xFF) break;
          r1 = SD_SPI_TxRx(0xFF);
      }
      SD_Deselect();

      // Send ACMD41
      SD_Select();
      SD_SPI_TxRx(ACMD41);
      SD_SPI_TxRx(0x40);  // HCS=1
      SD_SPI_TxRx(0x00);
      SD_SPI_TxRx(0x00);
      SD_SPI_TxRx(0x00);
      SD_SPI_TxRx(0x01);  // Dummy CRC
      r1 = SD_SPI_TxRx(0xFF);
      for (int j = 0; j < 8; j++) {
          if (r1 != 0xFF) break;
          r1 = SD_SPI_TxRx(0xFF);
      }
      SD_Deselect();

      if (r1 == 0x00) break;  // Card is ready
      HAL_Delay(10);
  }

  if (r1 != 0x00) {
      printf("SD ACMD41 failed, r1 = 0x%02X\r\n", r1);
      return false;
  }

  printf("SD ACMD41 OK, card initialized\r\n");

  // 5. CMD58: Read OCR
  SD_Select();
  SD_SPI_TxRx(CMD58);
  SD_SPI_TxRx(0x00);
  SD_SPI_TxRx(0x00);
  SD_SPI_TxRx(0x00);
  SD_SPI_TxRx(0x00);
  SD_SPI_TxRx(0x01);  // Dummy CRC
  for (int i = 0; i < 10; i++) {
        r1 = SD_SPI_TxRx(0xFF);
        if (r1 != 0xFF) break;
        HAL_Delay(1);
    }
  for (int i = 0; i < 4; i++) ocr[i] = SD_SPI_TxRx(0xFF);
  SD_Deselect();

  if (r1 != 0x00) {
      printf("SD CMD58 failed, r1 = 0x%02X\r\n", r1);
      return false;
  }

  if (ocr[0] & 0x40)
      printf("Card is SDHC/SDXC\r\n");
  else
      printf("Card is SDSC\r\n");

  return true;
}
//------The write block---------------------------------------------------------------------------------
bool SD_WriteBlock(uint32_t blockAddr, const uint8_t *data) {
    SD_Select();

    uint8_t r1;

    // CMD24: Write single block (blockAddr is already block-based for SDHC)
    SD_SPI_TxRx(0x40 | 24);
    SD_SPI_TxRx((blockAddr >> 24) & 0xFF);
    SD_SPI_TxRx((blockAddr >> 16) & 0xFF);
    SD_SPI_TxRx((blockAddr >> 8) & 0xFF);
    SD_SPI_TxRx(blockAddr & 0xFF);
    SD_SPI_TxRx(0xFF);  // Dummy CRC

    // Wait for response (R1)
    for (int i = 0; i < 10; i++) {
        r1 = SD_SPI_TxRx(0xFF);
        if ((r1 & 0x80) == 0) break;
    }
    if (r1 != 0x00) {
        printf("CMD24 failed, r1 = 0x%02X\r\n", r1);
        SD_Deselect();
        return false;
    }

    // Send start token
    SD_SPI_TxRx(0xFE);

    // Send 512 bytes of data
    for (int i = 0; i < 512; i++) {
        SD_SPI_TxRx(data[i]);
    }

    // Send dummy CRC
    SD_SPI_TxRx(0xFF);
    SD_SPI_TxRx(0xFF);

    // Read data response
    uint8_t response = SD_SPI_TxRx(0xFF);
    if ((response & 0x1F) != 0x05) {
        printf("Write rejected, response = 0x%02X\r\n", response);
        SD_Deselect();
        return false;
    }

    // Wait for write complete
    while (SD_SPI_TxRx(0xFF) == 0x00);

    SD_Deselect();
    SD_SPI_TxRx(0xFF);  // Extra 8 clocks

    printf("Block written successfully\r\n");
    return true;
}

//----------------------The read block---------------------------------------------------------
bool SD_ReadBlock(uint32_t blockAddr, uint8_t *buffer) {
    SD_Select();

    // Send CMD17 (read single block)
    SD_SPI_TxRx(0x40 | 17);
    SD_SPI_TxRx((blockAddr >> 24) & 0xFF);
    SD_SPI_TxRx((blockAddr >> 16) & 0xFF);
    SD_SPI_TxRx((blockAddr >> 8) & 0xFF);
    SD_SPI_TxRx(blockAddr & 0xFF);
    SD_SPI_TxRx(0xFF);  // Dummy CRC

    // Wait for R1
    uint8_t r1 = 0xFF;
    for (int i = 0; i < 10; i++) {
        r1 = SD_SPI_TxRx(0xFF);
        if ((r1 & 0x80) == 0) break;
    }

    if (r1 != 0x00) {
        printf("CMD17 failed, r1 = 0x%02X\r\n", r1);
        SD_Deselect();
        return false;
    }

    // Wait for data token (0xFE)
    for (int i = 0; i < 1000; i++) {
        uint8_t token = SD_SPI_TxRx(0xFF);
        if (token == 0xFE) break;
        HAL_Delay(1);
        if (i == 999) {
            printf("Read timeout waiting for token\r\n");
            SD_Deselect();
            return false;
        }
    }

    // Read 512 bytes
    for (int i = 0; i < 512; i++) {
        buffer[i] = SD_SPI_TxRx(0xFF);
    }

    // Read and discard 2-byte CRC
    SD_SPI_TxRx(0xFF);
    SD_SPI_TxRx(0xFF);

    SD_Deselect();
    SD_SPI_TxRx(0xFF);

    return true;
}
//------------------------------------logging sensor data in block from buffer increasing the block address/ starting at 100=====================

uint8_t sd_log_buffer[SD_BLOCK_SIZE];
uint16_t log_index = 0;
uint32_t next_block = 100;

void LogLineToSD(const char *line) {
    uint16_t len = strlen(line);

    if (log_index + len >= SD_BLOCK_SIZE) {
        SD_WriteBlock( next_block, sd_log_buffer);
        next_block++;
        memset(sd_log_buffer, 0x00, SD_BLOCK_SIZE);
        log_index = 0;
    }

    memcpy(&sd_log_buffer[log_index], line, len);
    log_index += len;
}
//----------------------------------reading------------------------
void ReadSDCardAndSendViaUSART2(void) {
    uint8_t buff[SD_BLOCK_SIZE];
    uint32_t block = 100;

    while (1) {
        int result = SD_ReadBlock(block, buff);
        if (result == 0) {
            HAL_UART_Transmit(&huart3, buff, SD_BLOCK_SIZE, HAL_MAX_DELAY);
            block++;  // next block
        } else {
            const char *err = "SD read failed\r\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
            break;
        }
    }
}

//-------------------------------------------------------
