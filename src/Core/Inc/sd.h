/*
 * sd.h
 *
 *  Created on: Sep 12, 2025
 *      Author: benno
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

//uint32_t CALCULATE_CRC32_HW(const void *data, size_t length);
//void VERIFY_CRC32(const void* data, size_t size, uint32_t expected_crc);

typedef enum {
    SD_OK = 0,
	SD_ERROR_TIMEOUT = 1,
	SD_ERROR_WRITE = 2,
	SD_ERROR_CRC_MISMATCH = 3,
	SD_ERROR_READ = 4,
	SD_ERROR_BLOCK_LIMIT_REACHED = 5,
	SD_ERROR_WRONG_MAGIC = 6,
	SD_ERROR_DMA_WRITE = 7,
} SD_STATUS;


typedef enum{
	SD_DMA_TRANSMISSION_STARTED,
	SD_DMA_BUSY_NOT_STARTED,
	SD_CARD_BUSY_NOT_STARTED,
	SD_DMA_ERROR_NOT_STARTED,
}SD_DMA_STATUS;

SD_STATUS SD_READ_BLOCK(uint8_t* data_storage, uint32_t block);
SD_STATUS SD_WRITE_BLOCK(uint8_t* data_array, uint32_t data_length_bytes, uint32_t block);
SD_DMA_STATUS SD_WRITE_DMA(uint8_t* data_array, uint32_t block, uint32_t num_blocks);

bool SD_DMA_BUSY(void);

uint32_t CALCULATE_CRC32_HW(const void *data, size_t length);
void VERIFY_CRC32(const void* data, size_t size, uint32_t expected_crc);

#define BLOCK_SIZE     512
#define CRC32_BYTE_SIZE 4
#define BLOCK_DATA_SIZE BLOCK_SIZE - CRC32_BYTE_SIZE
#define TIMEOUT_MS     1000
#define SD_DMA_BUFFER_REG_MAX_ENTRIES 10

#endif /* INC_SD_H_ */
