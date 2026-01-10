/*
 * sd.c
 *
 *  Created on: Sep 12, 2025
 *      Author: benno
 */

#include "sd.h"
#include "stm32f7xx_hal.h"

#include <string.h>

#include "main.h"

extern SD_HandleTypeDef hsd1;
extern CRC_HandleTypeDef hcrc;

static volatile bool dma_busy = false;

static volatile uint32_t dma_buffer[2048] = {0};

typedef struct{
	uint8_t block_count;
	uint32_t write_block;
	void* buffer_pointer;
}dma_buffer_register_t;

static dma_buffer_register_t buffer_register[SD_DMA_BUFFER_REG_MAX_ENTRIES] = {0};

static void LOG_FAIL_WITH_ERROR(uint8_t error_code){
#if DEBUG_ENDABLED
	ERROR_HANDLER_BLINKS(error_code);
	return;
#endif
}

// DMA complete callback
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    dma_busy = false;
}


uint32_t CALCULATE_CRC32_HW(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t word;
    uint32_t crc_final;

    hcrc.Instance->CR = CRC_CR_RESET;

    // process all full words
    while (length >= 4) {
        memcpy(&word, bytes, 4);
        hcrc.Instance->DR = __REV(word); // swap bytes, not bits
        bytes += 4;
        length -= 4;
    }

    // process remaining bytes (if any)
    if (length > 0) {
        word = 0;
        memcpy(&word, bytes, length);
        hcrc.Instance->DR = __REV(word); // zero-padded, byte-swapped
    }

    crc_final = hcrc.Instance->DR;
    return crc_final;
}




void VERIFY_CRC32(const void* data, size_t size, uint32_t expected_crc){
	uint32_t calculated_crc = CALCULATE_CRC32_HW(data, size);
	if(calculated_crc != expected_crc){
		LOG_FAIL_WITH_ERROR(SD_ERROR_CRC_MISMATCH);
	}
}

SD_STATUS SD_READ_BLOCK(uint8_t* data_storage, uint32_t block){

	uint8_t read_buffer[BLOCK_SIZE] = {0};

	uint32_t start = HAL_GetTick();
	while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
		if (HAL_GetTick() - start > TIMEOUT_MS) {
			LOG_FAIL_WITH_ERROR(SD_ERROR_TIMEOUT); // Timeout
			return SD_ERROR_TIMEOUT;
		}
	}
	if(HAL_SD_ReadBlocks(&hsd1, read_buffer, block, 1, TIMEOUT_MS) != HAL_OK){
		LOG_FAIL_WITH_ERROR(SD_ERROR_READ); // Write failed#
		return SD_ERROR_READ;
	}

	uint32_t block_crc32 = ((uint32_t)read_buffer[511] << 24) | ((uint32_t)read_buffer[510] << 16) | ((uint32_t)read_buffer[509] << 8)  | ((uint32_t)read_buffer[508]);
	VERIFY_CRC32(read_buffer, BLOCK_DATA_SIZE, block_crc32);
	memcpy(data_storage, read_buffer, BLOCK_DATA_SIZE);

	return SD_OK;
}

SD_STATUS SD_WRITE_BLOCK(uint8_t* data_array, uint32_t data_length_bytes, uint32_t block){

	if (data_length_bytes > BLOCK_DATA_SIZE) {
		LOG_FAIL_WITH_ERROR(SD_ERROR_BLOCK_LIMIT_REACHED); // Too much data
		return SD_ERROR_BLOCK_LIMIT_REACHED;
	}

	uint8_t single_write_buffer[BLOCK_SIZE] = {0};
	memcpy(single_write_buffer, data_array, data_length_bytes);
	uint32_t crc32 = CALCULATE_CRC32_HW(single_write_buffer, BLOCK_DATA_SIZE);
	single_write_buffer[508] = (crc32 >> 0);
	single_write_buffer[509] = (crc32 >> 8);
	single_write_buffer[510] = (crc32 >> 16);
	single_write_buffer[511] = (crc32 >> 24);

	uint32_t start = HAL_GetTick();
	while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER){
		if (HAL_GetTick() - start > TIMEOUT_MS) {
			LOG_FAIL_WITH_ERROR(SD_ERROR_TIMEOUT); // Timeout
			return SD_ERROR_TIMEOUT;
		}
	}
	if(HAL_SD_WriteBlocks(&hsd1, single_write_buffer, block, 1, TIMEOUT_MS) != HAL_OK){
		LOG_FAIL_WITH_ERROR(SD_ERROR_WRITE); // Write failed
		return SD_ERROR_WRITE;
	}

	return SD_OK;
}

SD_DMA_STATUS SD_WRITE_DMA(uint8_t* data_array, uint32_t block, uint32_t num_blocks){
	if(SD_DMA_BUSY()){
		return SD_DMA_BUSY_NOT_STARTED;
	}

    if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        // Card not ready, defer
        return SD_CARD_BUSY_NOT_STARTED; // Deferred
    }

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(&hsd1, data_array, block, num_blocks);

    if(status != HAL_OK){
    	return SD_DMA_ERROR_NOT_STARTED;
    }

    dma_busy = true;

	return SD_DMA_TRANSMISSION_STARTED;
}

bool SD_DMA_CHECK_IF_BUFFER_IS_FREE(const void* buffer_address){
	for(int i = 0; i < SD_DMA_BUFFER_REG_MAX_ENTRIES; i++){
		if(buffer_address == buffer_register[i].buffer_pointer)return false;
	}
	return true;
}

bool SD_DMA_BUSY(void){
	return dma_busy;
}
