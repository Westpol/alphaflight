// sd driver source code

#include "sd.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_crc.h"
#include <string.h>

extern SD_HandleTypeDef hsd1;

__attribute__((section(".dma_tx"), aligned(32))) static uint8_t sd_dma_buffer_1[SD_BLOCK_SIZE] = {0};
__attribute__((section(".dma_tx"), aligned(32))) static uint8_t sd_dma_buffer_2[SD_BLOCK_SIZE] = {0};
uint32_t buffer2_used = 0;

static __attribute__((aligned(32))) uint8_t tmp_buff[SD_BLOCK_SIZE] = {0};  // create write buffer for safer CRC logic

static uint32_t generate_crc32_hw(uint8_t* buffer_pointer);

SD_RETURN_TYPE SD_READ_BLOCK_BLOCKING(uint8_t* buff, uint32_t address, uint32_t copy_size, uint32_t timeout){
    if(copy_size > SD_USABLE_BLOCK_SIZE_BYTES) return SD_FAIL;

    uint32_t start = MILLIS32();

    while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER || hsd1.State != HAL_SD_STATE_READY){
        if((MILLIS32() - start) >= timeout) return SD_FAIL;
    }

    if(HAL_SD_ReadBlocks(&hsd1, tmp_buff, address * SD_SECTORS_PER_BLOCK, SD_SECTORS_PER_BLOCK, timeout) != HAL_OK) return SD_FAIL;

    uint32_t crc_on_sd;
    memcpy(&crc_on_sd, &tmp_buff[SD_USABLE_BLOCK_SIZE_BYTES], sizeof(uint32_t));    // load CRC from end of block

    if(crc_on_sd != generate_crc32_hw(tmp_buff)) return SD_WRONG_CRC;   // check if CRC on SD matches with block content

    memcpy(buff, &tmp_buff, copy_size);

    return SD_OKAY;
}

SD_RETURN_TYPE SD_WRITE_BLOCK_BLOCKING(uint8_t* buff, uint32_t address, uint32_t copy_size, uint32_t timeout){
    if(copy_size > SD_USABLE_BLOCK_SIZE_BYTES) return SD_FAIL;

    uint32_t start = MILLIS32();

    while(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER || hsd1.State != HAL_SD_STATE_READY){
        if((MILLIS32() - start) >= timeout) return SD_FAIL;
    }
    
    memset(tmp_buff, 0, sizeof(tmp_buff));
    memcpy(tmp_buff, buff, copy_size);

    uint32_t block_crc = generate_crc32_hw(tmp_buff);
    memcpy(&tmp_buff[SD_USABLE_BLOCK_SIZE_BYTES], &block_crc, sizeof(uint32_t));

    if(HAL_SD_WriteBlocks(&hsd1, tmp_buff, address * SD_SECTORS_PER_BLOCK, SD_SECTORS_PER_BLOCK, timeout) != HAL_OK) return SD_FAIL;
    return SD_OKAY;
}

SD_RETURN_TYPE SD_WRITE_BLOCK_DMA(uint8_t* buff, uint32_t address){
    if(HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER || hsd1.State != HAL_SD_STATE_READY){
        memcpy(sd_dma_buffer_2, buff, SD_USABLE_BLOCK_SIZE_BYTES);
        
        uint32_t block_crc = generate_crc32_hw(sd_dma_buffer_2);
        memcpy(&sd_dma_buffer_2[SD_USABLE_BLOCK_SIZE_BYTES], &block_crc, sizeof(uint32_t));

        //__HAL_SD_ENABLE_IT(&hsd1, HAL_SD_)
        buffer2_used++;
        return SD_BUSY;
    }
    memcpy(sd_dma_buffer_1, buff, SD_USABLE_BLOCK_SIZE_BYTES);

    uint32_t block_crc = generate_crc32_hw(sd_dma_buffer_1);
    memcpy(&sd_dma_buffer_1[SD_USABLE_BLOCK_SIZE_BYTES], &block_crc, sizeof(uint32_t));

    if(HAL_SD_WriteBlocks_DMA(&hsd1, sd_dma_buffer_1, address, SD_SECTORS_PER_BLOCK) != HAL_OK) return SD_FAIL;
    return SD_OKAY;
}

static uint32_t generate_crc32_hw(uint8_t* buffer_pointer){      // buffer pointer MUST BE pointing at first byte of block
    CRC->CR |= CRC_CR_RESET;    // reset crc engine
    uint32_t* p = (uint32_t*)buffer_pointer;
    for(size_t i = 0; i < SD_USABLE_BLOCK_SIZE_BYTES / 4; i++){       // sd_usable_block_size_bytes / 4 comes from 4092 byte block size divided by 4 bytes (32 bit words used for CRC engine)
        CRC->DR = p[i];
    }
    return CRC->DR ^0xFFFFFFFF;
}

