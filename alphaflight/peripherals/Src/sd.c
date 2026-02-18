// sd driver source code

#include "sd.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_crc.h"
#include <stddef.h>
#include <string.h>

__attribute__((section(".dma_rx"))) static uint8_t sd_dma_buffer_1[STM32_WORD_SIZE * 64] = {0};
__attribute__((section(".dma_rx"))) static uint8_t sd_dma_buffer_2[STM32_WORD_SIZE * 64] = {0};


static uint32_t generate_crc32_hw(uint8_t* buffer_pointer){      // buffer pointer MUST BE pointing at first byte of block
    CRC->CR = 1;    // reset crc engine
    for(size_t i = 0; i < sd_usable_block_size_bytes / 4; i++){       // sd_usable_block_size_bytes / 4 comes from 508 byte block size divided by 4 bytes (32 bit words used for CRC engine)
        uint32_t word;
        memcpy(&word, &buffer_pointer[i * 4], 4);
        CRC->DR = word;
    }
    return CRC->DR;
}


void bump(void){
    generate_crc32_hw(&sd_dma_buffer_1[0]);
    for(int i = 0; i < 2047; i++){
        sd_dma_buffer_1[i] += sd_dma_buffer_1[i + 1];
        sd_dma_buffer_2[i] += sd_dma_buffer_2[i + 1];
    }
}
