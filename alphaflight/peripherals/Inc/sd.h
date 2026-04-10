// sd driver header file

#include "common.h"

#ifndef SD_H_
#define SD_H_

#define SD_USABLE_BLOCK_SIZE_BYTES 4092
#define SD_SECTORS_PER_BLOCK 8
#define SD_BLOCK_SIZE 4096

typedef enum{
    SD_OKAY,
    SD_FAIL,
    SD_BUSY,
    SD_WRONG_CRC,
    SD_BUFF_FULL
}SD_RETURN_TYPE;

SD_RETURN_TYPE SD_WRITE_BLOCK_BLOCKING(uint8_t* buff, uint32_t address, uint32_t timeout);
SD_RETURN_TYPE SD_READ_BLOCK_BLOCKING(uint8_t* buff, uint32_t address, uint32_t timeout);
SD_RETURN_TYPE SD_WRITE_BLOCK_DMA(uint8_t* buff, uint32_t address);

#endif