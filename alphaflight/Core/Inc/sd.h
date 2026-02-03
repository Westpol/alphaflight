// sd driver header file

#include <stdint.h>

#ifndef SD_H_
#define SD_H_

#define sd_usable_block_size_bytes 508

void bump(void);

void SD_APPEND_TO_BUFFER(uint8_t* data_pointer, uint32_t len);

#endif