#include "common.h"

#ifndef FILESYSTEM_H
#define FILESYSTEM_H

typedef struct{
    uint32_t magic;     // 0x464C4D44 / "FLMD"
    uint32_t version;

    uint32_t flight_num_abs;
    uint32_t timestamp;
    uint32_t start_ms;
    uint32_t end_ms;
    int32_t lat;
    int32_t lon;

    uint32_t start_block;
    uint32_t end_block;
    uint32_t flags; // first bit if log started, second bit if log ended successfully
} metadata;

typedef struct{
    uint32_t magic;     // 0x53424C4B / "SBLK"
    uint32_t version;
    uint32_t seq;
    uint32_t curr_flight_num_rel;
    uint32_t curr_flight_num_abs;
    uint32_t flags;     // corrupted, full, etc.
    uint32_t metadata_block_start;
    uint32_t metadata_block_end;
    uint32_t data_block_start;
    uint32_t data_block_end;
} superblock;

#endif