#include "filesystem.h"
#include "sd.h"

#define metadata_per_block (SD_USABLE_BLOCK_SIZE_BYTES / sizeof(metadata))
#define metadata_version 0

superblock sb = {0};
metadata mb[metadata_per_block] = {0};

static void fs_init_metadata(){
    for(int i = 0; i < metadata_per_block; i++){
        mb[i].magic = 0x464C4D44;
        mb[i].version = metadata_version;
    }
}


void reset_superblock(){
    superblock sb_temp = {0};
    for(uint8_t i = 0; i < 2; i++){
        sb_temp.magic = 0x53424C4B;
        sb_temp.version = 0;
        sb_temp.seq = i;
        sb_temp.curr_flight_num_abs = 0;
        sb_temp.curr_flight_num_rel = 0;
        sb_temp.metadata_block_start = 100;
        sb_temp.metadata_block_end = 1000;
        sb_temp.data_block_start = 1001;
        sb_temp.data_block_end = 6000000;
        SD_WRITE_BLOCK_BLOCKING((uint8_t*)&sb_temp, 98 + i, sizeof(sb_temp), 100);
    }
}