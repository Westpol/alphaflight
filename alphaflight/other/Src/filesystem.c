#include "filesystem.h"
#include "sd.h"

#define metadata_per_block (SD_USABLE_BLOCK_SIZE_BYTES / sizeof(metadata))
#define metadata_version 0
#define superblock_address_start 98
static inline uint32_t rel_fn_to_index(uint32_t fn) {return fn % metadata_per_block;}
static inline uint32_t rel_fn_to_block(uint32_t fn) {return fn / metadata_per_block;}

superblock sb = {0};
uint8_t active_sb = 0;
metadata mb[metadata_per_block] = {0};
metadata last_flight = {0};
metadata current_flight = {0};

FS_RETURN_TYPE fs_load_last_flight(void);
static void fs_init_metadata(void);

FS_RETURN_TYPE FS_LOAD_SUPERBLOCK(void){
    superblock sb_temp_1 = {0};
    superblock sb_temp_2 = {0};
    SD_READ_BLOCK_BLOCKING((uint8_t*)&sb_temp_1, superblock_address_start, sizeof(sb_temp_1), 100);
    SD_READ_BLOCK_BLOCKING((uint8_t*)&sb_temp_2, superblock_address_start + 1, sizeof(sb_temp_2), 100);

    if(sb_temp_1.seq > sb_temp_2.seq){
        sb = sb_temp_1;
        active_sb = 0;
    }
    else {
        sb = sb_temp_2;
        active_sb = 1;
    }
    return FS_OKAY;
}

uint32_t FS_NEW_FLIGHT(void){
    uint32_t start_block = 1000;
    if(sb.curr_flight_num_rel != 0){
        fs_load_last_flight();
        start_block = last_flight.end_block + 1;
        current_flight.start_block = last_flight.end_block + 1;
        current_flight.flight_num_abs = sb.curr_flight_num_abs++;
    }
    else {  // first flight after format
        fs_init_metadata();
        mb[0].start_ms = MILLIS32();
        mb[0].start_block = sb.metadata_block_start;
    }
    return start_block;
}

FS_RETURN_TYPE FS_END_FLIGHT(uint32_t end_block){
    current_flight.end_block = end_block;
    current_flight.end_ms = MILLIS32();
    sb.seq++;
    SD_WRITE_BLOCK_BLOCKING((uint8_t*)&sb, superblock_address_start + active_sb, sizeof(sb), 100);
    active_sb = (active_sb + 1) % 2;
    return FS_OKAY;
}

FS_RETURN_TYPE fs_load_last_flight(void){
    uint32_t curr_fn = sb.curr_flight_num_rel;
    uint32_t last_fn = UTILS_MAX_I(curr_fn - 1, 0);
    SD_READ_BLOCK_BLOCKING((uint8_t*)&mb, rel_fn_to_block(last_fn), sizeof(mb), 100);
    last_flight = mb[rel_fn_to_index(last_fn)];
    SD_READ_BLOCK_BLOCKING((uint8_t*)&mb, rel_fn_to_block(curr_fn), sizeof(mb), 100);
    current_flight = mb[rel_fn_to_index(curr_fn)];
    return FS_OKAY;
}

void fs_reset_superblock(){
    superblock sb_temp = {0};
    for(uint8_t i = 0; i < 2; i++){
        sb_temp.magic = 0x53424C4B;
        sb_temp.version = 0;
        sb_temp.seq = i;
        sb_temp.curr_flight_num_abs = 0;
        sb_temp.curr_flight_num_rel = 0;
        sb_temp.metadata_block_start = 100;
        sb_temp.metadata_block_end = 999;
        sb_temp.data_block_start = 1000;
        sb_temp.data_block_end = 5999999;
        SD_WRITE_BLOCK_BLOCKING((uint8_t*)&sb_temp, 98 + i, sizeof(sb_temp), 100);
    }
}

static void fs_init_metadata(void){
    for(int i = 0; i < metadata_per_block; i++){
        mb[i].magic = 0x464C4D44;
        mb[i].version = metadata_version;
    }
}